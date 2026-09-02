'''Thermal + current duty limiters: smooth, composable, and ARMED by default.

Three things are asserted here, and each is a bug this tree has shipped:

1. THE DERATE IS CONTINUOUS. Upstream mapped [limit-10, limit+10] onto
   [throttle_max_at_high_rpm/2, 1] but only entered the map at temp > limit,
   so the ceiling went 2000 -> ~450 for ONE degree of overshoot, and back on
   one degree of recovery: a bang-bang across the whole derate range, no
   hysteresis, cycling on the board's thermal time constant. The per-degree
   step assertion is the mutation check - restore the old map and the step
   at the limit is ~1550 counts.

2. THE LIMITS ARE ON. Both factory JSONs and DroneCAN default_settings[]
   inherited 141/102 from the AM32 configurator skeleton, values that sit
   just OUTSIDE the ranges settings.c arms, so every ARK ESC shipped with
   both limiters silently disabled. A default-eeprom run must now come up
   derating at 105 C, and 255 must still disable it deliberately.

3. THE CEILINGS COMPOSE BY min(). Two limiters that multiplied would derate
   to 25% when each asked for 50%.

4. THE CURRENT LOOP HOLDS ITS SETPOINT. Softening its gains looks reasonable
   and is not - the /10000 in the ceiling update is a dead zone. Asserted on
   regulated current, on a plant that pulls twice the limit.

Observed through ZC_STATS v7, not through rpm: these limiters only ever
LOWER duty, so from outside a derating ESC and a weak plant look identical.
Internal state no test can see is how the dcm_hold no-op survived a full
hardware suite (see test_ceiling_hold.py).

NOTE every test here keeps a zero-throttle DShot stream running. Without an
input signal the firmware resets itself after ~3.4 s (faults.c signal-loss
NVIC_SystemReset), which re-seeds the temperature filter mid-measurement.
'''

from __future__ import annotations

import json
import os
import socket
import struct
import time

import sitl_dshot as sd
from sitl_harness import SITL_DIR, Sender, wait_for_state

MODELS = os.path.join(SITL_DIR, 'models')


def _tick(n, dt):
    '''n pauses of dt, so a sampling loop reads as one expression'''
    for _ in range(n):
        time.sleep(dt)
        yield None

STATE_MAGIC_CMD = 0x5353
ZC_STATS_MAGIC = 0x5356

# v7 layout (Mcu/SITL/Src/sitl_state.c). Appended-only; a short read is a
# firmware that predates the protection ceilings.
STATS_FMT = ('<HBBIIII' + 'B' * 8 + 'H' + 'BBH' + 'iI' + 'H' * 8 + 'B' * 7
             + 'HHH' + 'hhh')
STATS_FIELDS = (
    'zero_crosses', 'commutation_interval', 'dropped_edges', 'desync_happened',
    'old_routine', 'running', 'armed', 'zc_blind_steps', 'zc_stale_q8',
    'zc_deadline_armed', 'dcm_hold_ms', '_pad2', 'dcm_hold_value',
    'adv_kerpm_hold_ms', '_pad3', 'adv_kerpm_hold', 'zc_trend', 'zc_predicted',
    'wait_time', 'advance', 'gov_conf', 'gov_slope_q10', 'gov_duty_ceiling',
    'gov_stuck_ms', 'gov_release_ceil', 'gov_unlatch_count',
    'max_ramp_startup', 'max_ramp_low', 'max_ramp_high', 'ramp_divider',
    'max_ramp_startup_vcomp', 'acq_resist', 'desync_episode_bucket',
    'thermal_duty_ceiling', 'current_duty_ceiling', 'duty_limit_ceiling',
    'degrees_celsius', 'degrees_celsius_filt', 'actual_current')

# Src/runtime_loop.c runtimeThermalLimitTick(), Inc/motor_runtime.h
DERATE_BAND_C = 15  # THERMAL_DERATE_BAND_DEFAULT; eeprom 184 overrides
CEIL_FLOOR = 200
FULL_CEIL = 2000
# Src/DroneCAN/DroneCAN.c default_settings[43] / factory/*_eeprom_defaults.json
DEFAULT_TEMP_LIMIT_C = 105
# The published degree is rounded while the derate uses full-resolution Q12,
# so the ceiling can legitimately sit up to half a degree either side of the
# curve for the degree being reported: 0.5 C * (1800/band) counts, plus slack.
CEIL_TOL = 75


def _expected_ceiling(temp_c, limit_c=DEFAULT_TEMP_LIMIT_C, band=DERATE_BAND_C):
    over = temp_c - limit_c
    if over <= 0:
        return FULL_CEIL
    if over >= band:
        return CEIL_FLOOR
    return FULL_CEIL - ((FULL_CEIL - CEIL_FLOOR) * over) // band


def _open_ctl(sitl):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(('127.0.0.1', sitl.state_port))
    s.settimeout(0.5)
    return s


def _zc_stats(ctl, retries=8):
    want = struct.calcsize(STATS_FMT)
    for _ in range(retries):
        ctl.send(struct.pack('<HBB', STATE_MAGIC_CMD, 9, 0))
        try:
            pkt = ctl.recv(512)
        except socket.timeout:
            continue
        # the model-load reply shares this socket; match on magic, not order
        if len(pkt) >= want:
            vals = struct.unpack_from(STATS_FMT, pkt)
            if vals[0] == ZC_STATS_MAGIC:
                assert vals[1] >= 7, (
                    'ZC_STATS version %d lacks duty-limit fields' % vals[1])
                return dict(zip(STATS_FIELDS, vals[3:]))
    raise AssertionError('no v7 ZC_STATS reply from SITL state port')


def _hot_model(workdir, temp_c):
    '''an unloaded plant reporting a chosen ESC temperature'''
    spec = {
        'motor': {'kv': 900, 'poles': 14, 'resistance': 0.045,
                  'inductance': 2.1e-5, 'inertia': 2.0e-5, 'damping': 1.0e-6,
                  'static_friction': 0.003, 'load_k_omega2': 2.0e-9},
        'battery': {'voltage': 16.8, 'resistance': 0.012},
        'esc': {'temperature_c': float(temp_c)},
    }
    path = os.path.join(workdir, 'temp_%d.json' % temp_c)
    with open(path, 'w') as f:
        json.dump(spec, f)
    return os.path.abspath(path)


def _settle_at(sim, ctl, workdir, temp_c, timeout=10.0):
    '''load a plant at temp_c and wait for the die filter to reach it'''
    sim.load_model(_hot_model(workdir, temp_c))
    deadline = time.time() + timeout
    st = None
    while time.time() < deadline:
        time.sleep(0.2)
        st = _zc_stats(ctl)
        if st['degrees_celsius_filt'] == int(temp_c):
            # Let the Q12 residue drain, then require the reading to still be
            # there: one sample can land mid-transition on a loaded host.
            time.sleep(0.4)
            st = _zc_stats(ctl)
            if st['degrees_celsius_filt'] == int(temp_c):
                return st
    raise AssertionError(
        'die temperature never settled at %d C (raw=%s filt=%s)'
        % (temp_c, st and st['degrees_celsius'],
           st and st['degrees_celsius_filt']))


def _keepalive(sitl, value=0):
    '''DShot frames so the signal-loss reset never fires mid-test'''
    tx = Sender('127.0.0.1', sitl.input_port, sd.TYPE_DSHOT600)
    tx.value = value  # Sender starts its thread in __init__
    return tx


def _assert_on_curve(st, temp_c, band=DERATE_BAND_C):
    '''the ceiling must lie on the derate curve for the temperature the
    firmware is actually reporting - not for the one the test asked for.
    Under host load the filter can still be a degree away from the request,
    which is a slow SITL, not a wrong derate.'''
    filt = st['degrees_celsius_filt']
    want = _expected_ceiling(filt, band=band)
    got = st['thermal_duty_ceiling']
    assert want - CEIL_TOL <= got <= want + CEIL_TOL, (
        'at %d C (asked %d) ceiling %d, expected ~%d (%s)'
        % (filt, temp_c, got, want, st))


def test_thermal_derate_is_continuous_and_armed_by_default(
        sitl_factory, state_stream, workdir):
    '''the ceiling follows temperature smoothly, on the shipped defaults'''
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    sim = state_stream(sitl)
    assert wait_for_state(sim), sitl.log_tail()
    ctl = _open_ctl(sitl)
    tx = _keepalive(sitl)
    try:
        # Cool: no derate. Also proves the limiter is not simply stuck low.
        st = _settle_at(sim, ctl, workdir, DEFAULT_TEMP_LIMIT_C - 10)
        assert st['thermal_duty_ceiling'] == FULL_CEIL, st

        # Walk the band a degree at a time. The old threshold implementation
        # drops ~1550 counts on the first of these steps.
        prev = FULL_CEIL
        steps = []
        for temp in range(DEFAULT_TEMP_LIMIT_C, DEFAULT_TEMP_LIMIT_C + 6):
            st = _settle_at(sim, ctl, workdir, temp)
            ceil = st['thermal_duty_ceiling']
            assert ceil <= prev, (
                'ceiling rose at %d C: %d > %d' % (temp, ceil, prev))
            _assert_on_curve(st, temp)
            steps.append(prev - ceil)
            prev = ceil
        assert max(steps) <= 200, (
            'derate is a cliff, not a ramp: per-degree steps %s' % steps)
        assert sum(steps) > 250, 'no derate across the band: steps %s' % steps

        # Deep in the band the floor holds - authority, not a stop.
        st = _settle_at(sim, ctl, workdir,
                        DEFAULT_TEMP_LIMIT_C + DERATE_BAND_C + 8)
        assert st['thermal_duty_ceiling'] == CEIL_FLOOR, st
    finally:
        tx.stop()


def test_ceilings_compose_by_min_under_throttle(
        sitl_factory, state_stream, workdir):
    '''the applied ceiling is the binding limiter, not a product of both'''
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    sim = state_stream(sitl)
    assert wait_for_state(sim), sitl.log_tail()
    ctl = _open_ctl(sitl)

    tx = _keepalive(sitl, value=0)
    try:
        # duty_limit_ceiling is only recomputed on a throttle pass, so the
        # ESC has to arm first (a second of confirmed zero) and then be
        # given real throttle - otherwise the field reads its 2000 init.
        deadline = time.time() + 8.0
        while time.time() < deadline and not _zc_stats(ctl)['armed']:
            time.sleep(0.2)
        assert _zc_stats(ctl)['armed'], 'never armed on a zero stream'
        tx.value = 1200
        time.sleep(0.3)
        st = _settle_at(sim, ctl, workdir, DEFAULT_TEMP_LIMIT_C + 10)
        expect = min(st['thermal_duty_ceiling'], st['current_duty_ceiling'])
        assert st['duty_limit_ceiling'] == expect, (
            'applied %d != min(thermal %d, current %d)'
            % (st['duty_limit_ceiling'], st['thermal_duty_ceiling'],
               st['current_duty_ceiling']))
        # Thermal is mid-band and binding; the current loop is slack on an
        # unloaded plant. A product of the two factors would sit well below.
        assert st['duty_limit_ceiling'] == st['thermal_duty_ceiling'], st
        assert st['current_duty_ceiling'] == FULL_CEIL, (
            'current ceiling should be slack unloaded: %s' % (st,))
        assert CEIL_FLOOR < st['duty_limit_ceiling'] < FULL_CEIL, (
            'thermal ceiling should be mid-band here: %s' % (st,))
    finally:
        tx.value = 0
        time.sleep(0.2)
        tx.stop()


def test_derate_band_is_tunable(sitl_factory, state_stream, workdir):
    '''TEMP_DERATE_BAND changes the slope, not just the endpoint

    A narrow band is the closest thing to the "hard" response style other
    vendors expose, and it must stay a ramp: 5 C still gives 5 observable
    steps, not a cliff.
    '''
    import sitl_params

    band = 5
    with open(os.path.join(workdir, 'am32_eeprom.bin'), 'wb') as f:
        f.write(sitl_params.build_image({
            'TEMPERATURE_LIMIT': DEFAULT_TEMP_LIMIT_C,
            'TEMP_DERATE_BAND': band,
        }))

    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    sim = state_stream(sitl)
    assert wait_for_state(sim), sitl.log_tail()
    ctl = _open_ctl(sitl)
    tx = _keepalive(sitl)
    try:
        # Same degree, steeper slope: 3 C in must be far lower than it is on
        # the 15 C default, and the floor must arrive at onset+5 not onset+15.
        st = _settle_at(sim, ctl, workdir, DEFAULT_TEMP_LIMIT_C + 3)
        _assert_on_curve(st, DEFAULT_TEMP_LIMIT_C + 3, band=band)
        assert st['thermal_duty_ceiling'] < _expected_ceiling(
            DEFAULT_TEMP_LIMIT_C + 3, band=DERATE_BAND_C), (
            'band setting had no effect on the slope: %s' % (st,))

        st = _settle_at(sim, ctl, workdir, DEFAULT_TEMP_LIMIT_C + band + 2)
        assert st['thermal_duty_ceiling'] == CEIL_FLOOR, st
    finally:
        tx.stop()


def test_current_limiter_holds_the_limit_under_load(
        sitl_factory, state_stream, workdir):
    '''the current loop actually HOLDS the setpoint on a loaded plant

    This is the gain check, and it exists because reasoning about the loop
    got it backwards once already. The ceiling moves by
    doPidCalculations()/10000 duty units per tick, and that integer divide
    is a dead zone: nothing moves until the overshoot exceeds 5000/Kp
    centiamps. "Softening" the loop by lowering current_P widens that dead
    zone until the limiter is inert. Same plant, 8 A limit, measured:
    P=100 -> 7.7 A, P=50 -> 7.1 A, P=25 -> 6.5 A, P=5 -> 14.7 A.

    So the assertion is on the REGULATED CURRENT, not on the ceiling: a
    limiter that lets current run at twice the setpoint is the failure
    mode, and only a current assertion catches it.
    '''
    import pytest
    import sitl_params

    limit_raw = 4               # 2 A per count -> 8 A
    limit_ca = limit_raw * 2 * 100
    with open(os.path.join(workdir, 'am32_eeprom.bin'), 'wb') as f:
        f.write(sitl_params.build_image({
            'CURRENT_LIMIT': limit_raw,
            'TEMPERATURE_LIMIT': 255,   # isolate the current loop
            'MOTOR_KV': (380 - 20) // 40,
            'MOTOR_POLES': 22,
        }))

    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    sim = state_stream(sitl)
    assert wait_for_state(sim), sitl.log_tail()
    ctl = _open_ctl(sitl)
    # heavy_13inch: pulls ~16 A unlimited here, i.e. 2x the limit above.
    sim.load_model(os.path.join(MODELS, 'heavy_13inch.json'))
    time.sleep(0.6)

    tx = _keepalive(sitl, value=0)
    try:
        deadline = time.time() + 8.0
        while time.time() < deadline and not _zc_stats(ctl)['armed']:
            time.sleep(0.2)
        assert _zc_stats(ctl)['armed'], 'never armed on a zero stream'

        tx.value = 1800  # ask for far more than 8 A of load
        time.sleep(6.0)  # spool the heavy prop and let the loop settle

        samples = [_zc_stats(ctl) for _ in _tick(40, 0.05)]
        # Ignore samples taken while the motor was not driving: the ceiling
        # is released to 2000 there by design, and current is 0.
        driving = [s for s in samples if s['running'] and s['actual_current'] > 0]
        if len(driving) < 10:
            pytest.skip('plant never ran steadily on this host: %d/%d samples'
                        % (len(driving), len(samples)))

        avg_ca = sum(s['actual_current'] for s in driving) / len(driving)
        peak_ca = max(s['actual_current'] for s in driving)
        assert avg_ca <= limit_ca * 1.25, (
            'current not held: avg %.1f A against a %.1f A limit '
            '(gains too low - see the dead-zone note in control_loop.c)'
            % (avg_ca / 100.0, limit_ca / 100.0))
        assert peak_ca <= limit_ca * 1.6, (
            'current peak %.1f A against a %.1f A limit'
            % (peak_ca / 100.0, limit_ca / 100.0))
        # It must be the limiter doing this, not a plant that cannot pull
        # the current: the ceiling has to be binding well below full duty.
        assert min(s['current_duty_ceiling'] for s in driving) < FULL_CEIL - 100, (
            'ceiling never bound; the current assertion above is vacuous: %s'
            % ([s['current_duty_ceiling'] for s in driving][:8],))
        st = driving[-1]
        assert st['duty_limit_ceiling'] == st['current_duty_ceiling'], st
    finally:
        tx.value = 0
        time.sleep(0.2)
        tx.stop()


def test_thermal_limit_can_still_be_disabled(
        sitl_factory, state_stream, workdir):
    '''255 keeps the escape hatch: no derate however hot it gets'''
    import sitl_params

    with open(os.path.join(workdir, 'am32_eeprom.bin'), 'wb') as f:
        f.write(sitl_params.build_image({'TEMPERATURE_LIMIT': 255}))

    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    sim = state_stream(sitl)
    assert wait_for_state(sim), sitl.log_tail()
    ctl = _open_ctl(sitl)
    tx = _keepalive(sitl)
    try:
        st = _settle_at(sim, ctl, workdir, 130)
        assert st['thermal_duty_ceiling'] == FULL_CEIL, (
            'derate engaged with the limit disabled: %s' % (st,))
    finally:
        tx.stop()


def test_fine_ramp_keeps_the_startup_rate(sitl_factory, state_stream, workdir):
    '''max_ramp < 10 must not hand the spool-up ramp to a cruise setting

    ARK_G431_CAN ships max_ramp 5 (0.5 %/ms, matching larger 12S ESCs). That
    is fine mode: one step every 10th 20 kHz tick, so the stored number means
    a tenth of what it means in coarse mode. It used to be applied to all
    three regimes, which handed RAMP_SPEED_STARTUP to the cruise value and
    left the racer plant unable to start at all (test_acq_desync_rail, 12 s,
    deterministic). settings.c now scales the coarse startup ceiling into
    fine-cadence units instead.

    Read straight out of ZC_STATS: no motor, no timing, so this cannot go
    flaky and cannot pass by accident.
    '''
    import sitl_params

    with open(os.path.join(workdir, 'am32_eeprom.bin'), 'wb') as f:
        f.write(sitl_params.build_image({'MAX_RAMP': 5}))

    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    sim = state_stream(sitl)
    assert wait_for_state(sim), sitl.log_tail()
    ctl = _open_ctl(sitl)
    tx = _keepalive(sitl)
    try:
        st = _zc_stats(ctl)
        assert st['ramp_divider'] == 9, 'not in fine mode: %s' % (st,)
        # RAMP_SPEED_STARTUP (2, coarse) x10 == the same %/ms at this cadence.
        assert st['max_ramp_startup'] == 20, (
            'startup ramp took the cruise value: %s' % (st,))
        assert st['max_ramp_low'] == 5 and st['max_ramp_high'] == 5, (
            'cruise regimes did not take the eeprom value: %s' % (st,))
    finally:
        tx.stop()
