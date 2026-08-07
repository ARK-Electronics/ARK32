'''The configured ramp is FIXED: no desync episode may ever mutate it.

The desync episode charge used to halve every regime's duty step on a desync
episode, permanently for the power cycle ("learned ramp back-off"). That is
per-ESC state the flight controller cannot see, and the mixer assumes
symmetric actuators - one ESC that has learned a slower ramp than its
siblings is an asymmetric vehicle with no way to report it. It crashed a
vehicle taking off after a ground spool in long grass, where obstruction at
STEADY duty dragged an established run down and the episode was misread as
"ramp too fast". On the production tune (max_ramp 20 -> all regimes 2) one
halve already IS the fine-rate floor: a silent 20x slew-authority cut.

An attribution gate (halve only when the slew limiter was demonstrably
binding on rising demand) was tried and rejected: it narrows the
misattribution but keeps the architecture. The mechanism is gone instead.

Three assertions, each a mutation check against restoring it:

  1. An episode charged while duty slews AT the ramp limit - the exact
     condition the old gate treated as proof of "ramp too fast", and the
     one case a re-introduced halve would definitely fire on - leaves the
     configured ramp untouched.
  2. The grass case: a desync at steady duty leaves the ramp untouched AND
     does not stop the motor. An obstruction dragging a turning rotor is
     lost sync, not a lost rotor, and upstream AM32 rides it out.
  3. A racer hard spool surfaces SOMETHING observable - acq_resist (the
     "start resisted" telemetry an FC can refuse takeoff on), a stall trip,
     or a plain acquisition - and never touches the ramp.

Ramp fields asserted: the three configured regime steps plus ramp_divider.
Deliberately NOT max_ramp_startup_vcomp - that is the voltage-compensated
working copy and moves legitimately with pack sag.

CI is the authority for this file: the local SITL harness in the dev
container does not spin the motor at all (test_dshot fails with rpm=0), so
none of this was validated locally - see the PR discussion.
'''

from __future__ import annotations

import os
import socket
import struct
import time

import sitl_dshot as sd
from sitl_harness import SITL_DIR, Sender, rpm_from_state, wait_for_state

STATE_MAGIC_CMD = 0x5353
ZC_STATS_MAGIC = 0x5356
MODELS = os.path.join(SITL_DIR, 'models')

STATS_FIELDS = ('zero_crosses', 'commutation_interval', 'dropped_edges',
                'desync_happened', 'old_routine', 'running', 'armed',
                'zc_blind_steps', 'zc_stale_q8', 'zc_deadline_armed',
                'dcm_hold_ms', '_pad2', 'dcm_hold_value',
                'adv_kerpm_hold_ms', '_pad3', 'adv_kerpm_hold',
                'zc_trend', 'zc_predicted', 'wait_time', 'advance',
                'gov_conf', 'gov_slope_q10', 'gov_duty_ceiling',
                'gov_stuck_ms', 'gov_release_ceil', 'gov_unlatch_count',
                'max_ramp_startup', 'max_ramp_low', 'max_ramp_high',
                'ramp_divider', 'max_ramp_startup_vcomp', 'acq_resist',
                'bemf_timeout')
STATS_FMT = '<HBBIIIIBBBBBBBBHBBHiIHHHHHHHHBBBBBBB'

RAMP_FIELDS = ('max_ramp_startup', 'max_ramp_low', 'max_ramp_high',
               'ramp_divider')


def _ramp(s):
    return tuple(int(s[f]) for f in RAMP_FIELDS)


def _assert_ramp_fixed(base, s, what, sitl):
    assert _ramp(s) == base, (
        'THE CONFIGURED RAMP WAS MUTATED (%s): %s went %r -> %r. Nothing in '
        'the fault machinery may change tuning - the learned halve was '
        'removed deliberately (see the note at the top of faults.c).\n%r\n%s'
        % (what, RAMP_FIELDS, base, _ramp(s), s, sitl.log_tail()))


def _open_ctl(sitl):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(('127.0.0.1', sitl.state_port))
    s.settimeout(0.5)
    return s


def _zc_stats(ctl, retries=8):
    need = struct.calcsize(STATS_FMT)
    for _ in range(retries):
        # cmd 9 ZC_STATS (ARK extension; remapped from 4 when upstream
        # SITL claimed 3/4 for tone/audio subscribe — see #77).
        ctl.send(struct.pack('<HBB', STATE_MAGIC_CMD, 9, 0))
        try:
            pkt = ctl.recv(512)
        except socket.timeout:
            continue
        if len(pkt) >= need:
            vals = struct.unpack_from(STATS_FMT, pkt)
            if vals[0] == ZC_STATS_MAGIC:
                assert vals[1] >= 6, (
                    'ZC_STATS version %d lacks the ramp/episode fields'
                    % vals[1])
                return dict(zip(STATS_FIELDS, vals[3:]))
    raise AssertionError('no v6 ZC_STATS reply from SITL state port')


def _zc_fault(ctl, mode, duration_us):
    # cmd 8 ZC_FAULT (ARK extension; remapped from 3 — see #77).
    ctl.send(struct.pack('<HBBI', STATE_MAGIC_CMD, 8, mode, duration_us))


def _spool_established(sitl, sim, ctl, tx, value=900, rpm_min=2000.0,
                       budget=12.0):
    '''Drive to established closed loop clearly above idle.

    RPM_MIN must clear real running, not just the zc gate - see the
    ceiling-hold test for the arithmetic (arm gates sit near ~571 mech rpm
    on the 900kv/14p model, so 2000 keeps ~4x headroom).
    '''
    tx.value = value
    deadline = time.time() + budget
    rpm = 0.0
    while time.time() < deadline:
        s = _zc_stats(ctl)
        if s['running'] and not s['old_routine'] and s['zero_crosses'] > 100:
            rpm = rpm_from_state(sim, 0.2)
            if rpm > rpm_min:
                return s
        time.sleep(0.05)
    raise AssertionError(
        'never reached established closed loop above %.0f rpm '
        '(last rpm=%.0f)\n%s' % (rpm_min, rpm, sitl.log_tail()))


def test_slew_desync_leaves_ramp_fixed(sitl_factory, state_stream):
    '''An episode charged during a max-rate slew must not retune the ESC.

    This is the case a re-introduced learned halve would certainly fire on,
    so it is the load-bearing mutation check: restore the halve anywhere in
    the fault path and this test fails.

    Choreography (learned from the first CI run of this file): inject the
    edge suppression from ESTABLISHED STEADY state - the recipe the
    ceiling-hold test proves in CI - then keep the throttle toggling so the
    limiter is genuinely binding on a rising setpoint when the charge lands.

    Do not condition on desync_happened alone: a suppression bridged by
    blind steps resolves through the dead-reckoning-budget -> stall-rail
    handoff, which by design does NOT increment desync_happened (see the
    error_count comment in faults.h) but does increment
    bemf_timeout_happened. Either counter moving is a real fault event.
    '''
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    sim = state_stream(sitl)
    assert wait_for_state(sim), sitl.log_tail()
    sim.load_model(os.path.join(MODELS, 'unloaded.json'))
    time.sleep(1.0)

    tx = Sender('127.0.0.1', sitl.input_port, sd.TYPE_DSHOT600)
    ctl = _open_ctl(sitl)
    try:
        time.sleep(2.2)
        pre = _spool_established(sitl, sim, ctl, tx, value=900)
        base = _ramp(pre)

        saw_charge = False
        saw_desync = False
        for _ in range(6):
            s0 = _zc_stats(ctl)
            ci_us = max(s0['commutation_interval'] // 2, 100)
            bt0 = int(s0['bemf_timeout'])
            d0 = int(s0['desync_happened'])
            # The rail is time-based (BEMF_STALL_TICKS = 22.5 ms of dead
            # reckoning), not the step count this multiplier was sized
            # against: 60 * ci_us floors at 6 ms, rides out entirely on
            # blind steps and produces neither a desync nor a stall trip.
            # Floor at 3x the budget, as the blind-budget test does.
            _zc_fault(ctl, mode=1, duration_us=max(60 * ci_us, 68_000))
            # Toggle every 50 ms so the limiter is binding on a RISING
            # setpoint across the whole window the charge can land in.
            hi = True
            last_toggle = 0.0
            probe_end = time.time() + 0.5
            while time.time() < probe_end:
                now = time.time()
                if now - last_toggle > 0.05:
                    tx.value = 1900 if hi else 1100
                    hi = not hi
                    last_toggle = now
                s = _zc_stats(ctl)
                _assert_ramp_fixed(base, s, 'during max-rate slew', sitl)
                if s['desync_happened'] > d0:
                    saw_desync = True
                    saw_charge = True
                if s['bemf_timeout'] > bt0:
                    saw_charge = True
            if saw_charge:
                break
            # Recover for another attempt.
            _zc_fault(ctl, mode=0, duration_us=0)
            tx.value = 900
            try:
                _spool_established(sitl, sim, ctl, tx, value=900, budget=8.0)
            except AssertionError:
                continue

        # Require a real fault event on one of the two counters, or the
        # test proved nothing - a re-introduced halve would never have been
        # given a chance to fire.
        assert saw_charge, (
            'fault injection produced neither a desync nor a stall trip, so '
            'this test proved nothing (saw_desync=%s)\n%s'
            % (saw_desync, sitl.log_tail()))
        # Re-check after the dust settles: the post-episode re-spool is
        # itself a max-rate slew against a static setpoint.
        tx.value = 900
        _zc_fault(ctl, mode=0, duration_us=0)
        settle_end = time.time() + 1.5
        while time.time() < settle_end:
            _assert_ramp_fixed(base, _zc_stats(ctl), 'post-episode re-spool',
                               sitl)
            time.sleep(0.05)
    finally:
        tx.value = 0
        time.sleep(0.3)
        _zc_fault(ctl, mode=0, duration_us=0)
        tx.stop()
        ctl.close()


def test_steady_duty_desync_leaves_ramp_fixed(sitl_factory, state_stream):
    '''The grass case: obstruction at steady duty must not retune or stop.

    An external load dragging an established run down teaches nothing about
    the configured ramp. This is the episode that crashed the vehicle.

    Two assertions now, because this PR changed what a desync on a TURNING
    rotor is allowed to do. The ramp must not move (as before), and the
    motor must keep running: a jump-desync is lost sync, not a lost rotor,
    and upstream AM32 rides it out at flight throttle rather than cutting
    power. Only faultHandleStuckRotorIfNeeded stops, and only when repeated
    stalls prove poll mode finds no crossings either.

    Riding the fault out entirely - no desync at all - is a legitimate
    outcome here and is not a failure; blind stepping exists to produce
    exactly that. What is NOT allowed is a desync that stops the motor.
    '''
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    sim = state_stream(sitl)
    assert wait_for_state(sim), sitl.log_tail()
    sim.load_model(os.path.join(MODELS, 'unloaded.json'))
    time.sleep(1.0)

    tx = Sender('127.0.0.1', sitl.input_port, sd.TYPE_DSHOT600)
    ctl = _open_ctl(sitl)
    try:
        time.sleep(2.2)
        _spool_established(sitl, sim, ctl, tx, value=900)

        # Hold the stick still so duty is genuinely steady when the fault
        # lands: no rising demand, nothing the ramp could be blamed for.
        time.sleep(0.6)
        steady = _zc_stats(ctl)
        assert steady['running'], (steady, sitl.log_tail())
        base = _ramp(steady)

        ci_us = max(steady['commutation_interval'] // 2, 100)
        # Same time-budget correction as above: below BEMF_STALL_TICKS the
        # blackout rides out on blind steps and exercises nothing.
        _zc_fault(ctl, mode=1, duration_us=max(60 * ci_us, 68_000))
        first = None
        stopped = None
        rode_out = False
        probe_end = time.time() + 3.0
        while time.time() < probe_end:
            s = _zc_stats(ctl)
            _assert_ramp_fixed(base, s, 'steady-duty desync', sitl)
            if s['zc_blind_steps'] > steady['zc_blind_steps']:
                rode_out = True
            # The motor must never stop while throttle is commanded up.
            if not s['running'] and stopped is None:
                stopped = s
            if s['desync_happened'] > steady['desync_happened']:
                first = s
                break
        assert stopped is None, (
            'the motor STOPPED on a steady-duty fault while throttle was '
            'commanded up. Lost sync on a turning rotor must degrade to '
            'poll re-acquisition with the bridge live, not cut power - only '
            'a genuinely stalled rotor may stop: %r\n%s'
            % (stopped, sitl.log_tail()))
        assert first is not None or rode_out, (
            'fault injection produced neither a desync nor a blind step, so '
            'nothing was exercised\n' + sitl.log_tail())
    finally:
        tx.value = 0
        time.sleep(0.3)
        _zc_fault(ctl, mode=0, duration_us=0)
        tx.stop()
        ctl.close()


def test_acq_rail_surfaces_resist_without_touching_ramp(sitl_factory,
                                                        state_stream):
    '''racer_5inch hard spool: episodes must surface, not tune.

    The ramp assertion runs on every sample and is unconditional, which it
    could not be while the attribution gate existed: this model peaks past
    zc 100 and would then take a legitimate witness-armed halve. With no
    learned state at all, "the ramp never moves" holds for every path.

    Which rail the model exercises is NOT pinned here, deliberately. Before
    #77 it desynced ~20/s below acquisition and counted acq_resist; with the
    upstream SITL update it now acquires and then desyncs at established
    rpm, so the 20-event acq batch never completes and the stall rail trips
    instead. Both are legitimate, and test_acq_desync_rail.py is what pins
    the acquisition rail's mechanism. What must hold either way is that
    SOMETHING observable happened - otherwise a re-introduced halve was
    never given a chance to fire and this test proves nothing - and that
    the ramp did not move.

    Stall trips are the load-bearing case: those established-run events are
    exactly the ones the old halve fired on.
    '''
    path = os.path.join(MODELS, 'racer_5inch.json')
    assert os.path.isfile(path), path
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    sim = state_stream(sitl)
    assert wait_for_state(sim), sitl.log_tail()
    sim.load_model(path)
    deadline = time.time() + 3.0
    while time.time() < deadline:
        status = (sim.model_status or '').lower()
        if status and 'fail' not in status and 'error' not in status:
            if 'loading' not in status or 'racer' in status or 'ok' in status:
                break
        time.sleep(0.1)

    tx = Sender('127.0.0.1', sitl.input_port, sd.TYPE_DSHOT600)
    ctl = _open_ctl(sitl)
    try:
        time.sleep(2.2)
        base = _ramp(_zc_stats(ctl))
        tx.value = 700

        acq_seen = 0
        bemf_to_seen = 0
        # Peak, NOT the last sample: zero_crosses is reset to 0 by every
        # desync (runtime_loop.c), so a spool that acquires and is then
        # dragged back down reads 0 at the end even though it plainly ran.
        zc_seen = 0
        end = None
        t0 = time.time()
        while time.time() - t0 < 15.0:
            try:
                s = _zc_stats(ctl)
            except AssertionError:
                time.sleep(0.05)
                continue
            _assert_ramp_fixed(base, s, 'hard spool', sitl)
            acq_seen = max(acq_seen, s['acq_resist'])
            bemf_to_seen = max(bemf_to_seen, s['bemf_timeout'])
            zc_seen = max(zc_seen, s['zero_crosses'])
            end = s
            if acq_seen > 0 and bemf_to_seen > 0:
                break
            time.sleep(0.05)

        assert end is not None, sitl.log_tail()
        assert acq_seen > 0 or bemf_to_seen > 0 or zc_seen > 500, (
            'racer hard spool did nothing observable in 15 s - no stall '
            'trip, no resisted start, no acquisition - so a re-introduced '
            'halve would not have been given a chance to fire: '
            'acq_resist=%d bemf_timeout=%d zc_peak=%d end=%r\n%s'
            % (acq_seen, bemf_to_seen, zc_seen, end, sitl.log_tail()))
    finally:
        tx.value = 0
        time.sleep(0.5)
        tx.stop()
        ctl.close()
