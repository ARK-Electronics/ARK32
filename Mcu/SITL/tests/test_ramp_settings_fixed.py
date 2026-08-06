'''The configured ramp is FIXED: no desync episode may ever mutate it.

faultDesyncEpisodeCharge used to halve every regime's duty step on a desync
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
  2. The grass case: an episode at steady duty leaves the ramp untouched
     AND still charges the episode bucket. The always-on protection must
     not be weakened by removing the learned part.
  3. A racer hard spool charges the episode machinery - via the
     acquisition rail (acq_resist, the "start resisted" telemetry an FC can
     refuse takeoff on) or via established-run episodes, whichever this
     model produces - and never touches the ramp.

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
                'desync_episode_bucket')
STATS_FMT = '<HBBIIIIBBBBBBBBHBBHiIHHHHHHHHBBBBBBB'

RAMP_FIELDS = ('max_ramp_startup', 'max_ramp_low', 'max_ramp_high',
               'ramp_divider')


def _ramp(s):
    return tuple(int(s[f]) for f in RAMP_FIELDS)


def _assert_ramp_fixed(base, s, what, sitl):
    assert _ramp(s) == base, (
        'THE CONFIGURED RAMP WAS MUTATED (%s): %s went %r -> %r. Nothing in '
        'the episode machinery may change tuning - the learned halve was '
        'removed deliberately (see faultDesyncEpisodeCharge).\n%r\n%s'
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
            pkt = ctl.recv(96)
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
    so it is the load-bearing mutation check: restore the halve in
    faultDesyncEpisodeCharge and this test fails.

    Choreography (learned from the first CI run of this file): inject the
    edge suppression from ESTABLISHED STEADY state - the recipe the
    ceiling-hold test proves in CI - then keep the throttle toggling so the
    limiter is genuinely binding on a rising setpoint when the charge lands.

    Do not condition on desync_happened: a suppression bridged by blind
    steps resolves through the blind-limit -> stall-rail handoff, which by
    design does NOT increment desync_happened (see the error_count comment
    in faults.h) but still charges the episode bucket. The charge signal is
    desync_episode_bucket (v6).
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
            bucket0 = int(s0['desync_episode_bucket'])
            d0 = int(s0['desync_happened'])
            _zc_fault(ctl, mode=1, duration_us=60 * ci_us)
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
                if s['desync_episode_bucket'] > bucket0:
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

        # Require a real episode charge (bucket motion), not just
        # desync_happened: the learned halve lived only inside
        # faultDesyncEpisodeCharge, and some desync paths never call it.
        assert saw_charge, (
            'fault injection never charged desync_episode_bucket, so this '
            'test proved nothing - a re-introduced halve would not have '
            'been given a chance to fire (saw_desync=%s)\n%s'
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
    '''The grass case: obstruction at steady duty must not retune the ESC.

    An external load dragging an established run down teaches nothing about
    the configured ramp. This is the episode that crashed the vehicle. The
    bucket must still charge - removing the learned halve must not weaken
    the always-on escalation path.
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
        _zc_fault(ctl, mode=1, duration_us=60 * ci_us)
        first = None
        probe_end = time.time() + 3.0
        while time.time() < probe_end:
            s = _zc_stats(ctl)
            _assert_ramp_fixed(base, s, 'steady-duty desync', sitl)
            if s['desync_happened'] > steady['desync_happened']:
                first = s
                break
        assert first is not None, (
            'fault injection never produced a desync\n' + sitl.log_tail())
        assert first['desync_episode_bucket'] > steady[
            'desync_episode_bucket'], (
            'episode bucket did not charge on an established-run desync - '
            'removing the learned halve must not weaken the always-on '
            'escalation path: %r' % first)
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
    #77 it desynced ~20/s below acquisition and charged acq_resist; with the
    upstream SITL update it now acquires and then desyncs at established
    rpm, so the 20-event acq batch never completes and the bucket charges
    through the JUMP/STALL path instead. Both are legitimate, and
    test_acq_desync_rail.py is what pins the acquisition rail's mechanism.
    What must hold either way is that SOMETHING charged - otherwise a
    re-introduced halve was never given a chance to fire and this test
    proves nothing - and that the ramp did not move.

    Bucket motion is the load-bearing case: those established-run episodes
    are exactly the ones the old halve fired on.
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
        bucket_seen = 0
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
            bucket_seen = max(bucket_seen, s['desync_episode_bucket'])
            zc_seen = max(zc_seen, s['zero_crosses'])
            end = s
            if acq_seen > 0 and bucket_seen > 0:
                break
            time.sleep(0.05)

        assert end is not None, sitl.log_tail()
        assert acq_seen > 0 or bucket_seen > 0 or zc_seen > 500, (
            'racer hard spool did nothing observable in 15 s - no episode '
            'charge, no acquisition - so a re-introduced halve would not '
            'have been given a chance to fire: acq_resist=%d bucket=%d '
            'zc_peak=%d end=%r\n%s'
            % (acq_seen, bucket_seen, zc_seen, end, sitl.log_tail()))
        if acq_seen > 0:
            assert bucket_seen > 0, (
                'acq_resist charged (%d) but the episode bucket never moved '
                '- ACQ_FAIL must still escalate through the always-on '
                'protection path: %r\n%s'
                % (acq_seen, end, sitl.log_tail()))
    finally:
        tx.value = 0
        time.sleep(0.5)
        tx.stop()
        ctl.close()
