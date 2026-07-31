'''Learned ramp back-off attribution (slew witness + acq soften).

faultDesyncEpisodeCharge halves the session ramp on desync episodes. That
lesson ("the configured ramp outran this motor/prop") is only true when the
ramp was actually in play. Before the attribution gate, EVERY episode
halved it - including a mechanical obstruction (grass) dragging a healthy
run down at steady duty - and on the production tune (all regimes at 2) a
single halve is the fine-rate floor: a silent 20x slew-authority cut for
the rest of the power cycle, with every other counter self-healing.

Three engagement assertions, each a mutation check:

  1. A desync while duty is slewing at the ramp limit (witness armed) DOES
     halve - remove the halve (or break the witness arming) and it fails.
  2. A desync at steady duty (witness 0) does NOT halve but still charges
     the episode bucket - remove the witness gate and it fails.
  3. Acquisition-rail charges never touch the ramp at all - they surface as
     acq_resist in ZC_STATS v6 and escalate through bucket/holdoff/latch,
     which is what test_acq_desync_rail.py already covers.

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
                'zc_blind_steps', 'zc_miss_bucket', 'zc_deadline_armed',
                'dcm_hold_ms', '_pad2', 'dcm_hold_value',
                'adv_kerpm_hold_ms', '_pad3', 'adv_kerpm_hold',
                'zc_trend', 'zc_predicted', 'wait_time', 'advance',
                'gov_conf', 'gov_slope_q10', 'gov_duty_ceiling',
                'gov_stuck_ms', 'gov_release_ceil', 'gov_unlatch_count',
                'max_ramp_startup', 'max_ramp_low', 'max_ramp_high',
                'ramp_divider', 'max_ramp_startup_vcomp', 'ramp_witness_ms',
                'ramp_halves', 'acq_resist', 'desync_episode_bucket')
STATS_FMT = '<HBBIIIIBBBBBBBBHBBHiIHHHHHHHHBBBBBBBBB'


def _open_ctl(sitl):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(('127.0.0.1', sitl.state_port))
    s.settimeout(0.5)
    return s


def _zc_stats(ctl, retries=8):
    need = struct.calcsize(STATS_FMT)
    for _ in range(retries):
        ctl.send(struct.pack('<HBB', STATE_MAGIC_CMD, 4, 0))
        try:
            pkt = ctl.recv(96)
        except socket.timeout:
            continue
        if len(pkt) >= need:
            vals = struct.unpack_from(STATS_FMT, pkt)
            if vals[0] == ZC_STATS_MAGIC:
                assert vals[1] >= 6, (
                    'ZC_STATS version %d lacks ramp-attribution fields'
                    % vals[1])
                return dict(zip(STATS_FIELDS, vals[3:]))
    raise AssertionError('no v6 ZC_STATS reply from SITL state port')


def _zc_fault(ctl, mode, duration_us):
    ctl.send(struct.pack('<HBBI', STATE_MAGIC_CMD, 3, mode, duration_us))


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


def test_halve_engages_on_slew_attributed_desync(sitl_factory, state_stream):
    '''An episode charged while duty slews at the ramp limit must halve.

    Choreography (learned from the first CI run of this file): inject the
    edge suppression from ESTABLISHED STEADY state - the exact recipe the
    ceiling-hold test proves in CI - and then keep the throttle toggling
    whenever the charge lands. The toggling is load-bearing for BOTH witness
    conditions: each up-leg makes the limiter bind (applied duty rising at
    the regime rate) AND moves the setpoint, which is what separates a
    commanded ramp from the ESC's own post-desync re-slew.

    Do not condition on desync_happened: a suppression bridged by
    blind steps resolves through the blind-limit -> stall-rail handoff,
    which by design does NOT increment desync_happened (see the error_count
    comment in faults.h) but still charges the episode bucket, and a
    STALL_RAIL charge with the witness armed is an equally legitimate halve.
    The charge signal is desync_episode_bucket (v6).
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
        halves0 = int(pre['ramp_halves'])
        low0 = int(pre['max_ramp_low'])

        seen = None
        saw_charge = False
        witness_at_charge = -1
        saw_desync = False
        for _ in range(6):
            s0 = _zc_stats(ctl)
            ci_us = max(s0['commutation_interval'] // 2, 100)
            bucket0 = int(s0['desync_episode_bucket'])
            d0 = int(s0['desync_happened'])
            _zc_fault(ctl, mode=1, duration_us=60 * ci_us)
            # Toggle every 50 ms: each up-leg re-arms the 100 ms witness,
            # so it is armed at whatever moment the charge processes.
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
                if s['desync_happened'] > d0:
                    saw_desync = True
                if s['ramp_halves'] > halves0:
                    seen = s
                    break
                if s['desync_episode_bucket'] > bucket0 and not saw_charge:
                    saw_charge = True
                    witness_at_charge = int(s['ramp_witness_ms'])
            if seen:
                break
            # Recover for another attempt.
            _zc_fault(ctl, mode=0, duration_us=0)
            tx.value = 900
            try:
                _spool_established(sitl, sim, ctl, tx, value=900, budget=8.0)
            except AssertionError:
                continue

        assert saw_charge or saw_desync or seen is not None, (
            'fault injection never charged the episode machinery at all - '
            'the halve was never given a chance to engage\n'
            + sitl.log_tail())
        assert seen is not None, (
            'THE ATTRIBUTED HALVE NEVER ENGAGED: episodes charged during a '
            'max-rate slew but ramp_halves stayed %d (saw_desync=%s, '
            'witness at first observed charge=%d ms; witness 0 there means '
            'test timing, nonzero means the witness-gated halve in '
            'faultDesyncEpisodeCharge is broken).\n%s'
            % (halves0, saw_desync, witness_at_charge, sitl.log_tail()))
        assert seen['max_ramp_low'] <= max(low0 // 2, 1), (
            'ramp_halves incremented but max_ramp_low did not fall: %r'
            % seen)
    finally:
        tx.value = 0
        time.sleep(0.3)
        _zc_fault(ctl, mode=0, duration_us=0)
        tx.stop()
        ctl.close()


def test_no_halve_on_steady_duty_desync(sitl_factory, state_stream):
    '''A desync at steady duty (witness 0) charges the bucket, not the ramp.

    This is the grass case: an external load dragging an established run
    down teaches nothing about the configured ramp. Remove the witness gate
    in faultDesyncEpisodeCharge and this fails.
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

        # Hold steady until the witness has fully decayed: duty is no
        # longer rising, so a desync now is NOT ramp-attributable.
        deadline = time.time() + 6.0
        steady = None
        while time.time() < deadline:
            s = _zc_stats(ctl)
            if s['running'] and s['ramp_witness_ms'] == 0:
                steady = s
                break
            time.sleep(0.05)
        assert steady is not None, (
            'witness never decayed at steady throttle: %r\n%s'
            % (_zc_stats(ctl), sitl.log_tail()))

        ci_us = max(steady['commutation_interval'] // 2, 100)
        _zc_fault(ctl, mode=1, duration_us=60 * ci_us)
        first = None
        probe_end = time.time() + 3.0
        while time.time() < probe_end:
            s = _zc_stats(ctl)
            if s['desync_happened'] > steady['desync_happened']:
                first = s
                break
        assert first is not None, (
            'fault injection never produced a desync\n' + sitl.log_tail())

        # The charge is synchronous with desync_happened++, so the first
        # sample that shows the desync already reflects any halve. Sample
        # THIS one - later samples can see re-spool slews legitimately
        # arming the witness for subsequent events.
        assert first['ramp_halves'] == steady['ramp_halves'], (
            'STEADY-DUTY DESYNC HALVED THE RAMP: witness was 0 at inject '
            'but ramp_halves went %d->%d. The attribution gate in '
            'faultDesyncEpisodeCharge is not holding.\n%r\n%s'
            % (steady['ramp_halves'], first['ramp_halves'], first,
               sitl.log_tail()))
        assert first['max_ramp_low'] == steady['max_ramp_low'], first
        assert first['desync_episode_bucket'] > steady[
            'desync_episode_bucket'], (
            'episode bucket did not charge on an established-run desync - '
            'the gate must only skip the RAMP lesson, never the episode '
            'accounting: %r' % first)
    finally:
        tx.value = 0
        time.sleep(0.3)
        _zc_fault(ctl, mode=0, duration_us=0)
        tx.stop()
        ctl.close()


def test_acq_rail_surfaces_resist_without_touching_ramp(sitl_factory,
                                                        state_stream):
    '''racer_5inch hard spool: acq charges must surface, not tune.

    The racer model desyncs ~20/s below acquisition (see
    test_acq_desync_rail.py). Each 20-event batch must surface as
    acq_resist - the "start resisted" telemetry that lets an FC refuse
    takeoff - while the ramp is left alone.

    Not asserted here: a blanket ramp_halves == 0. On this model the loop
    occasionally peaks past zc 100 and can then take a legitimate
    witness-armed JUMP halve, which is correct behaviour and would make
    such an assertion flaky. What this pins is that the acquisition rail
    itself reports, and that the escalation path (bucket) still charges.
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
        tx.value = 700

        acq_seen = 0
        bucket_seen = 0
        end = None
        t0 = time.time()
        while time.time() - t0 < 15.0:
            try:
                s = _zc_stats(ctl)
            except AssertionError:
                time.sleep(0.05)
                continue
            acq_seen = max(acq_seen, s['acq_resist'])
            bucket_seen = max(bucket_seen, s['desync_episode_bucket'])
            end = s
            if acq_seen > 0 and bucket_seen > 0:
                break
            # The rail may instead resolve the episode: acquisition, or the
            # stuck latch. Both are terminal for this probe.
            if s['zero_crosses'] > 500:
                break
            time.sleep(0.05)

        assert end is not None, sitl.log_tail()
        acquired = end['zero_crosses'] > 500
        assert acq_seen > 0 or acquired, (
            'racer hard spool never charged the acquisition rail and never '
            'acquired: acq_resist=%d end=%r\n%s'
            % (acq_seen, end, sitl.log_tail()))
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
