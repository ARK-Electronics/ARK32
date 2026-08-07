'''Missed-ZC blind-step path tests (PR #41, BLHeli-style timeout commutation).

Uses state-port fault injection (cmd 3) to suppress comparator EXTI
delivery - the comparator LEVEL keeps tracking the physics, so poll mode
and the confirm loop stay honest while the interrupt path sees missed
crossings. Covers the three paths that previously rested only on bench
traces:

  - bridge:      a short full suppression is ridden out with blind steps,
                 no desync, no restart
  - budget:      a full suppression lasting longer than the dead-reckoning
                 budget (BEMF_STALL_TICKS of blind time) hands off to the
                 stall rail, then recovers when the fault clears
  - alternating: dropping every other commutation window (the demag
                 signature) must NOT restart. Every accepted crossing
                 clears the budget, so a 50% miss rate keeps flying on
                 degraded timing the way upstream does. This inverts the
                 old contract, where a leaky bucket tripped the stall rail
                 above a 25% miss rate and the restart could not improve
                 the miss rate that caused it.
'''

from __future__ import annotations

import socket
import struct
import time

import sitl_dshot as sd
from sitl_harness import Sender, rpm_from_state, wait_for_state

STATE_MAGIC_CMD = 0x5353
ZC_STATS_MAGIC = 0x5356

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
                'bemf_timeout', '_pad4', 'duty_cycle')
STATS_FMT = '<HBBIIIIBBBBBBBBHBBHiIHHHHHHHHBBBBBBBBH'


def _open_ctl(sitl):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(('127.0.0.1', sitl.state_port))
    s.settimeout(0.5)
    return s


def _zc_fault(ctl, mode, duration_us):
    ctl.send(struct.pack('<HBBI', STATE_MAGIC_CMD, 8, mode, duration_us))


def _zc_stats(ctl, retries=5):
    # Buffer must exceed the whole ZC_STATS datagram: a UDP recv() smaller
    # than the packet TRUNCATES it and silently drops the tail, so a reply
    # that arrived perfectly well then fails the length check below and is
    # indistinguishable from no reply at all. The wire grows every time a
    # field is appended (v7 is 68 bytes, pinned by a _Static_assert in
    # sitl_state.c), so size this generously rather than to the current
    # packet.
    short = 0
    for _ in range(retries):
        ctl.send(struct.pack('<HBB', STATE_MAGIC_CMD, 9, 0))
        try:
            pkt = ctl.recv(512)
        except socket.timeout:
            continue
        if len(pkt) >= struct.calcsize(STATS_FMT):
            vals = struct.unpack_from(STATS_FMT, pkt)
            if vals[0] == ZC_STATS_MAGIC:
                return dict(zip(STATS_FIELDS, vals[3:]))
        else:
            short = len(pkt)
    if short:
        raise AssertionError(
            'ZC_STATS reply was %d bytes, decoder wants %d - the firmware '
            'packet and STATS_FMT have diverged'
            % (short, struct.calcsize(STATS_FMT)))
    raise AssertionError('no ZC_STATS reply from SITL state port')


def _poll_stats(ctl, seconds):
    '''lockstep-sample stats as fast as the state port answers'''
    out = []
    deadline = time.time() + seconds
    while time.time() < deadline:
        out.append(_zc_stats(ctl))
    return out


def _spin_up(sitl, state_stream, value=500, arm_s=2.2, run_s=3.5):
    '''Spin into established closed loop (old_routine=0).

    value=250 leaves the new plant in open-loop poll on the default
    model (~1300 rpm, never crosses the handoff), so blind-step paths
    never arm. Mid-stick DShot300 is enough to close the loop.
    '''
    sim = state_stream(sitl)
    assert wait_for_state(sim), 'state stream dead\n' + sitl.log_tail()
    tx = Sender('127.0.0.1', sitl.input_port, sd.TYPE_DSHOT300)
    time.sleep(arm_s)
    tx.value = value
    time.sleep(run_s)
    rpm = rpm_from_state(sim)
    assert rpm > 1000, 'motor not spinning: rpm=%.0f\n%s' % (rpm, sitl.log_tail())
    return sim, tx, rpm


def _assert_recovered(ctl, sim, rpm_before, sitl, tx=None):
    # Clear any residual fault injection, then wait for stall-rail restart
    # + open-loop re-acquire under continuous throttle.
    _zc_fault(ctl, mode=0, duration_us=0)
    if tx is not None:
        # brief zero then re-apply: some stall paths need a clean re-arm
        held = tx.value
        tx.value = 0
        time.sleep(0.4)
        tx.value = held
    end = None
    rpm = 0.0
    deadline = time.time() + 10.0
    while time.time() < deadline:
        end = _zc_stats(ctl)
        closed = (end['running'] == 1 and end['old_routine'] == 0
                  and end['zero_crosses'] > 50)
        if closed:
            # poll RPM until the re-spool clears the floor; a single short
            # window right after closed-loop latch is often still <400 on
            # a loaded CI host even though the ring already shows >1k.
            rpm = rpm_from_state(sim, 0.4)
            if rpm > 300:
                break
        time.sleep(0.15)
    assert end is not None
    assert end['running'] == 1 and end['old_routine'] == 0, (
        'no closed-loop recovery: %r\n%s' % (end, sitl.log_tail()))
    assert end['zc_blind_steps'] == 0, end
    assert end['zc_stale_q8'] < 6, end
    assert rpm > 300, (
        'rpm did not recover: %.0f (was %.0f before)\n%s'
        % (rpm, rpm_before, sitl.log_tail()))


def test_blind_step_bridges_short_dropout(sitl_factory, state_stream):
    '''Short EXTI blackout must engage the blind-step path.

    On the upstream inertial-comparator plant a single dropped edge can
    cascade into several blind steps and occasionally a stall-rail
    restart (zero_crosses reset). That is plant-dependent; what this
    test locks is that the blind path *fires* on a short blackout and
    the motor returns to closed loop under continuous throttle. The
    consecutive-limit handoff itself is covered by
    test_blind_step_limit_hands_off_to_stall_rail.
    '''
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    sim, tx, rpm0 = _spin_up(sitl, state_stream)
    try:
        ctl = _open_ctl(sitl)
        pre = _zc_stats(ctl)
        assert pre['old_routine'] == 0 and pre['running'] == 1, pre
        # half-period in us (commutation_interval is 0.5 us ticks)
        step_us = max(pre['commutation_interval'] // 2, 80)
        # ~1 commutation window — enough to drop an edge and kick the
        # blind path, short of a deliberate multi-step blackout.
        fault_us = min(step_us + 50, 800)
        _zc_fault(ctl, mode=1, duration_us=fault_us)
        samples = _poll_stats(ctl, seconds=0.35)

        post = _zc_stats(ctl)
        assert post['dropped_edges'] > pre['dropped_edges'], (
            'fault gate never dropped an edge: %r -> %r (fault_us=%d)'
            % (pre, post, fault_us))
        assert any(s['zc_blind_steps'] > 0 or s['zc_stale_q8'] > 0
                   for s in samples), (
            'no blind step observed in %d samples (fault_us=%d pre=%r)'
            % (len(samples), fault_us, pre))
        # clean bridge (no restart) is preferred but not required on this
        # plant — either way the motor must be closed-loop again shortly.
        bridged = (min(s['zero_crosses'] for s in samples) >= pre['zero_crosses']
                   and post['desync_happened'] == pre['desync_happened']
                   and all(s['old_routine'] == 0 for s in samples)
                   and post['running'] == 1)
        if bridged:
            rpm = rpm_from_state(sim, 0.3)
            assert rpm > 500, 'rpm collapsed after clean bridge: %.0f' % rpm
        else:
            _assert_recovered(ctl, sim, rpm0, sitl, tx)
    finally:
        tx.stop()


def test_blind_budget_hands_off_to_stall_rail(sitl_factory, state_stream):
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    sim, tx, rpm0 = _spin_up(sitl, state_stream)
    try:
        ctl = _open_ctl(sitl)
        pre = _zc_stats(ctl)
        assert pre['old_routine'] == 0 and pre['zero_crosses'] > 100, pre
        # The budget is BEMF_STALL_TICKS (45000) of INTERVAL_TIMER time at
        # 2 MHz = 22.5 ms of blind running, independent of rpm. Suppress for
        # 3x that so the handoff is unambiguous on a loaded CI host.
        _zc_fault(ctl, mode=1, duration_us=68_000)
        samples = _poll_stats(ctl, seconds=1.0)

        # budget exhausted: stall-rail restart resets zero_crosses
        assert min(s['zero_crosses'] for s in samples) < 100, (
            'dead-reckoning budget never handed off to the stall rail '
            '(min zc %d over %d samples)\n%s'
            % (min(s['zero_crosses'] for s in samples), len(samples),
               sitl.log_tail()))
        _assert_recovered(ctl, sim, rpm0, sitl, tx)
    finally:
        tx.stop()


def test_alternating_misses_engage_blind_and_recover(sitl_factory, state_stream):
    '''Alternating (mode-2) misses must exercise blind and recover under throttle.

    Background: the leaky miss-rate bucket (+3 miss / -1 accept, trip at 25%)
    was removed so a sustained demag/miss rate would degrade via blind + duty
    fade instead of restarting forever. Mode-2 still drops every other plant
    edge so the blind deadline fires.

    Layered recovery (half-interval early-edge reject + poll re-entry) can
    still walk a pure 50% SITL fault into stall/poll restart on this plant —
    residual timing after a late blind is not a full step, and half-interval
    was re-added for wrong-phase edge rejection, not for guaranteeing
    restart-free 50% miss. This test therefore locks:

      1) blind steps under mode-2 (the miss path is live)
      2) closed-loop recovery once the fault ends (throttle continuous)

    It does NOT require zero_crosses to never fall during the fault window.
    '''
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    sim, tx, rpm0 = _spin_up(sitl, state_stream)
    try:
        ctl = _open_ctl(sitl)
        pre = _zc_stats(ctl)
        assert pre['old_routine'] == 0 and pre['zero_crosses'] > 100, pre

        # drop every other commutation window: real/missed alternation.
        _zc_fault(ctl, mode=2, duration_us=500_000)
        samples = _poll_stats(ctl, seconds=0.7)

        assert any(s['zc_blind_steps'] > 0 for s in samples), (
            'alternating fault never produced a blind step\n' + sitl.log_tail())
        assert any(s['dropped_edges'] > pre['dropped_edges'] for s in samples), (
            'mode-2 fault never dropped an edge\n' + sitl.log_tail())

        # Prefer a clean ride-through when the plant allows it; either way
        # the motor must re-establish closed loop after the fault clears.
        bridged = (min(s['zero_crosses'] for s in samples) >= pre['zero_crosses']
                   and all(s['running'] == 1 and s['old_routine'] == 0
                           for s in samples))
        if bridged:
            assert max(s['zc_stale_q8'] for s in samples) < 200, (
                'staleness ran away under a clean 50%% bridge (max %d/255)'
                % max(s['zc_stale_q8'] for s in samples))
            rpm = rpm_from_state(sim, 0.3)
            assert rpm > 300, 'rpm collapsed after clean bridge: %.0f' % rpm
        _assert_recovered(ctl, sim, rpm0, sitl, tx)
    finally:
        tx.stop()


def test_blind_step_cuts_duty_while_untrusted(sitl_factory, state_stream):
    '''Untrusted commutation must cost energy, not just time.

    This is the invariant BLHeli_32 and Bluejay enforce with their demag
    power cut, and the half of blind stepping ARK was missing: a blind step
    commutates on a GUESS, and driving a guessed rotor position at commanded
    duty is the damage vector on a board where three of four channels have no
    current limit.

    Nothing asserted this before. The fade that replaced the old fixed
    250/500 caps was normalised against the 22.5 ms stall budget rather than
    against a few commutation intervals, so at two blind steps it reduced duty
    by about 1% where the cap it replaced cut 75% - it only bit once the loop
    was already about to be torn down. That regression was invisible to every
    existing test because none of them looked at duty.

    Blackout is well inside the dead-reckoning budget, so this exercises the
    fade and not the restart.
    '''
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    sim, tx, rpm0 = _spin_up(sitl, state_stream)
    try:
        ctl = _open_ctl(sitl)
        pre = _zc_stats(ctl)
        assert pre['old_routine'] == 0 and pre['zero_crosses'] > 100, pre
        duty_before = pre['duty_cycle']
        assert duty_before > 200, (
            'need real drive before the fault to see a cut: duty=%d' % duty_before)

        # ~10 ms: many commutation intervals of blind stepping, but under the
        # 22.5 ms budget so the stall rail cannot be what moves duty.
        _zc_fault(ctl, mode=1, duration_us=10_000)
        samples = _poll_stats(ctl, seconds=0.05)

        assert any(s['zc_blind_steps'] > 0 for s in samples), (
            'blackout produced no blind step, so this proved nothing\n'
            + sitl.log_tail())

        low = min(s['duty_cycle'] for s in samples)
        assert low < duty_before // 2, (
            'duty was not cut while commutating blind: %d -> %d (need < %d)\n'
            % (duty_before, low, duty_before // 2) + sitl.log_tail())
        _assert_recovered(ctl, sim, rpm0, sitl, tx)
    finally:
        tx.stop()
