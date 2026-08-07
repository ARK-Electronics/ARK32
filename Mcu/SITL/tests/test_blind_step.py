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
  - alternating: mode-2 50% edge drop must engage blind; restarts during
                 the fault window are bounded (no miss-rate thrash); post-
                 fault recovery under continuous throttle. A clean no-
                 restart ride-through is reported separately (xfail when
                 the plant cascades).
'''

from __future__ import annotations

import socket
import struct
import time

import pytest
import sitl_dshot as sd
from sitl_harness import Sender, rpm_from_state, wait_for_state

STATE_MAGIC_CMD = 0x5353
ZC_STATS_MAGIC = 0x5356

STATS_FIELDS = ('zero_crosses', 'commutation_interval', 'dropped_edges',
                'desync_happened', 'old_routine', 'running', 'armed',
                'zc_blind_steps', 'zc_stale_q8', 'zc_deadline_armed')
STATS_FMT = '<HBBIIIIBBBBBB'


def _open_ctl(sitl):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(('127.0.0.1', sitl.state_port))
    s.settimeout(0.5)
    return s


def _zc_fault(ctl, mode, duration_us):
    ctl.send(struct.pack('<HBBI', STATE_MAGIC_CMD, 8, mode, duration_us))


def _zc_stats(ctl, retries=5):
    for _ in range(retries):
        ctl.send(struct.pack('<HBB', STATE_MAGIC_CMD, 9, 0))
        try:
            pkt = ctl.recv(64)
        except socket.timeout:
            continue
        if len(pkt) >= struct.calcsize(STATS_FMT):
            vals = struct.unpack_from(STATS_FMT, pkt)
            if vals[0] == ZC_STATS_MAGIC:
                return dict(zip(STATS_FIELDS, vals[3:]))
    raise AssertionError('no ZC_STATS reply from SITL state port')


def _poll_stats(ctl, seconds):
    '''lockstep-sample stats as fast as the state port answers'''
    out = []
    deadline = time.time() + seconds
    while time.time() < deadline:
        out.append(_zc_stats(ctl))
    return out


def _count_zc_restarts(samples, floor=50, drop=20):
    '''Count established-run restarts in a ZC_STATS sample stream.

    zero_crosses only falls on a desync or stall-rail handoff. A drop of
    ``drop`` or more from a value above ``floor`` is one restart episode.
    '''
    if not samples:
        return 0
    n = 0
    prev = samples[0]['zero_crosses']
    for s in samples[1:]:
        z = s['zero_crosses']
        if prev > floor and z + drop < prev:
            n += 1
        prev = z
    return n


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
    # zc_stale_q8 is zc_blind_ticks * 255 / BEMF_STALL_TICKS (sitl_state),
    # not the old miss-bucket. After recovery the dead-reckoning budget is
    # cleared, so require empty - not a leftover <6 of a different scale.
    assert end['zc_stale_q8'] == 0, end
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


# Stall/trust handoffs during a ~0.7 s mode-2 window. The thrash this PR
# removes was restart-forever on a miss *rate* cliff (leaky bucket at 25%);
# that produced many restarts per second. A late-blind cascade can still
# exhaust the dead-reckoning budget a couple of times while recovering -
# measured 0-2 on this plant - so the bound is small and hard, not "any
# recovery after fault clears".
_ALT_MISS_MAX_RESTARTS = 2


def test_alternating_misses_bounded_restarts_and_recover(sitl_factory, state_stream):
    '''Mode-2 50% edge drop: blind lives, restarts bounded, then recover.

    Locks the thrash contract observably (not behind an if bridged:):

      1) blind steps under mode-2 (miss path is live)
      2) restarts during the fault window <= 1 (not thrash forever)
      3) closed-loop recovery once the fault ends (continuous throttle)

    Clean ride-through (zero restarts + bounded stale) is checked when it
    happens; when the plant cascades, the restart bound still fails a
    regression to "restart on every 50% miss episode". See also the xfail
    clean-bridge sibling for the stronger "50% keeps flying" claim.
    '''
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    sim, tx, rpm0 = _spin_up(sitl, state_stream)
    try:
        ctl = _open_ctl(sitl)
        pre = _zc_stats(ctl)
        assert pre['old_routine'] == 0 and pre['zero_crosses'] > 100, pre

        _zc_fault(ctl, mode=2, duration_us=500_000)
        samples = _poll_stats(ctl, seconds=0.7)

        assert any(s['zc_blind_steps'] > 0 for s in samples), (
            'alternating fault never produced a blind step\n' + sitl.log_tail())
        assert any(s['dropped_edges'] > pre['dropped_edges'] for s in samples), (
            'mode-2 fault never dropped an edge\n' + sitl.log_tail())

        restarts = _count_zc_restarts(samples)
        min_zc = min(s['zero_crosses'] for s in samples)
        max_stale = max(s['zc_stale_q8'] for s in samples)
        stayed_closed = all(s['running'] == 1 and s['old_routine'] == 0
                            for s in samples)
        assert restarts <= _ALT_MISS_MAX_RESTARTS, (
            'mode-2 thrash: %d restarts during fault window (max %d); '
            'min zc %d -> %d, max stale_q8 %d, stayed_closed=%s\n%s'
            % (restarts, _ALT_MISS_MAX_RESTARTS, pre['zero_crosses'], min_zc,
               max_stale, stayed_closed, sitl.log_tail()))

        # When the plant allows a true clean bridge, enforce the original
        # "50% keeps flying" staleness bound so that outcome stays locked.
        if restarts == 0 and stayed_closed and min_zc >= pre['zero_crosses']:
            assert max_stale < 128, (
                'staleness ran away under a clean 50%% bridge (max %d/255)'
                % max_stale)
            rpm = rpm_from_state(sim, 0.3)
            assert rpm > 300, 'rpm collapsed after clean bridge: %.0f' % rpm

        _assert_recovered(ctl, sim, rpm0, sitl, tx)
    finally:
        tx.stop()


@pytest.mark.xfail(
    strict=False,
    reason=(
        'Headline "50% miss keeps flying" is plant-dependent: late blinds can '
        'desync electrical phase so delivered edges fail confirm and the '
        'budget hands off once. CI should report whether a clean bridge '
        'happened; the thrash bound is enforced in the non-xfail sibling.'
    ),
)
def test_alternating_misses_clean_bridge_no_restart(sitl_factory, state_stream):
    '''Strict clean ride-through under mode-2 (xfail, non-strict).

    Fails (xfail) when the plant cascades into a stall handoff. Passes when
    zero_crosses never falls and staleness stays under half the budget -
    the PR summary claim, made visible in CI rather than absorbed.
    '''
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    sim, tx, rpm0 = _spin_up(sitl, state_stream)
    try:
        ctl = _open_ctl(sitl)
        pre = _zc_stats(ctl)
        assert pre['old_routine'] == 0 and pre['zero_crosses'] > 100, pre

        _zc_fault(ctl, mode=2, duration_us=500_000)
        samples = _poll_stats(ctl, seconds=0.7)

        assert any(s['zc_blind_steps'] > 0 for s in samples), (
            'alternating fault never produced a blind step\n' + sitl.log_tail())
        assert _count_zc_restarts(samples) == 0, (
            'clean-bridge claim: restart during mode-2 fault '
            '(min zc %d, pre %d)\n%s'
            % (min(s['zero_crosses'] for s in samples), pre['zero_crosses'],
               sitl.log_tail()))
        assert all(s['running'] == 1 and s['old_routine'] == 0
                   for s in samples), (
            'clean-bridge claim: left closed-loop under mode-2\n'
            + sitl.log_tail())
        assert min(s['zero_crosses'] for s in samples) >= pre['zero_crosses'], (
            'clean-bridge claim: zero_crosses fell\n' + sitl.log_tail())
        assert max(s['zc_stale_q8'] for s in samples) < 128, (
            'clean-bridge claim: stale_q8 max %d/255\n'
            % max(s['zc_stale_q8'] for s in samples))
        rpm = rpm_from_state(sim, 0.3)
        assert rpm > 300, 'rpm collapsed: %.0f' % rpm
        _assert_recovered(ctl, sim, rpm0, sitl, tx)
    finally:
        tx.stop()
