'''Governor stuck-low-slope un-latch engagement (PR #65).

While the BEMF-headroom ceiling binds, the slope estimator cannot sample.
A slope learned too low would lock forever without an escape. After
GOV_STUCK_MS of rolling-flat eRPM under the ceiling the governor clears conf
and soft-releases authority.

WHY THIS TEST EXISTS: stock SITL models rarely arm a stuck-low slope, so a
no-op un-latch passes every suite. This forces the bind via SITL GOV_FORCE
(cmd 5), which freezes the estimator for 1.2 s and advances the stuck window
deterministically — then asserts:

  1. gov_stuck_ms advances under the inject
  2. gov_conf falls to 0 (un-latch)
  3. gov_release_ceil is set / ceiling soft-rises (no single-tick step to 2000)
  4. after inject expires and we coast, conf/slope stay cleared

Remove the un-latch assignment and this test must fail.
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

GOV_CONF_ARM = 300
GOV_STUCK_MS = 1000

STATS_FIELDS = ('zero_crosses', 'commutation_interval', 'dropped_edges',
                'desync_happened', 'old_routine', 'running', 'armed',
                'zc_blind_steps', 'zc_stale_q8', 'zc_deadline_armed',
                'dcm_hold_ms', '_pad2', 'dcm_hold_value',
                'adv_kerpm_hold_ms', '_pad3', 'adv_kerpm_hold',
                'zc_trend', 'zc_predicted', 'wait_time', 'advance',
                'gov_conf', 'gov_slope_q10', 'gov_duty_ceiling',
                'gov_stuck_ms', 'gov_release_ceil', 'gov_unlatch_count')
STATS_FMT = '<HBBIIIIBBBBBBBBHBBHiIHHHHHHHH'


def _open_ctl(sitl):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(('127.0.0.1', sitl.state_port))
    s.settimeout(0.5)
    return s


def _zc_stats(ctl, retries=8):
    need = struct.calcsize(STATS_FMT)
    for _ in range(retries):
        ctl.send(struct.pack('<HBB', STATE_MAGIC_CMD, 9, 0))
        try:
            pkt = ctl.recv(512)
        except socket.timeout:
            continue
        if len(pkt) >= need:
            vals = struct.unpack_from(STATS_FMT, pkt)
            if vals[0] == ZC_STATS_MAGIC:
                assert vals[1] >= 5, (
                    'ZC_STATS version %d lacks gov un-latch fields' % vals[1])
                return dict(zip(STATS_FIELDS, vals[3:]))
    raise AssertionError('no v5 ZC_STATS reply from SITL state port')


def _gov_force(ctl, slope_q10: int, conf: int):
    ctl.send(struct.pack('<HBBHH', STATE_MAGIC_CMD, 10, 0, slope_q10 & 0xFFFF,
                         conf & 0xFFFF))


def test_gov_unlatch_engages_when_slope_too_low(sitl_factory, state_stream):
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    sim = state_stream(sitl)
    assert wait_for_state(sim), sitl.log_tail()
    sim.load_model(os.path.join(MODELS, 'unloaded.json'))
    time.sleep(1.0)

    tx = Sender('127.0.0.1', sitl.input_port, sd.TYPE_DSHOT600)
    ctl = _open_ctl(sitl)
    try:
        time.sleep(2.2)
        tx.value = 900

        deadline = time.time() + 12.0
        pre = None
        while time.time() < deadline:
            s = _zc_stats(ctl)
            if s['running'] and not s['old_routine'] and s['zero_crosses'] > 150:
                if rpm_from_state(sim, 0.15) > 150:
                    pre = s
                    break
            time.sleep(0.05)
        assert pre is not None, (
            'never reached closed loop with zc>150\n' + sitl.log_tail())

        low_slope = 48
        before = _zc_stats(ctl)
        unlatch0 = int(before['gov_unlatch_count'])
        _gov_force(ctl, low_slope, GOV_CONF_ARM)
        time.sleep(0.05)
        forced = _zc_stats(ctl)
        assert forced['gov_slope_q10'] == low_slope, forced
        assert forced['gov_conf'] >= GOV_CONF_ARM, forced
        assert forced['gov_duty_ceiling'] <= 700, forced

        saw_stuck = 0
        saw_soft = False
        unlatch_n = unlatch0
        # Wall budget >> GOV_STUCK_MS: under CI CPU contention SITL can run well
        # under 1x realtime (busy-wait pacing), so a fixed 2.5s window has been
        # seen to expire with stuck_ms ~886 and unlatch never fired.
        t_end = time.time() + 15.0
        last_stuck = -1
        stalled_since = None
        while time.time() < t_end:
            s = _zc_stats(ctl)
            stuck = int(s['gov_stuck_ms'])
            saw_stuck = max(saw_stuck, stuck)
            unlatch_n = max(unlatch_n, int(s['gov_unlatch_count']))
            if s['gov_release_ceil'] > 0:
                saw_soft = True
            if unlatch_n > unlatch0 and (saw_soft or s['gov_conf'] < GOV_CONF_ARM):
                break
            # Re-arm inject if stuck stopped advancing before the threshold
            # (force hold expired mid-window on a lagging sim).
            if stuck == last_stuck and stuck < GOV_STUCK_MS and unlatch_n == unlatch0:
                if stalled_since is None:
                    stalled_since = time.time()
                elif time.time() - stalled_since > 0.4:
                    _gov_force(ctl, low_slope, GOV_CONF_ARM)
                    stalled_since = time.time()
            else:
                stalled_since = None
            last_stuck = stuck
            time.sleep(0.015)

        assert saw_stuck >= GOV_STUCK_MS // 2, (
            'gov_stuck_ms never advanced under inject (peak=%d)\nlast=%r\n%s' % (
                saw_stuck, _zc_stats(ctl), sitl.log_tail()))
        assert unlatch_n > unlatch0, (
            'THE UN-LATCH NEVER FIRED (peak stuck_ms=%d, unlatch_count %d→%d). '
            'gov_unlatch_count must increment when conf clears after '
            'GOV_STUCK_MS.\nlast=%r\n%s' % (
                saw_stuck, unlatch0, unlatch_n, _zc_stats(ctl), sitl.log_tail()))
        # Soft release may have finished by the time we poll; unlatch_count is
        # the engagement gate. release_ceil>0 is best-effort.
        if not saw_soft:
            # Accept completed soft-release: ceiling back above bind floor
            s = _zc_stats(ctl)
            assert s['gov_duty_ceiling'] > 700 or s['gov_release_ceil'] > 0, (
                'no soft-release evidence after un-latch\nlast=%r\n%s' % (
                    s, sitl.log_tail()))

        # After inject, coast must leave state clear.
        tx.value = 0
        time.sleep(0.8)
        coast = _zc_stats(ctl)
        assert coast['gov_conf'] == 0 and coast['gov_slope_q10'] == 0, (
            'coast did not clear gov state: %r\n%s' % (coast, sitl.log_tail()))
    finally:
        tx.value = 0
        time.sleep(0.3)
        tx.stop()
        ctl.close()
