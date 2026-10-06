'''Blind steps must not poison the spool-up predictor on the crossings after them.

PeriodElapsedCallback keeps zc_trend, an EMA of thiszctime - lastzctime, and
commutates against CI + clamp(zc_trend, +-CI/2). Blind steps skip the update,
but the real crossings after one used to feed it: lastzctime was the blind
step's inflated interval and the first real thiszctime is timed from the
blind commutation, not from a crossing, so the next two differences are
not intervals at all. On this plant a one-step dropout (7 blind steps, then
the loop resyncs) pulled the trend to about -CI/4 at constant speed.

This holds the motor at steady speed, blanks the comparator for about one
commutation window a few times (ZC_FAULT mode 1, as the short-dropout
blind-step test does), and watches zc_trend while the loop stays closed.
The speed does not change, so the true trend is ~0.

Mutation check: feed the post-blind steps back in and the trend reaches
about -0.28 x the pre-fault CI.
'''

from __future__ import annotations

import socket
import struct
import time

import sitl_dshot as sd
from sitl_harness import Sender, rpm_from_state, wait_for_state

from sitl_state_protocol import (
    STATE_MAGIC_CMD, STATE_CMD_ZC_STATS, STATE_CMD_ZC_FAULT,
)
ZC_STATS_MAGIC = 0x5356
ZC_TREND_MIN_ZC = 20

STATS_FIELDS = ('zero_crosses', 'commutation_interval', 'dropped_edges',
                'desync_happened', 'old_routine', 'running', 'armed',
                'zc_blind_steps', 'zc_stale_q8', 'zc_deadline_armed',
                'dcm_hold_ms', '_pad2', 'dcm_hold_value',
                'adv_kerpm_hold_ms', '_pad3', 'adv_kerpm_hold',
                'zc_trend', 'zc_predicted', 'wait_time', 'advance')
STATS_FMT = '<HBBIIIIBBBBBBBBHBBHiIHH'


def _open_ctl(sitl):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(('127.0.0.1', sitl.state_port))
    s.settimeout(0.5)
    return s


def _zc_stats(ctl, retries=5):
    for _ in range(retries):
        ctl.send(struct.pack('<HBB', STATE_MAGIC_CMD, STATE_CMD_ZC_STATS, 0))
        try:
            pkt = ctl.recv(96)
        except socket.timeout:
            continue
        if len(pkt) >= struct.calcsize(STATS_FMT):
            vals = struct.unpack_from(STATS_FMT, pkt)
            if vals[0] == ZC_STATS_MAGIC:
                assert vals[1] >= 4, 'ZC_STATS version %d lacks zc_trend' % vals[1]
                return dict(zip(STATS_FIELDS, vals[3:]))
    raise AssertionError('no v4 ZC_STATS reply from SITL state port')


def _zc_fault(ctl, mode, duration_us):
    ctl.send(struct.pack('<HBBI', STATE_MAGIC_CMD, STATE_CMD_ZC_FAULT, mode, duration_us))


def test_short_dropout_leaves_trend_near_zero(sitl_factory, state_stream):
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    sim = state_stream(sitl)
    assert wait_for_state(sim), sitl.log_tail()
    tx = Sender('127.0.0.1', sitl.input_port, sd.TYPE_DSHOT300)
    ctl = _open_ctl(sitl)
    worst = None
    blinded = 0
    try:
        time.sleep(2.2)
        tx.value = 500
        time.sleep(3.5)
        assert rpm_from_state(sim) > 1000, sitl.log_tail()
        for _ in range(3):
            pre = _zc_stats(ctl)
            assert pre['old_routine'] == 0 and pre['zero_crosses'] > 100, pre
            ci0 = pre['commutation_interval']
            step_us = max(ci0 // 2, 80)
            _zc_fault(ctl, mode=1, duration_us=min(step_us + 50, 800))
            end = time.time() + 0.08
            while time.time() < end:
                s = _zc_stats(ctl)
                if s['zc_blind_steps']:
                    blinded += 1
                closed = (s['running'] and not s['old_routine']
                          and s['zero_crosses'] >= ZC_TREND_MIN_ZC)
                if closed:
                    ratio = s['zc_trend'] / float(ci0)
                    if worst is None or ratio < worst[0]:
                        worst = (ratio, ci0, s)
            time.sleep(0.5)
    finally:
        tx.stop()

    assert blinded, 'the dropout never produced a blind step\n' + sitl.log_tail()
    assert worst is not None, 'no closed-loop sample after the dropouts\n' + sitl.log_tail()
    ratio, ci0, s = worst
    assert ratio > -0.08, (
        'zc_trend reached %.2f x the pre-fault CI at constant speed after a '
        'short dropout (trend %d, CI %d): post-blind steps are feeding the '
        'predictor\n%r' % (ratio, s['zc_trend'], ci0, s))
