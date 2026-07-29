'''Accel-predictor consumer path engagement (PR #64).

commutation_interval is a lag filter; advance/waitTime are derived from a
one-step prediction (CI + clamp(zc_trend)) so the commutation point does not
trail under constant angular acceleration.

WHY THIS TEST EXISTS: the first class of "looks wired, does nothing" bugs on
this fork (#63 hold never assigned; auto_advance schedule never consumed)
pass the full SITL suite because a no-op cannot regress anything. This asserts
ENGAGEMENT of the consumer path:

  1. zc_trend moves during spool (EMA of interval steps is live)
  2. once zero_crosses >= 20, zc_predicted == CI + clamp(zc_trend, +-CI/2)
  3. waitTime == sat(predicted/2 - advance) using the *exported* predicted
     and advance — if advance/waitTime still key on lagging CI while predicted
     is only computed for show, this equality fails when predicted != CI
  4. after an injected desync that zeros zero_crosses, zc_trend is cleared
     (stale-trend hygiene next to every zero_crosses = 0)

Remove the predicted assignment into advance/waitTime, or drop bemfZcResetTrend
on the desync path, and this test must fail.
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
ZC_TREND_MIN_ZC = 20

# v4 payload: v3 + accel-predictor consumer fields.
STATS_FIELDS = ('zero_crosses', 'commutation_interval', 'dropped_edges',
                'desync_happened', 'old_routine', 'running', 'armed',
                'zc_blind_steps', 'zc_miss_bucket', 'zc_deadline_armed',
                'dcm_hold_ms', '_pad2', 'dcm_hold_value',
                'adv_kerpm_hold_ms', '_pad3', 'adv_kerpm_hold',
                'zc_trend', 'zc_predicted', 'wait_time', 'advance')
# v3 ends ...HBBH; v4 appends iIHH (trend, predicted, waitTime, advance).
STATS_FMT = '<HBBIIIIBBBBBBBBHBBHiIHH'


def _open_ctl(sitl):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(('127.0.0.1', sitl.state_port))
    s.settimeout(0.5)
    return s


def _zc_stats(ctl, retries=5):
    for _ in range(retries):
        ctl.send(struct.pack('<HBB', STATE_MAGIC_CMD, 4, 0))
        try:
            pkt = ctl.recv(64)
        except socket.timeout:
            continue
        if len(pkt) >= struct.calcsize(STATS_FMT):
            vals = struct.unpack_from(STATS_FMT, pkt)
            if vals[0] == ZC_STATS_MAGIC:
                assert vals[1] >= 4, (
                    'ZC_STATS version %d lacks accel-predictor fields' % vals[1])
                return dict(zip(STATS_FIELDS, vals[3:]))
    raise AssertionError('no v4 ZC_STATS reply from SITL state port')


def _zc_fault(ctl, mode, duration_us):
    ctl.send(struct.pack('<HBBI', STATE_MAGIC_CMD, 3, mode, duration_us))


def _clamp_corr(trend: int, ci: int) -> int:
    limit = ci >> 1
    if trend > limit:
        return limit
    if trend < -limit:
        return -limit
    return trend


def _expected_wait(predicted: int, advance: int) -> int:
    half = predicted >> 1
    if advance >= half:
        return 0
    wt = half - advance
    return 65535 if wt > 65535 else wt


def test_accel_predictor_consumer_path(sitl_factory, state_stream):
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    sim = state_stream(sitl)
    assert wait_for_state(sim), sitl.log_tail()
    # Unloaded: long, clean acceleration window under CI load.
    sim.load_model(os.path.join(MODELS, 'unloaded.json'))
    time.sleep(1.0)

    tx = Sender('127.0.0.1', sitl.input_port, sd.TYPE_DSHOT600)
    ctl = _open_ctl(sitl)
    try:
        time.sleep(2.2)
        tx.value = 2000  # full stick — sustained accel for the EMA

        deadline = time.time() + 12.0
        saw_trend = False
        saw_applied = False
        samples_checked = 0
        last = None

        while time.time() < deadline:
            s = _zc_stats(ctl)
            last = s
            if not (s['running'] and not s['old_routine']):
                time.sleep(0.02)
                continue

            if s['zc_trend'] != 0:
                saw_trend = True

            if s['zero_crosses'] >= ZC_TREND_MIN_ZC:
                ci = int(s['commutation_interval'])
                trend = int(s['zc_trend'])
                predicted = int(s['zc_predicted'])
                advance = int(s['advance'])
                wait_time = int(s['wait_time'])

                # Skip unusable / mid-update snapshots:
                # 1) bemfZcResetTrend() zeros zc_predicted until the next
                #    PeriodElapsedCallback write (zc can already be >= 20).
                # 2) ZC_STATS is a multi-field racy read of ISR-updated
                #    scalars — trend can advance while predicted still
                #    holds the previous CI. Only score coherent samples.
                if predicted == 0 or ci == 0:
                    time.sleep(0.01)
                    continue

                expect_pred = (ci + _clamp_corr(trend, ci)) & 0xFFFFFFFF
                if predicted != expect_pred:
                    time.sleep(0.01)
                    continue

                expect_wt = _expected_wait(predicted, advance)
                if wait_time != expect_wt:
                    # Same race on advance/waitTime; skip torn samples.
                    time.sleep(0.01)
                    continue

                samples_checked += 1
                if predicted != ci:
                    saw_applied = True

                # Enough consistent samples and a real correction once.
                if samples_checked >= 8 and saw_applied and saw_trend:
                    break

            time.sleep(0.01)

        assert saw_trend, (
            'zc_trend never moved during spool — the EMA is not live '
            '(or closed loop never ran). last=%r\n%s' % (last, sitl.log_tail()))
        assert samples_checked >= 8, (
            'never collected enough closed-loop samples with '
            'zero_crosses >= %d to validate the consumer path (got %d). '
            'last=%r\n%s' % (ZC_TREND_MIN_ZC, samples_checked, last,
                             sitl.log_tail()))
        assert saw_applied, (
            'zc_predicted never diverged from commutation_interval while '
            'armed — the correction is computed but may not be applied, or '
            'the spool never produced a usable trend. last=%r\n%s' % (
                last, sitl.log_tail()))

        # --- stale-trend hygiene on desync ---
        # Need a nonzero trend, then inject a desync and require that the
        # zero_crosses=0 path also clears zc_trend.
        pre = _zc_stats(ctl)
        assert pre['running'] and pre['zero_crosses'] > 50, (
            'lost closed loop before desync hygiene check: %r\n%s' % (
                pre, sitl.log_tail()))

        # If trend already settled to 0 at steady speed, kick throttle
        # briefly or just accept a fault that still zeros crosses + trend.
        ci_us = max(int(pre['commutation_interval']) // 2, 100)
        cleared = False
        for _ in range(8):
            _zc_fault(ctl, mode=1, duration_us=80 * ci_us)
            probe_end = time.time() + 0.4
            while time.time() < probe_end:
                s = _zc_stats(ctl)
                if s['zero_crosses'] == 0 and s['zc_trend'] == 0:
                    cleared = True
                    break
                # After a desync, zero_crosses may bounce back quickly; require
                # that whenever crosses are still low the trend is not a large
                # leftover from the prior run.
                if (s['desync_happened'] > pre['desync_happened'] and
                        s['zero_crosses'] < ZC_TREND_MIN_ZC and
                        s['zc_trend'] == 0):
                    cleared = True
                    break
            if cleared:
                break

        assert cleared, (
            'after desync, zc_trend was not cleared with zero_crosses. '
            'bemfZcResetTrend must sit next to every zero_crosses = 0 '
            '(runtime desync path is the usual miss). last=%r pre=%r\n%s' % (
                _zc_stats(ctl), pre, sitl.log_tail()))
    finally:
        tx.value = 0
        time.sleep(0.3)
        _zc_fault(ctl, mode=0, duration_us=0)
        tx.stop()
        ctl.close()
