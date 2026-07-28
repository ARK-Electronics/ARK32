'''Post-desync advance-schedule rpm hold engagement (PR #63).

The auto-advance curve is keyed on k_erpm, so a desync collapses the rpm
estimate in one main-loop pass and drops the schedule to its bottom while the
rotor is still spinning fast - under-advancing exactly where the true lag is
highest. The hold keeps the last valid k_erpm for the schedule for
ADV_ERPM_HOLD_MS.

WHY THIS TEST EXISTS: the first revision of that change declared the hold
state, ticked it at 1 kHz, cleared it on the coast path and read it in the
auto-advance block - but never assigned it. adv_kerpm_hold_ms was permanently
0 and the feature did nothing. That no-op passed compilation (the statics are
read, so no unused-variable diagnostic), cppcheck, the size gate, the full
SITL suite AND a complete HWCI hardware suite, because a no-op cannot
regress anything.

So this asserts ENGAGEMENT, not absence of regression: after an injected
desync, adv_kerpm_hold_ms must be observed nonzero. Remove the two
assignments in runtimeProcessDesyncCheck and this test must fail - that
mutation check is the whole point.
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

# v3 payload: v1 + dcm hold (v2 / PR #62) + advance hold (v3 / PR #63).
# Appended-only; shorter unpackers keep working via length-check + unpack_from.
STATS_FIELDS = ('zero_crosses', 'commutation_interval', 'dropped_edges',
                'desync_happened', 'old_routine', 'running', 'armed',
                'zc_blind_steps', 'zc_miss_bucket', 'zc_deadline_armed',
                'dcm_hold_ms', '_pad2', 'dcm_hold_value',
                'adv_kerpm_hold_ms', '_pad3', 'adv_kerpm_hold')
STATS_FMT = '<HBBIIIIBBBBBBBBHBBH'


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
                assert vals[1] >= 3, (
                    'ZC_STATS version %d lacks advance-hold fields' % vals[1])
                return dict(zip(STATS_FIELDS, vals[3:]))
    raise AssertionError('no v3 ZC_STATS reply from SITL state port')


def _zc_fault(ctl, mode, duration_us):
    ctl.send(struct.pack('<HBBI', STATE_MAGIC_CMD, 3, mode, duration_us))


def test_advance_hold_engages_on_desync(sitl_factory, state_stream):
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    sim = state_stream(sitl)
    assert wait_for_state(sim), sitl.log_tail()
    # Unloaded: reaches real rpm quickly under CI load (default_7inch flaked
    # the ceiling-hold test at ~462 rpm against a hard >500 gate).
    sim.load_model(os.path.join(MODELS, 'unloaded.json'))
    time.sleep(1.0)

    tx = Sender('127.0.0.1', sitl.input_port, sd.TYPE_DSHOT600)
    ctl = _open_ctl(sitl)
    try:
        time.sleep(2.2)
        tx.value = 900

        # The hold only arms with a real rpm behind it (k_erpm > 0), so wait
        # for established closed loop before injecting anything.
        deadline = time.time() + 10.0
        pre = None
        rpm = 0.0
        while time.time() < deadline:
            s = _zc_stats(ctl)
            if s['running'] and not s['old_routine'] and s['zero_crosses'] > 100:
                rpm = rpm_from_state(sim, 0.2)
                if rpm > 300:
                    pre = s
                    break
            time.sleep(0.05)
        assert pre is not None, (
            'never reached established closed loop with rotor turning '
            '(last rpm=%.0f)\n%s' % (rpm, sitl.log_tail()))
        # Spool-up on this model legitimately produces a few desyncs, so the
        # hold may ALREADY be armed here - observed locally as
        # adv_kerpm_hold_ms=1 with desync_happened=6 at this point. Asserting
        # it is zero outright is therefore flaky. Wait for it to expire (it
        # decays at 1 kHz from ADV_ERPM_HOLD_MS) so that arming seen after the
        # injection below is caused by the injection rather than being a
        # decaying residual - which also keeps the causal link the mutation
        # check relies on.
        settle = time.time() + 2.0
        while time.time() < settle:
            pre = _zc_stats(ctl)
            if pre['adv_kerpm_hold_ms'] == 0:
                break
            time.sleep(0.01)
        assert pre['adv_kerpm_hold_ms'] == 0, (
            'hold never expired before injection, so a later nonzero reading '
            'could be residual rather than caused: %r\n%s' % (pre, sitl.log_tail()))

        ci_us = max(pre['commutation_interval'] // 2, 100)
        seen_hold = 0
        seen_value = 0
        saw_desync = False
        for _ in range(6):
            _zc_fault(ctl, mode=1, duration_us=60 * ci_us)
            probe_end = time.time() + 0.35
            while time.time() < probe_end:
                s = _zc_stats(ctl)
                seen_hold = max(seen_hold, s['adv_kerpm_hold_ms'])
                seen_value = max(seen_value, s['adv_kerpm_hold'])
                if s['desync_happened'] > pre['desync_happened']:
                    saw_desync = True
            if seen_hold:
                break

        assert saw_desync, (
            'fault injection never produced a desync, so the hold was never '
            'given a chance to arm\n' + sitl.log_tail())
        assert seen_hold > 0, (
            'THE HOLD NEVER ARMED. adv_kerpm_hold_ms stayed 0 across the '
            'injected desyncs - this is the exact no-op the first revision of '
            'PR #63 shipped. Check that runtimeProcessDesyncCheck still '
            'assigns adv_kerpm_hold and adv_kerpm_hold_ms in the desynced '
            'branch.\n' + sitl.log_tail())
        assert seen_value > 0, (
            'adv_kerpm_hold_ms armed but adv_kerpm_hold is 0, so the schedule '
            'holds nothing')
    finally:
        tx.value = 0
        time.sleep(0.3)
        _zc_fault(ctl, mode=0, duration_us=0)
        tx.stop()
        ctl.close()
