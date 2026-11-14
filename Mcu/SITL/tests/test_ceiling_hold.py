'''Post-desync throttle-ceiling hold engagement (PR #62).

duty_cycle_maximum is a pure function of the live k_erpm, so a desync
collapses the throttle ceiling in one main-loop pass while the rotor is
still turning. The hold freezes the pre-collapse ceiling for DCM_HOLD_MS so
re-acquisition is not starved of authority.

WHY THIS TEST EXISTS: the first revision of that change declared the hold
state, ticked it at 1 kHz, cleared it on the coast path and read it in the
consumer - but never assigned it. dcm_hold_ms was permanently 0 and the
feature did nothing. That no-op passed compilation (the statics are read, so
no unused-variable diagnostic), cppcheck, the size gate, the full SITL suite
AND a complete HWCI hardware suite, because a no-op cannot regress anything.

So this asserts ENGAGEMENT, not absence of regression: after an injected
desync, dcm_hold_ms must be observed nonzero. Remove the two assignments in
runtimeProcessDesyncCheck and this test must fail - that mutation check is
the whole point, and is worth re-running if the arming is ever refactored.
'''

from __future__ import annotations

import os
import socket
import struct
import time

import sitl_dshot as sd
from sitl_harness import SITL_DIR, Sender, rpm_from_state, wait_for_state

from sitl_state_protocol import (
    STATE_MAGIC_CMD, STATE_CMD_ZC_STATS, STATE_CMD_ZC_FAULT,
)
ZC_STATS_MAGIC = 0x5356
MODELS = os.path.join(SITL_DIR, 'models')

# v2 payload: v1 fields plus the hold state. Appended-only, so a v1 reader
# unpacking the shorter prefix is unaffected.
STATS_FIELDS = ('zero_crosses', 'commutation_interval', 'dropped_edges',
                'desync_happened', 'old_routine', 'running', 'armed',
                'zc_blind_steps', 'zc_stale_q8', 'zc_deadline_armed',
                'dcm_hold_ms', '_pad2', 'dcm_hold_value')
STATS_FMT = '<HBBIIIIBBBBBBBBH'


def _open_ctl(sitl):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(('127.0.0.1', sitl.state_port))
    s.settimeout(0.5)
    return s


def _zc_stats(ctl, retries=5):
    for _ in range(retries):
        ctl.send(struct.pack('<HBB', STATE_MAGIC_CMD, STATE_CMD_ZC_STATS, 0))
        try:
            pkt = ctl.recv(64)
        except socket.timeout:
            continue
        if len(pkt) >= struct.calcsize(STATS_FMT):
            vals = struct.unpack_from(STATS_FMT, pkt)
            if vals[0] == ZC_STATS_MAGIC:
                assert vals[1] >= 2, 'ZC_STATS version %d lacks hold fields' % vals[1]
                return dict(zip(STATS_FIELDS, vals[3:]))
    raise AssertionError('no v2 ZC_STATS reply from SITL state port')


def _zc_fault(ctl, mode, duration_us):
    ctl.send(struct.pack('<HBBI', STATE_MAGIC_CMD, STATE_CMD_ZC_FAULT, mode, duration_us))


def _established_with_hold_clear(sitl, sim, ctl, attempts):
    # The hold needs k_erpm > low_rpm_level, not merely a turning rotor.
    # Keep the existing 2000-rpm floor and require a stable 100-ms window:
    # a single snapshot just past acquisition can still precede the next
    # firmware pass that latches fault_run_established for desync reporting.
    deadline = time.monotonic() + 12.0
    stable_since = None
    rpm = 0.0
    while time.monotonic() < deadline:
        s = _zc_stats(ctl)
        rpm = rpm_from_state(sim, 0.2)
        if (s['running'] and not s['old_routine'] and s['zero_crosses'] > 100
                and s['dcm_hold_ms'] == 0 and rpm > 2000.0):
            now = time.monotonic()
            if stable_since is None:
                stable_since = now
            elif now - stable_since >= 0.1:
                return s
        else:
            stable_since = None
        time.sleep(0.02)
    raise AssertionError(
        'never reached stable established closed loop above 2000 rpm with '
        'hold clear (last rpm=%.0f, last=%r; completed attempts '
        '(before/after counter, hold)=%r)\n%s'
        % (rpm, s, attempts, sitl.log_tail()))


def test_ceiling_hold_engages_on_desync(sitl_factory, state_stream):
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    sim = state_stream(sitl)
    assert wait_for_state(sim), sitl.log_tail()
    # Unloaded: reaches real rpm quickly under CI load. default_7inch can
    # sit near ~450 mech RPM at mid throttle on a loaded runner and trip a
    # brittle >500 gate after closed loop is already healthy (CI flake).
    sim.load_model(os.path.join(MODELS, 'unloaded.json'))
    time.sleep(1.0)

    tx = Sender('127.0.0.1', sitl.input_port, sd.TYPE_DSHOT600)
    ctl = _open_ctl(sitl)
    try:
        time.sleep(2.2)
        tx.value = 900

        seen_hold = 0
        seen_value = 0
        saw_desync = False
        attempts = []
        for _ in range(10):
            # A blackout can take the blind-step/stall path instead of the
            # jump check. Reacquire before trying again: injecting into the
            # startup that follows a stall can arm the hold on an acquisition
            # jump without incrementing the established-run desync counter.
            _zc_fault(ctl, mode=0, duration_us=0)
            pre = _established_with_hold_clear(sitl, sim, ctl, attempts)
            ci_us = max(pre['commutation_interval'] // 2, 100)
            fault_us = max(80 * ci_us, 80_000)
            attempt_hold = 0
            attempt_value = 0
            attempt_desync = False
            _zc_fault(ctl, mode=1, duration_us=fault_us)
            probe_end = time.monotonic() + 0.5
            while time.monotonic() < probe_end:
                s = _zc_stats(ctl)
                attempt_hold = max(attempt_hold, s['dcm_hold_ms'])
                attempt_value = max(attempt_value, s['dcm_hold_value'])
                if s['desync_happened'] > pre['desync_happened']:
                    attempt_desync = True
            attempts.append((pre['desync_happened'], s['desync_happened'],
                             attempt_hold))
            if attempt_desync:
                saw_desync = True
                seen_hold = max(seen_hold, attempt_hold)
                seen_value = max(seen_value, attempt_value)
            # Both observations must come from the same established attempt.
            if attempt_desync and attempt_hold:
                break

        assert saw_desync, (
            'fault injection never produced a desync, so the hold was never '
            'given a chance to arm (before/after counter, hold: %r)\n%s'
            % (attempts, sitl.log_tail()))
        assert seen_hold > 0, (
            'THE HOLD NEVER ARMED. dcm_hold_ms stayed 0 across desyncs - '
            'this is the exact no-op the first revision of PR #62 shipped. '
            'Check that runtimeProcessDesyncCheck still assigns dcm_hold_ms '
            'and dcm_hold_value in the desynced branch.\n%s'
            % sitl.log_tail())
        assert seen_value > 0, (
            'dcm_hold_ms armed but dcm_hold_value is 0, so the clamp holds '
            'nothing: %d' % seen_value)
    finally:
        tx.value = 0
        time.sleep(0.3)
        _zc_fault(ctl, mode=0, duration_us=0)
        tx.stop()
        ctl.close()
