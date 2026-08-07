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

STATE_MAGIC_CMD = 0x5353
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
        ctl.send(struct.pack('<HBB', STATE_MAGIC_CMD, 9, 0))
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
    ctl.send(struct.pack('<HBBI', STATE_MAGIC_CMD, 8, mode, duration_us))


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

        # Spin up into established closed loop: the hold only arms when the
        # ceiling being frozen was earned at a real rpm (k_erpm >
        # low_rpm_level), so a stationary or barely-turning rotor is not a
        # valid starting condition for this test.
        RPM_MIN = 2000.0
        deadline = time.time() + 12.0
        pre = None
        rpm = 0.0
        while time.time() < deadline:
            s = _zc_stats(ctl)
            if s['running'] and not s['old_routine'] and s['zero_crosses'] > 100:
                # Short sample; poll until the rotor is clearly above idle so
                # a single slow window on a busy CI host cannot flake out.
                #
                # RPM_MIN must clear the hold's own ARM GATE, not just idle.
                # The gate is k_erpm > low_rpm_level, and low_rpm_level =
                # motor_kv * poles / 3200 (settings.c) = 900 * 14 / 3200 = 3
                # for this model, so k_erpm must reach 4. k_erpm is
                # mech_rpm * pole_pairs / 1000, i.e. ~571 mech rpm - well
                # above a 300 gate. Injecting the fault below the arm gate
                # would leave the hold legitimately un-armed and fail the
                # test against the firmware, so keep ~4x headroom.
                rpm = rpm_from_state(sim, 0.2)
                if rpm > RPM_MIN:
                    # Spin-up desyncs (common on loaded CI) arm the hold for
                    # DCM_HOLD_MS (~50 ms). Wait until it expires so the
                    # injected-fault arming is unambiguous.
                    if s['dcm_hold_ms'] == 0:
                        pre = s
                        break
            time.sleep(0.02)
        assert pre is not None, (
            'never reached established closed loop above %.0f rpm with '
            'hold clear (last rpm=%.0f, last=%r)\n%s'
            % (RPM_MIN, rpm, _zc_stats(ctl), sitl.log_tail()))
        assert pre['dcm_hold_ms'] == 0, 'hold already armed before any desync: %r' % pre

        # Suppress comparator edge delivery long enough to force a desync
        # rather than a bridgeable dropout: the interval jump has to exceed
        # the 50% jump check in runtimeProcessDesyncCheck.
        ci_us = max(pre['commutation_interval'] // 2, 100)
        seen_hold = 0
        seen_value = 0
        saw_desync = False
        # Longer blackouts: the inertial comparator plant can bridge a short
        # drop without a jump-desync, so scale with CI and floor at 80 ms.
        fault_us = max(80 * ci_us, 80_000)
        for _ in range(10):
            _zc_fault(ctl, mode=1, duration_us=fault_us)
            probe_end = time.time() + 0.5
            while time.time() < probe_end:
                s = _zc_stats(ctl)
                seen_hold = max(seen_hold, s['dcm_hold_ms'])
                seen_value = max(seen_value, s['dcm_hold_value'])
                if s['desync_happened'] > pre['desync_happened']:
                    saw_desync = True
            if seen_hold:
                break
            # clear any residual hold before the next inject
            time.sleep(0.1)

        assert saw_desync, (
            'fault injection never produced a desync, so the hold was never '
            'given a chance to arm\n' + sitl.log_tail())
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
