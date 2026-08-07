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

So this asserts ENGAGEMENT, not absence of regression: when a real jump
desync arms the hold (k_erpm > low_rpm_level at the desync), dcm_hold_ms must
be observed nonzero. Remove the assignments in runtimeProcessDesyncCheck and
this test must fail.

NOTE (layered recovery / blind): mode-1 EXTI blackout is NOT a jump-desync
path. Blind rides short dropouts; longer blackouts hand to the stall rail
without incrementing desync_happened or arming these holds. Engagement is
observed on real jump desyncs during spool (and optional throttle steps),
not via mode-1 fault injection.
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


def test_ceiling_hold_engages_on_desync(sitl_factory, state_stream):
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    sim = state_stream(sitl)
    assert wait_for_state(sim), sitl.log_tail()
    # Unloaded: reaches real rpm quickly under CI load.
    sim.load_model(os.path.join(MODELS, 'unloaded.json'))
    time.sleep(1.0)

    tx = Sender('127.0.0.1', sitl.input_port, sd.TYPE_DSHOT600)
    ctl = _open_ctl(sitl)
    try:
        time.sleep(2.2)
        tx.value = 900

        # Capture hold arming on real jump desyncs across the whole spool.
        # Do not use mode-1 EXTI blackout (blind/stall, not desync). Score
        # every desync_happened edge and also track the live hold fields —
        # DCM_HOLD_MS is only ~50 ms, easy to miss between samples.
        deadline = time.time() + 14.0
        last_desync = 0
        seen_hold = 0
        seen_value = 0
        saw_desync = False
        last = None
        step_at = time.time() + 4.0

        while time.time() < deadline:
            s = _zc_stats(ctl)
            last = s
            seen_hold = max(seen_hold, s['dcm_hold_ms'])
            seen_value = max(seen_value, s['dcm_hold_value'] if s['dcm_hold_ms'] else 0)

            if s['desync_happened'] > last_desync:
                saw_desync = True
                last_desync = s['desync_happened']
                probe_end = time.time() + 0.1
                while time.time() < probe_end:
                    s2 = _zc_stats(ctl)
                    seen_hold = max(seen_hold, s2['dcm_hold_ms'])
                    if s2['dcm_hold_ms']:
                        seen_value = max(seen_value, s2['dcm_hold_value'])
                    if seen_hold and seen_value:
                        break
                    time.sleep(0.001)
                if seen_hold and seen_value:
                    break

            # One mid-run throttle step after the motor has had time to spool —
            # can provoke another jump desync without mode-1 blackout.
            if time.time() >= step_at:
                step_at = deadline + 1.0  # once
                tx.value = 1600
                time.sleep(0.08)
                tx.value = 900

            if seen_hold and seen_value:
                break
            time.sleep(0.005)

        assert saw_desync, (
            'never observed a jump desync during spool/step, so the hold was '
            'never given a chance to arm (last=%r)\n%s'
            % (last, sitl.log_tail()))
        assert seen_hold > 0, (
            'THE HOLD NEVER ARMED. dcm_hold_ms stayed 0 across real jump '
            'desyncs - this is the exact no-op the first revision of PR #62 '
            'shipped. Check that runtimeProcessDesyncCheck still assigns '
            'dcm_hold_ms and dcm_hold_value in the desynced branch when '
            'k_erpm > low_rpm_level.\n%s' % sitl.log_tail())
        assert seen_value > 0, (
            'dcm_hold_ms armed but dcm_hold_value is 0, so the clamp holds '
            'nothing: %d' % seen_value)
        rpm = rpm_from_state(sim, 0.2)
        assert rpm > 100, 'rotor never turned: rpm=%.0f\n%s' % (rpm, sitl.log_tail())
    finally:
        tx.value = 0
        time.sleep(0.3)
        tx.stop()
        ctl.close()
