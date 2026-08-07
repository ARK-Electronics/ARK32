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

So this asserts ENGAGEMENT, not absence of regression: when desync_happened
increments (real jump desync), adv_kerpm_hold_ms must be observed nonzero.
Remove the assignments in runtimeProcessDesyncCheck and this test must fail.

NOTE (layered recovery / blind): mode-1 EXTI blackout is NOT a jump-desync
path. Blind rides short dropouts; longer blackouts hand to the stall rail
without incrementing desync_happened or arming these holds. Engagement is
observed on real jump desyncs during spool-up (common on this plant), not
via mode-1 fault injection.
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
                'zc_blind_steps', 'zc_stale_q8', 'zc_deadline_armed',
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
        ctl.send(struct.pack('<HBB', STATE_MAGIC_CMD, 9, 0))
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

        # Capture hold arming on real jump desyncs (spool roughness). Do not
        # use mode-1 EXTI blackout: that path is blind / stall, not desync.
        deadline = time.time() + 12.0
        last_desync = 0
        seen_hold = 0
        seen_value = 0
        saw_desync = False
        last = None
        while time.time() < deadline:
            s = _zc_stats(ctl)
            last = s
            if s['desync_happened'] > last_desync:
                saw_desync = True
                last_desync = s['desync_happened']
                # Hold is assigned in the same branch as desync_happened++;
                # poll a few ms so ZC_STATS is not a torn pre-assign sample.
                probe_end = time.time() + 0.08
                while time.time() < probe_end:
                    s2 = _zc_stats(ctl)
                    seen_hold = max(seen_hold, s2['adv_kerpm_hold_ms'])
                    seen_value = max(seen_value, s2['adv_kerpm_hold'])
                    if seen_hold and seen_value:
                        break
                    time.sleep(0.002)
                if seen_hold and seen_value:
                    break
            time.sleep(0.01)

        assert saw_desync, (
            'never observed a jump desync during spool, so the hold was '
            'never given a chance to arm (last=%r)\n%s'
            % (last, sitl.log_tail()))
        assert seen_hold > 0, (
            'THE HOLD NEVER ARMED. adv_kerpm_hold_ms stayed 0 across real '
            'jump desyncs - this is the exact no-op the first revision of '
            'PR #63 shipped. Check that runtimeProcessDesyncCheck still '
            'assigns adv_kerpm_hold and adv_kerpm_hold_ms in the desynced '
            'branch.\n' + sitl.log_tail())
        assert seen_value > 0, (
            'adv_kerpm_hold_ms armed but adv_kerpm_hold is 0, so the schedule '
            'holds nothing')
        # Sanity: rotor actually turned while we watched.
        rpm = rpm_from_state(sim, 0.2)
        assert rpm > 100, 'rotor never turned: rpm=%.0f\n%s' % (rpm, sitl.log_tail())
    finally:
        tx.value = 0
        time.sleep(0.3)
        tx.stop()
        ctl.close()
