'''Acquisition-regime desync visibility (PR #61, revised).

Every established-run rail (the jump check's counter, the dead-reckoning
budget, the stall trip) is gated on zero_crosses / zc_at_desync > 100.
A restart -> short spool -> desync cycle that never clears that gate
therefore leaves every counter at zero and can free-run at low eRPM (SITL
racer_5inch at DShot ~700: ~20 desyncs/s, zc never past ~48).

WHAT CHANGED. This used to escalate: batches of early desyncs charged an
ARK-only episode bucket which grew a restart holdoff and eventually latched
drive off. That escalator is gone (see the note at the top of faults.c) -
it was cutting power on turning rotors, which is the reported field failure
this branch exists to fix. Early desyncs now only COUNT, into acq_resist,
which an FC can read and refuse takeoff on.

So the contract is weaker than it was, deliberately, and matches upstream
AM32: a loop that cannot acquire keeps trying instead of being killed. What
must still hold is that the condition is never SILENT - either the loop
acquires (zero_crosses past the established-run gate), or the genuine
stuck-rotor rail kills drive (rotor not turning at all), or acq_resist
surfaces the resisted start to the flight controller.
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
                'bemf_timeout')
STATS_FMT = '<HBBIIIIBBBBBBBBHBBHiIHHHHHHHHBBBBBBB'
MODELS = os.path.join(SITL_DIR, 'models')


def _open_ctl(sitl):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(('127.0.0.1', sitl.state_port))
    s.settimeout(0.5)
    return s


def _zc_stats(ctl, retries=5):
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
    raise AssertionError('no ZC_STATS reply from SITL state port')


def test_racer_hard_spool_cannot_desync_forever_below_acquisition(
        sitl_factory, state_stream):
    '''racer_5inch + DShot 700: never a SILENT free-run below zc 100.'''
    path = os.path.join(MODELS, 'racer_5inch.json')
    assert os.path.isfile(path), path
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    sim = state_stream(sitl)
    assert wait_for_state(sim), sitl.log_tail()
    sim.load_model(path)
    # async model load over UDP
    deadline = time.time() + 3.0
    while time.time() < deadline:
        status = (sim.model_status or '').lower()
        if status and 'fail' not in status and 'error' not in status:
            if 'loading' not in status or 'racer' in status or 'ok' in status:
                break
        time.sleep(0.1)

    tx = Sender('127.0.0.1', sitl.input_port, sd.TYPE_DSHOT600)
    ctl = _open_ctl(sitl)
    try:
        time.sleep(2.2)
        tx.value = 700

        # Sample for long enough that a stuck-below-acq loop (PR description:
        # ~20 desyncs/s, 20 early events per JUMP charge) would either get
        # ramp-backoff help past acquisition, or pile enough charges to latch.
        max_zc = 0
        max_desync = 0
        max_acq = 0
        saw_running = False
        end = None
        t0 = time.time()
        while time.time() - t0 < 12.0:
            try:
                s = _zc_stats(ctl)
            except AssertionError:
                time.sleep(0.05)
                continue
            max_zc = max(max_zc, s['zero_crosses'])
            max_desync = max(max_desync, s['desync_happened'])
            max_acq = max(max_acq, s['acq_resist'])
            if s['running']:
                saw_running = True
            end = s
            # Early success: established-run gate cleared after back-off
            if s['zero_crosses'] > 100:
                break
            time.sleep(0.05)

        assert saw_running, 'motor never entered running\n' + sitl.log_tail()
        assert end is not None

        acquired = max_zc > 100
        # Genuine stuck-rotor rail: drive killed after many desyncs.
        latched_off = (max_desync >= 20 and end['running'] == 0)
        # Or the resisted start is at least visible to the FC.
        surfaced = max_acq > 0

        assert acquired or latched_off or surfaced, (
            'a sustained below-acquisition desync cycle was SILENT: it '
            'neither acquired, nor tripped the stuck-rotor rail, nor '
            'counted a resisted start into acq_resist - so an FC has no '
            'way to know this motor never got going. '
            'max_zc=%d max_desync=%d acq_resist=%d end=%r\n%s'
            % (max_zc, max_desync, max_acq, end, sitl.log_tail()))
    finally:
        tx.value = 0
        time.sleep(0.5)
        tx.stop()
        ctl.close()
