'''Acquisition-regime desync rail (PR #61).

Every established-run rail (JUMP charge, blind/grind, stall) is gated on
zero_crosses / zc_at_desync > 100. Without the acquisition rail, a
restart -> short spool -> desync cycle that never clears that gate
charges nothing and can free-run forever at low eRPM (SITL racer_5inch
at DShot ~700: ~20 desyncs/s, zc never past ~48, bucket stuck at 0).

With the rail, batches of early desyncs charge the episode machinery
(ramp back-off, holdoff, eventual latch). Behavioral regression: on the
racer model under hard spool, either the loop eventually acquires
(zero_crosses past the established-run gate) or drive is killed by the
stuck latch - it must not free-run indefinitely with zc forever below
acquisition.
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
                'zc_blind_steps', 'zc_miss_bucket', 'zc_deadline_armed')
STATS_FMT = '<HBBIIIIBBBBBB'
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
            pkt = ctl.recv(64)
        except socket.timeout:
            continue
        if len(pkt) >= struct.calcsize(STATS_FMT):
            vals = struct.unpack_from(STATS_FMT, pkt)
            if vals[0] == ZC_STATS_MAGIC:
                return dict(zip(STATS_FIELDS, vals[3:]))
    raise AssertionError('no ZC_STATS reply from SITL state port')


def test_racer_hard_spool_cannot_desync_forever_below_acquisition(
        sitl_factory, state_stream):
    '''racer_5inch + DShot 700 must not free-run forever below zc 100.'''
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
        # Acq desyncs no longer increment desync_happened (established-only
        # error_count). Success is acquire past zc 100, or drive stopped by
        # the acq-rail episode latch — not free-running forever below acq.
        latched_off = (end['running'] == 0 and not acquired)

        assert acquired or latched_off, (
            'acquisition rail did not break free-desync-below-zc100: '
            'max_zc=%d end=%r\n%s'
            % (max_zc, end, sitl.log_tail()))
    finally:
        tx.value = 0
        time.sleep(0.5)
        tx.stop()
        ctl.close()
