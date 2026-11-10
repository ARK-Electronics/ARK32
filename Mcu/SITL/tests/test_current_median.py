'''Median-of-3 in front of the current boxcar.

The 4IN1 samples the shunt once per 1 kHz tick. A single L·di/dt spike used
to enter the 50-sample average at 1/50. getSmoothedCurrent() now takes the
median of the last three raw samples first, so one bad sample never reaches
actual_current (limiter, KISS, EDT, DroneCAN status, mAh).

State command 0x83 adds a steady offset (0.01 A units) and can replace the
next ADC sample with a raw count. A full-scale spike with the median in
place leaves actual_current alone; the same spike through the boxcar alone
moves it by about (4095-baseline)/50 counts, ~3 A here. A later step of the
offset must still show up once the 50 ms boxcar fills.

SITL scale (AM32_SITL_CAN, MILLIVOLT_PER_AMP 20, CURRENT_OFFSET 0):
  raw = round(amps * 20 * 4095 / 3300)
  actual_current = (raw * 3300 / 41) / 20
  8.00 A -> raw 199 -> 800
  18.00 A -> raw 447 -> 1798
'''

from __future__ import annotations

import struct
import time

import sitl_dshot as sd
from sitl_harness import Sender
from sitl_state_protocol import STATE_CMD_CURRENT_INJECT, STATE_MAGIC_CMD
from test_state_protocol import Watch, control_socket

# Steady levels, in actual_current units (0.01 A). See the module comment.
BASE_CA = 800
STEP_CA = 1798
STEP_DELTA = STEP_CA - BASE_CA


def _inject(sock, offset_ca, spike_raw=0, arm=False):
    sock.send(struct.pack('<HBBhH', STATE_MAGIC_CMD, STATE_CMD_CURRENT_INJECT,
                          1 if arm else 0, offset_ca, spike_raw))


def _hold(watch, index, lo, hi, hold_s, timeout):
    '''Return the value once it has stayed inside [lo, hi] for hold_s.'''
    deadline = time.monotonic() + timeout
    stable_since = None
    last = None
    while time.monotonic() < deadline:
        watch.poll()
        value = watch.values.get(index)
        last = value
        if value is not None and lo <= value <= hi:
            if stable_since is None:
                stable_since = time.monotonic()
            elif time.monotonic() - stable_since >= hold_s:
                return value
        else:
            stable_since = None
    raise AssertionError(
        'actual_current did not hold %d..%d (last=%r values=%r)\n'
        % (lo, hi, last, watch.values))


def test_median_rejects_spike_and_tracks_step(sitl_factory, state_stream):
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    sim = state_stream(sitl, period_us=1000)
    tx = Sender('127.0.0.1', sitl.input_port, sd.TYPE_DSHOT600)
    try:
        with control_socket(sitl) as sock:
            watch = Watch(sock, [(2, 'actual_current'), (2, 'ADC_raw_current'),
                                 (1, 'sitl_tone_active'), (1, 'armed')],
                          min_period_ns=0)
            # Startup tones PWM the bridge, so bus current is not the
            # injected offset until the arm tune has finished.
            watch.until(lambda: watch.resolved == [1, 1, 1, 1]
                        and watch.values.get(3) == 1 and watch.values.get(2) == 0,
                        timeout=25)

            def wait_sim(seconds, timeout=5.0):
                '''Advance at least `seconds` of simulated time. latest()[0]
                is seconds (the state stream scales t_ns). Wall sleep would
                bunch spikes onto consecutive 1 kHz samples when the host
                is busy and the sim falls behind.'''
                deadline = time.monotonic() + timeout
                start = None
                while time.monotonic() < deadline:
                    watch.poll()
                    latest = sim.latest()
                    if latest is None:
                        continue
                    now = latest[0]
                    if start is None or now < start:
                        # A reboot zeros the sim clock. Start the wait over.
                        start = now
                        continue
                    if now - start >= seconds:
                        return
                raise AssertionError('sim clock did not advance %.3f s (start=%r last=%r)'
                                     % (seconds, start, sim.latest()))

            _inject(sock, BASE_CA, arm=False)
            # 50 ms boxcar plus the one-sample median delay.
            wait_sim(0.08)
            baseline = _hold(watch, 0, BASE_CA - 40, BASE_CA + 40, 0.08, 5.0)

            mark = len(watch.events)
            for _ in range(6):
                _inject(sock, BASE_CA, spike_raw=4095, arm=True)
                # 20 sim ms is 20 ADC samples: the spike has left the
                # 3-wide median before the next one is armed.
                wait_sim(0.02)

            fresh = watch.events[mark:]
            raw_hits = [ev for ev in fresh if ev[1] == 1 and ev[2] == 4095]
            assert len(raw_hits) >= 3, (
                'full-scale spikes never reached ADC_raw_current (%d hits)\n%s'
                % (len(raw_hits), sitl.log_tail()))
            actual_hits = [ev[2] for ev in fresh if ev[1] == 0]
            assert all(abs(v - baseline) <= 25 for v in actual_hits), (
                'spike leaked into actual_current: baseline=%d samples=%s'
                % (baseline, actual_hits[-12:]))
            assert abs(watch.values[0] - baseline) <= 25, watch.values

            _inject(sock, STEP_CA, arm=False)
            wait_sim(0.08)
            stepped = _hold(watch, 0, baseline + STEP_DELTA - 50,
                            baseline + STEP_DELTA + 50, 0.08, 5.0)
            assert abs((stepped - baseline) - STEP_DELTA) <= 50, (
                'step not tracked: baseline=%d stepped=%d' % (baseline, stepped))
    finally:
        tx.stop()
