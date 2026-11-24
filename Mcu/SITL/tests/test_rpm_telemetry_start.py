'''First start after boot: e_rpm must not report the unmeasured commutation interval.

commutation_intervals[] is zero from boot until the first commutation interval is
timed, so e_com_time is 2 us. 600000 / 2 truncated into e_rpm (uint16_t) read 37856
(3.8M erpm): esc.Status showed it as a 16512 rpm spike (int18 wrap, 14 poles) on the
first start after every ESC boot, bidirectional DShot reported a 2 us period, and the
same e_rpm opened the low-rpm throttle ceiling for the first commutations.
'''

from __future__ import annotations

import time

import sitl_dshot as sd
from sitl_harness import Sender, wait_for_state
from test_state_protocol import Watch, control_socket

# e_rpm is erpm / 100; 2000 is 200,000 erpm, far above any motor right after a start
E_RPM_PLAUSIBLE_MAX = 2000
START_THROTTLE = 300  # DShot, a gentle start on the default model
WATCH_SECONDS = 1.5


def test_first_start_after_boot_reports_no_rpm_spike(sitl_factory, state_stream):
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    sim = state_stream(sitl)
    assert wait_for_state(sim), sitl.log_tail()
    tx = Sender('127.0.0.1', sitl.input_port, sd.TYPE_DSHOT600)
    try:
        with control_socket(sitl) as sock:
            watch = Watch(sock, [(2, 'e_rpm'), (1, 'running')], min_period_ns=0)
            watch.until(lambda: watch.resolved is not None and 0 in watch.values)
            assert watch.resolved == [1, 1], 'e_rpm or running not found in the SITL symbol table'

            # arm on zero-throttle frames; keep polling so the watch lease (2 s) stays live
            # across the start, whose spike lasted well under 1 ms of simulated time
            arm_end = time.monotonic() + 2.2
            while time.monotonic() < arm_end:
                watch.poll()
            assert watch.values.get(1, 0) == 0, 'motor ran before the test started it'
            tx.value = START_THROTTLE

            deadline = time.monotonic() + WATCH_SECONDS
            while time.monotonic() < deadline:
                watch.poll()
    finally:
        tx.stop()

    running_seen = any(index == 1 and value == 1 for _, index, value in watch.events)
    e_rpm_values = [value for _, index, value in watch.events if index == 0]
    assert running_seen, 'motor never started\n' + sitl.log_tail()
    assert any(v > 0 for v in e_rpm_values), 'no e_rpm reported while running\n' + sitl.log_tail()
    assert max(e_rpm_values) <= E_RPM_PLAUSIBLE_MAX, (
        'e_rpm spiked to %d (erpm/100) on the first start after boot: %r'
        % (max(e_rpm_values), e_rpm_values[:20]))
