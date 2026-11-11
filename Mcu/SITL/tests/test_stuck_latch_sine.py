'''A stuck-rotor latch holds until pilot zero, with sine startup on too.

The stuck-rotor latch lives in bemf_timeout_happened (ESC_STUCK_LATCH).
faultUpdateBemfTimeoutPolicy() also clears that byte to forgive the
timeout count, and one of those clears ran on any input below 160 with sine
startup on. A pilot who held a low throttle after a jam (or cut the stick
back without reaching zero) got the latch wiped and the motor restarted in
sine on its own. A motor that a fault has stopped must never restart by
itself: the latch clears only on pilot zero (or a power cycle).

With sine startup on, the stop path also re-entered sine idle on every
input frame while latched: an illegal FAULT_STUCK -> SINE_START edge that
left esc_state (and every escIsFault() check) reading "not faulted" for
about a millisecond in four until the next reconcile.

The rotor is locked in the plant while running six-step, which trips the
latch. The rotor is then freed and the throttle dropped into the sine range,
above the sine dead band and below 160. The ESC must stay latched with the
motor off and no illegal edges; zero throttle then clears it and the next
throttle starts again.
'''

from __future__ import annotations

import time

import sitl_dshot as sd
from sitl_gui_backend import EepromClient
from sitl_harness import Sender, wait_for_state
from test_state_protocol import Watch, control_socket

SINE_STARTUP_OFFSET = 19             # schema/eeprom.json sineStartup
STUCK_ROTOR_PROTECTION_OFFSET = 22   # stuckRotorProtection
SINE_MODE_RANGE_OFFSET = 40          # sineModeRange, % (15 -> sine below input 300)
ESC_FAULT_STUCK = 7                  # Inc/esc_state.h
ESC_STUCK_LATCH = 102               # Inc/esc_state.h
VARS = [(1, 'esc_state'), (1, 'armed'), (1, 'running'), (1, 'stepper_sine'),
        (1, 'bemf_timeout_happened'), (2, 'esc_illegal_edge_count')]
STATE, ARMED, RUNNING, STEPPER, BEMF_TIMEOUT, ILLEGAL = range(len(VARS))

RUN_THROTTLE = 600    # six-step with a 15 % sine range
SINE_THROTTLE = 100   # in the sine range: above the dead band (30), below 160


def _hold_latched(watch, seconds, what, sitl):
    illegal = watch.values.get(ILLEGAL)
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        watch.poll()
        v = watch.values
        assert v.get(BEMF_TIMEOUT) == ESC_STUCK_LATCH, (
            '%s: stuck latch wiped (bemf_timeout_happened=%r)' % (what, v.get(BEMF_TIMEOUT)))
        assert v.get(ILLEGAL) == illegal, (
            '%s: illegal state edges while latched (%r -> %r)' % (what, illegal, v.get(ILLEGAL)))
        assert v.get(STATE) == ESC_FAULT_STUCK, (
            '%s: stuck latch cleared (esc_state=%r bemf_timeout_happened=%r)\n%s'
            % (what, v.get(STATE), v.get(BEMF_TIMEOUT), sitl.log_tail()))
        assert v.get(RUNNING) == 0 and v.get(STEPPER) == 0, (
            '%s: restarted on its own (running=%r stepper_sine=%r)'
            % (what, v.get(RUNNING), v.get(STEPPER)))


def test_stuck_latch_holds_through_low_sine_throttle(sitl_factory, state_stream):
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    sim = state_stream(sitl)
    assert wait_for_state(sim), sitl.log_tail()
    ee = EepromClient(port=sitl.state_port)
    for offset, value in ((SINE_STARTUP_OFFSET, 1), (STUCK_ROTOR_PROTECTION_OFFSET, 1),
                          (SINE_MODE_RANGE_OFFSET, 15)):
        ok, message = ee.set(offset, [value])
        assert ok, message

    tx = Sender('127.0.0.1', sitl.input_port, sd.TYPE_DSHOT600)
    try:
        with control_socket(sitl) as sock:
            watch = Watch(sock, VARS)
            watch.until(lambda: watch.values.get(ARMED) == 1, timeout=10)
            tx.value = RUN_THROTTLE
            watch.until(lambda: watch.values.get(RUNNING) == 1
                        and watch.values.get(STEPPER) == 0, timeout=10)
            time.sleep(0.5)

            sim.set_stuck(1.0)
            watch.until(lambda: watch.values.get(STATE) == ESC_FAULT_STUCK, timeout=10)
            sim.set_stuck(0.0)
            _hold_latched(watch, 1.0, 'run throttle after the jam', sitl)

            # Below 160 with sine startup used to wipe the latch at once.
            tx.value = SINE_THROTTLE
            _hold_latched(watch, 2.0, 'low sine throttle after the jam', sitl)

            # Pilot zero clears it; the next throttle starts normally.
            tx.value = 0
            watch.until(lambda: watch.values.get(STATE) != ESC_FAULT_STUCK, timeout=2)
            tx.value = SINE_THROTTLE
            watch.until(lambda: watch.values.get(STEPPER) == 1
                        or watch.values.get(RUNNING) == 1, timeout=5)
    finally:
        tx.stop()
