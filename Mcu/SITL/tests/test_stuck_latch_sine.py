'''A stuck-rotor latch holds until a pilot zero, with sine startup too.

The stuck-rotor latch lives in bemf_timeout_happened (ESC_STUCK_LATCH).
faultUpdateBemfTimeoutPolicy() also clears that byte to forgive the
timeout count, and one of those clears ran on any input below 160 with sine
startup on. A pilot who held a low throttle after a jam (or cut the stick
back without reaching zero) got the latch wiped and the motor restarted in
sine on its own. A motor that a fault has stopped must never restart by
itself: the latch clears only on a pilot zero (or a power cycle).

With sine startup on, the stop path also re-entered sine idle on every
input frame while latched: an illegal FAULT_STUCK -> SINE_START edge that
left esc_state (and every escIsFault() check) reading "not faulted" for
about a millisecond in four until the next reconcile.

A pilot zero is a stop the protocol received (DShot MOTOR_STOP here). Zeros
the firmware makes itself, such as the input it forces to zero for DShot
command frames 1-47, must not clear the latch either.

The rotor is locked in the plant while running six-step, which trips the
latch, then freed.
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
ESC_STUCK_LATCH = 102                # Inc/esc_state.h
VARS = [(1, 'esc_state'), (1, 'armed'), (1, 'running'), (1, 'stepper_sine'),
        (1, 'bemf_timeout_happened'), (2, 'esc_illegal_edge_count'), (2, 'input'), (2, 'e_rpm')]
STATE, ARMED, RUNNING, STEPPER, BEMF_TIMEOUT, ILLEGAL, INPUT, E_RPM = range(len(VARS))

RUN_THROTTLE = 600    # six-step with a 15 % sine range
SINE_THROTTLE = 100   # in the sine range: above the dead band (30), below 160
LED0_ON = 22          # a DShot command frame the firmware does not act on


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


def _jam(sitl_factory, state_stream, sine):
    '''Run six-step, lock the rotor until the latch trips, free it.'''
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    sim = state_stream(sitl)
    assert wait_for_state(sim), sitl.log_tail()
    ee = EepromClient(port=sitl.state_port)
    for offset, value in ((SINE_STARTUP_OFFSET, sine), (STUCK_ROTOR_PROTECTION_OFFSET, 1),
                          (SINE_MODE_RANGE_OFFSET, 15)):
        ok, message = ee.set(offset, [value])
        assert ok, message
    tx = Sender('127.0.0.1', sitl.input_port, sd.TYPE_DSHOT600)
    sock = control_socket(sitl)
    watch = Watch(sock, VARS)
    watch.until(lambda: watch.values.get(ARMED) == 1, timeout=10)
    tx.value = RUN_THROTTLE
    watch.until(lambda: watch.values.get(RUNNING) == 1 and watch.values.get(STEPPER) == 0, timeout=10)
    time.sleep(0.5)
    sim.set_stuck(1.0)
    watch.until(lambda: watch.values.get(STATE) == ESC_FAULT_STUCK, timeout=10)
    sim.set_stuck(0.0)
    _hold_latched(watch, 1.0, 'run throttle after the jam', sitl)
    return sitl, tx, sock, watch


def _zero_clears_and_restarts(watch, tx, throttle):
    '''A received stop clears the latch, and the next throttle drives the motor.'''
    tx.value = 0
    watch.until(lambda: watch.values.get(STATE) != ESC_FAULT_STUCK, timeout=2)
    tx.value = throttle
    # Sine idle at zero already has stepper_sine set, so require a mapped
    # throttle and a commutation rate, not just a drive flag.
    watch.until(lambda: (watch.values.get(INPUT) or 0) > 47 and (watch.values.get(E_RPM) or 0) > 0,
                timeout=5)


def test_stuck_latch_holds_through_low_sine_throttle(sitl_factory, state_stream):
    sitl, tx, sock, watch = _jam(sitl_factory, state_stream, sine=1)
    try:
        # Below 160 with sine startup used to wipe the latch at once.
        tx.value = SINE_THROTTLE
        _hold_latched(watch, 2.0, 'low sine throttle after the jam', sitl)
        _zero_clears_and_restarts(watch, tx, SINE_THROTTLE)
    finally:
        tx.stop()
        sock.close()


def test_dshot_command_frames_do_not_clear_a_stuck_latch(sitl_factory, state_stream):
    '''Command frames (1-47) zero the input, but they are not MOTOR_STOP.'''
    sitl, tx, sock, watch = _jam(sitl_factory, state_stream, sine=0)
    try:
        tx.value = LED0_ON
        _hold_latched(watch, 0.5, 'DShot command frames after the jam', sitl)
        tx.value = RUN_THROTTLE
        _hold_latched(watch, 1.5, 'throttle after command frames', sitl)
        _zero_clears_and_restarts(watch, tx, RUN_THROTTLE)
    finally:
        tx.stop()
        sock.close()
