"""Temporary direction requests must finish while stopped in sine mode."""

import time

import pytest
import sitl_dshot as sd
from sitl_gui_backend import EepromClient
from sitl_harness import Sender
from test_state_protocol import Watch, control_socket
from test_stop_path_safety import offset


VARIABLES = [(1, 'armed'), (1, 'sitl_tone_active'), (2, 'input'),
             (1, 'running'), (1, 'stepper_sine'), (1, 'forward'),
             (1, 'last_dshot_command'), (1, 'dshot_direction_pending'),
             (2, 'duty_cycle'), (2, 'duty_cycle_setpoint'),
             (1, 'do_once_sinemode'), (2, 'phase_A_position'), (1, 'old_routine'),
             (4, 'zero_crosses')]
(ARMED, TONE, INPUT, RUNNING, SINE, FORWARD, COMMAND, PENDING,
 DUTY, SETPOINT, SINE_IDLE, PHASE, POLL, ZC) = range(len(VARIABLES))


@pytest.mark.parametrize('bidirectional', [0, 1])
@pytest.mark.parametrize('stop_from', ['idle', 'six_step', 'sine_band', 'startup_sine_band'])
def test_reverse_from_sine_stop(sitl_factory, state_stream, bidirectional, stop_from):
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    tx = Sender('127.0.0.1', sitl.input_port, sd.TYPE_DSHOT600)
    sim = state_stream(sitl)
    ee = EepromClient(port=sitl.state_port)
    try:
        with control_socket(sitl) as sock:
            watch = Watch(sock, VARIABLES)
            watch.until(lambda: len(watch.values) == len(VARIABLES)
                        and watch.values[ARMED] == 1 and watch.values[TONE] == 0)
            # Arming tones include silent gaps. Wait for continuous quiet
            # before writing settings or replacing the stop stream.
            quiet_since = time.monotonic()
            deadline = quiet_since + 5
            while time.monotonic() - quiet_since < 0.15:
                assert time.monotonic() < deadline, sitl.log_tail()
                watch.poll()
                if watch.values[TONE]:
                    quiet_since = time.monotonic()

            for name, value in [('COMP_PWM', 1), ('USE_SINE_START', 1),
                                ('BI_DIRECTIONAL', bidirectional),
                                ('SINE_MODE_CHANGEOVER', 15),
                                ('BRAKE_ON_STOP', 1), ('DRAG_BRAKE_STRENGTH', 10)]:
                ok, message = ee.set(offset(name), [value])
                assert ok, message
            watch.until(lambda: watch.values[SINE] == 1 and watch.values[INPUT] == 0
                        and watch.values[SINE_IDLE] == 1)
            before, _ = ee.fetch()
            assert before is not None, sitl.log_tail()
            throttle = 1400 if bidirectional else 700

            if stop_from != 'idle':
                tx.value = throttle
                if stop_from == 'startup_sine_band':
                    # Catch acquisition before its duty floor disengages.
                    # This reproduces the stale-duty hold without injecting
                    # firmware state or depending on a random desync.
                    watch.until(lambda: watch.values[RUNNING] == 1 and watch.values[SINE] == 0
                                and watch.values[ZC] < 30)
                else:
                    watch.until(lambda: watch.values[RUNNING] == 1 and watch.values[SINE] == 0
                                and watch.values[POLL] == 0 and sim.samples
                                and sim.samples[-1][1] > 200)
                if stop_from.endswith('sine_band'):
                    # About 100 adjusted input in either throttle mode.
                    # During acquisition this leaves the six-step startup
                    # duty floor cached; the stepper sets PWM directly.
                    tx.value = 1075 if bidirectional else 100
                    watch.until(lambda: watch.values[SINE] == 1
                                and 48 < watch.values[INPUT] < 127
                                and watch.values[SINE_IDLE] == 0)
                tx.value = 0
                watch.until(lambda: watch.values[INPUT] == 0 and watch.values[SINE] == 1
                            and watch.values[SINE_IDLE] == 1 and watch.values[RUNNING] == 0)
                if stop_from != 'startup_sine_band':
                    assert sim.samples[-1][1] > 100, ('missed the coast window', watch.values)
                if stop_from == 'startup_sine_band':
                    assert watch.values[DUTY] > 0, 'did not exercise stale six-step duty'

            phase_at_stop = watch.values[PHASE]
            first = len(watch.events)
            tx.cmds = [21] * 10
            watch.until(lambda: watch.values[COMMAND] == 21, timeout=1)
            if stop_from != 'idle':
                # Renew throttle immediately: a pending request may hold it
                # only until the sine stop branch finishes, not indefinitely.
                tx.value = throttle
            watch.until(lambda: watch.values[PENDING] == 0 and watch.values[FORWARD] == 0,
                        timeout=0.5)
            events = watch.events[first:]
            turned_at = next(t for t, i, v in events if i == FORWARD and v == 0)
            assert not [(t, v) for t, i, v in events
                        if i == PHASE and t < turned_at and v != phase_at_stop], events
            if stop_from == 'idle':
                assert watch.values[INPUT] == 0 and watch.values[RUNNING] == 0, watch.values
                tx.value = throttle
            # Bound actual restart, and check the plant's signed velocity
            # rather than treating a changed direction flag as motor motion.
            watch.until(lambda: watch.values[RUNNING] == 1 and watch.values[SINE] == 0
                        and watch.values[INPUT] > 127 and sim.samples[-1][1] < -200,
                        timeout=5)
            after, _ = ee.fetch()
            assert after == before, 'temporary direction changed EEPROM'
    finally:
        tx.stop()
