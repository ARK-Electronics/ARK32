"""Stop-path invariants: zero throttle stays stopped, braking stays braking.

The ESC is armed and stopped throughout. Each case writes one setting and
then checks what zero throttle does with it.
"""

import time

import pytest
import sitl_dshot as sd
from sitl_gui_backend import EepromClient
from sitl_harness import Sender
from sitl_params import PARAMS_BY_NAME
from test_state_protocol import Watch, control_socket

VARIABLES = [(1, 'armed'), (1, 'sitl_tone_active'), (2, 'input'), (1, 'running'), (1, 'prop_brake_active')]
ARMED, TONE, INPUT, RUNNING, BRAKE = range(len(VARIABLES))


def offset(name):
    return PARAMS_BY_NAME[name][0]


@pytest.fixture
def stopped(sitl_factory):
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    tx = Sender('127.0.0.1', sitl.input_port, sd.TYPE_DSHOT600)
    port = sd.InputPort('127.0.0.1', sitl.input_port)
    with control_socket(sitl) as sock:
        watch = Watch(sock, VARIABLES)
        try:
            watch.until(lambda: watch.values.get(ARMED) == 1 and watch.values.get(TONE) == 0)
            # Arming tones have silent gaps: wait out a gap before taking over
            # the throttle stream, as test_dshot_programming does.
            quiet_since = time.monotonic()
            deadline = quiet_since + 5
            while time.monotonic() - quiet_since < 0.15:
                assert time.monotonic() < deadline, sitl.log_tail()
                watch.poll()
                if watch.values.get(TONE):
                    quiet_since = time.monotonic()
            tx.stop()

            def send(value):
                port.send_dshot(value, ptype=sd.TYPE_DSHOT600)
                time.sleep(0.004)

            def hold_zero(seconds):
                """Zero-throttle frames for `seconds`; returns the watch events seen."""
                first = len(watch.events)
                end = time.monotonic() + seconds
                while time.monotonic() < end:
                    send(0)
                    watch.poll()
                return watch.events[first:]

            yield send, hold_zero, EepromClient(port=sitl.state_port), sitl, watch
        finally:
            tx.stop()
            port.close()


def test_live_zero_sine_range_keeps_zero_throttle_stopped(stopped):
    send, hold_zero, ee, sitl, watch = stopped
    ok, message = ee.set(offset('USE_SINE_START'), [1])
    assert ok, message
    hold_zero(0.3)
    # DShot programming writes RAM without the load-time clamp (5..25). A
    # range of 0 used to map zero throttle to 160 and start the motor.
    for _ in range(6):
        send(36)
    for word in (offset('SINE_MODE_CHANGEOVER'), 0, 37):
        send(word)
    image, _ = ee.fetch()
    assert image is not None and image[offset('SINE_MODE_CHANGEOVER')] == 0, sitl.log_tail()
    events = hold_zero(1.0)
    driven = [e for e in events if e[1] in (INPUT, RUNNING) and e[2] != 0]
    assert not driven, (driven[:5], sitl.log_tail())
    assert watch.values.get(INPUT) == 0 and watch.values.get(RUNNING) == 0, sitl.log_tail()


def test_full_drag_brake_marks_the_brake_active(stopped):
    send, hold_zero, ee, sitl, watch = stopped
    # Strength 10 takes the fullBrake() branch. Without the flag the gate
    # driver sleep poll (ARK 4IN1 DRV8328 nSLEEP) treats the ESC as idle and
    # sleeps the driver under the brake.
    for name, value in (('DRAG_BRAKE_STRENGTH', 10), ('BRAKE_ON_STOP', 1)):
        ok, message = ee.set(offset(name), [value])
        assert ok, message
    hold_zero(0.5)
    assert watch.values.get(BRAKE) == 1, (watch.values, sitl.log_tail())
    assert watch.values.get(RUNNING) == 0, sitl.log_tail()


@pytest.mark.parametrize('raw,expected', [(5, 5), (6, 0), (255, 0)])
def test_active_brake_power_is_bounded_on_load(stopped, raw, expected):
    send, hold_zero, ee, sitl, watch = stopped
    # 255 would put the active-brake compare above the PWM period: the
    # phase pair selected by comStep(2) driven continuously at zero throttle.
    ok, message = ee.set(offset('ACTIVE_BRAKE_POWER'), [raw])
    assert ok, message
    image, _ = ee.fetch()
    assert image is not None, sitl.log_tail()
    assert image[offset('ACTIVE_BRAKE_POWER')] == expected, sitl.log_tail()
