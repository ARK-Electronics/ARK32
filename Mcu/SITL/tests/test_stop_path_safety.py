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


@pytest.mark.parametrize('bidirectional', [0, 1])
@pytest.mark.parametrize('command', [20, 21])
def test_direction_command_during_zero_throttle_coast(sitl_factory, state_stream,
                                                      bidirectional, command):
    """Latch 20/21 during coast; an early throttle pulse must wait for stop."""
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    tx = Sender('127.0.0.1', sitl.input_port, sd.TYPE_DSHOT600)
    sim = state_stream(sitl)
    ee = EepromClient(port=sitl.state_port)
    variables = [(1, 'armed'), (1, 'sitl_tone_active'), (2, 'input'),
                 (1, 'running'), (1, 'forward'), (1, 'last_dshot_command'),
                 (2, 'duty_cycle'), (1, 'old_routine')]
    try:
        with control_socket(sitl) as sock:
            watch = Watch(sock, variables)
            watch.until(lambda: watch.values.get(0) == 1 and watch.values.get(1) == 0)
            time.sleep(0.2)  # wait out the arming tune's silent gaps
            for name, value in [('BI_DIRECTIONAL', bidirectional),
                                ('BRAKE_ON_STOP', 1), ('DRAG_BRAKE_STRENGTH', 10)]:
                ok, message = ee.set(offset(name), [value])
                assert ok, message
            # Start reversed for command 20, so both cases must physically
            # reverse from their initial run. The
            # temporary DShot direction must not rewrite the stored setting.
            if command == 20:
                tx.cmds = [21] * 20
                watch.until(lambda: watch.values.get(5) == 21)
            initial_forward = 0 if command == 20 else 1
            throttle = 1400 if bidirectional else 700
            tx.value = throttle
            watch.until(lambda: watch.values.get(3) == 1 and watch.values.get(7) == 0
                        and watch.values.get(4) == initial_forward
                        and sim.samples and abs(sim.samples[-1][1]) > 200)
            before, _ = ee.fetch()
            assert before is not None, sitl.log_tail()

            tx.value = 0
            watch.until(lambda: watch.values.get(2) == 0)
            assert watch.values.get(3) == 1, ('missed the coast window', watch.values)
            assert abs(sim.samples[-1][1]) > 100, 'rotor already stopped'
            first = len(watch.events)
            tx.cmds = [command] * 12
            watch.until(lambda: watch.values.get(5) == command, timeout=1)
            assert watch.values.get(3) == 1, ('command was not latched during coast', watch.values)
            assert watch.values.get(4) == initial_forward, 'reversed commutation during coast'

            # Ask to start before the stop completes; pending direction must
            # keep the stop path at zero until it can safely take effect.
            tx.value = throttle
            watch.until(lambda: watch.values.get(4) == 1 - initial_forward
                        and watch.values.get(3) == 1 and watch.values.get(2, 0) > 47)
            events = watch.events[first:]
            stop_time = next(t for t, i, v in events if i == 3 and v == 0)
            turn_time = next(t for t, i, v in events if i == 4 and v == 1 - initial_forward)
            assert stop_time <= turn_time, events
            # Check actual plant rotation, not just the commutation flag.
            want_sign = 1 if command == 20 else -1
            watch.until(lambda: sim.samples and sim.samples[-1][1] * want_sign > 200)
            after, _ = ee.fetch()
            assert after == before, 'temporary direction changed EEPROM'
    finally:
        tx.stop()


def test_active_brake_stop_ends_the_run(sitl_factory, state_stream):
    """brake_on_stop 2 must end six-step at zero throttle, not run under it.

    tenKhzRoutine applies the active brake (comStep(2)) on every tick at zero
    throttle whether or not the run has ended. Left running, six-step kept
    commutating underneath it - blind steps between brake ticks, and false
    comparator crossings from the brake waveform resetting the dead-reckoning
    budget - so the stall rail never ended the stop. ARK 4IN1 bench: running=1
    for 1.6-7.6 s after zero throttle, ~6000 steps/s, an audible grind. SITL
    shows the same mechanism as ~1600 accepted crossings and ~0.4-0.7 s
    (brake_on_stop 0 ends through the stall rail in ~0.25 s).
    """
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    tx = Sender('127.0.0.1', sitl.input_port, sd.TYPE_DSHOT600)
    sim = state_stream(sitl)
    ee = EepromClient(port=sitl.state_port)
    variables = [(1, 'armed'), (1, 'sitl_tone_active'), (2, 'input'), (1, 'running'), (4, 'zero_crosses')]
    try:
        with control_socket(sitl) as sock:
            watch = Watch(sock, variables)
            watch.until(lambda: watch.values.get(0) == 1 and watch.values.get(1) == 0)
            time.sleep(0.2)  # wait out the arming tune's silent gaps
            ok, message = ee.set(offset('BRAKE_ON_STOP'), [2])
            assert ok, message
            tx.value = 700
            watch.until(lambda: watch.values.get(3) == 1 and sim.samples and abs(sim.samples[-1][1]) > 200)
            end = time.monotonic() + 1.0  # well past zc 100: an established run
            while time.monotonic() < end:
                watch.poll()
            assert watch.values.get(3) == 1, ('run did not hold', watch.values, sitl.log_tail())
            tx.value = 0
            watch.until(lambda: watch.values.get(2) == 0)
            t0 = time.monotonic()
            zc0 = watch.values.get(4)
            peak = zc0
            while watch.values.get(3) != 0:
                assert time.monotonic() - t0 < 3.0, ('stop never ended the run', watch.values)
                watch.poll()
                peak = max(peak, watch.values.get(4) or 0)
            took = time.monotonic() - t0
            assert took < 0.15, ('braked stop held running=1 for %.2f s' % took, sitl.log_tail())
            assert peak - zc0 < 50, '%d zero-crosses accepted after zero throttle' % (peak - zc0)
    finally:
        tx.stop()


def test_held_reversal_is_not_an_active_brake_stop(sitl_factory, state_stream):
    """A reversal the speed guard holds back must keep that guard.

    In bidirectional DShot (rc_car_reversing off) a reverse request while the
    rotor is fast becomes zero throttle until commutation_interval says it is
    slow. Ending the run there would reset commutation_interval to "stopped"
    and let the next reverse frame flip direction into a fast rotor, so the
    active-brake stop must leave this path alone.
    """
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    tx = Sender('127.0.0.1', sitl.input_port, sd.TYPE_DSHOT600)
    sim = state_stream(sitl)
    ee = EepromClient(port=sitl.state_port)
    variables = [(1, 'armed'), (1, 'sitl_tone_active'), (2, 'input'), (1, 'running'), (1, 'forward')]
    try:
        with control_socket(sitl) as sock:
            watch = Watch(sock, variables)
            watch.until(lambda: watch.values.get(0) == 1 and watch.values.get(1) == 0)
            time.sleep(0.2)  # wait out the arming tune's silent gaps
            for name, value in (('BI_DIRECTIONAL', 1), ('BRAKE_ON_STOP', 2)):
                ok, message = ee.set(offset(name), [value])
                assert ok, message
            tx.value = 1400  # forward
            watch.until(lambda: watch.values.get(3) == 1 and sim.samples and abs(sim.samples[-1][1]) > 200)
            end = time.monotonic() + 1.0
            while time.monotonic() < end:
                watch.poll()
            forward = watch.values.get(4)
            assert watch.values.get(3) == 1, ('run did not hold', watch.values, sitl.log_tail())
            tx.value = 600  # reverse, held back by the speed guard
            first = len(watch.events)
            end = time.monotonic() + 0.1
            while time.monotonic() < end:
                watch.poll()
            changed = [e for e in watch.events[first:] if e[1] in (3, 4)]
            assert not changed, ('held reversal ended the run or flipped direction', changed, abs(sim.samples[-1][1]))
            assert watch.values.get(4) == forward
    finally:
        tx.stop()
