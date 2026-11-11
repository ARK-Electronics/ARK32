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


MS = 1_000_000  # Watch event stamps are SITL time in ns


def _armed_watch(sitl, sock, variables):
    watch = Watch(sock, variables)
    watch.until(lambda: watch.values.get(0) == 1 and watch.values.get(1) == 0)
    time.sleep(0.2)  # wait out the arming tune's silent gaps
    return watch


def _settings(ee, **values):
    for name, value in values.items():
        ok, message = ee.set(offset(name), [value])
        assert ok, message


def _poll(watch, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        watch.poll()


def _established(watch, sim, throttle, run, old=None, zc=None):
    """Spin to an established run: running, rotor moving, closed loop past zc 100."""
    watch.until(lambda: watch.values.get(run) == 1 and sim.samples and abs(sim.samples[-1][1]) > 200)
    if old is not None:
        watch.until(lambda: watch.values.get(old) == 0 and (watch.values.get(zc) or 0) > 100, timeout=3)
    _poll(watch, 0.5)
    assert watch.values.get(run) == 1, ('run did not hold', watch.values)


def test_active_brake_stop_ends_the_run(sitl_factory, state_stream):
    """brake_on_stop 2 must end six-step at zero throttle, not run under it.

    tenKhzRoutine applies the active brake (comStep(2)) on every tick at zero
    throttle whether or not the run has ended. Left running, six-step kept
    commutating underneath it - blind steps between brake ticks, and false
    comparator crossings from the brake waveform resetting the dead-reckoning
    budget - so the stall rail never ended the stop. ARK 4IN1 bench: running=1
    for 1.6-7.6 s after zero throttle, ~6000 steps/s, an audible grind. SITL
    on main: 0.46-0.67 s and ~1600 accepted crossings.
    """
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    tx = Sender('127.0.0.1', sitl.input_port, sd.TYPE_DSHOT600)
    sim = state_stream(sitl)
    ee = EepromClient(port=sitl.state_port)
    variables = [(1, 'armed'), (1, 'sitl_tone_active'), (2, 'input'), (1, 'running'), (4, 'zero_crosses'),
                 (1, 'old_routine')]
    _, _, IN, RUN, ZC, OLD = range(len(variables))
    try:
        with control_socket(sitl) as sock:
            watch = _armed_watch(sitl, sock, variables)
            _settings(ee, BRAKE_ON_STOP=2)
            tx.value = 700
            _established(watch, sim, 700, RUN, OLD, ZC)
            first = len(watch.events)
            tx.value = 0
            watch.until(lambda: watch.values.get(IN) == 0)
            _poll(watch, 0.5)
            events = watch.events[first:]
            t_zero = next(ts for ts, i, v in events if i == IN and v == 0)
            t_stop = next((ts for ts, i, v in events if i == RUN and v == 0), None)
            assert t_stop is not None, ('braked stop never ended the run', watch.values, sitl.log_tail())
            assert t_stop - t_zero < 5 * MS, 'running=1 %.1f ms into the braked stop' % ((t_stop - t_zero) / MS)
            after = [(i, v) for ts, i, v in events if ts > t_stop and ((i == RUN and v) or (i == ZC and v > 5))]
            assert not after, ('six-step ran on under the brake after the stop', after)
    finally:
        tx.stop()


@pytest.mark.parametrize('approach', ['held_reversal', 'neutral_frame'])
def test_bidirectional_active_brake_keeps_mains_path(sitl_factory, state_stream, approach):
    """Bidirectional mode is outside the active-brake stop.

    Its speed guard decides "safe to reverse" from commutation_interval, which
    the stop resets to 5000 ("stopped") at any rotor speed, so a held reverse
    request, or forward -> neutral -> reverse, would flip direction into a fast
    rotor: in SITL 13.9 ms after the request at ~88% speed, every time. Assert
    that neither zero-throttle frame takes that stop. Deterministic on sim
    time: the stop acts on the frame itself, while main's own path cannot end
    the run or reset the interval before the stall rail's BEMF_STALL_TICKS
    (22.5 ms) without commutation.
    """
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    tx = Sender('127.0.0.1', sitl.input_port, sd.TYPE_DSHOT600)
    sim = state_stream(sitl)
    ee = EepromClient(port=sitl.state_port)
    variables = [(1, 'armed'), (1, 'sitl_tone_active'), (2, 'input'), (1, 'running'), (1, 'forward'),
                 (4, 'commutation_interval')]
    _, _, IN, RUN, FWD, CI = range(len(variables))
    try:
        with control_socket(sitl) as sock:
            watch = _armed_watch(sitl, sock, variables)
            _settings(ee, BI_DIRECTIONAL=1, BRAKE_ON_STOP=2)
            tx.value = 1400  # forward
            _established(watch, sim, 1400, RUN)
            first = len(watch.events)
            tx.value = 600 if approach == 'held_reversal' else 0  # reverse while fast / neutral
            watch.until(lambda: watch.values.get(IN) == 0, timeout=2)
            if approach == 'neutral_frame':
                tx.value = 600
            _poll(watch, 0.1)
            events = watch.events[first:]
            t_zero = next(ts for ts, i, v in events if i == IN and v == 0)
            early = [(round((ts - t_zero) / MS, 2), variables[i][1], v) for ts, i, v in events
                     if ts - t_zero < 10 * MS and ((i == RUN and v == 0) or (i == CI and v == 5000) or i == FWD)]
            assert not early, ('zero-throttle frame took the active-brake stop (ms, var, value)', early)
    finally:
        tx.stop()


@pytest.mark.parametrize('command', [20, 21])
def test_active_brake_stop_holds_a_direction_change(sitl_factory, state_stream, command):
    """A DShot 20/21 that arrives after an active-brake stop waits 100 ms.

    The stop sets running=0 at once, and dshotApplyPendingDirection otherwise
    only waits for that and for duty to reach 0 - a few ms, with the rotor
    still fast (ArduPilot turtle sends DSHOT_NORMAL on disarm right after a
    spin). The hold matches the 60-100 ms a brake 0/1 stop takes on the bench.
    """
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    tx = Sender('127.0.0.1', sitl.input_port, sd.TYPE_DSHOT600)
    sim = state_stream(sitl)
    ee = EepromClient(port=sitl.state_port)
    variables = [(1, 'armed'), (1, 'sitl_tone_active'), (2, 'input'), (1, 'running'), (1, 'forward'),
                 (1, 'last_dshot_command')]
    _, _, IN, RUN, FWD, CMD = range(len(variables))
    try:
        with control_socket(sitl) as sock:
            watch = _armed_watch(sitl, sock, variables)
            _settings(ee, BRAKE_ON_STOP=2)
            if command == 20:  # start reversed so that 20 is a change
                tx.cmds = [21] * 20
                watch.until(lambda: watch.values.get(CMD) == 21)
            tx.value = 700
            _established(watch, sim, 700, RUN)
            initial = watch.values.get(FWD)
            first = len(watch.events)
            tx.value = 0
            watch.until(lambda: watch.values.get(RUN) == 0, timeout=1)
            tx.cmds = [command] * 12
            watch.until(lambda: watch.values.get(FWD) != initial, timeout=2)
            events = watch.events[first:]
            t_stop = next(ts for ts, i, v in events if i == RUN and v == 0)
            t_cmd = next(ts for ts, i, v in events if i == CMD and v == command)
            t_flip = next(ts for ts, i, v in events if i == FWD and v != initial)
            assert t_cmd - t_stop < 50 * MS, 'command latched %.1f ms after the stop: too late to test the hold' % (
                (t_cmd - t_stop) / MS)
            assert t_flip - t_stop >= 95 * MS, 'direction applied %.1f ms after the active-brake stop' % (
                (t_flip - t_stop) / MS)
    finally:
        tx.stop()
