"""DRV8350H pin warnings and commutation-loss latch (host model)."""
from __future__ import annotations

import pytest

from hwci.gd_nfault_model import (
    FAULT_GD_OCP,
    FAULT_GD_OTSD,
    FAULT_GD_UNKNOWN,
    FAULT_GD_UVLO,
    GD,
    GD_NF_CLASSIFY,
    GD_NF_IDLE,
    GD_NF_LATCH,
    GD_NF_WARN,
    LOG_ERROR,
    LOG_NONE,
    LOG_WARNING,
    GdNfaultMachine,
    parse_gd_defines_from_c,
)


def _spinning(**kw) -> GdNfaultMachine:
    fields = dict(running=1, adjusted_input=500, input=500, actual_current=2000)
    fields.update(kw)
    return GdNfaultMachine(**fields)


def _pulse(m: GdNfaultMachine, duration_ms: int = 1) -> None:
    m.pin_low = 1
    m.step()
    m.run_ms(duration_ms)
    m.pin_low = 0
    m.step()


def test_defines_parsed_from_firmware():
    assert parse_gd_defines_from_c() == GD
    assert GD["GD_CLASSIFY_MS"] == 12
    assert GD["GD_FAULT_RECENT_MS"] == 100
    assert GD["GD_REARM_ZERO_MS"] == 100


@pytest.mark.parametrize("current", [0, 20, 49, 50, 80, 2000, 32767])
@pytest.mark.parametrize("drive", ["idle", "six_step", "sine"])
def test_held_nfault_warns_without_cutting_at_any_current(current, drive):
    m = _spinning(actual_current=current)
    if drive == "idle":
        m.adjusted_input = m.input = m.running = 0
    elif drive == "sine":
        m.running = 0
        m.stepper_sine = 1
    before = (m.input, m.running, m.stepper_sine)
    m.pin_low = 1
    m.step()
    m.run_ms(1000)
    assert m.state == GD_NF_WARN
    assert m.warning_active()
    assert not m.fault_active()
    assert (m.input, m.running, m.stepper_sine) == before
    assert m.consume_log() == (LOG_WARNING, FAULT_GD_UNKNOWN)


def test_zero_flickering_and_stale_current_never_cut_drive():
    m = _spinning(pin_low=1)
    m.step()
    m.run_ms(GD["GD_CLASSIFY_MS"])
    for current in [0, 49, 80, 0, 32767] * 10:
        m.actual_current = current
        m.run_ms(100)
        assert m.state == GD_NF_WARN
        assert m.running == 1


def test_idle_and_rethrottle_during_classification_do_not_cut_drive():
    m = _spinning(pin_low=1, actual_current=0, running=0, input=0, adjusted_input=0)
    m.step()
    m.run_ms(GD["GD_CLASSIFY_MS"] - 2)
    m.running = 1
    m.input = m.adjusted_input = 500
    m.run_ms(100)
    assert m.state == GD_NF_WARN
    assert m.running == 1


def test_visible_retry_pulses_have_no_shutdown_budget():
    m = _spinning()
    for _ in range(300):
        _pulse(m)
        assert m.state == GD_NF_IDLE
        assert m.running == 1
        assert not m.fault_active()
    assert m.consume_log() == (LOG_WARNING, FAULT_GD_OCP)


def test_pin_release_clears_warning():
    m = _spinning(pin_low=1)
    m.step()
    m.run_ms(GD["GD_CLASSIFY_MS"])
    m.pin_low = 0
    m.step()
    assert m.state == GD_NF_IDLE
    assert not m.warning_active()


@pytest.mark.parametrize("already_polled", [False, True])
def test_drive_loss_with_held_nfault_latches(already_polled):
    m = _spinning(pin_low=1)
    if already_polled:
        m.step()
        m.run_ms(GD["GD_CLASSIFY_MS"])
    assert m.drive_loss()
    assert m.state == GD_NF_LATCH
    assert m.fault_active()
    assert (m.input, m.running, m.stepper_sine) == (0, 0, 0)
    assert m.adjusted_input == 500
    assert m.consume_log() == (LOG_ERROR, FAULT_GD_UNKNOWN)
    assert m.drive_loss()


@pytest.mark.parametrize("voltage,temperature,cause", [
    (600, 40, FAULT_GD_UVLO),
    (3200, 120, FAULT_GD_OTSD),
    (3200, 40, FAULT_GD_UNKNOWN),
])
def test_drive_loss_labels_are_best_effort(voltage, temperature, cause):
    m = _spinning(pin_low=1, battery_voltage=voltage, degrees_celsius=temperature)
    assert m.drive_loss()
    assert m.consume_log() == (LOG_ERROR, cause)


@pytest.mark.parametrize("age", [0, 1, 50, 100])
def test_recent_released_nfault_latches_on_actual_drive_loss(age):
    m = _spinning()
    _pulse(m)
    m.run_ms(age)
    assert m.drive_loss()


@pytest.mark.parametrize("age", [101, 65536, 65536 + 50, 131072])
def test_stale_fault_history_does_not_latch_or_wrap_back_to_recent(age):
    m = _spinning()
    _pulse(m)
    m.run_ms(age)
    assert not m.drive_loss()
    assert m.running == 1
    assert not m.fault_active()


def test_ordinary_drive_loss_preserves_existing_recovery():
    m = _spinning()
    assert not m.drive_loss()
    assert m.state == GD_NF_IDLE
    assert m.running == 1
    assert m.consume_log()[0] == LOG_NONE


@pytest.mark.parametrize("adjusted,input_value", [(0, 0), (0, 500), (500, 0), (500, 47)])
def test_loss_without_drive_request_does_not_latch(adjusted, input_value):
    m = _spinning(pin_low=1, adjusted_input=adjusted, input=input_value)
    assert not m.drive_loss()
    assert not m.fault_active()


@pytest.mark.parametrize("awake,trusted", [(0, 1), (1, 0), (0, 0)])
def test_untrusted_or_sleeping_pin_does_not_create_fault_history(awake, trusted):
    m = _spinning(pin_low=1, awake=awake, pin_trusted=trusted)
    m.step()
    assert not m.drive_loss()
    assert not m.fault_seen


def test_pin_release_never_resumes_a_latched_motor_at_throttle():
    m = _spinning(pin_low=1)
    assert m.drive_loss()
    m.pin_low = 0
    m.run_ms(70000)
    assert m.state == GD_NF_LATCH
    assert m.adjusted_input == 500
    assert m.running == 0
    assert not m.awake


def test_one_zero_frame_does_not_clear_latch():
    m = _spinning(pin_low=1)
    assert m.drive_loss()
    m.pin_low = 0
    m.adjusted_input = 0
    m.run_ms(1)
    m.adjusted_input = 500
    m.run_ms(1000)
    assert m.state == GD_NF_LATCH
    assert m.zero_ms == 0


@pytest.mark.parametrize("pin_low,awake", [(1, 1), (0, 1), (1, 0), (0, 0)])
def test_continuous_zero_rearms_even_when_driver_asleep_or_pin_held(pin_low, awake):
    m = _spinning(pin_low=1)
    assert m.drive_loss()
    m.pin_low = pin_low
    m.awake = awake
    m.adjusted_input = 0
    m.run_ms(GD["GD_REARM_ZERO_MS"] - 1)
    assert m.state == GD_NF_LATCH
    m.run_ms(1)
    assert m.state == GD_NF_IDLE
    assert not m.fault_seen
    assert not m.awake
    assert m.sleep_calls > 0


def test_throttle_up_between_rearm_tick_and_poll_keeps_latch():
    m = _spinning(pin_low=1)
    assert m.drive_loss()
    m.adjusted_input = 0
    m.tick_ms(GD["GD_REARM_ZERO_MS"])
    m.adjusted_input = 500
    m.step()
    assert m.state == GD_NF_LATCH
    assert m.running == 0


def test_sleep_does_not_bypass_zero_throttle_rearm():
    m = _spinning(pin_low=1)
    assert m.drive_loss()
    m.awake = 0
    m.run_ms(1000)
    assert m.state == GD_NF_LATCH
    assert m.zero_ms == 0


def test_sleep_clears_nonlatched_warning_and_history():
    m = _spinning(pin_low=1)
    m.step()
    m.run_ms(GD["GD_CLASSIFY_MS"])
    m.awake = 0
    m.step()
    assert m.state == GD_NF_IDLE
    assert not m.fault_seen


def test_short_and_held_warning_logs_are_rate_limited():
    m = _spinning()
    _pulse(m)
    assert m.consume_log() == (LOG_WARNING, FAULT_GD_OCP)
    _pulse(m)
    assert m.consume_log()[0] == LOG_NONE
    m.run_ms(GD["GD_RETRY_LOG_MS"])
    _pulse(m)
    assert m.consume_log() == (LOG_WARNING, FAULT_GD_OCP)
    for expected in [LOG_WARNING, LOG_NONE]:
        m.pin_low = 1
        m.step()
        m.run_ms(GD["GD_CLASSIFY_MS"])
        assert m.consume_log()[0] == expected
        m.pin_low = 0
        m.step()
    m.run_ms(GD["GD_WARN_LOG_MS"])
    m.pin_low = 1
    m.step()
    m.run_ms(GD["GD_CLASSIFY_MS"])
    assert m.consume_log() == (LOG_WARNING, FAULT_GD_UNKNOWN)


def test_classifier_handles_clock_wrap():
    m = _spinning(pin_low=1, ms=65530)
    m.step()
    m.run_ms(GD["GD_CLASSIFY_MS"] - 1)
    assert m.state == GD_NF_CLASSIFY
    m.run_ms(1)
    assert m.state == GD_NF_WARN


def test_error_log_upgrades_warning_without_clobbering_equal_severity():
    m = _spinning()
    m.queue_log(LOG_WARNING, FAULT_GD_OCP)
    m.queue_log(LOG_ERROR, FAULT_GD_UVLO)
    m.queue_log(LOG_ERROR, FAULT_GD_UNKNOWN)
    assert m.consume_log() == (LOG_ERROR, FAULT_GD_UVLO)
    assert m.consume_log()[0] == LOG_NONE
