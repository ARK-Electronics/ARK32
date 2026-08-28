"""DRV8350H nFAULT classifier: host twin of Src/faults.c (no MCU)."""
from __future__ import annotations

from hwci.gd_nfault_model import (
    FAULT_GD_OCP,
    FAULT_GD_OTSD,
    FAULT_GD_OTW,
    FAULT_GD_UNKNOWN,
    FAULT_GD_UVLO,
    GD,
    GD_NAMES,
    GD_NF_CLASSIFY,
    GD_NF_HIZ,
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
    m = GdNfaultMachine(running=1, adjusted_input=500, actual_current=2000)
    for key, value in kw.items():
        setattr(m, key, value)
    return m


def test_defines_parsed_from_firmware():
    parsed = parse_gd_defines_from_c()
    assert parsed == GD
    assert GD["GD_CLASSIFY_MS"] == 12
    assert GD["GD_LIVE_CA_MIN"] == 50
    assert GD["GD_DEAD_DWELL_MS"] >= 50
    assert GD["GD_RESUME_BUDGET"] >= 1


def test_short_pulse_is_retry_not_hiz():
    m = _spinning()
    m.pin_low = 1
    m.step()
    assert m.state == GD_NF_CLASSIFY
    m.run_ms(GD["GD_CLASSIFY_MS"] - 1)
    assert m.state == GD_NF_CLASSIFY
    m.pin_low = 0
    m.step()
    assert m.state == GD_NF_IDLE
    assert not m.fault_active()
    assert m.retry_count == 1


def test_held_live_is_otw_warning():
    m = _spinning()
    m.pin_low = 1
    m.step()
    m.run_ms(GD["GD_CLASSIFY_MS"])
    assert m.state == GD_NF_WARN
    assert m.cause == FAULT_GD_OTW
    assert m.warning_active()
    assert not m.fault_active()
    level, cause = m.consume_log()
    assert level == LOG_WARNING
    assert cause == FAULT_GD_OTW


def test_idle_during_classify_is_warn_not_dead():
    m = _spinning()
    m.pin_low = 1
    m.step()
    m.adjusted_input = 0
    m.actual_current = 0
    m.run_ms(GD["GD_CLASSIFY_MS"])
    assert m.state == GD_NF_WARN
    assert not m.fault_active()


def test_otw_idle_current_drop_does_not_hiz():
    """Customer symptom: OTW + pilot idle must not latch / Hi-Z."""
    m = _spinning()
    m.pin_low = 1
    m.step()
    m.run_ms(GD["GD_CLASSIFY_MS"])
    assert m.state == GD_NF_WARN
    m.adjusted_input = 0
    m.running = 1
    m.actual_current = 0
    m.run_ms(GD["GD_DEAD_DWELL_MS"] + 20)
    assert m.state == GD_NF_WARN
    assert not m.fault_active()
    assert GD_NAMES[m.state] == "WARN"


def test_otw_rethrottle_boxcar_does_not_false_hiz():
    m = _spinning()
    m.pin_low = 1
    m.step()
    m.run_ms(GD["GD_CLASSIFY_MS"])
    m.adjusted_input = 0
    m.actual_current = 0
    m.step()
    m.adjusted_input = 500
    m.running = 1
    m.actual_current = 0
    m.run_ms(GD["GD_DEAD_DWELL_MS"] - 1)
    assert m.state == GD_NF_WARN
    m.actual_current = 200
    m.step()
    assert m.state == GD_NF_WARN
    assert not m.fault_active()


def test_commanded_collapse_after_dwell_is_hiz_not_latch():
    m = _spinning()
    m.pin_low = 1
    m.step()
    m.run_ms(GD["GD_CLASSIFY_MS"])
    assert m.state == GD_NF_WARN
    m.actual_current = 0
    m.run_ms(GD["GD_DEAD_DWELL_MS"] + 1)
    assert m.state == GD_NF_HIZ
    assert m.cause == FAULT_GD_UNKNOWN
    assert m.fault_active()
    assert m.state != GD_NF_LATCH


def test_mcu_hot_is_log_label_not_hiz_vs_latch():
    m = _spinning(degrees_celsius=120)
    m.pin_low = 1
    m.actual_current = 0
    m.step()
    m.run_ms(GD["GD_CLASSIFY_MS"])
    assert m.state == GD_NF_HIZ
    assert m.cause == FAULT_GD_OTSD
    assert m.state != GD_NF_LATCH


def test_mcu_cool_held_dead_is_hiz_not_gdf_latch():
    m = _spinning(degrees_celsius=25)
    m.pin_low = 1
    m.actual_current = 0
    m.step()
    m.run_ms(GD["GD_CLASSIFY_MS"])
    assert m.state == GD_NF_HIZ
    assert m.cause == FAULT_GD_UNKNOWN
    assert m.state != GD_NF_LATCH


def test_uvlo_is_hiz():
    m = _spinning(battery_voltage=600)
    m.pin_low = 1
    m.actual_current = 0
    m.step()
    m.run_ms(GD["GD_CLASSIFY_MS"])
    assert m.state == GD_NF_HIZ
    assert m.cause == FAULT_GD_UVLO


def test_hiz_zero_throttle_stays_hiz_while_pin_low():
    m = _spinning()
    m.pin_low = 1
    m.actual_current = 0
    m.step()
    m.run_ms(GD["GD_CLASSIFY_MS"])
    assert m.state == GD_NF_HIZ
    m.adjusted_input = 0
    m.step()
    assert m.state == GD_NF_HIZ
    assert m.keep_awake()


def test_hiz_pin_release_returns_idle():
    m = _spinning()
    m.pin_low = 1
    m.actual_current = 0
    m.step()
    m.run_ms(GD["GD_CLASSIFY_MS"])
    m.pin_low = 0
    m.step()
    assert m.state == GD_NF_IDLE
    assert not m.fault_active()


def test_hiz_trst_budget_latches():
    m = _spinning()
    m.pin_low = 1
    m.actual_current = 0
    m.step()
    m.run_ms(GD["GD_CLASSIFY_MS"])
    m.adjusted_input = 0
    for _ in range(GD["GD_RESUME_BUDGET"]):
        m.run_ms(GD["GD_HIZ_RST_MS"])
        assert m.state == GD_NF_HIZ
    m.run_ms(GD["GD_HIZ_RST_MS"])
    assert m.state == GD_NF_LATCH
    assert m.rst_pulses == GD["GD_RESUME_BUDGET"]


def test_retry_budget_latches_when_pulses_are_visible():
    m = _spinning()
    for _ in range(GD["GD_RETRY_BUDGET"]):
        m.pin_low = 1
        m.step()
        assert m.state == GD_NF_CLASSIFY
        m.pin_low = 0
        m.ms = (m.ms + 1) & 0xFFFF
        m.step()
        if m.state == GD_NF_LATCH:
            break
    assert m.state == GD_NF_LATCH
    assert m.cause == FAULT_GD_OCP


def test_queue_log_does_not_clobber_same_level():
    m = GdNfaultMachine()
    m.queue_log(LOG_ERROR, FAULT_GD_UVLO)
    m.queue_log(LOG_ERROR, FAULT_GD_UNKNOWN)
    m.queue_log(LOG_WARNING, FAULT_GD_OTW)
    level, cause = m.consume_log()
    assert level == LOG_ERROR
    assert cause == FAULT_GD_UVLO
    level, _ = m.consume_log()
    assert level == LOG_NONE


def test_otw_log_is_rate_limited():
    m = _spinning()
    m.pin_low = 1
    m.step()
    m.run_ms(GD["GD_CLASSIFY_MS"])
    assert m.consume_log()[0] == LOG_WARNING
    m.pin_low = 0
    m.step()
    m.pin_low = 1
    m.step()
    m.run_ms(GD["GD_CLASSIFY_MS"])
    assert m.consume_log()[0] == LOG_NONE
    m.pin_low = 0
    m.step()
    m.tick_ms(GD["GD_OTW_LOG_MS"])
    m.pin_low = 1
    m.step()
    m.run_ms(GD["GD_CLASSIFY_MS"])
    assert m.consume_log()[0] == LOG_WARNING
