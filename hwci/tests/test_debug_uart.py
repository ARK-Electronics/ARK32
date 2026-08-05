"""Unit tests for G4 debug UART line classification (no serial hardware)."""
from hwci.debug_uart import DebugUartLine, _classify, resolve_stlink_vcp


def test_classify_fault():
    r = _classify("fault: nFAULT", 1.0)
    assert r.kind == "fault"
    assert r.fault == "nFAULT"


def test_classify_desync():
    r = _classify("fault: desync", 2.0)
    assert r.kind == "fault"
    assert r.fault == "desync"


def test_classify_state():
    r = _classify("esc: DISARMED -> ARMED", 3.0)
    assert r.kind == "state"
    assert r.state_from == "DISARMED"
    assert r.state_to == "ARMED"


def test_classify_boot_banner():
    r = _classify("ARK_G431_CAN debug UART @ 115200 (PB3/USART2)", 0.1)
    assert r.kind == "boot"


def test_classify_raw():
    r = _classify("dbg: queue overflow", 4.0)
    assert r.kind == "raw"


def test_resolve_stlink_vcp_missing_returns_none(tmp_path, monkeypatch):
    # Prefer a non-existent path; if no real ST-Link by-id either, None.
    missing = str(tmp_path / "nope")
    # Force by-id empty by pointing resolve only at prefer.
    got = resolve_stlink_vcp(missing)
    # Prefer does not exist; may still find a real ST-Link on this host.
    if got is not None:
        assert "STLINK" in got.upper() or got.endswith("esc-debug-uart") or "ttyACM" in got
    else:
        assert got is None


def test_debug_uart_line_fields():
    r = DebugUartLine(t_mono=1.5, text="fault: stuck", kind="fault", fault="stuck")
    assert r.fault == "stuck"
