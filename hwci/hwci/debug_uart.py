"""ARK_G431_CAN debug UART (USART2 TX PB3 @ 115200 via ST-Link VCP).

Firmware (``Src/debug_uart.c``, ``USE_DEBUG_UART``) emits a small text console:

* boot banner: ``ARK_G431_CAN debug UART @ 115200 (PB3/USART2)``
* state: ``esc: <from> -> <to>``
* faults: ``fault: nFAULT`` / ``fault: desync`` / …

This is **not** KISS telemetry (that is a separate optional wire). On the
thrust-stand bench the TX pin is wired to the ST-Link V3 Virtual COM Port;
udev can name it ``/dev/esc-debug-uart`` (see ``scripts/99-hwci.rules``).
"""
from __future__ import annotations

import collections
import re
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

# Lines that mean the motor/drive path is unhealthy — live desync watch.
_ABORT_FAULTS = frozenset({
    "nFAULT",
    "desync",
    "acq_desync",
    "stuck",
    "stall",
})

_FAULT_RE = re.compile(r"^fault:\s*(\S+)", re.IGNORECASE)
_STATE_RE = re.compile(r"^esc:\s*(.+?)\s*->\s*(.+)\s*$", re.IGNORECASE)


@dataclass
class DebugUartLine:
    t_mono: float
    text: str
    kind: str = "raw"          # raw | fault | state | boot
    fault: str | None = None
    state_from: str | None = None
    state_to: str | None = None


@dataclass
class DebugUartReader:
    """Background reader for the ST-Link VCP debug console."""

    port: str
    baud: int = 115200
    max_lines: int = 5000
    _ser: object | None = field(default=None, init=False, repr=False)
    _stop: threading.Event = field(default_factory=threading.Event, init=False)
    _thread: threading.Thread | None = field(default=None, init=False, repr=False)
    _lock: threading.Lock = field(default_factory=threading.Lock, init=False)
    _lines: collections.deque = field(default=None, init=False, repr=False)
    _abort_fault: str | None = field(default=None, init=False)
    errors: int = field(default=0, init=False)
    last_error: Exception | None = field(default=None, init=False)

    def __post_init__(self) -> None:
        self._lines = collections.deque(maxlen=self.max_lines)

    def open(self) -> "DebugUartReader":
        import serial  # pyserial — already a harness dependency for KISS telem

        self._ser = serial.Serial(self.port, self.baud, timeout=0.1)
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._loop, daemon=True, name="debug-uart")
        self._thread.start()
        return self

    def _loop(self) -> None:
        assert self._ser is not None
        buf = bytearray()
        while not self._stop.is_set():
            try:
                chunk = self._ser.read(256)
            except Exception as e:
                self.errors += 1
                self.last_error = e
                time.sleep(0.05)
                continue
            if not chunk:
                continue
            buf.extend(chunk)
            while True:
                nl = buf.find(b"\n")
                if nl < 0:
                    break
                raw = bytes(buf[:nl])
                del buf[: nl + 1]
                text = raw.decode("utf-8", errors="replace").rstrip("\r")
                if not text:
                    continue
                rec = _classify(text, time.monotonic())
                with self._lock:
                    self._lines.append(rec)
                    if rec.kind == "fault" and rec.fault in _ABORT_FAULTS:
                        self._abort_fault = rec.fault

    def drain_abort_fault(self) -> str | None:
        """Return and clear a latched abort-worthy fault name, if any."""
        with self._lock:
            f = self._abort_fault
            self._abort_fault = None
            return f

    def lines(self) -> list[DebugUartLine]:
        with self._lock:
            return list(self._lines)

    def save_log(self, path: str | Path) -> Path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w") as fh:
            for rec in self.lines():
                fh.write(f"{rec.t_mono:.3f}\t{rec.text}\n")
        return path

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None


def _classify(text: str, t_mono: float) -> DebugUartLine:
    m = _FAULT_RE.match(text)
    if m:
        return DebugUartLine(
            t_mono=t_mono, text=text, kind="fault", fault=m.group(1))
    m = _STATE_RE.match(text)
    if m:
        return DebugUartLine(
            t_mono=t_mono, text=text, kind="state",
            state_from=m.group(1).strip(), state_to=m.group(2).strip())
    if "debug UART" in text or text.startswith("ARK_G431"):
        return DebugUartLine(t_mono=t_mono, text=text, kind="boot")
    return DebugUartLine(t_mono=t_mono, text=text, kind="raw")


def resolve_stlink_vcp(prefer: str | None = None) -> str | None:
    """Best-effort path to the ST-Link V3 VCP (interface 01).

    Order: explicit prefer if it exists → ``/dev/esc-debug-uart`` →
    ``/dev/serial/by-id/*STLINK*if01*``.
    """
    candidates: list[str] = []
    if prefer:
        candidates.append(prefer)
    candidates.append("/dev/esc-debug-uart")
    by_id = Path("/dev/serial/by-id")
    if by_id.is_dir():
        for p in sorted(by_id.glob("usb-STMicroelectronics_STLINK*if01*")):
            candidates.append(str(p))
        for p in sorted(by_id.glob("usb-STMicroelectronics_ST-LINK*if01*")):
            candidates.append(str(p))
    for c in candidates:
        if Path(c).exists():
            return c
    return None
