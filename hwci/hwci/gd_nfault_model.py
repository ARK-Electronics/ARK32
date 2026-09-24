"""Host twin of the DRV8350H nFAULT handling in Src/faults.c.

Constants are parsed from firmware. This remains a transcription, not a
test of the compiled C. step() observes nFAULT; drive_loss() represents the
existing BEMF stall or established-run desync path deciding that
commutation has already failed.
"""
from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path

GD_NF_IDLE = 0
GD_NF_CLASSIFY = 1
GD_NF_WARN = 2
GD_NF_LATCH = 3

GD_NAMES = ("IDLE", "CLASSIFY", "WARN", "LATCH")

FAULT_NONE = 0
FAULT_GD_UVLO = 4
FAULT_GD_OCP = 5
FAULT_GD_OTW = 6
FAULT_GD_OTSD = 7
FAULT_GD_UNKNOWN = 8

LOG_NONE = 0
LOG_WARNING = 1
LOG_ERROR = 2

_DEFINE_NAMES = (
    "GD_CLASSIFY_MS",
    "GD_UVLO_CV",
    "GD_THERMAL_C",
    "GD_FAULT_RECENT_MS",
    "GD_RETRY_LOG_MS",
    "GD_WARN_LOG_MS",
    "GD_REARM_ZERO_MS",
)


def _repo_faults_c() -> Path:
    return Path(__file__).resolve().parents[2] / "Src" / "faults.c"


def parse_gd_defines_from_c(path: Path | None = None) -> dict[str, int]:
    src = (path or _repo_faults_c()).read_text()
    block = src.split("#if defined(USE_DRV_NFAULT)", 1)
    if len(block) < 2:
        raise ValueError("USE_DRV_NFAULT block not found in faults.c")
    body = block[1].split("#elif defined(USE_DRV8328_NFAULT)", 1)[0]
    out: dict[str, int] = {}
    for name in _DEFINE_NAMES:
        m = re.search(rf"#\s*define\s+{name}\s+(\d+)", body)
        if not m:
            raise ValueError(f"{name} not found in faults.c nFAULT block")
        out[name] = int(m.group(1))
    return out


GD: dict[str, int] = parse_gd_defines_from_c()


def _u16(x: int) -> int:
    return x & 0xFFFF


@dataclass
class GdNfaultMachine:
    """Warnings observe the pin; only an actual drive-loss hook can latch."""

    state: int = GD_NF_IDLE
    cause: int = FAULT_NONE
    t0: int = 0
    retry_log_t0: int = 0
    retry_logged: int = 0
    warn_log_t0: int = 0
    warn_logged: int = 0
    fault_seen: int = 0
    fault_age_ms: int = 0
    zero_ms: int = 0
    log_level: int = LOG_NONE
    log_cause: int = FAULT_NONE
    ms: int = 0
    sleep_calls: int = 0

    pin_low: int = 0
    adjusted_input: int = 0
    input: int = 0
    running: int = 0
    stepper_sine: int = 0
    actual_current: int = 0
    battery_voltage: int = 3200
    degrees_celsius: int = 40
    awake: int = 1
    pin_trusted: int = 1

    def queue_log(self, level: int, cause: int) -> None:
        if level > self.log_level:
            self.log_level = level
            self.log_cause = cause

    def consume_log(self) -> tuple[int, int]:
        level, cause = self.log_level, self.log_cause
        self.log_level = LOG_NONE
        self.log_cause = FAULT_NONE
        return level, cause

    def rate_due(self, have_logged: int, t0: int, period: int) -> tuple[bool, int, int]:
        if not have_logged or _u16(self.ms - t0) >= period:
            return True, 1, self.ms
        return False, have_logged, t0

    def hold_cut(self) -> None:
        self.input = 0
        self.running = 0
        self.stepper_sine = 0

    def record_fault(self) -> None:
        self.fault_seen = 1
        self.fault_age_ms = 0

    def clear(self) -> None:
        self.state = GD_NF_IDLE
        self.cause = FAULT_NONE
        self.fault_seen = 0
        self.zero_ms = 0

    def drive_loss(self) -> bool:
        """Return whether the real commutation failure must remain latched."""
        if self.state == GD_NF_LATCH:
            return True
        if self.adjusted_input == 0 or self.input < 48:
            return False
        if self.awake and self.pin_trusted and self.pin_low:
            self.record_fault()
        if not self.fault_seen or self.fault_age_ms > GD["GD_FAULT_RECENT_MS"]:
            return False
        self.hold_cut()
        self.zero_ms = 0
        self.state = GD_NF_LATCH
        if 0 < self.battery_voltage < GD["GD_UVLO_CV"]:
            self.cause = FAULT_GD_UVLO
        elif self.degrees_celsius >= GD["GD_THERMAL_C"]:
            self.cause = FAULT_GD_OTSD
        else:
            self.cause = FAULT_GD_UNKNOWN
        self.queue_log(LOG_ERROR, self.cause)
        return True

    def fault_active(self) -> bool:
        return self.state == GD_NF_LATCH

    def warning_active(self) -> bool:
        return self.state == GD_NF_WARN

    def tick_ms(self, n: int = 1) -> None:
        for _ in range(n):
            self.ms = _u16(self.ms + 1)
            self.fault_age_ms = min(self.fault_age_ms + 1, GD["GD_FAULT_RECENT_MS"] + 1)
            if self.state == GD_NF_LATCH and self.adjusted_input == 0:
                self.zero_ms = min(self.zero_ms + 1, GD["GD_REARM_ZERO_MS"])
            else:
                self.zero_ms = 0

    def step(self) -> None:
        if self.state == GD_NF_LATCH:
            self.hold_cut()
            self.sleep_calls += 1
            self.awake = 0
            if self.adjusted_input == 0 and self.zero_ms >= GD["GD_REARM_ZERO_MS"]:
                self.clear()
            return
        if not self.awake:
            self.clear()
            return
        if not self.pin_trusted:
            return

        if self.pin_low:
            self.record_fault()

        if self.state == GD_NF_IDLE:
            if self.pin_low:
                self.t0 = self.ms
                self.cause = FAULT_NONE
                self.state = GD_NF_CLASSIFY
            return

        if self.state == GD_NF_CLASSIFY:
            if not self.pin_low:
                self.state = GD_NF_IDLE
                self.cause = FAULT_NONE
                due, self.retry_logged, self.retry_log_t0 = self.rate_due(
                    self.retry_logged, self.retry_log_t0, GD["GD_RETRY_LOG_MS"])
                if due:
                    self.queue_log(LOG_WARNING, FAULT_GD_OCP)
                return
            if _u16(self.ms - self.t0) >= GD["GD_CLASSIFY_MS"]:
                self.state = GD_NF_WARN
                self.cause = FAULT_GD_UNKNOWN
                due, self.warn_logged, self.warn_log_t0 = self.rate_due(
                    self.warn_logged, self.warn_log_t0, GD["GD_WARN_LOG_MS"])
                if due:
                    self.queue_log(LOG_WARNING, FAULT_GD_UNKNOWN)
            return

        if self.state == GD_NF_WARN:
            if not self.pin_low:
                self.state = GD_NF_IDLE
                self.cause = FAULT_NONE
            return

        self.clear()

    def run_ms(self, n: int) -> None:
        for _ in range(n):
            self.tick_ms()
            self.step()
