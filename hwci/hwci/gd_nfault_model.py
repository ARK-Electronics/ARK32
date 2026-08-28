"""Host twin of the DRV8350H nFAULT state machine in Src/faults.c.

Constants are parsed from firmware so Python cannot silently drift. The
step() body mirrors faultPollGateDriver()'s USE_DRV_NFAULT switch.
"""
from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path

GD_NF_IDLE = 0
GD_NF_CLASSIFY = 1
GD_NF_WARN = 2
GD_NF_HIZ = 3
GD_NF_LATCH = 4

GD_NAMES = ("IDLE", "CLASSIFY", "WARN", "HIZ", "LATCH")

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
    "GD_LIVE_CA_MIN",
    "GD_THERMAL_C",
    "GD_RETRY_WINDOW_MS",
    "GD_RETRY_BUDGET",
    "GD_RETRY_LOG_MS",
    "GD_OTW_LOG_MS",
    "GD_DEAD_DWELL_MS",
    "GD_HIZ_RST_MS",
    "GD_RESUME_BUDGET",
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
    """Behavioural twin of the DRV8350H poll in faults.c."""

    state: int = GD_NF_IDLE
    cause: int = FAULT_NONE
    t0: int = 0
    snap_ca: int = 0
    retry_count: int = 0
    retry_window_t0: int = 0
    retry_log_t0: int = 0
    otw_log_t0: int = 0
    dead_arm: int = 0
    dead_t0: int = 0
    resume_count: int = 0
    log_level: int = LOG_NONE
    log_cause: int = FAULT_NONE
    ms: int = 0
    rst_pulses: int = 0
    running: int = 0

    pin_low: int = 0
    adjusted_input: int = 0
    stepper_sine: int = 0
    actual_current: int = 0
    battery_voltage: int = 3200
    degrees_celsius: int = 40
    awake: int = 1
    pin_trusted: int = 1

    def drive_commanded(self) -> bool:
        return self.adjusted_input != 0 and (self.running or self.stepper_sine)

    def queue_log(self, level: int, cause: int) -> None:
        if level > self.log_level:
            self.log_level = level
            self.log_cause = cause

    def consume_log(self) -> tuple[int, int]:
        level, cause = self.log_level, self.log_cause
        self.log_level = LOG_NONE
        self.log_cause = FAULT_NONE
        return level, cause

    def hold_cut(self) -> None:
        self.running = 0
        self.stepper_sine = 0

    def enter_latch(self, cause: int) -> None:
        self.hold_cut()
        self.cause = cause
        self.state = GD_NF_LATCH
        self.queue_log(LOG_ERROR, cause)

    def enter_dead(self) -> None:
        self.hold_cut()
        self.t0 = self.ms
        self.dead_arm = 0
        self.state = GD_NF_HIZ
        if 0 < self.battery_voltage < GD["GD_UVLO_CV"]:
            self.cause = FAULT_GD_UVLO
        elif self.degrees_celsius >= GD["GD_THERMAL_C"]:
            self.cause = FAULT_GD_OTSD
        else:
            self.cause = FAULT_GD_UNKNOWN
        self.queue_log(LOG_ERROR, self.cause)

    def enter_warn(self, log_otw: bool) -> None:
        self.cause = FAULT_GD_OTW
        self.state = GD_NF_WARN
        if not log_otw:
            return
        if self.otw_log_t0 == 0 or _u16(self.ms - self.otw_log_t0) >= GD["GD_OTW_LOG_MS"]:
            self.otw_log_t0 = 1 if self.ms == 0 else self.ms
            self.queue_log(LOG_WARNING, FAULT_GD_OTW)

    def note_retry(self) -> None:
        if _u16(self.ms - self.retry_window_t0) > GD["GD_RETRY_WINDOW_MS"]:
            self.retry_count = 0
            self.retry_window_t0 = self.ms
        if self.retry_count < 255:
            self.retry_count += 1
        if self.retry_count >= GD["GD_RETRY_BUDGET"]:
            self.retry_count = 0
            self.enter_latch(FAULT_GD_OCP)
            return
        if _u16(self.ms - self.retry_log_t0) >= GD["GD_RETRY_LOG_MS"]:
            self.retry_log_t0 = self.ms
            self.queue_log(LOG_WARNING, FAULT_GD_OCP)

    def fault_active(self) -> bool:
        return self.state in (GD_NF_HIZ, GD_NF_LATCH)

    def warning_active(self) -> bool:
        return self.state == GD_NF_WARN

    def keep_awake(self) -> bool:
        return self.state == GD_NF_HIZ

    def tick_ms(self, n: int = 1) -> None:
        self.ms = _u16(self.ms + n)

    def step(self) -> None:
        if not self.awake:
            if self.adjusted_input == 0 and self.state != GD_NF_IDLE:
                self.state = GD_NF_IDLE
                self.cause = FAULT_NONE
                self.dead_arm = 0
                self.resume_count = 0
            return
        if not self.pin_trusted:
            return

        pin_low = bool(self.pin_low)
        now = self.ms
        drive_on = self.drive_commanded()
        current_live = self.actual_current >= GD["GD_LIVE_CA_MIN"]

        if self.state == GD_NF_IDLE:
            if pin_low:
                self.snap_ca = self.actual_current
                self.t0 = now
                self.dead_arm = 0
                self.cause = FAULT_NONE
                self.state = GD_NF_CLASSIFY
            return

        if self.state == GD_NF_CLASSIFY:
            if not pin_low:
                self.state = GD_NF_IDLE
                self.note_retry()
                return
            if _u16(now - self.t0) < GD["GD_CLASSIFY_MS"]:
                return
            if (
                drive_on
                and self.snap_ca >= GD["GD_LIVE_CA_MIN"]
                and self.actual_current >= (self.snap_ca >> 1)
            ):
                self.enter_warn(True)
                return
            if drive_on and not current_live:
                self.enter_dead()
                return
            self.enter_warn(False)
            return

        if self.state == GD_NF_WARN:
            if not pin_low:
                self.state = GD_NF_IDLE
                self.cause = FAULT_NONE
                self.dead_arm = 0
                return
            if not drive_on:
                self.dead_arm = 0
                return
            if not current_live:
                if not self.dead_arm:
                    self.dead_arm = 1
                    self.dead_t0 = now
                elif _u16(now - self.dead_t0) >= GD["GD_DEAD_DWELL_MS"]:
                    self.enter_dead()
            else:
                self.dead_arm = 0
            return

        if self.state == GD_NF_HIZ:
            self.hold_cut()
            if not pin_low:
                self.resume_count = 0
                self.state = GD_NF_IDLE
                self.cause = FAULT_NONE
                return
            if 0 < self.battery_voltage < GD["GD_UVLO_CV"]:
                return
            if self.adjusted_input != 0:
                return
            if _u16(now - self.t0) >= GD["GD_HIZ_RST_MS"]:
                self.t0 = now
                if self.resume_count >= GD["GD_RESUME_BUDGET"]:
                    self.enter_latch(
                        FAULT_GD_UNKNOWN if self.cause == FAULT_NONE else self.cause
                    )
                else:
                    self.resume_count += 1
                    self.rst_pulses += 1
            return

        if self.state == GD_NF_LATCH:
            self.hold_cut()
            if (not pin_low) and self.adjusted_input == 0:
                self.state = GD_NF_IDLE
                self.cause = FAULT_NONE
            return

        self.state = GD_NF_IDLE
        self.cause = FAULT_NONE

    def run_ms(self, n: int) -> None:
        for _ in range(n):
            self.tick_ms(1)
            self.step()
