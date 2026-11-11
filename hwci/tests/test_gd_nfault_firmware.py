"""Run the actual nFAULT C implementation with host-side hardware stubs.

The state machine and the BEMF stall/restart decision are extracted verbatim
from Src/faults.c. Only their hardware and surrounding control dependencies
are stubbed; unlike the Python model, changes to the firmware logic change
these tests immediately. Every case uses a fresh process to reset C statics.
"""
from pathlib import Path
import shutil
import subprocess

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
FIXTURE = Path(__file__).parent / "fixtures" / "gd_nfault_host.c"


def _firmware_source() -> str:
    source = (REPO_ROOT / "Src" / "faults.c").read_text()
    start = source.index("#if defined(USE_DRV_NFAULT) || defined(USE_DRV8328_NFAULT)")
    end = source.index("uint8_t faultHandleStuckRotorIfNeeded(void)", start)
    stall = source.index("void faultHandleBemfIntervalStall(void)")
    # The stall handler is the last function in faults.c. Keeping the entire
    # tail also preserves any helpers subsequently added after that handler.
    return source[start:end] + "\n" + source[stall:]


def _build(tmp_path_factory, name, defines=()):
    compiler = shutil.which("cc") or shutil.which("gcc")
    if compiler is None:
        pytest.skip("no host C compiler available")
    build = tmp_path_factory.mktemp(name)
    (build / "gd_nfault_firmware.inc").write_text(_firmware_source())
    executable = build / "gd_nfault_host"
    result = subprocess.run(
        [compiler, "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror", *defines,
         f"-I{REPO_ROOT / 'Inc'}", f"-I{build}", str(FIXTURE),
         "-o", str(executable)],
        capture_output=True, text=True, timeout=30,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    return executable


@pytest.fixture(scope="session")
def gd_firmware_executable(tmp_path_factory):
    return _build(tmp_path_factory, "gd_nfault_firmware")


@pytest.fixture(scope="session")
def drv8328_firmware_executable(tmp_path_factory):
    """The ARK 4IN1 (F051, DRV8328) branch of the same policy."""
    return _build(tmp_path_factory, "drv8328_nfault_firmware", ("-DHOST_DRV8328",))


@pytest.mark.parametrize("scenario", [
    "low_current_warning",
    "repeated_pulses",
    "stale_current_warning",
    "sine_warning",
    "recent_fault_stall",
    "expired_fault_stall",
    "age_saturation",
    "manual_zero_clear",
    "zero_waits_for_reset_sleep",
    "brake_wakes_before_clear",
    "release_does_not_restart",
    "sleep_does_not_clear_at_throttle",
    "ordinary_stall_restarts",
    "commanded_stop_does_not_latch",
    "untrusted_pin_ignored",
    "warning_log_upgrades_to_error",
    "prop_brake_cannot_keep_latch_awake",
])
def test_actual_gate_driver_firmware(gd_firmware_executable, scenario):
    result = subprocess.run(
        [str(gd_firmware_executable), scenario],
        capture_output=True, text=True, timeout=10,
    )
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.parametrize("current", [0, 49, 50, 2000, 32767])
def test_actual_stall_latches_independently_of_current(gd_firmware_executable, current):
    result = subprocess.run(
        [str(gd_firmware_executable), "fault_stall", str(current)],
        capture_output=True, text=True, timeout=10,
    )
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.parametrize("scenario", [
    "held_pin_rides_through",
    "recent_fault_stall",
    "expired_fault_stall",
    "age_saturation",
    "ordinary_stall_restarts",
    "commanded_stop_does_not_latch",
    "untrusted_pin_ignored",
    "sleep_voids_old_evidence",
    "release_does_not_restart",
    "sleep_does_not_clear_at_throttle",
    "zero_needs_sleep",
    "brake_wakes_before_clear",
    "prop_brake_cannot_keep_latch_awake",
])
def test_actual_drv8328_firmware(drv8328_firmware_executable, scenario):
    """ARK 4IN1: nFAULT is evidence; only a stall or hard desync within
    100 ms of it latches, and only pilot zero after sleep clears that."""
    result = subprocess.run(
        [str(drv8328_firmware_executable), scenario],
        capture_output=True, text=True, timeout=10,
    )
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.parametrize("current", [0, 2000, 32767])
def test_actual_drv8328_stall_latches_independently_of_current(drv8328_firmware_executable, current):
    result = subprocess.run(
        [str(drv8328_firmware_executable), "fault_stall", str(current)],
        capture_output=True, text=True, timeout=10,
    )
    assert result.returncode == 0, result.stdout + result.stderr
