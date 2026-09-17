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


@pytest.fixture(scope="session")
def gd_firmware_executable(tmp_path_factory):
    compiler = shutil.which("cc") or shutil.which("gcc")
    if compiler is None:
        pytest.skip("no host C compiler available")
    build = tmp_path_factory.mktemp("gd_nfault_firmware")
    (build / "gd_nfault_firmware.inc").write_text(_firmware_source())
    executable = build / "gd_nfault_host"
    result = subprocess.run(
        [compiler, "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
         f"-I{REPO_ROOT / 'Inc'}", f"-I{build}", str(FIXTURE),
         "-o", str(executable)],
        capture_output=True, text=True, timeout=30,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    return executable


@pytest.mark.parametrize("scenario", [
    "low_current_warning",
    "repeated_pulses",
    "stale_current_warning",
    "sine_warning",
    "recent_fault_stall",
    "expired_fault_stall",
    "age_saturation",
    "manual_zero_clear",
    "zero_blip_retains_latch",
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
