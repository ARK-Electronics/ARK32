"""Exercise ADC scheduling and fault logging extracted from the firmware.

These host C harnesses stub hardware and record outputs. The clock blocks
and logging function come directly from their production source files.
"""

from pathlib import Path
import shutil
import subprocess

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]


def _compile_and_run(tmp_path, source, defines=()):
    compiler = shutil.which("cc") or shutil.which("gcc")
    if compiler is None:
        pytest.skip("no host C compiler available")
    harness = tmp_path / "integration.c"
    harness.write_text(source)
    executable = tmp_path / "integration"
    result = subprocess.run(
        [compiler, "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
         *defines, f"-I{REPO_ROOT / 'Inc'}", str(harness), "-o", str(executable)],
        capture_output=True, text=True, timeout=30,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    result = subprocess.run(
        [str(executable)], capture_output=True, text=True, timeout=10,
    )
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.parametrize("gate_driver", [False, True])
def test_adc_schedule_in_sine_and_six_step(tmp_path, gate_driver):
    source = (REPO_ROOT / "Src" / "control_loop.c").read_text()
    start = source.index("RAM_FUNC void tenKhzRoutine()")
    end = source.index("\tif (!escIsArmed())", start)
    clock_block = source[start:end]
    start = source.index("\t\tif (one_khz_loop_counter > PID_LOOP_DIVIDER)", end)
    end = source.index("\t\t\tfaultDesyncEpisodeTick1kHz();", start)
    pid_clock_block = source[start:end]

    harness = r"""
#include <assert.h>
#include <stdint.h>
#define RAM_FUNC
#define HWCI_PERF_CTRL_ENTER() ((void)0)
#define PID_LOOP_DIVIDER 20
static unsigned duty_cycle, duty_cycle_setpoint, tenkhzcounter, ledcounter;
static unsigned ramp_count, one_khz_loop_counter, PROCESS_ADC_FLAG;
static unsigned gd_ticks, pid_ticks;
static int sine;
#ifdef USE_DRV_NFAULT
static void faultGateDriverTick1kHz(void) { gd_ticks++; }
#endif
"""
    # Retain the actual scheduling blocks, with the PID block in the same
    # six-step-only branch as production. Unrelated control work is omitted.
    harness += clock_block
    harness += "\tif (!sine) {\n" + pid_clock_block
    harness += "\t\t\tpid_ticks++;\n\t\t}\n\t}\n}\n"
    harness += r"""
static void exercise(int sine_mode, unsigned expected_pid_ticks) {
    sine = sine_mode;
    gd_ticks = pid_ticks = PROCESS_ADC_FLAG = 0;
    unsigned adc_samples = 0;
    for (unsigned i = 0; i < 20000; i++) {
        tenKhzRoutine();
        adc_samples += PROCESS_ADC_FLAG != 0;
        PROCESS_ADC_FLAG = 0;
    }
#ifdef USE_DRV_NFAULT
    assert(adc_samples == 1000 && gd_ticks == 1000);
#else
    assert(adc_samples == expected_pid_ticks && gd_ticks == 0);
#endif
    assert(pid_ticks == expected_pid_ticks);
}
int main(void) {
    exercise(0, 952);
    exercise(1, 0);
    /* The legacy PID counter accumulates during sine mode, so it fires
     * immediately on six-step re-entry. The ADC clock remains at 1 kHz. */
    exercise(0, 953);
}
"""
    _compile_and_run(tmp_path, harness, ("-DUSE_DRV_NFAULT",) if gate_driver else ())


def test_fault_warnings_preserve_independent_stuck_error(tmp_path):
    source = (REPO_ROOT / "Src" / "DroneCAN" / "DroneCAN.c").read_text()
    start = source.index("static void DroneCAN_pollFaultLogMessages(void)")
    end = source.index("/* Map ESC / gate state", start)
    harness = r"""
#include <assert.h>
#include <stdint.h>
#include <string.h>
#include "faults.h"
enum { ESC_IDLE, ESC_FAULT_STUCK };
enum { UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_WARNING = 2,
       UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_ERROR = 3 };
static uint8_t pending, state;
static fault_id_t pending_cause;
static struct { uint8_t level; const char *message; } logs[4];
static unsigned log_count;
uint8_t faultGateDriverConsumeLog(fault_id_t *cause) {
    *cause = pending_cause;
    uint8_t level = pending;
    pending = FAULT_GD_LOG_NONE;
    return level;
}
static uint8_t escGetState(void) { return state; }
static void can_log(uint8_t level, const char *message) {
    assert(log_count < 4);
    logs[log_count].level = level;
    logs[log_count++].message = message;
}
"""
    harness += source[start:end]
    harness += r"""
static void reset(void) {
    state = ESC_IDLE;
    pending = FAULT_GD_LOG_NONE;
    DroneCAN_pollFaultLogMessages();
    log_count = 0;
}
static void expect_log(unsigned index, uint8_t level, const char *message) {
    assert(index < log_count);
    assert(logs[index].level == level);
    assert(strcmp(logs[index].message, message) == 0);
}
static void warning(fault_id_t cause, const char *message, uint8_t stuck) {
    reset();
    state = stuck ? ESC_FAULT_STUCK : ESC_IDLE;
    pending = FAULT_GD_LOG_WARNING;
    pending_cause = cause;
    DroneCAN_pollFaultLogMessages();
    assert(log_count == (stuck ? 2u : 1u));
    expect_log(0, UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_WARNING, message);
    if (stuck) {
        expect_log(1, UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_ERROR, "stuck");
    }
    DroneCAN_pollFaultLogMessages();
    assert(log_count == (stuck ? 2u : 1u));
}
int main(void) {
    warning(FAULT_GD_UNKNOWN, "nFAULT", 0);
    warning(FAULT_GD_UNKNOWN, "nFAULT", 1);
    warning(FAULT_GD_OCP, "nFAULT retry", 1);
    warning(FAULT_GD_OTW, "nFAULT OTW", 1);
    reset();
    state = ESC_FAULT_STUCK;
    pending = FAULT_GD_LOG_ERROR;
    pending_cause = FAULT_GD_UVLO;
    DroneCAN_pollFaultLogMessages();
    assert(log_count == 1);
    expect_log(0, UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_ERROR, "nFAULT UVLO");
    DroneCAN_pollFaultLogMessages();
    assert(log_count == 1);
    reset();
    state = ESC_FAULT_STUCK;
    DroneCAN_pollFaultLogMessages();
    assert(log_count == 1);
    expect_log(0, UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_ERROR, "stuck");
}
"""
    _compile_and_run(tmp_path, harness)
