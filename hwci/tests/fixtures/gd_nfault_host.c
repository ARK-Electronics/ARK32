/* Host hardware adapter for verbatim Src/faults.c regression tests. */
#include <assert.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "faults.h"

#define USE_DRV_NFAULT
#define USE_DRV_ENABLE
#define BEMF_STALL_TICKS 45000u
#define DSHOT_CMD_MAX 47u
#define DSHOT_MIN_THROTTLE 48u
#define ESC_STUCK_LATCH 255u
#define ESC_FAULT_STUCK 7u
#define DBG_EVT_STUCK 1u

static struct {
	uint32_t IDR;
} nfault_port = {1};
#define NFAULT_PORT (&nfault_port)
#define NFAULT_PIN 1u

static uint8_t running = 1, stepper_sine, prop_brake_active;
static uint16_t input = 500, adjusted_input = 500;
static uint16_t duty_cycle_setpoint = 700, duty_cycle = 700, last_duty_cycle = 700;
static uint16_t battery_voltage = 3200;
static int16_t actual_current = 2000, degrees_celsius = 40;
static uint32_t zero_crosses = 200;
static uint32_t desync_happened;
static uint8_t acq_fail_desyncs;
static uint8_t bemf_timeout_happened;
static uint32_t interval_timer_count;
#define INTERVAL_TIMER_COUNT interval_timer_count
volatile uint32_t fault_stall_trips;
volatile uint8_t fault_run_established = 1;
volatile uint8_t fault_acq_resist_events;

static uint8_t awake = 1, pin_trusted = 1, esc_state;
static unsigned off_calls, mask_calls, reset_calls, restart_calls, wake_calls;
static unsigned disable_timer_calls;
static uint16_t pwm_compare = 700;
#define SET_DUTY_CYCLE_ALL(value) (pwm_compare = (value))
#define DISABLE_COM_TIMER_INT() (disable_timer_calls++)

static void allOff(void)
{
	off_calls++;
}
static void maskPhaseInterrupts(void)
{
	mask_calls++;
}
static void bemfZcResetTrend(void) {}
uint8_t escGetState(void)
{
	return esc_state;
}
static void escToFaultStuck(void)
{
	esc_state = ESC_FAULT_STUCK;
	bemf_timeout_happened = ESC_STUCK_LATCH;
	input = 0;
	running = 0;
	stepper_sine = 0;
}
static uint8_t escIsFault(void)
{
	return bemf_timeout_happened == ESC_STUCK_LATCH || faultGateDriverFaultActive();
}
static uint8_t escInOpenLoop(void)
{
	return running && !stepper_sine && !escIsFault();
}
static uint8_t escInClosedLoop(void)
{
	return 0;
}
static void escNoteStallOrDesync(uint8_t stop)
{
	if (stop && input < 48) {
		running = 0;
	}
}
static uint8_t gateDriverIsAwake(void)
{
	return awake;
}
static uint8_t gateDriverNfaultPinTrusted(void)
{
	return awake && pin_trusted;
}
static void gateDriverNfaultGraceTick(void) {}
static void gateDriverSleep(void)
{
	awake = 0;
}
void gateDriverWakeBlocking(void)
{
	awake = 1;
	wake_calls++;
}
void gateDriverFaultResetPulse(void)
{
	reset_calls++;
}
void faultDesyncEpisodeCharge(desync_episode_kind_t kind)
{
	(void)kind;
}
uint8_t faultDesyncRestartHoldoffActive(void)
{
	return 0;
}
static void zcfoundroutine(void)
{
	restart_calls++;
	interval_timer_count = 0;
	zero_crosses++;
}

#include "gd_nfault_firmware.inc"

static void poll_ms(unsigned count)
{
	while (count--) {
		faultGateDriverTick1kHz();
		faultPollGateDriver();
	}
}

static void assert_no_cut(void)
{
	assert(!faultGateDriverFaultActive());
	assert(off_calls == 0);
	assert(mask_calls == 0);
	assert(disable_timer_calls == 0);
	assert(reset_calls == 0);
	assert(restart_calls == 0);
	assert(pwm_compare == 700);
	assert(duty_cycle_setpoint == 700);
	assert(duty_cycle == 700 && last_duty_cycle == 700);
}

static void stall(void)
{
	interval_timer_count = BEMF_STALL_TICKS + 1;
	faultHandleBemfIntervalStall();
}

static void assert_latched(void)
{
	assert(faultGateDriverFaultActive());
	assert(!running && !stepper_sine);
	assert(!prop_brake_active);
	assert(pwm_compare == 0);
	assert(duty_cycle_setpoint == 0);
	assert(duty_cycle == 0 && last_duty_cycle == 0);
	assert(disable_timer_calls > 0);
	assert(restart_calls == 0);
	assert(reset_calls == 0);
}

static void latch_fault(void)
{
	nfault_port.IDR = 0;
	faultPollGateDriver();
	stall();
	assert_latched();
}

int main(int argc, char **argv)
{
	assert(argc >= 2);
	const char *scenario = argv[1];
	if (!strcmp(scenario, "low_current_warning")) {
		actual_current = 0;
		nfault_port.IDR = 0;
		poll_ms(10000);
		assert_no_cut();
		assert(faultGateDriverWarningActive());
	} else if (!strcmp(scenario, "repeated_pulses")) {
		for (unsigned pulse = 0; pulse < 1000; pulse++) {
			actual_current = (pulse % 2) ? 0 : 32767;
			nfault_port.IDR = 0;
			poll_ms(8);
			nfault_port.IDR = 1;
			poll_ms(1);
		}
		assert_no_cut();
	} else if (!strcmp(scenario, "stale_current_warning")) {
		nfault_port.IDR = 0;
		faultPollGateDriver();
		for (unsigned ms = 1; ms <= 10000; ms++) {
			actual_current = ms <= 50 ? (int16_t)(2000 * (50 - ms) / 50) : ((ms / 20) % 2 ? 0 : 80);
			poll_ms(1);
		}
		assert_no_cut();
	} else if (!strcmp(scenario, "sine_warning")) {
		running = 0;
		stepper_sine = 1;
		actual_current = 0;
		nfault_port.IDR = 0;
		poll_ms(10000);
		assert_no_cut();
		assert(stepper_sine);
	} else if (!strcmp(scenario, "fault_stall")) {
		assert(argc == 3);
		actual_current = (int16_t)atoi(argv[2]);
		latch_fault();
	} else if (!strcmp(scenario, "recent_fault_stall")) {
		nfault_port.IDR = 0;
		poll_ms(8);
		nfault_port.IDR = 1;
		poll_ms(25);
		stall();
		assert_latched();
	} else if (!strcmp(scenario, "expired_fault_stall") || !strcmp(scenario, "age_saturation")) {
		nfault_port.IDR = 0;
		poll_ms(8);
		nfault_port.IDR = 1;
		poll_ms(!strcmp(scenario, "age_saturation") ? 65550 : 1000);
		stall();
		assert(!faultGateDriverFaultActive());
		assert(restart_calls == 1);
	} else if (!strcmp(scenario, "manual_zero_clear")) {
		latch_fault();
		adjusted_input = 0;
		gateDriverSleep();
		faultPollGateDriver();
		poll_ms(99);
		assert_latched();
		poll_ms(1);
		assert(!faultGateDriverFaultActive());
		assert(!running && !stepper_sine);
		assert(restart_calls == 0 && wake_calls == 0 && reset_calls == 0);
	} else if (!strcmp(scenario, "zero_blip_retains_latch")) {
		latch_fault();
		adjusted_input = 0;
		gateDriverSleep();
		faultPollGateDriver();
		poll_ms(60);
		adjusted_input = 500;
		poll_ms(1);
		adjusted_input = 0;
		faultPollGateDriver();
		poll_ms(60);
		assert_latched();
	} else if (!strcmp(scenario, "release_does_not_restart")) {
		latch_fault();
		nfault_port.IDR = 1;
		poll_ms(10000);
		assert_latched();
		assert(wake_calls == 0);
	} else if (!strcmp(scenario, "sleep_does_not_clear_at_throttle")) {
		latch_fault();
		gateDriverSleep();
		poll_ms(10000);
		assert_latched();
	} else if (!strcmp(scenario, "ordinary_stall_restarts")) {
		stall();
		assert(!faultGateDriverFaultActive());
		assert(restart_calls == 1);
	} else if (!strcmp(scenario, "commanded_stop_does_not_latch")) {
		input = 0;
		adjusted_input = 0;
		nfault_port.IDR = 0;
		faultPollGateDriver();
		stall();
		assert(!faultGateDriverFaultActive());
		assert(restart_calls == 0);
	} else if (!strcmp(scenario, "untrusted_pin_ignored")) {
		pin_trusted = 0;
		nfault_port.IDR = 0;
		poll_ms(1000);
		assert_no_cut();
		stall();
		assert(!faultGateDriverFaultActive());
		assert(restart_calls == 1);
	} else if (!strcmp(scenario, "warning_log_upgrades_to_error")) {
		nfault_port.IDR = 0;
		poll_ms(20);
		fault_id_t cause = FAULT_NONE;
		assert(faultGateDriverConsumeLog(&cause) == FAULT_GD_LOG_WARNING);
		assert(cause == FAULT_GD_UNKNOWN);
		stall();
		assert_latched();
		assert(faultGateDriverConsumeLog(&cause) == FAULT_GD_LOG_ERROR);
		assert(cause == FAULT_GD_UNKNOWN);
		poll_ms(1000);
		assert(faultGateDriverConsumeLog(&cause) == FAULT_GD_LOG_NONE);
		assert(cause == FAULT_NONE);
	} else if (!strcmp(scenario, "prop_brake_cannot_keep_latch_awake")) {
		prop_brake_active = 1;
		latch_fault();
		faultPollGateDriver();
		assert_latched();
		assert(!awake);
	} else {
		assert(!"unknown scenario");
	}
	return 0;
}
