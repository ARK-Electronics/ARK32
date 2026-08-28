/*
 * faults.c - extracted from main/control_loop (behavior-neutral)
 */

#include "faults.h"

#include "main.h"
#include "common.h"
#include "motor_runtime.h"
#include "phaseouts.h"
#include "comparator.h"
#include "peripherals.h"
#include "functions.h"
#include "eeprom.h"
#include "signal.h"
#include "commutation.h"
#include "bemf_zc.h"
#include "targets.h"
#include "esc_state.h"
#include "gate_driver.h"
#include "debug_uart.h"
#include "IO.h"
#include "sounds.h"
/* commutation_interval for stall debug line */
#include "motor_runtime.h"

#ifdef USE_RGB_LED
extern void setIndividualRGBLed(uint8_t, uint8_t, uint8_t);
#endif

extern volatile uint16_t zero_input_count;
extern volatile uint32_t dma_buffer[64];
extern void resetInputCaptureTimer(void);

/* See the comments on the declarations in faults.h. */
volatile uint32_t fault_stall_trips = 0;
volatile uint8_t fault_run_established = 0;

/* Acquisition-rail early-desync count (see faultNoteEarlyDesync). */
static uint8_t acq_fail_desyncs;

/* "Start resisted" counter (see the declaration in faults.h). */
volatile uint8_t fault_acq_resist_events;

#if defined(MCU_G431)
/*
 * ARK 12S CAN acquisition grace budget (see faultUpdateBemfTimeoutPolicy).
 * Sized above one startup-matrix start segment (3.0 s of continuous
 * throttle) so a tier that never acquires is still measured, while any
 * genuinely locked rotor latches once the budget runs out. Throttle
 * returns to zero between starts, which rearms the full budget.
 * Clocked in faultDesyncEpisodeTick1kHz.
 */
#	define ACQ_GRACE_MS_MAX 4000u
/* Above this, acquisition should be near instant; a jam here is the
 * dangerous case, so it gets no grace at all. Well above the 447 seen at
 * the matrix's top 20% tier. */
#	define ACQ_GRACE_MAX_INPUT 1000
static uint16_t acq_grace_ms;
#endif

#if defined(USE_DRV_NFAULT) || defined(USE_DRV8328_NFAULT)
#	define FAULT_HAS_DRV_NFAULT 1
#else
#	define FAULT_HAS_DRV_NFAULT 0
#endif

#if defined(USE_DRV_NFAULT)
/*
 * DRV8350H (ARK 12S CAN). nFAULT is an OR of VDS OCP / UVLO / OTW / OTSD /
 * GDF with no SPI status. Do not cut PWM on the falling edge: OTW keeps
 * the drivers active, and VDS auto-retries in ~8 ms. Classify from pin
 * duration + whether the bridge is still conducting *while drive is
 * commanded*. A throttle-idle current drop looks like Hi-Z on the shunt.
 *
 * VDS tRETRY is ~8 ms; GD_CLASSIFY_MS is 12 ms (1.5×). Back-to-back retries
 * into a persistent short leave nFAULT low with only microsecond-scale
 * highs between pulses. A free-running main-loop poll can easily never
 * observe a release, so GD_RETRY_BUDGET may never arm. The held path
 * (CLASSIFY/WARN → Hi-Z) is the real backstop; the pulse count is
 * opportunistic. Host twin: hwci/hwci/gd_nfault_model.py.
 */
#	define GD_CLASSIFY_MS 12u
#	define GD_UVLO_CV 800u
/* Smoothed centiamps (actual_current). 0.50 A is ~6 raw LSB on G431
 * (MILLIVOLT_PER_AMP 10, CURRENT_OFFSET 0) — conduction vs noise, not a
 * "motor is loaded" floor. Bench: capture the actual_current distribution
 * at warm idle AND at the lowest nonzero throttle the ESC is commanded at.
 * If either straddles 50 cA, raise this — a barely-spinning motor that
 * never holds 40 ms consecutive live hits GD_WARN_CEIL_MS and false-drops. */
#	define GD_LIVE_CA_MIN 50
/* MCU *die* via __LL_ADC_CALC_TEMPERATURE, not the DRV (OTW ~150 C /
 * OTSD ~175 C). Log label only — never pick HIZ vs latch from this. */
#	define GD_THERMAL_C 100
#	define GD_RETRY_WINDOW_MS 100u
#	define GD_RETRY_BUDGET 8u
#	define GD_RETRY_LOG_MS 200u
#	define GD_OTW_LOG_MS 200u
/* actual_current is a 50-sample / ~1 kHz boxcar (~50 ms). Dwell past that
 * so OTW + re-throttle is not Hi-Z'd on stale idle current. */
#	define GD_DEAD_DWELL_MS 80u
/* Consecutive current_live under drive before we trust OTW (no ceiling). */
#	define GD_OTW_CONFIRM_MS 40u
/* Pin held + drive commanded, never confirmed live → Hi-Z. Closes a
 * starvable dwell (one sample ≥ LIVE_CA_MIN resets the 80 ms window).
 * Accumulates commanded-held time across idle blips (idle does not
 * restart the 250 ms). */
#	define GD_WARN_CEIL_MS 250u
/* Zero-throttle ENABLE tRST while nFAULT is still held (GDF).
 * Consecutive failed pulses; pin-high recovery resets the count. */
#	define GD_HIZ_RST_MS 2000u
#	define GD_RESUME_BUDGET 3u

enum {
	GD_NF_IDLE = 0,
	GD_NF_CLASSIFY,
	GD_NF_WARN,
	GD_NF_HIZ,
	GD_NF_LATCH,
};

static uint8_t gd_state;
static fault_id_t gd_cause;
static uint16_t gd_t0;
static int16_t gd_snap_ca;
static uint8_t gd_retry_count;
static uint16_t gd_retry_window_t0;
static uint16_t gd_retry_log_t0;
static uint8_t gd_retry_logged;
static uint16_t gd_otw_log_t0;
static uint8_t gd_otw_logged;
static uint8_t gd_dead_arm;
static uint16_t gd_dead_t0;
static uint8_t gd_live_arm;
static uint16_t gd_live_t0;
static uint8_t gd_cmd_arm;
static uint16_t gd_cmd_t0;
static uint16_t gd_cmd_held_ms;
static uint8_t gd_otw_confirmed;
static uint8_t gd_resume_count;
static uint8_t gd_log_level;
static fault_id_t gd_log_cause;
static volatile uint16_t gd_ms;

static void gd_queue_log(uint8_t level, fault_id_t cause)
{
	/* Upgrade only: same-level must not clobber (ERROR UVLO → ERROR UNKNOWN). */
	if (level > gd_log_level) {
		gd_log_level = level;
		gd_log_cause = cause;
	}
}

/* Wrap-safe rate limit. First event always logs (*have_logged == 0); no
 * t0==0 sentinel, so a gd_ms wrap cannot look like "never logged". */
static uint8_t gd_rate_due(uint16_t now, uint8_t *have_logged, uint16_t *t0, uint16_t period)
{
	if (!*have_logged || (uint16_t)(now - *t0) >= period) {
		*have_logged = 1;
		*t0 = now;
		return 1;
	}
	return 0;
}

#	ifdef USE_DEBUG_UART
static uint8_t gd_dbg_event(fault_id_t cause, uint8_t warning)
{
	if (warning) {
		return (cause == FAULT_GD_OCP) ? DBG_EVT_NFAULT_RETRY : DBG_EVT_NFAULT_OTW;
	}
	switch (cause) {
		case FAULT_GD_UVLO:
			return DBG_EVT_NFAULT_UVLO;
		case FAULT_GD_OCP:
			return DBG_EVT_NFAULT_OCP;
		case FAULT_GD_OTSD:
			return DBG_EVT_NFAULT_OTSD;
		default:
			return DBG_EVT_NFAULT;
	}
}
#	endif

static void gd_hold_cut(void)
{
	allOff();
	maskPhaseInterrupts();
	SET_DUTY_CYCLE_ALL(0);
	running = 0;
	stepper_sine = 0;
}

/* Sine start drives the bridge with running==0. */
static uint8_t gd_drive_commanded(void)
{
	return (uint8_t)(adjusted_input != 0 && (running || stepper_sine));
}

static void gd_enter_latch(fault_id_t cause)
{
	gd_hold_cut();
	gd_cause = cause;
	gd_state = GD_NF_LATCH;
	gd_queue_log(FAULT_GD_LOG_ERROR, cause);
#	ifdef USE_DEBUG_UART
	debugUartLogEvent(gd_dbg_event(cause, 0));
#	endif
	if (escGetState() != ESC_FAULT_STUCK) {
		escToFaultStuck();
#	ifdef USE_RGB_LED
		setIndividualRGBLed(1, 0, 0);
#	endif
	}
}

static void gd_enter_dead(uint16_t now)
{
	gd_hold_cut();
	gd_t0 = now;
	gd_dead_arm = 0;
	gd_live_arm = 0;
	gd_cmd_arm = 0;
	gd_cmd_held_ms = 0;
	gd_state = GD_NF_HIZ;
	if (battery_voltage > 0u && battery_voltage < GD_UVLO_CV) {
		gd_cause = FAULT_GD_UVLO;
	} else if (degrees_celsius >= GD_THERMAL_C) {
		gd_cause = FAULT_GD_OTSD;
	} else {
		gd_cause = FAULT_GD_UNKNOWN;
	}
	gd_queue_log(FAULT_GD_LOG_ERROR, gd_cause);
#	ifdef USE_DEBUG_UART
	debugUartLogEvent(gd_dbg_event(gd_cause, 0));
#	endif
}

/* Elapsed commanded-held time. Idle clears cmd_arm so the next drive
 * poll does not add the idle gap; gd_cmd_held_ms is kept. */
static void gd_cmd_held_add(uint16_t now)
{
	if (!gd_cmd_arm) {
		gd_cmd_arm = 1;
		gd_cmd_t0 = now;
		return;
	}
	const uint16_t dt = (uint16_t)(now - gd_cmd_t0);
	gd_cmd_t0 = now;
	if (dt > (uint16_t)(0xffffu - gd_cmd_held_ms)) {
		gd_cmd_held_ms = 0xffffu;
	} else {
		gd_cmd_held_ms = (uint16_t)(gd_cmd_held_ms + dt);
	}
}

static void gd_enter_warn(uint16_t now)
{
	gd_cause = FAULT_GD_OTW;
	gd_state = GD_NF_WARN;
	if (gd_rate_due(now, &gd_otw_logged, &gd_otw_log_t0, GD_OTW_LOG_MS)) {
		gd_queue_log(FAULT_GD_LOG_WARNING, FAULT_GD_OTW);
#	ifdef USE_DEBUG_UART
		debugUartLogEvent(gd_dbg_event(FAULT_GD_OTW, 1));
#	endif
	}
}

static void gd_note_retry(uint16_t now)
{
	if ((uint16_t)(now - gd_retry_window_t0) > GD_RETRY_WINDOW_MS) {
		gd_retry_count = 0;
		gd_retry_window_t0 = now;
	}
	if (gd_retry_count < 255u) {
		gd_retry_count++;
	}
	if (gd_retry_count >= GD_RETRY_BUDGET) {
		gd_retry_count = 0;
		gd_enter_latch(FAULT_GD_OCP);
		return;
	}
	if (gd_rate_due(now, &gd_retry_logged, &gd_retry_log_t0, GD_RETRY_LOG_MS)) {
		gd_queue_log(FAULT_GD_LOG_WARNING, FAULT_GD_OCP);
#	ifdef USE_DEBUG_UART
		debugUartLogEvent(gd_dbg_event(FAULT_GD_OCP, 1));
#	endif
	}
}

#elif defined(USE_DRV8328_NFAULT)
/*
 * DRV8328 (ARK 4IN1). Every nFAULT disables the gate drivers (no OTW-only
 * report, no 8 ms VDS retry). VDS / OTSD / GDF stay latched until nSLEEP
 * is pulled low — sleep-on-idle already does that. Firmware: cut PWM, latch
 * stuck until zero throttle. Ignore the pin while asleep (nSLEEP UVLO).
 */
static uint8_t drv_nfault_latched;
static fault_id_t drv_nfault_cause;
#endif

const char *faultGateDriverCauseName(fault_id_t cause)
{
	switch (cause) {
		case FAULT_GD_UVLO:
			return "UVLO";
		case FAULT_GD_OCP:
			return "OCP";
		case FAULT_GD_OTW:
			return "OTW";
		case FAULT_GD_OTSD:
			return "OTSD";
		case FAULT_GD_UNKNOWN:
			return "nFAULT";
		default:
			return "";
	}
}

fault_id_t faultGateDriverCause(void)
{
#if defined(USE_DRV_NFAULT)
	if (gd_state != GD_NF_IDLE) {
		return gd_cause;
	}
#elif defined(USE_DRV8328_NFAULT)
	if (drv_nfault_latched) {
		return drv_nfault_cause;
	}
#endif
	return FAULT_NONE;
}

uint8_t faultGateDriverConsumeLog(fault_id_t *cause)
{
#if defined(USE_DRV_NFAULT)
	const uint8_t level = gd_log_level;
	if (cause) {
		*cause = (level != FAULT_GD_LOG_NONE) ? gd_log_cause : FAULT_NONE;
	}
	gd_log_level = FAULT_GD_LOG_NONE;
	gd_log_cause = FAULT_NONE;
	return level;
#else
	if (cause) {
		*cause = FAULT_NONE;
	}
	return FAULT_GD_LOG_NONE;
#endif
}

uint32_t faultErrorCount(void)
{
	return desync_happened + fault_stall_trips;
}

void faultErrorCountReset(void)
{
	/* Per-arm cycle (DSDL: error_count resets when the motor restarts). */
	fault_stall_trips = 0;
	desync_happened = 0;
	acq_fail_desyncs = 0;
	/* Lifetimes must match the counters this latch gates. */
	fault_run_established = 0;
}

uint8_t faultGateDriverFaultActive(void)
{
#if defined(USE_DRV_NFAULT)
	return (uint8_t)(gd_state == GD_NF_HIZ || gd_state == GD_NF_LATCH);
#elif defined(USE_DRV8328_NFAULT)
	return drv_nfault_latched;
#else
	return 0;
#endif
}

uint8_t faultGateDriverWarningActive(void)
{
#if defined(USE_DRV_NFAULT)
	return (uint8_t)(gd_state == GD_NF_WARN);
#else
	return 0;
#endif
}

uint8_t faultGateDriverKeepAwake(void)
{
#if defined(USE_DRV_NFAULT)
	/* HIZ auto-recover needs ENABLE high to see the pin release, and to
	 * tRST a latched GDF at zero throttle. LATCH lets sleep-on-idle reset. */
	return (uint8_t)(gd_state == GD_NF_HIZ);
#else
	return 0;
#endif
}

void faultGateDriverTick1kHz(void)
{
#if defined(USE_DRV_NFAULT)
	gd_ms++;
#endif
}

void faultPollGateDriver(void)
{
#if defined(USE_DRV_NFAULT)
	if (!gateDriverIsAwake()) {
		if (adjusted_input == 0 && gd_state != GD_NF_IDLE) {
			gd_state = GD_NF_IDLE;
			gd_cause = FAULT_NONE;
			gd_dead_arm = 0;
			gd_live_arm = 0;
			gd_cmd_arm = 0;
			/* gd_resume_count is consecutive-failed tRST. Sleep-idle
			 * does not observe pin-high, so it does not reset. */
		}
		return;
	}

	gateDriverNfaultGraceTick();
	if (!gateDriverNfaultPinTrusted()) {
		return;
	}

	const uint8_t pin_low = (uint8_t)((NFAULT_PORT->IDR & NFAULT_PIN) == 0u);
	const uint16_t now = gd_ms;
	const uint8_t drive_on = gd_drive_commanded();
	const uint8_t current_live = (uint8_t)(actual_current >= GD_LIVE_CA_MIN);

	switch (gd_state) {
		case GD_NF_IDLE:
			if (pin_low) {
				gd_snap_ca = actual_current;
				gd_t0 = now;
				gd_dead_arm = 0;
				gd_live_arm = 0;
				gd_cmd_arm = 0;
				gd_cmd_held_ms = 0;
				gd_otw_confirmed = 0;
				gd_cause = FAULT_NONE;
				gd_state = GD_NF_CLASSIFY;
			}
			break;

		case GD_NF_CLASSIFY:
			if (!pin_low) {
				gd_state = GD_NF_IDLE;
				gd_note_retry(now);
				break;
			}
			if ((uint16_t)(now - gd_t0) < GD_CLASSIFY_MS) {
				break;
			}
			if (drive_on && gd_snap_ca >= GD_LIVE_CA_MIN && actual_current >= (int16_t)(gd_snap_ca >> 1)) {
				/* Falling-edge snap and this sample are 12 ms and two
				 * independent boxcar updates apart. Both live and not
				 * collapsed vs snap is enough to confirm; the 40 ms
				 * consecutive dwell is for the unconfirmed fallthrough. */
				gd_otw_confirmed = 1;
				gd_enter_warn(now);
				break;
			}
			if (drive_on && !current_live) {
				gd_enter_dead(now);
				break;
			}
			gd_enter_warn(now);
			break;

		case GD_NF_WARN:
			if (!pin_low) {
				gd_state = GD_NF_IDLE;
				gd_cause = FAULT_NONE;
				gd_dead_arm = 0;
				gd_live_arm = 0;
				gd_cmd_arm = 0;
				gd_cmd_held_ms = 0;
				gd_otw_confirmed = 0;
				break;
			}
			if (!drive_on) {
				gd_dead_arm = 0;
				gd_live_arm = 0;
				gd_cmd_arm = 0; /* pause the ceiling clock; keep held_ms */
				break;
			}
			gd_cmd_held_add(now);
			if (current_live) {
				gd_dead_arm = 0;
				if (!gd_live_arm) {
					gd_live_arm = 1;
					gd_live_t0 = now;
				} else if ((uint16_t)(now - gd_live_t0) >= GD_OTW_CONFIRM_MS) {
					gd_otw_confirmed = 1;
				}
			} else {
				gd_live_arm = 0;
				if (!gd_dead_arm) {
					gd_dead_arm = 1;
					gd_dead_t0 = now;
				} else if ((uint16_t)(now - gd_dead_t0) >= GD_DEAD_DWELL_MS) {
					gd_enter_dead(now);
					break;
				}
			}
			if (!gd_otw_confirmed && gd_cmd_held_ms >= GD_WARN_CEIL_MS) {
				gd_enter_dead(now);
			}
			break;

		case GD_NF_HIZ:
			gd_hold_cut();
			if (!pin_low) {
				gd_resume_count = 0; /* pin-high: DRV recovered */
				gd_state = GD_NF_IDLE;
				gd_cause = FAULT_NONE;
				break;
			}
			/* Stay while nFAULT is held. Zero throttle must not return
			 * to IDLE — that re-queued ERROR every classify window
			 * (prop-brake keeps ENABLE high with adjusted_input == 0). */
			if (battery_voltage > 0u && battery_voltage < GD_UVLO_CV) {
				break;
			}
			if (adjusted_input != 0) {
				break;
			}
			if ((uint16_t)(now - gd_t0) >= GD_HIZ_RST_MS) {
				gd_t0 = now;
				if (gd_resume_count >= GD_RESUME_BUDGET) {
					gd_enter_latch(gd_cause == FAULT_NONE ? FAULT_GD_UNKNOWN : gd_cause);
				} else {
					gd_resume_count++;
					gateDriverFaultResetPulse();
				}
			}
			break;

		case GD_NF_LATCH:
			gd_hold_cut();
			if (!pin_low && adjusted_input == 0) {
				gd_resume_count = 0;
				gd_state = GD_NF_IDLE;
				gd_cause = FAULT_NONE;
			}
			/* ENABLE tRST is sleep-on-idle (gateDriverPoll), not a
			 * pulse from this state — KeepAwake is false here. */
			break;

		default:
			gd_state = GD_NF_IDLE;
			gd_cause = FAULT_NONE;
			break;
	}

#elif defined(USE_DRV8328_NFAULT)
	if (!gateDriverIsAwake()) {
		if (drv_nfault_latched && adjusted_input == 0) {
			drv_nfault_latched = 0;
			drv_nfault_cause = FAULT_NONE;
		}
		return;
	}

	gateDriverNfaultGraceTick();
	if (!gateDriverNfaultPinTrusted()) {
		return;
	}

	if ((NFAULT_PORT->IDR & NFAULT_PIN) == 0u) {
		if (!drv_nfault_latched) {
			drv_nfault_cause = (battery_voltage > 0u && battery_voltage < 800u) ? FAULT_GD_UVLO : FAULT_GD_UNKNOWN;
			drv_nfault_latched = 1;
		}
		allOff();
		maskPhaseInterrupts();
		SET_DUTY_CYCLE_ALL(0);
		running = 0;
		stepper_sine = 0;
		if (escGetState() != ESC_FAULT_STUCK) {
			escToFaultStuck();
		}
	} else if (drv_nfault_latched && adjusted_input == 0) {
		drv_nfault_latched = 0;
		drv_nfault_cause = FAULT_NONE;
	}
#endif
}

uint8_t faultHandleStuckRotorIfNeeded(void)
{
#ifndef BRUSHED_MODE
#	if FAULT_HAS_DRV_NFAULT
	/* Gate-driver Hi-Z / latch: do not map throttle onto the bridge. */
	if (faultGateDriverFaultActive()) {
		allOff();
		maskPhaseInterrupts();
		return 1;
	}
#	endif
	if ((bemf_timeout_happened > bemf_timeout) && eepromBuffer.stuck_rotor_protection) {
		allOff();
		maskPhaseInterrupts();
		/* Log once on entry — this path runs every main-loop tick while latched. */
		if (escGetState() != ESC_FAULT_STUCK) {
			debugUartLogEvent(DBG_EVT_STUCK);
		}
		escToFaultStuck();
#	ifdef USE_RGB_LED
		setIndividualRGBLed(1, 0, 0);
#	endif
		return 1;
	}
#endif
	return 0;
}

/* RAM-resident: called from tenKhzRoutine every 50 us on F051. */
RAM_FUNC void faultSignalTimeoutTick(void)
{
#if defined(FIXED_DUTY_MODE) || defined(FIXED_SPEED_MODE)
	if (getInputPinState()) {
		signaltimeout++;
		if (signaltimeout > LOOP_FREQUENCY_HZ) {
			bootSoundMarkSignalLost();
			NVIC_SystemReset();
		}
	} else {
		signaltimeout = 0;
	}
#else
	signaltimeout++;
#endif
}

void faultPollSignalTimeout(void)
{
	if (signaltimeout > (LOOP_FREQUENCY_HZ >> 1)) { // half second timeout when armed;
		if (escIsArmed()) {
			allOff();
			debugUartLogEvent(DBG_EVT_SIGNAL);
			debugUartService(); /* flush before reset */
			escToFaultSignal();
			zero_input_count = 0;
			SET_DUTY_CYCLE_ALL(0);
			resetInputCaptureTimer();
			for (int i = 0; i < 64; i++) {
				dma_buffer[i] = 0;
			}
			bootSoundMarkSignalLost();
			NVIC_SystemReset();
		}
		if (signaltimeout > LOOP_FREQUENCY_HZ << 1) { // 2 second when not armed
			allOff();
			debugUartLogEvent(DBG_EVT_SIGNAL);
			debugUartService();
			escToFaultSignal();
			zero_input_count = 0;
			SET_DUTY_CYCLE_ALL(0);
			resetInputCaptureTimer();
			for (int i = 0; i < 64; i++) {
				dma_buffer[i] = 0;
			}
			bootSoundMarkSignalLost();
			NVIC_SystemReset();
		}
	}
}

void faultUpdateBemfTimeoutPolicy(void)
{
#ifndef BRUSHED_MODE
	if ((zero_crosses > 1000) || (adjusted_input == 0)) {
		bemf_timeout_happened = 0;
	}
	if (adjusted_input == 0) {
		// Zero throttle is pilot intervention: clear the episode rail the
		// same way the legacy latch clears. Deliberately NOT on
		// zero_crosses > 1000 - a bad-tune cycle that respins fast between
		// desyncs must keep accumulating (that reset defeating the stuck
		// latch is one of the gaps this rail exists to close).
		//
		// Also clear the acquisition-rail partial batch: otherwise 19 early
		// desyncs, a pilot cut, then one more early desync on the next blip
		// would fire a JUMP-sized charge as if the cut never happened.
		// Match episode-bucket pilot semantics.
		desync_episode_bucket = 0;
		desync_restart_holdoff_ms = 0;
		acq_fail_desyncs = 0;
	}
#	if defined(MCU_G431)
	/*
	 * ARK 12S CAN free-run restarts thrash in the acquisition window
	 * (zero_crosses reset each stall) while still commanded throttle.
	 * Accumulating bemf_timeout_happened across those kicks latched
	 * stuck_rotor after a few start* cycles on the stand. F051 4IN1 does
	 * not show this with the same shared rails. Forgive acquisition-only
	 * stalls; keep the counter once the loop is established (zc > 100).
	 *
	 * Time-bounded on purpose. A truly locked rotor never reaches
	 * zc >= 100, so an unbounded clear disables stuck_rotor entirely
	 * during acquisition -- and, because ESC_STUCK_LATCH is itself carried
	 * in bemf_timeout_happened, silently wipes an already-latched stuck
	 * fault too (bench 2026-08: 16 FAULT_STUCK entries that all recovered
	 * within one tick). A jammed 12S prop would re-kick forever.
	 *
	 * A plain throttle gate does not work either: it has to sit above the
	 * highest tier being characterized, and the startup matrix reaches
	 * input 447 at its 20% tier, so any threshold low enough to be useful
	 * lands mid-sweep. Bound the grace in time instead -- start attempts
	 * are short and separated by zero throttle, a jam is continuous.
	 */
	if (zero_crosses < 100 && acq_grace_ms < ACQ_GRACE_MS_MAX && adjusted_input < ACQ_GRACE_MAX_INPUT) {
		bemf_timeout_happened = 0;
	}
#	endif
	if (zero_crosses > 100 && adjusted_input < 200) {
		bemf_timeout_happened = 0;
	}
	if (eepromBuffer.use_sine_start && adjusted_input < 160) {
		bemf_timeout_happened = 0;
	}

#	if defined(MCU_G431)
	/*
		 * ARK 12S CAN (G4): free-run high-KV crawl lives above DShot~150 while
		 * BEMF is still weak. F051 gets clean edges from COMP hysteresis there;
		 * until G4 hysteresis settles, keep the soft stall budget (100) out to
		 * input 400 so restart thrash does not latch stuck_rotor in 10 kicks.
		 * Established high-throttle stuck detection still uses 10.
		 */
	if (adjusted_input < 400) {
		bemf_timeout = 100;
	} else {
		bemf_timeout = 10;
	}
#	else
	if (adjusted_input < 150) { // startup duty cycle should be low enough to not burn motor
		bemf_timeout = 100;
	} else {
		bemf_timeout = 10;
	}
#	endif
#endif
}

/*
 * Cross-episode desync protection. Per-episode rails (blind-step cap, miss
 * bucket, demag-late power cut) bound a single event; this bucket bounds a
 * *repeating* restart→desync cycle from a bad tune / wrong prop match.
 *
 * Charge rates are tuned so 3-5 hard episodes latch within ~1-2 s of cycling,
 * while a couple of honest in-flight desyncs drain out in a few seconds of
 * healthy closed-loop (stall: 4 episodes at 12; jump: 5 at 8).
 *
 * The FIRST episode must restart immediately - an honest in-flight desync
 * that coasts even 100 ms is a dropped motor on a quad - so restart holdoff
 * only arms once the bucket shows repetition (>= MIN_BUCKET, i.e. from the
 * second episode on) and then grows so the FETs spend most of each later
 * cycle cooling instead of immediately re-entering the wrong-phase spike.
 * Stall path: ep2 300 ms, ep3 500 ms, ep4 latch. Jump path: ep2 100 ms,
 * ep3 300 ms, ep4 500 ms, ep5 latch.
 */
#define DESYNC_EPISODE_LIMIT 40
#define DESYNC_EPISODE_CHARGE_JUMP 8
#define DESYNC_EPISODE_CHARGE_STALL 12
#define DESYNC_EPISODE_DRAIN_MS 100  /* -1 charge per this many ms of healthy CL */
#define DESYNC_BACKOFF_MIN_BUCKET 16 /* below this (first episode): no holdoff */
#define DESYNC_BACKOFF_BASE_MS 100
#define DESYNC_BACKOFF_STEP_MS 25
#define DESYNC_BACKOFF_MAX_MS 500
/*
 * Acquisition rail (see faultNoteEarlyDesync in faults.h).
 *
 * 20 early desyncs with no successful acquisition in between buy one
 * JUMP-sized charge. Deliberately conservative: a healthy hard start spends
 * 2-4 rough desyncs acquiring, so 20 is unambiguously abnormal, while at the
 * ~20/s rate a genuinely stuck loop produces it still charges once a second.
 * The point of this rail is to make the failure VISIBLE to the escalation
 * machinery (bucket, holdoff, latch) and to the fault_acq_resist_events
 * counter, not to be the fastest path to the latch. It deliberately does
 * nothing to the ramp: a start that cannot complete is at least as likely
 * mechanically resisted (grass, debris) as mis-tuned, and duty is
 * near-always slewing during a start, so there is no signal here that
 * separates the two - see faultDesyncEpisodeCharge. Established-run rails
 * (blind/grind/stall) still need zero_crosses > 100 and only engage once
 * the loop actually gets past that gate.
 *
 * This constant is a starting point from SITL and wants bench calibration
 * against real hard starts before release.
 *
 * ACQ_FAIL_CLEAR_ZC is deliberately well above the 100 that arms blind
 * stepping: the count must only be forgiven by an acquisition that actually
 * held, not by one that reached the gate and immediately lost sync again.
 */
#define ACQ_FAIL_DESYNC_LIMIT 20
#define ACQ_FAIL_CLEAR_ZC 500

/*
 * The blind-grind rail that used to live here (sample the blind-step count
 * every 100 ms, force a restart on one hot window, hold a power cut for
 * 250 ms) is gone. It existed because blind steps reset INTERVAL_TIMER and so
 * hid a grind from the stall rail below; bemf_zc.c now measures dead-reckoning
 * time directly in zc_blind_ticks, which makes a grind visible to that one
 * rail without a second rate detector, three more tuned constants, and a
 * restart on a single sampling window. Duty during a grind is handled by the
 * continuous authority fade in control_loop.c rather than a held cut.
 */

static uint8_t desync_episode_drain_ms;

static void desync_episode_apply_backoff(void)
{
	if (desync_episode_bucket < DESYNC_BACKOFF_MIN_BUCKET) {
		return; // first episode restarts immediately (legacy behavior)
	}
	uint16_t hold =
		(uint16_t)(DESYNC_BACKOFF_BASE_MS + (uint16_t)(desync_episode_bucket - DESYNC_BACKOFF_MIN_BUCKET) * DESYNC_BACKOFF_STEP_MS);
	if (hold > DESYNC_BACKOFF_MAX_MS) {
		hold = DESYNC_BACKOFF_MAX_MS;
	}
	if (hold > desync_restart_holdoff_ms) {
		desync_restart_holdoff_ms = hold;
	}
}

void faultDesyncEpisodeCharge(desync_episode_kind_t kind)
{
#ifndef BRUSHED_MODE
	uint8_t inc = DESYNC_EPISODE_CHARGE_STALL;
	if (kind == DESYNC_EPISODE_JUMP || kind == DESYNC_EPISODE_ACQ_FAIL) {
		/* The acquisition rail has already required ACQ_FAIL_DESYNC_LIMIT
		 * events before getting here, so one charge per batch is the same
		 * weight as a single established-run jump. */
		inc = DESYNC_EPISODE_CHARGE_JUMP;
	}
	if ((uint16_t)desync_episode_bucket + inc > 255) {
		desync_episode_bucket = 255;
	} else {
		desync_episode_bucket = (uint8_t)(desync_episode_bucket + inc);
	}
	desync_episode_drain_ms = 0;
	desync_episode_apply_backoff();

	// NO PERSISTENT LEARNED STATE. An episode charges the escalation
	// machinery above (bucket -> holdoff -> latch) and nothing else: the
	// configured ramp, and every other tuning parameter, is left exactly as
	// loadEEpromSettings set it. This is a deliberate reversal of the
	// learned ramp back-off that used to live here (each episode halved
	// every regime's duty step for the rest of the power cycle, flooring at
	// fine 0.1%/ms).
	//
	// Why it is gone, on a multirotor specifically: the halve is per-ESC
	// state that the flight controller cannot see. The mixer assumes
	// symmetric actuators, so one ESC that has learned a slower ramp than
	// its three siblings is an asymmetric vehicle with no way to report it.
	// That is not a hypothetical - it crashed a vehicle taking off after a
	// ground spool in long grass, where prop obstruction at STEADY duty
	// dragged an established run down and the episode was misread as "ramp
	// too fast". On the production tune (max_ramp 20 -> all regimes 2) one
	// halve already IS the fine-rate floor: a silent 20x slew-authority cut.
	//
	// An attribution gate was tried first (a slew witness requiring the
	// limiter to be binding on rising demand before halving) and it does
	// narrow the misattribution, but it keeps the architecture: still
	// per-ESC learned divergence, still invisible to the FC, still one
	// wrong attribution away from the same asymmetry. The line drawn
	// instead is TRANSIENT SELF-RECOVERING RESPONSES ONLY - the bucket,
	// holdoff and latch all converge back to configured settings, so an
	// ESC either behaves as configured or stops. A too-fast ramp is a
	// tuning defect and gets fixed by retuning, not by each ESC quietly
	// deciding its own limit mid-flight. This also keeps us on upstream
	// AM32's fixed-ramp semantics rather than diverging further.
	//
	// ACQ_FAIL episodes additionally bump fault_acq_resist_events, which is
	// a counter only - see faults.h for why that one has to survive the
	// self-healing.
	if (kind == DESYNC_EPISODE_ACQ_FAIL) {
		if (fault_acq_resist_events < 255) {
			fault_acq_resist_events++;
		}
	}

	if (desync_episode_bucket >= DESYNC_EPISODE_LIMIT) {
		allOff();
		maskPhaseInterrupts();
		if (escGetState() != ESC_FAULT_STUCK) {
			debugUartLogEvent(DBG_EVT_STUCK);
		}
		escToFaultStuck();
#	ifdef USE_RGB_LED
		setIndividualRGBLed(1, 0, 0);
#	endif
	}
#else
	(void)kind;
#endif
}

void faultNoteEarlyDesync(void)
{
#ifndef BRUSHED_MODE
	if (++acq_fail_desyncs < ACQ_FAIL_DESYNC_LIMIT) {
		return;
	}
	acq_fail_desyncs = 0;
	debugUartLogEvent(DBG_EVT_ACQ_DESYNC);
	faultDesyncEpisodeCharge(DESYNC_EPISODE_ACQ_FAIL);
#endif
}

void faultDesyncEpisodeTick1kHz(void)
{
#ifndef BRUSHED_MODE
	/* A loop that got solidly established did acquire, whatever roughness
	 * it passed through on the way - forgive the whole batch. */
	if (zero_crosses > ACQ_FAIL_CLEAR_ZC) {
		acq_fail_desyncs = 0;
	}
#	if defined(MCU_G431)
	/* Acquisition grace clock. Runs only while throttle is commanded and
	 * the loop has not established; pilot cut or a real acquire rearms it. */
	if (zero_crosses >= 100 || adjusted_input == 0) {
		acq_grace_ms = 0;
	} else if (acq_grace_ms < ACQ_GRACE_MS_MAX) {
		acq_grace_ms++;
	}
#	endif
	if (desync_restart_holdoff_ms > 0) {
		desync_restart_holdoff_ms--;
	}

	/* Drain only in established, trusted closed loop. bemf_timeout_happened
	 * is deliberately NOT in the gate: it stays nonzero until
	 * zero_crosses > 1000 (5-10 s at crawler rpm), which would block drain
	 * through exactly the healthy running that should earn it. */
	if (escInClosedLoop() && zc_blind_steps == 0 && zc_demag_run == 0 && desync_episode_bucket > 0) {
		if (desync_episode_drain_ms < 255) {
			desync_episode_drain_ms++;
		}
		if (desync_episode_drain_ms >= DESYNC_EPISODE_DRAIN_MS) {
			desync_episode_drain_ms = 0;
			desync_episode_bucket--;
		}
	} else {
		desync_episode_drain_ms = 0;
	}
#endif
}

uint8_t faultDesyncRestartHoldoffActive(void)
{
	return (uint8_t)(desync_restart_holdoff_ms > 0);
}

void faultHandleBemfIntervalStall(void)
{
	/* Six-step only (not sine soft-start). */
	if (INTERVAL_TIMER_COUNT > BEMF_STALL_TICKS && (escInOpenLoop() || escInClosedLoop())) {
		/* Commanded stop (input < 48 == escNoteStallOrDesync): duty already
		 * zero, running still set while BEMF dies — INTERVAL_TIMER expiry
		 * is expected, not a stall. Skip trip count / log / episode charge. */
		const uint8_t commanded_stop = (input < 48);

		maskPhaseInterrupts();
		if (!commanded_stop) {
			bemf_timeout_happened++;
			// Charge the episode rail only when this run was ESTABLISHED
			// before it died (the bad-tune restart->spool->desync cycle
			// always reaches closed loop first). A start attempt that never
			// got going is the legacy stuck-rotor rail's job, with its
			// throttle-scaled tolerance (bemf_timeout 100 below input 150) -
			// heavy props legitimately kick many times at low throttle, and
			// charging those latched the ESC on the 4th kick. This gate also
			// covers the blind/miss-limit handoff (bemf_zc kicks
			// INTERVAL_TIMER past BEMF_STALL_TICKS with comparator interrupts masked, so
			// this rail is guaranteed to run next pass): blind stepping only
			// arms at zero_crosses >= 100, so those episodes always charge.
			//
			// Reporting and escalation take DIFFERENT gates, for the same
			// reason as the jump check in runtimeProcessDesyncCheck():
			//   - fault_stall_trips feeds esc.Status.error_count, so it asks
			//     "did an established run fault this arm cycle?" -> latch,
			//     which survives the zero_crosses reset this rail performs.
			//   - the episode charge asks "is THIS trip an established-run
			//     failure or acquisition thrash?" -> live count. After the
			//     first trip the loop is back in acquisition and kicks
			//     repeatedly; charging those would fill the bucket in a few
			//     events and latch the ESC during its own recovery.
			if (fault_run_established) {
				/* Established run died this arm cycle. This is the ONLY
				 * place the stall rail is counted, and it is the
				 * aggregation point for the grind rail and the blind/
				 * miss-limit handoff, both of which reach the loop through
				 * this trip - see faultErrorCount(). */
				fault_stall_trips++;
#ifdef USE_DEBUG_UART
				/* Single UART line (LogEvent would print "fault: stall" twice). */
				debugUartPrintf("fault: stall zc=%lu bemf_to=%u/%u e_com=%lu\r\n", (unsigned long)zero_crosses,
						(unsigned)bemf_timeout_happened, (unsigned)bemf_timeout,
						(unsigned long)commutation_interval);
#endif
			}
			if (zero_crosses > 100) {
				faultDesyncEpisodeCharge(DESYNC_EPISODE_STALL_RAIL);
			}
		}
		if (escIsFault()) {
			/* Episode rail latched: do not re-enter startup. */
			zero_crosses = 0;
			bemfZcResetTrend();
			running = 0;
			return;
		}
		escNoteStallOrDesync(1);
		zero_crosses = 0;
		bemfZcResetTrend();
		if (commanded_stop || faultDesyncRestartHoldoffActive()) {
			/* Coast / commanded stop: do not zcfound re-enter. */
			running = 0;
			allOff();
			return;
		}
		zcfoundroutine();
	}
}
