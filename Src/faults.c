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

/*
 * DRV nFAULT is a single open-drain OR (VDS OCP, UVLO, OTW, GDF). Hardware
 * interface (DRV8350H / DRV8328 without SPI status) cannot report which bit
 * fired — classify from MCU ADC context at the rising edge of the latch.
 */
#if defined(USE_DRV_NFAULT) || defined(USE_DRV8328_NFAULT)
#	define FAULT_HAS_DRV_NFAULT 1
#endif

#if FAULT_HAS_DRV_NFAULT
/* Sticky until nFAULT releases; keeps FAULT_STUCK if bemf latch is cleared
 * by the zero-throttle path while the DRV is still asserting. */
static uint8_t drv_nfault_latched;
static fault_id_t drv_nfault_cause;
#	if defined(USE_DRV_ENABLE)
/* Rate-limit ENABLE recovery pulses (~main-loop iterations). */
static uint16_t drv_enable_retry_div;
#	endif

/*
 * Best-effort guess: the DRV does not tell us which OR-term pulled nFAULT.
 * Use MCU ADC at the latch edge (call BEFORE clearing running/duty context).
 *
 * Priority: UVLO → OTW → OCP → unknown (VDS/GDF/etc.).
 * Units: battery_voltage 10 mV, actual_current 10 mA, degrees_celsius °C.
 *
 * Thresholds are deliberately coarse — a wrong-but-specific label is more
 * useful on the bench/FC than a generic "nFAULT". They are not datasheet
 * proofs of the DRV comparator that fired.
 */
static fault_id_t fault_classify_drv_nfault(void)
{
	const uint16_t v_cv = battery_voltage;
	const int16_t i_ca = actual_current;
	const int16_t t_c = degrees_celsius;

	/* ~8 V: below useful pack voltage for these ESCs; near DRV VM UVLO. */
	if (v_cv > 0u && v_cv < 800u) {
		return FAULT_GD_UVLO;
	}
	/* FET NTC / motor path hot — OTW class. */
	if (t_c >= 100) {
		return FAULT_GD_OTW;
	}
	/* High current, or was driving with meaningful duty when the pin fell. */
	if (i_ca >= 2000 || (running != 0 && duty_cycle > 200u)) {
		return FAULT_GD_OCP;
	}
	return FAULT_GD_UNKNOWN;
}

#	ifdef USE_DEBUG_UART
static uint8_t fault_drv_nfault_dbg_event(fault_id_t cause)
{
	switch (cause) {
		case FAULT_GD_UVLO:
			return DBG_EVT_NFAULT_UVLO;
		case FAULT_GD_OCP:
			return DBG_EVT_NFAULT_OCP;
		case FAULT_GD_OTW:
			return DBG_EVT_NFAULT_OTW;
		default:
			return DBG_EVT_NFAULT;
	}
}
#	endif
#endif /* FAULT_HAS_DRV_NFAULT */

const char *faultGateDriverCauseName(fault_id_t cause)
{
	switch (cause) {
		case FAULT_GD_UVLO:
			return "UVLO";
		case FAULT_GD_OCP:
			return "OCP";
		case FAULT_GD_OTW:
			return "OTW";
		case FAULT_GD_UNKNOWN:
			return "nFAULT";
		default:
			return "";
	}
}

fault_id_t faultGateDriverCause(void)
{
#if FAULT_HAS_DRV_NFAULT
	if (drv_nfault_latched || ((NFAULT_PORT->IDR & NFAULT_PIN) == 0u)) {
		return drv_nfault_cause;
	}
#endif
	return FAULT_NONE;
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
#if FAULT_HAS_DRV_NFAULT
	const uint8_t pin_ok = (NFAULT_PORT->IDR & NFAULT_PIN) != 0u;
	return (uint8_t)(!pin_ok || drv_nfault_latched);
#else
	return 0;
#endif
}

void faultPollGateDriver(void)
{
#if FAULT_HAS_DRV_NFAULT
	/*
	 * While asleep ENABLE is held low (gate_driver sleep). That already
	 * satisfies DRV8350H t_RST for latched VDS/GDF faults — but we used to
	 * return here without ever clearing drv_nfault_latched, so FAULT_STUCK
	 * survived zero throttle + VM restore until reboot. Clear the software
	 * sticky at zero demand while asleep; nFAULT is not meaningful with
	 * ENABLE low, so do not re-assert from the pin in that state.
	 */
	if (!gateDriverIsAwake()) {
		if (drv_nfault_latched && adjusted_input == 0) {
			drv_nfault_latched = 0;
			drv_nfault_cause = FAULT_NONE;
#	if defined(USE_DRV_ENABLE)
			drv_enable_retry_div = 0;
#	endif
		}
		return;
	}

	/* Active low (open-drain). High = healthy. */
	const uint8_t pin_ok = (NFAULT_PORT->IDR & NFAULT_PIN) != 0u;

	if (!pin_ok) {
		if (!drv_nfault_latched) {
			/* Classify while running/duty still reflect the trip context. */
			drv_nfault_cause = fault_classify_drv_nfault();
			drv_nfault_latched = 1;
#	ifdef USE_DEBUG_UART
			debugUartLogEvent(fault_drv_nfault_dbg_event(drv_nfault_cause));
#	endif
		}
		allOff();
		maskPhaseInterrupts();
		SET_DUTY_CYCLE_ALL(0);
		running = 0;
		stepper_sine = 0;
		if (escGetState() != ESC_FAULT_STUCK) {
			/* Reuses stuck latch: reconcile holds FAULT_STUCK via
			 * bemf_timeout_happened == ESC_STUCK_LATCH until zero
			 * throttle clears it — same pilot-clear semantics. */
			escToFaultStuck();
#	ifdef USE_RGB_LED
			setIndividualRGBLed(1, 0, 0);
#	endif
		}
#	if defined(USE_DRV_ENABLE)
		/*
		 * DRV8350H: latched VDS / gate faults stay asserted until
		 * ENABLE is driven low for t_RST then high again. Only retry
		 * at zero demand so we never re-arm into a short.
		 */
		if (adjusted_input == 0) {
			if (++drv_enable_retry_div >= 2000u) {
				drv_enable_retry_div = 0;
				gateDriverFaultResetPulse();
			}
		} else {
			drv_enable_retry_div = 0;
		}
#	endif
	} else if (drv_nfault_latched && adjusted_input == 0) {
		/* Pin recovered and pilot at zero: drop sticky so arm can proceed. */
		drv_nfault_latched = 0;
		drv_nfault_cause = FAULT_NONE;
#	if defined(USE_DRV_ENABLE)
		drv_enable_retry_div = 0;
#	endif
	}
#endif
}

uint8_t faultHandleStuckRotorIfNeeded(void)
{
#ifndef BRUSHED_MODE
#	if FAULT_HAS_DRV_NFAULT
	/* Gate-driver trip: do not map throttle back onto the bridge while
	 * nFAULT is low or still latched (cause in faultGateDriverCause()). */
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
