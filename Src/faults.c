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
#include "IO.h"
#include "sounds.h"

#ifdef USE_RGB_LED
extern void setIndividualRGBLed(uint8_t, uint8_t, uint8_t);
#endif

extern volatile uint16_t zero_input_count;
extern volatile uint32_t dma_buffer[64];
extern void resetInputCaptureTimer(void);

/* See the comments on the declarations in faults.h. */
volatile uint32_t fault_stall_trips = 0;

/* Acquisition-rail early-desync count (see faultNoteEarlyDesync). */
static uint8_t acq_fail_desyncs;

/* Ramp-attribution state (see the declarations in faults.h). */
volatile uint8_t fault_ramp_slew_witness_ms;
volatile uint8_t fault_ramp_halves;
volatile uint8_t fault_acq_resist_events;
volatile uint8_t fault_acq_soften;
/* Previous 1 kHz duty sample for the slew witness. */
static uint16_t ramp_witness_prev_duty;

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
}

uint8_t faultHandleStuckRotorIfNeeded(void)
{
#ifndef BRUSHED_MODE
	if ((bemf_timeout_happened > bemf_timeout) && eepromBuffer.stuck_rotor_protection) {
		allOff();
		maskPhaseInterrupts();
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
		// would fire a JUMP-sized charge (and another startup-soften step)
		// as if the cut never happened. Match episode-bucket pilot
		// semantics - and forgive the soften itself: zero throttle is the
		// pilot dealing with whatever was resisting the start.
		desync_episode_bucket = 0;
		desync_restart_holdoff_ms = 0;
		acq_fail_desyncs = 0;
		fault_acq_soften = 0;
	}
	if (zero_crosses > 100 && adjusted_input < 200) {
		bemf_timeout_happened = 0;
	}
	if (eepromBuffer.use_sine_start && adjusted_input < 160) {
		bemf_timeout_happened = 0;
	}

	if (crawler_mode) {
		if (adjusted_input < 400) {
			bemf_timeout_happened = 0;
		}
	} else {
		if (adjusted_input < 150) { // startup duty cycle should be low enough to not burn motor
			bemf_timeout = 100;
		} else {
			bemf_timeout = 10;
		}
	}
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
 * machinery (bucket, holdoff, latch), not to be the fastest path to the latch.
 * The side effect that most helps a struggling start finally clear
 * acquisition is the startup soften inside faultDesyncEpisodeCharge
 * (fault_acq_soften) - acquisition-scoped and forgiven on success, because a
 * start that cannot complete is at least as likely mechanically resisted
 * (grass, debris) as mis-tuned, and duty is near-always slewing during a
 * start so the slew witness cannot separate the two there. The PERMANENT
 * learned halve is reserved for established-run episodes with the witness
 * armed; established-run rails (blind/grind/stall) still need
 * zero_crosses > 100 and only help once the loop actually gets past that
 * gate.
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
 * Ramp-attribution slew witness window. From the start of a too-fast
 * transient to its episode charge: ~15 ms electrically silent, the ZC
 * window closes within ~3 commutations, then jump detection and the charge
 * run on the next main pass - tens of ms end to end. 100 ms covers that
 * with margin, and a snap-induced grind keeps re-arming it (every power-cut
 * release re-slews at the limit), while at steady duty it decays to 0 in
 * 100 ms so a load-dragged desync at cruise reads witness 0.
 */
#define RAMP_ATTR_WITNESS_MS 100
/* Acquisition soften cap: 3 right-shifts of the startup vcomp (/8). */
#define ACQ_SOFTEN_MAX 3

/*
 * Blind-GRIND rail. The episode rail above only sees DISCRETE failures
 * (jump desync, stall trip). Bench 2026-07-23 (pr48-snap-rail-1, 900KV +
 * 10x5x3 6S, snap 0.20->0.55): a spinning snap can drop the loop into a
 * CONTINUOUS partial desync that sits below every threshold - 423 blind
 * steps/s at a 19.5% miss rate (miss bucket needs >25% and net-drains;
 * consecutive counter resets on every accepted crossing; CI step +45% is
 * under the jump check's 50%; the stall rail never fires because blind
 * steps keep resetting INTERVAL_TIMER) - while the power cut engages and
 * releases in a limit cycle (one accepted crossing resets zc_blind_steps,
 * duty re-slews at max_ramp, spike to 102 A, desync, repeat).
 *
 * The unambiguous signal is the blind-step RATE: healthy runs total 9-27
 * blind steps per RUN; the grind produces 40+ per 100 ms. Sample
 * zc_blind_window_count each 100 ms; ONE hot window holds the power cut
 * AND forces the stall rail (mask + kick INTERVAL_TIMER, same primitives
 * as the blind-limit handoff), which restarts the loop and charges the
 * episode bucket (zero_crosses is far above 100 in a grind), so
 * repetition escalates through backoff to the latch like any other
 * episode source.
 *
 * Restart on the FIRST hot window, not after N consecutive: requiring
 * consecutive hot windows is self-defeating (bench pr48-snap-rail-2) -
 * the held cut drops the blind rate below threshold, the streak resets,
 * no restart ever fires, and each hold expiry re-slews duty into a fresh
 * spike (33-88 A at ~600 ms period). The detection margin carries a
 * single-window trigger, and a false trip costs one restart plus one
 * episode charge that drains in ~1.2 s of healthy running.
 */
#define GRIND_WINDOW_MS 100
#define GRIND_WINDOW_BLIND_LIMIT 15 /* >=150 blind/s = grind; healthy bursts stay single-digit */
#define GRIND_HOLD_MS 250

static uint8_t desync_episode_drain_ms;
static uint8_t grind_window_ms;

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

	// Learned ramp back-off, gated on ATTRIBUTION. A desync only teaches
	// "the configured ramp outran what this motor/prop can follow" when
	// the ramp was actually in play: the slew witness (sampled in
	// faultDesyncEpisodeTick1kHz) says duty was rising at >= 75% of the
	// active regime's limit within the last RAMP_ATTR_WITNESS_MS. A
	// too-fast transient always charges inside that window (no reactive
	// signal can catch it earlier - the first ~15 ms are electrically
	// silent, see the slew-limiter note in control_loop.c), and a
	// snap-induced grind keeps re-arming it through the power-cut
	// release/re-slew cycle. What the witness excludes is the
	// load-not-tune episode: a mechanical obstruction (grass, line wrap)
	// dragging an established run down at STEADY duty. Halving there
	// stores a wrong lesson for the whole power cycle - and on the
	// production tune (max_ramp 20 -> all regimes 2) a single halve IS
	// the fine-rate floor, a silent 20x slew-authority cut with no
	// telemetry and every other counter self-healing (crashed a vehicle
	// taking off after a ground spool in long grass).
	//
	// ACQ_FAIL episodes never take the permanent halve: duty is
	// near-always slewing during a start, so the witness cannot separate
	// resisted from mis-tuned there. They take fault_acq_soften instead -
	// a startup-regime-only back-off (consumed in
	// runtimeTransientGovernorTick) that is forgiven by the first genuine
	// acquisition (ACQ_FAIL_CLEAR_ZC) or pilot zero throttle - which
	// preserves the "soften until it finally acquires" escalation for an
	// honestly bad tune without neutering the session after the
	// obstruction is gone.
	//
	// Halve each attributed episode's regimes (floor 1); once every
	// regime is at 1, force fine mode (ramp_divider = 9) so the floor is
	// the bench-proven 0.1%/ms rate. Deliberately not cleared at zero
	// throttle (unlike the bucket): re-arming the fast ramp would
	// re-desync on the next transient. A settings write reruns
	// loadEEpromSettings and restores the configured values - retuning is
	// the way back within a session.
	if (kind == DESYNC_EPISODE_ACQ_FAIL) {
		if (fault_acq_soften < ACQ_SOFTEN_MAX) {
			fault_acq_soften++;
		}
		if (fault_acq_resist_events < 255) {
			fault_acq_resist_events++;
		}
	} else if (fault_ramp_slew_witness_ms) {
		max_ramp_startup = max_ramp_startup > 1 ? (uint8_t)(max_ramp_startup / 2) : 1;
		max_ramp_low_rpm = max_ramp_low_rpm > 1 ? (uint8_t)(max_ramp_low_rpm / 2) : 1;
		max_ramp_high_rpm = max_ramp_high_rpm > 1 ? (uint8_t)(max_ramp_high_rpm / 2) : 1;
		if (max_ramp_startup <= 1 && max_ramp_low_rpm <= 1 && max_ramp_high_rpm <= 1) {
			ramp_divider = 9;
		}
		if (fault_ramp_halves < 255) {
			fault_ramp_halves++;
		}
	}

	if (desync_episode_bucket >= DESYNC_EPISODE_LIMIT) {
		allOff();
		maskPhaseInterrupts();
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
	faultDesyncEpisodeCharge(DESYNC_EPISODE_ACQ_FAIL);
#endif
}

void faultDesyncEpisodeTick1kHz(void)
{
#ifndef BRUSHED_MODE
	/* Ramp-attribution slew witness: is duty rising at (near) the active
	 * regime's limit? Sampled here at 1 kHz so the comparison is directly
	 * per-ms: a regime step of vcomp per (ramp_divider + 1) 20 kHz ticks
	 * is vcomp * 20 / (ramp_divider + 1) duty units per ms. Arm at >= 75%
	 * of that (rise * 4 >= budget * 3, integer-robust at the fine floor
	 * where the budget is 2/ms). Rising only: power cuts and throttle
	 * chops slew down hard, and the lesson under attribution is about
	 * upward authority. Regime pick mirrors the 20 kHz slew limiter in
	 * tenKhzRoutine. A rise below the limit means the ramp was not the
	 * binding constraint, so a desync then teaches nothing about it. */
	{
		const uint16_t d = duty_cycle;
		const int32_t rise = (int32_t)d - (int32_t)ramp_witness_prev_duty;
		ramp_witness_prev_duty = d;
		uint8_t step;
		if (zero_crosses < 150 || last_duty_cycle < 150) {
			step = max_ramp_startup_vcomp;
		} else if (average_interval > 500) {
			step = max_ramp_low_rpm_vcomp;
		} else {
			step = max_ramp_high_rpm_vcomp;
		}
		const uint16_t budget = (uint16_t)(((uint16_t)step * 20u) / (uint8_t)(ramp_divider + 1u));
		if (running && rise > 0 && (uint32_t)rise * 4u >= (uint32_t)budget * 3u) {
			fault_ramp_slew_witness_ms = RAMP_ATTR_WITNESS_MS;
		} else if (fault_ramp_slew_witness_ms) {
			fault_ramp_slew_witness_ms--;
		}
	}

	/* A loop that got solidly established did acquire, whatever roughness
	 * it passed through on the way - forgive the whole batch, and the
	 * acquisition-scoped startup soften with it: whatever was resisting
	 * the start (or however mis-tuned it looked) is no longer preventing
	 * acquisition, and startup slew must not stay degraded into the
	 * flight that follows. */
	if (zero_crosses > ACQ_FAIL_CLEAR_ZC) {
		acq_fail_desyncs = 0;
		fault_acq_soften = 0;
	}
	if (desync_restart_holdoff_ms > 0) {
		desync_restart_holdoff_ms--;
	}
	if (zc_grind_hold_ms > 0) {
		zc_grind_hold_ms--;
	}

	/* Blind-grind window (see block comment above the constants). */
	if (++grind_window_ms >= GRIND_WINDOW_MS) {
		grind_window_ms = 0;
		uint8_t blind_in_window = zc_blind_window_count;
		zc_blind_window_count = 0; // a step between read and clear is one lost count - fine
		if (blind_in_window >= GRIND_WINDOW_BLIND_LIMIT && running && !old_routine) {
			// Hot window: hold the power cut (bridges the gap until the
			// restart takes and covers any deferred path) and force the
			// stall rail, same primitives as the blind-limit handoff in
			// PeriodElapsedCallback: with phase interrupts masked and
			// the deadline disarmed, nothing can reset INTERVAL_TIMER
			// before the main loop runs faultHandleBemfIntervalStall,
			// which restarts and charges the episode bucket.
			zc_grind_hold_ms = GRIND_HOLD_MS;
			maskPhaseInterrupts();
			DISABLE_COM_TIMER_INT();
			zc_deadline_armed = 0;
			SET_INTERVAL_TIMER_COUNT(46000);
		}
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
	if (INTERVAL_TIMER_COUNT > 45000 && (escInOpenLoop() || escInClosedLoop())) {
		bemf_timeout_happened++;

		maskPhaseInterrupts();
		// Charge the episode rail only when this run was ESTABLISHED
		// before it died (the bad-tune restart->spool->desync cycle
		// always reaches closed loop first). A start attempt that never
		// got going is the legacy stuck-rotor rail's job, with its
		// throttle-scaled tolerance (bemf_timeout 100 below input 150) -
		// heavy props legitimately kick many times at low throttle, and
		// charging those latched the ESC on the 4th kick. This gate also
		// covers the blind/miss-limit handoff (bemf_zc kicks
		// INTERVAL_TIMER to 46000 with comparator interrupts masked, so
		// this rail is guaranteed to run next pass): blind stepping only
		// arms at zero_crosses >= 100, so those episodes always charge.
		if (zero_crosses > 100) {
			/* Established run died here. This is the ONLY place the
			 * stall rail is counted, and it is the aggregation point
			 * for the grind rail and the blind/miss-limit handoff,
			 * both of which reach the loop through this trip - see
			 * faultErrorCount(). */
			fault_stall_trips++;
			faultDesyncEpisodeCharge(DESYNC_EPISODE_STALL_RAIL);
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
		if (faultDesyncRestartHoldoffActive()) {
			/* Coast until holdoff expires; main loop will re-arm. */
			running = 0;
			allOff();
			return;
		}
		zcfoundroutine();
	}
}
