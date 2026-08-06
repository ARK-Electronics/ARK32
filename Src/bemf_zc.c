/*
 * bemf_zc.c - extracted from main.c (behavior-neutral split)
 */

#include "bemf_zc.h"
#include "motor_runtime.h"
#include "commutation.h"
#include "main.h"
#include "common.h"
#include "comparator.h"
#include "phaseouts.h"
#include "targets.h"
#include "IO.h"
#include "peripherals.h"
#include "functions.h"
#include "eeprom.h"
#include "hwci_perf.h"

/*
 * Missed-ZC fallback (BLHeli-style timeout commutation). After each
 * commutation COM_TIMER is re-armed as a deadline for the next crossing:
 * expected arrival plus 50% grace. An accepted crossing re-arms COM_TIMER
 * for the commutation schedule (cancelling the deadline); if the deadline
 * fires instead, commutate blind at extrapolated timing and keep watching.
 * A missed crossing costs one extrapolated step, not a mode change - there
 * is no fallback to poll mode at runtime.
 */
/*
 * Dead-reckoning budget. A blind step commutates on extrapolated timing, so
 * between the last accepted crossing and the next one the rotor position is
 * an estimate. zc_blind_ticks accumulates that estimated time (INTERVAL_TIMER
 * ticks) and an accepted crossing clears it: it is "how long since the rotor
 * last told us where it was".
 *
 * That single quantity replaces the consecutive-blind-step limit, the leaky
 * miss-rate bucket and the blind-grind window that used to sit here and in
 * faults.c. All three were answering the same question - how much blind is
 * too much - in different units, each with its own tuned threshold, and each
 * answering it by RESTARTING. The miss-rate bucket in particular gained 3 per
 * blind step and drained 1 per accepted crossing, so it tripped above a 25%
 * miss rate exactly: a cliff with no hysteresis, where one side runs forever
 * and the other restarts forever, and a restart cannot improve the miss rate
 * that caused it.
 *
 * There is no threshold to pick here. Upstream restarts when INTERVAL_TIMER
 * shows no commutation evidence for BEMF_STALL_TICKS, and the only reason
 * that rail could not see a blind grind is that blind steps reset
 * INTERVAL_TIMER. Measuring the blind time separately restores upstream's own
 * criterion - and with it upstream's behaviour, which is to keep flying on
 * degraded timing rather than stop. A 50% miss rate now flies: every accepted
 * crossing clears the budget, so only a genuine absence of crossings can
 * exhaust it.
 */
#define ZC_DEAD_RECKONING_BUDGET BEMF_STALL_TICKS

#define ZC_DEADLINE_MIN_ZC 100

/*
 * Acceleration-aware commutation point.
 *
 * commutation_interval is a two-pole lag over the last two measured
 * intervals: the pair is averaged, then blended 50/50 with the previous
 * estimate. Under constant angular acceleration a lag filter has constant
 * error, so the estimate trails the true interval by an amount proportional
 * to the acceleration - and waitTime = CI/2 - advance is computed from that
 * trailing value, which puts the commutation systematically late through
 * every spool-up. Late drive raises current, and more current lengthens
 * demag, which is the same spiral the demag-late detector exists to break
 * (see interruptRoutine).
 *
 * Track the per-interval trend and commutate against the interval the loop
 * is about to see rather than the one it just averaged. zc_trend is an EMA
 * (tau = 4 intervals) of thiszctime - lastzctime, so a steady acceleration
 * produces a steady correction while single-interval jitter is smoothed.
 *
 * commutation_interval itself is deliberately NOT changed: it feeds the
 * stall threshold, the ZC filter schedule, variable PWM, the DShot priority
 * flip and telemetry, none of which want a predicted value. Only the
 * advance/waitTime pair moves onto the prediction.
 *
 * Bounds: the correction is clamped to +-50% of the current estimate, so a
 * corrupted trend can never move the commutation point further than a
 * single missed crossing already does.
 *
 * Blind steps do not feed the trend - thiszctime is the deadline's inflated
 * 1.5x value there, not a measurement.
 */
#define ZC_TREND_MIN_ZC 20
static int32_t zc_trend;
/* Last predicted interval used for advance/waitTime (observability + SITL). */
static uint32_t zc_predicted;

/*
 * Extrapolated-average taint window (see bemfZcAverageTainted in bemf_zc.h).
 *
 * A blind step's inflated interval reaches the jump-desync detector by two
 * routes, and both have to flush before a comparison means anything:
 *
 *   commutation_intervals[] -> e_com_time is a SUM OF SIX entries (main.c), so
 *   six clean commutations are needed to push the last blind interval out.
 *
 *   average_interval is then compared against last_average_interval, which is
 *   sampled one electrical revolution earlier - so a second clean revolution
 *   is needed before BOTH endpoints of the comparison are measurements.
 *
 * Twelve commutations, i.e. two electrical revolutions. At 5k rpm on a 14-pole
 * motor that is ~3.4 ms of suppression per blind step, during which the
 * dead-reckoning budget in this file is still watching - the loop is not
 * unprotected, the jump check is simply not the rail that owns this case.
 */
#define ZC_TAINT_COMMUTATIONS 12
static uint8_t zc_taint;

int32_t bemfZcGetTrend(void)
{
	return zc_trend;
}

uint8_t bemfZcAverageTainted(void)
{
	return (uint8_t)(zc_taint != 0);
}

uint32_t bemfZcGetPredicted(void)
{
	return zc_predicted;
}

RAM_FUNC void PeriodElapsedCallback()
{
	uint8_t blind = 0;
	DISABLE_COM_TIMER_INT(); // disable interrupt
	if (!running || old_routine) {
		// COM events are meaningless outside interrupt closed-loop: a
		// stale blind-step deadline after a stop or forced restart, or a
		// poll-mode COM_TIMER ARR write landing while the deadline's
		// interrupt enable was still set. Legacy never ran this handler
		// in poll mode (the enable stayed off); keep that invariant, or
		// the cancel-race guard below re-arms COM against the poll
		// startup and spurious commutations fight zcfoundroutine.
		zc_deadline_armed = 0;
		return;
	}
	if (zc_deadline_armed) {
		// Deadline firing, not a commutation scheduled by an accepted
		// zero-cross (interruptRoutine cancels the deadline first).
		zc_deadline_armed = 0;
		if (zc_blind_ticks >= ZC_DEAD_RECKONING_BUDGET) {
			// No accepted crossing for a whole dead-reckoning budget:
			// position is genuinely unknown, not merely uncertain. Hand
			// off exactly as before - kick INTERVAL_TIMER past the stall
			// threshold so faultHandleBemfIntervalStall restarts through
			// the startup path on its next main-loop pass, and charges the
			// episode bucket once. With phase interrupts masked nothing can
			// reset INTERVAL_TIMER before it runs.
			HWCI_ZC_TRACE_EV(HWCI_ZC_EV_BUDGET, INTERVAL_TIMER_COUNT);
			maskPhaseInterrupts();
			SET_INTERVAL_TIMER_COUNT(BEMF_STALL_TICKS + 1000u);
			return;
		}
		blind = 1;
		zc_blind_steps++;
		HWCI_PERF_BLIND_STEP();
		/*
		 * Commutate now on the existing estimate, and record NO interval
		 * measurement - see the commutation_interval update below.
		 *
		 * This deliberately reverses the older behaviour, which took the
		 * full elapsed deadline time as a "late crossing measurement" on
		 * the theory that an inflated interval makes timing hunt slower,
		 * the safe direction for a decelerating rotor, and that the next
		 * accepted crossing would resync it immediately.
		 *
		 * The second half of that is false at any sustained miss rate, and
		 * the feedback runs away. The deadline is armed at 1.5x the
		 * estimate, so a blind step's elapsed time is ALWAYS ~1.5x - it is
		 * a property of the grace period, not a measurement of the rotor.
		 * Feeding it to the average raises the estimate, which lengthens
		 * the next deadline, which produces a larger elapsed, and so on.
		 *
		 * Measured in SITL under mode-2 fault injection (drop every other
		 * commutation window, a 50% miss rate on a rotor holding speed):
		 * commutation_interval went 432 -> 1880 -> 3281 -> 3942 -> 4396 ->
		 * 5128 -> 5823 -> 6853 in 70 ms, a 16x inflation with the rotor at
		 * constant rpm, until the stall rail tripped on the ESC's own
		 * runaway and restarted the loop. Every accepted crossing in
		 * between cleared zc_blind_ticks exactly as designed - the
		 * dead-reckoning budget was never the thing that fired.
		 *
		 * A blind step's real information content is a LOWER BOUND: "no
		 * crossing arrived before 1.5x". Treating a lower bound as a point
		 * estimate is the whole bug. The rotor slowing down is still
		 * caught, by the next accepted crossing measuring long and by
		 * zc_blind_ticks if crossings stop arriving altogether.
		 */
		maskPhaseInterrupts();
		uint32_t elapsed = INTERVAL_TIMER_COUNT;
		if (elapsed > 65535u) {
			elapsed = 65535u;
		}
		/*
		 * Not atomic on Cortex-M0, and does not need to be: the only other
		 * writer is the clear in interruptRoutine, and maskPhaseInterrupts()
		 * three lines up has already shut the comparator EXTI off - so no
		 * accepted crossing can land between this load and its store. The
		 * remaining concurrent access is the read in setInput (DShot ISR on
		 * F051), which can see the pre-increment value for one frame; that
		 * is a monotone counter read one update late, and it costs at most
		 * one frame of slightly-too-generous duty authority.
		 */
		zc_blind_ticks += elapsed; // extrapolated time, cleared by a real crossing
		HWCI_ZC_TRACE_EV(HWCI_ZC_EV_BLIND, elapsed);
		/* lastzctime/thiszctime deliberately NOT written: they are the
		 * measured-interval history. Resetting INTERVAL_TIMER still means
		 * the next accepted crossing measures the window from this blind
		 * commutation to that crossing, which IS a real measurement. */
		SET_INTERVAL_TIMER_COUNT(0);
	} else {
		// Cancel race: SET_AND_ENABLE_COM_INT clears the peripheral flag,
		// but a deadline event that reached NVIC pending just before an
		// accepted zero-cross cancelled it still runs this handler once.
		// The genuine commutation fires with INTERVAL_TIMER at ~waitTime
		// (both reset together at the crossing); far earlier means this
		// entry is that stale pend - re-arm the remainder and bail.
		uint32_t t = INTERVAL_TIMER_COUNT;
		if (t + 4u < waitTime) {
			SET_AND_ENABLE_COM_INT((uint16_t)(waitTime - t));
			return;
		}
	}
	commutate();
	/* Only a measured crossing updates the interval estimate. A blind step
	 * contributes nothing: see the note in the deadline branch above for the
	 * runaway this prevents. The estimate simply holds, so the next deadline
	 * stays at 1.5x the last MEASURED interval instead of compounding. */
	if (!blind) {
		commutation_interval = ((commutation_interval) + ((lastzctime + thiszctime) >> 1)) >> 1;
	}
	/* See zc_trend: predict the interval about to be timed, not the lagging
	 * average of the ones already measured. */
	uint32_t predicted = commutation_interval;
	if (!blind) {
		const int32_t step = (int32_t)thiszctime - (int32_t)lastzctime;
		zc_trend += (step - zc_trend) >> 2;
	}
	if (zero_crosses >= ZC_TREND_MIN_ZC) {
		const int32_t limit = (int32_t)(commutation_interval >> 1);
		int32_t corr = zc_trend;
		if (corr > limit) {
			corr = limit;
		} else if (corr < -limit) {
			corr = -limit;
		}
		predicted = (uint32_t)((int32_t)commutation_interval + corr);
	}
	zc_predicted = predicted;
	if (!eepromBuffer.auto_advance) {
		advance = (predicted * temp_advance) >> 6; // 60 divde 64 0.9375 degree increments
	} else {
		advance = (predicted * auto_advance_level) >> 6; // 60 divde 64 0.9375 degree increments
	}
	/* Saturate: predicted can be as low as CI/2 under max negative
	 * correction, so (predicted/2 - advance) must not wrap uint16. */
	waitTime = bemfZcWaitTimeFromInterval(predicted, advance);
	if (!old_routine) {
		enableCompInterrupts(); // enable comp interrupt
		if (zero_crosses >= ZC_DEADLINE_MIN_ZC) {
			// Next crossing expected (commutation_interval - waitTime)
			// from now; arm the blind-step deadline at expected + 50%
			// grace. The 16-bit clamp shrinks the grace above CI ~40000,
			// but the trust rail restarts long before a healthy loop
			// runs that slow.
			uint32_t deadline = (commutation_interval - waitTime) + (commutation_interval >> 1);
			if (deadline > 65535u) {
				deadline = 65535u;
			}
			zc_deadline_armed = 1;
			SET_AND_ENABLE_COM_INT((uint16_t)deadline);
		}
	}
	/* Taint the interval average for as long as an extrapolated interval can
	 * still be inside e_com_time's six-deep window or inside the previous
	 * endpoint of the jump check's comparison. */
	if (blind) {
		zc_taint = ZC_TAINT_COMMUTATIONS;
	} else if (zc_taint) {
		zc_taint--;
	}
	if (!blind && zero_crosses < 10000) {
		zero_crosses++;
	}
	if (!blind) {
		// Blind steps must not feed the jitter accumulators: the
		// extrapolated 1.5x interval is not a measured crossing, and the
		// perf gate (zero_crosses >= 100) is the same window in which
		// blind stepping is armed - counting them would conflate real
		// jitter with blind-step count in exactly the runs being read.
		// Blind steps have their own counter (HWCI_PERF_BLIND_STEP).
		HWCI_PERF_ZC();
	}
}

RAM_FUNC void interruptRoutine()
{
	HWCI_PERF_ZC_PHASE_CAPTURE(); // TIM1 phase at ISR entry, pre-confirm
#ifdef MCU_F051
	// PWM phase (TIM1 CNT) at the edge, sampled before the confirm loop's
	// wall-clock window advances it: the turn-on-pileup compensation below
	// keys off where the comparator edge actually appeared, not where it was
	// finally accepted.
	uint16_t zc_pwm_cnt = (uint16_t)TIM1->CNT;
#endif
	/*
	 * NO MINIMUM-INTERVAL GATE HERE.
	 *
	 * A gate rejecting any edge arriving sooner than a quarter of the tracked
	 * interval used to sit at this point. It was built on the hypothesis that
	 * the wrong-phase lock this branch chases is a false lock onto switching-
	 * transient noise, and the F051 has no hardware defence against that -
	 * comparator output blanking is a G0/G4 peripheral feature (see the
	 * MCU_G071 guard in Mcu/f051/Src/peripherals.c enableCorePeripherals);
	 * the STM32F0 COMP has no blanking source at all, so the software confirm
	 * loop below is the only filter and it deliberately tolerates
	 * filter_level/4 bad samples. Upstream AM32 carries the same guard,
	 * commented out, immediately below ("should be impossible, desync?").
	 *
	 * The hypothesis was wrong, and the measurement that killed it is worth
	 * recording so it is not rebuilt. A PWM-phase histogram of ACCEPTED
	 * crossings (hwci_perf zc_phase_hist, binned on TIM1->CNT at ISR entry;
	 * TIM1 is edge-aligned UP so CNT maps monotonically onto the PWM period)
	 * was taken across healthy running and two hand-provoked grinds on an AOS
	 * Supernova 2207 1980KV, 6S, 5" tri-blade, ARK 4IN1:
	 *
	 *     class   frac@turn-on  frac@turn-off  peak enrich  bins holding 80%
	 *     healthy         0.5%          22.2%          9.8               6.7
	 *     grind           0.6%           5.8%          5.7              11.0
	 *     uniform         3.1%           9.4%          1.0              26
	 *
	 * Healthy running is sharply locked to the turn-off edge, which is what a
	 * genuine crossing looks like when the floating phase is only clean during
	 * PWM-off. The grind is BROADER than healthy and DEPLETED at both
	 * switching edges - the opposite of a switching artefact. Nor is it
	 * broadband noise: the intervals repeat far too tightly (see the detector
	 * below). The edges the loop accepts during a grind are real BEMF
	 * crossings at a phase relationship the commutation table does not match.
	 *
	 * That is fatal to any per-edge filter. A real crossing cannot be
	 * rejected on its own merits without also rejecting correct ones, and the
	 * early edge of the pair is as likely to be the genuine one as the late
	 * edge. The gate also disarmed itself in practice: rejected edges leave
	 * commutation_interval alone, but ACCEPTED-but-wrong ones update it, so
	 * the threshold ratcheted down with the estimate it was derived from -
	 * measured ci 719 -> 570 -> 527 -> 217 -> 86 -> 52 with the gate
	 * following 179 -> 142 -> 131 -> 54 -> 21 -> 13. A detector whose
	 * threshold derives from the quantity the failure corrupts cannot work.
	 *
	 * The replacement is a sequence test, not an edge test: see the
	 * wrong-phase lock detector further down this function.
	 */
#ifdef HWCI_PERF
	/* Interval at ISR entry, for the per-commutation trace only - with the
	 * gate gone nothing in the control path reads it. */
	const uint32_t zc_elapsed = INTERVAL_TIMER_COUNT;
#endif
	//        stuckcounter++; // stuck at 100 interrupts before the main loop happens
	//                        // again.
	//        if (stuckcounter > 100) {
	//            maskPhaseInterrupts();
	//            zero_crosses = 0;
	//            return;
	//        }
	//    }
	// Zero-cross confirm: reject unless the window's reads hold the
	// post-crossing level. Loop speed sets the sampling window: inlining
	// getCompOutputLevel removes the per-sample call overhead, and with the
	// companion RAM-execution change the loop is faster still; filter_level
	// is scaled (42/10/7 on F051) to keep the same wall-clock window as the
	// stock ~56-cycle sampling cadence.
#ifdef MCU_F051
	{
		/*
		 * HELD-LEVEL CONFIRM (position-scored), F051 only.
		 *
		 * This is the entry door to the wrong-phase lock, and the one place
		 * ARK diverges from upstream on the zero-cross path.
		 *
		 * Upstream - the non-F051 branch immediately below, still used by
		 * every other MCU - rejects an edge on the FIRST sample that reads
		 * the pre-crossing level. ARK replaced that here with a quota: count
		 * bad samples, accept if fewer than filter_level/4 are wrong.
		 *
		 * The quota was not gratuitous. 33d93ad moved this loop to RAM and
		 * inlined getCompOutputLevel, taking the sampling cadence from ~56
		 * cycles to ~16, which lands on brief comparator glitches about 3x
		 * more often; a strict confirm then defers detection to the next
		 * comparator edge, up to a PWM period late, and bench-measured 2-3x
		 * higher commutation jitter at 15-20 kHz (23c8387).
		 *
		 * But scoring by COUNT ignores WHERE the bad samples fall. Up to ten
		 * of forty-two may read wrong ANYWHERE, including at the very end of
		 * the window - where "wrong" means the level never actually held. On
		 * the F051 that matters more than anywhere else: there is no hardware
		 * comparator blanking (a G0/G4 peripheral feature; the STM32F0 COMP
		 * has no blanking source at all), so this loop is the entire filter.
		 * Bench capture with the rotor STATIONARY showed accepted intervals
		 * of 54, 103 and 59 ticks - around 200k eRPM - self-sustaining on the
		 * ESC's own commutation transients, which are exactly the kind of
		 * never-settled level a count-scored window admits.
		 *
		 * Score by POSITION instead. A real zero crossing is a PERMANENT
		 * level change; a glitch is transient. So the back half of the window
		 * must be entirely clean and the front half is not scored at all.
		 * That is simultaneously STRICTER than the quota - which tolerated
		 * bad samples precisely where they disprove the crossing - and MORE
		 * FORGIVING than upstream, because glitches cluster in the post-edge
		 * settling region this now ignores. The quota's purpose (do not
		 * reject a real crossing over one glitch) is kept; its side effect
		 * (accept a level that never held) is not.
		 *
		 * At filter_level 42 the back half is 21 samples, ~7us of held level
		 * at this cadence - far longer than commutation-transient ringing,
		 * and still inside the crossing itself.
		 *
		 * MEASURE JITTER when touching this. perf_zc_jitter_max is what the
		 * quota was bought with; if it regresses, move the scoring boundary
		 * rather than going back to a count.
		 */
		const int scored_from = filter_level >> 1;
		for (int i = 0; i < filter_level; i++) {
			if (getCompOutputLevel() == rising && i >= scored_from) {
				HWCI_PERF_CONFIRM_REJECT();
				return;
			}
		}
	}
#else
	for (int i = 0; i < filter_level; i++) {
#	if defined(MCU_F031) || defined(MCU_G031)
		if (((current_GPIO_PORT->IDR & current_GPIO_PIN) == !(rising))) {
#	else
		if (getCompOutputLevel() == rising) {
#	endif
			HWCI_PERF_CONFIRM_REJECT();
			return;
		}
	}
#endif
#ifdef MCU_F051
	// Turn-on-pileup timestamp compensation. A zero-cross that physically
	// occurs during the PWM off-window (freewheel) is invisible to the
	// comparator and is only registered at the next turn-on, quantizing the
	// timestamp to the PWM grid; those corrupted intervals feed waitTime and
	// the commutation loop hunts, amplifying the quantization into the
	// measured jitter hump (bench: ~23% at t60/t70 where the commutation rate
	// beats against the 24 kHz PWM). Detections of such crossings land 1-3
	// phase bins after turn-on (bin 1 peak = comparator+ISR latency of
	// ~1.3-2.6 us; bin 0 stays at the uniform rate) and the true crossing lay
	// on average half the off-window earlier - back-date thiszctime by that
	// half-window so the loop tracks the estimated true crossing instead of
	// the grid position.
	//
	// Guards, each tied to a bench-observed failure of the unguarded version
	// (which cut hump jitter 23.4->17.9% but desynced on a throttle step and
	// doubled low-RPM jitter):
	//  - correct thiszctime ONLY; the interval timer still resets to zero at
	//    the detection instant, so one bad estimate perturbs one interval and
	//    cannot cascade into the next (the cascading variant lost sync at 70A)
	//  - duty-slew guard: skip while adjusted_duty_cycle is moving, i.e.
	//    during throttle transients, where the loop must track real
	//    acceleration rather than have timestamps rewritten under it
	//  - hump band only (commutation interval < 2 PWM periods): at low RPM the
	//    interval dwarfs the PWM period, there is no hunting to break, and the
	//    correction only adds variance
	//  - clamp to interval/8 as a backstop against any degenerate state
	// Units: INTERVAL_TIMER (TIM2) is 2 MHz and TIM1 is 48 MHz, so one INTERVAL
	// tick is 24 TIM1 ticks; half an off-window of N TIM1 ticks is N/48 INTERVAL
	// ticks, computed as (N * 1365) >> 16 to keep a soft divide out of the ISR.
	uint16_t zc_grid_comp = 0;
	{
		static uint16_t zc_prev_duty;
		const uint16_t zc_arr = tim1_arr;
		const uint16_t zc_duty = adjusted_duty_cycle;
		const uint16_t zc_slew =
			(zc_duty >= zc_prev_duty) ? (uint16_t)(zc_duty - zc_prev_duty) : (uint16_t)(zc_prev_duty - zc_duty);
		zc_prev_duty = zc_duty;
		const uint32_t zc_ci = commutation_interval;
		if (zero_crosses >= 100 && zc_duty < zc_arr					  /* off-window exists */
		    && zc_pwm_cnt >= (uint16_t)(zc_arr >> 5)					  /* pile-up bins 1-3 */
		    && zc_pwm_cnt < (uint16_t)(zc_arr >> 3) && zc_slew <= (uint16_t)(zc_arr >> 8) /* duty steady */
		    && (zc_ci * 12u) < (uint32_t)zc_arr + 1u					  /* hump band only */
		) {
			uint32_t comp = ((uint32_t)(zc_arr - zc_duty) * 1365u) >> 16;
			const uint32_t cap = zc_ci >> 3;
			if (comp > cap) {
				comp = cap;
			}
			zc_grid_comp = (uint16_t)comp;
		}
	}
#endif
	__disable_irq();
	maskPhaseInterrupts();
	zc_deadline_armed = 0; // COM_TIMER now times commutation, not the missed-ZC deadline
	zc_blind_steps = 0;
	// Demag-late crossing detection, decision half (sampler:
	// runtimeSampleBemfPreLevel). No pre-crossing dwell observed since the
	// last commutation means this edge is the freewheel demag clamp
	// releasing over an already-past crossing - the loop is commutating
	// late by the demag duration. Late drive raises current and more
	// current lengthens demag, so left alone this settles into a stable
	// mistimed lock (bench 2026-07-22: ~30% slow at 8-10x current on ~1/3
	// of warm snap starts, 100% of hot ones; never self-heals; invisible
	// to the desync jump check and the trust rail because the interval is
	// steady and fast-looking). Count the consecutive run and let it fade
	// duty authority (control_loop): less current shortens demag, the
	// pre-level dwell reappears and the loop re-times itself - the firmware
	// equivalent of the throttle-blip escape verified on the bench. Gated to
	// CI > 500 (250 us+), where the healthy pre-level dwell spans many
	// main-loop passes so the sampler cannot miss it.
	//
	// The response is the authority fade ONLY - a demag-late crossing still
	// clears the dead-reckoning budget, so it can never reach the stall
	// rail. Normal spool-up under load is genuinely demag-late for long
	// stretches (high slip current), so any restart escalation turns every
	// hard start into a kick loop (bench: 2-3 restarts and ~10 desyncs per
	// 2 s start attempt with bucket escalation enabled). The cut is
	// self-resolving in both cases: during spool-up it bounds accel
	// current until slip drops, and in the mistimed lock it collapses the
	// current that sustains the wrong timing. A loop that stays demag-late
	// forever just stays current-limited - strictly better than the 8-10x
	// current of the untreated lock.
	if (commutation_interval > 500 && !zc_pre_seen) {
		zc_demag_accepts++;
		if (zc_demag_run < 255) {
			zc_demag_run++;
		}
	} else {
		zc_demag_run = 0;
	}
	zc_pre_seen = 0;
	/*
	 * The rotor has just reported its position, so the dead-reckoning budget
	 * is spent and starts again. This clears unconditionally, INCLUDING on a
	 * demag-late crossing: a late report is still a report, and normal
	 * spool-up under load is genuinely demag-late for long stretches (high
	 * slip current). Charging those to the budget walks a hard start into a
	 * restart, which is the kick loop the demag-late block above is explicit
	 * about avoiding - its response is a current reduction only, never an
	 * escalation. Demag authority is handled separately in control_loop.c.
	 */
	zc_blind_ticks = 0;
	/* Per-commutation trace: this edge passed the minimum-interval gate AND
	 * the confirm loop, so it is what the loop actually acted on. The
	 * trigger arms on the entry signature of the false lock - an accepted
	 * interval less than half the tracked estimate - so the ring freezes
	 * holding the commutations either side of entry. Instrumentation only:
	 * it changes no control decision. */
	HWCI_ZC_TRACE_EV(HWCI_ZC_EV_ACCEPT, zc_elapsed);
#ifdef MCU_F051
	const uint16_t newzctime = (uint16_t)(INTERVAL_TIMER_COUNT - zc_grid_comp);
#else
	const uint16_t newzctime = (uint16_t)INTERVAL_TIMER_COUNT;
#endif
	lastzctime = thiszctime;
	thiszctime = newzctime;
	SET_INTERVAL_TIMER_COUNT(0);
	SET_AND_ENABLE_COM_INT(waitTime + 1); // enable COM_TIMER interrupt
	__enable_irq();
	HWCI_PERF_ZC_PHASE_COMMIT(); // accepted edge: bin its PWM phase
}

void bemfZcResetTrend(void)
{
	zc_trend = 0;
	zc_predicted = 0;
}

void startMotor()
{
	if (running == 0) {
		DISABLE_COM_TIMER_INT(); // a stale blind-step deadline must not commutate the restart
		zc_deadline_armed = 0;
		zc_blind_steps = 0;
		zc_blind_ticks = 0;
		zc_taint = 0;	    // no extrapolated intervals survive a stop
		bemfZcResetTrend(); // no interval history across a stop
		zc_pre_seen = 1;
		zc_demag_run = 0;
		commutate();
		commutation_interval = 10000;
		SET_INTERVAL_TIMER_COUNT(5000);
		running = 1;
	}
	enableCompInterrupts();
}
