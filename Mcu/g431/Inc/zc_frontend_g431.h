/*
 * Zero-crossing front end: ARK 12S CAN (STM32G431), and the ARK_G4_12S_SITL_CAN SITL target.
 *
 * The deadline / blind-step half of PeriodElapsedCallback() and the
 * comparator-edge path interruptRoutine() (confirm loop, timestamp, COM
 * arm) for its two comparators; the edge gate in COMP1_2_3_IRQHandler drops an early edge.
 * Included only by Src/bemf_zc.c, through Inc/zc_frontend.h, so every MCU
 * keeps that translation unit and its LTO order. Not a standalone header.
 */

RAM_FUNC void PeriodElapsedCallback()
{
	uint8_t blind = 0;
	/* Blind step only: INTERVAL_TIMER at this commutation, the grace
	 * already spent since the virtual crossing it follows. */
	uint32_t blind_since_crossing = 0;
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
			maskPhaseInterrupts();
			SET_INTERVAL_TIMER_COUNT(BEMF_STALL_TICKS + 1000u);
			return;
		}
		blind = 1;
		zc_blind_steps++;
		HWCI_PERF_BLIND_STEP();
		// Place the missed crossing where it was expected - one estimated
		// interval after the last crossing, real or virtual - and
		// commutate now. The interval timer restarts at the grace already
		// spent, not 0, so
		// the next real crossing is timed and gated from the virtual one
		// as after any accepted crossing, and the next deadline is armed
		// from it too (below).
		//
		// The old step took the whole elapsed time (~1.5 CI) as the
		// interval and restarted the timer at the commutation. That put
		// the timer origin ~CI/2 late and grew the estimate 12.5% per
		// step, so the next real crossing arrived inside the comparator
		// gate's blanking (CI/2 on G431/SITL, where it is dropped;
		// average_interval/2 on F051, where it is deferred and timed
		// late): one missed window became a cascade (SITL: 7 blind steps
		// and CI x2 every time, more than half ending in a desync restart;
		// ARK 4IN1 bench: 13-41 A spikes). Dead reckoning no longer hunts
		// slower: a rotor that slows during the dropout is caught only if
		// its crossing lands before the next deadline (1.5 CI after the
		// previous crossing), and a stopped one still exhausts the
		// budget, which counts real time since the last accepted crossing.
		maskPhaseInterrupts();
		uint32_t elapsed = INTERVAL_TIMER_COUNT;
		if (elapsed > 65535u) {
			elapsed = 65535u;
		}
		uint32_t interval = commutation_interval;
		if (interval > elapsed) {
			interval = elapsed;
		}
		blind_since_crossing = elapsed - interval;
		// Real time since the previous commutation: the timer started at 0
		// at a real crossing, at zc_blind_timer_start after a blind step.
		zc_blind_ticks += elapsed - zc_blind_timer_start; // cleared by a real crossing
		zc_blind_timer_start = (uint16_t)blind_since_crossing;
		lastzctime = thiszctime;
		thiszctime = (uint16_t)interval;
		SET_INTERVAL_TIMER_COUNT(blind_since_crossing);
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
	commutation_interval = ((commutation_interval) + ((lastzctime + thiszctime) >> 1)) >> 1;
	/* See zc_trend: predict the interval about to be timed, not the lagging
	 * average of the ones already measured. */
	uint32_t predicted = commutation_interval;
	if (blind) {
		zc_trend_hold = 2;
	} else if (zc_trend_hold) {
		zc_trend_hold--;
	} else {
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
	if (!eepromBuffer.auto_timing) {
		advance = (predicted * temp_advance) >> 6; // 60 divde 64 0.9375 degree increments
	} else {
		advance = (predicted * auto_advance_level) >> 6; // 60 divde 64 0.9375 degree increments
	}
	/* Saturate: predicted can be as low as CI/2 under max negative
	 * correction, so (predicted/2 - advance) must not wrap uint16. */
	waitTime = bemfZcWaitTimeFromInterval(predicted, advance);
	if (blind) {
		// A real crossing's commutation comes waitTime after it, and the
		// comparator gate stays shut until CI/2: `advance` of blanking
		// over the demag that follows. A blind commutation comes after
		// the gate has opened, so blank the same `advance` here - capped
		// at CI/4, half the ~CI/2 left to the expected crossing, so a
		// crossing on time is never blanked even at high advance.
		const uint32_t cap = commutation_interval >> 2;
		const uint32_t blank = (advance < cap) ? advance : cap;
		zc_blank_until = (uint16_t)(blind_since_crossing + blank);
	}
	if (!old_routine) {
		enableCompInterrupts(); // enable comp interrupt
		if (zero_crosses >= ZC_DEADLINE_MIN_ZC) {
			// Next crossing expected (commutation_interval - time since
			// the crossing this commutation follows) from now; arm the
			// blind-step deadline at expected + 50% grace. After a real
			// crossing that time is waitTime, as before. After a blind
			// step it is the grace already spent since the virtual
			// crossing, so each blind commutation stays 0.5 CI after its
			// own virtual crossing (1.5 CI after the previous one)
			// instead of drifting one advance later each step. The 16-bit clamp shrinks the grace
			// above CI ~40000, but the trust rail restarts long before a
			// healthy loop runs that slow.
			uint32_t since_crossing = blind ? blind_since_crossing : waitTime;
			if (since_crossing > commutation_interval) {
				since_crossing = commutation_interval;
			}
			uint32_t deadline = (commutation_interval - since_crossing) + (commutation_interval >> 1);
			if (deadline > 65535u) {
				deadline = 65535u;
			}
			zc_deadline_armed = 1;
			SET_AND_ENABLE_COM_INT((uint16_t)deadline);
		}
	}
	if (!blind && zero_crosses < 10000) {
		zero_crosses++;
	}
	if (!blind) {
		// Blind steps must not feed the jitter accumulators: the
		// dead-reckoned interval is not a measured crossing, and the
		// perf gate (zero_crosses >= 100) is the same window in which
		// blind stepping is armed - counting them would conflate real
		// jitter with blind-step count in exactly the runs being read.
		// Blind steps have their own counter (HWCI_PERF_BLIND_STEP).
		HWCI_PERF_ZC();
	}
}

RAM_FUNC void interruptRoutine()
{
	if (INTERVAL_TIMER_COUNT < zc_blank_until) { // 0 = no blanking
		return;				     // a blind commutation's demag window (PeriodElapsedCallback)
	}
	HWCI_PERF_ZC_PHASE_CAPTURE(); // TIM1 phase at ISR entry, pre-confirm
				      /* ARK F051 + G431: glitch-tolerant confirm + turn-on-grid compensation
	 * (ported from the ARK_4IN1_F051 bench work). Other MCUs keep stock. */
	// PWM phase (TIM1 CNT) at the edge, sampled before the confirm loop's
	// wall-clock window advances it: the turn-on-pileup compensation below
	// keys off where the comparator edge actually appeared, not where it was
	// finally accepted.
	uint16_t zc_pwm_cnt = (uint16_t)TIM1->CNT;
	//   if (average_interval > 125) {
	//        if ((INTERVAL_TIMER_COUNT < 125) && (duty_cycle < 600) && (zero_crosses < 500)) { // should be impossible, desync?exit anyway
	//           return;
	//        }
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
	// Glitch-tolerant variant: a fast sampling cadence lands on a brief
	// comparator glitch more often than stock, and a strict
	// all-samples-must-agree confirm defers detection to the NEXT
	// comparator edge - up to a PWM period late. Bench-measured on the
	// ARK 4IN1: 2-3x higher commutation jitter at 15-20 kHz commutation
	// rates vs stock cadence (upstream 2.1% vs 5.4% at full throttle).
	// Tolerating up to filter_level/4 bad samples per window accepts
	// through isolated glitches while a genuinely un-crossed level still
	// rejects via the early-out; the full window length (and so the
	// sustained-noise immunity of the filter_level retune) is unchanged.
	// Same path on ARK_G431_CAN (MCU_G431); G431 filter_level schedule is
	// scaled in runtime_loop.c for 160 MHz wall-clock.
	{
		int bad = 0;
		const int tolerance = filter_level >> 2;
		for (int i = 0; i < filter_level; i++) {
			if (getCompOutputLevel() == rising) {
				if (++bad > tolerance) {
					HWCI_PERF_CONFIRM_REJECT();
					return;
				}
			}
		}
	}
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
	//
	// Clocking (INTERVAL_TIMER TIM2 is 2 MHz on both ARK targets):
	//  - F051: TIM1 @ 48 MHz => 24 TIM1 ticks / INTERVAL tick; half off-window
	//    of N TIM1 ticks is N/48 INTERVAL ticks => (N * 1365) >> 16 (~1/48 Q16).
	//    Hump band: ci < 2 * pwm_period => ci * 12 < arr.
	//  - G431: TIM1 @ 160 MHz => 80 TIM1 ticks / INTERVAL tick; half off-window
	//    is N/160 INTERVAL ticks => (N * 409) >> 16 (~1/160 Q16).
	//    Hump band was ci * 40 < arr (~2 PWM periods), matching the F051
	//    scale. Free-run 1800 KV map (noprop_zc_map, 48 kHz) put the measured
	//    jitter peak at ~2.3–3.2 PWM periods (50–70% thr, ~150–210k eRPM) —
	//    outside that band so the compensator never armed. Use ~3.5 PWM
	//    periods: ci * 24 < arr (at 48 kHz arr≈3332 → ci < ~139 ticks).
	const uint32_t zc_hump_mult = 24u;
	const uint32_t zc_half_off_q16 = 409u;
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
		    && (zc_ci * zc_hump_mult) < (uint32_t)zc_arr + 1u				  /* hump band only */
		) {
			uint32_t comp = ((uint32_t)(zc_arr - zc_duty) * zc_half_off_q16) >> 16;
			const uint32_t cap = zc_ci >> 3;
			if (comp > cap) {
				comp = cap;
			}
			zc_grid_comp = (uint16_t)comp;
		}
	}
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
	// steady and fast-looking). Charge the miss bucket and let the
	// consecutive counter drive the same power cut as blind steps
	// (control_loop): less current shortens demag, the pre-level dwell
	// reappears and the loop re-times itself - the firmware equivalent of
	// the throttle-blip escape verified on the bench. Sustained demag-late
	// accepts escalate to the stall rail exactly like a sustained miss
	// rate. Gated to CI > 500 (250 us+), where the healthy pre-level dwell
	// spans many main-loop passes so the sampler cannot miss it.
	//
	// The response is the power cut ONLY - no miss-bucket charge, no stall
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
	zc_blank_until = 0; // the timer restarts at 0 below
	zc_blind_timer_start = 0;
	lastzctime = thiszctime;
	thiszctime = (uint16_t)(INTERVAL_TIMER_COUNT - zc_grid_comp);
	SET_INTERVAL_TIMER_COUNT(0);
	SET_AND_ENABLE_COM_INT(waitTime + 1); // enable COM_TIMER interrupt
	__enable_irq();
	HWCI_PERF_ZC_PHASE_COMMIT(); // accepted edge: bin its PWM phase
}
