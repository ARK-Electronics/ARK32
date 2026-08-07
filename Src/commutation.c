/*
 * commutation.c - extracted from main.c (behavior-neutral split)
 */

#include "commutation.h"
#include "motor_runtime.h"
#include "bemf_zc.h"
#include "main.h"
#include "common.h"
#include "comparator.h"
#include "phaseouts.h"
#include "targets.h"
#include "IO.h"
#include "peripherals.h"
#include "functions.h"
#include "eeprom.h"

RAM_FUNC void getBemfState()
{
	uint8_t current_state = 0;
#if defined(MCU_F031) || defined(MCU_G031)
	if (step == 1 || step == 4) {
		current_state = PHASE_C_EXTI_PORT->IDR & PHASE_C_EXTI_PIN;
	}
	if (step == 2 || step == 5) { //        in phase two or 5 read from phase A Pf1
		current_state = PHASE_A_EXTI_PORT->IDR & PHASE_A_EXTI_PIN;
	}
	if (step == 3 || step == 6) { // phase B pf0
		current_state = PHASE_B_EXTI_PORT->IDR & PHASE_B_EXTI_PIN;
	}
#else
	//Get current comparator output level
	current_state = !getCompOutputLevel(); // polarity reversed
#endif
	if (rising) {
		if (current_state) {
			bemfcounter++;
		} else {
			bad_count++;
			if (bad_count > bad_count_threshold) {
				bemfcounter = 0;
			}
		}
	} else {
		if (!current_state) {
			bemfcounter++;
		} else {
			bad_count++;
			if (bad_count > bad_count_threshold) {
				bemfcounter = 0;
			}
		}
	}
}

RAM_FUNC void commutate()
{
	if (forward == 1) {
		step++;
		if (step > 6) {
			step = 1;
			desync_check = 1;
		}
		rising = step % 2;
	} else {
		step--;
		if (step < 1) {
			step = 6;
			desync_check = 1;
		}
		rising = !(step % 2);
	}
#ifdef INVERTED_EXTI
	rising = !rising;
#endif
	__disable_irq(); // don't let dshot interrupt
	if (!prop_brake_active) {
		comStep(step);
	}
	__enable_irq();
	changeCompInput();
#ifndef NO_POLLING_START
	/*
	 * Poll-mode re-entry (upstream AM32, restored for established run too).
	 *
	 * Layered recovery with blind stepping:
	 *
	 *   1) Missed ZC in closed loop → blind-step at the current estimate
	 *      (bemf_zc.c). Transient dropouts ride through without a mode
	 *      change and without a desync restart - that is the ARK improvement.
	 *
	 *   2) When the measured average interval has gone slow past the poll
	 *      changeover (same test as upstream), fall back to open-loop poll
	 *      ZC at the commanded duty. That is stock recovery: leave interrupt
	 *      closed-loop, hunt BEMF in the 20 kHz poll path, re-enter CL when
	 *      the interval is short enough again (zcfoundroutine / startMotor
	 *      handoff thresholds below).
	 *
	 * Blind does not feed the interval average, so a short run of blinds
	 * does not itself push average_interval into this band. Sustained lost
	 * lock that lengthens the measured average - or a coast through the
	 * changeover - takes the poll exit like upstream. Jump desync and the
	 * stall rail remain separate exits (runtime_loop / faults).
	 *
	 * The zero_crosses < 100 gate that used to wrap this test made poll
	 * re-entry startup-only and left established CL with no upstream-style
	 * soft exit; that is removed. Startup self-heal is unchanged: the same
	 * condition still fires early when noise collapses CI into a fake CL.
	 */
	if (average_interval > polling_mode_changeover + 500) {
		old_routine = 1;
	}
#endif
	bemfcounter = 0;
	zcfound = 0;
	commutation_intervals[step - 1] = commutation_interval; // just used to calulate average

#ifdef USE_PULSE_OUT
	if (step == 1 || step == 4) {
		WRITE_REG(RPM_PULSE_PORT->ODR, READ_REG(RPM_PULSE_PORT->ODR) ^ RPM_PULSE_PIN);
	}
#endif
}

void advanceincrement()
{
	if (!forward) {
		phase_A_position++;
		if (phase_A_position > 359) {
			phase_A_position = 0;
		}
		phase_B_position++;
		if (phase_B_position > 359) {
			phase_B_position = 0;
		}
		phase_C_position++;
		if (phase_C_position > 359) {
			phase_C_position = 0;
		}
	} else {
		phase_A_position--;
		if (phase_A_position < 0) {
			phase_A_position = 359;
		}
		phase_B_position--;
		if (phase_B_position < 0) {
			phase_B_position = 359;
		}
		phase_C_position--;
		if (phase_C_position < 0) {
			phase_C_position = 359;
		}
	}
#ifdef GIMBAL_MODE
	setPWMCompare1(((2 * pwmSinLookup(phase_A_position)) + gate_drive_offset) * TIMER1_MAX_ARR / 2000);
	setPWMCompare2(((2 * pwmSinLookup(phase_B_position)) + gate_drive_offset) * TIMER1_MAX_ARR / 2000);
	setPWMCompare3(((2 * pwmSinLookup(phase_C_position)) + gate_drive_offset) * TIMER1_MAX_ARR / 2000);
#else
	setPWMCompare1((((2 * pwmSinLookup(phase_A_position) / SINE_DIVIDER) + gate_drive_offset) * TIMER1_MAX_ARR / 2000) *
		       eepromBuffer.sine_mode_power / 10);
	setPWMCompare2((((2 * pwmSinLookup(phase_B_position) / SINE_DIVIDER) + gate_drive_offset) * TIMER1_MAX_ARR / 2000) *
		       eepromBuffer.sine_mode_power / 10);
	setPWMCompare3((((2 * pwmSinLookup(phase_C_position) / SINE_DIVIDER) + gate_drive_offset) * TIMER1_MAX_ARR / 2000) *
		       eepromBuffer.sine_mode_power / 10);
#endif
}

void zcfoundroutine()
{ // only used in polling mode, blocking routine.
	thiszctime = INTERVAL_TIMER_COUNT;
	SET_INTERVAL_TIMER_COUNT(0);
	commutation_interval = (thiszctime + (3 * commutation_interval)) / 4;
	advance = (temp_advance * commutation_interval) >> 6; //   7.5 degree increments
	waitTime = bemfZcWaitTimeFromInterval(commutation_interval, advance);
	while ((INTERVAL_TIMER_COUNT) < (waitTime)) {
		if (zero_crosses < 5) {
			break;
		}
	}
#ifdef MCU_GDE23
	TIMER_CAR(COM_TIMER) = waitTime;
#endif
#ifdef STMICRO
	COM_TIMER->ARR = waitTime;
#endif
#ifdef MCU_AT32
	COM_TIMER->pr = waitTime;
#endif
#ifdef NXP
	//	COM_TIMER->MSR[0] = waitTime;
	COM_TIMER->MR[0] = waitTime;
#endif

	commutate();
	bemfcounter = 0;
	bad_count = 0;

	zero_crosses++;
	// Poll → interrupt handoff thresholds (upstream). Closed loop may later
	// fall back to poll via the average_interval test in commutate(); these
	// conditions re-enter interrupt ZC when the ramp is fast enough again.
#ifdef NO_POLLING_START // changes to interrupt mode after 2 zero crosses, does not re-enter
	if (zero_crosses > 2) {
		old_routine = 0;
		enableCompInterrupts(); // enable interrupt
	}
#else
	if (eepromBuffer.stall_protection || eepromBuffer.rc_car_reverse) {
		if (zero_crosses >= 20 && commutation_interval <= 2000) {
			old_routine = 0;
			enableCompInterrupts(); // enable interrupt
		}
	} else {
		if (commutation_interval < polling_mode_changeover) {
			old_routine = 0;
			enableCompInterrupts(); // enable interrupt
		}
	}
#endif
	if (!old_routine) {
		// Fresh closed-loop run (first handoff from poll, or re-entry after
		// poll recovery): a blind-step budget left over from a previous run
		// must not shorten this one, and a stale deadline flag must not
		// misread the first scheduled commutation.
		zc_deadline_armed = 0;
		zc_blind_steps = 0;
		zc_blind_ticks = 0;
		// Fresh closed loop: the interval trend from a previous run (or
		// from the poll ramp) is not a measurement of this one.
		bemfZcResetTrend();
		// Seed the pre-level flag: the main-loop sampler only runs in
		// interrupt mode, so the first closed-loop window has no dwell
		// history yet and must not read as demag-late.
		zc_pre_seen = 1;
		zc_demag_run = 0;
	}
}
