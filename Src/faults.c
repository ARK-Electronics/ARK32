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

/* "Start resisted" counter (see the declaration in faults.h). */
volatile uint8_t fault_acq_resist_events;

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
		// Zero throttle is pilot intervention: clear the acquisition-rail
		// partial batch too, or 19 early desyncs, a pilot cut, then one more
		// early desync on the next blip would count a resisted start as if
		// the cut never happened.
		acq_fail_desyncs = 0;
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
 * Acquisition-rail batching (see faultNoteEarlyDesync in faults.h).
 *
 * 20 early desyncs with no successful acquisition in between count as one
 * resisted start. Deliberately conservative: a healthy hard start spends 2-4
 * rough desyncs acquiring, so 20 is unambiguously abnormal, while at the
 * ~20/s rate a genuinely stuck loop produces it still counts once a second.
 *
 * ACQ_FAIL_CLEAR_ZC is deliberately well above the 100 that arms blind
 * stepping: the count must only be forgiven by an acquisition that actually
 * held, not by one that reached the gate and immediately lost sync again.
 */
#define ACQ_FAIL_DESYNC_LIMIT 20
#define ACQ_FAIL_CLEAR_ZC 500

/*
 * WHY THERE IS NO EPISODE ESCALATOR HERE ANY MORE.
 *
 * This file used to carry an ARK-only leaky bucket across restart cycles:
 * every jump-desync charged 8 and every stall trip charged 12, a bucket over
 * 16 armed a growing allOff() coast (ep2 300 ms, ep3 500 ms on the stall
 * path), and 40 latched ESC_FAULT_STUCK. Bench-reported symptom on a 2160kv
 * article: the ESC cycled between long power cuts and set throttle, got very
 * hot, and could not take off, where stock AM32 on the same article stayed
 * flying through the same induced desyncs.
 *
 * Running the arithmetic explains the report exactly - three stall episodes
 * inside a few seconds buy a 300 ms cut, then a 500 ms cut, then a latch, and
 * the bucket needs 4 s of clean closed loop to drain. None of it is upstream.
 *
 * The reasoning error was treating LOST SYNC and LOST ROTOR as the same
 * event. They are not:
 *
 *   lost sync   - the rotor is turning, the ESC no longer knows where it is.
 *                 Coasting cannot improve this; it only removes the drive
 *                 that would let the loop re-find the rotor, and on a quad it
 *                 is a dropped corner. Upstream rides it out and so do we.
 *   lost rotor  - the rotor is not turning. Driving it dumps locked-rotor
 *                 current into one phase pair, so this one MUST stop.
 *
 * Upstream separates them with two rails and no state in between: the stall
 * rail hands back to poll ZC with the bridge live (faultHandleBemfIntervalStall
 * below), and only faultHandleStuckRotorIfNeeded cuts power, when repeated
 * stalls with no healthy run in between prove poll mode is not finding
 * crossings either. Both are keyed on one throttle-scaled counter,
 * bemf_timeout_happened. We now do exactly that.
 *
 * ARK's robustness over upstream is therefore entirely in FRONT of these
 * rails, not layered on top of them: blind stepping rides out dropouts that
 * would freewheel upstream for up to a full stall window, the dead-reckoning
 * budget in bemf_zc.c bounds how long that may go on, and the authority fade
 * scales duty by remaining confidence. Those reduce how often we reach
 * upstream's rails. What happens once we do reach them is upstream's.
 *
 * The blind-grind rail that also used to live here (sample blind steps every
 * 100 ms, force a restart on one hot window, hold a 250 ms power cut) is gone
 * for the same reason plus one more: it existed only because blind steps
 * reset INTERVAL_TIMER and so hid a grind from the stall rail. bemf_zc.c now
 * measures dead-reckoning time directly in zc_blind_ticks, which restores the
 * stall rail's own visibility without a second rate detector.
 */

void faultNoteEarlyDesync(void)
{
#ifndef BRUSHED_MODE
	if (++acq_fail_desyncs < ACQ_FAIL_DESYNC_LIMIT) {
		return;
	}
	acq_fail_desyncs = 0;
	if (fault_acq_resist_events < 255) {
		fault_acq_resist_events++;
	}
#endif
}

void faultAcqResistTick1kHz(void)
{
#ifndef BRUSHED_MODE
	/* A loop that got solidly established did acquire, whatever roughness
	 * it passed through on the way - forgive the whole batch. */
	if (zero_crosses > ACQ_FAIL_CLEAR_ZC) {
		acq_fail_desyncs = 0;
	}
#endif
}

void faultHandleBemfIntervalStall(void)
{
	/* Six-step only (not sine soft-start). */
	if (INTERVAL_TIMER_COUNT > BEMF_STALL_TICKS && (escInOpenLoop() || escInClosedLoop())) {
		/* Upstream AM32 main.c, "INTERVAL_TIMER_COUNT > 45000", verbatim in
		 * behaviour. bemf_timeout_happened is the counter that carries this
		 * to the genuine stop in faultHandleStuckRotorIfNeeded; nothing here
		 * cuts power or refuses to restart. */
		bemf_timeout_happened++;
		maskPhaseInterrupts();

		/* Counter only, for uavcan esc.Status error_count. Gated on an
		 * ESTABLISHED run (the same zero_crosses > 100 that arms blind
		 * stepping) so the many legitimate kicks a heavy prop needs at low
		 * throttle are not reported as errors. This is also the aggregation
		 * point for the dead-reckoning budget handoff in bemf_zc.c, which
		 * reaches the loop through this trip - see faultErrorCount(). */
		if (zero_crosses > 100) {
			fault_stall_trips++;
		}

		/* old_routine = 1 (hand back to poll ZC); stop ONLY if throttle is
		 * already commanded off, which is upstream's "input < 48" clause. */
		escNoteStallOrDesync(1);
		zero_crosses = 0;
		bemfZcResetTrend();

		/* Re-acquisition must not resume at the commanded setpoint: come
		 * back at the startup floor and let max_ramp carry duty up, the same
		 * resume upstream's desync path uses. Without this the loop re-enters
		 * closed loop at whatever duty it lost sync on, which is the duty
		 * that just failed. */
		last_duty_cycle = min_startup_duty / 2;

		zcfoundroutine();
	}
}
