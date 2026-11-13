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
#include "zc_frontend.h"

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
 * Blind steps do not feed the trend - thiszctime is the expected interval
 * there, not a measurement. Nor do the next two real crossings: the first
 * is timed from the blind step's virtual crossing rather than a real one,
 * and both of their differences include it. With the old blind step (the
 * deadline's 1.5x elapsed time as the interval, timer restarted at the
 * blind commutation) feeding them in pulled the trend to about -CI/4 after
 * a short dropout (SITL), and alternating misses would pin the correction
 * at -CI/2: real commutations 6-15 degrees early. Now that the virtual
 * crossing sits where the crossing was expected, those differences are
 * close to true ones and the hold is defence in depth.
 */
#define ZC_TREND_MIN_ZC 20
static int32_t zc_trend;
static uint8_t zc_trend_hold; /* real crossings to skip after a blind step */
/* After a blind commutation: INTERVAL_TIMER value the timer restarted at,
 * so the next blind step adds real time to zc_blind_ticks... */
static uint16_t zc_blind_timer_start;
/* ...and the INTERVAL_TIMER value before which no crossing is accepted, the
 * post-commutation blanking a real crossing's commutation gets from the
 * comparator gate (0 = none). */
static uint16_t zc_blank_until;
/* Last predicted interval used for advance/waitTime (observability + SITL). */
static uint32_t zc_predicted;

int32_t bemfZcGetTrend(void)
{
	return zc_trend;
}

uint32_t bemfZcGetPredicted(void)
{
	return zc_predicted;
}

/* Per-MCU front end: Inc/zc_frontend.h. */
#include ZC_FRONTEND_IMPL

void bemfZcResetTrend(void)
{
	zc_trend = 0;
	zc_trend_hold = 0;
	zc_predicted = 0;
	zc_blank_until = 0;
}

void startMotor()
{
	if (running == 0) {
		DISABLE_COM_TIMER_INT(); // a stale blind-step deadline must not commutate the restart
		zc_deadline_armed = 0;
		zc_blind_steps = 0;
		zc_blind_ticks = 0;
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
