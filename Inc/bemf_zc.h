/*
 * bemf_zc.h - zero-cross detection and commutation scheduling
 */
#ifndef BEMF_ZC_H_
#define BEMF_ZC_H_

#include <stdint.h>

void PeriodElapsedCallback(void);
void interruptRoutine(void);
void startMotor(void);
/* Clear the acceleration trend used for the commutation point (bemf_zc.c).
 * Call next to every zero_crosses = 0 so a desync/coast cannot leave a
 * stale EMA that would bias the next closed-loop run once ZC_TREND_MIN_ZC
 * re-arms. */
void bemfZcResetTrend(void);
/* SITL / debug snapshots of the predictor consumer path. */
int32_t bemfZcGetTrend(void);
uint32_t bemfZcGetPredicted(void);

/* Saturating waitTime = interval/2 - advance (never underflows). Inline so
 * PeriodElapsedCallback (RAM_FUNC) does not call out to flash for it. */
static inline uint16_t bemfZcWaitTimeFromInterval(uint32_t interval, uint16_t advance_ticks)
{
	const uint32_t half = interval >> 1;
	if (advance_ticks >= half) {
		return 0;
	}
	const uint32_t wt = half - (uint32_t)advance_ticks;
	return (wt > 65535u) ? 65535u : (uint16_t)wt;
}

#endif /* BEMF_ZC_H_ */
