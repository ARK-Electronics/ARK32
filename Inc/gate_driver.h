/*
 * gate_driver.h - sleep/wake for DRV8350 ENABLE / DRV8328 nSLEEP
 *
 * Active-high run: low = sleep, high = awake. After rising edge, block ~1 ms
 * (datasheet tWAKE) before PWM/commutation — IC bias must settle; pin stays high.
 */
#ifndef GATE_DRIVER_H_
#define GATE_DRIVER_H_

#include <stdint.h>
#include "targets.h"

#if defined(USE_DRV_ENABLE) || defined(USE_DRV8328_NSLEEP)
#	define GATE_DRIVER_SLEEP_SUPPORT 1

extern volatile uint8_t gate_driver_awake;

void gateDriverInit(void);
void gateDriverWakeBlocking(void);
void gateDriverSleep(void);
void gateDriverFaultResetPulse(void);
void gateDriverPoll(void);

static inline uint8_t gateDriverIsAwake(void)
{
	return gate_driver_awake;
}

static inline void gateDriverEnsure(void)
{
	if (!gate_driver_awake) {
		gateDriverWakeBlocking();
	}
}

#else

#	define GATE_DRIVER_SLEEP_SUPPORT 0

static inline void gateDriverInit(void) {}
static inline void gateDriverWakeBlocking(void) {}
static inline void gateDriverSleep(void) {}
static inline void gateDriverFaultResetPulse(void) {}
static inline void gateDriverPoll(void) {}
static inline uint8_t gateDriverIsAwake(void)
{
	return 1;
}
static inline void gateDriverEnsure(void) {}

#endif

#endif /* GATE_DRIVER_H_ */
