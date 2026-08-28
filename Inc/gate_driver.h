/*
 * gate_driver.h - sleep/wake for DRV8350 ENABLE / DRV8328 nSLEEP
 *
 * Active-high run: low = sleep, high = awake. After rising edge, block for
 * t_WAKE settle (1 ms DRV8328; 3 ms DRV8350H) before PWM/commutation.
 *
 * DRV8350H (ARK 12S CAN, USE_DRV_ENABLE): wake forces bridge inputs inactive
 * (allOff + duty 0) before ENABLE rises — avoids the common sleep-exit blip.
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
#	if defined(USE_DRV_ENABLE)
void gateDriverFaultResetPulse(void);
#	else
/* DRV8328 never calls this (nSLEEP sleep-on-idle is the latch reset). */
static inline void gateDriverFaultResetPulse(void) {}
#	endif
void gateDriverPoll(void);
/* 1 when ENABLE/nSLEEP is high and nFAULT is past the post-wake settle.
 * Pin is asserted in sleep (VCP UVLO) and can glitch on the first PWM. */
uint8_t gateDriverNfaultPinTrusted(void);
void gateDriverNfaultGraceTick(void);

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
static inline uint8_t gateDriverNfaultPinTrusted(void)
{
	return 1;
}
static inline void gateDriverNfaultGraceTick(void) {}
static inline void gateDriverEnsure(void) {}

#endif

#endif /* GATE_DRIVER_H_ */
