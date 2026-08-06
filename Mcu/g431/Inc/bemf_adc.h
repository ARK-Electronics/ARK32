/*
 * bemf_adc.h - ADC zero-cross detection for the floating phase (STM32G4)
 *
 * Alternative low-RPM zero-cross detector for ARK_G431_CAN. See bemf_adc.c
 * for what it does and why the comparator cannot do it. Compiled only when
 * the target defines USE_ADC_BEMF; every hook below is a no-op otherwise.
 */
#ifndef BEMF_ADC_H_
#define BEMF_ADC_H_

#include <stdint.h>

#include "targets.h"

#ifdef USE_ADC_BEMF

/* Bring up ADC2's injected group and the TIM1 OC4 trigger that fires it.
 * Call after ADC_Init()/activateADC() (which own the shared ADC12 common
 * block) and after MX_TIM1_Init(). */
void bemfAdcInit(void);

/* Take over the zero-cross search for the step that is about to run, if the
 * ADC path is the better detector right now. Returns 1 when it has taken
 * ownership, in which case the caller must leave the comparator EXTI masked.
 * Called from enableCompInterrupts(). */
uint8_t bemfAdcArm(void);

/* End the search. Called from maskPhaseInterrupts(), so it runs on every
 * accepted crossing, blind step and fault path without extra plumbing. */
void bemfAdcDisarm(void);

/* ISR body, called from ADC1_2_IRQHandler(). */
void bemfAdcIrqHandler(void);

/* Bench instrumentation: searches the ADC path owned, and crossings it
 * committed. A gap between them is a search that timed out into a blind
 * step. */
uint32_t bemfAdcGetSearches(void);
uint32_t bemfAdcGetAccepts(void);

#else

#	define bemfAdcInit() ((void)0)
#	define bemfAdcArm() (0u)
#	define bemfAdcDisarm() ((void)0)

#endif /* USE_ADC_BEMF */

#endif /* BEMF_ADC_H_ */
