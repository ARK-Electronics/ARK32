
#include "stm32g4xx_it.h"
#include "ADC.h"
#include "IO.h"
#include "WS2812.h"
#include "main.h"
#include "targets.h"
#include "common.h"
#include "signal.h"

extern void transfercomplete();
extern void PeriodElapsedCallback();
extern void interruptRoutine();
extern void tenKhzRoutine();
extern void processDshot();

extern volatile char send_telemetry;
uint16_t interrupt_time = 0;
extern volatile char servoPwm;
extern volatile char dshot_telemetry;
extern volatile char armed;
extern volatile char out_put;
extern volatile uint8_t compute_dshot_flag;
extern volatile uint32_t commutation_interval;

int interrupt = 0;

/*
 * DShot capture framing (G431 only; the F051 decode path is unchanged).
 *
 * The decoder (computeDshotDMA) frames a DShot packet as the 32 edges captured
 * after each re-arm, and checks only the 16-bit span dma_buffer[31]-[0] and the
 * nibble-XOR CRC. Every rotation of a valid frame, inverted or not, is
 * CRC-valid. On the G431 the capture timer (80-160 MHz, 16-bit) wraps every
 * 410-820 us, less than common frame periods, so a window that starts inside
 * one frame and ends inside the next can pass the span check by aliasing.
 *
 * Bench, 12S CAN ESC + 33in prop, Flight Stand DShot300 on a long lead:
 * - 2026-11-16, old firmware: 52 CRC-valid wrong-throttle events (2-7.5x for
 *   5-40 ms) in 84 min on the G431, none in 68 min on the F051.
 * - 2026-11-17, with the per-bit check below: one escape (155 -> 620, a 2-bit
 *   rotation of 0x1364; 66 A). Windows also stayed out of step for up to ~450
 *   frames (up to ~460 ms without a good frame), because a window counts edges
 *   and was re-armed straight away.
 *
 * Three checks, all polarity-agnostic (bidirectional DShot works the same):
 * 1. dshotWindowAligned(): the 15 bit-start intervals must all equal span/15
 *    +-20 %, which rejects windows offset by an edge.
 * 2. dshotWindowEndsFrame(): no further edge within one bit after the 32nd.
 *    A window that started inside a frame ends inside one, and the next edge
 *    follows within 0.75 bit. One bit is far below the timer wrap, so this
 *    cannot alias: it is what makes a rotated window fail deterministically.
 *    Accept only if no edge at all was captured in that bit (CC1IF and CC1OF
 *    clear; both are sticky until software clears them), so preemption
 *    between reads cannot let a later edge stand in for an earlier one.
 * 3. dshotRejectFrame() / dshotResyncService(): after a reject, stop DMA
 *    requests but keep capturing, and re-arm from the 20 kHz TIM6 tick once the
 *    line has been quiet for two bits, so the next window starts at a frame
 *    boundary. Nothing spins in this interrupt (with bidirectional DShot it can
 *    run at priority 0, above commutation; see runtimeUpdateDshotIrqPriority).
 *    If the line never goes quiet, reception stays off and the normal signal
 *    timeout applies.
 * The input filter (Mcu/g431/Src/IO.c) is a mitigation for ringing on long
 * leads (suspected on the stand, not measured).
 */

/* One DShot bit in capture counts. checkDshot() (Src/signal.c) picks
 * prescaler 0 for DShot600 and 1 for DShot300, so at 160 MHz a bit is 266.7
 * counts at either rate. A constant, not a value taken from a window that may
 * be broken. */
#define G4_DSHOT_BIT_COUNTS ((uint16_t)((CPU_FREQUENCY_MHZ * 5u) / 3u))

volatile uint32_t g4_dshot_framing_errors;   /* all rejected windows */
volatile uint32_t g4_dshot_tail_rejects;     /* of those: an edge followed within one bit */
volatile uint32_t g4_dshot_resync_ticks_max; /* longest wait for a quiet line, in 50 us ticks */
/* consecutive rejects (resync time) and the worst run seen, for the bench */
volatile uint16_t g4_dshot_reject_run;
volatile uint16_t g4_dshot_reject_run_max;
extern volatile uint8_t g4_dshot_resync_pending; /* IO.c: cleared by receiveDshotDma() */
extern void dshotFramingRejected(void);
static uint16_t resync_quiet_from; /* capture count of the last edge or flag clear */
static uint32_t resync_ticks;

static void dshotRejectFrame(void)
{
	g4_dshot_framing_errors++;
	if (++g4_dshot_reject_run > g4_dshot_reject_run_max) {
		g4_dshot_reject_run_max = g4_dshot_reject_run;
	}
	dshotFramingRejected(); /* "consecutive valid frames" restarts, as on a CRC failure */

	/* Keep TIM16 capturing with DMA requests off, and hand the re-arm to the
	 * TIM6 tick (dshotResyncService). The channel is already disabled. */
	IC_TIMER_REGISTER->DIER &= ~TIM_DIER_CC1DE;
	(void)IC_TIMER_REGISTER->CCR1;	       /* clears CC1IF */
	IC_TIMER_REGISTER->SR = ~TIM_SR_CC1OF; /* rc_w0 */
	resync_quiet_from = (uint16_t)IC_TIMER_REGISTER->CNT;
	resync_ticks = 0;
	g4_dshot_resync_pending = 1;
}

/* From the 20 kHz TIM6 interrupt. Re-arm once there has been no edge since the
 * previous look and at least two bits have passed since the last activity:
 * inside a frame an edge comes every 0.75 bit or sooner, so that is the gap
 * between frames. */
static void dshotResyncService(void)
{
	if (!g4_dshot_resync_pending) {
		return;
	}
	if ((IC_TIMER_REGISTER->CCMR1 & TIM_CCMR1_CC1S) != TIM_CCMR1_CC1S_0) {
		/* the timer is not capturing (a telemetry reply owns it); its flags
		 * mean nothing here, and whoever owns it re-arms receive when done */
		g4_dshot_resync_pending = 0;
		return;
	}
	if (++resync_ticks > g4_dshot_resync_ticks_max) {
		g4_dshot_resync_ticks_max = resync_ticks;
	}
	if ((IC_TIMER_REGISTER->SR & (TIM_SR_CC1IF | TIM_SR_CC1OF)) != 0u) {
		resync_quiet_from = (uint16_t)IC_TIMER_REGISTER->CCR1; /* latest edge; clears CC1IF */
		IC_TIMER_REGISTER->SR = ~TIM_SR_CC1OF;
		return;
	}
	if ((uint16_t)((uint16_t)IC_TIMER_REGISTER->CNT - resync_quiet_from) < (uint16_t)(2u * G4_DSHOT_BIT_COUNTS)) {
		return;
	}
	receiveDshotDma(); /* clears g4_dshot_resync_pending */
}

static inline uint8_t dshotWindowAligned(void)
{
	const uint16_t bit = (uint16_t)((uint16_t)dma_buffer[30] - (uint16_t)dma_buffer[0]) / 15u;
	/* 20 % of a bit: a one-edge offset moves an interval by about 37.5 %,
	 * and DMA-timed DShot from a flight controller or stand jitters far less */
	const uint16_t tol = (uint16_t)(bit / 5u);
	for (uint32_t i = 0; i < 30u; i += 2u) {
		const uint16_t d = (uint16_t)((uint16_t)dma_buffer[i + 2u] - (uint16_t)dma_buffer[i]);
		if ((uint16_t)(d - bit + tol) > (uint16_t)(2u * tol)) {
			return 0;
		}
	}
	return 1;
}

/* The DMA's read of the 32nd capture cleared CC1IF, so a set CC1IF (or CC1OF,
 * also set if a capture was lost during the window) means an edge after or
 * inside the window. Wait until a bit has passed since the 32nd edge (normally
 * 1-3 us of this interrupt), then look once. A late interrupt that already
 * sees the next frame's first edge drops a good frame; the resync recovers. */
static inline uint8_t dshotWindowEndsFrame(void)
{
	const uint16_t last = (uint16_t)dma_buffer[31];
	uint32_t guard = 0;
	while ((uint16_t)((uint16_t)IC_TIMER_REGISTER->CNT - last) < G4_DSHOT_BIT_COUNTS) {
		if (++guard >= 4096u) {
			return 0; /* could not establish the quiet bit: fail closed */
		}
	}
	return (IC_TIMER_REGISTER->SR & (TIM_SR_CC1IF | TIM_SR_CC1OF)) == 0u;
}

/* Both framing checks; counts which one fired. */
static inline uint8_t dshotWindowOk(void)
{
	if (!dshotWindowAligned()) {
		return 0;
	}
	if (!dshotWindowEndsFrame()) {
		g4_dshot_tail_rejects++;
		return 0;
	}
	return 1;
}

void NMI_Handler(void)
{
	/* A flash double-ECC error also raises NMI on the G4. The bootloader
	 * migration (Src/bootloader_update.c) reads the bootloader region, which
	 * an interrupted rewrite can leave with torn doublewords: clear the error
	 * and carry on (the compare then fails and the region is rewritten). */
	if ((FLASH->ECCR & FLASH_ECCR_ECCD) != 0u) {
		FLASH->ECCR |= FLASH_ECCR_ECCD;
		return;
	}
	while (1) {}
}

void HardFault_Handler(void)
{
	while (1) {}
}

void MemManage_Handler(void)
{
	while (1) {}
}

void BusFault_Handler(void)
{
	while (1) {}
}

void UsageFault_Handler(void)
{
	while (1) {}
}

void SVC_Handler(void) {}

void DebugMon_Handler(void) {}

void PendSV_Handler(void) {}

void SysTick_Handler(void) {}

void DMA1_Channel1_IRQHandler(void)
{
	if (armed && dshot_telemetry) {
		const uint8_t complete = (DMA1->ISR & DMA_ISR_TCIF1) != 0u;
		DMA1->IFCR |= DMA_IFCR_CGIF1;
		DMA1_Channel1->CCR = 0x00;
		if (!out_put) {
			if (!complete || !dshotWindowOk()) {
				/* misframed receive: drop it, no telemetry reply, listen again */
				dshotRejectFrame();
				return;
			}
			g4_dshot_reject_run = 0;
		}
		if (out_put) {
			receiveDshotDma();
			compute_dshot_flag = 2;
		} else {
			sendDshotDma();
			compute_dshot_flag = 1;
		}
		EXTI->SWIER1 |= LL_EXTI_LINE_15;

		return;
	}
	/* USER CODE BEGIN DMA1_Channel1_IRQn 0 */
	if (LL_DMA_IsActiveFlag_HT1(DMA1)) {
		if (servoPwm) {
			LL_TIM_IC_SetPolarity(IC_TIMER_REGISTER, IC_TIMER_CHANNEL, LL_TIM_IC_POLARITY_FALLING);
			LL_DMA_ClearFlag_HT1(DMA1);
		}
	}
	if (LL_DMA_IsActiveFlag_TC1(DMA1) == 1) {
		DMA1->IFCR |= DMA_IFCR_CGIF1;
		DMA1_Channel1->CCR = 0x00;
#if DRONECAN_SUPPORT
		if (!out_put && autoWireDetectOnCapture()) {
			return; /* RawCommand came first: detect the wire protocol (Src/signal.c) */
		}
#endif
		/* receive completions only: before arming, a bidirectional ESC's
		 * telemetry reply also ends here (out_put set, the timer in output
		 * mode), and its flags say nothing about a received window */
		if (dshot && inputSet == 1 && buffersize == 32u && !out_put) {
			if (!dshotWindowOk()) {
				/* misframed: drop it before decode and re-arm */
				dshotRejectFrame();
				return;
			}
			g4_dshot_reject_run = 0;
		}
		transfercomplete();
		EXTI->SWIER1 |= LL_EXTI_LINE_15;
	} else if (LL_DMA_IsActiveFlag_TE1(DMA1) == 1) {
		LL_DMA_ClearFlag_GI1(DMA1);
	}
}

void COMP1_2_3_IRQHandler(void)
{
	if (INTERVAL_TIMER->CNT > ZC_SEARCH_BLANK(commutation_interval)) {
		interrupt++;
		if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_22)) {
			LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_22);
			interruptRoutine();
			return;
		}

		if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_21)) {
			LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_21);
			interruptRoutine();
			return;
		}
	} else {
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_21);
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_22);
	}
}

void TIM6_DAC_IRQHandler(void)
{
	if (LL_TIM_IsActiveFlag_UPDATE(TIM6) == 1) {
		LL_TIM_ClearFlag_UPDATE(TIM6);
		dshotResyncService();
		tenKhzRoutine();
	}
}

#ifdef USE_TIMER_16_CHANNEL_1
void TIM7_IRQHandler(void)
{
	PeriodElapsedCallback();
	LL_TIM_ClearFlag_UPDATE(TIM7);
}
#else
void TIM1_UP_TIM16_IRQHandler(void)
{
	PeriodElapsedCallback();
	LL_TIM_ClearFlag_UPDATE(TIM16);
}
#endif

void EXTI15_10_IRQHandler(void)
{
	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_15);
	processDshot();
}

void DMA1_Channel3_IRQHandler(void) {}
