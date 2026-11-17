
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
 * DShot framing check (G431 only; the F051 decode path is unchanged).
 *
 * The decoder (computeDshotDMA) frames a DShot packet as the 32 edges captured
 * after each re-arm and checks only the 16-bit span dma_buffer[31]-[0] and the
 * nibble-XOR CRC. Every rotation of a valid frame, inverted or not, is
 * CRC-valid, and on the G431 the capture timer (80-160 MHz, 16-bit) wraps
 * every 410-820 us, so a window that started mid-frame or straddled two frames
 * could pass the span window by aliasing. The TC interrupt is also blocked by
 * the 20 kHz control loop (TIM6 priority 0, versus 3 on the F051), so late
 * re-arms land mid-frame. Bench 2026-11-16: 52 CRC-valid wrong-throttle
 * events (2-7.5x for 5-40 ms) in 84 min of stand DShot on the G431, none in
 * 68 min on the F051.
 *
 * A correctly framed window starts on a bit-start edge, so every even edge
 * pair dma_buffer[2i+2]-dma_buffer[2i] is one bit period. A window offset by
 * one edge measures bit-end to bit-end, which moves by about +-0.375 T wherever
 * adjacent bits differ, and a window straddling a frame gap carries the gap in
 * one interval. Reject such windows before decode. Polarity-agnostic, so
 * bidirectional (inverted) DShot works the same. A whole-bit rotation of a
 * truly gapless stream is undetectable by any receiver.
 */
volatile uint32_t g4_dshot_framing_errors;
/* consecutive rejects (resync time) and the worst run seen, for the bench */
volatile uint16_t g4_dshot_reject_run;
volatile uint16_t g4_dshot_reject_run_max;
extern void dshotFramingRejected(void);

static void dshotRejectFrame(void)
{
	g4_dshot_framing_errors++;
	if (++g4_dshot_reject_run > g4_dshot_reject_run_max) {
		g4_dshot_reject_run_max = g4_dshot_reject_run;
	}
	dshotFramingRejected(); /* "consecutive valid frames" restarts, as on a CRC failure */
	receiveDshotDma();
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
		DMA1->IFCR |= DMA_IFCR_CGIF1;
		DMA1_Channel1->CCR = 0x00;
		if (!out_put) {
			if (!dshotWindowAligned()) {
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
		if (dshot && inputSet == 1 && buffersize == 32u) {
			if (!dshotWindowAligned()) {
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
