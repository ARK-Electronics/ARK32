/*
 * IO.c
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */

#include "IO.h"

#include "common.h"
#include "dshot.h"
#include "functions.h"
#include "serial_telemetry.h"
#include "targets.h"

uint8_t buffer_padding = 7;
char ic_timer_prescaler = CPU_FREQUENCY_MHZ / 6;
volatile uint32_t dma_buffer[64] = {0};
volatile char out_put = 0;
/* set while the capture waits for a quiet line before re-arming
 * (stm32g4xx_it.c dshotResyncService); any re-arm ends the wait */
volatile uint8_t g4_dshot_resync_pending;

/*
 * Input-capture filter. IC1F counts timer clocks (f_DTS = 160 MHz here), so
 * the AM32 value 0x41 (IC1F 4: f_DTS/2, N=6) is only 75 ns on the G431,
 * against 250 ns on the F051 at 48 MHz. Ringing on a long signal lead could
 * get through as extra edges (suspected on the 2026-11-17 bench, not
 * measured; see stm32g4xx_it.c). For DShot:
 * - DShot600 (prescaler 0): IC1F 7 (f_DTS/4, N=8), 200 ns. Shortest
 *   legitimate level about 417 ns.
 * - DShot300 (prescaler 1): IC1F 8 (f_DTS/8, N=6), 300 ns. Shortest about
 *   833 ns.
 * Both edges pass the same filter, so a pulse width moves by at most one
 * filter sample (25-50 ns), far inside the bit decision. Detection, servo and
 * other inputs keep 0x41.
 */
static inline uint32_t icInputCcmr1(void)
{
	if (ic_timer_prescaler == 0) {
		return 0x71;
	}
	if (ic_timer_prescaler == 1) {
		return 0x81;
	}
	return 0x41;
}

void receiveDshotDma()
{
	g4_dshot_resync_pending = 0;
#ifdef USE_TIMER_3_CHANNEL_1
	RCC->APB1RSTR1 |= LL_APB1_GRP1_PERIPH_TIM3;
	RCC->APB1RSTR1 &= ~LL_APB1_GRP1_PERIPH_TIM3;
	IC_TIMER_REGISTER->CCMR1 = icInputCcmr1();
	IC_TIMER_REGISTER->CCER = 0xa;
#endif
#ifdef USE_TIMER_15_CHANNEL_1
	LL_APB2_GRP1_ForceReset(LL_APB2_GRP1_PERIPH_TIM15);
	LL_APB2_GRP1_ReleaseReset(LL_APB2_GRP1_PERIPH_TIM15);
	IC_TIMER_REGISTER->CCMR1 = icInputCcmr1();
	IC_TIMER_REGISTER->CCER = 0xa;
#endif
#ifdef USE_TIMER_16_CHANNEL_1
	LL_APB2_GRP1_ForceReset(LL_APB2_GRP1_PERIPH_TIM16);
	LL_APB2_GRP1_ReleaseReset(LL_APB2_GRP1_PERIPH_TIM16);
	IC_TIMER_REGISTER->CCMR1 = icInputCcmr1();
	IC_TIMER_REGISTER->CCER = 0xa;
#endif
	IC_TIMER_REGISTER->PSC = ic_timer_prescaler;
	IC_TIMER_REGISTER->ARR = 0xFFFF;
	IC_TIMER_REGISTER->EGR |= TIM_EGR_UG;
	out_put = 0;
	IC_TIMER_REGISTER->CNT = 0;
	DMA1_Channel1->CMAR = (uint32_t)&dma_buffer;
	DMA1_Channel1->CPAR = (uint32_t)&IC_TIMER_REGISTER->CCR1;
	DMA1_Channel1->CNDTR = buffersize;
	DMA1_Channel1->CCR = 0x98b;
	IC_TIMER_REGISTER->DIER |= TIM_DIER_CC1DE;
	IC_TIMER_REGISTER->CCER |= IC_TIMER_CHANNEL;
	IC_TIMER_REGISTER->CR1 |= TIM_CR1_CEN;
}

void sendDshotDma()
{
#ifdef USE_TIMER_3_CHANNEL_1
	RCC->APB1RSTR1 |= LL_APB1_GRP1_PERIPH_TIM3;
	RCC->APB1RSTR1 &= ~LL_APB1_GRP1_PERIPH_TIM3;
	IC_TIMER_REGISTER->CCMR1 = 0x60;
	IC_TIMER_REGISTER->CCER = 0x3;
#endif
#ifdef USE_TIMER_15_CHANNEL_1
	LL_APB2_GRP1_ForceReset(LL_APB2_GRP1_PERIPH_TIM15);
	LL_APB2_GRP1_ReleaseReset(LL_APB2_GRP1_PERIPH_TIM15);
	IC_TIMER_REGISTER->CCMR1 = 0x60;
	IC_TIMER_REGISTER->CCER = 0x3;
#endif
#ifdef USE_TIMER_16_CHANNEL_1
	LL_APB2_GRP1_ForceReset(LL_APB2_GRP1_PERIPH_TIM16);
	LL_APB2_GRP1_ReleaseReset(LL_APB2_GRP1_PERIPH_TIM16);
	IC_TIMER_REGISTER->CCMR1 = 0x60;
	IC_TIMER_REGISTER->CCER = 0x3;
#endif

	IC_TIMER_REGISTER->PSC = output_timer_prescaler;
	IC_TIMER_REGISTER->ARR = 108;
	out_put = 1;
	LL_TIM_GenerateEvent_UPDATE(IC_TIMER_REGISTER);

	DMA1_Channel1->CMAR = (uint32_t)&gcr;
	DMA1_Channel1->CPAR = (uint32_t)&IC_TIMER_REGISTER->CCR1;
	DMA1_Channel1->CNDTR = 23 + buffer_padding;
	DMA1_Channel1->CCR = 0x99b;
	IC_TIMER_REGISTER->DIER |= TIM_DIER_CC1DE;
	IC_TIMER_REGISTER->CCER |= IC_TIMER_CHANNEL;
	IC_TIMER_REGISTER->BDTR |= TIM_BDTR_MOE;
	IC_TIMER_REGISTER->CR1 |= TIM_CR1_CEN;
}

uint8_t getInputPinState()
{
	return (INPUT_PIN_PORT->IDR & INPUT_PIN);
}

void setInputPolarityRising()
{
	LL_TIM_IC_SetPolarity(IC_TIMER_REGISTER, IC_TIMER_CHANNEL, LL_TIM_IC_POLARITY_RISING);
}

void setInputPullDown()
{
	LL_GPIO_SetPinPull(INPUT_PIN_PORT, INPUT_PIN, LL_GPIO_PULL_DOWN);
}

void setInputPullUp()
{
	LL_GPIO_SetPinPull(INPUT_PIN_PORT, INPUT_PIN, LL_GPIO_PULL_UP);
}

void enableHalfTransferInt()
{
	LL_DMA_EnableIT_HT(DMA1, INPUT_DMA_CHANNEL);
}
void setInputPullNone()
{
	LL_GPIO_SetPinPull(INPUT_PIN_PORT, INPUT_PIN, LL_GPIO_PULL_NO);
}
