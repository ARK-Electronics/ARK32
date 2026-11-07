/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: GPL-3.0-only
 */

#ifndef MCU_A133_INC_MCXA133_IT_H_
#define MCU_A133_INC_MCXA133_IT_H_

#include "main.h"

void GPIO1_IRQHandler(void);
void GPIO2_IRQHandler(void);

void CMP0_IRQHandler(void);
void CMP1_IRQHandler(void);
void LPTMR0_IRQHandler(void);
void DMA_CH0_IRQHandler(void);
void DMA_CH1_IRQHandler(void);
void LPSPI0_IRQHandler(void);

#endif /* MCU_A133_INC_MCXA133_IT_H_ */
