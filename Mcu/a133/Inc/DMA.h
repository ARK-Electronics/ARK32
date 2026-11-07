/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: GPL-3.0-only
 */

#ifndef MCU_A133_INC_DMA_H_
#define MCU_A133_INC_DMA_H_

#include "main.h"

void initDMA_DshotPWM(void);
void initDMA_ADC(void);
void initDMA_UART(void);

void enableDMA_DshotPWM(void);
void enableDMA_ADC(void);
void enableDMA_UART(void);

void doDshotCorrection(void);

#endif /* MCU_A133_INC_DMA_H_ */
