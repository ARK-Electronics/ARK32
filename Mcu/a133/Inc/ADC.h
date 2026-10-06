/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: GPL-3.0-only
 */

#ifndef ADC_H_
#define ADC_H_

#include "main.h"

uint32_t LPADC_GetGainConvResult(float gainAdjustment);
void calibADC(void);
void enableADC();
void initADC(void);
void startADCConversion(void);
void ADC_DMA_Callback();

int16_t computeTemperature(uint16_t raw_temp_val1, uint16_t raw_temp_val2);

#endif /* ADC_H_ */
