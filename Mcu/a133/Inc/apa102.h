/*
 * Copyright 2026 NXP
 *
 * SPDX-License-Identifier: GPL-3.0-only
 */

#ifndef MCU_A133_INC_APA102_H_
#define MCU_A133_INC_APA102_H_

#include "main.h"

void init_apa102(void);
void send_LED_RGB(uint8_t red, uint8_t green, uint8_t blue);

#endif /* MCU_A133_INC_APA102_H_ */
