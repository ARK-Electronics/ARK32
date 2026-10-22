/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: GPL-3.0-only
 */

#ifndef PERIPHERALS_H_
#define PERIPHERALS_H_

#include "main.h"

void initAfterJump(void);
void initCorePeripherals(void);
void SystemInit(void);
void SystemClock_Config(void);
void initGPIO(void);
void initSPI(void);
void enableCorePeripherals(void);

#endif /* PERIPHERALS_H_ */
