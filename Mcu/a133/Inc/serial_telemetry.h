/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: GPL-3.0-only
 */

#ifndef SERIAL_TELEMETRY_H_
#define SERIAL_TELEMETRY_H_

#include "main.h"

void telem_UART_Init(void);
void enable_telem_UART(void);
void send_telem_DMA(uint8_t bytes);

void telem_UART_Init_CH4(void);
void send_telem_DMA_CH4();

#endif /* SERIAL_TELEMETRY_H_ */
