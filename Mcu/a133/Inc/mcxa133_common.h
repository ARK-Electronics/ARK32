/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: GPL-3.0-only
 */

#ifndef MCU_A133_INC_MCXA133_COMMON_H_
#define MCU_A133_INC_MCXA133_COMMON_H_

#include "main.h"

static inline void modifyReg32(volatile uint32_t *regAddr, uint32_t clearbits, uint32_t setbits)
{
	uint32_t reg = *regAddr;
	reg &= ~clearbits;
	reg |= setbits;
	*regAddr = reg;
}

static inline void modifyReg16(volatile uint16_t *regAddr, uint16_t clearbits, uint16_t setbits)
{
	uint16_t reg = *regAddr;
	reg &= ~clearbits;
	reg |= setbits;
	*regAddr = reg;
}

static inline void modifyReg8(volatile uint8_t *regAddr, uint8_t clearbits, uint8_t setbits)
{
	uint8_t reg = *regAddr;
	reg &= ~clearbits;
	reg |= setbits;
	*regAddr = reg;
}

#endif /* MCU_A133_INC_MCXA133_COMMON_H_ */
