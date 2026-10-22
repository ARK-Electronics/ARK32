/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: GPL-3.0-only
 */

#ifndef COMPARATOR_H_
#define COMPARATOR_H_

#include "main.h"

void initComp0(void);
void initComp1(void);
void enableComparator(void);
void disableComparators(void);
uint8_t getCompOutputLevel();
void maskPhaseInterrupts();
void enableCompInterrupts();
void changeMainComp(LPCMP_Type *CMPx);
void changeCompInput();

extern volatile char rising;
extern char step;

#endif /* COMPARATOR_H_ */
