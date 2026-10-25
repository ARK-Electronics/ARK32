#pragma once

#include "eeprom_layout.h"

/* Validate before publishing a settings byte to IRQ/main-loop consumers. */
static inline uint8_t sanitizeMotorPoles(uint8_t poles)
{
	return poles >= MOTOR_POLES_MIN && poles <= MOTOR_POLES_MAX ? poles : 14;
}

void read_flash_bin(uint8_t *data, uint32_t add, int out_buff_len);
void save_flash_nolib(uint8_t *data, int length, uint32_t add);
extern EEprom_t eepromBuffer;
