/*
 * dshot.h
 *
 *  Created on: Apr. 22, 2020
 *      Author: Alka
 */

#include "main.h"
#include "settings.h"

#ifndef INC_DSHOT_H_
#	define INC_DSHOT_H_

/*
 * DShot special commands (11-bit value 0..47) and throttle range (48..2047).
 *
 * Names follow Betaflight (dshot_command.h / dshot.h) and PX4 (drv_dshot.h)
 * so a grep for DSHOT_CMD_SPIN_DIRECTION_NORMAL (etc.) hits this firmware.
 */

enum {
	DSHOT_CMD_MOTOR_STOP = 0,
	DSHOT_CMD_BEACON1 = 1,
	DSHOT_CMD_BEACON2 = 2,
	DSHOT_CMD_BEACON3 = 3,
	DSHOT_CMD_BEACON4 = 4,
	DSHOT_CMD_BEACON5 = 5,
	DSHOT_CMD_ESC_INFO = 6,
	DSHOT_CMD_SPIN_DIRECTION_1 = 7,
	DSHOT_CMD_SPIN_DIRECTION_2 = 8,
	DSHOT_CMD_3D_MODE_OFF = 9,
	DSHOT_CMD_3D_MODE_ON = 10,
	DSHOT_CMD_SETTINGS_REQUEST = 11, /* not implemented (Betaflight/PX4 same) */
	DSHOT_CMD_SAVE_SETTINGS = 12,
	DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE = 13,
	DSHOT_CMD_EXTENDED_TELEMETRY_DISABLE = 14,
	DSHOT_CMD_SPIN_DIRECTION_NORMAL = 20,
	DSHOT_CMD_SPIN_DIRECTION_REVERSED = 21,
	DSHOT_CMD_LED0_ON = 22, /* BLHeli32 only; not handled here */
	DSHOT_CMD_LED1_ON = 23,
	DSHOT_CMD_LED2_ON = 24,
	DSHOT_CMD_LED3_ON = 25,
	DSHOT_CMD_LED0_OFF = 26,
	DSHOT_CMD_LED1_OFF = 27,
	DSHOT_CMD_LED2_OFF = 28,
	DSHOT_CMD_LED3_OFF = 29,
	DSHOT_CMD_AUDIO_STREAM_MODE_ON_OFF = 30, /* KISS; not handled here */
	DSHOT_CMD_SILENT_MODE_ON_OFF = 31,
	DSHOT_CMD_ENTER_PROGRAMMING_MODE = 36, /* AM32 eeprom write (PX4) */
	DSHOT_CMD_EXIT_PROGRAMMING_MODE = 37,
	DSHOT_CMD_MAX = 47, /* last command; > this is throttle */
};

/* Throttle range. Betaflight dshot.h names. */
#	define DSHOT_MIN_THROTTLE 48
#	define DSHOT_MAX_THROTTLE 2047
#	define DSHOT_3D_FORWARD_MIN_THROTTLE 1048
#	define DSHOT_3D_REVERSE_MAX (DSHOT_3D_FORWARD_MIN_THROTTLE - 1)

/* Consecutive identical command frames required (beacons fire immediately). */
#	define DSHOT_CMD_REPEATS 6

/* Frame layout: [11-bit value][telem request][4-bit crc] */
#	define DSHOT_TELEMETRY_BIT 0x10
#	define DSHOT_VALUE_SHIFT 5

/* Extended DShot Telemetry type field (matches PX4 DSHOT_EDT_*). */
enum {
	DSHOT_EDT_ERPM = 0x00,
	DSHOT_EDT_TEMPERATURE = 0x02, /* deg C */
	DSHOT_EDT_VOLTAGE = 0x04,     /* 0.25 V / step */
	DSHOT_EDT_CURRENT = 0x06,     /* 1 A / step */
	DSHOT_EDT_DEBUG1 = 0x08,
	DSHOT_EDT_DEBUG2 = 0x0A,
	DSHOT_EDT_DEBUG3 = 0x0C,
	DSHOT_EDT_STATE_EVENT = 0x0E,
};

#	define DSHOT_EDT_PACK(type, data) ((uint16_t)(((type) << 8) | ((uint8_t)(data))))

enum {
	DSHOT_PROG_IDLE = 0,
	DSHOT_PROG_WAIT_ADDRESS = 1,
	DSHOT_PROG_WAIT_VALUE = 2,
	DSHOT_PROG_WAIT_COMMIT = 3,
};

/* PX4 / existing AM32 Python aliases for the same values. */
#	define DSHOT_CMD_BEEP1 DSHOT_CMD_BEACON1
#	define DSHOT_CMD_EDT_ENABLE DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE
#	define DSHOT_CMD_EDT_DISABLE DSHOT_CMD_EXTENDED_TELEMETRY_DISABLE
#	define DSHOT_EXTENDED_TELEMETRY_ENABLE DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE
#	define DSHOT_CMD_MIN_THROTTLE DSHOT_MIN_THROTTLE
#	define DSHOT_CMD_MAX_THROTTLE DSHOT_MAX_THROTTLE

void computeDshotDMA(void);
void make_dshot_package(uint16_t com_time);

extern void playInputTune(void);
extern void playInputTune2(void);
extern void playBeaconTune3(void);

extern volatile char dshot_telemetry;
extern volatile char armed;
extern char buffer_divider;
extern uint8_t last_dshot_command;
extern volatile uint32_t commutation_interval;

#endif /* INC_DSHOT_H_ */
