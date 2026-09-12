#include "main.h"

#pragma once

#ifndef EEPROM_H_
#	define EEPROM_H_

/*
 * Configurable magnet pole count (eeprom byte 27, not pole pairs).
 *
 * The minimum keeps the pole-pair math (motor_poles / 2) non-zero. The maximum
 * is what the DroneCAN MOTOR_POLES parameter accepts and what the kv-derived
 * schedules in settings.c treat as a plausible pole count; it covers the large
 * high-pole-count outrunners and direct-drive/hub motors that go well past the
 * 36 poles the configurators historically allowed. It stays below 0xff so an
 * erased eeprom (0 or 0xff) still reads as out of range and the derived
 * schedules fall back to their unrestricted / duty-proxy behavior.
 */
#	define MOTOR_POLES_MIN 2
#	define MOTOR_POLES_MAX 128

typedef union EEprom_u {
	struct {
		uint8_t reserved_0;	//0
		uint8_t eeprom_version; //1
		uint8_t reserved_1;	//2
		struct {
			uint8_t major; //3
			uint8_t minor; //4
		} version;
		uint8_t max_ramp;		   //5 0.1% per ms to 25% per ms
		uint8_t minimum_duty_cycle;	   //6  0.2% to 51 percent
		uint8_t disable_stick_calibration; //7
		uint8_t absolute_voltage_cutoff;   //8  voltage level 1 to 100 in 0.5v increments
		uint8_t current_P;		   //9  0-255
		uint8_t current_I;		   //10 0-255
		uint8_t current_D;		   //11 0-255
		uint8_t active_brake_power;	   //12  1-5 percent duty cycle
		char reserved_eeprom_3[4];	   //13-16
		uint8_t dir_reversed;		   // 17
		uint8_t bi_direction;		   // 18
		uint8_t use_sine_start;		   // 19
		uint8_t comp_pwm;		   // 20
		uint8_t variable_pwm;		   // 21
		uint8_t stuck_rotor_protection;	   // 22
		uint8_t advance_level;		   // 23
		uint8_t pwm_frequency;		   // 24
		uint8_t startup_power;		   // 25
		uint8_t motor_kv;		   // 26
		uint8_t motor_poles;		   // 27
		uint8_t brake_on_stop;		   // 28
		uint8_t stall_protection;	   // 29
		uint8_t beep_volume;		   // 30
		uint8_t telemetry_on_interval;	   // 31
		struct {
			uint8_t low_threshold;	// 32
			uint8_t high_threshold; // 33
			uint8_t neutral;	// 34
			uint8_t dead_band;	// 35
		} servo;
		uint8_t low_voltage_cut_off;		    // 36
		uint8_t low_cell_volt_cutoff;		    // 37
		uint8_t rc_car_reverse;			    // 38
		uint8_t use_hall_sensors;		    // 39
		uint8_t sine_mode_changeover_thottle_level; // 40
		uint8_t drag_brake_strength;		    // 41
		uint8_t driving_brake_strength;		    // 42
		struct {
			uint8_t temperature; // 43
			uint8_t current;     // 44
		} limits;
		uint8_t sine_mode_power; // 45
		uint8_t input_type;	 // 46
		uint8_t auto_advance;	 // 47
		uint8_t tune[128];	 // 48-175
		struct {
			uint8_t can_node;	       // 176
			uint8_t esc_index;	       // 177
			uint8_t require_arming;	       // 178
			uint8_t telem_rate;	       // 179
			uint8_t require_zero_throttle; // 180
			uint8_t filter_hz;	       // 181
			uint8_t debug_rate;	       // 182
			uint8_t term_enable;	       // 183
			/* Width in C of the thermal foldback ramp below full
			 * derate; the onset is limits.temperature. Lives in the
			 * can block because that is where the free bytes are -
			 * the limiter itself is target-agnostic, and 0xFF on a
			 * board that never wrote this page is coerced in
			 * settings.c. */
			uint8_t temp_derate_band; // 184
			/* CAN FD data-phase bitrate in Mbps: 0 = auto-match
			 * the host (1/2/4/5), 1/2/4/5 = pinned. Byte 185. */
			uint8_t fd_mbps;     // 185
			uint8_t reserved[6]; // 186-191
		} can;
	};
	uint8_t buffer[192];
} EEprom_t;

extern EEprom_t eepromBuffer;

// void save_to_flash(uint8_t *data);
// void read_flash(uint8_t* data, uint32_t address);
// void save_to_flash_bin(uint8_t *data, int length, uint32_t add);
void read_flash_bin(uint8_t *data, uint32_t add, int out_buff_len);
void save_flash_nolib(uint8_t *data, int length, uint32_t add);

#endif /* EEPROM_H_ */
