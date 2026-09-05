/*
 * settings.c - extracted from main.c (behavior-neutral)
 */

#include "settings.h"

#include "main.h"
#include "common.h"
#include "motor_runtime.h"
#include "eeprom.h"
#include "eeprom_onload.h"
#include "functions.h"
#include "peripherals.h"
#include "sounds.h"
#include "signal.h"
#include "targets.h"
#include "IO.h"
#include "version.h"
#include "pwm_app.h"

void loadEEpromSettings(void)
{
	read_flash_bin(eepromBuffer.buffer, eeprom_address, sizeof(eepromBuffer.buffer));
	if (eepromBuffer.eeprom_version < EEPROM_VERSION) {
		eepromBuffer.max_ramp_speed = TARGET_DEFAULT_MAX_RAMP; // 0.1% per ms steps (see targets.h)
		eepromBuffer.min_duty_cycle = 1;		       // 0.2% to 51 percent
		eepromBuffer.disable_stick_calibration = 0;	       //
		eepromBuffer.absolute_voltage_cutoff = 10;	       // voltage level 1 to 100 in 0.5v increments
		eepromBuffer.current_pid_p = 100;		       // 0-255
		eepromBuffer.current_pid_i = 0;			       // 0-255
		eepromBuffer.current_pid_d = 100;		       // 0-255
		eepromBuffer.active_brake_power = 0;		       // 1-5 percent duty cycle
		eepromBuffer.reserved0[0] = 0;			       //14-16  for crsf input
		eepromBuffer.reserved0[1] = 0;
		eepromBuffer.reserved0[2] = 0;
		eepromBuffer.reserved0[3] = 0;
	}
	// eepromBuffer.timing_advance can either be set to 0-3 with config tools less than 1.90 or 10-42 with 1.90 or above
	if (eepromBuffer.timing_advance > 42 || (eepromBuffer.timing_advance < 10 && eepromBuffer.timing_advance > 3)) {
		temp_advance = 16;
	}
	if (eepromBuffer.timing_advance < 4) { // old format needs to be converted to 0-32 range
		temp_advance = (eepromBuffer.timing_advance << 3);
		eepromBuffer.timing_advance = temp_advance + 10;
	}
	if (eepromBuffer.timing_advance < 43 && eepromBuffer.timing_advance > 9) { // new format subtract 10 from advance
		temp_advance = eepromBuffer.timing_advance - 10;
	}

	if (eepromBuffer.pwm_frequency < 145 && eepromBuffer.pwm_frequency > 7) {
		int divider = eepromBuffer.pwm_frequency * 100 / 6;
		TIMER1_MAX_ARR = TIM1_AUTORELOAD * 400 / divider;
		SET_AUTO_RELOAD_PWM(TIMER1_MAX_ARR);
	} else {
		tim1_arr = TIM1_AUTORELOAD;
		SET_AUTO_RELOAD_PWM(tim1_arr);
	}
	if (eepromBuffer.min_duty_cycle < 51 && eepromBuffer.min_duty_cycle > 0) {
		minimum_duty_cycle = eepromBuffer.min_duty_cycle * 10;
	} else {
		minimum_duty_cycle = 0;
	}
	if (eepromBuffer.startup_power < 151 && eepromBuffer.startup_power > 49) {
		min_startup_duty = minimum_duty_cycle + eepromBuffer.startup_power;
	} else {
		min_startup_duty = minimum_duty_cycle;
	}
	startup_max_duty_cycle = minimum_duty_cycle + 400;

	motor_kv = (eepromBuffer.motor_kv * 40) + 20;
#ifdef THREE_CELL_MAX
	motor_kv = motor_kv / 2;
#endif
#ifdef ONE_TWO_CELL_MAX
	motor_kv = motor_kv / 16;
#endif
	setVolume(2);
	if (eepromBuffer.eeprom_version > 0) { // these commands weren't introduced until eeprom version 1.
#ifdef CUSTOM_RAMP

#else
		if (eepromBuffer.beep_volume > 11) {
			setVolume(5);
		} else {
			setVolume(eepromBuffer.beep_volume);
		}
#endif
		servo_low_threshold = (eepromBuffer.servo_low_threshold * 2) + 750;    // anything below this point considered 0
		servo_high_threshold = (eepromBuffer.servo_high_threshold * 2) + 1750; // anything above this point considered 2000 (max)
		servo_neutral = (eepromBuffer.servo_neutral) + 1374;
		servo_dead_band = eepromBuffer.servo_deadband;

		low_cell_volt_cutoff = eepromBuffer.low_voltage_threshold + 250; // 2.5 to 3.5 volts per cell range

#ifndef HAS_HALL_SENSORS
		eepromBuffer.hall_sensors = 0;
#endif

		eeprom_apply_on_load(&eepromBuffer);

		if (eepromBuffer.running_brake_level < 10) {
			dead_time_override = DEAD_TIME + (150 - (eepromBuffer.running_brake_level * 10));
			if (dead_time_override > 200) {
				dead_time_override = 200;
			}
			min_startup_duty = min_startup_duty + dead_time_override;
			minimum_duty_cycle = minimum_duty_cycle + dead_time_override;
			throttle_max_at_low_rpm = throttle_max_at_low_rpm + dead_time_override;
			startup_max_duty_cycle = startup_max_duty_cycle + dead_time_override;
			setPwmDeadTime(dead_time_override);
		}

		if (eepromBuffer.current_limit > 0 && eepromBuffer.current_limit <= 100) {
			use_current_limit = 1;
		}

		currentPid.Kp = eepromBuffer.current_pid_p * 2;
		currentPid.Ki = eepromBuffer.current_pid_i;
		currentPid.Kd = eepromBuffer.current_pid_d * 2;

		// unsinged int cant be less than 0
		if (eepromBuffer.input_type < 10) {
			switch (eepromBuffer.input_type) {
				case AUTO_IN:
					dshot = 0;
					servoPwm = 0;
					EDT_ARMED = 1;
					break;
				case DSHOT_IN:
					dshot = 1;
					EDT_ARMED = 1;
					break;
				case SERVO_IN:
					servoPwm = 1;
					break;
				case SERIAL_IN:
					break;
				case EDTARM_IN:
					EDT_ARM_ENABLE = 1;
					EDT_ARMED = 0;
					dshot = 1;
					break;
			};
		} else {
			dshot = 0;
			servoPwm = 0;
			EDT_ARMED = 1;
		}

		// Reset to compile defaults first so this load is idempotent: the
		// eeprom value below only lowers (min semantics). Nothing at
		// runtime writes these - the ramp an ESC flies is the ramp
		// configured here (see faultDesyncEpisodeCharge for why the
		// learned back-off that used to clamp them was removed).
		//
		// NOTE the min semantics flatten the graded schedule: max_ramp 20
		// (the current factory default, 2.0 %/ms) clamps ALL THREE regimes
		// to 2, so the 2/6/16 startup->low->high progression in targets.h
		// never engages and the ESC flies the startup rate at cruise.
		// max_ramp >= 160 is what preserves the full progression.
		max_ramp_startup = RAMP_SPEED_STARTUP;
		max_ramp_low_rpm = RAMP_SPEED_LOW_RPM;
		max_ramp_high_rpm = RAMP_SPEED_HIGH_RPM;
		if (eepromBuffer.max_ramp_speed < 10) {
			ramp_divider = 9;
			max_ramp_startup = eepromBuffer.max_ramp_speed;
			max_ramp_low_rpm = eepromBuffer.max_ramp_speed;
			max_ramp_high_rpm = eepromBuffer.max_ramp_speed;
		} else {
			ramp_divider = 0;
			if ((eepromBuffer.max_ramp_speed / 10) < max_ramp_startup) {
				max_ramp_startup = eepromBuffer.max_ramp_speed / 10;
			}
			if ((eepromBuffer.max_ramp_speed / 10) < max_ramp_low_rpm) {
				max_ramp_low_rpm = eepromBuffer.max_ramp_speed / 10;
			}
			if ((eepromBuffer.max_ramp_speed / 10) < max_ramp_high_rpm) {
				max_ramp_high_rpm = eepromBuffer.max_ramp_speed / 10;
			}
		}

		if (motor_kv < 300) {
			low_rpm_throttle_limit = 0;
		}
		/* Throttle-restriction envelope in kerpm, scaled from kv and pole
		 * count. The envelope is (kv / N) * (motor_poles / 32): multiply
		 * before dividing so the pole term keeps its fractional part -
		 * the old "kv / N / (32 / poles)" form truncated 32/poles to an
		 * integer (2 for the common 14-pole motor instead of 2.29, and 0
		 * for anything above 32 poles, which zeroed the envelope and
		 * removed the restriction entirely).
		 *
		 * The high divisor is 17, not the 12 that shipped in v2.20: 12
		 * raises the erpm at which full duty is released by ~1.42x, so a
		 * correctly configured high-kv motor stays in the restricted
		 * region well past where it should and never reaches full thrust
		 * (am32-firmware/AM32#405 - the community workaround of entering
		 * 50-60% of the real kv cancels exactly this factor).
		 *
		 * Out-of-range pole counts (an erased eeprom reads 0 or 0xff)
		 * leave the envelope at zero, which map() treats as "always at
		 * the high-rpm limit" - i.e. unrestricted, as before.
		 * MOTOR_POLES_MIN..MOTOR_POLES_MAX is the range the DroneCAN
		 * MOTOR_POLES parameter accepts (see eeprom.h). Worst case
		 * 10220 kv * 128 poles / 544 = 2405 kerpm, well inside the
		 * uint16 the levels are stored in. */
		if (eepromBuffer.motor_poles >= MOTOR_POLES_MIN && eepromBuffer.motor_poles <= MOTOR_POLES_MAX) {
			low_rpm_level = ((uint32_t)motor_kv * eepromBuffer.motor_poles) / (100U * 32U);
			high_rpm_level = ((uint32_t)motor_kv * eepromBuffer.motor_poles) / (17U * 32U);
		} else {
			low_rpm_level = 0;
			high_rpm_level = 0;
		}
	}
	/*
	 * Advance-schedule normalization (see motor_runtime.h and the advance block
	 * in runtime_loop.c). Ideal max_erpm at 100% duty is kv * volts * poles/2;
	 * folding the centivolt scale and the Q12 fraction in gives
	 *
	 *   kerpm per centivolt, Q12 = kv * poles * 4096 / 200000
	 *                            = kv * poles * 256 / 12500
	 *
	 * so the hot path is a multiply and a shift with no division. Real free-run
	 * tops out a few percent below that ideal (IR drop, iron loss), which would
	 * leave full-throttle free-run one advance notch short of the duty-curve
	 * top (level 23). Scale the constant by 15/16 once here so the schedule
	 * high end is a realistic free-run ceiling; map() still clamps anything
	 * above it to 23, so a motor that truly hits ideal stays at the top.
	 * Computed after motor_kv has taken its final value (the cell-count
	 * reductions above). Left at 0 - meaning "use the duty proxy" - for a kV
	 * below the range the throttle limiter already treats as unusable, or a
	 * pole count outside the MOTOR_POLES_MIN..MOTOR_POLES_MAX the DroneCAN
	 * MOTOR_POLES parameter accepts (an erased eeprom reads 0 or 0xff).
	 *
	 * The reduced 256/12500 form above is the one evaluated below rather than
	 * the equivalent 4096/200000: both are the same rational number (divided
	 * by 16) so they truncate to the same integer, but the 4096 numerator
	 * overflows uint32 above ~102 poles, while 256 keeps the intermediate
	 * inside uint32 for the whole byte range (10220 * 255 * 256 = 6.7e8,
	 * against 4.29e9). Worst case scale is then 53372, so the 15/16
	 * scale-down and the uint16 it lands in are both safe, as is the hot
	 * path's (scale * centivolts) >> 12 on a 12S pack.
	 */
	advance_erpm_scale_q12 = 0;
	if (motor_kv >= 300 && eepromBuffer.motor_poles >= MOTOR_POLES_MIN && eepromBuffer.motor_poles <= MOTOR_POLES_MAX) {
		uint16_t scale = (uint16_t)(((uint32_t)motor_kv * eepromBuffer.motor_poles * 256u) / 12500u);
		advance_erpm_scale_q12 = (uint16_t)(scale - (scale >> 4)); /* * 15/16 */
	}

	reverse_speed_threshold = map(motor_kv, 300, 3000, 1000, 500);
	if (eepromBuffer.bidirectional_mode) {
		polling_mode_changeover = POLLING_MODE_THRESHOLD / 2;
	} else {
		polling_mode_changeover = POLLING_MODE_THRESHOLD;
	}
}

void saveEEpromSettings(void)
{
	save_flash_nolib(eepromBuffer.buffer, sizeof(eepromBuffer.buffer), eeprom_address);
}

/*
  check device info from the bootloader, confirming pin code and eeprom location
 */
void __attribute__((noinline)) checkDeviceInfo(void)
{
#ifdef MCU_SITL
	// no bootloader device info page in SITL
	return;
#endif
#ifdef NXP
	uint32_t pflashBlockBase = 0U;
	uint32_t pflashTotalSize = 0U;
	uint32_t pflashSectorSize = 0U;

	//Get flash properties
	FLASH_API->flash_get_property(&s_flashDriver, kFLASH_PropertyPflashBlockBaseAddr, &pflashBlockBase);
	FLASH_API->flash_get_property(&s_flashDriver, kFLASH_PropertyPflashSectorSize, &pflashSectorSize);
	FLASH_API->flash_get_property(&s_flashDriver, kFLASH_PropertyPflashTotalSize, &pflashTotalSize);
#else
#	define DEVINFO_MAGIC1 0x5925e3da
#	define DEVINFO_MAGIC2 0x4eb863d9

	// Fixed bootloader address; GCC 12+ -Warray-bounds needs care.
	struct devinfo {
		uint32_t magic1;
		uint32_t magic2;
		uint8_t deviceInfo[9];
	};
	volatile const struct devinfo *devinfo = (volatile const struct devinfo *)(uintptr_t)(0x1000u - 32u);
#	if defined(__GNUC__) && (__GNUC__ >= 12)
#		pragma GCC diagnostic push
#		pragma GCC diagnostic ignored "-Warray-bounds"
#		pragma GCC diagnostic ignored "-Wstringop-overread"
#	endif
	const uint32_t magic1 = devinfo->magic1;
	const uint32_t magic2 = devinfo->magic2;
	const uint8_t eeprom_code = devinfo->deviceInfo[4];
#	if defined(__GNUC__) && (__GNUC__ >= 12)
#		pragma GCC diagnostic pop
#	endif
	if (magic1 != DEVINFO_MAGIC1 || magic2 != DEVINFO_MAGIC2) {
		// bootloader does not support this feature, nothing to do
		return;
	}
	// change eeprom_address based on the code in the bootloaders device info
	switch (eeprom_code) {
		case 0x1f:
			eeprom_address = 0x08007c00;
			break;
		case 0x35:
			eeprom_address = 0x0800f800;
			break;
		case 0x2b:
			eeprom_address = 0x0801f800;
			break;
	}
#endif

	// TODO: check pin code and reboot to bootloader if incorrect
}
