/*
 * IO.c
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */

#include "signal.h"
#include "IO.h"
#include "common.h"
#include "dshot.h"
#include "functions.h"
#include "serial_telemetry.h"
#include "sounds.h"
#include "targets.h"
#include "faults.h"
#if DRONECAN_SUPPORT
#	include "DroneCAN/DroneCAN.h"
#endif
int max_servo_deviation = 250;
int servorawinput;
uint16_t smallestnumber = 20000;
uint8_t enter_calibration_count = 0;
uint8_t calibration_required = 0;
uint8_t high_calibration_counts = 0;
uint8_t high_calibration_set = 0;
uint16_t last_high_threshold = 0;
uint8_t low_calibration_counts = 0;
uint16_t last_input = 0;
char output_timer_prescaler;
uint8_t buffersize = 32;
uint32_t average_signal_pulse;
uint8_t average_count;
uint32_t average_packet_length;
uint16_t dshot_frametime_high = 50000;
uint16_t dshot_frametime_low = 0;

void computeServoInput()
{
	if (((dma_buffer[1] - dma_buffer[0]) > 800) && ((dma_buffer[1] - dma_buffer[0]) < 2200)) {
		signaltimeout = 0;
		if (calibration_required) {
			if (!high_calibration_set) {
				if (high_calibration_counts == 0) {
					last_high_threshold = dma_buffer[1] - dma_buffer[0];
				}
				high_calibration_counts++;
				if (getAbsDif(last_high_threshold, servo_high_threshold) > 50) {
					calibration_required = 0;
				} else {
					servo_high_threshold = ((7 * servo_high_threshold + (dma_buffer[1] - dma_buffer[0])) >> 3);
					if (high_calibration_counts > 50) {
						servo_high_threshold = servo_high_threshold - 25;
						eepromBuffer.servo_high_threshold = (servo_high_threshold - 1750) / 2;
						high_calibration_set = 1;
						playDefaultTone();
					}
				}
				last_high_threshold = servo_high_threshold;
			}
			if (high_calibration_set) {
				if (dma_buffer[1] - dma_buffer[0] < 1250) {
					low_calibration_counts++;
					servo_low_threshold = ((7 * servo_low_threshold + (dma_buffer[1] - dma_buffer[0])) >> 3);
				}
				if (low_calibration_counts > 75) {
					servo_low_threshold = servo_low_threshold + 25;
					eepromBuffer.servo_low_threshold = (servo_low_threshold - 750) / 2;
					calibration_required = 0;
					saveEEpromSettings();
					low_calibration_counts = 0;
					playChangedTone();
				}
			}
			signaltimeout = 0;
		} else {
			if (eepromBuffer.bidirectional_mode) {
				if (dma_buffer[1] - dma_buffer[0] <= servo_neutral) {
					servorawinput = map((dma_buffer[1] - dma_buffer[0]), servo_low_threshold, servo_neutral, 0, 1000);
				} else {
					servorawinput =
						map((dma_buffer[1] - dma_buffer[0]), servo_neutral + 1, servo_high_threshold, 1001, 2000);
				}
			} else {
				servorawinput = map((dma_buffer[1] - dma_buffer[0]), servo_low_threshold, servo_high_threshold,
						    DSHOT_CMD_MAX, DSHOT_MAX_THROTTLE);
				if (servorawinput <= DSHOT_MIN_THROTTLE) {
					servorawinput = 0;
				}
			}
			/* The received pulse, not the slewed newinput: slewing through
			 * neutral is not a pilot zero. */
#if DRONECAN_SUPPORT
			{
				const uint8_t stop =
					(uint8_t)(eepromBuffer.bidirectional_mode ? servorawinput + (servo_dead_band << 1) >= 1000 &&
											    servorawinput <= 1000 + (servo_dead_band << 1)
										  : servorawinput == 0);
				/* While RawCommand owns throttle the wire is not the pilot.
				 * After it lets go, a wire stop counts only once the wire
				 * itself has demanded throttle. */
				if (!DroneCAN_active()) {
					if (!stop) {
						wire_drove_since_can = 1;
					}
					fault_pilot_zero = (uint8_t)(stop && wire_drove_since_can);
				}
			}
#else
			fault_pilot_zero =
				(uint8_t)(eepromBuffer.bidirectional_mode ? servorawinput + (servo_dead_band << 1) >= 1000 &&
										    servorawinput <= 1000 + (servo_dead_band << 1)
									  : servorawinput == 0);
#endif
			signaltimeout = 0;
		}
	} else {
		zero_input_count = 0; // reset if out of range
	}

#if DRONECAN_SUPPORT
	/* AUTO dual-path: live RawCommand owns throttle; still refreshed signaltimeout above. */
	if (DroneCAN_active()) {
		return;
	}
#endif
	if (servorawinput - newinput > max_servo_deviation) {
		newinput += max_servo_deviation;
	} else if (newinput - servorawinput > max_servo_deviation) {
		newinput -= max_servo_deviation;
	} else {
		newinput = servorawinput;
	}
}

#if DRONECAN_SUPPORT
uint8_t wire_drove_since_can = 1;

/*
 * set_input() used to call transfercomplete() on every RawCommand and on the
 * 1 kHz re-injection. That re-decoded the last wire capture: after the 250 ms
 * RawCommand timeout it replayed a stale PWM stop as a pilot zero, which
 * could clear a fault latched under CAN (PR #147 review). It also re-armed
 * the capture mid-frame and could send a bidirectional DShot reply out of
 * turn. RawCommand needs only the input-side arming bookkeeping below; it is
 * transfercomplete()'s !armed block without the DShot frame-time average,
 * which belongs to the wire.
 */
void canInputTick(void)
{
	if (armed) {
		return;
	}
	if (adjusted_input == 0 && calibration_required == 0) {
		zero_input_count++;
	} else if (!eepromBuffer.disable_stick_calibration) {
		zero_input_count = 0;
		if (adjusted_input > 1500) {
			if (getAbsDif(adjusted_input, last_input) > 50) {
				enter_calibration_count = 0;
			} else {
				enter_calibration_count++;
			}
			if (enter_calibration_count > 50 && (!high_calibration_set)) {
				playBeaconTune3();
				calibration_required = 1;
				enter_calibration_count = 0;
			}
			last_input = adjusted_input;
		}
	}
}

/*
 * AUTO with RawCommand first: set_input() sets inputSet before any wire
 * protocol is known, so transfercomplete() never runs detection and never
 * re-arms the capture; the wire could not take over after a RawCommand
 * timeout (PR #147 review). Called on a real capture completion only (the
 * capture interrupt), never from set_input(): the 1 kHz re-injection would
 * restart the capture mid-frame. Returns 1 if it handled the capture.
 */
uint8_t autoWireDetectOnCapture(void)
{
	if (inputSet != 1 || dshot || servoPwm || eepromBuffer.input_type == DRONECAN_IN) {
		return 0;
	}
	detectInput();
	receiveDshotDma();
	return 1;
}
#endif

void transfercomplete()
{
#ifndef MCU_F031 // f031 does not use software EXTI event to process dshot
	if (armed && dshot_telemetry) {
		if (out_put) {
			receiveDshotDma();
			compute_dshot_flag = 2;
			return;
		} else {
			sendDshotDma();
			compute_dshot_flag = 1;
			return;
		}
	}
#endif
	if (inputSet == 0) {
		detectInput();
		receiveDshotDma();
		return;
	}
	if (inputSet == 1) {
		if (dshot_telemetry) {
			if (out_put) {
				make_dshot_package(e_com_time);
				computeDshotDMA();
				receiveDshotDma();
				return;
			} else {
				sendDshotDma();
				return;
			}
		} else {
			if (dshot == 1) {
				computeDshotDMA();
				receiveDshotDma();
			}
			if (servoPwm == 1) {
				if (getInputPinState()) {
					buffersize = 3;
				} else {
					buffersize = 2;
					computeServoInput();
				}
				receiveDshotDma();
			}
		}
		if (!armed) {
			if (dshot && (average_count < 8) && (zero_input_count > 5)) {
				average_count++;
				average_packet_length = average_packet_length + (uint16_t)(dma_buffer[31] - dma_buffer[0]);
				if (average_count == 8) {
					dshot_frametime_high = (average_packet_length >> 3) + (average_packet_length >> 7);
					dshot_frametime_low = (average_packet_length >> 3) - (average_packet_length >> 7);
				}
			}
			if (adjusted_input == 0 && calibration_required == 0) { // note this in input..not newinput so it
				// will be adjusted be main loop
				zero_input_count++;
			} else {
				if (!eepromBuffer.disable_stick_calibration) {
					zero_input_count = 0;
					if (adjusted_input > 1500) {
						if (getAbsDif(adjusted_input, last_input) > 50) {
							enter_calibration_count = 0;
						} else {
							enter_calibration_count++;
						}

						if (enter_calibration_count > 50 && (!high_calibration_set)) {
							playBeaconTune3();
							calibration_required = 1;
							enter_calibration_count = 0;
						}
						last_input = adjusted_input;
					}
				}
			}
		}
	}
}

void checkDshot()
{
	if ((smallestnumber >= 1) && (smallestnumber < 4) && (average_signal_pulse < 60)) {
		ic_timer_prescaler = 0;
		if (CPU_FREQUENCY_MHZ > 100) {
			output_timer_prescaler = 1;
		} else {
			output_timer_prescaler = 0;
		}
		//	dshot_runout_timer = 1000;
		dshot = 1;
		buffer_padding = 14;
		buffersize = 32;
		inputSet = 1;
	}
	if ((smallestnumber >= 4) && (smallestnumber <= 8) && (average_signal_pulse < 100)) {
		dshot = 1;
		ic_timer_prescaler = 1;
		if (CPU_FREQUENCY_MHZ > 100) {
			output_timer_prescaler = 3;
		} else {
			output_timer_prescaler = 1;
		}
		buffer_padding = 7;
		buffersize = 32;
		inputSet = 1;
	}
}
void checkServo()
{
	if (smallestnumber > 200 && smallestnumber < 20000) {
		servoPwm = 1;
		ic_timer_prescaler = CPU_FREQUENCY_MHZ - 1;
		buffersize = 2;
		inputSet = 1;
	}
}

void detectInput()
{
	smallestnumber = 20000;
	average_signal_pulse = 0;
	int lastnumber = dma_buffer[0];
	for (int j = 1; j < 31; j++) {
		if (dma_buffer[j] - lastnumber > 0) {
			if ((dma_buffer[j] - lastnumber) < smallestnumber) {
				smallestnumber = dma_buffer[j] - lastnumber;
			}
			average_signal_pulse += (dma_buffer[j] - lastnumber);
		}
		lastnumber = dma_buffer[j];
	}
	average_signal_pulse = average_signal_pulse / 32;

	if (dshot == 1) {
		checkDshot();
	}
	if (servoPwm == 1) {
		checkServo();
	}

	if (!dshot && !servoPwm) {
		checkDshot();
		checkServo();
	}
}
