/*
 * dshot.c
 *
 *  Created on: Apr. 22, 2020
 *      Author: Alka
 */

#include "dshot.h"
#include "IO.h"
#include "common.h"
#include "functions.h"
#include "sounds.h"
#include "targets.h"
#include "hwci_perf.h"
#if DRONECAN_SUPPORT
#	include "DroneCAN/DroneCAN.h"
#endif

const char gcr_encode_table[16] = {0b11001, 0b11011, 0b10010, 0b10011, 0b11101, 0b10101, 0b10110, 0b10111,
				   0b11010, 0b01001, 0b01010, 0b01011, 0b11110, 0b01101, 0b01110, 0b01111};

typedef struct {
	uint16_t temp_count;
	uint16_t voltage_count;
	uint16_t current_count;
	uint8_t last_sent_extended;
} dshot_telem_scheduler_t;

static dshot_telem_scheduler_t telem_scheduler = {0};

// These divisors create ratios regardless of input rate:
// - Temperature: every 200 calls (4Hz at 800Hz input)
// - Voltage: every 200 calls (4Hz at 800Hz input)
// - Current: every 40 calls (20Hz at 800Hz input)
// - eRPM: fills all other slots

#define TEMP_EDT_RATE_DIVISOR 200
#define VOLTAGE_EDT_RATE_DIVISOR 200
#define CURRENT_EDT_RATE_DIVISOR 40

char send_EDT_init;
char send_EDT_deinit;
char EDT_ARM_ENABLE = 0;
char EDT_ARMED = 0;
int shift_amount = 0;
volatile uint32_t gcrnumber;
extern volatile char send_telemetry;
extern uint8_t max_duty_cycle_change;
int dshot_full_number;
extern char play_tone_flag;
extern char send_esc_info_flag;
uint8_t command_count = 0;
uint8_t last_command = 0;
uint8_t high_pin_count = 0;
uint32_t gcr[37] = {0};
uint16_t dshot_frametime;
uint16_t dshot_goodcounts;
uint16_t dshot_badcounts;
/* Consecutive drive-demand frames discarded before throttle may leave zero.
 * 2 discards = the 3rd corroborating frame is the one accepted, costing two
 * frame periods (2.5 ms at 800 Hz) on a genuine spool-up and nothing at all
 * once moving. See the idle-exit holdoff in computeDshotDMA(). */
#define DSHOT_IDLE_EXIT_HOLDOFF_FRAMES 2
static uint8_t dshot_idle_exit_holdoff;
/* Diagnostic, alongside good/badcounts: frames rejected by the holdoff. A
 * healthy link ticks this by exactly DSHOT_IDLE_EXIT_HOLDOFF_FRAMES per real
 * spool-up; anything beyond that is single-frame drive demands that never
 * corroborated, i.e. the fault the holdoff exists to absorb. */
uint16_t dshot_idle_exit_blocked;
uint8_t dshot_extended_telemetry = 0;
uint16_t processtime = 0;
uint16_t halfpulsetime = 0;

uint8_t programming_mode;
uint16_t position;
uint8_t new_byte;

void computeDshotDMA()
{
	dshot_frametime = dma_buffer[31] - dma_buffer[0];
	halfpulsetime = dshot_frametime >> 5;
	if ((dshot_frametime > dshot_frametime_low) && (dshot_frametime < dshot_frametime_high)) {
		signaltimeout = 0;
		// Shift the 16 decoded pulses straight into one register-resident word,
		// msb first: bit (15 - i) is pulse i. The frame layout is then
		// [11 bit value][telem req][4 bit crc], so every field below is a
		// shift-and-mask instead of an indexed load out of a 64 byte array.
		uint16_t frame = 0;
		for (int i = 0; i < 16; i++) {
			// note that dma_buffer[] is uint32_t, we cast the difference to uint16_t to handle
			// timer wrap correctly
			const uint16_t pdiff = dma_buffer[(i << 1) + 1] - dma_buffer[(i << 1)];
			frame = (frame << 1) | (pdiff > halfpulsetime);
		}
		// crc nibble is the xor of the three value nibbles
		uint8_t calcCRC = ((frame >> 4) ^ (frame >> 8) ^ (frame >> 12)) & 0xF;
		uint8_t checkCRC = frame & 0xF;

		if (!armed) {
			if (dshot_telemetry == 0) {
				if (getInputPinState()) { // if the pin is high for 100 checks between
					// signal pulses its inverted
					high_pin_count++;
					if (high_pin_count > 100) {
						dshot_telemetry = 1;
#ifdef HWCI_PERF
						hwci_perf.dshot_telem_mode = 1;
#endif
					}
				}
			}
		}
		if (dshot_telemetry) {
			checkCRC = (~checkCRC) & 0xF;
		}

		int tocheck = frame >> DSHOT_VALUE_SHIFT; // upper 11 bits: throttle value or command

		if (calcCRC == checkCRC) {
			signaltimeout = 0;
			dshot_goodcounts++;
#ifdef HWCI_PERF
			hwci_perf.dshot_rx_good++;
			hwci_perf.dshot_telem_mode = (uint8_t)dshot_telemetry;
			hwci_perf.dshot_edt_mode = dshot_extended_telemetry;
#endif
			if (frame & DSHOT_TELEMETRY_BIT) {
				send_telemetry = 1;
			}
			if (programming_mode > DSHOT_PROG_IDLE) {
				if (programming_mode == DSHOT_PROG_WAIT_ADDRESS) {
					position = tocheck; /* eepromBuffer index */
					programming_mode = DSHOT_PROG_WAIT_VALUE;
					return;
				}
				if (programming_mode == DSHOT_PROG_WAIT_VALUE) {
					new_byte = tocheck;
					programming_mode = DSHOT_PROG_WAIT_COMMIT;
					return;
				}
				if (programming_mode == DSHOT_PROG_WAIT_COMMIT) {
					/* RAM only; DSHOT_CMD_SAVE_SETTINGS makes it permanent. */
					if (tocheck == DSHOT_CMD_EXIT_PROGRAMMING_MODE) {
						eepromBuffer.buffer[position] = new_byte;
						programming_mode = DSHOT_PROG_IDLE;
					}
				}
				return; /* ignore throttle / other commands while programming */
			}
			if (tocheck > DSHOT_CMD_MAX) {
				if (EDT_ARMED) {
#if DRONECAN_SUPPORT
					/* AUTO dual-path: live RawCommand owns throttle. */
					if (DroneCAN_active()) {
						return;
					}
#endif
					/* Idle-exit holdoff. Neither integrity check in this
					 * function can reject a single corrupted frame that
					 * decodes to full throttle: an all-ones capture
					 * (0xFFFF) is a CRC-VALID DShot frame carrying
					 * throttle 2047, and its frame span lands only
					 * +1.63% off nominal - inside the +-6.25% span gate
					 * above. One lost edge on the signal line inverts
					 * the pulse/gap pairing, so a commanded stop
					 * (0x0000, every pulse short) becomes commanded
					 * full throttle. Worse, newinput then LATCHES: it
					 * is only overwritten by the next accepted frame,
					 * and the same marginal link that corrupted this
					 * one tends to drop the next several, so the bogus
					 * demand survives for as long as the dropout.
					 * Measured on an ARK G431 CAN bench as a 30-90 ms,
					 * ~29% duty spin-up kick at armed idle, roughly
					 * once a minute (hwci/runs/gate_blip_watch.log).
					 *
					 * So require the demand to repeat before acting on
					 * it, but only while throttle is still at zero -
					 * once moving this costs nothing, and in-flight
					 * slew stays governed by max_duty_cycle_change /
					 * max_ramp in the duty domain. */
					if (newinput == 0 && dshot_idle_exit_holdoff < DSHOT_IDLE_EXIT_HOLDOFF_FRAMES) {
						dshot_idle_exit_holdoff++;
						dshot_idle_exit_blocked++;
						dshotcommand = 0;
						command_count = 0;
						return;
					}
					newinput = tocheck;
					dshotcommand = 0;
					command_count = 0;
					return;
				}
			}

			if ((tocheck <= DSHOT_CMD_MAX) && (tocheck > DSHOT_CMD_MOTOR_STOP)) {
				/* Any non-drive frame breaks the run of drive demands the
				 * holdoff above is counting - reset ahead of the DroneCAN
				 * early-out so the run always reflects the wire. */
				dshot_idle_exit_holdoff = 0;
#if DRONECAN_SUPPORT
				if (DroneCAN_active()) {
					return;
				}
#endif
				newinput = 0;
				dshotcommand = tocheck;
			}
			if (tocheck == DSHOT_CMD_MOTOR_STOP) {
				dshot_idle_exit_holdoff = 0;
				if (EDT_ARM_ENABLE == 1) {
					EDT_ARMED = 0;
				}
#if DRONECAN_SUPPORT
				if (DroneCAN_active()) {
					/* Keep signaltimeout fed; do not zero over CAN demand. */
					return;
				}
#endif
				newinput = 0;
				dshotcommand = 0;
				command_count = 0;
			}

			if ((dshotcommand > DSHOT_CMD_MOTOR_STOP) && (running == 0) && armed) {
				if (dshotcommand != last_command) {
					last_command = dshotcommand;
					command_count = 0;
				}
				if (dshotcommand <= DSHOT_CMD_BEACON5) {
					command_count = DSHOT_CMD_REPEATS; /* beacons fire immediately */
				}
				command_count++;
				if (command_count >= DSHOT_CMD_REPEATS) {
					command_count = 0;
					switch (dshotcommand) {
						case DSHOT_CMD_BEACON1:
							play_tone_flag = 1;
							break;
						case DSHOT_CMD_BEACON2:
							play_tone_flag = 2;
							break;
						case DSHOT_CMD_BEACON3:
							play_tone_flag = 3;
							break;
						case DSHOT_CMD_BEACON4:
							play_tone_flag = 4;
							break;
						case DSHOT_CMD_BEACON5:
							play_tone_flag = 5;
							break;
						case DSHOT_CMD_ESC_INFO:
							send_esc_info_flag = 1;
							break;
						case DSHOT_CMD_SPIN_DIRECTION_1:
							eepromBuffer.dir_reversed = 0;
							forward = 1 - eepromBuffer.dir_reversed;
							break;
						case DSHOT_CMD_SPIN_DIRECTION_2:
							eepromBuffer.dir_reversed = 1;
							forward = 1 - eepromBuffer.dir_reversed;
							break;
						case DSHOT_CMD_3D_MODE_OFF:
							eepromBuffer.bi_direction = 0;
							break;
						case DSHOT_CMD_3D_MODE_ON:
							eepromBuffer.bi_direction = 1;
							break;
						case DSHOT_CMD_SAVE_SETTINGS:
							saveEEpromSettings();
							play_tone_flag = 1 + eepromBuffer.dir_reversed;
							break;
						case DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE:
							dshot_extended_telemetry = 1;
							send_EDT_init = 1;
							if (EDT_ARM_ENABLE == 1) {
								EDT_ARMED = 1;
							}
#ifdef HWCI_PERF
							hwci_perf.dshot_edt_mode = 1;
#endif
							break;
						case DSHOT_CMD_EXTENDED_TELEMETRY_DISABLE:
							dshot_extended_telemetry = 0;
							send_EDT_deinit = 1;
#ifdef HWCI_PERF
							hwci_perf.dshot_edt_mode = 0;
#endif
							break;
						case DSHOT_CMD_SPIN_DIRECTION_NORMAL:
							forward = 1 - eepromBuffer.dir_reversed;
							break;
						case DSHOT_CMD_SPIN_DIRECTION_REVERSED:
							forward = eepromBuffer.dir_reversed;
							break;
						case DSHOT_CMD_ENTER_PROGRAMMING_MODE:
							programming_mode = DSHOT_PROG_WAIT_ADDRESS;
							break;
					}
					last_dshot_command = dshotcommand;
					dshotcommand = 0;
				}
			}
		} else {
			dshot_badcounts++;
#ifdef HWCI_PERF
			hwci_perf.dshot_rx_bad++;
#endif
			programming_mode = DSHOT_PROG_IDLE;
		}
	}
}

void make_dshot_package(uint16_t com_time)
{
	uint16_t extended_frame_to_send = 0;
#ifdef HWCI_PERF
	/* Snapshot the period the reply will advertise (stopped → 65535 later). */
	const uint16_t _hwci_com_snap = com_time;
#endif

	if (dshot_extended_telemetry) {
		// Only send extended telemetry if last frame wasn't extended. This ensures eRPM interleaving.
		if (telem_scheduler.last_sent_extended) {
			telem_scheduler.last_sent_extended = 0;

		} else {
			telem_scheduler.current_count++;
			telem_scheduler.voltage_count++;
			telem_scheduler.temp_count++;

			if (telem_scheduler.current_count >= CURRENT_EDT_RATE_DIVISOR) {
				extended_frame_to_send = DSHOT_EDT_PACK(DSHOT_EDT_CURRENT, actual_current / 100);
				telem_scheduler.current_count = 0;
			} else if (telem_scheduler.voltage_count >= VOLTAGE_EDT_RATE_DIVISOR) {
				extended_frame_to_send = DSHOT_EDT_PACK(DSHOT_EDT_VOLTAGE, battery_voltage / 25);
				telem_scheduler.voltage_count = 0;
			} else if (telem_scheduler.temp_count >= TEMP_EDT_RATE_DIVISOR) {
				extended_frame_to_send = DSHOT_EDT_PACK(DSHOT_EDT_TEMPERATURE, degrees_celsius);
				telem_scheduler.temp_count = 0;
			}
		}
	}
	if (send_EDT_init) {
		extended_frame_to_send = DSHOT_EDT_PACK(DSHOT_EDT_STATE_EVENT, 0x00);
		send_EDT_init = 0;
	}
	if (send_EDT_deinit) {
		extended_frame_to_send = DSHOT_EDT_PACK(DSHOT_EDT_STATE_EVENT, 0xFF);
		send_EDT_deinit = 0;
	}

	if (extended_frame_to_send > 0) {
		dshot_full_number = extended_frame_to_send;
		telem_scheduler.last_sent_extended = 1;

	} else {
		if (!running) {
			com_time = 65535;
		}
		//	calculate shift amount for data in format eee mmm mmm mmm, first 1 found
		// in first seven bits of data determines shift amount
		// this allows for a range of up to 65408 microseconds which would be
		// shifted 0b111 (eee) or 7 times.
		for (int i = 15; i >= 9; i--) {
			if (com_time >> i == 1) {
				shift_amount = i + 1 - 9;
				break;
			} else {
				shift_amount = 0;
			}
		}
		// shift the commutation time to allow for expanded range and put shift
		// amount in first three bits
		dshot_full_number = ((shift_amount << 9) | (com_time >> shift_amount));
	}
	// calculate checksum
	uint16_t csum = 0;
	uint16_t csum_data = dshot_full_number;
	for (int i = 0; i < 3; i++) {
		csum ^= csum_data; // xor data by nibbles
		csum_data >>= 4;
	}
	csum = ~csum; // invert it
	csum &= 0xf;

	dshot_full_number = (dshot_full_number << 4) | csum; // put checksum at the end of 12 bit dshot number

	// GCR RLL encode 16 to 20 bit

	gcrnumber = gcr_encode_table[(dshot_full_number >> 12)] << 15	       // first set of four digits
		    | gcr_encode_table[(0xf & (dshot_full_number >> 8))] << 10 // 2nd set of 4 digits
		    | gcr_encode_table[(0xf & (dshot_full_number >> 4))] << 5  // 3rd set of four digits
		    | gcr_encode_table[(0xf & (dshot_full_number >> 0))];      // last four digits
// GCR RLL encode 20 to 21bit output
#if defined(MCU_F051) || defined(MCU_F031) || defined(MCU_CH32V203)
	gcr[1 + buffer_padding] = 64;
	for (int i = 19; i >= 0; i--) { // each digit in gcrnumber
		gcr[buffer_padding + 20 - i + 1] = ((((gcrnumber & 1 << i)) >> i) ^ (gcr[buffer_padding + 20 - i] >> 6))
						   << 6; // exclusive ored with number before it multiplied by 64 to match
							 // output timer.
	}
	gcr[buffer_padding] = 0;
#elif defined(NXP)
	//Do gray encoding
	//Apparently GCR RLL is not done here but Grey encoding.
	uint32_t binary = gcrnumber;
	while (gcrnumber >>= 1) {
		binary ^= gcrnumber;
	}
	gcrnumber = binary;
#else
	gcr[1 + buffer_padding] = 128;
	for (int i = 19; i >= 0; i--) { // each digit in gcrnumber
		// exclusive ored with number before it multiplied by 64 to match output timer.
		gcr[buffer_padding + 20 - i + 1] = ((((gcrnumber & 1 << i)) >> i) ^ (gcr[buffer_padding + 20 - i] >> 7)) << 7;
	}
	gcr[buffer_padding] = 0;
#endif
#ifdef HWCI_PERF
	hwci_perf.dshot_tx_frames++;
	/* Prefer the period that was actually encoded (65535 when !running). */
	hwci_perf.dshot_last_com_us = (!running && extended_frame_to_send == 0) ? 65535u : _hwci_com_snap;
	hwci_perf.dshot_telem_mode = (uint8_t)dshot_telemetry;
	hwci_perf.dshot_edt_mode = dshot_extended_telemetry;
#endif
}
