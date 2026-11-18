/*
 * IO.h
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */

#include "main.h"

extern volatile char out_put;
extern volatile char inputSet;
extern char dshot;
extern volatile char servoPwm;
extern volatile char send_telemetry;
extern volatile int16_t degrees_celsius;

extern uint16_t ADC_raw_volts;
extern uint16_t servo_low_threshold;  // anything below this point considered 0
extern uint16_t servo_high_threshold; // anything above this point considered 2000 (max)
extern uint16_t servo_neutral;
extern uint8_t servo_dead_band;

void detectInput();

/*
 * AUTO dual path (DRONECAN_SUPPORT builds; Src/signal.c):
 * - wire_drove_since_can: the wire has demanded throttle since RawCommand last
 *   owned it. A wire stop counts as a pilot zero only then, so neither a
 *   standing wire stop nor a stale capture clears a fault latched under CAN.
 * - canInputTick(): set_input()'s per-RawCommand input bookkeeping, without
 *   touching the wire capture.
 * - autoWireDetectOnCapture(): a wire capture completed while RawCommand had
 *   marked the input set but no wire protocol is known yet.
 */
extern uint8_t wire_drove_since_can;
void canInputTick(void);
uint8_t autoWireDetectOnCapture(void);
