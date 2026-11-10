/*
 * sine_table.h - sine-startup PWM table
 *
 * Self-contained (stdint only) so Mcu/SITL/tests/test_sine_table.py can build
 * it on the host and compare it with the table it replaced.
 */
#ifndef SINE_TABLE_H_
#define SINE_TABLE_H_

#include <stdint.h>

/*
 * The original AM32 table is 360 int16_t entries, 0..360 centred on 180. It has
 * two exact symmetries (both checked for all 360 angles by the test):
 *
 *   pwmSin[a + 180] == 360 - pwmSin[a]      half-wave
 *   pwmSin[180 - a] == pwmSin[a]            quarter-wave
 *
 * so the first quarter is enough, and with the 180 offset removed every entry
 * fits a byte. 91 B of flash instead of 720 B (360 B with half-wave only).
 * pwmSinQuarter[i] == pwmSin[i] - 180 for i in 0..90.
 */
static const uint8_t pwmSinQuarter[91] = {
	0,   3,	  6,   9,   13,	 16,  19,  22,	25,  28,  31,  34,  37,	 40,  44,  47,	50,  53,  56,  59,  62,	 65,  67,
	70,  73,  76,  79,  82,	 85,  87,  90,	93,  95,  98,  101, 103, 106, 108, 111, 113, 116, 118, 120, 123, 125, 127,
	129, 132, 134, 136, 138, 140, 142, 144, 146, 147, 149, 151, 153, 154, 156, 157, 159, 160, 162, 163, 164, 166, 167,
	168, 169, 170, 171, 172, 173, 174, 175, 175, 176, 177, 177, 178, 178, 179, 179, 179, 180, 180, 180, 180, 180,
};

/* Table value for an angle in degrees; a must be in [0, 360). */
static inline int16_t pwmSinLookup(int16_t a)
{
	const int16_t half = (a < 180) ? a : (int16_t)(a - 180);
	const int16_t v = pwmSinQuarter[(half <= 90) ? half : 180 - half];
	return (int16_t)((a < 180) ? 180 + v : 180 - v);
}

#endif /* SINE_TABLE_H_ */
