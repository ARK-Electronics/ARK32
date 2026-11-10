/*
 * Host harness for test_sine_table.py: Inc/sine_table.h must return, for every
 * angle, what the table it replaced returned.
 *
 * old_half[] and old_lookup() are that table and its lookup, verbatim from
 * Src/motor_runtime.c and Inc/motor_runtime.h before the quarter-wave change.
 * Prints "OK 360" or one "FAIL angle got want" line per mismatch.
 */
#include <stdint.h>
#include <stdio.h>

#include "sine_table.h"

static const int16_t old_half[180] = {
	180, 183, 186, 189, 193, 196, 199, 202, 205, 208, 211, 214, 217, 220, 224, 227, 230, 233, 236, 239, 242, 245, 247, 250, 253, 256,
	259, 262, 265, 267, 270, 273, 275, 278, 281, 283, 286, 288, 291, 293, 296, 298, 300, 303, 305, 307, 309, 312, 314, 316, 318, 320,
	322, 324, 326, 327, 329, 331, 333, 334, 336, 337, 339, 340, 342, 343, 344, 346, 347, 348, 349, 350, 351, 352, 353, 354, 355, 355,
	356, 357, 357, 358, 358, 359, 359, 359, 360, 360, 360, 360, 360, 360, 360, 360, 360, 359, 359, 359, 358, 358, 357, 357, 356, 355,
	355, 354, 353, 352, 351, 350, 349, 348, 347, 346, 344, 343, 342, 340, 339, 337, 336, 334, 333, 331, 329, 327, 326, 324, 322, 320,
	318, 316, 314, 312, 309, 307, 305, 303, 300, 298, 296, 293, 291, 288, 286, 283, 281, 278, 275, 273, 270, 267, 265, 262, 259, 256,
	253, 250, 247, 245, 242, 239, 236, 233, 230, 227, 224, 220, 217, 214, 211, 208, 205, 202, 199, 196, 193, 189, 186, 183};

static int16_t old_lookup(int16_t a)
{
	return (a < 180) ? old_half[a] : (int16_t)(360 - old_half[a - 180]);
}

int main(void)
{
	int fails = 0;
	for (int16_t a = 0; a < 360; a++) {
		const int16_t want = old_lookup(a);
		const int16_t got = pwmSinLookup(a);
		if (got != want) {
			printf("FAIL %d got %d want %d\n", a, got, want);
			fails++;
		}
	}
	if (!fails) {
		printf("OK 360\n");
	}
	return fails ? 1 : 0;
}
