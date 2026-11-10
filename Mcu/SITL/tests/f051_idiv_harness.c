/*
 * Cortex-M0 harness for test_f051_idiv.py: compares Mcu/f051/Src/idiv.s
 * (linked here as ark_idiv) with libgcc's __divsi3 (reached through `/`).
 *
 * Built with the pinned arm-none-eabi-gcc and run under qemu-arm with
 * semihosting. Prints "OK <count>" or "FAIL n d got want" lines.
 */
#include <stdint.h>
#include <stdio.h>

extern int32_t ark_idiv(int32_t n, int32_t d);

/* Opaque to the optimiser, so `/` stays a real libgcc call. */
static int32_t __attribute__((noinline)) ref_idiv(int32_t n, int32_t d)
{
	return n / d;
}

static const int32_t edges[] = {
	0,	  1,	     -1,	 2,	     -2,	 3,	     -3,	  7,	     -7,	 10,
	-10,	  41,	     100,	 -100,	     255,	 256,	     360,	  2000,	     -2000,	 4095,
	10000,	  -10000,    32767,	 -32767,     -32768,	 65535,	     65536,	  -65536,    600000,	 -600000,
	60000000, -60000000, 0x3fffffff, 0x40000000, 0x7ffffffe, 0x7fffffff, -0x7fffffff, INT32_MIN, 0x55555555, -0x55555555,
};
#define N_EDGES (sizeof(edges) / sizeof(edges[0]))

static uint32_t rng = 0x9e3779b9u;
static uint32_t next(void)
{
	rng ^= rng << 13;
	rng ^= rng >> 17;
	rng ^= rng << 5;
	return rng;
}

static unsigned long count;
static int fails;

static void check(int32_t n, int32_t d)
{
	const int32_t want = ref_idiv(n, d);
	const int32_t got = ark_idiv(n, d);
	count++;
	if (got != want && fails++ < 20) {
		printf("FAIL %ld %ld got %ld want %ld\n", (long)n, (long)d, (long)got, (long)want);
	}
}

int main(void)
{
	/* every edge pair, including d == 0 and INT32_MIN / -1 */
	for (unsigned i = 0; i < N_EDGES; i++) {
		for (unsigned j = 0; j < N_EDGES; j++) {
			check(edges[i], edges[j]);
		}
	}
	/* random pairs at every combination of operand magnitude */
	for (unsigned ns = 0; ns < 32; ns++) {
		for (unsigned ds = 0; ds < 32; ds++) {
			for (unsigned k = 0; k < 64; k++) {
				check((int32_t)next() >> ns, (int32_t)next() >> ds);
			}
		}
	}
	if (!fails) {
		printf("OK %lu\n", count);
	}
	return fails ? 1 : 0;
}
