/*
 * idiv.s - signed 32-bit divide for the F051, built on libgcc's unsigned one
 *
 * The Cortex-M0 has no divide instruction, so every `/` on a non-constant
 * becomes a call to a libgcc helper. libgcc ships the signed and the unsigned
 * helper as two separate, fully unrolled routines: __udivsi3 (266 B) and
 * __divsi3 (460 B). The firmware needs the unsigned one regardless, and C
 * integer promotion makes most of its divides signed (any uint8_t/uint16_t
 * operand promotes to int), so both were linked.
 *
 * Defining __aeabi_idiv here keeps the linker from pulling libgcc's _divsi3.o
 * and costs 44 B instead of 468 B. No call site changes.
 *
 * Results are identical to libgcc's __divsi3 for every input pair:
 *   - both operands non-negative: the unsigned quotient is the signed one
 *   - otherwise: |n| / |d| with the sign of n ^ d, i.e. C truncation toward
 *     zero. INT_MIN is handled because the magnitudes are divided unsigned.
 *   - d == 0: libgcc's __divsi3 and __udivsi3 both return 0 (after calling the
 *     no-op __aeabi_idiv0); so does this, since -0 == 0.
 * Mcu/SITL/tests/test_f051_idiv.py runs both under qemu and compares them.
 *
 * Cost: 5 cycles more than libgcc when both operands are non-negative (the
 * literal load and bx), about 20 more when either is negative.
 *
 * __aeabi_idivmod (signed %) is deliberately not provided. Nothing uses it; a
 * new signed `%` would pull libgcc's _divsi3.o back in and fail the link with
 * a duplicate __aeabi_idiv, which is the prompt to make that `%` unsigned.
 */
	.syntax unified
	.cpu cortex-m0
	.thumb

	.section .text.__aeabi_idiv, "ax", %progbits
	.global __aeabi_idiv
	.global __divsi3
	.type __aeabi_idiv, %function
	.type __divsi3, %function
	.thumb_func
__aeabi_idiv:
	.thumb_func
__divsi3:
	movs	r3, r0
	orrs	r3, r1
	bmi	1f
	/* both non-negative: tail-call the unsigned divide */
	ldr	r3, =__aeabi_uidiv
	bx	r3
1:
	push	{r4, lr}
	movs	r4, r0
	eors	r4, r1		/* bit 31 = sign of the quotient */
	asrs	r2, r0, #31
	eors	r0, r2
	subs	r0, r0, r2	/* r0 = |n| */
	asrs	r2, r1, #31
	eors	r1, r2
	subs	r1, r1, r2	/* r1 = |d| */
	bl	__aeabi_uidiv
	asrs	r4, r4, #31
	eors	r0, r4
	subs	r0, r0, r4	/* negate if the signs differed */
	pop	{r4, pc}
	.ltorg
	.size __aeabi_idiv, . - __aeabi_idiv
	.size __divsi3, . - __divsi3
