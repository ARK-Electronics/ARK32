///*
// * comparator.c
// *
// *  Created on: Sep. 26, 2020
// *      Author: Alka
// */
//
// #include "comparator.h"
// #include "targets.h"
//
//
// void maskPhaseInterrupts(){
//	EXTI->IMR1 &= ~(1 << 18);
//	EXTI->RPR1 = EXTI_LINE;
//	EXTI->FPR1 = EXTI_LINE;
////	LL_EXTI_ClearRisingFlag_0_31(EXTI_LINE);
////	LL_EXTI_ClearFallingFlag_0_31(EXTI_LINE);
//}
//
// void enableCompInterrupts(){
//    EXTI->IMR1 |= (1 << 18);
//}
//

/*
 * comparator.c
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */

#include "comparator.h"

#include "common.h"
#include "motor_runtime.h"
#include "targets.h"

COMP_TypeDef *active_COMP = COMP2;
uint32_t current_EXTI_LINE = LL_EXTI_LINE_22;

uint8_t getCompOutputLevel()
{
	//  return (active_COMP->CSR >> 30 & 1);
	return LL_COMP_ReadOutputLevel(active_COMP);
}

void maskPhaseInterrupts()
{
	EXTI->IMR1 &= ~(1 << 21);
	EXTI->IMR1 &= ~(1 << 22);
	EXTI->PR1 = LL_EXTI_LINE_22;
	EXTI->PR1 = LL_EXTI_LINE_21;
}

void enableCompInterrupts()
{
	EXTI->IMR1 |= current_EXTI_LINE;
}

void changeCompInput()
{
	if (step == 1 || step == 4) { // c floating

		current_EXTI_LINE = PHASE_C_EXTI_LINE;
		active_COMP = PHASE_C_COMP_NUMBER;

		LL_COMP_ConfigInputs(active_COMP, PHASE_C_COMP, PHASE_C_INPUT_PLUS);
	}

	if (step == 2 || step == 5) { // a floating

		current_EXTI_LINE = PHASE_A_EXTI_LINE;
		active_COMP = PHASE_A_COMP_NUMBER;

		LL_COMP_ConfigInputs(active_COMP, PHASE_A_COMP, PHASE_A_INPUT_PLUS);
	}

	if (step == 3 || step == 6) { // b floating

		current_EXTI_LINE = PHASE_B_EXTI_LINE;
		active_COMP = PHASE_B_COMP_NUMBER;

		LL_COMP_ConfigInputs(active_COMP, PHASE_B_COMP, PHASE_B_INPUT_PLUS);
	}
	/*
	 * G4 dual-COMP path: F051 enables hyst whenever CI is slow. On this 12S
	 * free-run high-KV article, pure free-run BEMF can sit near/under 10 mV
	 * at crawl RPM — always-on 10 mV hyst blocked real edges and latched
	 * stuck-rotor at 5–6% (bench 2026-08). Only add hyst when the interval
	 * is slow AND duty is already into a real drive (loaded start / prop),
	 * where F051 is clean and BEMF is larger. Free-run crawl stays NONE.
	 *
	 * Turning hysteresis UP is a standing suggestion for the low-rpm noise
	 * and it is the wrong direction on this part. DS13122 Rev 4 Table 73:
	 * LL_COMP_HYSTERESIS_10MV is HYST = 1, which is 9 mV typical but up to
	 * 16 mV, on top of an input offset of -9..+3 mV. The next step up
	 * (HYST = 2) is 18 mV typical and up to 32 mV. Against a signal of
	 * ~132 mV at 1000 rpm falling linearly with speed, code 1 already
	 * consumes a worst-case 25 mV of the crawl budget - which is what the
	 * 2026-08 bench saw as blocked edges and a latched stuck rotor. The
	 * hysteresis knob is out of room here; noise has to be rejected in
	 * time (ZC_SEARCH_BLANK_64THS, COMP_BLANK_TICKS) or the signal has to
	 * be read with something that has a lower floor than the comparator.
	 */
	if (average_interval >= 400 && duty_cycle > 200) {
		LL_COMP_SetInputHysteresis(COMP1, LL_COMP_HYSTERESIS_10MV);
		LL_COMP_SetInputHysteresis(COMP2, LL_COMP_HYSTERESIS_10MV);
	} else {
		LL_COMP_SetInputHysteresis(COMP1, LL_COMP_HYSTERESIS_NONE);
		LL_COMP_SetInputHysteresis(COMP2, LL_COMP_HYSTERESIS_NONE);
	}
#ifdef COMP_BLANK_TICKS
	/*
	 * COMP output blanking is polarity-asymmetric, so it has to be armed per
	 * commutation step instead of left latched on in COMPx_CSR.
	 *
	 * Blanking gates the comparator OUTPUT low while OC5REF is high. It does
	 * not gate COMPx_CSR.VALUE — ST specifies that value is "taken before
	 * polarity and blanking are applied" (stm32g4xx_ll_comp.h,
	 * LL_COMP_ReadOutputLevel), which is what getCompOutputLevel() reads.
	 * Zero crosses come from the EXTI edge on the gated output, so what
	 * matters is what gating does to EDGES, and that depends on which level
	 * the comparator sits at before the crossing:
	 *
	 *   rising == 0 (falling BEMF): phase is above neutral before the
	 *     crossing, so the raw output sits LOW and the awaited edge is rising
	 *     (armed below). Forcing LOW changes nothing before the crossing, and
	 *     a crossing landing inside the window is not lost — the output goes
	 *     high when the window closes, at most COMP_BLANK_TICKS late.
	 *     Blanking is a pure win on these three steps.
	 *
	 *   rising == 1 (rising BEMF): the raw output sits HIGH before the
	 *     crossing and the awaited edge is falling. Forcing LOW then
	 *     MANUFACTURES a falling edge at every window open — exactly the
	 *     armed polarity, once per PWM period, for the entire search window.
	 *     And a real crossing inside the window is destroyed rather than
	 *     delayed: the output is already low, the raw level is low by the
	 *     time the window closes, so no edge is ever produced and the step
	 *     falls through to a blind step.
	 *
	 * Hence: blank the falling-BEMF steps, run the rising ones ungated. The
	 * manufactured edges do get rejected downstream today — interruptRoutine's
	 * confirm loop reads the ungated CSR.VALUE and still sees the pre-crossing
	 * level — so the visible cost was an ISR at the PWM rate plus a
	 * confirm-reject storm rather than accepted false crossings. The destroyed
	 * crossings were silent. Neither belongs on half the steps.
	 *
	 * Blanking all six steps would need COMPx_CSR.POLARITY inverted on the
	 * rising steps so the pre-crossing level is LOW again, with the EXTI edge
	 * flipped to match. That is only correct if POLARITY is applied BEFORE the
	 * blanking gate in the G4 output path; if the gate comes first, inverting
	 * reproduces the same manufactured edges with the sign flipped. RM0440
	 * §24.3 does not state the order and the LL header only pins CSR.VALUE
	 * ahead of both, so that variant needs a scope on a redirected COMP output
	 * to settle — it is deliberately not guessed at here.
	 */
	if (rising) {
		LL_COMP_SetOutputBlankingSource(COMP1, LL_COMP_BLANKINGSRC_NONE);
		LL_COMP_SetOutputBlankingSource(COMP2, LL_COMP_BLANKINGSRC_NONE);
	} else {
		LL_COMP_SetOutputBlankingSource(COMP1, LL_COMP_BLANKINGSRC_TIM1_OC5_COMP1);
		LL_COMP_SetOutputBlankingSource(COMP2, LL_COMP_BLANKINGSRC_TIM1_OC5_COMP2);
	}
#endif
	if (rising) {
		LL_EXTI_DisableRisingTrig_0_31(LL_EXTI_LINE_22);
		LL_EXTI_DisableRisingTrig_0_31(LL_EXTI_LINE_21);
		LL_EXTI_EnableFallingTrig_0_31(current_EXTI_LINE);
	} else { // falling bemf
		LL_EXTI_EnableRisingTrig_0_31(current_EXTI_LINE);
		LL_EXTI_DisableFallingTrig_0_31(LL_EXTI_LINE_21);
		LL_EXTI_DisableFallingTrig_0_31(LL_EXTI_LINE_22);
	}
}

// void changeCompInput() {
//	if (step == 1 || step == 4) {   // c floating
//		COMP2->CSR = 0x000281;
//	}

//	if (step == 2 || step == 5) {     // a floating
//		COMP2->CSR = 0x000261;
//	}

//	if (step == 3 || step == 6) {      // b floating
//		COMP2->CSR = 0x000271;
//	}
//	if (rising){
//		  EXTI->RTSR1 &= ~(LL_EXTI_LINE_18);
//		  EXTI->FTSR1 |= LL_EXTI_LINE_18;
//	}else{                          // falling bemf
//		  EXTI->RTSR1 |= LL_EXTI_LINE_18;
//		  EXTI->FTSR1 &= ~(LL_EXTI_LINE_18);
//	}
//}
// void changeCompInput() {
//	if (step == 1 || step == 4) {   // c floating
// #ifdef INVERTED_COMP
//		COMP2->CSR = (((((COMP2->CSR))) & (~((0xFUL << (4U)) | (0x3UL
//<< (8U))))) | (COMP_COMMON | PHASE_C_COMP)); #else 		COMP2->CSR =
//(((((COMP2->CSR))) & (~((0xFUL << (4U)) | (0x3UL << (8U))))) | (PHASE_C_COMP
//|
// LL_COMP_INPUT_PLUS_IO3)); #endif
//	}

//	if (step == 2 || step == 5) {     // a floating
// #ifdef INVERTED_COMP
//		COMP2->CSR = (((((COMP2->CSR))) & (~((0xFUL << (4U)) | (0x3UL
//<< (8U))))) | (COMP_COMMON | PHASE_A_COMP)); #else 		COMP2->CSR =
//(((((COMP2->CSR))) & (~((0xFUL << (4U)) | (0x3UL << (8U))))) | (PHASE_A_COMP
//|
// LL_COMP_INPUT_PLUS_IO3)); #endif

//	}
//	if (step == 3 || step == 6) {      // b floating
// #ifdef INVERTED_COMP
//		COMP2->CSR = (((((COMP2->CSR))) & (~((0xFUL << (4U)) | (0x3UL
//<< (8U))))) | (COMP_COMMON | PHASE_B_COMP)); #else 		COMP2->CSR =
//(((((COMP2->CSR))) & (~((0xFUL << (4U)) | (0x3UL << (8U))))) | (PHASE_B_COMP
//|
// LL_COMP_INPUT_PLUS_IO3)); #endif

//	}

//	if (rising){
//		  EXTI->RTSR1 &= ~(LL_EXTI_LINE_18);
//		  EXTI->FTSR1 |= LL_EXTI_LINE_18;
//	}else{                          // falling bemf
//		  EXTI->RTSR1 |= LL_EXTI_LINE_18;
//		  EXTI->FTSR1 &= ~(LL_EXTI_LINE_18);

//	}
//}
