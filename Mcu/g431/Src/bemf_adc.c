/*
 * bemf_adc.c - ADC zero-cross detection for the floating phase (STM32G4)
 *
 * WHY
 * ---
 * The comparator path has a hard low-RPM floor that no amount of filtering or
 * blanking moves. G4 comparator input offset is single-digit mV and the
 * smallest hysteresis step is 10 mV, so once the differential BEMF presented
 * to the comparator approaches their sum the edge simply stops being
 * produced. On this article (360 KV, 14 pole pairs, 21:1 sense divider) that
 * lands around 350-450 rpm, and it is why changeCompInput() has to keep
 * hysteresis off at crawl - 10 mV was blocking real edges.
 *
 * The ADC sees the same node with 0.806 mV per LSB (12 bit, 3V3) and, more
 * usefully, returns a MAGNITUDE rather than a sign. That buys three things
 * the comparator cannot give:
 *
 *   1. Resolution below the comparator's floor. During the PWM on-window the
 *      floating phase sits at Vbus/2 + e and SENS_COMMON at Vbus/2 + e/3, so
 *      the pair differ by (2/3)e, or e/31.5 after the divider. At 350 rpm
 *      that is ~31 mV = 38 LSB; at 150 rpm still ~16 LSB.
 *   2. Common-mode rejection in software. Both nodes ride Vbus/2, and
 *      subtracting them removes bus ripple that the comparator has to reject
 *      with its own (fixed) input stage.
 *   3. Sub-period timing. The comparator reports an edge whenever it happens;
 *      the ADC samples on a grid, which would normally be worse. But two
 *      samples that straddle the crossing give its position BETWEEN them by
 *      linear interpolation, so the timestamp is not quantised to the PWM
 *      period the way the turn-on pile-up in bemf_zc.c is.
 *
 * WHAT IT DOES NOT DO
 * -------------------
 * It does not sample in the PWM off-window. During freewheel both driven
 * phases are clamped to ground, the floating node sits at (3/2)e and
 * SENS_COMMON at e/2 - both near or below 0 V for half of every electrical
 * revolution, which neither the ADC nor the comparator can resolve. That is
 * the same reason bemf_zc.c says off-window crossings are invisible and only
 * register at the next turn-on. Off-window sampling is not a lever on this
 * hardware topology; the usable window is the PWM on-time, and the ADC path
 * lives inside it exactly like the comparator does.
 *
 * Nor does it use the ADC's hardware oversampler. Averaging N SAR
 * conversions multiplies the conversion time by N, and the on-window at the
 * 6 % startup tier is only ~2.5 us wide. Averaging happens across PWM periods
 * instead, in the sense that the interpolation uses two samples and the
 * dead-band threshold rejects single-sample noise - and unlike a fixed
 * oversampling ratio, that is naturally RPM-adaptive, because a slow rotor
 * simply spends more periods approaching the crossing.
 *
 * HOW
 * ---
 * ADC2 is untouched by ADC_Init() on this target (ADC1 alone carries temp,
 * Vbus and current), so the whole instance is available. Its injected group
 * runs two ranks - floating phase, then SENS_COMMON - triggered from TIM1
 * TRGO2 driven by OC4REF, which puts the sample pair at a chosen offset
 * inside the on-window. Injected is the right group: it needs no DMA, it has
 * its own trigger and result registers, and it would preempt rather than
 * disturb a regular sequence if ADC2 ever gains one.
 *
 * Ownership is decided per step in bemfAdcArm(), called from
 * enableCompInterrupts(). When the ADC path owns a step the comparator EXTI
 * stays masked and this file commits the crossing through
 * bemfZcAcceptCrossing() - the same path interruptRoutine() uses, so blind
 * stepping, the miss bucket, the demag-late rail and the commutation schedule
 * all behave identically. A search that finds nothing degrades into the
 * existing missed-ZC deadline and a blind step; it cannot hang the loop.
 */

#include "bemf_adc.h"

#ifdef USE_ADC_BEMF

#	include "bemf_zc.h"
#	include "common.h"
#	include "main.h"
#	include "motor_runtime.h"

/*
 * TIM1 runs at 160 MHz and INTERVAL_TIMER at 2 MHz, so one INTERVAL tick is
 * 80 TIM1 ticks. Used to convert the interpolated fraction of a PWM period
 * into the units bemfZcAcceptCrossing() back-dates in.
 */
#	define ADC_BEMF_TIM1_PER_INTERVAL_TICK 80u

/*
 * Cap on how many PWM periods may separate the last clearly-pre-crossing
 * sample from the one that resolves the crossing. Saturating rather than
 * discarding the reference matters: at crawl rpm the differential creeps
 * through the dead band for many periods, and dropping the reference there
 * would disable detection in exactly the case this path exists for. A
 * saturated span under-estimates the back-date, which lands the timestamp
 * slightly late - the safe direction.
 */
#	define ADC_BEMF_MAX_SPAN 32u

static uint16_t adc_ref_mag;  /* |differential| at the last pre-crossing sample */
static uint16_t adc_span;     /* PWM periods since that sample */
static uint32_t adc_searches; /* steps this path owned */
static uint32_t adc_accepts;  /* crossings it committed */
static uint8_t adc_armed;

static uint32_t bemfAdcPhaseChannel(void)
{
	if (step == 1 || step == 4) {
		return ADC_BEMF_PHASE_C_CHANNEL;
	}
	if (step == 2 || step == 5) {
		return ADC_BEMF_PHASE_A_CHANNEL;
	}
	return ADC_BEMF_PHASE_B_CHANNEL;
}

void bemfAdcInit(void)
{
	/*
	 * TIM1 CH4 drives the sample point. PWM2 (inactive while CNT < CCR4)
	 * makes OC4REF RISE at CNT == CCR4, which is what the ADC triggers on;
	 * PWM1 would rise at CNT == 0, inside the dead time. CC4E is
	 * deliberately left disabled - OC4REF is generated regardless, TRGO2
	 * taps it internally, and TIM1_CH4's pin on this package is PA11,
	 * which the board uses for FDCAN_RX.
	 */
	LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH4, LL_TIM_OCMODE_PWM2);
	LL_TIM_OC_SetCompareCH4(TIM1, ADC_BEMF_TRIGGER_TICKS);
	LL_TIM_SetTriggerOutput2(TIM1, LL_TIM_TRGO2_OC4);

	/*
	 * ADC2 bring-up. The ADC12 common block (clock source and CKMODE =
	 * PCLK/4 = 40 MHz) is already configured by ADC_Init() for ADC1 and is
	 * shared, so it must not be re-initialised here.
	 */
	if (LL_ADC_IsEnabled(ADC2) == 0) {
		LL_ADC_DisableDeepPowerDown(ADC2);
		LL_ADC_EnableInternalRegulator(ADC2);
		uint32_t wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
		while (wait_loop_index != 0) {
			wait_loop_index--;
		}

		LL_ADC_StartCalibration(ADC2, LL_ADC_SINGLE_ENDED);
		while (LL_ADC_IsCalibrationOnGoing(ADC2) != 0) {}
		wait_loop_index = (LL_ADC_DELAY_CALIB_ENABLE_ADC_CYCLES * 64) >> 1;
		while (wait_loop_index != 0) {
			wait_loop_index--;
		}
	}

	LL_ADC_SetResolution(ADC2, LL_ADC_RESOLUTION_12B);
	LL_ADC_SetDataAlignment(ADC2, LL_ADC_DATA_ALIGN_RIGHT);
	LL_ADC_SetOverSamplingScope(ADC2, LL_ADC_OVS_DISABLE);

	/*
	 * Queue disabled so a JSQR write takes effect directly. With the queue
	 * enabled (reset default) each write pushes a context, which is the
	 * wrong model for a sequence that is rewritten once per commutation.
	 * Must be set while JADSTART is clear, i.e. here.
	 */
	LL_ADC_INJ_SetQueueMode(ADC2, LL_ADC_INJ_QUEUE_DISABLE);
	LL_ADC_INJ_SetSequencerDiscont(ADC2, LL_ADC_INJ_SEQ_DISCONT_DISABLE);
	LL_ADC_INJ_SetSequencerLength(ADC2, LL_ADC_INJ_SEQ_SCAN_ENABLE_2RANKS);
	LL_ADC_INJ_SetTriggerSource(ADC2, LL_ADC_INJ_TRIG_EXT_TIM1_TRGO2);
	LL_ADC_INJ_SetTriggerEdge(ADC2, LL_ADC_INJ_TRIG_EXT_RISING);

	/*
	 * 6.5 ADC cycles at 40 MHz is 162 ns of sampling. The sense node is a
	 * ~952 ohm source into the ~5 pF sample-and-hold, so the time constant
	 * is ~10 ns and 162 ns settles well past 12-bit. Total per rank is
	 * 6.5 + 12.5 = 19 cycles = 475 ns, so the pair completes 950 ns after
	 * the trigger - the number ADC_BEMF_MIN_DUTY_TICKS is built from.
	 */
	static const uint32_t channels[4] = {ADC_BEMF_PHASE_A_CHANNEL, ADC_BEMF_PHASE_B_CHANNEL, ADC_BEMF_PHASE_C_CHANNEL,
					     ADC_BEMF_COMMON_CHANNEL};
	for (uint32_t i = 0; i < 4; i++) {
		LL_ADC_SetChannelSamplingTime(ADC2, channels[i], LL_ADC_SAMPLINGTIME_6CYCLES_5);
		LL_ADC_SetChannelSingleDiff(ADC2, channels[i], LL_ADC_SINGLE_ENDED);
	}

	LL_ADC_INJ_SetSequencerRanks(ADC2, LL_ADC_INJ_RANK_1, ADC_BEMF_PHASE_A_CHANNEL);
	LL_ADC_INJ_SetSequencerRanks(ADC2, LL_ADC_INJ_RANK_2, ADC_BEMF_COMMON_CHANNEL);

	if (LL_ADC_IsEnabled(ADC2) == 0) {
		LL_ADC_Enable(ADC2);
		while (LL_ADC_IsActiveFlag_ADRDY(ADC2) == 0) {}
	}

	/* Same priority class as the comparator EXTI it stands in for. */
	NVIC_SetPriority(ADC1_2_IRQn, 0);
	NVIC_EnableIRQ(ADC1_2_IRQn);
}

uint8_t bemfAdcArm(void)
{
	/*
	 * Ownership, decided fresh at every arm rather than latched at
	 * commutation, so a throttle or speed change takes effect on the next
	 * step rather than the one after.
	 *
	 *  - slow enough that the comparator is near or past its floor, which
	 *    is the only reason to pay for this path
	 *  - enough on-window to fit the sample pair after the trigger offset;
	 *    below that the second conversion straddles turn-off and returns
	 *    freewheel garbage
	 */
	if (average_interval < ADC_BEMF_MIN_INTERVAL || adjusted_duty_cycle < ADC_BEMF_MIN_DUTY_TICKS) {
		return 0;
	}

	LL_ADC_INJ_StopConversion(ADC2);
	while (LL_ADC_INJ_IsStopConversionOngoing(ADC2) != 0) {}

	LL_ADC_INJ_SetSequencerRanks(ADC2, LL_ADC_INJ_RANK_1, bemfAdcPhaseChannel());

	adc_ref_mag = 0;
	adc_span = 0;
	adc_armed = 1;
	adc_searches++;

	LL_ADC_ClearFlag_JEOS(ADC2);
	LL_ADC_EnableIT_JEOS(ADC2);
	LL_ADC_INJ_StartConversion(ADC2);
	return 1;
}

void bemfAdcDisarm(void)
{
	if (!adc_armed) {
		return;
	}
	adc_armed = 0;
	LL_ADC_DisableIT_JEOS(ADC2);
	LL_ADC_INJ_StopConversion(ADC2);
}

RAM_FUNC void bemfAdcIrqHandler(void)
{
	if (LL_ADC_IsActiveFlag_JEOS(ADC2) == 0) {
		return;
	}
	LL_ADC_ClearFlag_JEOS(ADC2);

	if (!adc_armed) {
		return;
	}

	/*
	 * Same post-commutation blanking COMP1_2_3_IRQHandler applies to the
	 * comparator edge: nothing before half the expected interval is a
	 * crossing, it is the freewheel demag clamp on the phase that was just
	 * released. Returning before the reference is taken also keeps demag
	 * readings out of the interpolation.
	 */
	if (INTERVAL_TIMER->CNT <= (commutation_interval >> 1)) {
		return;
	}

	/*
	 * The on-window can collapse under the sample pair mid-search (a
	 * throttle chop, or the low-rpm ceiling pulling duty down). Drop the
	 * sample without ageing the reference: duty can recover inside the
	 * same search, and a freewheel reading would otherwise be interpreted
	 * as a crossing.
	 */
	if (adjusted_duty_cycle < ADC_BEMF_MIN_DUTY_TICKS) {
		return;
	}

	const int32_t phase = (int32_t)LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_1);
	const int32_t common = (int32_t)LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_2);

	/*
	 * Normalise so that positive always means "past the crossing",
	 * whichever direction this step expects. During the on-window the pair
	 * differ by (2/3)e, so the raw difference carries the sign of the
	 * floating phase's BEMF: a rising step crosses from negative to
	 * positive, a falling step the other way. This matches the comparator,
	 * whose output is (SENS_COMMON > phase) and so reads high exactly when
	 * the raw difference is negative.
	 */
	int32_t diff = phase - common;
	if (!rising) {
		diff = -diff;
	}

	if (adc_ref_mag != 0 && adc_span < ADC_BEMF_MAX_SPAN) {
		adc_span++;
	}

	if (diff <= -(int32_t)ADC_BEMF_MIN_LSB) {
		/* Unambiguously before the crossing - this is the reference the
		 * interpolation will run from. */
		adc_ref_mag = (uint16_t)(-diff);
		adc_span = 0;
		return;
	}

	if (diff < (int32_t)ADC_BEMF_MIN_LSB || adc_ref_mag == 0) {
		/* Inside the dead band, or nothing to interpolate from yet. */
		return;
	}

	/*
	 * Straddled. The crossing lies between the reference sample and this
	 * one, at the fraction ref/(ref + now) of the way across, so it
	 * happened now/(ref + now) of the span ago. Converting the span from
	 * PWM periods to INTERVAL_TIMER ticks gives the back-date directly.
	 *
	 * Worst case magnitudes: 4095 * 32 * 83 fits an unsigned 32-bit
	 * product with room to spare.
	 */
	const uint32_t mag_now = (uint32_t)diff;
	const uint32_t mag_ref = adc_ref_mag;
	const uint32_t span = (adc_span != 0) ? adc_span : 1u;
	const uint32_t period_ticks = (uint32_t)tim1_arr / ADC_BEMF_TIM1_PER_INTERVAL_TICK;

	uint32_t back = (mag_now * span * period_ticks) / (mag_ref + mag_now);

	/* Same backstop the comparator's pile-up estimate uses: a degenerate
	 * interpolation must not be able to rewrite the timestamp by more than
	 * a fraction of the interval it is measuring. */
	const uint32_t cap = commutation_interval >> 2;
	if (back > cap) {
		back = cap;
	}

	adc_armed = 0;
	adc_accepts++;
	bemfZcAcceptCrossing((uint16_t)back);
}

uint32_t bemfAdcGetSearches(void)
{
	return adc_searches;
}

uint32_t bemfAdcGetAccepts(void)
{
	return adc_accepts;
}

#endif /* USE_ADC_BEMF */
