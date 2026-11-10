/*
  ADC.c - SITL. Converts the simulation sensor values into the raw ADC
  counts the firmware expects, inverting the conversions in main.c
 */

#include "ADC.h"

#include "sitl.h"
#include "targets.h"

extern uint16_t ADC_raw_temp;
extern uint16_t ADC_raw_volts;
extern uint16_t ADC_raw_current;

/* Written by the sim thread (sitl_current_inject), consumed here on the
 * firmware thread. Plain volatile words; the SITL assumes x86 TSO. */
static volatile int16_t sitl_current_offset_ca;
static volatile uint16_t sitl_current_spike_raw;
static volatile uint8_t sitl_current_spike_arm;

void sitl_current_inject(int16_t offset_ca, uint16_t spike_raw, bool arm_spike)
{
	/* Raw and offset land before the arm flag so a conversion that observes
	 * arm == 1 also observes the spike count that goes with it. */
	sitl_current_spike_raw = spike_raw;
	sitl_current_offset_ca = offset_ca;
	sitl_current_spike_arm = arm_spike ? 1u : 0u;
}

void ADC_DMA_Callback(void)
{
	sitl_sensors_t s;
	sitl_sensors_read(&s);

	// main.c: battery_voltage(10mV) = raw * 3300 / 4095 * VOLTAGE_DIVIDER / 100
	const float pin_mv_volts = s.bus_voltage * 1000.0f * 10.0f / TARGET_VOLTAGE_DIVIDER;
	ADC_raw_volts = (uint16_t)(pin_mv_volts * 4095.0f / 3300.0f + 0.5f);

	// main.c: actual_current(10mA) = ((raw*3300/41) - CURRENT_OFFSET*100) / MILLIVOLT_PER_AMP
	float pin_mv_current = (s.bus_current + (float)sitl_current_offset_ca * 0.01f) * MILLIVOLT_PER_AMP + CURRENT_OFFSET;
	if (pin_mv_current < 0) {
		pin_mv_current = 0;
	}
	float counts = pin_mv_current * 4095.0f / 3300.0f + 0.5f;
	if (sitl_current_spike_arm) {
		sitl_current_spike_arm = 0;
		counts = (float)sitl_current_spike_raw;
	}
	if (counts < 0.0f) {
		counts = 0.0f;
	}
	if (counts > 4095.0f) {
		counts = 4095.0f;
	}
	ADC_raw_current = (uint16_t)counts;

	// __LL_ADC_CALC_TEMPERATURE is a pass through in SITL
	ADC_raw_temp = (uint16_t)(s.temperature_c + 0.5f);
}

void ADC_Init(void) {}
void enableADC_DMA(void) {}
void activateADC(void) {}
