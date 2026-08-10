/*
 * gate_driver.c - sleep/wake for DRV8350 ENABLE / DRV8328 nSLEEP
 */

#include "gate_driver.h"

#if GATE_DRIVER_SLEEP_SUPPORT

#	include "functions.h"
#	include "main.h" /* GPIO_TypeDef / BRR / BSRR */
#	include "motor_runtime.h"
#	include "targets.h"
#	if defined(USE_DRV_ENABLE)
/* DRV8350H on ARK 12S CAN: force bridge inputs inactive before ENABLE. */
#		include "peripherals.h"
#		include "phaseouts.h"
#	endif

/*
 * After nSLEEP/ENABLE rises, wait before gate inputs are valid (pin stays high).
 * DRV8350H (USE_DRV_ENABLE): datasheet t_WAKE ~1 ms is a minimum; charge pump
 * and bootstrap need longer or a half-bridge mismatch blips the motor. Use 3 ms.
 * DRV8328 nSLEEP (ARK 4in1): 1 ms is enough in practice.
 */
#	if defined(USE_DRV_ENABLE)
#		define GD_WAKE_US 3000u
#	else
#		define GD_WAKE_US 1000u
#	endif
#	define GD_FAULT_RST_US 50u

#	if defined(USE_DRV_ENABLE)
#		define GD_PORT DRV_ENABLE_PORT
#		define GD_PIN DRV_ENABLE_PIN
#	elif defined(USE_DRV8328_NSLEEP)
#		define GD_PORT NSLEEP_PORT
#		define GD_PIN NSLEEP_PIN
#	endif

volatile uint8_t gate_driver_awake;

void gateDriverInit(void)
{
	LL_GPIO_InitTypeDef s = {0};
	s.Pin = GD_PIN;
	s.Mode = LL_GPIO_MODE_OUTPUT;
	s.Speed = LL_GPIO_SPEED_FREQ_LOW;
	s.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	s.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GD_PORT, &s);
	GD_PORT->BRR = GD_PIN;
	gate_driver_awake = 0;
}

void gateDriverWakeBlocking(void)
{
	if (gate_driver_awake) {
		return; /* already high; no delay */
	}
#	if defined(USE_DRV_ENABLE)
	/*
	 * DRV8350H wake sequence (required to avoid motor blip):
	 *   1) Force all INHx/INLx inactive (6x mode: both low via allOff)
	 *   2) Zero TIM CCRs so AF re-entry cannot drive non-zero duty
	 *   3) Assert ENABLE high
	 *   4) Wait GD_WAKE_US for charge pump / VGLS / bootstrap settle
	 *   5) Caller may then apply PWM / commutation
	 */
	allOff();
	SET_DUTY_CYCLE_ALL(0);
#	endif
	GD_PORT->BSRR = GD_PIN;
	delayMicros(GD_WAKE_US);
	gate_driver_awake = 1;
}

void gateDriverSleep(void)
{
	if (!gate_driver_awake) {
		return;
	}
#	if defined(USE_DRV_ENABLE)
	/* Leave bridge inputs inactive while ENABLE is low (internal gate PDs). */
	allOff();
	SET_DUTY_CYCLE_ALL(0);
#	endif
	GD_PORT->BRR = GD_PIN;
	gate_driver_awake = 0;
}

void gateDriverFaultResetPulse(void)
{
	/* DRV8350H ENABLE pulse only; nSLEEP boards do not need t_RST. */
#	if defined(USE_DRV_ENABLE)
	/* PWM inactive for the whole reset+wake; wake re-does force-low + t_settle. */
	allOff();
	SET_DUTY_CYCLE_ALL(0);
	GD_PORT->BRR = GD_PIN;
	delayMicros(GD_FAULT_RST_US);
	gate_driver_awake = 0;
	gateDriverWakeBlocking();
#	endif
}

void gateDriverPoll(void)
{
	if (running || stepper_sine || prop_brake_active) {
		gateDriverWakeBlocking();
	} else {
		gateDriverSleep();
	}
}

#endif /* GATE_DRIVER_SLEEP_SUPPORT */
