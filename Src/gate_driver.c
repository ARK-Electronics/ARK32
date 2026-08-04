/*
 * gate_driver.c - sleep/wake for DRV8350 ENABLE / DRV8328 nSLEEP
 */

#include "gate_driver.h"

#if GATE_DRIVER_SLEEP_SUPPORT

#	include "functions.h"
#	include "main.h"
#	include "motor_runtime.h"
#	include "targets.h"

/* Datasheet tWAKE: after nSLEEP/ENABLE rises, wait before gate inputs are valid.
 * Pin stays high; this is not a toggle hold — bias/charge pump must come up. */
#	define GD_WAKE_US 1000u
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
	GD_PORT->BSRR = GD_PIN;
	delayMicros(GD_WAKE_US); /* tWAKE — see GD_WAKE_US */
	gate_driver_awake = 1;
}

void gateDriverSleep(void)
{
	GD_PORT->BRR = GD_PIN;
	gate_driver_awake = 0;
}

void gateDriverFaultResetPulse(void)
{
#	if defined(USE_DRV_ENABLE)
	GD_PORT->BRR = GD_PIN;
	delayMicros(GD_FAULT_RST_US);
	gate_driver_awake = 0;
#	endif
	gateDriverWakeBlocking();
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
