/*
 * Speed values reported from the electrical period (e_com_time, us per
 * electrical revolution). Self-contained so the conversions can be tested on
 * the host (Mcu/SITL/tests/test_telemetry_rpm.py).
 */
#ifndef TELEMETRY_RPM_H
#define TELEMETRY_RPM_H

#include <stdint.h>

/* e_com_time is no measurement below this: it is 2 us from boot until the first
 * commutation interval has been timed. 10 us is 6,000,000 erpm, beyond any motor,
 * and keeps 600000 / e_com_time within the 16-bit e_rpm. */
#define E_COM_TIME_MIN_US 10

/* uavcan.equipment.esc.Status rpm is int18 */
#define ESC_STATUS_RPM_MAX ((int32_t)((1L << 17) - 1))

/* DShot telemetry period that reports no rotation */
#define DSHOT_TELEMETRY_PERIOD_STOPPED 65535u

/* e_rpm (erpm / 100) for an electrical period; an unmeasured period reads as 0 */
static inline uint16_t erpmHundredsFromComTime(int32_t com_time_us)
{
	return (com_time_us >= E_COM_TIME_MIN_US) ? (uint16_t)(600000 / com_time_us) : 0;
}

/* esc.Status rpm from e_rpm, saturated instead of wrapping the int18 field */
static inline int32_t escStatusRpm(uint16_t e_rpm, uint8_t motor_poles)
{
	if (motor_poles == 0) {
		return 0;
	}

	const int32_t rpm = ((int32_t)e_rpm * 200) / motor_poles;
	return (rpm > ESC_STATUS_RPM_MAX) ? ESC_STATUS_RPM_MAX : rpm;
}

/* Bidirectional DShot electrical period; no rotation when stopped, slower than
 * the 16-bit range, or not yet measured after boot */
static inline uint16_t dshotTelemetryPeriodUs(uint32_t com_time_us, uint8_t running)
{
	if (!running || com_time_us < E_COM_TIME_MIN_US || com_time_us > DSHOT_TELEMETRY_PERIOD_STOPPED) {
		return DSHOT_TELEMETRY_PERIOD_STOPPED;
	}

	return (uint16_t)com_time_us;
}

#endif
