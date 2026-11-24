/*
 * Host harness for test_telemetry_rpm.py: checks the conversions in
 * Inc/telemetry_rpm.h. Prints "OK <count>" or one "FAIL ..." line per mismatch.
 */
#include <stdint.h>
#include <stdio.h>

#include "telemetry_rpm.h"

static int checks;
static int failures;

static void expect(const char *what, long long got, long long want)
{
	checks++;

	if (got != want) {
		failures++;
		printf("FAIL %s got %lld want %lld\n", what, got, want);
	}
}

int main(void)
{
	/* e_com_time is 2 us from boot until the first commutation interval is timed:
	 * 600000 / 2 = 300000 used to truncate to 37856 in the 16-bit e_rpm. */
	expect("erpm(2 us, unmeasured)", erpmHundredsFromComTime(2), 0);
	expect("erpm(0 us)", erpmHundredsFromComTime(0), 0);
	expect("erpm(-1 us)", erpmHundredsFromComTime(-1), 0);
	expect("erpm(9 us)", erpmHundredsFromComTime(E_COM_TIME_MIN_US - 1), 0);
	expect("erpm(10 us)", erpmHundredsFromComTime(E_COM_TIME_MIN_US), 60000);
	expect("erpm(300 us, 200k erpm)", erpmHundredsFromComTime(300), 2000);
	expect("erpm(5000 us, 12k erpm)", erpmHundredsFromComTime(5000), 120);
	expect("erpm(65535 us)", erpmHundredsFromComTime(65535), 9);

	/* esc.Status rpm: 14 poles at 2414 rpm (the bench) and the saturated int18 */
	expect("status(169, 14)", escStatusRpm(169, 14), 2414);
	expect("status(0, 14)", escStatusRpm(0, 14), 0);
	expect("status(60000, 2) saturates", escStatusRpm(60000, 2), ESC_STATUS_RPM_MAX);
	expect("status(1310, 2) below the limit", escStatusRpm(1310, 2), 131000);
	expect("status(37856, 14) no longer wraps", escStatusRpm(37856, 14), ESC_STATUS_RPM_MAX);
	expect("status(x, 0 poles)", escStatusRpm(1000, 0), 0);

	/* DShot telemetry period */
	expect("dshot(stopped)", dshotTelemetryPeriodUs(300, 0), DSHOT_TELEMETRY_PERIOD_STOPPED);
	expect("dshot(2 us, unmeasured)", dshotTelemetryPeriodUs(2, 1), DSHOT_TELEMETRY_PERIOD_STOPPED);
	expect("dshot(9 us)", dshotTelemetryPeriodUs(E_COM_TIME_MIN_US - 1, 1), DSHOT_TELEMETRY_PERIOD_STOPPED);
	expect("dshot(10 us)", dshotTelemetryPeriodUs(E_COM_TIME_MIN_US, 1), E_COM_TIME_MIN_US);
	expect("dshot(300 us)", dshotTelemetryPeriodUs(300, 1), 300);
	expect("dshot(65535 us)", dshotTelemetryPeriodUs(65535, 1), 65535);
	expect("dshot(70000 us, too slow)", dshotTelemetryPeriodUs(70000, 1), DSHOT_TELEMETRY_PERIOD_STOPPED);

	if (failures == 0) {
		printf("OK %d\n", checks);
	}

	return failures != 0;
}
