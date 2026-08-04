/*
 * debug_uart.h — ARK 12S CAN debug console (USART2 TX on PB3 AF7 @ 115200)
 *
 * Same pinout as AM32-bootloader#60 / ARK bootloader USE_DEBUG_UART.
 * App uses polled TX (not DMA1 CH1 — that is the DShot capture channel).
 *
 * Opt in with USE_DEBUG_UART on the target (ARK_G431_CAN).
 *
 * ISR-safe path: debugUartLog*() queues events; debugUartService() drains
 * them from the main loop with blocking UART writes.
 */
#ifndef DEBUG_UART_H_
#define DEBUG_UART_H_

#include <stdint.h>
#include "targets.h"

/* Event codes for debugUartLogEvent() — always defined so call sites compile
 * on targets without USE_DEBUG_UART (stubs are no-ops). */
enum {
	DBG_EVT_BOOT = 1,
	DBG_EVT_NFAULT,
	DBG_EVT_STUCK,
	DBG_EVT_LVC,
	DBG_EVT_SIGNAL,
	DBG_EVT_DESYNC,
	DBG_EVT_ACQ_DESYNC,
	DBG_EVT_STALL,
	DBG_EVT_GD_WAKE,
	DBG_EVT_GD_SLEEP,
};

#ifdef USE_DEBUG_UART

void debugUartInit(void);
/* Blocking print (main / init only — not from ISRs). */
void debugUartPrint(const char *msg);
void debugUartPrintf(const char *fmt, ...);
/* Queue a state transition (ISR-safe). */
void debugUartLogState(uint8_t from, uint8_t to);
/* Queue a named event (ISR-safe). */
void debugUartLogEvent(uint8_t event);
/* Drain the queue — call once per main-loop tick. */
void debugUartService(void);

#else /* !USE_DEBUG_UART */

/* Macros (not empty inlines): format-string args are not evaluated and
 * do not land in F051 .rodata when the target has no debug UART. */
#	define debugUartInit() ((void)0)
#	define debugUartPrint(msg) ((void)0)
#	define debugUartPrintf(...) ((void)0)
#	define debugUartLogState(from, to) ((void)0)
#	define debugUartLogEvent(event) ((void)0)
#	define debugUartService() ((void)0)

#endif /* USE_DEBUG_UART */

#endif /* DEBUG_UART_H_ */
