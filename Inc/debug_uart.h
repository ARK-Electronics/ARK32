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

/* Event codes for debugUartLogEvent() */
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

#else /* !USE_DEBUG_UART */

static inline void debugUartInit(void) {}
static inline void debugUartPrint(const char *msg)
{
	(void)msg;
}
static inline void debugUartPrintf(const char *fmt, ...)
{
	(void)fmt;
}
static inline void debugUartLogState(uint8_t from, uint8_t to)
{
	(void)from;
	(void)to;
}
static inline void debugUartLogEvent(uint8_t event)
{
	(void)event;
}
static inline void debugUartService(void) {}

#endif /* USE_DEBUG_UART */

#endif /* DEBUG_UART_H_ */
