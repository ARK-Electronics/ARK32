/*
 * debug_uart.c — USART2 TX PB3 debug console for ARK 12S CAN
 */

#include "targets.h"
#include "debug_uart.h"

#ifdef USE_DEBUG_UART

#	include "main.h"
#	include "esc_state.h"

#	include <stdarg.h>
#	include <stdio.h>
#	include <string.h>

#	include "stm32g4xx_ll_bus.h"
#	include "stm32g4xx_ll_gpio.h"
#	include "stm32g4xx_ll_usart.h"
#	include "stm32g4xx_ll_rcc.h"

/*
 * Event queue: small fixed records so ISRs never block on UART.
 * Main loop formats and transmits.
 */
#	define DBG_Q_LEN 16

typedef struct {
	uint8_t kind; /* 0=empty, 1=state, 2=event */
	uint8_t a;
	uint8_t b;
} dbg_rec_t;

static volatile uint8_t q_head;
static volatile uint8_t q_tail;
static dbg_rec_t q[DBG_Q_LEN];
static uint8_t q_overflow;

static void dbg_putc(char c)
{
	while (!LL_USART_IsActiveFlag_TXE(USART2)) {}
	LL_USART_TransmitData8(USART2, (uint8_t)c);
}

static void dbg_write(const char *s, uint32_t n)
{
	while (n--) {
		dbg_putc(*s++);
	}
	while (!LL_USART_IsActiveFlag_TC(USART2)) {}
}

void debugUartInit(void)
{
	LL_GPIO_InitTypeDef gpio = {0};
	LL_USART_InitTypeDef usart = {0};

	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
	LL_RCC_SetUSARTClockSource(LL_RCC_USART2_CLKSOURCE_PCLK1);

	/* PB3 = USART2_TX AF7 (same as bootloader USE_DEBUG_UART) */
	gpio.Pin = LL_GPIO_PIN_3;
	gpio.Mode = LL_GPIO_MODE_ALTERNATE;
	gpio.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	gpio.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	gpio.Pull = LL_GPIO_PULL_UP;
	gpio.Alternate = LL_GPIO_AF_7;
	LL_GPIO_Init(GPIOB, &gpio);

	/* 115200 8N1, TX only. Sysclk 160 MHz, APB1 undivided. */
	usart.PrescalerValue = LL_USART_PRESCALER_DIV1;
	usart.BaudRate = 115200;
	usart.DataWidth = LL_USART_DATAWIDTH_8B;
	usart.StopBits = LL_USART_STOPBITS_1;
	usart.Parity = LL_USART_PARITY_NONE;
	usart.TransferDirection = LL_USART_DIRECTION_TX;
	usart.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
	usart.OverSampling = LL_USART_OVERSAMPLING_16;
	LL_USART_Init(USART2, &usart);
	LL_USART_DisableFIFO(USART2);
	LL_USART_Enable(USART2);
	while (!LL_USART_IsActiveFlag_TEACK(USART2)) {}

	q_head = q_tail = 0;
	q_overflow = 0;

	debugUartPrint("\r\nARK_G431_CAN debug UART @ 115200 (PB3/USART2)\r\n");
}

void debugUartPrint(const char *msg)
{
	if (!msg) {
		return;
	}
	dbg_write(msg, (uint32_t)strlen(msg));
}

void debugUartPrintf(const char *fmt, ...)
{
	char buf[96];
	va_list ap;
	va_start(ap, fmt);
	int n = vsnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);
	if (n <= 0) {
		return;
	}
	if (n >= (int)sizeof(buf)) {
		n = (int)sizeof(buf) - 1;
	}
	dbg_write(buf, (uint32_t)n);
}

static void q_push(uint8_t kind, uint8_t a, uint8_t b)
{
	uint8_t next = (uint8_t)((q_head + 1u) % DBG_Q_LEN);
	if (next == q_tail) {
		q_overflow = 1;
		return;
	}
	q[q_head].kind = kind;
	q[q_head].a = a;
	q[q_head].b = b;
	q_head = next;
}

void debugUartLogState(uint8_t from, uint8_t to)
{
	if (from == to) {
		return;
	}
	q_push(1, from, to);
}

void debugUartLogEvent(uint8_t event)
{
	if (event == 0) {
		return;
	}
	/*
	 * Drop duplicate consecutive fault events so a latched rail does not
	 * flood the UART (e.g. stuck polled every main-loop tick).
	 */
	if (q_head != q_tail) {
		uint8_t last = (uint8_t)((q_head + DBG_Q_LEN - 1u) % DBG_Q_LEN);
		if (q[last].kind == 2 && q[last].a == event) {
			return;
		}
	}
	q_push(2, event, 0);
}

static const char *evt_name(uint8_t e)
{
	switch (e) {
		case DBG_EVT_BOOT:
			return "boot";
		case DBG_EVT_NFAULT:
			return "nFAULT";
		case DBG_EVT_NFAULT_UVLO:
			return "nFAULT UVLO";
		case DBG_EVT_NFAULT_OCP:
			return "nFAULT OCP";
		case DBG_EVT_NFAULT_OTW:
			return "nFAULT OTW";
		case DBG_EVT_NFAULT_OTSD:
			return "nFAULT OTSD";
		case DBG_EVT_NFAULT_RETRY:
			return "nFAULT retry";
		case DBG_EVT_STUCK:
			return "stuck";
		case DBG_EVT_LVC:
			return "LVC";
		case DBG_EVT_SIGNAL:
			return "signal_lost";
		case DBG_EVT_DESYNC:
			return "desync";
		case DBG_EVT_ACQ_DESYNC:
			return "acq_desync";
		case DBG_EVT_STALL:
			return "stall";
		case DBG_EVT_GD_WAKE:
			return "gd_wake";
		case DBG_EVT_GD_SLEEP:
			return "gd_sleep";
		default:
			return "event";
	}
}

void debugUartService(void)
{
	if (q_overflow) {
		q_overflow = 0;
		debugUartPrint("dbg: queue overflow\r\n");
	}

	while (q_tail != q_head) {
		dbg_rec_t rec = q[q_tail];
		q_tail = (uint8_t)((q_tail + 1u) % DBG_Q_LEN);

		if (rec.kind == 1) {
			/* state change */
			const char *from = escStateName((esc_state_t)rec.a);
			const char *to = escStateName((esc_state_t)rec.b);
			debugUartPrintf("esc: %s -> %s\r\n", from, to);
		} else if (rec.kind == 2) {
			/* OTW / VDS-retry are warnings: HWCI aborts on "fault: nFAULT". */
			const char *pfx = (rec.a == DBG_EVT_NFAULT_OTW || rec.a == DBG_EVT_NFAULT_RETRY) ? "warn" : "fault";
			debugUartPrintf("%s: %s\r\n", pfx, evt_name(rec.a));
		}
	}
}

#endif /* USE_DEBUG_UART */
