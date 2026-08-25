/*
 * sys_can_stm32_CANFD.c - STM32G4 FDCAN, classic CAN and CAN FD.
 *
 * Based on ArduPilot CANFDIface.cpp. Nominal (arbitration) bit timing is
 * 1 Mbps at 80 MHz. Data-phase timings are the Kvaser/ArduPilot 80 MHz table
 * (1/2/4/5 Mbps). EEPROM CAN_FD_MBPS 0 auto-matches the host data rate;
 * 1/2/4/5 pins it. TX FDF/BRS follows CanardCANFrame.canfd (set from RX).
 */

#include "targets.h"

#if DRONECAN_SUPPORT && defined(MCU_G431)

#	include "main.h"
#	include "sys_can.h"
#	include <blutil.h>
#	include <string.h>

#	ifndef MCU_FLASH_START
#		define MCU_FLASH_START 0x08000000
#	endif
#	ifndef EEPROM_START_ADD
#		define EEPROM_START_ADD (MCU_FLASH_START + 0x1f800)
#	endif
#	define EEPROM_FD_MBPS_OFF 185

#	define FDCAN_FRAME_BUFFER_SIZE 18

#	define FDCAN_MESSAGERAM_STRIDE 0x350
#	define FDCAN_EXFILTER_OFFSET 0x70
#	define FDCAN_RXFIFO0_OFFSET 0xB0
#	define FDCAN_RXFIFO1_OFFSET 0x188
#	define FDCAN_TXFIFO_OFFSET 0x278

#	define FDCAN_RXF0S_F0GI_SHIFT 8
#	define FDCAN_RXF1S_F1GI_SHIFT 8

#	define MaskStdID 0x7FFU
#	define MaskExtID 0x1FFFFFFFU

#	define FDCAN_ELMT_FDF (1U << 21)
#	define FDCAN_ELMT_BRS (1U << 20)

/* TDC offset 10 × 12.5 ns = 125 ns (MCP2557FD-class transceiver delay). */
#	define FDCAN_TDCO_TICKS 10U
#	define FD_ERR_STREAK_RETUNE 32U

typedef struct {
	uint32_t id_flags;
	uint32_t dlc_timestamp;
	uint32_t data[16];
} RxMessageRAM;

typedef struct {
	uint32_t id_flags;
	uint32_t dlc_flags;
	uint32_t data[16];
} TxMessageRAM;

volatile struct {
	uint32_t rx_overflow;
} can_stats;

static uint32_t MessageRam_StandardFilterSA;
static uint32_t MessageRam_ExtendedFilterSA;
static uint32_t MessageRam_RxFIFO0SA;
static uint32_t MessageRam_RxFIFO1SA;
static uint32_t MessageRam_TxFIFOQSA;

/*
 * Data-phase timings at 80 MHz FDCAN kernel clock.
 * Values are real (prescaler, bs1, bs2, sjw); registers store minus one.
 * Auto-scan order: 5 Mbps (product max) then 4, 2, 1.
 */
static const struct {
	uint8_t mbps;
	uint8_t prescaler;
	uint8_t bs1;
	uint8_t bs2;
	uint8_t sjw;
} fd_timings[] = {
	{5, 1, 11, 4, 4},
	{4, 1, 14, 5, 5},
	{2, 2, 14, 5, 5},
	{1, 4, 14, 5, 5},
};

static uint8_t fd_idx;
static uint8_t fd_auto;
static uint8_t fd_locked;
static uint8_t saw_classic;
static uint16_t fd_err_streak;
static volatile uint8_t fd_need_retune;

static uint8_t dlc_to_len(uint8_t dlc)
{
	static const uint8_t map[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
	return map[dlc & 0xF];
}

static uint8_t len_to_dlc(uint8_t len)
{
	if (len <= 8) {
		return len;
	}
	if (len <= 12) {
		return 9;
	}
	if (len <= 16) {
		return 10;
	}
	if (len <= 20) {
		return 11;
	}
	if (len <= 24) {
		return 12;
	}
	if (len <= 32) {
		return 13;
	}
	if (len <= 48) {
		return 14;
	}
	return 15;
}

static int8_t fd_idx_for_mbps(uint8_t mbps)
{
	for (uint8_t i = 0; i < (uint8_t)(sizeof(fd_timings) / sizeof(fd_timings[0])); i++) {
		if (fd_timings[i].mbps == mbps) {
			return (int8_t)i;
		}
	}
	return -1;
}

static void apply_fd_timings(uint8_t idx)
{
	const uint8_t n = (uint8_t)(sizeof(fd_timings) / sizeof(fd_timings[0]));
	if (idx >= n) {
		idx = 0;
	}
	const uint8_t presc = fd_timings[idx].prescaler;
	const uint8_t bs1 = fd_timings[idx].bs1;
	const uint8_t bs2 = fd_timings[idx].bs2;
	const uint8_t sjw = fd_timings[idx].sjw;
	FDCAN1->DBTP = (((uint32_t)(sjw - 1) << FDCAN_DBTP_DSJW_Pos) | ((uint32_t)(bs1 - 1) << FDCAN_DBTP_DTSEG1_Pos) |
			((uint32_t)(bs2 - 1) << FDCAN_DBTP_DTSEG2_Pos) | ((uint32_t)(presc - 1) << FDCAN_DBTP_DBRP_Pos) | FDCAN_DBTP_TDC);
	FDCAN1->TDCR = (FDCAN_TDCO_TICKS << FDCAN_TDCR_TDCO_Pos);
	fd_idx = idx;
}

static void setupMessageRam(void)
{
	const uint32_t base = SRAMCAN_BASE + FDCAN_MESSAGERAM_STRIDE * 0;

	memset((void *)base, 0, FDCAN_MESSAGERAM_STRIDE);

	MessageRam_StandardFilterSA = base;
	MessageRam_ExtendedFilterSA = base + FDCAN_EXFILTER_OFFSET;
	MessageRam_RxFIFO0SA = base + FDCAN_RXFIFO0_OFFSET;
	MessageRam_RxFIFO1SA = base + FDCAN_RXFIFO1_OFFSET;
	MessageRam_TxFIFOQSA = base + FDCAN_TXFIFO_OFFSET;

	FDCAN1->TXBC = 0;
	(void)MessageRam_StandardFilterSA;
	(void)MessageRam_ExtendedFilterSA;
}

static bool can_send(const CanardCANFrame *frame)
{
	if ((FDCAN1->TXFQS & FDCAN_TXFQS_TFQF) != 0) {
		return false;
	}

	uint32_t put_index = (FDCAN1->TXFQS & FDCAN_TXFQS_TFQPI) >> FDCAN_TXFQS_TFQPI_Pos;

	volatile TxMessageRAM *tx_mailbox = (volatile TxMessageRAM *)(MessageRam_TxFIFOQSA + (put_index * FDCAN_FRAME_BUFFER_SIZE * 4));

	if (frame->id & CANARD_CAN_FRAME_EFF) {
		tx_mailbox->id_flags = (frame->id & MaskExtID) | (1U << 30);
	} else {
		tx_mailbox->id_flags = (frame->id & MaskStdID) << 18;
	}

	if (frame->id & CANARD_CAN_FRAME_RTR) {
		tx_mailbox->id_flags |= (1U << 29);
	}

	const uint8_t dlc = len_to_dlc(frame->data_len);
	uint32_t dlc_flags = ((uint32_t)dlc << 16);
#	if CANARD_ENABLE_CANFD
	if (frame->canfd) {
		dlc_flags |= FDCAN_ELMT_FDF | FDCAN_ELMT_BRS;
	}
#	endif
	tx_mailbox->dlc_flags = dlc_flags;

	uint8_t nbytes = frame->data_len;
	if (nbytes > 64) {
		nbytes = 64;
	}
	const uint8_t nwords = (uint8_t)((nbytes + 3) / 4);
	const uint32_t *data_ptr = (const uint32_t *)frame->data;
	for (uint8_t i = 0; i < nwords; i++) {
		tx_mailbox->data[i] = data_ptr[i];
	}

	FDCAN1->TXBAR = (1U << put_index);

	return true;
}

static void handleRxInterrupt(uint8_t fifo_index)
{
	volatile uint32_t *fifo_status_reg = (fifo_index == 0) ? &FDCAN1->RXF0S : &FDCAN1->RXF1S;
	volatile uint32_t *fifo_ack_reg = (fifo_index == 0) ? &FDCAN1->RXF0A : &FDCAN1->RXF1A;

	uint32_t fifo_level_mask = (fifo_index == 0) ? FDCAN_RXF0S_F0FL : FDCAN_RXF1S_F1FL;
	uint32_t get_index_mask = (fifo_index == 0) ? FDCAN_RXF0S_F0GI : FDCAN_RXF1S_F1GI;
	uint32_t get_index_shift = (fifo_index == 0) ? FDCAN_RXF0S_F0GI_SHIFT : FDCAN_RXF1S_F1GI_SHIFT;

	if ((*fifo_status_reg & fifo_level_mask) == 0) {
		return;
	}

	uint32_t get_index = (*fifo_status_reg & get_index_mask) >> get_index_shift;

	uint32_t rx_fifo_addr = (fifo_index == 0) ? MessageRam_RxFIFO0SA : MessageRam_RxFIFO1SA;
	volatile RxMessageRAM *rx_mailbox = (volatile RxMessageRAM *)(rx_fifo_addr + (get_index * FDCAN_FRAME_BUFFER_SIZE * 4));

	CanardCANFrame frame = {};

	uint32_t id_flags = rx_mailbox->id_flags;
	if (id_flags & (1U << 30)) {
		frame.id = (id_flags & MaskExtID) | CANARD_CAN_FRAME_EFF;
	} else {
		frame.id = (id_flags >> 18) & MaskStdID;
	}

	if (id_flags & (1U << 29)) {
		frame.id |= CANARD_CAN_FRAME_RTR;
	}

	const uint32_t dlc_word = rx_mailbox->dlc_timestamp;
	const uint8_t dlc = (uint8_t)((dlc_word >> 16) & 0xF);
	const bool is_fd = (dlc_word & FDCAN_ELMT_FDF) != 0;
#	if CANARD_ENABLE_CANFD
	frame.canfd = is_fd;
	frame.data_len = is_fd ? dlc_to_len(dlc) : (dlc > 8 ? 8 : dlc);
#	else
	(void)is_fd;
	frame.data_len = dlc > 8 ? 8 : dlc;
#	endif

	uint8_t nbytes = frame.data_len;
	if (nbytes > (uint8_t)sizeof(frame.data)) {
		nbytes = (uint8_t)sizeof(frame.data);
		frame.data_len = nbytes;
	}
	const uint8_t nwords = (uint8_t)((nbytes + 3) / 4);
	uint32_t *data_ptr = (uint32_t *)frame.data;
	for (uint8_t i = 0; i < nwords; i++) {
		data_ptr[i] = rx_mailbox->data[i];
	}

	*fifo_ack_reg = get_index;

	fd_err_streak = 0;
	if (is_fd) {
		fd_locked = 1;
	} else {
		saw_classic = 1;
	}

	DroneCAN_handleFrame(&frame);
}

static void handleTxCompleteInterrupt(void) {}

static void pollErrorFlagsFromISR(void)
{
	const uint8_t cel = (uint8_t)(FDCAN1->ECR >> 16);
	if (cel == 0) {
		return;
	}
	if (fd_auto && !fd_locked && !saw_classic) {
		if (fd_err_streak < 0xFFFF) {
			fd_err_streak++;
		}
		if (fd_err_streak >= FD_ERR_STREAK_RETUNE) {
			fd_need_retune = 1;
			fd_err_streak = 0;
		}
	}
}

void sys_can_getUniqueID(uint8_t id[16])
{
	const uint8_t *uidbase = (const uint8_t *)UID_BASE;
	memcpy(id, uidbase, 12);

	const uint32_t cpuid = SCB->CPUID;
	memcpy(&id[12], &cpuid, 4);
}

void FDCAN1_IT0_IRQHandler(void)
{
	if ((FDCAN1->IR & FDCAN_IR_RF0N) || (FDCAN1->IR & FDCAN_IR_RF0F)) {
		FDCAN1->IR = FDCAN_IR_RF0N | FDCAN_IR_RF0F;
		handleRxInterrupt(0);
	}

	if ((FDCAN1->IR & FDCAN_IR_RF1N) || (FDCAN1->IR & FDCAN_IR_RF1F)) {
		FDCAN1->IR = FDCAN_IR_RF1N | FDCAN_IR_RF1F;
		handleRxInterrupt(1);
	}

	pollErrorFlagsFromISR();
}

void FDCAN1_IT1_IRQHandler(void)
{
	if (FDCAN1->IR & FDCAN_IR_TC) {
		FDCAN1->IR = FDCAN_IR_TC;
		handleTxCompleteInterrupt();
	}

	if (FDCAN1->IR & FDCAN_IR_BO) {
		FDCAN1->IR = FDCAN_IR_BO;
		FDCAN1->CCCR &= ~FDCAN_CCCR_INIT;
	}

	pollErrorFlagsFromISR();
}

int16_t sys_can_transmit(const CanardCANFrame *txf)
{
	return can_send(txf) ? 1 : 0;
}

int16_t sys_can_receive(CanardCANFrame *rx_frame)
{
	(void)rx_frame;
	return -1;
}

void sys_can_disable_IRQ(void)
{
	NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
	NVIC_DisableIRQ(FDCAN1_IT1_IRQn);
}

void sys_can_enable_IRQ(void)
{
	NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
	NVIC_EnableIRQ(FDCAN1_IT1_IRQn);
}

static bool waitForBitState(volatile uint32_t *reg, uint32_t mask, bool target_state)
{
	while (true) {
		bool current_state = ((*reg) & mask) != 0;
		if (current_state == target_state) {
			return true;
		}
	}
	return false;
}

#	pragma GCC optimize("Os")

static void can_init(void)
{
	RCC->APB1ENR1 |= RCC_APB1ENR1_FDCANEN;

	for (volatile int i = 0; i < 10000; i++) {
		__NOP();
	}

	RCC->APB1RSTR1 |= RCC_APB1RSTR1_FDCANRST;
	RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_FDCANRST;

	for (volatile int i = 0; i < 100; i++) {
		__NOP();
	}

	FDCAN1->CCCR &= ~FDCAN_CCCR_CSR;

	if (!waitForBitState(&FDCAN1->CCCR, FDCAN_CCCR_CSA, false)) {
		return;
	}

	FDCAN1->CCCR |= FDCAN_CCCR_INIT;

	if (!waitForBitState(&FDCAN1->CCCR, FDCAN_CCCR_INIT, true)) {
		return;
	}

	FDCAN1->CCCR |= FDCAN_CCCR_CCE;

	/* 1 Mbps nominal, 80 MHz kernel clock, 10 tq, 90% sample point. */
	const uint8_t sjw = 1;
	const uint8_t bs1 = 8;
	const uint8_t bs2 = 1;
	const uint8_t prescaler = 8;

	FDCAN1->NBTP = (((sjw - 1) << FDCAN_NBTP_NSJW_Pos) | ((bs1 - 1) << FDCAN_NBTP_NTSEG1_Pos) | ((bs2 - 1) << FDCAN_NBTP_NTSEG2_Pos) |
			((prescaler - 1) << FDCAN_NBTP_NBRP_Pos));

	apply_fd_timings(fd_idx);

	FDCAN1->CCCR |= FDCAN_CCCR_FDOE | FDCAN_CCCR_BRSE;

	setupMessageRam();

	FDCAN1->IR = 0x3FFFFFFF;

	FDCAN1->IE = FDCAN_IE_RF0NE | FDCAN_IE_RF0FE | FDCAN_IE_RF1NE | FDCAN_IE_RF1FE | FDCAN_IE_TCE | FDCAN_IE_BOE;

	FDCAN1->ILS = FDCAN_ILS_PERR | FDCAN_ILS_SMSG;

	FDCAN1->TXBTIE = 0x7;

	FDCAN1->ILE = 0x3;

	FDCAN1->CCCR &= ~FDCAN_CCCR_INIT;

	waitForBitState(&FDCAN1->CCCR, FDCAN_CCCR_INIT, false);
}

static void fd_select_initial_rate(void)
{
	const uint8_t *eeprom = (const uint8_t *)EEPROM_START_ADD;
	uint8_t want = eeprom[EEPROM_FD_MBPS_OFF];
	if (eeprom[0] != 1) {
		want = 0;
	}
	const int8_t idx = fd_idx_for_mbps(want);
	if (want == 0 || idx < 0) {
		fd_auto = 1;
		fd_locked = 0;
		fd_idx = 0; /* 5 Mbps, product max */
	} else {
		fd_auto = 0;
		fd_locked = 1;
		fd_idx = (uint8_t)idx;
	}
	saw_classic = 0;
	fd_err_streak = 0;
	fd_need_retune = 0;
}

void sys_can_service(void)
{
	if (!fd_need_retune || !fd_auto || fd_locked || saw_classic) {
		fd_need_retune = 0;
		return;
	}
	fd_need_retune = 0;
	const uint8_t n = (uint8_t)(sizeof(fd_timings) / sizeof(fd_timings[0]));
	const uint8_t next = (uint8_t)((fd_idx + 1) % n);

	FDCAN1->CCCR |= FDCAN_CCCR_INIT;
	if (!waitForBitState(&FDCAN1->CCCR, FDCAN_CCCR_INIT, true)) {
		return;
	}
	FDCAN1->CCCR |= FDCAN_CCCR_CCE;
	apply_fd_timings(next);
	FDCAN1->CCCR |= FDCAN_CCCR_FDOE | FDCAN_CCCR_BRSE;
	FDCAN1->CCCR &= ~FDCAN_CCCR_INIT;
	waitForBitState(&FDCAN1->CCCR, FDCAN_CCCR_INIT, false);
}

void sys_can_init(void)
{
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_9;

	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);

	GPIO_InitStruct.Pin = CAN_RX_PIN;
	LL_GPIO_Init(CAN_RX_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = CAN_TX_PIN;
	LL_GPIO_Init(CAN_TX_PORT, &GPIO_InitStruct);

	fd_select_initial_rate();
	can_init();

	NVIC_SetPriority(FDCAN1_IT0_IRQn, 5);
	NVIC_SetPriority(FDCAN1_IT1_IRQn, 5);
	NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
	NVIC_EnableIRQ(FDCAN1_IT1_IRQn);
}

uint32_t get_rtc_backup_register(uint8_t idx)
{
	const volatile uint32_t *bkp = &TAMP->BKP0R;
	return bkp[idx];
}

void set_rtc_backup_register(uint8_t idx, uint32_t value)
{
	volatile uint32_t *bkp = &TAMP->BKP0R;
	bkp[idx] = value;
}

void setup_portpin(uint16_t portpin, bool enable)
{
	const uint8_t port = portpin >> 8;
	const uint8_t pin = portpin & 0xff;
	const uint32_t pinshift = 1U << pin;
	GPIO_TypeDef *const ports[] = {GPIOA, GPIOB, GPIOC};
	if (port >= sizeof(ports) / sizeof(ports[0])) {
		return;
	}
	GPIO_TypeDef *pport = ports[port];

	LL_AHB2_GRP1_EnableClock(1U << port);

	if (enable) {
		LL_GPIO_SetOutputPin(pport, pinshift);
	} else {
		LL_GPIO_ResetOutputPin(pport, pinshift);
	}

	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = pinshift;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(pport, &GPIO_InitStruct);
}

#endif /* DRONECAN_SUPPORT && defined(MCU_G431) */
