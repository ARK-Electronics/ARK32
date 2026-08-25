/*
 * Per-board bootloader configuration for the in-tree G431 CAN bootloader.
 *
 * Only the ARK 12S CAN ESC is built from this tree. Force-included into every
 * translation unit via -include, with -DARK_G431_CAN from the Makefile.
 */

#define GPIO_PORT_PIN(portnum, pinnum) ((portnum) << 8 | (pinnum))

#ifdef ARK_G431_CAN
#	define FILE_NAME "ARK_G431_CAN"
#	define TARGET_TAG ARKG4
#	define USE_PB4
/* Must match ARK32 Inc/targets.h. PX4 board_id = (major << 8) | minor = 71. */
#	define DRONECAN_HW_VERSION_MAJOR 0
#	define DRONECAN_HW_VERSION_MINOR 71

/* FDCAN1: RX PA11, TX PB9 (AF9) */
#	define CAN_RX_PORT GPIOA
#	define CAN_RX_PIN LL_GPIO_PIN_11
#	define CAN_TX_PORT GPIOB
#	define CAN_TX_PIN LL_GPIO_PIN_9

#	define USE_RGB_LED
#	define RED_PORT GPIOC
#	define RED_PIN LL_GPIO_PIN_6
#	define GREEN_PORT GPIOC
#	define GREEN_PIN LL_GPIO_PIN_7
#	define BLUE_PORT GPIOC
#	define BLUE_PIN LL_GPIO_PIN_8

/* DRV8350H ENABLE (PC9): hold low so the gate driver stays in sleep in BL */
#	define GATE_DRIVER_OFF_PORT GPIOC
#	define GATE_DRIVER_OFF_PIN LL_GPIO_PIN_9

#	define CAN_TERM_PIN GPIO_PORT_PIN(2, 12) /* PC12 */
#	define CAN_TERM_POLARITY 1

#	define USE_HSE
#	undef HSE_VALUE
#	define HSE_VALUE 8000000
#	define USE_HSE_BYPASS 0

/* G491 has 112KB RAM; allow app stack pointer above the 64KB default */
#	define RAM_LIMIT_KB 112
#endif
