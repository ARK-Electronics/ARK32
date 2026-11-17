#include "stm32g4xx_it.h"
#include "main.h"

/*
  A bootloader fault must not park the ESC. Before V20 every fault handler
  spun forever, and the RTC firmware-update request (TAMP BKP0) survives a
  reset, so a fault during a DroneCAN update (V18: misaligned canard pool)
  left the unit unreachable over CAN until someone attached SWD.

  Clear the request and reset instead. The next boot either finds a valid
  app (CRC checked in DroneCAN_boot_ok) or stays in the bootloader and
  reports no image CRC, so the flight controller starts a fresh update.

  The handlers are naked and use no stack: a fault caused by a corrupt stack
  pointer would fault again on the first push and lock up instead of
  resetting. TAMP BKP0 is at 0x40002500 (backup-domain write access is
  enabled by the clock setup; before that the store is ignored and the reset
  still happens). BKP3 counts these resets (see fw_update_abort): it bounds a
  deterministic fault that would otherwise reset forever.
 */
__attribute__((naked, noreturn, used)) void bl_fault_reset(void)
{
  __asm volatile(
    "ldr r0, =0x40002500 \n" /* TAMP->BKP0R: drop the firmware-update request */
    "movs r1, #0         \n"
    "str r1, [r0]        \n"
    "ldr r1, [r0, #12]   \n" /* TAMP->BKP3R: abort/fault reset count */
    "adds r1, r1, #1     \n"
    "str r1, [r0, #12]   \n"
    "dsb                 \n"
    "ldr r0, =0xE000ED0C \n" /* SCB->AIRCR */
    "ldr r1, =0x05FA0004 \n" /* VECTKEY | SYSRESETREQ */
    "str r1, [r0]        \n"
    "dsb                 \n"
    "1: b 1b             \n"
    ".ltorg              \n");
}

/*
  NMI on the G4 also signals a flash double-ECC error. A doubleword torn by a
  power cut during an update is exactly what DroneCAN_boot_ok() reads in its
  CRC pass (now primed before CAN starts), so resetting here would loop before
  the node ever reaches the bus. Clear the error and record it instead:
  boot_ok treats it as a CRC failure, the unit stays in the bootloader, and
  the next update erases the bad page. Any other NMI resets.
 */
volatile uint8_t bl_flash_ecc_error;

void NMI_Handler(void)
{
  if ((FLASH->ECCR & FLASH_ECCR_ECCD) != 0u) {
    FLASH->ECCR |= FLASH_ECCR_ECCD; /* write 1 to clear */
    bl_flash_ecc_error = 1;
    return;
  }
  bl_fault_reset();
}

__attribute__((naked)) void HardFault_Handler(void)
{
  __asm volatile("b bl_fault_reset");
}

__attribute__((naked)) void MemManage_Handler(void)
{
  __asm volatile("b bl_fault_reset");
}

__attribute__((naked)) void BusFault_Handler(void)
{
  __asm volatile("b bl_fault_reset");
}

__attribute__((naked)) void UsageFault_Handler(void)
{
  __asm volatile("b bl_fault_reset");
}

void SVC_Handler(void) { }

void DebugMon_Handler(void) { }

void PendSV_Handler(void) { }

void SysTick_Handler(void) { }
