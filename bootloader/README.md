# ARK 12S CAN bootloader

In-tree STM32G431 DroneCAN bootloader for the **ARK 12S CAN ESC** (`ARK_G431_CAN`).

This is the G431 CAN slice of [ARK32-bootloader](https://github.com/ARK-Electronics/ARK32-bootloader) `feat/ark-g431-can-dual-protocol` (`060c417`, UAVCAN hw **0.71** / board_id 71), plus CAN FD. HAL/CMSIS is the app tree at `Mcu/g431/Drivers`. Other AM32 bootloader targets (F051, L431, …) stay in ARK32-bootloader.

## Build

From the repo root (uses the same pinned Arm GCC as the app):

```bash
make bootloader-g431-can
```

or `make -C bootloader`. Output:

- `bootloader/obj/AM32_G431_BOOTLOADER_ARKG4_CAN_V18.{elf,bin,hex}`
- copied to `Bootloaders/AM32_G431_BOOTLOADER_ARKG4_CAN_V18.bin` for app embed and factory images

`make ARK_G431_CAN` rebuilds this image when bootloader sources change.

## Hardware

| Item | Value |
|---|---|
| Signal pin | PB4 |
| FDCAN | TX PB9 / RX PA11, AF9 |
| Gate driver | DRV8350H ENABLE PC9 held low |
| CAN terminator | PC12, EEPROM byte 183 |
| Region | 16 KiB at `0x08000000`, app at `0x08004000` |

## CAN FD

The FDCAN peripheral runs in FD mode (FDOE + BRSE). Driver source is the app file `Src/DroneCAN/sys_can_stm32_CANFD.c` (no bootloader copy). Nominal (arbitration) bit timing is 1 Mbps. Data-phase bit timing uses the 80 MHz Kvaser/ArduPilot table (1/2/4/5 Mbps, 70–75% sample point, TDC). EEPROM byte 185 (`CAN_FD_MBPS`) is 0 = auto-match the host data rate, or 1/2/4/5 to pin it. Classic DNA does not lock the data rate. TX FDF/BRS latches on the first received CAN FD frame.
