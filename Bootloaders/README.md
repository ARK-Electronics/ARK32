# Bootloader images

Prebuilt **[ARK32-bootloader](https://github.com/ARK-Electronics/ARK32-bootloader)** binaries, committed so:

1. **F051 (ARK 4IN1):** the app can embed one and rewrite the bootloader region at boot (see `Src/bootloader_update.c`).
2. **G431 CAN (ARK 12S CAN):** `make factory-image-g431-can` can assemble a full-flash production image (BL + app + EEPROM) without a separate BL flash step.

[ARK32-bootloader](https://github.com/ARK-Electronics/ARK32-bootloader) is ARK’s fork of [upstream AM32-bootloader](https://github.com/am32-firmware/AM32-bootloader). For ARK hardware, always take images from **ARK32-bootloader**, not from the upstream bootloader repo alone.

These files are release (or branch) artifacts as flat binaries. They are stored as `.bin` rather than a C array so the bytes stay comparable against a build or release asset (a hex dump in a header cannot).

| File | Target | Signal pin | Source |
|---|---|---|---|
| `AM32_F051_BOOTLOADER_PB4_V18.bin` | STM32F051, ARK 4IN1 / `HARDWARE_GROUP_F0_B` | PB4 | [ARK32-bootloader v18.0.0](https://github.com/ARK-Electronics/ARK32-bootloader/releases/tag/v18.0.0) (`0d667c5`), asset `AM32_F051_BOOTLOADER_PB4_V18.hex` |
| `AM32_G431_BOOTLOADER_ARKG4_CAN_V18.bin` | STM32G431/G491, ARK 12S CAN ESC (`ARK_G431_CAN`) | PB4 (also FDCAN TX PB9 / RX PA11) | [ARK32-bootloader](https://github.com/ARK-Electronics/ARK32-bootloader) dual-protocol + bl-params (`086755b`), target `AM32_G431_BOOTLOADER_ARKG4_CAN` |

| Resource | URL |
|---|---|
| Repository | https://github.com/ARK-Electronics/ARK32-bootloader |
| Releases | https://github.com/ARK-Electronics/ARK32-bootloader/releases |
| Upstream (non-ARK) | https://github.com/am32-firmware/AM32-bootloader |

`.gitignore` ignores `*.bin` repo-wide; this directory is un-ignored explicitly.

## How the images are used

### F051 app-side embed

`Src/bl_image.S` pulls the F051 file in with `.incbin` and pads it to the bootloader region size with `0xFF`. The linker script places it in a dedicated `.bl_image` section and asserts that it is exactly the region size and that it does not live inside the region it will erase.

The image is linked for **all F051 targets by default**, including `HWCI_PERF=1`
(LTO leaves enough flash for the image and the perf struct). To strip it for a
size emergency or pure perf A/B:

```bash
make ARK_4IN1_F051 EMBED_BOOTLOADER=0
make ARK_4IN1_F051 HWCI_PERF=1 NO_EMBED_BL=1
```

Either kill switch disables embed (`EMBED_BOOTLOADER=0` or `NO_EMBED_BL=1`).

### G431 CAN factory image

`BL_IMAGE_G431_CAN` in the `Makefile` points at `AM32_G431_BOOTLOADER_ARKG4_CAN_V18.bin`. That file is **not** app-side embedded yet (no G431 `.bl_image` path); it is only consumed by:

```bash
make factory-image-g431-can
# or
make factory-image-check
```

The factory builder places it at `0x08000000` (16 KiB region) and pads with `0xFF` to the app base at `0x08004000`. See [factory/README.md](../factory/README.md).

## Risk / recovery

App-side BL rewrite erases flash pages at `0x08000000` (reset vectors and the one-wire bootloader). Power loss after page 0 is erased and before vectors are rewritten leaves the chip without a valid reset vector — it will not reach the app or the previous bootloader.

- **In-field recovery:** SWD (or another pre-existing ROM/system boot path) is required to reflash the bootloader region. Keep a known-good `AM32_F051_BOOTLOADER_*.bin` (or the release `.hex`) and OpenOCD/probe procedure for production and bench recovery.
- **Software path:** On program/verify failure the update aborts without `NVIC_SystemReset`, re-enables IRQs, and continues into the already-running app so the board can still be reached if vectors survived. Sticky flash faults cannot hang forever with IRQs off (bounded per-page retries + IWDG armed for the update).
- **Success cue:** After a verified rewrite the app soft-resets; the next boot plays a short rising two-beep chirp (`playBootloaderUpdatedTone`) before the normal ARK/BlueJay startup tune so a field flash can be confirmed by ear.
- **Power-loss mid-update** is still a brick until SWD reflash — that is inherent to rewriting the boot region in place.

## Updating

### F051

1. Download the matching `AM32_F051_BOOTLOADER_*_V*.hex` from the [ARK32-bootloader releases](https://github.com/ARK-Electronics/ARK32-bootloader/releases).
2. Convert to a flat binary with the pinned toolchain:

   ```sh
   arm-none-eabi-objcopy -I ihex -O binary \
     AM32_F051_BOOTLOADER_PB4_V18.hex \
     Bootloaders/AM32_F051_BOOTLOADER_PB4_V18.bin
   ```

3. Point `BL_IMAGE_F051` in the `Makefile` at the new path if the filename changed.
4. Update the table above with the release tag and source commit.
5. Rebuild — the `.bin` is an explicit prerequisite of the F051 `.elf`, so the change is picked up.

### G431 CAN

1. Build from [ARK32-bootloader](https://github.com/ARK-Electronics/ARK32-bootloader):

   ```sh
   make AM32_G431_BOOTLOADER_ARKG4_CAN
   cp obj/AM32_G431_BOOTLOADER_ARKG4_CAN_V*.bin \
     /path/to/ARK32/Bootloaders/
   ```

2. Point `BL_IMAGE_G431_CAN` in the `Makefile` at the new path if the filename changed.
3. Update the table above with the source commit / release tag.
4. Rebuild factory image: `make factory-image-g431-can` (or `factory-image-check`).

Keep the old file until the new one has been flown; reverting is then a one-line `Makefile` change (or restoring the previous `.bin`).

## Verifying a build embedded the right image

The blob lands at the `.bl_image` symbols, so it can be pulled straight back out of the ELF and compared:

```sh
arm-none-eabi-objcopy -O binary --only-section=.bl_image \
  obj/AM32_ARK_4IN1_F051_*.elf /tmp/embedded.bin
cmp -n "$(wc -c < Bootloaders/AM32_F051_BOOTLOADER_PB4_V18.bin)" \
  /tmp/embedded.bin Bootloaders/AM32_F051_BOOTLOADER_PB4_V18.bin
```

The compare is limited to the source file's length because `.bl_image` is padded out to the full 4096-byte region with `0xFF`.
