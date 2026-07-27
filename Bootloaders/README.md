# Bootloader images

Prebuilt ARK32 bootloader binaries, committed so the app can embed one and rewrite the bootloader region at boot (see `Src/bootloader_update.c`).

These are release artifacts of [ARK32-bootloader](https://github.com/ARK-Electronics/ARK32-bootloader) converted from the published Intel HEX to a flat binary and checked in. They are stored as `.bin` rather than a C array so the bytes stay comparable against a release asset (a hex dump in a header cannot).

| File | Target | Signal pin | Source |
|---|---|---|---|
| `AM32_F051_BOOTLOADER_PB4_V18.bin` | STM32F051, ARK 4IN1 / `HARDWARE_GROUP_F0_B` | PB4 | [ARK32-bootloader v18.0.0](https://github.com/ARK-Electronics/ARK32-bootloader/releases/tag/v18.0.0) (`0d667c5`), asset `AM32_F051_BOOTLOADER_PB4_V18.hex` |

`.gitignore` ignores `*.bin` repo-wide; this directory is un-ignored explicitly.

## How the image is used

`Src/bl_image.S` pulls the file in with `.incbin` and pads it to the bootloader region size with `0xFF`. The linker script places it in a dedicated `.bl_image` section and asserts that it is exactly the region size and that it does not live inside the region it will erase.

The image is only linked for F051 targets built without `HWCI_PERF=1`.

## Risk / recovery

App-side BL rewrite erases flash pages at `0x08000000` (reset vectors and the one-wire bootloader). Power loss after page 0 is erased and before vectors are rewritten leaves the chip without a valid reset vector — it will not reach the app or the previous bootloader.

- **In-field recovery:** SWD (or another pre-existing ROM/system boot path) is required to reflash the bootloader region. Keep a known-good `AM32_F051_BOOTLOADER_*.bin` (or the release `.hex`) and OpenOCD/probe procedure for production and bench recovery.
- **Software path:** On program/verify failure the update aborts without `NVIC_SystemReset`, re-enables IRQs, and continues into the already-running app so the board can still be reached if vectors survived. Sticky flash faults cannot hang forever with IRQs off (bounded per-page retries + IWDG armed for the update).
- **Power-loss mid-update** is still a brick until SWD reflash — that is inherent to rewriting the boot region in place.

## Updating

1. Download the matching `AM32_*_BOOTLOADER_*_V*.hex` from the [ARK32-bootloader releases](https://github.com/ARK-Electronics/ARK32-bootloader/releases) (ARK 4IN1 uses F051 PB4).
2. Convert to a flat binary with the pinned toolchain:

   ```sh
   arm-none-eabi-objcopy -I ihex -O binary \
     AM32_F051_BOOTLOADER_PB4_V18.hex \
     Bootloaders/AM32_F051_BOOTLOADER_PB4_V18.bin
   ```

3. Point `BL_IMAGE_F051` in the `Makefile` at the new path if the filename changed.
4. Update the table above with the release tag and source commit.
5. Rebuild — the `.bin` is an explicit prerequisite of the `.elf`, so the change is picked up.

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
