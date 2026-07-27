# Bootloader images

Prebuilt AM32 bootloader binaries, committed so the app can embed one and rewrite the bootloader region at boot (see `Src/bootloader_update.c`).

These are the build artifacts of [AM32-bootloader](https://github.com/AlexKlimaj/AM32-bootloader) checked in verbatim. They are committed as `.bin` rather than converted to a C array so the bytes stay diffable against a release artifact — a hex dump in a header cannot be checked against anything.

| File | Target | Signal pin | Source |
|---|---|---|---|
| `AM32_F051_BOOTLOADER_PB4_V18.bin` | STM32F051, ARK 4IN1 / `HARDWARE_GROUP_F0_B` | PB4 | AM32-bootloader `4a33bf0` |

`.gitignore` ignores `*.bin` repo-wide; this directory is un-ignored explicitly.

## How the image is used

`Src/bl_image.S` pulls the file in with `.incbin` and pads it to the bootloader region size with `0xFF`. The linker script places it in a dedicated `.bl_image` section and asserts that it is exactly the region size and that it does not live inside the region it will erase.

The image is only linked for F051 targets built without `HWCI_PERF=1`.

## Updating

1. Build the new bootloader and drop the `.bin` in here, keeping the version in the filename.
2. Point `BL_IMAGE_F051` in the `Makefile` at it.
3. Add a row above with the source commit.
4. Rebuild — the `.bin` is an explicit prerequisite of the `.elf`, so the change is picked up.

Keep the old file until the new one has been flown; reverting is then a one-line `Makefile` change.

## Verifying a build embedded the right image

The blob lands at the `.bl_image` symbols, so it can be pulled straight back out of the ELF and compared:

```sh
arm-none-eabi-objcopy -O binary --only-section=.bl_image obj/AM32_ARK_4IN1_F051_*.elf /tmp/embedded.bin
cmp -n 4092 /tmp/embedded.bin Bootloaders/AM32_F051_BOOTLOADER_PB4_V18.bin
```

The compare is limited to the source file's length because `.bl_image` is padded out to the full 4096-byte region.
