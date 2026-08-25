# Bootloader images

This directory does **not** store binaries in git. Get them from the two ARK32 repos:

| What | Where |
|------|--------|
| Bootloader source and releases | [ARK-Electronics/ARK32-bootloader](https://github.com/ARK-Electronics/ARK32-bootloader) · [releases](https://github.com/ARK-Electronics/ARK32-bootloader/releases) |
| Firmware source and releases | [ARK-Electronics/ARK32](https://github.com/ARK-Electronics/ARK32) · [releases](https://github.com/ARK-Electronics/ARK32/releases) |

For ARK hardware use those repos — not [upstream AM32-bootloader](https://github.com/am32-firmware/AM32-bootloader) alone. The ARK 4IN1 bootloader holds DRV8328 `nSLEEP` (PA15) low and has bidirectional DShot idle detection.

`make ARK_4IN1_F051` (embed on) and `make factory-image` call [`scripts/fetch-ark32-bootloader.sh`](../scripts/fetch-ark32-bootloader.sh) if the local image is missing. That clones ARK32-bootloader at the pin in the Makefile (`BOOTLOADER_REF`) and builds the ARK 4IN1 product (`AM32_F051_BOOTLOADER_ARK4IN1`). The file lands here and is gitignored.

```bash
make bootloader-image
# -> Bootloaders/AM32_F051_BOOTLOADER_ARK4IN1_V18.bin
```

You can also copy a `.bin` you already built:

```bash
# in ARK32-bootloader
make AM32_F051_BOOTLOADER_ARK4IN1
cp obj/AM32_F051_BOOTLOADER_ARK4IN1_V18.bin \
  /path/to/ARK32/Bootloaders/
```

**Do not use the generic PB4 image on ARK 4IN1.** It does not hold `nSLEEP` low; first app boot would rewrite a real ARK4IN1 bootloader back to stock.

## How the image is used

`Src/bl_image.S` pulls the file in with `.incbin` and pads it to the bootloader region with `0xFF`. F051 builds embed it by default, including `HWCI_PERF=1`. To strip it:

```bash
make ARK_4IN1_F051 EMBED_BOOTLOADER=0
make ARK_4IN1_F051 HWCI_PERF=1 NO_EMBED_BL=1
```

At boot the app compares the on-chip bootloader to the embedded image and rewrites it if they differ (`Src/bootloader_update.c`). Success soft-resets; the next boot plays `playBootloaderUpdatedTone` then the normal startup tune.

Power loss after page 0 is erased and before vectors are rewritten bricks the chip until you reflash the bootloader over SWD. Keep a known-good image from [ARK32-bootloader](https://github.com/ARK-Electronics/ARK32-bootloader) for recovery.

## Verify an embed

```sh
arm-none-eabi-objcopy -O binary --only-section=.bl_image \
  obj/ARK32_ARK_4IN1_F051_*.elf /tmp/embedded.bin
cmp -n "$(wc -c < Bootloaders/AM32_F051_BOOTLOADER_ARK4IN1_V18.bin)" \
  /tmp/embedded.bin Bootloaders/AM32_F051_BOOTLOADER_ARK4IN1_V18.bin
```

The compare is limited to the source file length because `.bl_image` is padded to 4096 bytes with `0xFF`.
