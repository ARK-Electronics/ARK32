# Bootloader images

Prebuilt **[ARK32-bootloader](https://github.com/ARK-Electronics/ARK32-bootloader)** binaries, committed so:

1. **F051 (ARK 4IN1):** the app embeds one and rewrites the bootloader region at boot (see `Src/bootloader_update.c`).
2. **G431 CAN (ARK 12S CAN):** the app always embeds one (including `HWCI_PERF=1`) and rewrites the on-chip BL if it differs — so a manual or PX4 app-only flash still lands UAVCAN hw 0.71. `make factory-image-g431-can` also places the same image at `0x08000000` for blank-chip programming.

[ARK32-bootloader](https://github.com/ARK-Electronics/ARK32-bootloader) is ARK’s fork of [upstream AM32-bootloader](https://github.com/am32-firmware/AM32-bootloader). For ARK hardware, always take images from **ARK32-bootloader**, not from the upstream bootloader repo alone.

These files are release (or branch) artifacts as flat binaries. They are stored as `.bin` rather than a C array so the bytes stay comparable against a build or release asset (a hex dump in a header cannot).

| File | Target | Signal pin | Source |
|---|---|---|---|
| `AM32_F051_BOOTLOADER_ARK4IN1_V18.bin` | STM32F051, ARK 4IN1 (default embed) | PB4; **PA15 nSLEEP low** | [ARK32-bootloader](https://github.com/ARK-Electronics/ARK32-bootloader) `master` (`cdce0a2`, #4); product `AM32_F051_BOOTLOADER_ARK4IN1` |
| `AM32_F051_BOOTLOADER_PB4_V18.bin` | STM32F051 generic PB4 (reference only) | PB4 | [ARK32-bootloader v18.0.0](https://github.com/ARK-Electronics/ARK32-bootloader/releases/tag/v18.0.0) (`0d667c5`), asset `AM32_F051_BOOTLOADER_PB4_V18.hex` |
| `AM32_G431_BOOTLOADER_ARKG4_CAN_V18.bin` | STM32G431/G491, ARK 12S CAN ESC (`ARK_G431_CAN`) | PB4 (also FDCAN TX PB9 / RX PA11); **DRV ENABLE (PC9) held low** | [ARK32-bootloader](https://github.com/ARK-Electronics/ARK32-bootloader) `feat/ark-g431-can-dual-protocol` (`060c417`: UAVCAN hw **0.71** / board_id 71 so PX4 SD-card recovery matches the app), target `AM32_G431_BOOTLOADER_ARKG4_CAN` |

| Resource | URL |
|---|---|
| Repository | https://github.com/ARK-Electronics/ARK32-bootloader |
| Releases | https://github.com/ARK-Electronics/ARK32-bootloader/releases |
| Upstream (non-ARK) | https://github.com/am32-firmware/AM32-bootloader |

`.gitignore` ignores `*.bin` repo-wide; this directory is un-ignored explicitly.

## How the images are used

### F051 app-side embed

`Src/bl_image.S` pulls the file in with `.incbin` and pads it to the bootloader region size with `0xFF`. The linker script places it in a dedicated `.bl_image` section and asserts that it is exactly the region size and that it does not live inside the region it will erase.

**F051** embeds on release builds by default. `HWCI_PERF=1` does **not** embed it (flash goes to the perf struct; the HWCI rig already has a bootloader on-chip).

**G431 CAN** always embeds (16 KiB region, including `HWCI_PERF=1`). First app boot rewrites `0x08000000..0x08003FFF` if the on-chip BL differs.

To strip the embed:

```bash
make ARK_4IN1_F051 EMBED_BOOTLOADER=0
make ARK_G431_CAN  NO_EMBED_BL=1
```

Either kill switch disables embed (`EMBED_BOOTLOADER=0` or `NO_EMBED_BL=1`).
`HWCI_PERF=1` also disables embed automatically on F051 only.

### G431 CAN factory image

`BL_IMAGE_G431_CAN` in the `Makefile` points at `AM32_G431_BOOTLOADER_ARKG4_CAN_V18.bin`. The same file is app-side embedded (above) and consumed by:

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

1. Build or download the ARK 4IN1 product from [ARK32-bootloader](https://github.com/ARK-Electronics/ARK32-bootloader) (`make AM32_F051_BOOTLOADER_ARK4IN1`), or convert a release `.hex` with the pinned toolchain.
2. Install the flat binary:

   ```sh
   cp obj/AM32_F051_BOOTLOADER_ARK4IN1_V18.bin \
     Bootloaders/AM32_F051_BOOTLOADER_ARK4IN1_V18.bin
   # or from a release hex:
   # arm-none-eabi-objcopy -I ihex -O binary \
   #   AM32_F051_BOOTLOADER_ARK4IN1_V18.hex \
   #   Bootloaders/AM32_F051_BOOTLOADER_ARK4IN1_V18.bin
   ```

3. Point `BL_IMAGE_F051` in the `Makefile` at the new path if the filename changed (default is already the ARK4IN1 blob).
4. Update the table above with the release tag / source commit.
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
4. Rebuild the app (and factory image): `make ARK_G431_CAN` / `make factory-image-g431-can`. The `.bin` is an explicit prerequisite of the G431 CAN `.elf`.

**Do not embed the generic `…_PB4` image on ARK 4IN1** if the chip has ARK4IN1 BL: first boot would rewrite PA15 nSLEEP-off back to stock.

Keep the previous file until the new one has been flown; reverting is a one-line `Makefile` change (or restoring the previous `.bin`).

## Verifying a build embedded the right image

The blob lands at the `.bl_image` symbols, so it can be pulled straight back out of the ELF and compared:

```sh
arm-none-eabi-objcopy -O binary --only-section=.bl_image \
  obj/ARK32_ARK_4IN1_F051_*.elf /tmp/embedded.bin
cmp -n "$(wc -c < Bootloaders/AM32_F051_BOOTLOADER_ARK4IN1_V18.bin)" \
  /tmp/embedded.bin Bootloaders/AM32_F051_BOOTLOADER_ARK4IN1_V18.bin
```

The compare is limited to the source file's length because `.bl_image` is padded out to the full region (F051 4096, G431 CAN 16384) with `0xFF`. For G431:

```sh
arm-none-eabi-objcopy -O binary --only-section=.bl_image \
  obj/AM32_ARK_G431_CAN_*.elf /tmp/embedded.bin
cmp -n "$(wc -c < Bootloaders/AM32_G431_BOOTLOADER_ARKG4_CAN_V18.bin)" \
  /tmp/embedded.bin Bootloaders/AM32_G431_BOOTLOADER_ARKG4_CAN_V18.bin
```
