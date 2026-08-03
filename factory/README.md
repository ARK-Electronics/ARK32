# Factory / production flash images

Production programming of **ARK 4IN1** ESCs used to be a multi-step bench flow:

1. Flash the bootloader (ST-Link)
2. Flash the application
3. Open AM32 Configurator and write EEPROM defaults
4. Hand-tune settings (kV, ramp, timing, PWM endpoints)
5. Dump the whole flash and ship that binary on the line

That dump is now built from this repo.

## One command

From the repository root (pinned Arm GCC required, same as any firmware build):

```bash
make factory-image
```

Artifacts under `obj/`:

| File | Contents |
|------|----------|
| `AM32_ARK_4IN1_F051_<ver>.bin` / `.hex` | Application only (existing make target) |
| `AM32_ARK_4IN1_F051_<ver>.factory.bin` | **Full 32 KiB flash** — BL + app + EEPROM |
| `AM32_ARK_4IN1_F051_<ver>.factory.hex` | Same image as Intel HEX |
| `AM32_ARK_4IN1_F051_<ver>.eeprom.bin` | 1 KiB EEPROM page alone (debug / SWD poke) |

Flash the `.factory.bin` (or `.factory.hex`) at **`0x08000000`** with ST-Link / OpenOCD / production fixture. No configurator step is required for stock defaults.

Example OpenOCD (ST-Link + STM32F0):

```bash
openocd -f interface/stlink.cfg -f target/stm32f0x.cfg \
  -c "init; halt; flash erase_sector 0 0 last; \
      flash write_bank 0 obj/AM32_ARK_4IN1_F051_3.0-ark.factory.bin 0; \
      flash verify_bank 0 obj/AM32_ARK_4IN1_F051_3.0-ark.factory.bin 0; \
      reset run; exit"
```

## Flash map (STM32F051, 32 KiB)

| Region | Address | Size | Source |
|--------|---------|------|--------|
| Bootloader | `0x08000000` | 4 KiB | `Bootloaders/AM32_F051_BOOTLOADER_ARK4IN1_V18.bin` (0xFF padded; PB4 signal + PA15 nSLEEP low) |
| Application | `0x08001000` | 27 KiB | `make ARK_4IN1_F051` app `.bin` (0xFF padded) |
| EEPROM | `0x08007C00` | 1 KiB | Defaults from [`ARK_4IN1_F051_eeprom_defaults.json`](ARK_4IN1_F051_eeprom_defaults.json) |

## Production EEPROM defaults

Defined in [`ARK_4IN1_F051_eeprom_defaults.json`](ARK_4IN1_F051_eeprom_defaults.json) (human units). Current ARK 4IN1 ship settings:

| Setting | Value |
|---------|-------|
| Variable PWM | **PWM by RPM** (`variable_pwm = 1`) |
| Motor kV | **1020** |
| Motor poles | 14 |
| Throttle ramp | **2 %/ms** (`max_ramp = 20`) |
| Timing advance | **15° fixed** (`advance_level = 26`, `auto_advance = 0`) |
| PWM input min | **1020 µs** |
| PWM input max | **1980 µs** |
| Input type | DShot (servo endpoints still stored for PWM mode) |

Encodings follow `Inc/eeprom.h` and `Src/settings.c`. Rebuild after editing the JSON:

```bash
make factory-image
```

## Field updates vs factory image

- **Production / blank chip:** flash the `.factory.bin` once.
- **In-field app update:** flash the normal app `.bin` / `.hex` at `0x08001000` (or use the configurator / BL one-wire path). EEPROM settings on the chip are left alone. F051 builds also embed the bootloader for optional app-side BL refresh (`EMBED_BOOTLOADER`, see [Bootloaders/README.md](../Bootloaders/README.md)).

## Script

[`scripts/build_factory_image.py`](../scripts/build_factory_image.py) — pure Python 3, no third-party deps. Invoked by `make factory-image`.

## CI

Every PR/push runs **`factory-image`** in [`.github/workflows/static-analysis.yml`](../.github/workflows/static-analysis.yml):

```bash
make factory-image-check   # build + scripts/check-factory-image-ark.sh
```

The job fails if the 32 KiB layout is wrong or the EEPROM page drifts from `ARK_4IN1_F051_eeprom_defaults.json`. Artifacts (`*.factory.bin` / `.hex` / `.eeprom.bin`) are uploaded as `ark-4in1-factory-image`.
