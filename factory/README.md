# Factory / production flash images

Production programming of ARK ESCs used to be a multi-step bench flow:

1. Flash the bootloader (ST-Link)
2. Flash the application
3. Open AM32 Configurator and write EEPROM defaults
4. Hand-tune settings (kV, ramp, timing, PWM endpoints)
5. Dump the whole flash and ship that binary on the line

That dump is now built from this repo for **ARK 4IN1 (F051)** and **ARK G431 12S CAN**.

## One command

From the repository root (pinned Arm GCC required, same as any firmware build):

```bash
make factory-image          # both products
make factory-image-f051     # ARK_4IN1_F051 only
make factory-image-g431-can # ARK_G431_CAN only
make factory-image-check    # both + layout/defaults gate (CI)
```

Artifacts under `obj/`:

| File | Contents |
|------|----------|
| `ARK32_ARK_4IN1_F051_<ver>.bin` / `.hex` | Application only (existing make target) |
| `ARK32_ARK_4IN1_F051_<ver>.factory.bin` | **Full 32 KiB flash** — BL + app + EEPROM |
| `ARK32_ARK_G431_CAN_<ver>.factory.bin` | **Full 128 KiB flash** — BL region + app + EEPROM |
| `*.factory.hex` / `*.eeprom.bin` | HEX of full image / EEPROM page alone |

Flash the `.factory.bin` (or `.factory.hex`) at **`0x08000000`**. No configurator step is required for stock defaults.

Example OpenOCD (ST-Link + STM32F0 for 4IN1):

```bash
openocd -f interface/stlink.cfg -f target/stm32f0x.cfg \
  -c "init; halt; flash erase_sector 0 0 last; \
      flash write_bank 0 obj/ARK32_ARK_4IN1_F051_3.0.3.factory.bin 0; \
      flash verify_bank 0 obj/ARK32_ARK_4IN1_F051_3.0.3.factory.bin 0; \
      reset run; exit"
```

## Flash maps

### STM32F051 ARK 4IN1 (32 KiB)

| Region | Address | Size | Source |
|--------|---------|------|--------|
| Bootloader | `0x08000000` | 4 KiB | `Bootloaders/AM32_F051_BOOTLOADER_ARK4IN1_V18.bin` (0xFF padded; PB4 signal + PA15 nSLEEP low) |
| Application | `0x08001000` | 27 KiB | `make ARK_4IN1_F051` app `.bin` (0xFF padded) |
| EEPROM | `0x08007C00` | 1 KiB | Defaults from [`ARK_4IN1_F051_eeprom_defaults.json`](ARK_4IN1_F051_eeprom_defaults.json) |

### STM32G431 ARK 12S CAN (128 KiB)

| Region | Address | Size | Source |
|--------|---------|------|--------|
| Bootloader | `0x08000000` | 16 KiB | Optional `Bootloaders/AM32_G431*_CAN*.bin`; else **0xFF pad** (flash BL separately) |
| Application | `0x08004000` | ~111.5 KiB | `make ARK_G431_CAN` (`ldscript_CAN.ld`) |
| EEPROM | `0x0801F800` | 2 KiB | [`ARK_G431_CAN_eeprom_defaults.json`](ARK_G431_CAN_eeprom_defaults.json) |

## Production EEPROM defaults (shared ARK vehicle policy)

Both products ship the same ramp/timing/kV policy; input type differs:

| Setting | ARK 4IN1 F051 | ARK G431 CAN |
|---------|---------------|--------------|
| Variable PWM | PWM by RPM | PWM by RPM |
| Motor kV | 1020 | 1020 |
| Throttle ramp | **2 %/ms** | **2 %/ms** |
| Timing advance | **15° fixed** | **15° fixed** |
| PWM input min/max | 1020 / 1980 µs | 1020 / 1980 µs |
| Input type | DShot (`1`) | **DroneCAN** (`5`) |

Encodings follow `Inc/eeprom.h` and `Src/settings.c`. What each setting does, including the ones not listed above, is in [`doc/eeprom-settings.md`](../doc/eeprom-settings.md). Rebuild after editing the JSON:

```bash
make factory-image
```

## Field updates vs factory image

- **Production / blank chip:** flash the `.factory.bin` once (G431: commit a CAN bootloader under `Bootloaders/` first, or flash BL then the factory image / app+eeprom).
- **In-field app update:** flash the normal app `.bin` / `.hex` at the app base (F051 `0x08001000`, G431 CAN `0x08004000`). EEPROM is left alone. F051 builds also embed the bootloader for optional app-side BL refresh (`EMBED_BOOTLOADER`, see [Bootloaders/README.md](../Bootloaders/README.md)). There is **no** G431 app-side BL embed yet (no committed G431 BL image).

## Script

[`scripts/build_factory_image.py`](../scripts/build_factory_image.py) — pure Python 3. Layout comes from `flash_map` in the product JSON (F051 defaults if omitted).

## CI

Every PR/push runs **`factory-image`** in [`.github/workflows/static-analysis.yml`](../.github/workflows/static-analysis.yml):

```bash
make factory-image-check   # build + scripts/check-factory-image-ark.sh
```

The job fails if the 32 KiB layout is wrong or the EEPROM page drifts from `ARK_4IN1_F051_eeprom_defaults.json`. Artifacts (`*.factory.bin` / `.hex` / `.eeprom.bin`) are uploaded as `ark-4in1-factory-image`.
