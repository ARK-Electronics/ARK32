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
| Bootloader | `0x08000000` | 16 KiB | `Bootloaders/AM32_G431_BOOTLOADER_ARKG4_CAN_V18.bin` (16 KiB region, file ~14 KiB + 0xFF pad) |
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
| Thermal derate | off (`141`) | **105 → 120 °C die** |
| Derate band | — (15 °C default) | **15 °C** (byte 184) |
| Current limit | off (`102`) | **200 A** (stored `100`, 2 A/count) |
| Current PID (P/I/D) | 100 / 0 / 50 | 100 / 0 / 50 |

The two protection limits are the one place the products differ on policy.
Both are stored as a single byte whose *armed* range is narrow — 70..140 °C
and 1..100 (2..200 A), per `Src/settings.c` — and the AM32 configurator
skeleton both JSONs were seeded from carries 141/102, which sit just outside
those ranges and therefore mean OFF. The G431 CAN board arms both:

- **Thermal**: a soft foldback, never a cutoff. The duty ceiling falls
  linearly from full authority at the onset (`temperature_limit`) to 10%
  one band later (`temperature_derate_band`, DroneCAN `TEMP_DERATE_BAND`,
  5..40 °C), off a 64 ms-filtered die temperature. 105 → 120 °C by default.
- **Current**: a 200 A backstop below the DRV8350 VDS trip, ~80% of the
  shunt rating. It regulates 50 ms-averaged current, so it protects against
  sustained draw, not against a short.
- The two compose by `min()` in `setInput()` and both only ever lower the
  ceiling; the 20 kHz ramp limiter slews applied duty toward it either way.
- **Do not lower `current_P`.** The ceiling moves by `pid_output / 10000`
  duty units per tick and that integer divide is a dead zone: the loop is
  inert until the overshoot exceeds `5000/Kp` centiamps. At the shipped
  Kp 200 that is ~0.5 A of resolution. Measured in SITL on `heavy_13inch`
  with an 8 A limit: P=100 holds 7.7 A, P=50 7.1 A, P=25 6.5 A, and P=5
  fails outright at 14.7 A.

The 105 °C onset follows professional 12S practice (APD and industrial
T-Motor fold back around 110 °C), with one caveat to settle on the bench:
**those vendors read an NTC on the power stage, this reads the MCU die.**
The G4 die sensor is factory-calibrated at 30 °C and 110 °C, so the default
band keeps almost the whole ramp inside the calibrated span and puts full
derate below the part's 125 °C junction limit — but if the die-to-FET delta
on this board turns out large, the onset belongs lower.

The 4IN1 stays off: its shunt is shared by all four ESCs, so per-motor
current limiting is not meaningful there, and its thermal placement has not
been benched.

Encodings follow `Inc/eeprom.h` and `Src/settings.c`. What each setting does, including the ones not listed above, is in [`doc/eeprom-settings.md`](../doc/eeprom-settings.md). Rebuild after editing the JSON:

```bash
make factory-image
```

## Field updates vs factory image

- **Production / blank chip:** flash the `.factory.bin` once (includes BL + app + EEPROM for both products).
- **In-field app update:** flash the normal app `.bin` / `.hex` at the app base (F051 `0x08001000`, G431 CAN `0x08004000`). EEPROM is left alone. F051 builds also embed the bootloader for optional app-side BL refresh (`EMBED_BOOTLOADER`, see [Bootloaders/README.md](../Bootloaders/README.md)). G431 currently uses the committed BL only for the factory full-flash image (no app-side BL rewrite path yet).

## Script

[`scripts/build_factory_image.py`](../scripts/build_factory_image.py) — pure Python 3. Layout comes from `flash_map` in the product JSON (F051 defaults if omitted).

## CI

Every PR/push runs **`factory-image`** in [`.github/workflows/static-analysis.yml`](../.github/workflows/static-analysis.yml):

```bash
make factory-image-check   # build + scripts/check-factory-image-ark.sh
```

The job fails if the flash layout is wrong or the EEPROM page drifts from the product defaults JSON (F051 or G431). Artifacts (`*.factory.bin` / `.hex` / `.eeprom.bin`) are uploaded as `ark-factory-images`.
