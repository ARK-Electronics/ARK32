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
| `ARK32_ARK_4IN1_F051_<ver>.factory.img` | **Full 32 KiB flash** — BL + app + EEPROM |
| `ARK32_ARK_G431_CAN_<ver>.factory.img` | **Full 128 KiB flash** — BL region + app + EEPROM |
| `*.factory.hex` / `*.eeprom.bin` | HEX of full image / EEPROM page alone |

Flash the `.factory.img` (or `.factory.hex`) at **`0x08000000`**. No configurator step is required for stock defaults. The extension is **`.img` on purpose**: PX4's SD-card updater globs every `*.bin` on the card root and scans the **whole file** for an APDescriptor, so a G431 factory image named `.bin` would be copied to `/ufw/71.bin` and written at the app base (off the end of flash).

Example OpenOCD (ST-Link + STM32F0 for 4IN1):

```bash
openocd -f interface/stlink.cfg -f target/stm32f0x.cfg \
  -c "init; halt; flash erase_sector 0 0 last; \
      flash write_bank 0 obj/ARK32_ARK_4IN1_F051_3.0.3.factory.img 0; \
      flash verify_bank 0 obj/ARK32_ARK_4IN1_F051_3.0.3.factory.img 0; \
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

Both products share the timing/kV policy; the throttle ramp and the
protection limits differ by product:

| Setting | ARK 4IN1 F051 | ARK G431 CAN |
|---------|---------------|--------------|
| Variable PWM | PWM by RPM | PWM by RPM |
| Motor kV | 1020 | 1020 |
| Throttle ramp | **2 %/ms** | **0.5 %/ms** (200 ms full scale) |
| Timing advance | **15° fixed** | **15° fixed** |
| PWM input min/max | 1020 / 1980 µs | 1020 / 1980 µs |
| Input type | DShot (`1`) | **DroneCAN** (`5`) |
| Thermal derate | off (`141`) | **105 → 120 °C die** |
| Derate band | — (15 °C default) | **15 °C** (byte 184) |
| Current limit | off (`102`) | **200 A** (stored `100`, 2 A/count) |
| Current PID (P/I/D) | 100 / 0 / 50 | 100 / 0 / 50 |

The 12S CAN board ramps at **0.5 %/ms** (full scale in 200 ms), matching what
larger 12S ESCs ship — APD and Hargrave default to 50 % per 100 ms — rather
than the 4IN1's 2.0 %/ms. Two consequences worth knowing:

- Below 10 the stored byte selects the firmware's *fine* mode
  (`ramp_divider 9`, a step every 500 µs) and applies the rate to **all three
  regimes including startup**, instead of min-ing against the `RAMP_SPEED_*`
  ceilings in `targets.h`. So spool-up slews at 0.5 %/ms too; that is the part
  to watch on the bench.
- It is a rate limit, not a bandwidth limit: only commands large enough to hit
  the limit are slewed, so ordinary attitude corrections are untouched while a
  full-stick step takes 200 ms.

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

- **Production / blank chip:** flash the `.factory.img` (or `.factory.hex`) once (includes BL + app + EEPROM for both products). Never put this file on a PX4 SD card.
- **In-field app update:** flash the normal app `.bin` / `.hex` at the app base (F051 `0x08001000`, G431 CAN `0x08004000`). EEPROM is left alone. Both products embed the bootloader and rewrite the on-chip BL if it differs (`EMBED_BOOTLOADER`, see [Bootloaders/README.md](../Bootloaders/README.md)). G431 CAN always embeds (including HWCI), so a manual or PX4 app flash still lands the 0.71 BL.
- **PX4 SD-card update (ARK G431 CAN):** `make ARK_G431_CAN` (or `make factory-image-g431-can`) also writes a PX4-signed copy named `<board_id>-<MAJOR.MINOR.PATCH>.<githash>.uavcan.bin` under `obj/` — e.g. `71-3.0.2.59efc137.uavcan.bin`. Copy **only** that file to the **root of the flight-controller SD card** and reboot. PX4 reads the APDescriptor (it scans the whole `*.bin`, not just the first 1 KiB), stages `/ufw/71.bin`, and flashes every ESC whose GetNodeInfo hardware version is 0.71. See [README.md](../README.md#bootloader).

## Script

[`scripts/build_factory_image.py`](../scripts/build_factory_image.py) — pure Python 3. Layout comes from `flash_map` in the product JSON (F051 defaults if omitted).

## CI

Every PR/push runs **`factory-image`** in [`.github/workflows/static-analysis.yml`](../.github/workflows/static-analysis.yml):

```bash
make factory-image-check   # build + scripts/check-factory-image-ark.sh
```

The job fails if the flash layout is wrong, the EEPROM page drifts from the product defaults JSON (F051 or G431), the G431 PX4 SD-card `.uavcan.bin` is missing / unsigned, or the app `.hex` does not cover every byte of the app `.bin`. Artifacts (`*.factory.img` / `.factory.hex` / `.eeprom.bin` / `*.uavcan.bin`) are uploaded as `ark-factory-images`.
