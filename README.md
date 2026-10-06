# ARK32

Firmware for ARM-based brushless ESC (electronic speed controllers).

**ARK32** is a **fork of [upstream AM32](https://github.com/am32-firmware/AM32)** maintained by **[ARK Electronics](https://arkelectron.com/)**. It tracks upstream capability while carrying ARK’s product line, control-path work, and test infrastructure. (This repository was previously published as `ARK-Electronics/AM32`; that URL redirects here.)

| | Upstream AM32 | ARK32 |
|--|----------|-----------|
| Remote | [am32-firmware/AM32](https://github.com/am32-firmware/AM32) | [ARK-Electronics/ARK32](https://github.com/ARK-Electronics/ARK32) |
| Product branch | `main` | **`main`** |
| Focus | Multi-vendor ESC firmware | ARK targets + maintainability + CI |

For stock AM32 releases, configurators, Discord, and community support, prefer **[am32.ca](https://am32.ca)** and the [upstream project](https://github.com/am32-firmware/AM32).

---

## What ARK32 adds

### Commutation: how the motor is timed

The ESC times the motor two ways:

1. **Poll** (open loop). The 20 kHz loop reads BEMF and steps when it sees a crossing. This is slower. It is how the motor starts, and how it recovers if timing is lost.
2. **Interrupt** (closed loop). A comparator interrupt fires on each zero-cross. This is faster. It is used once the motor is spinning.

Upstream AM32 returns to poll on any commutation where the average step time is longer than `polling_mode_changeover + 500`. A missed crossing makes one step look long, so near the changeover speed a miss can hand control back to poll.

ARK32 still uses poll to start and to recover. It does **not** drop to poll just because one crossing was missed. This was motivated by a SITL motor model that bounced between the two modes. On the bench it measured performance-neutral against the previous ARK firmware ([#41](https://github.com/ARK-Electronics/ARK32/pull/41)). Treat it as a robustness change, not a measured speed-up.

ARK32 raises the changeover on the ARK 4IN1 (`POLLING_MODE_THRESHOLD` 5000, upstream default 2000), so interrupt mode runs down to lower rpm. On a 900 KV motor with no prop, 5% throttle ran closed loop instead of 100% poll ([#39](https://github.com/ARK-Electronics/ARK32/pull/39)).

**Startup.** The motor starts in poll. When the step time is short enough (`polling_mode_changeover`), it switches to interrupts. Until 100 crossings have been counted since the start (poll and interrupt together), a still-slow average puts it back in poll. That stops a noisy first kick from jumping into interrupt mode with no real BEMF. With stall protection or RC-car reverse enabled, it switches after 20 crossings with a step below 2000 instead.

**Missed crossings (interrupt mode).** Once 100 crossings have been counted, after each step the ESC sets a deadline for the next crossing: expected time plus 50%. If the crossing arrives, the deadline is cancelled. If the deadline fires first, the ESC steps anyway (a **blind step**) and uses that late time as the interval. Timing moves slower — the safe direction if the rotor is slowing. The next real crossing locks on again. Blind steps do not count as zero-crosses.

**How long it will run blind.** Time spent without a real crossing is added up (`zc_blind_ticks`). A real crossing clears it. If that time reaches the same stall budget as upstream (`BEMF_STALL_TICKS`, 22.5 ms), the ESC no longer knows rotor position: it stops and restarts through poll. As the budget is used, duty is scaled down in proportion to what is left. The applied duty drops at once and climbs back at the configured ramp after a real crossing. Crossings that arrive demag-late (no pre-crossing level seen) get the same fade, floored at half.

How high a sustained miss rate the motor can fly through has not been measured on hardware. SITL checks a 50% alternating-miss case only as an expected failure (`xfail`).

**When it does go back to poll after startup**

- **Desync** (the per-revolution average step time changes by more than half). Return to poll and re-acquire. The check only runs while the average is below 2000 ticks, upstream's value. Between 2000 and the slow-loop handback below, a bad lock is caught only by that handback or by the blind budget.
- **Sustained slow closed loop.** If interrupt mode stays slower than `polling_mode_changeover + 500` for 4 electrical revs (2 at high duty), hand back to poll with no desync count. A throttle chop that slows through this band then behaves like upstream. A false lock on a stopped rotor finds no real crossings in poll, and the stall restart takes over.

A missed crossing costs one extra step, not a mode change. Poll is the slow-speed and recovery path, not the reaction to every miss.

Source: [`Src/bemf_zc.c`](Src/bemf_zc.c) (deadline, blind step, stall handoff, demag-late), [`Src/commutation.c`](Src/commutation.c) (poll ↔ interrupt), [`Src/runtime_loop.c`](Src/runtime_loop.c) (desync and slow-average handback), [`Src/control_loop.c`](Src/control_loop.c) (duty fade while blind), [`Src/faults.c`](Src/faults.c) (stall restart), [`Inc/targets.h`](Inc/targets.h) (`POLLING_MODE_THRESHOLD`).

### Throttle and protection

**Ramp.** The ramp setting can only lower the built-in ceilings of 2, 6 and 16 %/ms (startup, low rpm, high rpm). The factory 2 %/ms therefore runs at 2 %/ms in every regime. The setting is referenced to a 4S pack at 14.8 V. The ESC scales the duty slew by 14.8 V / pack voltage, clamped to 0.5×–1.5×, and carries the fractional part from step to step. The same setting then gives about the same volts per millisecond on any pack. Upstream AM32 applies the setting as duty per step with no scaling. For the same setting ARK32 slews 0.59–0.67× as fast on 6S (25.2 V full to 22.2 V nominal) and about 1.33× as fast on a nominal 3S.

Nothing lowers the ramp after a desync. A learned back-off that did so was removed because it made one motor lag the others.

**Speed-based duty ceiling.** After about 0.3 s of steady closed-loop running above 12.5% duty, the ESC learns how much rpm each unit of duty buys. It then caps duty at what the measured rpm needs plus about 3 V of headroom, which bounds slip current while a heavy prop spools. The ceiling starts from scratch whenever the motor coasts, drops to poll or desyncs. Until it has re-learned, a snap is not limited, including the first snap after every start. If the cap holds rpm flat for 1 s, it releases gradually.

**Repeat desyncs or stalls** charge a bucket. Only runs that got past 100 crossings count. The first one restarts immediately. Back to back, later ones go:

| | 2nd | 3rd | 4th | 5th |
|--|--|--|--|--|
| Desync | wait 100 ms | wait 300 ms | wait 500 ms | latch stuck |
| Stall restart | wait 300 ms | wait 500 ms | latch stuck | |

Healthy closed-loop running drains the bucket. Zero throttle clears the bucket and the stuck latch.

Source: [`Src/control_loop.c`](Src/control_loop.c) (slew limit), [`Src/settings.c`](Src/settings.c) (ramp ceilings), [`Src/runtime_loop.c`](Src/runtime_loop.c) (voltage scaling, duty ceiling), [`Src/faults.c`](Src/faults.c) (episode bucket).

### Performance vs upstream: what is measured

One hardware A/B against upstream AM32 has been run: 2026-07-02, ARK 4IN1, 1800 KV 2306 with an HQProp 5136 on 6S, one sweep per firmware. It predates most of the commutation and protection work above, and the switch to `-Os` with LTO. No hardware run exists for current `main`. The A/B protocol for re-measuring is [hwci/docs/UPSTREAM_AB.md](hwci/docs/UPSTREAM_AB.md).

| Area | Compared against | Result |
|--|--|--|
| CPU load | Upstream (07-02) | 0.5–4.5 points lower at every step from 10% to 100%. Worst steady main loop 151 µs vs 185 µs. |
| Efficiency | Upstream (07-02); earlier ARK firmware ([#19](https://github.com/ARK-Electronics/ARK32/pull/19)) | No difference the bench can resolve (±4% noise). An earlier +12.4% figure was a thrust-stand artifact and was withdrawn. |
| Commutation jitter | Upstream figures from 07-02 ([#30](https://github.com/ARK-Electronics/ARK32/pull/30)) | 30% throttle: 4.7% vs 9.1–9.9%. 60%: 18% vs 17–19%. 100%: 3.1% vs 2.1%. Different sessions. |
| Low-rpm closed loop | Upstream changeover value on ARK ([#39](https://github.com/ARK-Electronics/ARK32/pull/39)) | 900 KV, no prop: closed loop from ~4% throttle instead of poll. |
| Blind-step commutation | Earlier ARK firmware ([#41](https://github.com/ARK-Electronics/ARK32/pull/41)) | Performance-neutral. No blind steps in 82 healthy holds. |
| Demag-late detection | Earlier ARK firmware ([#45](https://github.com/ARK-Electronics/ARK32/pull/45)) | 900 KV 10" on 6S, hot 20% snap starts: 6/6 mistimed at 12–15 A became 6/6 clean at 2 A. Not run on upstream. |

The upstream numbers are in `hwci/baselines/ARK_4IN1_F051_HQ5136_upstream_main.json` on the `main_instrumented` branch.

### Gate driver sleep

On the ARK 4IN1 the DRV8328 is put to sleep (`nSLEEP`) when the ESC is not driving, braking, or beeping. See [`Src/gate_driver.c`](Src/gate_driver.c).

### Global refactor
Large control-path split out of a monolithic `main.c` into focused modules (runtime, settings, motor control helpers, and related MCU/F051 work). The goal is safer changes, clearer ownership of hot paths, and room for instrumentation without growing one file forever.

On the F051 the commutation interrupts and the 20 kHz loop run from RAM (`RAM_FUNC`, built `-O3`). Divisions in the 20 kHz loop were replaced with Q16 multiplies. The rest of the firmware builds with global `-Os` and LTO to fit flash ([`Makefile`](Makefile)); upstream builds with global `-O3`. Global `-O3` was tried here and broke the full-throttle hold on the ARK F051. See [Performance vs upstream](#performance-vs-upstream-what-is-measured) for what these changes measured.

### SITL (software-in-the-loop)
Native Linux build of the firmware against a simulated motor / bridge / battery, with DroneCAN over multicast UDP. Useful for protocol, startup, and logic tests without ESC hardware.

- Overview: [Mcu/SITL/README.md](Mcu/SITL/README.md)
- CI: `.github/workflows/SITL.yml`

```bash
make AM32_SITL_CAN    # host gcc; no Arm toolchain needed
obj/ARK32_AM32_SITL_CAN_*.elf --node-id 10 --verbose
```

### HITL / Hardware-CI
In-the-loop bench automation for the **ARK 4IN1**: build/flash, drive the motor (Flight Stand and/or PX4 BDShot setups), read on-device performance counters over SWD, and produce metrics / pass-fail reports.

- Harness: [hwci/README.md](hwci/README.md)
- Bench setups: [hwci/docs/BENCH_SETUPS.md](hwci/docs/BENCH_SETUPS.md)
- CI: `.github/workflows/hwci.yml`

### Bootloader fixes
Field bootloaders and app-side BL update use **[ARK32-bootloader](https://github.com/ARK-Electronics/ARK32-bootloader)** (see [Bootloader](#bootloader) below and [Bootloaders/README.md](Bootloaders/README.md)).

---

## Branch model

| Branch | Role |
|--------|------|
| **`main`** | ARK integration line — open product PRs here |
| `upstream_main` | Tracks upstream AM32 `main` |
| Feature branches | Short-lived; rebase onto `main` unless targeting pure upstream work |

---

## Build (make + GCC)

IDE project trees (Keil / MRS) are **not** maintained in ARK32. Use the Makefile and the pinned **xPack GNU Arm Embedded GCC** (see `make/tools.mk`).

```bash
# Install the pinned xPack GNU Arm Embedded GCC 15 into tools/<os>/
# (required — distro gcc-arm-none-eabi is not used for firmware builds)
make arm_sdk_install
make arm_sdk_check          # optional: confirm GCC 15.x at the pin path

# List / build targets (examples)
make targets
make -j$(nproc) ARK_4IN1_F051

# Production full-flash image (bootloader + app + factory EEPROM defaults)
make factory-image
# -> obj/ARK32_ARK_4IN1_F051_<ver>.factory.bin  (flash at 0x08000000)
make factory-image-check   # same + layout/defaults gate (CI)
```

Firmware objects land under `obj/`. This fork’s product targets are **`ARK_4IN1_F051`** (ship firmware) and **`AM32_SITL_CAN`** (host sim). See [`Inc/targets.h`](Inc/targets.h). MCU HAL trees for other AM32 families are still in `Mcu/`, but they are not buildable products here.

### Production release image (ARK 4IN1)

Do **not** hand-assemble production firmware (flash BL → flash app → configurator EEPROM → ST-Link dump). Build the full 32 KiB image from the repo:

```bash
make factory-image
```

That links the release app, then runs [`scripts/build_factory_image.py`](scripts/build_factory_image.py) to lay out:

| Region | Source |
|--------|--------|
| Bootloader @ `0x08000000` | [`Bootloaders/`](Bootloaders/) ARK32-bootloader image |
| Application @ `0x08001000` | `make ARK_4IN1_F051` |
| EEPROM @ `0x08007C00` | [`factory/ARK_4IN1_F051_eeprom_defaults.json`](factory/ARK_4IN1_F051_eeprom_defaults.json) |

Ship/program `obj/ARK32_ARK_4IN1_F051_*.factory.bin` (or `.factory.hex`). Defaults (variable PWM, 1020 kV, 2 %/ms ramp, 15° fixed advance, PWM min/max 1020/1980 µs) are documented in [`factory/README.md`](factory/README.md).

Optional static analysis / size / format helpers:

```bash
make format            # apply clang-format (.clang-format) to app + MCU sources
make check_format      # fail if sources need formatting (used in PR CI)
make format_changed    # format only files changed vs origin/main
make cppcheck          # static analysis of the ARK F051 control path
make size-check-ark    # ARK F051 flash/RAM gate (HWCI+embed worst case, then release)
```

Style is **PX4-inspired** via clang-format (Linux braces, tab indent width 8, `int *p`, column 140) — same `make format` / `check_format` workflow as PX4, not astyle itself. See `.clang-format`.

`make format` skips vendor trees (`Mcu/**/Drivers`, CMSIS, DroneCAN `dsdl_generated` / `libcanard`). It requires clang-format **22.1.5** (same pin as CI) and bootstraps that into `tools/clang-format-venv` if PATH has a different version — Ubuntu's 18.x will not pass CI. Formatting and checking leave Git hook configuration unchanged.

To enable the optional formatting pre-push hook in a clone without existing hooks, run `git config --local core.hooksPath .githooks`. If you already use hooks, integrate `.githooks/pre-push` with your hook manager instead of replacing its configuration. The hook checks each pushed branch or commit tag in a temporary directory using its committed sources and `.clang-format`, so unrelated working-tree edits do not affect the result. Deleted refs and tags pointing to trees or blobs are skipped. To check a commit manually, run `scripts/format.sh --check --ref REV`.

---

## Features (shared with upstream AM32)

- Firmware upgrade via Betaflight passthrough, single-wire serial, or related tools  
- Servo PWM and DShot (300 / 600), including bi-directional DShot  
- KISS-style ESC telemetry  
- Variable PWM frequency and sinusoidal startup for larger motors  
- Multi-vehicle use with a flight controller; crawler-oriented builds exist upstream  

Upstream feature docs and crawler notes: [AM32 wiki / crawler hardware](https://github.com/AlkaMotors/AM32-MultiRotor-ESC-firmware/wiki/Crawler-Hardware-and-AM32).

---

## Motor beeps and sounds

An ESC has no speaker. Beeps are PWM on the motor phases so the windings act as a small transducer (same idea as other BLHeli-family ESCs). Implementation: [`Src/sounds.c`](Src/sounds.c) / [`Inc/sounds.h`](Inc/sounds.h). Volume is EEPROM `beep_volume` (0–11; DroneCAN param `BEEP_VOLUME`, default 5). Sounds only run when the motor is **not spinning** (idle / disarmed / zero throttle as applicable).

Pitch below is **relative** (higher PWM timer prescaler → lower pitch). Exact Hz depends on MCU clock and timer setup. The ARK signature tunes (startup and arm/beacon-4 morse) instead use fixed note frequencies via `playBJNote`.

### Quick reference

| When you hear it | Pattern (pitch) | Function | Meaning |
|------------------|-----------------|----------|---------|
| Power-up | Morse **“ARK”** (·– / ·–· / –·–) rising C6 → E6 → G6 (or custom melody) | `playStartupTune` | Firmware booted and is ready for input |
| Signal lost (after soft-reset) | Single short low blip on C5 (~70 ms) | `playSignalLostTone` | RC/input timeout; distinct from the ARK boot tune |
| Arm / throttle zero accepted | Morse **“R”** (·–·) on G6 | `playInputTune` | ESC armed / input lock-in (“roger”) |
| Arm + cell LVC enabled | That “R” **once per cell** | `playInputTune` × N | Detected pack cell count (`Vbat / 3.70`) |
| Stick cal entered (PWM) | Descending whoop/sweep | `playBeaconTune3` | Entered servo high/low calibration |
| Stick cal high done | 2 notes, **rising** | `playDefaultTone` | Max endpoint captured |
| Stick cal low done | 2 notes, **falling** | `playChangedTone` | Min endpoint saved to EEPROM |
| DShot beacon 1 or 5 | Same as “default” 2-note rising | `playDefaultTone` | Beacon / locate |
| DShot beacon 2 | 2 notes, **falling** | `playChangedTone` | Beacon |
| DShot beacon 3 | Descending sweep | `playBeaconTune3` | Beacon |
| DShot beacon 4 | Morse **“R”** (·–·) on E6 | `playInputTune2` | Beacon |
| DShot cmd 12 (save settings) | Rising if normal dir, falling if reversed | `playDefaultTone` / `playChangedTone` | Settings written |

### Startup

| Function | When | Pattern |
|----------|------|---------|
| **`playStartupTune`** | Normal boot after init | If the previous run soft-reset from an RC **signal timeout**, plays **`playSignalLostTone`** instead (see below). Else if the previous run finished an app-side **bootloader update**, plays **`playBootloaderUpdatedTone`** then continues. Else if EEPROM custom tune byte 0 is programmed (not `0xFF`): plays **BlueJay-compatible** melody from `eepromBuffer.tune[]` via `playBlueJayTune`. Otherwise default: the **ARK signature tune** — “ARK” in morse code (·– / ·–· / –·–), one letter per step up a C major arpeggio (C6 → E6 → G6), ~1.4 s total. |
| **`playSignalLostTone`** | Soft-reset after armed (~0.5 s) or disarmed (~2 s) input timeout (`faultPollSignalTimeout` → `NVIC_SystemReset`) | **One short low blip** on C5 (≈ 523 Hz, ~70 ms). Marked via a `.noinit` cookie before reset so cold boot still plays the full ARK tune. The F051 linker provides `.noinit`. |
| **`playBootloaderUpdatedTone`** | Next boot after a successful app-side bootloader rewrite (`maybe_update_bootloader` → `bootSoundMarkBootloaderUpdated` → reset) | **Two rising beeps** E6 → G6 (~90 ms + ~140 ms). Then the normal ARK/BlueJay startup continues. Cookie in `.noinit` so cold power-on never false-triggers. |
| **`playBlueJayTune`** | Custom startup only | Notes/rests encoded in EEPROM tune blob (configurator “custom startup music”). Inter-note pause can scale with tune header byte 3. |

### Armed / input recognition

Played from the 20 kHz control path when the ESC transitions to armed-idle after a stable zero throttle (`Src/control_loop.c`).

| Function | When | Pattern |
|----------|------|---------|
| **`playInputTune`** | Armed with **low-voltage cutoff mode 1** (cell-based) **off**, or as each cell beep | **Morse “R”** (·–·, “roger — signal received”) on G6 (≈ 1568 Hz), ~320 ms (same busy-wait budget as the old tune). |
| **Cell-count beeps** | Armed **and** `low_voltage_cut_off == 1` | `cell_count = battery_voltage / 370` (≈ 3.70 V/cell), then **`playInputTune` once per cell** with ~100 ms gaps. Count the “R”s to read pack cell count. |
| **`playInputTune2`** | DShot beacon 4 | Same **morse “R”** one arpeggio step lower (E6, ≈ 1319 Hz) so the beacon is distinguishable from the arm tune. |

### Servo PWM stick calibration

Only for **servo PWM** input when stick calibration is not disabled. Sequence in `Src/signal.c`:

1. Hold **high stick** long enough → **`playBeaconTune3`** (descending multi-step whoop) — calibration mode entered.
2. Hold steady **max** until accepted → **`playDefaultTone`** (two notes, **rising**: lower then higher).
3. Move to **min** and hold until accepted → **`playChangedTone`** (two notes, **falling**: higher then lower) — endpoints saved.

### DShot special commands (beacons & save)

DShot commands run only when **armed**, **motor not running**, and the command is repeated enough times (`Src/dshot.c`). Beacons 1–5 set `play_tone_flag`; actual audio plays on the next idle/low-throttle slot in `setInput` (`Src/control_loop.c`).

| DShot cmd | `play_tone_flag` | Sound | Typical use |
|-----------|------------------|-------|-------------|
| **1** | 1 | `playDefaultTone` — 2 notes rising | Beacon 1 |
| **2** | 2 | `playChangedTone` — 2 notes falling | Beacon 2 |
| **3** | 3 | `playBeaconTune3` — long descending sweep | Beacon 3 |
| **4** | 4 | `playInputTune2` — morse “R” on E6 | Beacon 4 |
| **5** | 5 | `playDefaultTone` — same as beacon 1 | Beacon 5 |
| **12** | `1 + dir_reversed` | Rising if direction normal, falling if reversed | **Save settings** confirmation |

Other DShot commands (direction, bi-dir, EDT, programming mode, etc.) do **not** play a dedicated melody unless noted above. Direction set (7/8) has no confirmation beep.

EEPROM writes over DShot (commands 36 and 37): [`doc/dshot-programming.md`](doc/dshot-programming.md).

### Beacon sweep detail

**`playBeaconTune3`**: stepped descending pitch with phase stepping (~10 ms steps, prescaler from high down toward lower values). Used as DShot beacon 3 and as the “entered stick calibration” cue.

### Volume and silence

| Setting | Effect |
|---------|--------|
| **`beep_volume` 0–11** | Duty cycle of the beep PWM (`volume * 3` compare counts). 0 is effectively silent; higher is louder (still limited so the motor barely moves). |
| Motor spinning / throttle up | Deferred tone flags wait until throttle is at idle; beeps are not mixed into normal drive. |
| No throttle signal after boot | ESC may stay in bootloader or keep waiting for input — you may only hear the **startup** tune, not the arm tune, until a valid zero throttle is seen. |

### Source map

| File | Role |
|------|------|
| [`Src/sounds.c`](Src/sounds.c) | All melody generators |
| [`Src/main.c`](Src/main.c) | Startup tune at boot |
| [`Src/control_loop.c`](Src/control_loop.c) | Arm beeps, cell count, deferred DShot tones |
| [`Src/dshot.c`](Src/dshot.c) | DShot command → tone flag |
| [`Src/signal.c`](Src/signal.c) | Servo stick-calibration tones |
| [`Src/settings.c`](Src/settings.c) | Applies `beep_volume` from EEPROM |

---

## Bootloader

ARK ESCs use **[ARK32-bootloader](https://github.com/ARK-Electronics/ARK32-bootloader)** (fork of upstream [AM32-bootloader](https://github.com/am32-firmware/AM32-bootloader)). Use ARK release images for ARK hardware — not the stock upstream bootloader alone when you need ARK-specific fixes (e.g. bidirectional DShot idle detection).

| | |
|--|--|
| Source / releases | [ARK-Electronics/ARK32-bootloader](https://github.com/ARK-Electronics/ARK32-bootloader) · [releases](https://github.com/ARK-Electronics/ARK32-bootloader/releases) |
| Committed F051 image for app embed | [`Bootloaders/`](Bootloaders/) (see [Bootloaders/README.md](Bootloaders/README.md)) |
| App-side BL update | F051 builds embed the image by default (including `HWCI_PERF=1`) and rewrite the on-chip BL if it differs (`Src/bootloader_update.c`). Success soft-resets; the next boot plays **`playBootloaderUpdatedTone`** (two rising beeps) then the normal startup tune. Strip with `EMBED_BOOTLOADER=0` or `NO_EMBED_BL=1`. |

To put ARK32 on a **blank production ESC**, flash the full-chip factory image (`make factory-image` → `obj/*factory.bin` at `0x08000000`) so bootloader, app, and EEPROM defaults land in one step — see [factory/README.md](factory/README.md). For development or field app-only updates, flash a matching **ARK32-bootloader** with ST-LINK (if needed), then the application `.bin`/`.hex` at `0x08001000` (or use a configurator / one-wire serial). Later app flashes can also carry and apply a newer BL via the embed path above.

## EEPROM settings

Every user-facing EEPROM field is documented in [`doc/eeprom-settings.md`](doc/eeprom-settings.md): what it does, the range the configurator shows, and the ARK 4IN1 factory default. The same text is the **Settings guide** in [ARK32 Configurator](https://github.com/ARK-Electronics/ark32-configurator).

## Configuration tools & stock firmware

For ARK hardware use **[ARK32 Configurator](https://github.com/ARK-Electronics/ark32-configurator)**. Settings text is the same as [`doc/eeprom-settings.md`](doc/eeprom-settings.md).

These are **upstream / community** tools; they are not ARK-specific:

- [AM32 Configurator](https://am32.ca) (web) and [downloads](https://am32.ca/downloads)  
- [esc-configurator.com](https://esc-configurator.com/)  
- Upstream bootloaders (non-ARK): [am32-firmware/AM32-bootloader](https://github.com/am32-firmware/AM32-bootloader)  
- Upstream target list: [am32-firmware/AM32 `Inc/targets.h`](https://github.com/am32-firmware/AM32/blob/main/Inc/targets.h)

---

## Hardware

ARK32 ships the **ARK 4IN1** (four STM32F051 channels, DRV8328). SITL is a host build of the same firmware for tests. Other AM32 boards are not product targets in this tree — use [upstream AM32](https://github.com/am32-firmware/AM32) for those.

---

## Support

| Topic | Where |
|-------|--------|
| **ARK32** | [ARK-Electronics/ARK32](https://github.com/ARK-Electronics/ARK32) issues and PRs on `main` |
| **Upstream AM32** | [Discord](https://discord.gg/h7ddYMmEVV), [Patreon](https://www.patreon.com/user?u=44228479), [am32.ca](https://am32.ca) |

---

## License

GPL-3.0 — see [LICENSE](LICENSE). Same license family as upstream AM32.

---

## Upstream credits

AM32 exists because of its authors, sponsors, and community. ARK32 inherits that work; see the [upstream README](https://github.com/am32-firmware/AM32/blob/main/README.md) for the full sponsor and contributor lists.
