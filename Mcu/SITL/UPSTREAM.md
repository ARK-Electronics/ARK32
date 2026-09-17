# Upstream SITL alignment

Reviewed on 2026-09-15 against these pinned revisions:

| Component | Revision |
| --- | --- |
| AM32 simulator | [5bf674ae93fb710940ba1637ddb064732c904c8c](https://github.com/am32-firmware/AM32/commit/5bf674ae93fb710940ba1637ddb064732c904c8c) |
| ESCSim tools and tests | [9d1b0010d8baeb8aa937f9b466dbf3c21fc843e5](https://github.com/am32-firmware/ESCSim/commit/9d1b0010d8baeb8aa937f9b466dbf3c21fc843e5) |

AM32 moved its GUI, models, datasets and Python tests to ESCSim. ARK keeps its
product models, calibration captures, GUI and control-path pytest suite here.
The upstream compatibility job checks out ESCSim at `escsim-revision.txt` and
runs its behavioral suite through `run_upstream_tests.py`. The adapter checks
the revision before running; changing external `main` cannot silently change CI.

## Simulator changes since the previous alignment

The previous baseline was AM32 `79af09b00c52021895cddff4080a983aa1b1279c`.
This update ports upstream's shared runtime changes:

- Variable watches on state command 8, including initial values, change events,
  refreshable subscriptions and the `sitl_tone_active` global.
- State command 9 resets the ESC, preserving EEPROM. Execution is deferred
  outside nested interrupt stepping; a chained bootloader receives the
  `software` reset cause.
- UDP input, CAN and state polling follows simulated time, including time
  advanced while an interrupt handler plays a tone.
- Opt-in version 3 scope samples include instantaneous back EMF, filtered
  comparator inputs, body-diode conduction, active duty and desync count.
  Subscribe flag bit 1 selects 100-byte v3 samples and overrides averaging.
  Ordinary subscribers keep the 60-byte v2 layout used by the local ARK GUI.

Firmware control behavior, ARK fault injection, ZC statistics and governor
hooks are preserved. The scope adds observations to the motor model without
changing its dynamics. Native MinGW builds retain upstream's limitation:
variable watch names are unresolved because that backend has no `dlsym`.

### ARK command migration

Upstream assigned commands 8 and 9 to watch and reset. ARK extensions now use
this separate range; the committed ARK clients are updated together:

| Command | Previous ID | Current ID |
| --- | --- | --- |
| ZC_FAULT | 8 | `0x80` |
| ZC_STATS | 9 | `0x81` |
| GOV_FORCE | 10 | `0x82` |

Update external ARK test clients before using the new simulator. The old
ZC_STATS request is identical to upstream RESET, so it cannot remain an alias.
The ZC_STATS v6 reply fields and magic remain unchanged.

## Test adaptations

The upstream runner and its clients are loaded directly from ESCSim. The
small ARK adapter retains these deliberate differences:

- ARK32 binary discovery and input/state ports below Windows' ephemeral range.
- ARK Morse startup tones and 1047 Hz physics audio from `Src/sounds.c`.
  Short contiguous audio windows preserve dots and handle dropped UDP batches.
- Zero-throttle CAN keepalives during parameter round trips, because ARK
  resets after input timeout.
- With stuck-rotor protection disabled, a brief obstruction must recover
  without cycling throttle. A sustained obstruction must still trip ARK's
  independent acquisition/desync episode rail, remain stopped after release,
  and recover after zero throttle. Upstream's fixed three-second obstruction
  falls near that rail's latch threshold and cannot distinguish these cases.
- ARK firmware version defaults are selected before ESCSim imports its
  parameter generator. Its defaults reader is connected to ARK's schema-backed
  defaults because the literal C array has been replaced by a generated header.
  Those defaults include the target's input-mode and protection corrections
  after the shared EEPROM skeleton is applied.
  Upstream parameter parsing and image construction remain in use; all three
  ARK and three upstream model/parameter pairs are checked.
- CAN dependencies and multicast startup failures fail the suite. Sanitizer
  diagnostics in the child log also fail the suite.

CI runs ESCSim behavioral tests and native state-protocol regressions on
Linux, Linux ASan/UBSan, and macOS. ESCSim adds scope acquisition, live v2/v3
wire compatibility, repeated FC passthrough sessions and virtual USB/IP tests.
Bootloader four-way and direct-serial tests require an explicit `--bootloader`
SITL binary and otherwise report skips. Existing ARK pytest, calibration, GUI
and Windows jobs remain.

```sh
make AM32_SITL_CAN
python3 -m pip install -r Mcu/SITL/requirements-ci.txt
git clone https://github.com/am32-firmware/ESCSim
git -C ESCSim checkout "$(cat Mcu/SITL/escsim-revision.txt)"
mkdir -p upstream_test_run
cd upstream_test_run
python3 ../Mcu/SITL/run_upstream_tests.py --escsim ../ESCSim \
  --sitl ../obj/ARK32_AM32_SITL_CAN_*.elf
```

From the repository root, run the ARK suite separately:

```sh
python3 Mcu/SITL/run_ci_tests.py --sitl obj/ARK32_AM32_SITL_CAN_*.elf
```

For ESCSim's new demagnetization scope, install `ESCSim/SITL/requirements.txt`
and launch its GUI with the ARK binary selected:

```sh
AM32_ROOT="$PWD" AM32_SITL="$PWD/obj/ARK32_AM32_SITL_CAN_*.elf" \
  python3 ESCSim/SITL/sitl_gui.py
```

See ESCSim's `SITL/DEMAG.md` and `SITL/TIMING-DESIGN.md` for scope and scheduler
usage. The local GUI remains usable with its existing v2 stream.

For the next sync, compare against both revisions above, port shared runtime
changes, update the ESCSim pin and adapter as needed, and run both suites and
ARK calibration gates. Preserve ARK control-path tests and their expected
behavior.
