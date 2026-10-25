# Two exclusive bench setups

This harness supports **two different physical setups**. Use **one at a time** —
only one device may drive the ESC signal wire.

| | **SETUP A — Flight Stand** | **SETUP B — ARK FPV BDShot** |
|--|----------------------------|------------------------------|
| **Who drives ESC signal** | Flight Stand ESC output (uni DShot/PWM) | ARK FPV motor out (BDShot300/600 + EDT) |
| **PX4** | Not in the path | Required (USB MAVLink) |
| **BDShot replies** | No (stand is not a BDShot master) | Yes (eRPM / EDT on signal wire) |
| **Thrust / torque / optical RPM** | Flight Stand gRPC | Optional (stand sensors only; pin disconnected from stand ESC out) |
| **SWD `hwci_perf`** | Yes (ST-Link) | Optional (same ST-Link) |
| **Primary rig file** | `rig.yaml` / `config/rig.g431_can.yaml` / `config/rig.flightstand.yaml` | `config/rig.px4_bdshot.yaml` |
| **How to command motor** | `hwci run --profile noprop_…` | `scripts/px4_motor_stream.py` (+ capture) |
| **Typical profiles** | `noprop_smoke*`, `efficiency_sweep`, demag, … | `bdshot_smoke` (SWD log only; throttle is PX4) |

## MCU family (F051 vs G4)

| | **ARK_4IN1_F051** | **ARK_G431_CAN** (12S CAN ESC) |
|--|-------------------|--------------------------------|
| Core | Cortex-M0 | Cortex-M4 (DWT available; harness still uses `hwci_perf` RAM poll) |
| OpenOCD target | `target/stm32f0x.cfg` | `target/stm32g4x.cfg` |
| App load address | `0x08001000` (4 KiB BL) | `0x08004000` (16 KiB BL) |
| Typical SWD clock | 2 MHz | 8 MHz |
| Debug console | none | USART2 TX **PB3** @ 115200 → ST-Link VCP |
| Rig template | `config/rig.flightstand.yaml` | `config/rig.g431_can.yaml` (active `rig.yaml` on G4 bench) |

Known targets fill `openocd_configs`, `app_load_addr`, and `adapter_speed_khz`
automatically via `TARGET_PRESETS` when those keys are omitted from the YAML.

### G4 debug UART (ST-Link VCP)

Firmware (`USE_DEBUG_UART`) prints state transitions and faults on PB3.
Wire PB3 → ST-Link V3 VCP RX (and common GND). Install udev rules so the VCP
appears as `/dev/esc-debug-uart` (`scripts/99-hwci.rules`, interface 01).

```bash
# Live console (boot banner, esc: A -> B, fault: nFAULT / desync, …)
hwci debug-uart --config rig.yaml

# Each hardware run with debug_uart_backend: serial also writes
#   <run_dir>/debug_uart.log
# and aborts on fault: nFAULT. Other fault lines remain in the log.
# warn: nFAULT / retry does not abort a run.
```

### G4 nFAULT warning and drive-loss latch

On DRV8350H, nFAULT alone does not distinguish an overtemperature warning
from a fault that disables the bridge. A held pin reports a warning; short
observed pulses report retry warnings. Neither cuts drive. The smoothed
current reading is telemetry, not evidence that the driver is live or dead.

The existing BEMF stall and established-run desync paths latch drive off
when drive is still commanded and nFAULT is low or was observed recently.
Releasing nFAULT while throttle remains applied does not restart the motor.
Rearming requires zero throttle continuously for 100 ms; an isolated zero
frame does not clear the latch. There is no timed high-throttle restart.

The nFAULT jumper checks the **warning** path. Use a stable, unloaded bench
run and save the UART log and phase-current trace:

1. Hold a stable throttle, then pull nFAULT low for longer than 12 ms.
2. Expect `warn: nFAULT` while the motor continues running.
3. Release the jumper. Expect the warning to clear without a drive cut.

Grounding this reporting pin does not disable the gate driver. A drive-loss
latch test needs an actual driver-disabling event, or a controlled injection
that combines nFAULT with lost BEMF. Do not count the warning-jumper test as
validation of this path. For a drive-loss test, verify `fault: nFAULT`, zero
drive through pin release at nonzero throttle, and recovery only after the
continuous-zero-throttle rearm interval. Save the phase-current trace to
check the first restart commutation as well as the UART events.

### G4 current and coast measurements

The existing `g431_current_floor` profile records idle and low-throttle
current for bench characterization. Its historical name does not imply a
current threshold in the nFAULT policy:

```bash
.venv/bin/python -m hwci run --config rig.yaml --profile g431_current_floor \
  --out runs/g431-current-floor
```

In `samples.csv` / `hwci_perf.current_ca`, compare the distributions of:

* `idle` (armed, throttle 0, DRV may be asleep — offset / noise)
* `hold05` / `hold08` (lowest nonzero throttle this ESC is actually commanded)

The existing `g431_hiz_spindown` profile measures ordinary stop-then-punch
behavior at several coast durations. `require_eeprom` aborts unless
`brake_on_stop=0` and `rc_car_reverse=0`, so these runs measure an unbraked
coast. The harness checks the settings on the device before starting:

```bash
.venv/bin/python -m hwci run --config rig.yaml --profile g431_hiz_spindown \
  --out runs/g431-hiz-spindown
```

For each `re20_*` segment, measure time from throttle-up to a stable running
RPM and inspect `fault: stall` bursts. These measurements describe that
motor and bench load; they do not validate an automatic nFAULT restart or
represent a windmilling propeller in flight.

## SETUP A — Flight Stand throttle (no PX4 / no BDShot)

```
  Linux host ──gRPC──► Flight Stand ──ESC signal (uni DShot)──► AM32
       │                    │
       └──SWD (ST-Link)─────┴── optional optical RPM / thrust / current
```

- Wire ESC **signal** to the Flight Stand ESC output only.
- Do **not** connect ARK FPV motor outputs to that pin at the same time.
- Config: see `config/rig.flightstand.yaml` (active bench file is usually `rig.yaml`).
- Examples:

```bash
cd hwci
# flash + free-run smoke (stand drives throttle)
.venv/bin/python -m hwci flash --config rig.yaml --bin ../obj/ARK32_ARK_4IN1_F051_*.bin
.venv/bin/python -m hwci run --config rig.yaml --profile noprop_smoke_100pct_3a --out runs/stand-smoke-1
```

## SETUP B — ARK FPV BDShot (no Flight Stand throttle)

```
  Linux host ──USB──► ARK FPV (PX4) ──BDShot+EDT signal──► AM32
       │                  │
       │                  └── optional KISS serial telem UART
       └──SWD (ST-Link) optional hwci_perf dshot_* counters
```

- Wire ESC **signal** to the FPV motor pad only (not stand ESC out).
- Flight Stand may stay powered for **sensors** only if the signal pin is free;
  typically leave stand throttle disconnected for this setup.
- Config: `config/rig.px4_bdshot.yaml` (`throttle_backend: none`).
- Motor command + eRPM live on PX4; scripts:

```bash
cd hwci
# flash instrumented FW (throttle not driven by harness)
.venv/bin/python -m hwci flash --config config/rig.px4_bdshot.yaml \
  --bin ../obj/ARK32_ARK_4IN1_F051_*.bin

# drive motor via PX4 ACTUATOR_TEST (soft re-fire + ramp-down)
./scripts/px4_motor_stream.py --port /dev/ttyACM2 \
  --steps 0.12:5,0.25:5,0.40:6 --refresh 1.5 --timeout 3.0 --ramp-down 6

# optional: log esc_status only
./scripts/px4_bdshot_capture.py --port /dev/ttyACM2 --duration 40 -o runs/px4-cap.csv
```

Details: [setup_px4_bdshot.md](setup_px4_bdshot.md) (formerly `bdshot_baseline.md`).

## Switching between setups

1. **Power down** ESC / motor bus.
2. **Move the signal wire** (stand ESC out ↔ FPV motor pad). Never parallel both.
3. Use the matching **rig config** and **command path** from the table above.
4. For SETUP B on a battery, prefer soft ramp-down; on a 3 A brick, hard stop
   can fault the supply (see PX4 docs).

## What is shared

- Same AM32 target / `HWCI_PERF=1` firmware (BDShot counters are zero-cost when
  idle if the host never enables BDShot). F051 also **embeds the bootloader**
  by default in that image; after a BL version bump the first boot may rewrite
  the BL and soft-reset (see [hwci/README.md](../README.md#embedded-bootloader-f051)).
- Same ST-Link debug header.
- Same motor/ESC hardware (wiring of the **signal** pin differs).
