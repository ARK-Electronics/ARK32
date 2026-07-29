# ARK 4IN1 F051 + 900KV motor — hardware calibration captures

Real-hardware logs for AM32 SITL motor-model calibration, captured on the
ARK Electronics hardware-CI thrust stand (Tyto Robotics Flight Stand 50).

JSONL schema matches `scripts/esc_measure.py` / `esc_square.py` / `esc_chirp.py`
(same as the SEQURE / VimDrones data sets under `Mcu/SITL/data/`).

## Setup

| Item | Value |
|------|--------|
| ESC | ARK 4IN1, STM32F051, channel 1, AM32 (ARK fork) |
| Target | `ARK_4IN1_F051` |
| Signal | Flight Stand ESC out, **uni DShot** (not DroneCAN) |
| Throttle map | DShot 0 = idle/arm, 48…2047 = 0…100% |
| Motor | **900 KV**, **14 poles** |
| Prop | **none** (noprop free-run) |
| EEPROM | `motor_kv=22` (→ 900), `motor_poles=14`, `variable_pwm=1` (PWM by RPM), `pwm_frequency=48` kHz, `advance_level=18` |
| Supply | 6S LiPo, ~24.7 V at session start |
| RPM | Flight Stand optical |
| V / I | Flight Stand HV + hall current |
| Date | 2026-07-20 |

No FlexDebug/debug1 (no DroneCAN). Status lines may include extra `thrust_n`
(near zero noprop); stock analyzers ignore unknown fields.

## Files (tridge capture battery)

| File | Profile |
|------|---------|
| `first_spin_010.jsonl` | Hold 0.10 for 6 s after arm |
| `sweep1.jsonl` | **0.05…0.50** step 0.05, **4 s** holds, up then down |
| `square1.jsonl` | Mid **0.35**, deltas 0.03…0.24, 3 cycles |
| `chirp_120s.jsonl` | **120 s** chirp 0.5…40 Hz, mid 0.35 ± 0.15 |
| `capture_meta.json` | Machine-readable setup |
| `expected.json` | Reference numbers from this capture |

## Summary numbers

### Steady sweep (up staircase)

| throttle | rpm | V | A |
|----------|-----|---|---|
| 0.05 | 1037 | 24.71 | 0.03 |
| 0.10 | 2328 | 24.70 | 0.11 |
| 0.15 | 3522 | 24.69 | 0.23 |
| 0.20 | 4648 | 24.68 | 0.33 |
| 0.25 | 5797 | 24.67 | 0.46 |
| 0.30 | 6901 | 24.66 | 0.58 |
| 0.35 | 8089 | 24.65 | 0.70 |
| 0.40 | 9197 | 24.64 | 0.82 |
| 0.45 | 10347 | 24.63 | 0.96 |
| 0.50 | 11550 | 24.62 | 1.06 |

### Square (`esc_square.py analyze`)

| delta | up_ms | down_ms |
|-------|-------|---------|
| 0.030 | 37 | 20 |
| 0.060 | 47 | 46 |
| 0.090 | 17 | 29 |
| 0.120 | 29 | 33 |
| 0.150 | 47 | 49 |
| 0.180 | 46 | 33 |
| 0.210 | 34 | 47 |
| 0.240 | 46 | 65 |

### Chirp (`esc_chirp.py fit`)

- baseline centre ≈ 8115 rpm, amplitude ≈ 3462 rpm  
- empirical **accel 3 dB ≈ 7.92 Hz**, **brake 3 dB ≈ 8.71 Hz**  
  (noprop; braking not aero-dominated)

## Analyze

```bash
python3 scripts/esc_analyze.py  Mcu/SITL/data/ARK_4IN1_F051_900kv_noprop/sweep1.jsonl
python3 scripts/esc_square.py analyze Mcu/SITL/data/ARK_4IN1_F051_900kv_noprop/square1.jsonl
python3 scripts/esc_chirp.py fit Mcu/SITL/data/ARK_4IN1_F051_900kv_noprop/chirp_120s.jsonl --plot
```

## Notes

- Capture tool: `hwci/scripts/sitl_cal_capture.py` (DShot via Flight Stand gRPC).  
- Free-run at 50% ≈ 11.6k rpm on ~24.6 V is consistent with ~900 KV.  
- Contact: ARK Electronics (AM32 fork).
