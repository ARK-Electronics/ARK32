# ARK 4IN1 F051 + JS 2807 1300KV — hardware calibration captures

Real-hardware logs for AM32 SITL motor-model calibration, captured on the
ARK Electronics hardware-CI thrust stand (Tyto Robotics Flight Stand 50).

These files use the **same JSONL schema** as the DroneCAN tools in
`scripts/esc_measure.py`, `esc_square.py`, and `esc_chirp.py`
(`type=status|cmd|chirp|square_config|…` with `t`, `rpm`, `volt`, `curr`, …)
so they drop into the same analysis path as the SEQURE / VimDrones data sets
under `Mcu/SITL/data/`.

## Setup

| Item | Value |
|------|--------|
| ESC | ARK 4IN1, STM32F051, channel 1, AM32 (ARK fork) |
| Target name | `ARK_4IN1_F051` |
| Signal / throttle | Flight Stand ESC output, **uni DShot** (not DroneCAN) |
| Throttle map | DShot 0 = idle/arm, 48…2047 = 0…100% throttle |
| Motor | JS Technology **2807 1300KV**, **14 poles** |
| Prop | **none** (noprop free-run) |
| Supply | 6S LiPo, ~24.9 V at start of session |
| RPM truth | Flight Stand optical encoder (`stand_rpm` → `rpm` in JSONL) |
| V / I truth | Flight Stand HV voltage + hall current |
| Date | 2026-07-20 |

No FlexDebug / debug1 fields (no DroneCAN). `err` is 0 in these logs
(no bemf-timeout channel on the stand path). Extra key `thrust_n` is present
on status lines (near zero with noprop) and can be ignored by stock tools.

## Capture profiles (tridge battery)

| File | Profile |
|------|---------|
| `first_spin_010.jsonl` | Hold throttle 0.10 for 6 s after arm |
| `sweep1.jsonl` | Staircase **0.05…0.50** step 0.05, **4 s** holds, **up then down** |
| `square1.jsonl` | Mid **0.35**, deltas 0.03…0.24, 3 cycles each |
| `chirp_120s.jsonl` | **120 s** exponential chirp 0.5…40 Hz, mid 0.35 ± 0.15 |
| `capture_meta.json` | Machine-readable setup summary |
| `expected.json` | Steady / chirp / square reference numbers from this capture |

## Summary numbers (this session)

### Steady sweep (up staircase, tail of each hold)

| throttle | rpm | V | A |
|----------|-----|---|---|
| 0.05 | 1577 | 24.86 | 0.03 |
| 0.10 | 3296 | 24.86 | 0.09 |
| 0.15 | 4907 | 24.85 | 0.17 |
| 0.20 | 6539 | 24.84 | 0.27 |
| 0.25 | 8199 | 24.83 | 0.35 |
| 0.30 | 9681 | 24.82 | 0.45 |
| 0.35 | 11311 | 24.81 | 0.52 |
| 0.40 | 12849 | 24.80 | 0.60 |
| 0.45 | 14408 | 24.80 | 0.68 |
| 0.50 | 15929 | 24.79 | 0.75 |

### Square (t10–90, `esc_square.py analyze`)

| delta | up_ms | down_ms |
|-------|-------|---------|
| 0.030 | 17 | 31 |
| 0.060 | 48 | 21 |
| 0.090 | 46 | 27 |
| 0.120 | 54 | 30 |
| 0.150 | 50 | 48 |
| 0.180 | 33 | 30 |
| 0.210 | 50 | 64 |
| 0.240 | 48 | 46 |

### Chirp (`esc_chirp.py fit`)

- baseline centre ≈ 11248 rpm, amplitude ≈ 4698 rpm  
- empirical **accel 3 dB ≈ 9.46 Hz**, **brake 3 dB ≈ 4.97 Hz**

## How to analyze (same as other data sets)

```bash
python3 scripts/esc_analyze.py  Mcu/SITL/data/ARK_4IN1_F051_2807_1300kv/sweep1.jsonl
python3 scripts/esc_square.py analyze Mcu/SITL/data/ARK_4IN1_F051_2807_1300kv/square1.jsonl
python3 scripts/esc_chirp.py fit Mcu/SITL/data/ARK_4IN1_F051_2807_1300kv/chirp_120s.jsonl --plot
python3 scripts/esc_plot.py Mcu/SITL/data/ARK_4IN1_F051_2807_1300kv/sweep1.jsonl \
  Mcu/SITL/data/ARK_4IN1_F051_2807_1300kv/square1.jsonl --html
```

## Notes for model fit

- Unloaded 2807 on ~25 V: peak no-load current in this sweep &lt; 1 A.  
- Optical RPM is mechanical; poles=14 if converting eRPM elsewhere.  
- Capture tool: `hwci/scripts/sitl_cal_capture.py` (DShot via Flight Stand gRPC).  
- Contact: ARK Electronics (AM32 fork); data intended for upstream SITL cal work.
