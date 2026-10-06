# Upstream AM32 vs ARK32: hardware A/B protocol

This is how to measure whether ARK32 `main` performs differently from current
upstream AM32 on the ARK 4IN1. The README's performance claims should cite a
run of this protocol.

The last upstream comparison ran on 2026-07-02, with one run per firmware, on
the fork-point upstream. Almost all of today's control path landed after it.
Since then, bench work has compared ARK32 only against older ARK32 builds.

**Rule:** report a difference only when each arm has at least 3 runs and the
two arms' ranges do not overlap. `hwci ab-summary` applies this rule for you.
Below that bar, the result is "within spread", whichever median looks better.

## 1. The two arms

| Arm | Firmware | Build |
|---|---|---|
| **A: upstream** | Branch `ab/upstream-instrumented-8574381` (#124): upstream `8574381` plus `HWCI_PERF` hooks only. Without `HWCI_PERF` it builds byte-identical to upstream. | In its own checkout, with upstream's pinned GCC 10.3.1 and `-O3`: `make arm_sdk_install && make ARK_4IN1_F051 HWCI_PERF=1`. Produces `obj/AM32_ARK_4IN1_F051_2.21.{elf,bin}`. |
| **B: ARK32** | `main` HEAD (record the sha) | `make clean && make ARK_4IN1_F051 HWCI_PERF=1` with the pinned GCC 15. Produces `obj/ARK32_ARK_4IN1_F051_<ver>.{elf,bin}`. |

```bash
# from the ARK32 checkout
git worktree add ../ARK32-upstream-arm ab/upstream-instrumented-8574381
(cd ../ARK32-upstream-arm && make arm_sdk_install && make ARK_4IN1_F051 HWCI_PERF=1)
make clean && make ARK_4IN1_F051 HWCI_PERF=1
```

- **Toolchain:** each arm uses the toolchain and optimisation flags its project
  ships. The compiler is part of what is being compared, so do not rebuild
  upstream with GCC 15 or ARK32 with `-O3`.
- **Instrumentation:** both arms emit the same `hwci_perf` v7 struct (172
  bytes), decoded by this checkout's `hwci`. On the upstream arm:
  - `esc_state` is derived from upstream's flags.
  - `zc_blind_steps` and `esc_illegal_edge_count` are always 0, because
    upstream has neither mechanism.
  - `zc_confirm_reject` counts upstream's strict filter-loop rejects, a
    different filter from ARK32's glitch-tolerant confirm.
- **Bootloader:** ARK32 builds embed ARK32-bootloader and install it on the
  first boot if the chip has a different one. The upstream app never touches
  the bootloader. Flash arm B once before the first arm A run so both arms
  run on the same bootloader.

## 2. Rig files

`hwci run` reads `hwci_perf` using the ELF from the rig file. It also stamps
`meta.json` with `git rev-parse HEAD` of `repo_root`. Keep one rig file per
arm, identical except for these lines (absolute paths, because relative paths
resolve against the shell's cwd):

```yaml
# config/rig.ab-upstream.yaml: copy of your bench rig file, plus
repo_root: /abs/path/ARK32-upstream-arm
elf_path:  /abs/path/ARK32-upstream-arm/obj/AM32_ARK_4IN1_F051_2.21.elf

# config/rig.ab-ark32.yaml: copy of your bench rig file, plus
repo_root: /abs/path/ARK32
elf_path:  /abs/path/ARK32/obj/ARK32_ARK_4IN1_F051_<ver>.elf
```

`hwci ab-summary` warns when both arms report the same `git_sha`, which
usually means a rig file points at the wrong checkout.

## 3. Identical settings on both arms

**Layout:** offsets 0–191 are the same in ARK32's schema (EEPROM v3) and
upstream's `eeprom.h` (v4), with one exception. Byte 13 is
`brake_on_zero_throttle` upstream and reserved in ARK32. The factory page has
`0x30` there (an old firmware-name byte), which upstream treats as invalid and
runs as 0, meaning off.

**Header bytes:** bytes 0–4 belong to whichever firmware is flashed (boot
byte, EEPROM version, bootloader version, firmware version). If a firmware
finds another firmware's version there, it rewrites the page at boot. Neither
arm resets user settings on the other's version: both reset only below v3.
Even so, `--keep-identity` (below) keeps each firmware's own header so no
rewrite happens.

**Reference settings:** the factory EEPROM (`factory/ARK_4IN1_F051_eeprom_defaults.json`)
with motor kV and pole count set to the bench motor:

| Setting | Value |
|---|---|
| Ramp | 2 %/ms (`max_ramp` 20) |
| Advance | 15° fixed (`advance_level` 26, `auto_advance` 0) |
| PWM | Variable PWM by RPM, 24 kHz |
| Complementary PWM | On |
| Startup power | 100 |
| Input | DShot |

Both firmwares use kV (upstream for its low-RPM throttle limit, ARK32 for its
RPM envelope), so it must be right and identical. kV is stored as 20 + 40n,
so use 900 for the 900KV motor and 1780 or 1820 for the 1800KV.

```bash
python3 - <<'EOF'
import json, sys
sys.path.insert(0, "scripts")
from build_factory_image import build_eeprom_page
d = json.load(open("factory/ARK_4IN1_F051_eeprom_defaults.json"))
d["settings"].update(motor_kv=1780, motor_poles=14)      # the bench motor
open("hwci/runs/ab/settings_2306.bin", "wb").write(build_eeprom_page(d, 32, 0, 3)[:192])
EOF
```

After every app flash, write the page and confirm it from `hwci/`:

```bash
python -m hwci settings write --config config/rig.ab-<arm>.yaml --bin runs/ab/settings_2306.bin --keep-identity
python -m hwci settings diff  --config config/rig.ab-<arm>.yaml --bin runs/ab/settings_2306.bin --keep-identity   # "no differences"
```

## 4. Setups and profiles

Use SETUP A (Flight Stand, [BENCH_SETUPS.md](BENCH_SETUPS.md)) on a 6S pack.
Run both setups; ARK32's control-path changes target heavy props, so a 5"-only
A/B misses the case they were built for.

| Setup | Profiles (one A/B per profile) |
|---|---|
| **5"**: JS 2306 1800KV + HQProp 5136 | `efficiency_sweep`, `jitter_probe`, `demag_step_stress`, `startup_reliability`, `low_rpm_crawl` |
| **10"**: 900KV + HQProp 10x5x3 | `efficiency_sweep_10inch` (t10–t70, 10" safety limits), `ramp_snap_10inch`, `startup_reliability`, `low_rpm_crawl` |

`efficiency_sweep` runs to 100% with 40 A / 16 N cutoffs sized for the 5", so
it aborts on the 10". Use `efficiency_sweep_10inch` there.

## 5. Run order and discipline

- **Counterbalanced order:** at least 3 runs per arm per profile, 4 preferred,
  in the order A B B A A B B A. A strict ABAB order lets pack sag and motor
  heating bias whichever arm runs second; ABBA cancels a linear drift.
- **Per run:** flash the arm's app (`hwci flash --config rig.ab-<arm>.yaml --bin <arm .bin>`),
  write and diff the settings page (§3), then run:
  `hwci run --config rig.ab-<arm>.yaml --profile <p> --battery-cells 6 --out runs/ab/<setup>/<p>/<arm>-<k>`.
- **Same conditions for both arms:** use the same pack, or swap packs at the
  same point in both arms' sequence. Use the same prop and mount, and a cold
  start (FET and motor temperature back near ambient before each run).
  `--battery-cells` refuses a sagging pack.
- **Aborts are data:** an abort (desync, current or thrust cutoff) is a
  result, so do not re-run it away. `ab-summary` counts aborts per arm
  (`summary.aborted`) and lists them.
- **Keep everything:** keep every run directory, including failed ones.

## 6. Analysis

```bash
cd hwci
python -m hwci ab-summary \
  --a "runs/ab/5in/efficiency_sweep/upstream-*" \
  --b "runs/ab/5in/efficiency_sweep/ark32-*" \
  --a-label upstream --b-label ark32 --out runs/ab/5in/efficiency_sweep/summary.md
```

For each arm, `ab-summary` reports n, the median and the [min..max] range,
then the median delta and the verdict, for:

- **Every steady point:** rpm, thrust, current, power, g/W, CPU load,
  control/main-loop worst case, ZC jitter, confirm rejects per ZC, and
  temperatures.
- **Run summaries:** peak current, demag events, BEMF-timeout samples, blind
  steps, start failures and time-to-run, and aborts.
- **Spool-up:** for every step or snap segment, the time to reach 90% of its
  settled rpm (`<segment>.spool_t90_ms`). A run that never reaches 90% reports
  nothing for that segment rather than a slow time.

Verdicts:

| Verdict | Meaning |
|---|---|
| `n<min` | Fewer than `--min-n` (default 3) runs on an arm. No call. |
| `within spread` | The arms' ranges overlap. No call, whatever the medians. |
| `<b-label> higher` / `<b-label> lower` | The ranges are disjoint. |

Whether "higher" is better depends on the metric. Thrust and g/W: higher is
better. Current, CPU load, jitter, spool time, desync counts and aborts: lower
is better.

**Strength of a call:** disjoint ranges means every B run beat every A run.
The exact two-sided permutation p-value of that outcome is 2 / C(nA+nB, nA):

| Runs per arm | p |
|---|---|
| 3 v 3 | 0.10 |
| 4 v 4 | 0.029 |
| 5 v 5 | 0.008 |

A claim made at 3 v 3 is therefore suggestive, not conclusive. Run 4 per arm
when the result will be published.

**Noise floor on this bench:** small differences will rarely separate.

- Efficiency: worst repeatability spread across gated points was about 9–10%
  on identical firmware (see `efficiency_sweep.yaml`).
- Other run-to-run spread seen on identical firmware: about 19% on worst
  time-to-run, about 5% on current, and BEMF-timeout sample counts between 36
  and 114.

Use `noise_floor_probe` 2–3 times on one arm to re-measure the floor after any
rig change.

## 7. Reporting

For each setup and profile, report:

- Both firmware SHAs and the settings page (the `settings_*.bin` sha256 from
  `hwci settings read`).
- Pack and prop.
- Run order.
- The `ab-summary` table, unedited.

Quote only rows with a verdict other than `within spread`, and say which
direction is better for each. Commit the summary tables, not the raw runs,
with any README change that cites them.

### Known differences the A/B measures, not controls

- Compiler and optimisation: GCC 10.3.1 `-O3` vs GCC 15 `-Os` with selective
  `-O3`.
- Zero-cross filter: upstream rejects on any single disagreeing sample; ARK32
  tolerates up to 25%.
- ARK32-only mechanisms, for example blind steps, the transient governor,
  voltage-compensated ramp and demag-late detection, and upstream behaviours
  ARK32 changed (for example, the poll-mode fallback). These are the point of
  the comparison.
