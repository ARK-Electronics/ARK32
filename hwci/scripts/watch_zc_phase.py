#!/usr/bin/env python3
"""Passive PWM-phase histogram of ACCEPTED zero-crossings.

PASSIVE. Opens OpenOCD, resolves hwci_perf from the ELF and polls it. It never
arms the stand, never commands throttle, never resets the target. Drive the
motor BY HAND while this watches.

WHAT THIS DECIDES
-----------------
The twelve ZC traces captured on the bench all enter the wrong-phase grind the
same way: an accepted crossing at ratio ~1.5, immediately followed by an
accepted crossing at ratio 0.22-0.48, the pair summing to ~2.0. Timing alone
cannot say which of those two edges is the impostor:

  (a) the 1.5 is spurious and the 0.45 is the real crossing arriving on
      schedule after one was missed  -> tightening the minimum-interval gate
      to ci/2 would reject the REAL edge and entrench the lock;
  (b) the 0.45 is spurious           -> a ci/2 gate kills the grind outright.

TIM1 on the F051 is edge-aligned (LL_TIM_COUNTERMODE_UP) in PWM1 mode, so
TIM1->CNT maps monotonically onto the PWM period and the phase histogram is a
direct test. Switching transients are locked to the PWM edges:

    turn-ON  -> bin 0
    turn-OFF -> bin ~ HWCI_ZC_PHASE_BINS * duty_cycle / 2000

A real back-EMF crossing is asynchronous to the PWM carrier, so its phase must
SMEAR. It will not be flat, though, and that is the interpretation trap: the
floating phase is only observable over part of the PWM period, so even a
perfectly healthy loop shows a broad hump spanning the sensing window (PR #23
measured 3.3x edge-window enrichment at t30). What distinguishes the two cases
is the SHAPE, not the mere presence of structure:

    healthy  -> broad hump, several bins wide, moving with duty
    noise    -> narrow spike pinned to bin 0 or to the turn-off bin

Hence: always take a baseline on a smooth ramp at comparable duty BEFORE
provoking the grind, and compare R1 and the bar shape between the two. A single
grind capture in isolation proves nothing.

The F051 has no hardware comparator blanking (that is a G0/G4 peripheral
feature), which is exactly why the turn-on transient can be seen here at all.

READING THE OUTPUT
------------------
  R1      circular resultant length of the delta distribution, 0..1.
          ~0 = uniform (asynchronous, healthy), ->1 = pinned to one phase.
          Bins are circular over the PWM period, so R1 is invariant to WHICH
          bin the peak lands in - it measures concentration, not location.
  peak    enrichment of the fullest bin over uniform (1.0 = uniform).
  bin     index of the fullest bin, with (on)/(off) annotated when it lands on
          a predicted switching bin.
  bar     the 32 bins of the window delta, scaled to the window max.
  arr*    marks a window in which tim1_arr moved: variable_pwm rescaled the
          bins mid-window, so that line's distribution is smeared. Ignore it.

Counts are only binned while zero_crosses >= 100 (same "stable running" gate
as the rest of the ZC instrumentation), so a desync that zeroes the crossing
count blanks the histogram for 100 crossings - watch the zc column.

Usage:

    hwci/.venv/bin/python hwci/scripts/watch_zc_phase.py --config hwci/rig.yaml

Take a baseline on a smooth ramp first, then provoke the grind with a step and
compare R1 across the two.
"""
from __future__ import annotations

import argparse
import cmath
import csv
import math
import os
import struct
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from hwci import elf as elfmod                        # noqa: E402
from hwci.config import load_rig                      # noqa: E402
from hwci.debugger.openocd import OpenOcdDebugger      # noqa: E402
from hwci.perf_reader import PerfReader                # noqa: E402

BINS = 32
# duty_cycle is 0..2000 in firmware terms; CCR = duty_cycle * tim1_arr / 2000,
# so the turn-off edge sits at this fraction of the period regardless of ARR.
DUTY_FULL_SCALE = 2000.0
BLOCKS = " ▁▂▃▄▅▆▇█"
COLS = ("t", "n", "R1", "peak", "peak_bin", "off_bin", "ci", "rpm", "zc",
        "duty", "state", "tim1_arr", "hist")


def resultant(hist) -> float:
    """Circular resultant length of a binned phase distribution, 0..1."""
    total = sum(hist)
    if total <= 0:
        return 0.0
    acc = sum(h * cmath.exp(2j * math.pi * k / BINS) for k, h in enumerate(hist))
    return abs(acc) / total


def bar(hist) -> str:
    top = max(hist) if hist else 0
    if top <= 0:
        return " " * BINS
    return "".join(BLOCKS[min(len(BLOCKS) - 1, (h * (len(BLOCKS) - 1) + top - 1) // top)]
                   for h in hist)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", required=True)
    ap.add_argument("--window", type=float, default=0.5,
                    help="seconds of histogram delta to accumulate per line")
    ap.add_argument("--hz", type=float, default=50.0)
    ap.add_argument("--min-count", type=int, default=20,
                    help="skip windows with fewer than this many binned "
                         "crossings; R1 is meaningless on a handful of samples")
    ap.add_argument("--out", default="zc_phase.csv")
    args = ap.parse_args()

    rig = load_rig(args.config)
    elf = rig.resolved_elf()
    if elf is None:
        print("error: no ELF for target; build first", file=sys.stderr)
        return 2

    dbg = OpenOcdDebugger(rig.openocd_configs, openocd_bin=rig.openocd_bin,
                          search_dirs=rig.openocd_search_dirs).open()
    try:
        reader = PerfReader(dbg, str(elf))
        # tim1_arr is not in hwci_perf; read the global directly so a window
        # spanning a variable_pwm rescale can be flagged rather than believed.
        try:
            arr_sym = elfmod.find_symbol(str(elf), "tim1_arr")
            arr_addr = arr_sym.address
        except Exception:
            arr_addr = None

        def read_arr() -> int:
            if arr_addr is None:
                return 0
            try:
                return struct.unpack_from("<H", dbg.read_memory(arr_addr, 2), 0)[0]
            except Exception:
                return 0

        print(f"watching hwci_perf @ 0x{reader.address:08x} via {elf.name}")
        print("PASSIVE - no throttle is commanded. Drive by hand. Ctrl-C to stop.")
        print("Baseline on a smooth ramp first, then step into the grind.\n")
        print(f"{'t':>7} {'n':>5} {'R1':>5} {'peak':>5} {'bin':>9} "
              f"{'ci':>6} {'rpm':>6} {'duty':>5} {'st':>2}  bars")

        period = 1.0 / args.hz
        t0 = time.monotonic()
        prev_hist = None
        acc = [0] * BINS
        win_start = t0
        arr_seen: set[int] = set()
        duty_seen: list[int] = []

        with open(args.out, "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=COLS)
            w.writeheader()
            try:
                while True:
                    loop_start = time.monotonic()
                    try:
                        pf = reader.read()
                    except Exception as e:
                        if prev_hist is None:
                            print(f"perf read failed: {e!r}", file=sys.stderr)
                        time.sleep(period)
                        continue
                    raw = getattr(pf, "raw", None) or {}
                    hist = list(raw.get("zc_phase_hist") or [])
                    if len(hist) != BINS:
                        print("error: firmware has no v4+ zc_phase_hist; "
                              "rebuild with HWCI_PERF=1", file=sys.stderr)
                        return 2
                    if prev_hist is not None:
                        for i in range(BINS):
                            acc[i] += (hist[i] - prev_hist[i]) & 0xFFFF
                    prev_hist = hist
                    duty_seen.append(int(raw.get("duty_cycle") or 0))
                    arr_seen.add(read_arr())

                    now = time.monotonic()
                    if now - win_start >= args.window:
                        n = sum(acc)
                        state = int(raw.get("esc_state") or 0)
                        ci = int(raw.get("commutation_interval") or 0)
                        zc = int(raw.get("zero_cross_count") or 0)
                        duty = round(sum(duty_seen) / len(duty_seen)) if duty_seen else 0
                        rpm = 0.0
                        if state == 5 and rig.pole_pairs:
                            try:
                                rpm = float(pf.e_rpm) / rig.pole_pairs
                            except Exception:
                                rpm = 0.0
                        arr = max(arr_seen) if arr_seen else 0
                        moved = len(arr_seen) > 1
                        off_bin = int(BINS * duty / DUTY_FULL_SCALE) % BINS
                        if n >= args.min_count:
                            r1 = resultant(acc)
                            pk = max(range(BINS), key=lambda i: acc[i])
                            enrich = acc[pk] / (n / BINS)
                            tag = ""
                            if pk == 0:
                                tag = "(on)"
                            elif abs(pk - off_bin) <= 1:
                                tag = "(off)"
                            print(f"{now - t0:>7.2f} {n:>5d} {r1:>5.2f} "
                                  f"{enrich:>5.1f} {pk:>4d}{tag:<5} "
                                  f"{ci:>6d} {rpm:>6.0f} {duty:>5d} {state:>2d}  "
                                  f"|{bar(acc)}|{'  arr*' if moved else ''}")
                            w.writerow({
                                "t": round(now - t0, 3), "n": n,
                                "R1": round(r1, 4), "peak": round(enrich, 2),
                                "peak_bin": pk, "off_bin": off_bin, "ci": ci,
                                "rpm": round(rpm, 1), "zc": zc, "duty": duty,
                                "state": state,
                                "tim1_arr": ("*" if moved else "") + str(arr),
                                "hist": ";".join(str(v) for v in acc),
                            })
                            fh.flush()
                        acc = [0] * BINS
                        win_start = now
                        arr_seen = set()
                        duty_seen = []

                    delay = period - (time.monotonic() - loop_start)
                    if delay > 0:
                        time.sleep(delay)
            except KeyboardInterrupt:
                print(f"\nstopped -> {args.out}")
    finally:
        try:
            dbg.close()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
