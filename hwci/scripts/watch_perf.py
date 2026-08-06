#!/usr/bin/env python3
"""Passive hwci_perf logger - watch the ESC over SWD while a HUMAN drives it.

This script has NO throttle authority. It opens OpenOCD, resolves hwci_perf
from the ELF and polls it; it never arms the stand, never commands an output,
and never resets the target. That is the whole point: the grinding wrong-phase
state on this article is provoked by hard step inputs, and the safe way to
capture it is for the operator to drive the throttle by hand while the host
only observes.

    hwci/.venv/bin/python hwci/scripts/watch_perf.py --config hwci/rig.yaml \
        --out /tmp/grind.csv

Prints a line whenever something interesting changes and writes every sample
to CSV. Ctrl-C to stop.

The columns are chosen for the wrong-phase grind specifically:

  ci            commutation_interval, 0.5us units. In the grind this collapses
                (19-38 was measured at near-zero rpm), so watch it against rpm.
  rpm           mechanical, derived from the firmware's own e_rpm - CLOSED LOOP
                ONLY, it is meaningless otherwise (see runner.esc_perf_rpm).
  zc            zero_crosses. Going BACKWARDS = the loop restarted.
  blind         zc_blind_steps, consecutive dead-reckoned commutations.
  reject        zc_confirm_reject, edges the ZC filter threw out. A healthy
                loop rejects a steady trickle; a wrong-phase lock accepts
                almost everything, so this going QUIET while ci collapses is
                the signature to look for.
  bemfTO        bemf_timeout_happened - the stall rail, and the counter that
                escalates to the genuine stuck-rotor stop.
  state         esc_state: 4 = OPEN_LOOP (poll ZC), 5 = CLOSED_LOOP,
                7 = FAULT_STUCK.
"""
from __future__ import annotations

import argparse
import csv
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from hwci.config import load_rig                      # noqa: E402
from hwci.debugger.openocd import OpenOcdDebugger      # noqa: E402
from hwci.perf_reader import PerfReader                # noqa: E402

COLS = ("t", "ci", "rpm", "zc", "blind", "reject", "jitmax", "bemfTO",
        "state", "running", "armed", "input", "duty")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", required=True)
    ap.add_argument("--out", default="perf_watch.csv")
    ap.add_argument("--hz", type=float, default=50.0)
    args = ap.parse_args()

    rig = load_rig(args.config)
    elf = rig.resolved_elf()
    if elf is None:
        print("error: no ELF for target; build first", file=sys.stderr)
        return 2

    dbg = OpenOcdDebugger(rig.openocd_configs, openocd_bin=rig.openocd_bin,
                          search_dirs=rig.openocd_search_dirs).open()
    reader = PerfReader(dbg, str(elf))
    print(f"watching hwci_perf @ 0x{reader.address:08x} via {elf.name}")
    print("PASSIVE - no throttle is commanded. Drive by hand. Ctrl-C to stop.\n")

    period = 1.0 / args.hz
    t0 = time.monotonic()
    prev = {}
    n = 0
    with open(args.out, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=COLS)
        w.writeheader()
        print("%8s %6s %8s %7s %6s %7s %6s %6s %5s %6s" %
              ("t", "ci", "rpm", "zc", "blind", "reject", "jitmax",
               "bemfTO", "st", "duty"))
        try:
            while True:
                loop_start = time.monotonic()
                try:
                    pf = reader.read()
                except Exception as e:
                    if n == 0:
                        print(f"perf read failed: {e!r}", file=sys.stderr)
                    time.sleep(period)
                    continue
                raw = getattr(pf, "raw", None) or {}
                state = int(raw.get("esc_state") or 0)
                ci = int(raw.get("commutation_interval") or 0)
                zc = int(raw.get("zero_cross_count") or 0)
                rpm = 0.0
                if state == 5 and rig.pole_pairs:
                    try:
                        rpm = float(pf.e_rpm) / rig.pole_pairs
                    except Exception:
                        rpm = 0.0
                row = {
                    "t": round(time.monotonic() - t0, 4),
                    "ci": ci,
                    "rpm": round(rpm, 1),
                    "zc": zc,
                    "blind": int(raw.get("zc_blind_steps") or 0),
                    "reject": int(raw.get("zc_confirm_reject") or 0),
                    "jitmax": int(raw.get("zc_jitter_max") or 0),
                    "bemfTO": int(raw.get("bemf_timeout") or 0),
                    "state": state,
                    "running": int(raw.get("running") or 0),
                    "armed": int(raw.get("armed") or 0),
                    "input": int(raw.get("input") or 0),
                    "duty": int(raw.get("duty_cycle") or 0),
                }
                w.writerow(row)
                n += 1
                if n % 20 == 0:
                    fh.flush()
                # Print on anything worth seeing: a restart, a blind burst, a
                # collapsing interval while barely turning, or a stall trip.
                interesting = (
                    prev.get("zc", 0) - row["zc"] > 2
                    or row["blind"] > 0
                    or row["bemfTO"] != prev.get("bemfTO", row["bemfTO"])
                    or (row["state"] == 5 and 0 < ci < 200)
                    or row["state"] != prev.get("state", row["state"])
                )
                if interesting or n % 25 == 0:
                    print("%8.3f %6d %8.0f %7d %6d %7d %6d %6d %5d %6d" %
                          (row["t"], ci, rpm, row["zc"], row["blind"],
                           row["reject"], row["jitmax"], row["bemfTO"],
                           state, row["duty"]))
                prev = row
                delay = period - (time.monotonic() - loop_start)
                if delay > 0:
                    time.sleep(delay)
        except KeyboardInterrupt:
            print(f"\nstopped; {n} samples -> {args.out}")
        finally:
            try:
                dbg.close()
            except Exception:
                pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
