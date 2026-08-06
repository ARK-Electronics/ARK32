#!/usr/bin/env python3
"""Dump the firmware's per-commutation ZC trace over SWD.

PASSIVE. Opens OpenOCD, resolves ``hwci_zc_trace`` from the ELF and reads it.
It never arms the stand, never commands throttle, never resets the target.

The ring free-runs while armed and the FIRMWARE freezes it on the entry
signature of the false lock - an accepted crossing whose interval is less than
half the tracked estimate - then records HWCI_ZC_TRACE_POST more events. So a
frozen buffer holds the commutations either side of entry, which is what a
host-side freeze cannot catch (a whole poll period of latency, by which time
the lock is established).

Usage, with the motor being driven BY HAND:

    # poll until the firmware trips the trigger, then print and re-arm
    hwci/.venv/bin/python hwci/scripts/dump_zc_trace.py --config hwci/rig.yaml --watch

    # one-shot read of whatever is in the buffer right now
    hwci/.venv/bin/python hwci/scripts/dump_zc_trace.py --config hwci/rig.yaml

``ci`` in the output is RECONSTRUCTED, not stored: the firmware logs only the
measured interval per event (4 bytes/event was what the F051 RAM budget
allowed), and the estimate is replayed here through the same IIR the firmware
uses in PeriodElapsedCallback,

    ci <- (ci + (last + this) / 2) / 2

anchored on ci_at_arm and advanced ONLY on accepted crossings - blind steps do
not update the estimate (see bemf_zc.c). Treat it as a close reconstruction,
not ground truth.
"""
from __future__ import annotations

import argparse
import os
import struct
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from hwci import elf as elfmod                        # noqa: E402
from hwci.config import load_rig                      # noqa: E402
from hwci.debugger.openocd import OpenOcdDebugger      # noqa: E402

SYMBOL = "hwci_zc_trace"
HDR = "<HBBIBBH"          # write_idx, frozen, post, total, wrapped, _pad, ci_at_arm
HDR_SIZE = struct.calcsize(HDR)
EV = "<HBB"               # interval, kind, duty8
EV_SIZE = struct.calcsize(EV)

KINDS = {0: "-", 1: "ACCEPT", 2: "REJ_MININT", 3: "REJ_CONFIRM",
         4: "BLIND", 5: "BUDGET"}
# Mirrors HWCI_CMD_ARM_ZC_TRACE / hwci_perf.host_cmd.
ARM_CMD = 0x5A
# Mirrors HWCI_PERF_MAGIC in Inc/hwci_perf.h. Lives in .data, so it is only
# present once the firmware's startup has run - which is exactly what makes it
# a usable "is this image actually running?" probe.
HWCI_PERF_MAGIC = 0x31435748


def read_trace(dbg, addr: int, size: int):
    raw = dbg.read_memory(addr, size)
    write_idx, frozen, post, total, wrapped, _pad, ci_at_arm = \
        struct.unpack_from(HDR, raw, 0)
    n = (size - HDR_SIZE) // EV_SIZE
    evs = [struct.unpack_from(EV, raw, HDR_SIZE + i * EV_SIZE)
           for i in range(n)]
    return dict(write_idx=write_idx, frozen=frozen, post=post, total=total,
                wrapped=wrapped, ci_at_arm=ci_at_arm, n=n, evs=evs)


def ordered(tr):
    """Oldest-first. Once wrapped, the oldest entry is at write_idx."""
    evs, n, w = tr["evs"], tr["n"], tr["write_idx"]
    if not tr["wrapped"]:
        return evs[:w]
    return evs[w:] + evs[:w]


def show(tr, out=None) -> None:
    lines: list[str] = []

    def emit(line: str = "") -> None:
        print(line)
        lines.append(line)

    evs = ordered(tr)
    emit(f"\nframes={len(evs)} total={tr['total']} frozen={tr['frozen']} "
         f"ci_at_arm={tr['ci_at_arm']}")
    if not evs:
        emit("  (empty - nothing recorded yet)")
        _flush(lines, out)
        return
    ci = float(tr["ci_at_arm"]) or float(evs[0][0])
    last = this = float(evs[0][0])
    emit(f"{'#':>3} {'kind':<12} {'interval':>9} {'ci(recon)':>10} "
         f"{'ratio':>7} {'duty':>6}")
    for i, (interval, kind, duty8) in enumerate(evs):
        ratio = (interval / ci) if ci else 0.0
        flag = ""
        if kind == 1 and ratio < 0.5:
            flag = "  <-- TRIGGER (accepted at < ci/2)"
        elif kind == 2:
            flag = "  (gate rejected)"
        emit(f"{i:>3} {KINDS.get(kind, '?'):<12} {interval:>9} {ci:>10.0f} "
             f"{ratio:>7.2f} {duty8 * 8:>6}{flag}")
        # Replay the firmware's estimate: accepted crossings only.
        if kind == 1:
            last, this = this, float(interval)
            ci = (ci + (last + this) / 2.0) / 2.0
    _flush(lines, out)


def _flush(lines, out) -> None:
    if not out:
        return
    with open(out, "a") as fh:
        fh.write("\n".join(lines) + "\n")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", required=True)
    ap.add_argument("--watch", action="store_true",
                    help="poll until frozen, print, re-arm, repeat")
    ap.add_argument("--arm", action="store_true",
                    help="just re-arm the trace and exit")
    ap.add_argument("--live", type=float, metavar="SECONDS", default=None,
                    help="stay attached and dump the ring every SECONDS "
                         "whether or not it froze. This is the mode for "
                         "characterising HEALTHY running: --watch only ever "
                         "prints on a trigger, and the arm/drive/read flow "
                         "needs the session dropped while you drive.")
    ap.add_argument("--out", default="zc_trace.log",
                    help="append every capture here as well as printing it; "
                         "--watch re-arms after each freeze, so without this "
                         "a capture exists only in the terminal scrollback")
    args = ap.parse_args()

    rig = load_rig(args.config)
    elf = rig.resolved_elf()
    if elf is None:
        print("error: no ELF for target; build first", file=sys.stderr)
        return 2
    sym = elfmod.find_symbol(str(elf), SYMBOL)
    if not sym.size:
        print(f"error: {SYMBOL} has no size in {elf}; is this an "
              "HWCI_PERF=1 build?", file=sys.stderr)
        return 2

    dbg = OpenOcdDebugger(rig.openocd_configs, openocd_bin=rig.openocd_bin,
                          search_dirs=rig.openocd_search_dirs).open()
    try:
        perf = elfmod.find_symbol(str(elf), "hwci_perf")
        # Health check BEFORE anything else. Reading a trace out of a target
        # that is not running THIS image yields plausible-looking garbage -
        # a stale buffer, or another build's RAM at the same address - and
        # there is no way to tell from the event data alone. A whole bench
        # session was once logged as 101 "captures" that were one stale
        # buffer reprinted, because this check did not exist.
        magic, version, ssize = struct.unpack_from(
            "<IHH", dbg.read_memory(perf.address, 8), 0)
        if magic != HWCI_PERF_MAGIC:
            print(f"error: hwci_perf.magic = 0x{magic:08X}, expected "
                  f"0x{HWCI_PERF_MAGIC:08X}.\n"
                  "The target is not running this firmware. Either it is "
                  "unpowered / held in reset, or the flashed image is not\n"
                  f"  {elf}\n"
                  "Flash it first:\n"
                  f"  hwci flash --config {args.config} --bin "
                  f"{str(elf).replace('.elf', '.bin')}",
                  file=sys.stderr)
            return 2
        print(f"target ok: hwci_perf v{version}, {ssize} bytes")
        # host_cmd sits at offset 60 in hwci_perf and is frozen there across
        # every struct version, precisely so an A/B session can command any
        # vintage without a layout lookup (see Inc/hwci_perf.h).
        cmd_addr = perf.address + 60

        def rearm(verify: bool = True) -> bool:
            """Re-arm and confirm the firmware consumed it.

            The command is serviced from the main loop, so a target that is
            wedged - or whose host_cmd offset does not match this ELF - will
            silently never clear it. Without this check --watch spins on the
            still-frozen buffer and reprints it every poll.
            """
            dbg.write_u32(cmd_addr, ARM_CMD)
            if not verify:
                return True
            for _ in range(40):
                time.sleep(0.05)
                if not read_trace(dbg, sym.address, sym.size)["frozen"]:
                    return True
            print("error: re-arm was not consumed within 2 s - the trace is "
                  "still frozen.\nThe firmware services host_cmd from the "
                  "main loop, so this means the target is wedged or is not "
                  "running this image. Not reprinting the stale buffer.",
                  file=sys.stderr)
            return False

        print(f"{SYMBOL} @ 0x{sym.address:08x} ({sym.size} bytes)")
        if args.arm:
            rearm()
            print("re-armed")
            return 0

        if args.live is not None:
            print(f"PASSIVE - drive by hand. Dumping every {args.live}s. "
                  "Ctrl-C to stop.")
            rearm(verify=False)
            n = 0
            try:
                while True:
                    time.sleep(args.live)
                    tr = read_trace(dbg, sym.address, sym.size)
                    n += 1
                    print(f"\n=== dump {n} "
                          f"({'FROZEN - detector armed' if tr['frozen'] else 'live'}) ===")
                    show(tr, args.out)
                    # Always re-arm: frozen or not, the next dump should be a
                    # fresh window rather than the same wrapped ring.
                    rearm(verify=False)
            except KeyboardInterrupt:
                print(f"\nstopped after {n} dumps")
            return 0

        if not args.watch:
            show(read_trace(dbg, sym.address, sym.size), args.out)
            return 0

        print("PASSIVE - drive by hand. Waiting for the firmware to trip the "
              "trigger. Ctrl-C to stop.")
        if not rearm():
            return 2
        n = 0
        try:
            while True:
                tr = read_trace(dbg, sym.address, sym.size)
                if tr["frozen"]:
                    n += 1
                    print(f"\n=== capture {n} ===")
                    show(tr, args.out)
                    print("re-arming...")
                    if not rearm():
                        return 2
                time.sleep(0.05)
        except KeyboardInterrupt:
            print("\nstopped")
        return 0
    finally:
        try:
            dbg.close()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
