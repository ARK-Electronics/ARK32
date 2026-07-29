#!/usr/bin/env python3
"""Force a too-low BEMF-headroom slope on a live ARK F051 (PR #65 HWCI).

Finds ``gov_slope_q10`` / ``gov_conf`` in the ELF and pokes them over SWD
without halting the core, so a running seed hold can be turned into a
ceiling-bind. Pair with profile ``pr65_gov_unlatch_900kv``:

  # terminal A — profile (seed + step)
  hwci run --config rig.yaml --profile pr65_gov_unlatch_900kv \\
      --battery-cells 6 --out runs/pr65-gov-unlatch

  # terminal B — ~4 s into seed25 (after arm_settle + ramp + ~1 s hold)
  python3 scripts/gov_force_low_slope.py --elf ../obj/AM32_ARK_4IN1_F051_*.elf \\
      --slope 8 --conf 300

Default slope 8 is well below a healthy 900KV/10\" Q10 observation and binds
the ceiling under mid stick; conf 300 is GOV_CONF_ARM.
"""
from __future__ import annotations

import argparse
import struct
import sys
import time
from pathlib import Path

# Allow `python3 scripts/...` from hwci/ without install.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from hwci.debugger.openocd import OpenOcdDebugger  # noqa: E402
from hwci import elf as elfmod  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--elf", required=True, help="HWCI_PERF ARK F051 ELF")
    ap.add_argument("--slope", type=int, default=8, help="gov_slope_q10 to force")
    ap.add_argument("--conf", type=int, default=300, help="gov_conf to force (arm)")
    ap.add_argument("--repeat", type=float, default=0.0,
                    help="if >0, re-poke every this many seconds (fight estimator)")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="with --repeat, total seconds to keep forcing")
    args = ap.parse_args()

    elf_path = str(Path(args.elf).expanduser().resolve())
    slope_sym = elfmod.find_symbol(elf_path, "gov_slope_q10")
    conf_sym = elfmod.find_symbol(elf_path, "gov_conf")
    print(f"gov_slope_q10 @ 0x{slope_sym.address:08x}")
    print(f"gov_conf      @ 0x{conf_sym.address:08x}")

    # Little-endian uint16 pair: if they are adjacent we can mww once, but
    # do two RMW-safe halfword pokes via 32-bit read/modify/write.
    dbg = OpenOcdDebugger()
    try:
        def poke_u16(addr: int, value: int) -> None:
            word_addr = addr & ~3
            raw = dbg.read_memory(word_addr, 4)
            w = int.from_bytes(raw, "little")
            shift = (addr & 3) * 8
            w = (w & ~(0xFFFF << shift)) | ((value & 0xFFFF) << shift)
            dbg.write_u32(word_addr, w)

        def force() -> None:
            poke_u16(slope_sym.address, args.slope)
            poke_u16(conf_sym.address, args.conf)
            # Read back
            s = int.from_bytes(dbg.read_memory(slope_sym.address, 2), "little")
            c = int.from_bytes(dbg.read_memory(conf_sym.address, 2), "little")
            print(f"poked slope={s} conf={c} @ {time.strftime('%H:%M:%S')}")

        force()
        if args.repeat > 0 and args.duration > 0:
            t0 = time.time()
            while time.time() - t0 < args.duration:
                time.sleep(args.repeat)
                force()
    finally:
        dbg.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
