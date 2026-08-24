#!/usr/bin/env python3
"""PR #65 HWCI: unlatch profile + SWD force of low slope mid-seed.

Attaches to the hwci run's existing OpenOCD Tcl-RPC (port 6666) so it does
not fight the ST-Link. Timing: poke starts after arm_settle+ramp+seed head.

  cd hwci
  python3 scripts/run_gov_unlatch_hwci.py \\
      --elf ../obj/ARK32_ARK_4IN1_F051_3.0.3.elf \\
      --out runs/pr65-gov-unlatch-$(date +%Y%m%d_%H%M%S)
"""
from __future__ import annotations

import argparse
import json
import socket
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from hwci import elf as elfmod  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
_RPC_SEP = b"\x1a"


class TclRpc:
    """Minimal client for an already-running openocd Tcl-RPC server."""

    def __init__(self, port: int = 6666, timeout: float = 10.0):
        self.sock = socket.create_connection(("127.0.0.1", port), timeout=timeout)
        self.sock.settimeout(3.0)
        self._lock = threading.Lock()

    def rpc(self, cmd: str) -> str:
        with self._lock:
            self.sock.sendall(cmd.encode() + _RPC_SEP)
            buf = bytearray()
            while _RPC_SEP not in buf:
                chunk = self.sock.recv(4096)
                if not chunk:
                    raise RuntimeError("openocd RPC closed")
                buf.extend(chunk)
            return buf.split(_RPC_SEP, 1)[0].decode(errors="replace")

    def read_u16(self, addr: int) -> int:
        out = self.rpc(f"read_memory 0x{addr:08x} 16 1")
        toks = out.replace("{", " ").replace("}", " ").split()
        return int(toks[0], 0) & 0xFFFF

    def write_u16(self, addr: int, value: int) -> None:
        # RMW 32-bit word so adjacent halfword is preserved.
        word = addr & ~3
        out = self.rpc(f"read_memory 0x{word:08x} 32 1")
        toks = out.replace("{", " ").replace("}", " ").split()
        w = int(toks[0], 0) & 0xFFFFFFFF
        shift = (addr & 3) * 8
        w = (w & ~(0xFFFF << shift)) | ((value & 0xFFFF) << shift)
        err = self.rpc(f"mww 0x{word:08x} 0x{w:08x}")
        if err.strip():
            raise RuntimeError(f"mww failed: {err!r}")

    def close(self) -> None:
        try:
            self.sock.close()
        except OSError:
            pass


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default="rig.yaml")
    ap.add_argument("--elf", required=True)
    ap.add_argument("--eeprom", default="config/eeprom_900kv_10inch_best.bin")
    ap.add_argument("--out", required=True)
    ap.add_argument("--slope", type=int, default=48)
    ap.add_argument("--conf", type=int, default=300)
    ap.add_argument("--battery-cells", type=int, default=6)
    ap.add_argument("--poke-delay-s", type=float, default=9.0)
    ap.add_argument("--poke-repeat-s", type=float, default=0.12)
    ap.add_argument("--poke-duration-s", type=float, default=3.5)
    ap.add_argument("--tcl-port", type=int, default=6666)
    args = ap.parse_args()

    out = Path(args.out)
    if not out.is_absolute():
        out = ROOT / out
    out.mkdir(parents=True, exist_ok=True)
    elf_path = str(Path(args.elf).resolve())
    bin_path = Path(elf_path).with_suffix(".bin")
    addrs = {
        "slope": elfmod.find_symbol(elf_path, "gov_slope_q10").address,
        "conf": elfmod.find_symbol(elf_path, "gov_conf").address,
        "ceil": elfmod.find_symbol(elf_path, "gov_duty_ceiling").address,
        "stuck": elfmod.find_symbol(elf_path, "gov_stuck_ms").address,
        "rel": elfmod.find_symbol(elf_path, "gov_release_ceil").address,
    }
    (out / "symbols.json").write_text(json.dumps(
        {k: f"0x{v:08x}" for k, v in addrs.items()}, indent=2) + "\n")

    stats = {"pokes": 0, "max_stuck": 0, "saw_conf0": False, "saw_release": False,
             "min_ceil": 9999, "max_ceil": 0, "rpc_errors": 0}
    logf = out / "force_poke.log"
    stop = threading.Event()

    def poker() -> None:
        time.sleep(args.poke_delay_s)
        # Wait until openocd from hwci run is listening
        rpc = None
        for _ in range(50):
            if stop.is_set():
                return
            try:
                rpc = TclRpc(args.tcl_port)
                break
            except OSError:
                time.sleep(0.2)
        if rpc is None:
            logf.write_text("could not attach to openocd Tcl-RPC\n")
            return
        try:
            t0 = time.time()
            with logf.open("a") as f:
                f.write(f"force start slope={args.slope} conf={args.conf}\n")
            while not stop.is_set() and time.time() - t0 < args.poke_duration_s:
                try:
                    rpc.write_u16(addrs["slope"], args.slope)
                    rpc.write_u16(addrs["conf"], args.conf)
                    cl = rpc.read_u16(addrs["ceil"])
                    if cl > 900:
                        rpc.write_u16(addrs["ceil"], 900)
                    stats["pokes"] += 1
                    st = rpc.read_u16(addrs["stuck"])
                    cf = rpc.read_u16(addrs["conf"])
                    cl = rpc.read_u16(addrs["ceil"])
                    rel = rpc.read_u16(addrs["rel"])
                    stats["max_stuck"] = max(stats["max_stuck"], st)
                    stats["min_ceil"] = min(stats["min_ceil"], cl)
                    stats["max_ceil"] = max(stats["max_ceil"], cl)
                    if rel:
                        stats["saw_release"] = True
                    if cf < 300:
                        stats["saw_conf0"] = True
                    with logf.open("a") as f:
                        f.write(f"poke conf={cf} stuck={st} ceil={cl} rel={rel}\n")
                except Exception as e:
                    stats["rpc_errors"] += 1
                    with logf.open("a") as f:
                        f.write(f"poke err: {e}\n")
                time.sleep(args.poke_repeat_s)
            t1 = time.time()
            while not stop.is_set() and time.time() - t1 < 5.0:
                try:
                    st = rpc.read_u16(addrs["stuck"])
                    cf = rpc.read_u16(addrs["conf"])
                    cl = rpc.read_u16(addrs["ceil"])
                    rel = rpc.read_u16(addrs["rel"])
                    stats["max_stuck"] = max(stats["max_stuck"], st)
                    stats["min_ceil"] = min(stats["min_ceil"], cl)
                    stats["max_ceil"] = max(stats["max_ceil"], cl)
                    if rel:
                        stats["saw_release"] = True
                    if cf < 300:
                        stats["saw_conf0"] = True
                    with logf.open("a") as f:
                        f.write(f"watch conf={cf} stuck={st} ceil={cl} rel={rel}\n")
                except Exception as e:
                    stats["rpc_errors"] += 1
                    with logf.open("a") as f:
                        f.write(f"watch err: {e}\n")
                time.sleep(0.1)
        finally:
            rpc.close()

    print("flash…")
    subprocess.run([sys.executable, "-m", "hwci", "flash", "--config", args.config,
                    "--bin", str(bin_path)], cwd=str(ROOT), check=True)
    print("eeprom…")
    subprocess.run([sys.executable, "-m", "hwci", "settings", "write",
                    "--config", args.config, "--bin", args.eeprom],
                   cwd=str(ROOT), check=True)

    thr = threading.Thread(target=poker, daemon=True)
    thr.start()
    try:
        print("profile pr65_gov_unlatch_900kv…")
        rc = subprocess.run(
            [sys.executable, "-m", "hwci", "run", "--config", args.config,
             "--profile", "pr65_gov_unlatch_900kv",
             "--battery-cells", str(args.battery_cells),
             "--out", str(out / "suite")],
            cwd=str(ROOT)).returncode
    finally:
        stop.set()
        thr.join(timeout=20)

    (out / "force_stats.json").write_text(json.dumps(stats, indent=2) + "\n")
    subprocess.run([sys.executable, "-m", "hwci", "analyze", str(out / "suite")],
                   cwd=str(ROOT), check=False)
    engage = stats["pokes"] > 0 and (
        stats["saw_conf0"] or stats["saw_release"] or stats["max_stuck"] >= 400)
    (out / "README.txt").write_text(
        f"PR65 gov unlatch HWCI\nelf={elf_path}\nstats={stats}\n"
        f"run_rc={rc}\nengagement={'PASS' if engage else 'WEAK'}\n")
    print("STATS", stats)
    print("engagement", "PASS" if engage else "WEAK", "run_rc", rc)
    return 0 if rc == 0 else rc


if __name__ == "__main__":
    raise SystemExit(main())
