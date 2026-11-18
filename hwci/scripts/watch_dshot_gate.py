#!/usr/bin/env python3
"""Read-only SWD capture that localises DShot frame loss on ARK G431 CAN.

WHY: signaltimeout climbing tells us frames are not being accepted, but not
WHERE they die. computeDshotDMA() zeroes signaltimeout the instant the frame
span gate passes (Src/dshot.c, before the CRC compare), so a climbing
signaltimeout proves the frame never cleared the SPAN GATE - CRC is not
involved and dshot_badcounts cannot see it. Two very different faults produce
that same symptom:

  (a) edges never arrived / arrived corrupted   -> wire or stand-side problem
  (b) edges arrived, transfer completed, but the
      measured span fell outside the learned
      +-0.78% window                            -> firmware gate too tight

This script separates them. It samples the cheap scalars fast, and when a gap
opens (or a bogus throttle appears) it grabs the raw input-capture DMA buffer
plus DMA1_Channel1->CNDTR and re-runs the firmware's own arithmetic on those
exact words:

  CNDTR == 32, buffer frozen  -> no edges at all           => (a) frames absent
  CNDTR stuck mid-count       -> edges stopped mid-frame   => (a) edge loss
  buffer moving, span outside
  [dshot_frametime_low, high] -> transfer completed,
                                 gate rejected it          => (b) gate

Everything here is a memory read. No throttle is commanded, nothing is
halted, and no firmware is flashed - safe to run against an armed, idling
bench exactly as watch_gate_blip.py is.

Addresses are resolved from the ELF at startup (watch_gate_blip.py hardcodes
them; that drifts silently across rebuilds).

Usage:
  python3 hwci/scripts/watch_dshot_gate.py --duration 600 --elf obj/AM32_ARK_G431_CAN_3.0.1-ark.elf
"""
from __future__ import annotations

import argparse
import struct
import sys
import time
from pathlib import Path

_HWCI = Path(__file__).resolve().parents[1]
if str(_HWCI) not in sys.path:
	sys.path.insert(0, str(_HWCI))

from hwci.debugger.openocd import G4_APP_LOAD_ADDR, G4_CONFIGS, OpenOcdDebugger
from hwci.elf import find_symbol

# STM32G4: PERIPH_BASE 0x40000000 + AHB1 0x20000; channel 1 regs at +0x08,
# CNDTR at +0x04 within the channel block (stm32g431xx.h DMA1_Channel1_BASE).
DMA1_CHANNEL1_BASE = 0x40020008
DMA1_C1_CNDTR = DMA1_CHANNEL1_BASE + 0x04

DSHOT_BUFFERSIZE = 32  # edges per frame capture (16 bits, both edges)

# Symbols read every sample (cheap) and only on trigger (dma_buffer).
FAST_SYMS = (
	"zero_input_count",
	"signaltimeout",
	"send_telemetry",
	"newinput",
	"adjusted_input",
	"running",
	"armed",
	"dshot_goodcounts",
	"dshot_badcounts",
	"dshot_idle_exit_blocked",
	"dshot_frametime_low",
)
# NOT static: every one of these is written by detectInput()/checkDshot() and
# the learning block in transfercomplete(), all of which run AFTER attach.
# Sampling them once at t=0 reads post-boot defaults (dshot=0, inputSet=0,
# ic_psc=CPU/6, frametime_high=50000) and makes the gate window look wide
# open, which is exactly the bug the first version of this script had.
SLOW_SYMS = (
	"dshot_frametime_high",
	"average_count",
	"average_packet_length",
	"ic_timer_prescaler",
	"buffersize",
	"smallestnumber",
	"inputSet",
	"dshot",
)
WIDTH = {
	"zero_input_count": 2,
	"signaltimeout": 2,
	"send_telemetry": 1,
	"newinput": 2,
	"adjusted_input": 2,
	"running": 1,
	"armed": 1,
	"dshot_goodcounts": 2,
	"dshot_badcounts": 2,
	"dshot_frametime_low": 2,
	"dshot_idle_exit_blocked": 2,
	"dshot_frametime_high": 2,
	"average_count": 1,
	"average_packet_length": 4,
	"ic_timer_prescaler": 1,
	"buffersize": 1,
	"smallestnumber": 2,
	"inputSet": 1,
	"dshot": 1,
}


def _u(raw: bytes) -> int:
	if len(raw) == 1:
		return raw[0]
	if len(raw) == 2:
		return struct.unpack("<H", raw)[0]
	return struct.unpack("<I", raw[:4])[0]


def decode_frame(buf: list[int]) -> dict[str, int | bool]:
	"""Mirror computeDshotDMA()'s arithmetic exactly (Src/dshot.c:69-108)."""
	span = (buf[31] - buf[0]) & 0xFFFF
	half = span >> 5
	frame = 0
	for i in range(16):
		pdiff = (buf[(i << 1) + 1] - buf[i << 1]) & 0xFFFF
		frame = (frame << 1) | (1 if pdiff > half else 0)
	calc_crc = ((frame >> 4) ^ (frame >> 8) ^ (frame >> 12)) & 0xF
	return {
		"span": span,
		"half": half,
		"frame": frame,
		"crc_ok": calc_crc == (frame & 0xF),
		"tocheck": frame >> 5,
		"telem_req": bool(frame & 0x10),
	}


class Watcher:
	def __init__(self, dbg: OpenOcdDebugger, elf: str) -> None:
		self.dbg = dbg
		self.addr = {}
		for name in FAST_SYMS + SLOW_SYMS + ("dma_buffer",):
			self.addr[name] = find_symbol(elf, name).address

	def slow(self) -> dict[str, int]:
		return {n: _u(self.dbg.read_memory(self.addr[n], WIDTH[n])) for n in SLOW_SYMS}

	def fast(self) -> dict[str, int]:
		out = {n: _u(self.dbg.read_memory(self.addr[n], WIDTH[n])) for n in FAST_SYMS}
		out["cndtr"] = _u(self.dbg.read_memory(DMA1_C1_CNDTR, 4))
		return out

	def dma_snapshot(self) -> list[int]:
		raw = self.dbg.read_memory(self.addr["dma_buffer"], DSHOT_BUFFERSIZE * 4)
		return list(struct.unpack("<%dI" % DSHOT_BUFFERSIZE, raw))


def main() -> int:
	ap = argparse.ArgumentParser(description=__doc__)
	ap.add_argument("--duration", type=float, default=600.0)
	ap.add_argument("--hz", type=float, default=120.0)
	ap.add_argument(
		"--elf",
		default="obj/AM32_ARK_G431_CAN_3.0.1-ark.elf",
		help="ELF to resolve symbols from",
	)
	ap.add_argument(
		"--gap-ticks",
		type=int,
		default=60,
		help="signaltimeout (50us units) above which we grab the DMA buffer; "
		"60 = ~3 ms = more than two frame periods at 800 Hz",
	)
	ap.add_argument(
		"--settle",
		type=float,
		default=4.0,
		help="seconds to wait before trusting the detection/window globals",
	)
	ap.add_argument("--log", type=Path, default=Path("hwci/runs/dshot_gate_watch.log"))
	args = ap.parse_args()

	args.log.parent.mkdir(parents=True, exist_ok=True)
	period = 1.0 / max(args.hz, 1.0)

	dbg = OpenOcdDebugger(
		configs=list(G4_CONFIGS),
		app_load_addr=G4_APP_LOAD_ADDR,
		adapter_speed_khz=4000,
	)
	print(f"opening SWD (G4)… elf={args.elf} log={args.log}")
	dbg.open()
	try:
		w = Watcher(dbg, args.elf)
		# Attach itself can re-enter the app (ensure_app_running rewrites
		# pc/msp when the BL is stuck, which re-runs startup and zeroes BSS),
		# so sample the detection state twice: immediately, then after the
		# input-detect + 8-frame learning window has had time to complete.
		# If these two differ, the app (re)started under us at attach.
		at_attach = w.slow()
		time.sleep(args.settle)
		settled = w.slow()
		hdr = (
			f"# start {time.strftime('%Y-%m-%d %H:%M:%S')} duration={args.duration} "
			f"hz={args.hz} gap_ticks={args.gap_ticks} settle={args.settle}\n"
			f"# at_attach: {at_attach}\n"
			f"# settled  : {settled}\n"
			f"# restarted_at_attach={at_attach != settled}\n"
		)
		print(hdr, end="", flush=True)

		t0 = time.monotonic()
		n = 0
		triggers = 0
		prev_blocked: int | None = None
		verdicts: dict[str, int] = {}
		spans: dict[int, int] = {}
		with args.log.open("a", buffering=1) as log:
			log.write(hdr)
			while time.monotonic() - t0 < args.duration:
				now = time.monotonic()
				try:
					s = w.fast()
				except Exception as e:  # noqa: BLE001 - probe hiccup, keep going
					log.write(f"{now - t0:8.3f} READ_ERR {e}\n")
					time.sleep(0.05)
					continue
				n += 1
				# Once the idle-exit holdoff is in the firmware, newinput no
				# longer reaches 2047 on this fault - the guard absorbs it. So
				# trigger on the guard firing too, otherwise the very event we
				# are hunting becomes invisible the moment it is fixed.
				blocked = s["dshot_idle_exit_blocked"]
				guard_fired = prev_blocked is not None and blocked != prev_blocked
				prev_blocked = blocked
				hot = (
					s["signaltimeout"] > args.gap_ticks
					or s["newinput"] > 47
					or guard_fired
				)
				if hot:
					triggers += 1
					b1 = w.dma_snapshot()
					c1 = _u(dbg.read_memory(DMA1_C1_CNDTR, 4))
					b2 = w.dma_snapshot()
					sl_now = w.slow()  # window is learned at runtime; re-read it
					d = decode_frame(b1)
					lo = s["dshot_frametime_low"]
					hi = sl_now["dshot_frametime_high"]
					in_win = lo < d["span"] < hi
					moving = b1 != b2
					if not moving and s["cndtr"] == DSHOT_BUFFERSIZE:
						verdict = "NO_EDGES"
					elif not moving and 0 < s["cndtr"] < DSHOT_BUFFERSIZE:
						verdict = "EDGES_STALLED_MIDFRAME"
					elif in_win:
						verdict = "GATE_PASS"
					else:
						verdict = "GATE_REJECT_SPAN"
					verdicts[verdict] = verdicts.get(verdict, 0) + 1
					spans[d["span"]] = spans.get(d["span"], 0) + 1
					line = (
						f"{now - t0:8.3f} {verdict:22s} sigto={s['signaltimeout']:5d} "
						f"cndtr={s['cndtr']:2d}/{c1:2d} moving={int(moving)} "
						f"span={d['span']:5d} win=[{lo},{hi}] in_win={int(in_win)} "
						f"frame=0x{d['frame']:04X} crc={int(d['crc_ok'])} "
						f"tocheck={d['tocheck']:4d} telem={int(d['telem_req'])} "
						f"new={s['newinput']:4d} adj={s['adjusted_input']:4d} "
						f"run={s['running']} armed={s['armed']} "
						f"good={s['dshot_goodcounts']} bad={s['dshot_badcounts']} "
						f"blocked={s['dshot_idle_exit_blocked']} "
						f"dshot={sl_now['dshot']} inSet={sl_now['inputSet']} "
						f"psc={sl_now['ic_timer_prescaler']}\n"
					)
					log.write(line)
					if guard_fired:
						verdicts["IDLE_EXIT_BLOCKED"] = verdicts.get("IDLE_EXIT_BLOCKED", 0) + 1
					if s["newinput"] > 47 or guard_fired:
						print(line, end="", flush=True)
				sl = period - (time.monotonic() - now)
				if sl > 0:
					time.sleep(sl)

			top = sorted(spans.items(), key=lambda kv: -kv[1])[:12]
			summary = (
				f"# done samples={n} triggers={triggers} "
				f"elapsed={time.monotonic() - t0:.1f}s\n"
				f"# verdicts: {verdicts}\n"
				f"# top spans (span:count): {top}\n"
			)
			print(summary, end="", flush=True)
			log.write(summary)
	finally:
		try:
			dbg.close()
		except Exception:  # noqa: BLE001
			pass
	return 0


if __name__ == "__main__":
	raise SystemExit(main())
