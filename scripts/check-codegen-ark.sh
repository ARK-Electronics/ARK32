#!/usr/bin/env bash
# Assert the codegen invariants LTO can silently break on the ARK F051.
#
# Both failures this guards against link cleanly, pass every functional test,
# and only show up as a slower motor or an unidentifiable image on the bench:
#
#   1. RAM residency. The comparator / control-loop / commutation paths are
#      RAM_FUNC so they run out of RAM instead of paying F051 flash wait states
#      in an ISR. LTO is free to inline a RAM_FUNC body into a flash-resident
#      caller, which drops the hot path back into flash with nothing to notice
#      it.
#
#      Policy (two tiers):
#        - Required symbols must exist and live in RAM (0x20000000..). These are
#          the F051 vector-table ISR entry points for the zero-cross comparator,
#          20 kHz control loop, and commutation timer. LTO may absorb callees
#          into them; the entries themselves must remain RAM-resident.
#        - Optional hot callees (other known RAM_FUNC names). If still present
#          as out-of-line symbols they must be in RAM — that is the silent
#          "thin RAM ISR + flash body" failure mode. If LTO fully absorbs them
#          into a RAM-resident caller, the symbol is gone and that is OK.
#
#   2. Section-placed data. .file_name and .app_signature are read out of the
#      image by external tooling and never from C, so LTO drops them as dead
#      unless they carry __attribute__((used)). The section stays in the ELF at
#      size 0, so only a size check catches it.
#
# Usage: scripts/check-codegen-ark.sh [--no-ramfunc] [elf]
#   defaults to the ARK F051 build. --no-ramfunc skips the RAM residency block
#   for targets where RAM_FUNC carries no section attribute (see Inc/targets.h:
#   only F051 places these in .ramfunc; elsewhere the macro is optimize("O3")
#   alone), leaving the section checks, which apply to every target.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

CHECK_RAMFUNC=1
if [ "${1:-}" = "--no-ramfunc" ]; then
  CHECK_RAMFUNC=0
  shift
fi

ELF="${1:-}"
if [ -z "$ELF" ]; then
  ELF=$(ls -1 obj/ARK32_ARK_4IN1_F051_*.elf 2>/dev/null | head -1 || true)
fi
if [ -z "$ELF" ] || [ ! -f "$ELF" ]; then
  echo "error: no ARK F051 elf — build ARK_4IN1_F051 first" >&2
  exit 2
fi

# Pinned xPack binutils, same policy as check-size-ark.sh.
find_tool() {
  local name="$1" out="${2:-}"
  for c in \
    tools/linux/xpack-arm-none-eabi-gcc-15.2.1-1.1/bin/arm-none-eabi-$name \
    tools/macos/xpack-arm-none-eabi-gcc-15.2.1-1.1/bin/arm-none-eabi-$name \
    tools/windows/xpack-arm-none-eabi-gcc-15.2.1-1.1/bin/arm-none-eabi-$name.exe
  do
    if [ -x "$ROOT/$c" ]; then echo "$ROOT/$c"; return 0; fi
  done
  echo "error: pinned arm-none-eabi-$name not found — run: make arm_sdk_install" >&2
  exit 127
}
NM_BIN="${NM_BIN:-$(find_tool nm)}"
READELF_BIN="${READELF_BIN:-$(find_tool readelf)}"

SYMS=$("$NM_BIN" -S "$ELF")
SECS=$("$READELF_BIN" -S -W "$ELF")

rc=0

# --- 1. hot-path symbols must live in RAM (0x20000000..) when present --------
# Required: F051 vector-table ISR entry points (must exist + RAM).
# Optional: known RAM_FUNC callees — if out-of-line, must be RAM; if absorbed, OK.
REQUIRED_HOT_SYMS="ADC1_COMP_IRQHandler TIM6_DAC_IRQHandler TIM14_IRQHandler"
OPTIONAL_HOT_SYMS="interruptRoutine tenKhzRoutine PeriodElapsedCallback getBemfState commutate comStep changeCompInput maskPhaseInterrupts enableCompInterrupts faultSignalTimeoutTick"

check_sym_ram() {
  local sym="$1" mode="$2"  # mode: required | optional
  local addr
  addr=$(awk -v s="$sym" '$NF==s {print $1; exit}' <<<"$SYMS")
  if [ -z "$addr" ]; then
    if [ "$mode" = "required" ]; then
      echo "  FAIL: $sym not found in the image" >&2
      rc=1
    else
      echo "  ok   $sym absent (LTO absorbed into caller)"
    fi
    return
  fi
  case "$addr" in
    20*) echo "  ok   $sym @ 0x$addr (RAM)" ;;
    *)   echo "  FAIL: $sym @ 0x$addr is not in RAM — RAM_FUNC placement lost" >&2; rc=1 ;;
  esac
}

echo "=== codegen check: $ELF ==="
if [ "$CHECK_RAMFUNC" -eq 1 ]; then
  for sym in $REQUIRED_HOT_SYMS; do
    check_sym_ram "$sym" required
  done
  for sym in $OPTIONAL_HOT_SYMS; do
    check_sym_ram "$sym" optional
  done
else
  echo "  --   RAM residency skipped (RAM_FUNC has no section attribute here)"
fi

# --- 2. special sections must be non-empty when present -----------------------
# .app_signature only exists in CAN builds; absent is fine, present-but-empty is
# not. Locate the size relative to the section name rather than by fixed column:
# readelf writes the index as "[ 9]" but "[12]", so a single-digit index splits
# into two fields and shifts every positional column by one.
check_section() {
  local name="$1" required="$2" empty_hint="$3"
  local size
  size=$(awk -v n="$name" '{for (i = 1; i <= NF; i++) if ($i == n) { print $(i + 4); exit }}' <<<"$SECS")
  if [ -z "$size" ]; then
    if [ "$required" = "required" ]; then
      echo "  FAIL: section $name missing" >&2; rc=1
    else
      echo "  ok   $name absent (not a CAN build)"
    fi
    return
  fi
  if [ $((16#$size)) -eq 0 ]; then
    echo "  FAIL: section $name is present but empty — $empty_hint" >&2
    rc=1
  else
    echo "  ok   $name = $((16#$size)) bytes"
  fi
}
# Externally-read image metadata: nothing in C reads these, so LTO DCE empties
# them without __attribute__((used)).
check_section .file_name required "LTO dropped its contents (object needs __attribute__((used)))"
check_section .app_signature optional "LTO dropped its contents (object needs __attribute__((used)))"
check_section .px4_app_descriptor optional "LTO dropped its contents (object needs __attribute__((used)))"
# Soft-reset cookie / retained SRAM (read+written from C). Empty here usually
# means the cookie was removed or the linker script changed — not LTO DCE.
check_section .noinit required "expected boot_sound_cookie (or other .noinit) — cookie removed or linker script changed?"

if [ "$rc" -ne 0 ]; then
  echo "check-codegen-ark: FAIL" >&2
  exit 1
fi
echo "check-codegen-ark: PASS"
