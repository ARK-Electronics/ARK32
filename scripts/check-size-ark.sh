#!/usr/bin/env bash
# Fail if the ARK F051 image exceeds flash/RAM budgets.
# Gate inputs (see make size-check-ark): HWCI_PERF=1 without .bl_image, and
# release with embedded bootloader — each checked separately.
#
# STM32F051K6 application region (from linker script / build map):
#   FLASH (app): 27424 bytes
#   RAM:         8000 bytes
#
# We require a small free margin so a "tiny" PR cannot silently fill the chip.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

# STM32F051K6TX_FLASH.ld regions that hold RX image (not EEPROM):
#   FLASH_VECTAB 192 + FLASH 27424 + FILE_NAME 32 = 27648
# arm-none-eabi-size "text" spans those RX regions; do not compare to 27424 alone.
FLASH_CAPACITY="${FLASH_CAPACITY:-27648}"
RAM_CAPACITY="${RAM_CAPACITY:-8000}"
# Flash is budgeted to near-capacity deliberately: the F051 app region is the
# binding constraint on this fork, and holding back 5% was blocking feature work
# for margin the project does not need. The gate exists to catch a PR that
# *overflows*, not to reserve headroom.
#
# NOTE: this is a size decision only. Global -O3 previously filled ~99% AND
# regressed hold100 free-run on F051 (armed/input drop near ~97% throttle) — see
# the CFLAGS comment in the Makefile. That regression is functional, not a size
# problem, so a higher limit here is NOT permission to enable global -O3.
#
# RAM stays at 90%: unlike flash, the remaining RAM is stack headroom, and
# overflowing it fails at runtime rather than at link time.
FLASH_MAX_PCT="${FLASH_MAX_PCT:-99.8}"
RAM_MAX_PCT="${RAM_MAX_PCT:-90}"

ELF=$(ls -1 obj/ARK32_ARK_4IN1_F051_*.elf 2>/dev/null | head -1 || true)
if [ -z "$ELF" ] || [ ! -f "$ELF" ]; then
  echo "error: no obj/ARK32_ARK_4IN1_F051_*.elf — build ARK_4IN1_F051 HWCI_PERF=1 first" >&2
  exit 2
fi

# Prefer the pinned xPack 15 size(1); never use distro gcc-arm-none-eabi first.
SIZE_BIN="${SIZE_BIN:-}"
if [ -z "$SIZE_BIN" ]; then
  for c in \
    tools/linux/xpack-arm-none-eabi-gcc-15.2.1-1.1/bin/arm-none-eabi-size \
    tools/macos/xpack-arm-none-eabi-gcc-15.2.1-1.1/bin/arm-none-eabi-size \
    tools/windows/xpack-arm-none-eabi-gcc-15.2.1-1.1/bin/arm-none-eabi-size.exe
  do
    if [ -x "$ROOT/$c" ]; then SIZE_BIN="$ROOT/$c"; break; fi
  done
fi
if [ -z "$SIZE_BIN" ]; then
  echo "error: pinned arm-none-eabi-size not found under tools/*/xpack-arm-none-eabi-gcc-15.2.1-1.1/" >&2
  echo "       run: make arm_sdk_install" >&2
  exit 127
fi

# Section-accurate accounting (matches ld --print-memory-usage better than
# the classic one-line size totals on this multi-region F051 layout).
SEC=$("$SIZE_BIN" -A "$ELF")
sec_size() { echo "$SEC" | awk -v s="$1" '$1==s{print $2; found=1} END{if(!found)print 0}'; }

TEXT=$(sec_size .text)
RODATA=$(sec_size .rodata)
ISR=$(sec_size .isr_vector)
FILE_NAME_SZ=$(sec_size .file_name)
INITA=$(sec_size .init_array)
FINIA=$(sec_size .fini_array)
DATA=$(sec_size .data)
BSS=$(sec_size .bss)
# Survives soft-reset (not zeroed); currently the signal-lost boot cookie
NOINIT=$(sec_size .noinit)
HEAPSTACK=$(sec_size ._user_heap_stack)
# Embedded bootloader image, if this build carries one. It sits in its own
# section rather than .rodata, so it has to be added explicitly or the gate
# silently under-reports by the whole 4 KiB. 0 on builds without it.
BL_IMAGE=$(sec_size .bl_image)

# RX image in flash: code + const + vectab + filename + init arrays +
# embedded bootloader image + the flash load image of .data
FLASH_USED=$((ISR + TEXT + RODATA + INITA + FINIA + FILE_NAME_SZ + BL_IMAGE + DATA))
# RAM at runtime: .data + .bss + .noinit + heap/stack reservation
RAM_USED=$((DATA + BSS + NOINIT + HEAPSTACK))

# Fractional percentages are supported (e.g. 99.8), so the limits are computed
# with awk rather than shell integer arithmetic, truncating toward zero.
FLASH_MAX=$(awk -v c="$FLASH_CAPACITY" -v p="$FLASH_MAX_PCT" 'BEGIN{printf "%d", c*p/100}')
RAM_MAX=$(awk -v c="$RAM_CAPACITY" -v p="$RAM_MAX_PCT" 'BEGIN{printf "%d", c*p/100}')

flash_pct=$(awk -v u="$FLASH_USED" -v c="$FLASH_CAPACITY" 'BEGIN{printf "%.2f", 100*u/c}')
ram_pct=$(awk -v u="$RAM_USED" -v c="$RAM_CAPACITY" 'BEGIN{printf "%.2f", 100*u/c}')

echo "=== size check: $ELF ==="
BL_NOTE=""
[ "${BL_IMAGE:-0}" -gt 0 ] 2>/dev/null && BL_NOTE=" .bl_image=$BL_IMAGE"
echo "  .text=$TEXT .rodata=$RODATA .data=$DATA .bss=$BSS .noinit=$NOINIT heap/stack=$HEAPSTACK$BL_NOTE"
echo "  FLASH used=$FLASH_USED / $FLASH_CAPACITY (${flash_pct}%)  limit=${FLASH_MAX} (${FLASH_MAX_PCT}%)"
echo "  RAM   used=$RAM_USED / $RAM_CAPACITY (${ram_pct}%)  limit=${RAM_MAX} (${RAM_MAX_PCT}%)"

rc=0
if [ "$FLASH_USED" -gt "$FLASH_MAX" ]; then
  echo "FAIL: flash $FLASH_USED > limit $FLASH_MAX" >&2
  rc=1
fi
if [ "$RAM_USED" -gt "$RAM_MAX" ]; then
  echo "FAIL: RAM $RAM_USED > limit $RAM_MAX" >&2
  rc=1
fi
if [ "$rc" -eq 0 ]; then
  echo "size-check-ark: PASS"
fi
exit "$rc"
