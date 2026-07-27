#!/usr/bin/env bash
# Verify the ARK 4IN1 factory full-flash image built by `make factory-image`.
# Intended for CI: fails on wrong size, missing regions, or ship-default drift.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

OBJ="${OBJ:-obj}"
BL="${BL_IMAGE_F051:-Bootloaders/AM32_F051_BOOTLOADER_PB4_V18.bin}"
DEFAULTS_JSON="${FACTORY_DEFAULTS:-factory/ARK_4IN1_F051_eeprom_defaults.json}"

shopt -s nullglob
factory_bins=("$OBJ"/AM32_ARK_4IN1_F051_*.factory.bin)
app_bins=("$OBJ"/AM32_ARK_4IN1_F051_*.bin)
# Exclude .factory.bin / .eeprom.bin from the app list
app_bins=()
for f in "$OBJ"/AM32_ARK_4IN1_F051_*.bin; do
	case "$f" in
		*.factory.bin|*.eeprom.bin) ;;
		*) app_bins+=("$f") ;;
	esac
done

if [[ ${#factory_bins[@]} -ne 1 ]]; then
	echo "error: expected exactly one factory.bin under $OBJ, found ${#factory_bins[@]}" >&2
	printf '  %s\n' "${factory_bins[@]:-}" >&2
	exit 1
fi
if [[ ${#app_bins[@]} -ne 1 ]]; then
	echo "error: expected exactly one app .bin under $OBJ, found ${#app_bins[@]}" >&2
	printf '  %s\n' "${app_bins[@]:-}" >&2
	exit 1
fi
if [[ ! -f "$BL" ]]; then
	echo "error: bootloader image missing: $BL" >&2
	exit 1
fi
if [[ ! -f "$DEFAULTS_JSON" ]]; then
	echo "error: defaults JSON missing: $DEFAULTS_JSON" >&2
	exit 1
fi

FACTORY="${factory_bins[0]}"
APP="${app_bins[0]}"

python3 - "$FACTORY" "$APP" "$BL" "$DEFAULTS_JSON" <<'PY'
import json
import sys
from pathlib import Path

FLASH_SIZE = 32 * 1024
BL_OFFSET = 0x0000
APP_OFFSET = 0x1000
EEPROM_OFFSET = 0x7C00
EEPROM_PAGE = 1024

factory_path, app_path, bl_path, defaults_path = map(Path, sys.argv[1:5])
img = factory_path.read_bytes()
app = app_path.read_bytes()
bl = bl_path.read_bytes()
defaults = json.loads(defaults_path.read_text(encoding="utf-8"))
s = defaults["settings"]

errors: list[str] = []

if len(img) != FLASH_SIZE:
    errors.append(f"factory image size {len(img)} != {FLASH_SIZE}")

if len(bl) > APP_OFFSET:
    errors.append(f"bootloader {len(bl)} exceeds region {APP_OFFSET}")
if img[BL_OFFSET:BL_OFFSET + len(bl)] != bl:
    errors.append("bootloader region does not match Bootloaders/*.bin")
if any(b != 0xFF for b in img[len(bl):APP_OFFSET]):
    errors.append("gap between bootloader and app is not 0xFF-padded")

if len(app) > EEPROM_OFFSET - APP_OFFSET:
    errors.append(f"app {len(app)} exceeds app region")
if img[APP_OFFSET:APP_OFFSET + len(app)] != app:
    errors.append("app region does not match release .bin")

ee = img[EEPROM_OFFSET:EEPROM_OFFSET + EEPROM_PAGE]
if len(ee) != EEPROM_PAGE:
    errors.append("eeprom page truncated")

# Encodings must match scripts/build_factory_image.py / Src/settings.c
def expect(off: int, want: int, name: str) -> None:
    got = ee[off]
    if got != want:
        errors.append(f"eeprom[{off}] {name}: got {got} want {want}")

kv = int(s["motor_kv"])
kv_byte = (kv - 20) // 40
if kv_byte * 40 + 20 != kv:
    errors.append(f"defaults motor_kv {kv} not encodable")
max_ramp = int(round(float(s["max_ramp_percent_per_ms"]) * 10))
adv_temp = int(round(int(s["advance_degrees"]) * 64 / 60.0))
adv_level = adv_temp + 10
servo_lo = (int(s["pwm_input_min_us"]) - 750) // 2
servo_hi = (int(s["pwm_input_max_us"]) - 1750) // 2

expect(1, int(defaults.get("eeprom_version", 3)), "eeprom_version")
expect(5, max_ramp, "max_ramp")
expect(21, int(s["variable_pwm"]), "variable_pwm")
expect(23, adv_level, "advance_level")
expect(26, kv_byte, "motor_kv")
expect(32, servo_lo, "servo_low")
expect(33, servo_hi, "servo_high")
expect(47, int(s["auto_advance"]), "auto_advance")

# Trailing pad of the page after the 192-byte settings union should stay erased
if any(b != 0xFF for b in ee[192:]):
    errors.append("eeprom page bytes 192..1023 are not 0xFF")

if errors:
    print("factory image check FAILED:", file=sys.stderr)
    for e in errors:
        print(f"  - {e}", file=sys.stderr)
    sys.exit(1)

print(f"OK {factory_path} ({len(img)} bytes)")
print(f"  bl={len(bl)}B app={len(app)}B eeprom@0x{EEPROM_OFFSET:04X}")
print(
    f"  defaults: kv={kv} max_ramp={max_ramp} var_pwm={s['variable_pwm']} "
    f"advance={adv_level} (~{s['advance_degrees']}°) "
    f"pwm={s['pwm_input_min_us']}/{s['pwm_input_max_us']} "
    f"auto_advance={s['auto_advance']}"
)
PY
