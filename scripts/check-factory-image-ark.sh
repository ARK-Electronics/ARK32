#!/usr/bin/env bash
# Verify an ARK factory full-flash image built by `make factory-image*`.
# Intended for CI: fails on wrong size, missing regions, or ship-default drift.
#
# Env overrides (or defaults for ARK_4IN1_F051):
#   FACTORY_PRODUCT   e.g. ARK_4IN1_F051 or ARK_G431_CAN
#   FACTORY_DEFAULTS  path to factory/*_eeprom_defaults.json
#   BL_IMAGE          optional bootloader .bin (omit / empty = allow 0xFF BL)
#   BL_IMAGE_F051     fallback default for ARK_4IN1_F051 when BL_IMAGE unset
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

OBJ="${OBJ:-obj}"
PRODUCT="${FACTORY_PRODUCT:-ARK_4IN1_F051}"
DEFAULTS_JSON="${FACTORY_DEFAULTS:-factory/${PRODUCT}_eeprom_defaults.json}"
BL="${BL_IMAGE:-}"

# Default BL for the classic F051 product when caller did not set BL_IMAGE.
# Must match Makefile BL_IMAGE_F051 (ARK 4IN1 nSLEEP-off BL, not generic PB4).
if [[ -z "$BL" && "$PRODUCT" == "ARK_4IN1_F051" ]]; then
	BL="${BL_IMAGE_F051:-Bootloaders/AM32_F051_BOOTLOADER_ARK4IN1_V18.bin}"
fi

shopt -s nullglob
factory_bins=("$OBJ"/ARK32_"${PRODUCT}"_*.factory.bin)
# App .bin only: exclude factory, eeprom, and local experiment names (*nsleep*).
app_bins=()
for f in "$OBJ"/ARK32_"${PRODUCT}"_*.bin; do
	base="$(basename "$f")"
	case "$base" in
		*.factory.bin|*.eeprom.bin|*factory*|*nsleep*|*hwci*) ;;
		ARK32_"${PRODUCT}"_*.bin) app_bins+=("$f") ;;
	esac
done

if [[ ${#factory_bins[@]} -ne 1 ]]; then
	echo "error: expected exactly one ${PRODUCT} factory.bin under $OBJ, found ${#factory_bins[@]}" >&2
	printf '  %s\n' "${factory_bins[@]:-}" >&2
	exit 1
fi
if [[ ${#app_bins[@]} -ne 1 ]]; then
	echo "error: expected exactly one ${PRODUCT} app .bin under $OBJ, found ${#app_bins[@]}" >&2
	printf '  %s\n' "${app_bins[@]:-}" >&2
	exit 1
fi
if [[ ! -f "$DEFAULTS_JSON" ]]; then
	echo "error: defaults JSON missing: $DEFAULTS_JSON" >&2
	exit 1
fi
if [[ -n "$BL" && ! -f "$BL" ]]; then
	echo "error: bootloader image missing: $BL" >&2
	exit 1
fi

FACTORY="${factory_bins[0]}"
APP="${app_bins[0]}"

python3 - "$FACTORY" "$APP" "$DEFAULTS_JSON" "${BL:-}" <<'PY'
import json
import sys
from pathlib import Path

factory_path = Path(sys.argv[1])
app_path = Path(sys.argv[2])
defaults_path = Path(sys.argv[3])
bl_path = Path(sys.argv[4]) if sys.argv[4] else None

defaults = json.loads(defaults_path.read_text(encoding="utf-8"))
s = defaults["settings"]

# Default F051 map; product JSON may override (ARK_G431_CAN).
fmap = {
    "flash_size": 32 * 1024,
    "bl_offset": 0,
    "bl_region_size": 4 * 1024,
    "app_offset": 0x1000,
    "eeprom_offset": 0x7C00,
    "eeprom_page_size": 1 * 1024,
}
for k, v in (defaults.get("flash_map") or {}).items():
    if k.startswith("_"):
        continue
    if k in fmap or k in (
        "flash_size",
        "bl_offset",
        "bl_region_size",
        "app_offset",
        "eeprom_offset",
        "eeprom_page_size",
    ):
        fmap[k] = int(v)

FLASH_SIZE = fmap["flash_size"]
BL_OFFSET = fmap["bl_offset"]
BL_REGION = fmap["bl_region_size"]
APP_OFFSET = fmap["app_offset"]
EEPROM_OFFSET = fmap["eeprom_offset"]
EEPROM_PAGE = fmap["eeprom_page_size"]
APP_REGION = EEPROM_OFFSET - APP_OFFSET

img = factory_path.read_bytes()
app = app_path.read_bytes()
bl = bl_path.read_bytes() if bl_path is not None else b""

errors: list[str] = []

if len(img) != FLASH_SIZE:
    errors.append(f"factory image size {len(img)} != {FLASH_SIZE}")

if len(bl) > BL_REGION:
    errors.append(f"bootloader {len(bl)} exceeds region {BL_REGION}")
if bl:
    if img[BL_OFFSET : BL_OFFSET + len(bl)] != bl:
        bl_name = bl_path.name if bl_path is not None else "Bootloaders/*.bin"
        errors.append(
            f"bootloader region does not match {bl_name} "
            f"(factory image BL must equal BL_IMAGE)"
        )
    if any(b != 0xFF for b in img[BL_OFFSET + len(bl) : APP_OFFSET]):
        errors.append("gap between bootloader and app is not 0xFF-padded")
else:
    if any(b != 0xFF for b in img[BL_OFFSET:APP_OFFSET]):
        errors.append("empty-bootloader mode: BL region is not all 0xFF")

if len(app) > APP_REGION:
    errors.append(f"app {len(app)} exceeds app region {APP_REGION}")
if img[APP_OFFSET : APP_OFFSET + len(app)] != app:
    errors.append("app region does not match release .bin")

ee = img[EEPROM_OFFSET : EEPROM_OFFSET + EEPROM_PAGE]
if len(ee) != EEPROM_PAGE:
    errors.append("eeprom page truncated")

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
expect(46, int(s["input_type"]), "input_type")
expect(47, int(s["auto_advance"]), "auto_advance")
# Protection limits and the current loop that enforces one of them. Gated
# because "armed" and "disabled" differ by a single byte just outside a
# range check (settings.c), which is how both products shipped with the
# limiters silently off. The gate is equality with the JSON; whether that
# JSON arms them is a product decision, reported below either way.
expect(9, int(s["current_P"]), "current_P")
expect(10, int(s["current_I"]), "current_I")
expect(11, int(s["current_D"]), "current_D")
expect(43, int(s["temperature_limit"]), "temperature_limit")
expect(44, int(s["current_limit"]), "current_limit")
if "temperature_derate_band" in s:
    expect(184, int(s["temperature_derate_band"]), "temp_derate_band")
elif ee[184] != 0xFF:
    errors.append(f"eeprom[184] temp_derate_band: got {ee[184]} want 255 (JSON omits it)")

t_lim = int(s["temperature_limit"])
i_lim = int(s["current_limit"])
band = int(s.get("temperature_derate_band", 15))
thermal_desc = (
    f"derate {t_lim}->{t_lim + band} C" if 70 <= t_lim <= 140 else f"OFF ({t_lim})"
)
current_desc = f"{i_lim * 2} A" if 0 < i_lim <= 100 else f"OFF ({i_lim})"

if any(b != 0xFF for b in ee[192:]):
    errors.append(f"eeprom page bytes 192..{EEPROM_PAGE - 1} are not 0xFF")

if errors:
    print("factory image check FAILED:", file=sys.stderr)
    for e in errors:
        print(f"  - {e}", file=sys.stderr)
    sys.exit(1)

print(f"OK {factory_path} ({len(img)} bytes)")
print(
    f"  bl={len(bl)}B/{BL_REGION}B app={len(app)}B/{APP_REGION}B "
    f"eeprom@0x{EEPROM_OFFSET:04X} page={EEPROM_PAGE}"
)
print(
    f"  defaults: kv={kv} max_ramp={max_ramp} var_pwm={s['variable_pwm']} "
    f"advance={adv_level} (~{s['advance_degrees']}°) "
    f"pwm={s['pwm_input_min_us']}/{s['pwm_input_max_us']} "
    f"input_type={s['input_type']} auto_advance={s['auto_advance']}"
)
print(
    f"  limits: thermal={thermal_desc} current={current_desc} "
    f"current_pid={s['current_P']}/{s['current_I']}/{s['current_D']}"
)
PY
