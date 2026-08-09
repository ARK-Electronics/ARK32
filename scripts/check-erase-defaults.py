#!/usr/bin/env python3
"""Gate: a DroneCAN "restore defaults" must land on the shipped protection.

Three copies of the same numbers have to agree, or a field param-erase
silently changes a shipped ESC's protection envelope:

  1. Inc/targets.h            TARGET_DEFAULT_{TEMPERATURE_LIMIT,CURRENT_LIMIT,
                              TEMP_DERATE_BAND} for the product
  2. Src/DroneCAN/DroneCAN.c  default_settings[43]/[44] - the bytes an ERASE
                              memcpy's back over the page (must be literals:
                              Mcu/SITL/sitl_params.py parses this array)
  3. factory/<PRODUCT>_eeprom_defaults.json   what production actually flashes

Byte 184 (the foldback band) lives past the 48-byte skeleton and is written
by apply_post_skeleton_defaults() from the macro, so it is checked against
the macro and the JSON but not against the array.

Only DroneCAN products can reach the erase path, so only those are checked;
the array is compiled into DroneCAN.c and is unreachable on the 4IN1.

Usage: scripts/check-erase-defaults.py [PRODUCT ...]   (default: ARK_G431_CAN)
"""

from __future__ import annotations

import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

# (macro suffix, DroneCAN.c eeprom offset or None, JSON key)
FIELDS = (
    ("TEMPERATURE_LIMIT", 43, "temperature_limit"),
    ("CURRENT_LIMIT", 44, "current_limit"),
    ("TEMP_DERATE_BAND", None, "temperature_derate_band"),
)


def target_macros(product: str) -> dict[str, int]:
    """macros defined inside the product's #ifdef block in targets.h"""
    text = (ROOT / "Inc" / "targets.h").read_text(encoding="utf-8")
    start = text.find("#ifdef %s\n" % product)
    if start < 0:
        raise SystemExit("error: no #ifdef %s in Inc/targets.h" % product)
    # the block ends at the next top-level #ifdef, which is how the file is
    # organised (one block per board, no nesting at column 0)
    nxt = text.find("\n#ifdef ", start + 1)
    block = text[start:nxt if nxt > 0 else len(text)]
    out = {}
    for suffix, _off, _key in FIELDS:
        m = re.search(r"#\s*define\s+TARGET_DEFAULT_%s\s+(\d+)" % suffix, block)
        if m:
            out[suffix] = int(m.group(1))
    return out


def erase_bytes() -> bytes:
    text = (ROOT / "Src" / "DroneCAN" / "DroneCAN.c").read_text(encoding="utf-8")
    m = re.search(r"default_settings\[\]\s*=\s*\{(.*?)\}\s*;", text, re.S)
    if not m:
        raise SystemExit("error: no default_settings[] in Src/DroneCAN/DroneCAN.c")
    return bytes(int(v, 16) for v in re.findall(r"0x([0-9a-fA-F]+)", m.group(1)))


def check(product: str) -> list[str]:
    errors: list[str] = []
    macros = target_macros(product)
    arr = erase_bytes()
    jpath = ROOT / "factory" / ("%s_eeprom_defaults.json" % product)
    if not jpath.is_file():
        return ["%s: missing %s" % (product, jpath)]
    settings = json.loads(jpath.read_text(encoding="utf-8"))["settings"]

    for suffix, off, key in FIELDS:
        macro = "TARGET_DEFAULT_%s" % suffix
        if suffix not in macros:
            errors.append(
                "%s: %s not defined in its targets.h block - an erase would "
                "restore the generic default, not the shipped value"
                % (product, macro))
            continue
        want = macros[suffix]
        if key not in settings:
            errors.append("%s: factory JSON has no %s (macro says %d)"
                          % (product, key, want))
        elif int(settings[key]) != want:
            errors.append("%s: factory JSON %s=%d but %s=%d"
                          % (product, key, int(settings[key]), macro, want))
        if off is not None:
            if off >= len(arr):
                errors.append("%s: default_settings[] too short for byte %d"
                              % (product, off))
            elif arr[off] != want:
                errors.append(
                    "%s: default_settings[%d]=%d but %s=%d - a param ERASE "
                    "would not restore the shipped value"
                    % (product, off, arr[off], macro, want))

    # An erase must leave both limiters ARMED, per Src/settings.c ranges.
    t = macros.get("TEMPERATURE_LIMIT")
    c = macros.get("CURRENT_LIMIT")
    if t is not None and not (70 <= t <= 140):
        errors.append("%s: TARGET_DEFAULT_TEMPERATURE_LIMIT=%d is outside the "
                      "70..140 settings.c arms - erase would disable the "
                      "thermal derate" % (product, t))
    if c is not None and not (0 < c <= 100):
        errors.append("%s: TARGET_DEFAULT_CURRENT_LIMIT=%d is outside the "
                      "1..100 settings.c arms - erase would disable the "
                      "current limiter" % (product, c))
    return errors


def main(argv: list[str]) -> int:
    products = argv[1:] or ["ARK_G431_CAN"]
    errors: list[str] = []
    for p in products:
        errors += check(p)
    if errors:
        print("erase-defaults check FAILED:", file=sys.stderr)
        for e in errors:
            print("  - %s" % e, file=sys.stderr)
        return 1
    for p in products:
        m = target_macros(p)
        print("OK %s: erase restores thermal %d C over %d C band, current %d A"
              % (p, m["TEMPERATURE_LIMIT"], m["TEMP_DERATE_BAND"],
                 m["CURRENT_LIMIT"] * 2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
