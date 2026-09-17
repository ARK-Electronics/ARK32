#!/usr/bin/env python3
"""Gate: a DroneCAN "restore defaults" must land on the shipped protection.

Three copies of the same numbers have to agree, or a field param-erase
silently changes a shipped ESC's protection envelope:

  1. Inc/targets.h            TARGET_DEFAULT_{TEMPERATURE_LIMIT,CURRENT_LIMIT,
                              TEMP_DERATE_BAND} for the product
  2. Src/DroneCAN/DroneCAN.c  apply_post_skeleton_defaults() - what an ERASE
                              re-applies on top of the generated configurator
                              skeleton, which still carries upstream's
                              protection-disabled bytes
  3. factory/<PRODUCT>_eeprom_defaults.json   what production actually flashes

The skeleton itself is the AM32 configurator image and is deliberately left
alone (schema/eeprom-defaults.hex is golden-pinned), so every value below is
checked against the post-skeleton restore rather than the skeleton bytes.

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

# (macro suffix, C member restored post-skeleton or None, JSON key, JSON scale)
# scale converts the human JSON unit to the stored byte; max_ramp is the one
# field the JSON expresses in %/ms rather than raw counts.
FIELDS = (
    ("TEMPERATURE_LIMIT", "temperature_limit", "temperature_limit", 1),
    ("CURRENT_LIMIT", "current_limit", "current_limit", 0.5),
    ("TEMP_DERATE_BAND", "can_temp_derate_band", "temperature_derate_band", 1),
    # member None: the skeleton still carries upstream's 160 on purpose (see
    # the MAX_RAMP note in DroneCAN.c) and nothing restores it, so only the
    # macro and the product JSON are held together here.
    ("INPUT_TYPE", "input_type", "input_type", 1),
    ("MAX_RAMP", None, "max_ramp_percent_per_ms", 10),
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
    for suffix, _member, _key, _scale in FIELDS:
        m = re.search(r"#\s*define\s+TARGET_DEFAULT_%s\s+(\d+)" % suffix, block)
        if m:
            out[suffix] = int(m.group(1))
    return out


def erase_bytes() -> bytes:
    # The skeleton is generated from the schema now, so read it from the same
    # source the build does rather than re-parsing a C literal.
    sys.path.insert(0, str(ROOT / "scripts" / "eeprom"))
    from schema import build_context, default_bytes, load_schema  # noqa: E402
    layout, firmware = build_context()
    return default_bytes(load_schema(), layout, firmware)


def post_skeleton_restores() -> dict[str, str]:
    """C member -> the TARGET_DEFAULT_* suffix apply_post_skeleton_defaults()
    assigns it, so a member that quietly stops being restored is caught."""
    text = (ROOT / "Src" / "DroneCAN" / "DroneCAN.c").read_text(encoding="utf-8")
    m = re.search(r"static void apply_post_skeleton_defaults\(EEprom_t \*e\)\s*\{(.*?)\n\}",
                  text, re.S)
    if not m:
        raise SystemExit(
            "error: no apply_post_skeleton_defaults() in Src/DroneCAN/DroneCAN.c")
    return dict((member, macro) for member, macro in re.findall(
        r"e->(\w+)\s*=\s*TARGET_DEFAULT_(\w+)\s*;", m.group(1)))


def check(product: str) -> list[str]:
    errors: list[str] = []
    macros = target_macros(product)
    restores = post_skeleton_restores()
    jpath = ROOT / "factory" / ("%s_eeprom_defaults.json" % product)
    if not jpath.is_file():
        return ["%s: missing %s" % (product, jpath)]
    settings = json.loads(jpath.read_text(encoding="utf-8"))["settings"]

    for suffix, member, key, scale in FIELDS:
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
        else:
            shipped = int(round(float(settings[key]) * scale))
            if shipped != want:
                errors.append("%s: factory JSON %s=%s (stored %d) but %s=%d"
                              % (product, key, settings[key], shipped, macro, want))
        if member is not None and restores.get(member) != suffix:
            errors.append(
                "%s: apply_post_skeleton_defaults() does not set %s from %s - "
                "a param ERASE would leave the configurator skeleton's value"
                % (product, member, macro))

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
    arr = erase_bytes()
    for p in products:
        m = target_macros(p)
        print("OK %s: erase restores thermal %d C over %d C band, current %d A"
              % (p, m["TEMPERATURE_LIMIT"], m["TEMP_DERATE_BAND"],
                 m["CURRENT_LIMIT"] * 2))
        # Said separately because it is NOT restored: the array still carries
        # upstream's value, so an erase changes the ramp the product ships.
        print("   ramp: ships %.1f %%/ms, erase writes %.1f %%/ms%s"
              % (m["MAX_RAMP"] / 10.0, arr[5] / 10.0,
                 "" if arr[5] == m["MAX_RAMP"] else "  <-- not restored"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
