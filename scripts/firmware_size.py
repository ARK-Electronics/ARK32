#!/usr/bin/env python3
"""Compare flash and static RAM usage of two firmware ELFs."""

import argparse
import json
import os
from pathlib import Path
import subprocess

# The F051 app region is 27 KiB, so a few hundred bytes is a lot of it.
LARGE_DELTA = 256


def memory_usage(elf: Path) -> dict[str, int]:
    headers = subprocess.check_output(
        ["arm-none-eabi-objdump", "--section-headers", "--wide", str(elf)],
        text=True,
        env={**os.environ, "LC_ALL": "C"},
    )
    flash = 0
    ram = 0

    for line in headers.splitlines():
        fields = line.split(maxsplit=7)
        if len(fields) < 8 or not fields[0].isdigit():
            continue

        size, vma, lma = (int(value, 16) for value in fields[2:5])
        flags = {flag.strip() for flag in fields[7].split(",")}
        if "ALLOC" not in flags:
            continue

        # Summed rather than spanned like objcopy -O binary: .file_name is
        # pinned to the end of the app region, so the span never changes.
        in_image = {"LOAD", "CONTENTS"} <= flags
        if in_image:
            flash += size

        # Initialized data and RAM functions are copied out of the image;
        # NOLOAD sections only reserve RAM.
        if not (in_image and vma == lma):
            ram += size

    if not flash:
        raise ValueError(f"{elf}: no flash load image found")

    return {"flash": flash, "ram": ram}


def indicator(delta: int) -> str:
    if delta > LARGE_DELTA:
        return "🔴 "
    if delta > 0:
        return "🟡 "
    if delta < 0:
        return "🟢 "
    return ""


def format_change(before: int, after: int) -> str:
    delta = after - before
    percentage = f"{delta / before:+.2%}" if before else "n/a"
    return f"{indicator(delta)}{delta:+,} B ({percentage})"


def summarize(before: dict[str, int], after: dict[str, int]) -> dict:
    return {
        "flash": format_change(before["flash"], after["flash"]),
        "ram": format_change(before["ram"], after["ram"]),
        # The build is reproducible, so any delta comes from the change.
        "changed": before != after,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--before", type=Path, required=True)
    parser.add_argument("--after", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(summarize(memory_usage(args.before), memory_usage(args.after))))


if __name__ == "__main__":
    main()
