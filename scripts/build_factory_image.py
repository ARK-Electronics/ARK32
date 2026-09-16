#!/usr/bin/env python3
"""Build a production full-flash image for ARK F051 ESCs.

Assembles a single binary that covers the entire STM32F051 32 KiB flash map:

  0x08000000  bootloader region (4 KiB)     — ARK32-bootloader .bin, 0xFF padded
  0x08001000  application region (27 KiB)  — make ARK_4IN1_F051 .bin, 0xFF padded
  0x08007C00  EEPROM settings page (1 KiB) — factory defaults, 0xFF padded

Production used to: flash BL, flash app, write EEPROM via AM32 configurator,
tweak settings, then ST-Link dump the whole chip. This script produces that
same image from the repo so a release is one `make factory-image` artifact.

EEPROM offsets and exact display conversions come from schema/eeprom.json.
"""

from __future__ import annotations

import argparse
import json
import re
import struct
import sys
from pathlib import Path

_SCRIPTS = Path(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))
from px4_uavcan_image import DESC_LEN, find_descriptor, unpack_desc

sys.path.insert(0, str(Path(__file__).resolve().parent / "eeprom"))
from schema import default_bytes, from_raw, load_schema, raw_range, resolve_schema, to_raw, build_context

_SCHEMA = load_schema()
_LAYOUT, _FIRMWARE = build_context()
_FIELDS = resolve_schema(_SCHEMA, _LAYOUT, _FIRMWARE)["fields"]

# STM32F051 flash map used by ARK_4IN1_F051 (Inc/targets.h, STM32F051K6TX_FLASH.ld)
FLASH_SIZE = 32 * 1024
BL_OFFSET = 0x0000
BL_REGION_SIZE = 4 * 1024
APP_OFFSET = 0x1000
APP_REGION_SIZE = 27 * 1024  # ends at EEPROM_OFFSET
EEPROM_OFFSET = 0x7C00
EEPROM_PAGE_SIZE = 1 * 1024
EEPROM_SETTINGS_SIZE = _SCHEMA['bufferSize']

DEFAULT_FLASH_MAP = {
    "flash_size": FLASH_SIZE,
    "bl_offset": BL_OFFSET,
    "bl_region_size": BL_REGION_SIZE,
    "app_offset": APP_OFFSET,
    "eeprom_offset": EEPROM_OFFSET,
    "eeprom_page_size": EEPROM_PAGE_SIZE,
}

assert BL_REGION_SIZE + APP_REGION_SIZE + EEPROM_PAGE_SIZE == FLASH_SIZE
assert APP_OFFSET == BL_OFFSET + BL_REGION_SIZE
assert EEPROM_OFFSET == APP_OFFSET + APP_REGION_SIZE


class FactoryImageError(SystemExit):
    pass


def _read_version_h(path: Path) -> tuple[int, int, int]:
    text = path.read_text(encoding="utf-8")

    def one(name: str) -> int:
        m = re.search(rf"#define\s+{name}\s+(\d+)", text)
        if not m:
            raise FactoryImageError(f"{path}: missing #define {name}")
        return int(m.group(1))

    return one("VERSION_MAJOR"), one("VERSION_MINOR"), one("EEPROM_VERSION")


def _encode_field(field: dict, value) -> int:
    # A disabled value is a raw sentinel, even when it lies outside the
    # ordinary setting's valid range (for example, temperature 255).
    disabled = field.get("disabledValue", {}).get("raw")
    if disabled is not None and value == disabled:
        return disabled
    # The historical erase image deliberately carries out-of-range disable
    # bytes. Preserve only its documented sentinels; ordinary values must be
    # representable exactly in display units.
    sentinel = field.get("default", {}).get("raw")
    minimum, maximum = raw_range(field)
    if ("disabledValue" in field and isinstance(sentinel, int)
            and not minimum <= sentinel <= maximum and value == sentinel):
        return sentinel
    try:
        return to_raw(field, value, exact=True)
    except ValueError as exc:
        raise FactoryImageError(str(exc)) from exc


def encode_motor_kv(kv):
    return _encode_field(_FIELDS["motorKv"], kv)


def encode_max_ramp_percent_per_ms(pct):
    return _encode_field(_FIELDS["maxRampSpeed"], pct)


def encode_advance_degrees(degrees):
    return _encode_field(_FIELDS["timingAdvance"], degrees)


def encode_servo_low_us(us):
    return _encode_field(_FIELDS["servoLowThreshold"], us)


def encode_servo_high_us(us):
    return _encode_field(_FIELDS["servoHighThreshold"], us)


def encode_servo_neutral_us(us):
    return _encode_field(_FIELDS["servoNeutral"], us)


def resolve_flash_map(defaults: dict) -> dict:
    """Merge the product's flash_map over the F051 default map; validate sizes."""
    m = dict(DEFAULT_FLASH_MAP)
    raw = defaults.get("flash_map") or {}
    for k in DEFAULT_FLASH_MAP:
        if k in raw:
            m[k] = int(raw[k])
    flash, bl_off, bl_sz = m["flash_size"], m["bl_offset"], m["bl_region_size"]
    app_off, ee_off, ee_sz = m["app_offset"], m["eeprom_offset"], m["eeprom_page_size"]
    if bl_off != 0:
        raise FactoryImageError("flash_map.bl_offset must be 0")
    if app_off != bl_off + bl_sz:
        raise FactoryImageError(
            f"flash_map: app_offset 0x{app_off:X} != bl_offset+bl_region_size "
            f"0x{bl_off + bl_sz:X}"
        )
    if ee_off + ee_sz > flash:
        raise FactoryImageError(
            f"flash_map: eeprom ends past flash_size "
            f"(0x{ee_off + ee_sz:X} > 0x{flash:X})"
        )
    if ee_off < app_off:
        raise FactoryImageError("flash_map: eeprom_offset before app_offset")
    m["app_region_size"] = ee_off - app_off
    return m


def build_eeprom_page(defaults: dict, version_major: int, version_minor: int,
                      eeprom_version: int, page_size: int = EEPROM_PAGE_SIZE) -> bytes:
    """Build the exact product page over the schema's historical erase prefix."""
    fields = resolve_schema(_SCHEMA, eeprom_version, (version_major, version_minor))["fields"]
    prefix = default_bytes(_SCHEMA, eeprom_version, (version_major, version_minor))
    buf = bytearray(b"\xff" * page_size)
    buf[:len(prefix)] = prefix
    stamps = {"eepromVersion": eeprom_version, "firmwareMajor": version_major,
              "firmwareMinor": version_minor}
    for key, value in stamps.items():
        buf[fields[key]["offset"]] = value
    for field in fields.values():
        if "factory" not in field:
            continue
        key = field["factory"]["key"]
        if key not in defaults["settings"]:
            # An optional key is one only some products have (the CAN block on
            # a board with no CAN bus); its bytes stay erased.
            if field["factory"].get("optional"):
                continue
            raise FactoryImageError(f"missing factory setting: {key}")
        raw = _encode_field(field, defaults["settings"][key])
        offset, size = field["offset"], field["size"]
        buf[offset:offset + size] = raw.to_bytes(size, "little", signed=field["type"].startswith("int"))
    return bytes(buf)


def decode_summary(page: bytes) -> list[str]:
    """Display the schema-resolved product values for a production check."""
    fields = resolve_schema(_SCHEMA, page[1], (page[3], page[4]))["fields"]
    return [f"eeprom_version={page[1]} fw={page[3]}.{page[4]}"] + [
        f"{field['factory']['key']}={from_raw(field, page[field['offset']])} {field.get('unit', '')}".rstrip()
        for field in fields.values()
        if "factory" in field and not (field["factory"].get("optional")
                                       and page[field["offset"]] == 0xFF)]


def place_region(image: bytearray, offset: int, region_size: int, data: bytes,
                 name: str) -> None:
    if len(data) > region_size:
        raise FactoryImageError(
            f"{name}: {len(data)} bytes exceeds region size {region_size} "
            f"(offset 0x{offset:04X})"
        )
    image[offset:offset + len(data)] = data
    # remainder of region already 0xFF


def build_full_image(bootloader: bytes, app: bytes, eeprom_page: bytes,
                     fmap: dict | None = None) -> bytes:
    fmap = fmap or {**DEFAULT_FLASH_MAP, "app_region_size": APP_REGION_SIZE}
    image = bytearray([0xFF] * fmap["flash_size"])
    place_region(image, fmap["bl_offset"], fmap["bl_region_size"], bootloader, "bootloader")
    place_region(image, fmap["app_offset"], fmap["app_region_size"], app, "application")
    place_region(image, fmap["eeprom_offset"], fmap["eeprom_page_size"], eeprom_page, "eeprom")
    return bytes(image)


def write_intel_hex(path: Path, data: bytes, base: int = 0x08000000) -> None:
    """Minimal I32HEX writer for a contiguous flash image."""
    lines: list[str] = []
    # extended linear address for 0x08000000
    ela = (base >> 16) & 0xFFFF
    lines.append(_hex_record(0, 0x04, struct.pack(">H", ela)))
    offset = base & 0xFFFF
    i = 0
    while i < len(data):
        chunk = data[i:i + 16]
        addr = (offset + i) & 0xFFFF
        # cross 64K boundary
        abs_addr = base + i
        if i > 0 and (abs_addr & 0xFFFF) == 0:
            ela = (abs_addr >> 16) & 0xFFFF
            lines.append(_hex_record(0, 0x04, struct.pack(">H", ela)))
        lines.append(_hex_record(addr, 0x00, chunk))
        i += len(chunk)
    lines.append(_hex_record(0, 0x01, b""))
    path.write_text("\n".join(lines) + "\n", encoding="ascii")


def _hex_record(addr: int, rtype: int, data: bytes) -> str:
    n = len(data)
    payload = bytes([n, (addr >> 8) & 0xFF, addr & 0xFF, rtype]) + data
    csum = ((-sum(payload)) & 0xFF)
    return ":" + payload.hex().upper() + f"{csum:02X}"


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--defaults", type=Path, required=True,
                   help="factory/*_eeprom_defaults.json")
    p.add_argument("--bootloader", type=Path, required=True,
                   help="Bootloaders/AM32_F051_BOOTLOADER_*.bin")
    p.add_argument("--app", type=Path, required=True,
                   help="obj/ARK32_ARK_4IN1_F051_*.bin application image")
    p.add_argument("--version-h", type=Path, default=Path("Inc/version.h"))
    p.add_argument("--out-bin", type=Path, required=True,
                   help="full 32 KiB flash image (.bin)")
    p.add_argument("--out-hex", type=Path, default=None,
                   help="optional Intel HEX of the full image")
    p.add_argument("--out-eeprom", type=Path, default=None,
                   help="optional 1 KiB eeprom page dump")
    args = p.parse_args(argv)

    defaults = json.loads(args.defaults.read_text(encoding="utf-8"))
    ver_maj, ver_min, eeprom_ver_h = _read_version_h(args.version_h)
    eeprom_ver = int(defaults.get("eeprom_version", eeprom_ver_h))
    if eeprom_ver != eeprom_ver_h:
        print(
            f"warning: defaults eeprom_version={eeprom_ver} != "
            f"version.h EEPROM_VERSION={eeprom_ver_h}",
            file=sys.stderr,
        )

    fmap = resolve_flash_map(defaults)
    eeprom_page = build_eeprom_page(defaults, ver_maj, ver_min, eeprom_ver,
                                    fmap["eeprom_page_size"])
    bootloader = args.bootloader.read_bytes()
    app = args.app.read_bytes()

    image = build_full_image(bootloader, app, eeprom_page, fmap)

    # PX4 globs the SD-card root for *.bin and flashes anything carrying a live
    # APDescriptor at the *app* base. A full-flash image starts at the
    # bootloader, so ingesting one would write 128 KiB off the end of G431
    # flash. Refuse to emit it under a name PX4 will pick up.
    if args.out_bin.name.endswith(".bin"):
        off = find_descriptor(image)
        if off >= 0:
            d = unpack_desc(image[off:off + DESC_LEN])
            if d["image_crc"] != 0:
                raise FactoryImageError(
                    f"{args.out_bin}: PX4 will ingest any SD-card-root *.bin "
                    f"with a live APDescriptor (found at 0x{off:x}, "
                    f"board_id={d['board_id']}). Write this image as "
                    f".factory.img (or .factory.hex), never .bin."
                )

    args.out_bin.parent.mkdir(parents=True, exist_ok=True)
    args.out_bin.write_bytes(image)
    if args.out_hex:
        args.out_hex.parent.mkdir(parents=True, exist_ok=True)
        write_intel_hex(args.out_hex, image)
    if args.out_eeprom:
        args.out_eeprom.parent.mkdir(parents=True, exist_ok=True)
        args.out_eeprom.write_bytes(eeprom_page)

    print(f"factory image: {args.out_bin} ({len(image)} bytes)")
    if args.out_hex:
        print(f"factory hex:   {args.out_hex}")
    if args.out_eeprom:
        print(f"eeprom page:   {args.out_eeprom}")
    print("eeprom defaults:")
    for line in decode_summary(eeprom_page):
        print(f"  {line}")
    print(
        f"regions: bl={len(bootloader)}B/{fmap['bl_region_size']}B "
        f"app={len(app)}B/{fmap['app_region_size']}B "
        f"eeprom@0x{fmap['eeprom_offset']:X} "
        f"settings={EEPROM_SETTINGS_SIZE}B page={fmap['eeprom_page_size']}B"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
