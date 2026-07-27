#!/usr/bin/env python3
"""Build a production full-flash image for ARK F051 ESCs.

Assembles a single binary that covers the entire STM32F051 32 KiB flash map:

  0x08000000  bootloader region (4 KiB)     — ARK32-bootloader .bin, 0xFF padded
  0x08001000  application region (27 KiB)  — make ARK_4IN1_F051 .bin, 0xFF padded
  0x08007C00  EEPROM settings page (1 KiB) — factory defaults, 0xFF padded

Production used to: flash BL, flash app, write EEPROM via AM32 configurator,
tweak settings, then ST-Link dump the whole chip. This script produces that
same image from the repo so a release is one `make factory-image` artifact.

EEPROM field encodings match Inc/eeprom.h and Src/settings.c.
"""

from __future__ import annotations

import argparse
import json
import re
import struct
import sys
from pathlib import Path

# STM32F051 flash map used by ARK_4IN1_F051 (Inc/targets.h, STM32F051K6TX_FLASH.ld)
FLASH_SIZE = 32 * 1024
BL_OFFSET = 0x0000
BL_REGION_SIZE = 4 * 1024
APP_OFFSET = 0x1000
APP_REGION_SIZE = 27 * 1024  # ends at EEPROM_OFFSET
EEPROM_OFFSET = 0x7C00
EEPROM_PAGE_SIZE = 1 * 1024
EEPROM_SETTINGS_SIZE = 192

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


def encode_motor_kv(kv: int) -> int:
    """eeprom motor_kv byte: runtime kv = eeprom * 40 + 20."""
    if kv < 20 or kv > 10220:
        raise FactoryImageError(f"motor_kv {kv} out of range 20..10220")
    stored = (kv - 20) // 40
    if stored * 40 + 20 != kv:
        # nearest legal value that encodes exactly
        nearest = stored * 40 + 20
        raise FactoryImageError(
            f"motor_kv {kv} is not representable exactly "
            f"(step 40 from 20); nearest lower is {nearest}"
        )
    return stored


def encode_max_ramp_percent_per_ms(pct: float) -> int:
    """eeprom max_ramp: for values >= 10, storage is percent_per_ms * 10."""
    if pct <= 0 or pct > 25.5:
        raise FactoryImageError(f"max_ramp_percent_per_ms {pct} out of range")
    # Prefer the coarse form (value >= 10) used by the configurator for
    # whole-percent rates: 2.0 %/ms -> 20.
    stored = int(round(pct * 10))
    if stored < 10:
        # fine mode is < 10 raw units (0.1 %/ms with ramp_divider); allow
        # explicit sub-1%/ms via fractional pct if someone needs it.
        fine = int(round(pct * 10))
        if fine < 1 or fine > 9:
            raise FactoryImageError(
                f"max_ramp_percent_per_ms {pct} does not encode cleanly"
            )
        return fine
    if abs(stored / 10.0 - pct) > 1e-6:
        raise FactoryImageError(
            f"max_ramp_percent_per_ms {pct} not representable (step 0.1)"
        )
    return stored


def encode_advance_degrees(degrees: int) -> int:
    """Fixed timing advance.

    Closed-loop advance is (temp_advance * commutation_interval) >> 6 on a
    60° step, so degrees ≈ temp_advance * 60/64. 15° => temp_advance 16.
    New-format eeprom stores temp_advance + 10 (range 10..42).
    """
    if degrees < 0 or degrees > 30:
        raise FactoryImageError(f"advance_degrees {degrees} out of range 0..30")
    # exact integer temp that best matches: temp = round(deg * 64 / 60)
    temp = int(round(degrees * 64 / 60.0))
    if temp > 32:
        temp = 32
    stored = temp + 10
    return stored


def encode_servo_low_us(us: int) -> int:
    # settings.c: servo_low_threshold = eeprom * 2 + 750
    if us < 750 or us > 750 + 2 * 255:
        raise FactoryImageError(f"pwm_input_min_us {us} out of encodable range")
    if (us - 750) % 2:
        raise FactoryImageError(f"pwm_input_min_us {us} must be even offset from 750")
    return (us - 750) // 2


def encode_servo_high_us(us: int) -> int:
    # settings.c: servo_high_threshold = eeprom * 2 + 1750
    if us < 1750 or us > 1750 + 2 * 255:
        raise FactoryImageError(f"pwm_input_max_us {us} out of encodable range")
    if (us - 1750) % 2:
        raise FactoryImageError(f"pwm_input_max_us {us} must be even offset from 1750")
    return (us - 1750) // 2


def encode_servo_neutral_us(us: int) -> int:
    # settings.c: servo_neutral = eeprom + 1374
    stored = us - 1374
    if stored < 0 or stored > 255:
        raise FactoryImageError(f"pwm_input_neutral_us {us} out of encodable range")
    return stored


def build_eeprom_page(defaults: dict, version_major: int, version_minor: int,
                      eeprom_version: int) -> bytes:
    """Build a 1 KiB EEPROM page (192-byte settings + 0xFF pad)."""
    s = defaults["settings"]
    buf = bytearray([0xFF] * EEPROM_PAGE_SIZE)

    # Offsets from Inc/eeprom.h. Start from upstream configurator skeleton
    # (Src/DroneCAN/DroneCAN.c default_settings) then overlay ARK production
    # values so unused / reserved bytes stay configurator-compatible.
    skeleton = bytes([
        0x01, 0x03, 0x01, 0x01, 0x23, 0xa0, 0x04, 0x00, 0x0a, 0x64, 0x00, 0x32,
        0x02, 0x30, 0x35, 0x31, 0x20, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x1a,
        0x18, 0x64, 0x37, 0x0e, 0x00, 0x00, 0x05, 0x00, 0x80, 0x80, 0x80, 0x32,
        0x00, 0x32, 0x00, 0x00, 0x0f, 0x0a, 0x0a, 0x8d, 0x66, 0x06, 0x01, 0x00,
    ])
    buf[0:len(skeleton)] = skeleton

    def put(off: int, val: int) -> None:
        if not 0 <= val <= 255:
            raise FactoryImageError(f"byte at offset {off} out of range: {val}")
        buf[off] = val

    put(1, eeprom_version)
    put(3, version_major)
    put(4, version_minor)

    put(5, encode_max_ramp_percent_per_ms(float(s["max_ramp_percent_per_ms"])))
    put(6, int(s["minimum_duty_cycle"]))
    put(7, int(s["disable_stick_calibration"]))
    put(8, int(s["absolute_voltage_cutoff"]))
    put(9, int(s["current_P"]))
    put(10, int(s["current_I"]))
    put(11, int(s["current_D"]))
    put(12, int(s["active_brake_power"]))
    # 13-16: leave skeleton "051 " MCU tag

    put(17, int(s["dir_reversed"]))
    put(18, int(s["bi_direction"]))
    put(19, int(s["use_sine_start"]))
    put(20, int(s["comp_pwm"]))
    put(21, int(s["variable_pwm"]))
    put(22, int(s["stuck_rotor_protection"]))
    put(23, encode_advance_degrees(int(s["advance_degrees"])))
    put(24, int(s["pwm_frequency_khz"]))
    put(25, int(s["startup_power"]))
    put(26, encode_motor_kv(int(s["motor_kv"])))
    put(27, int(s["motor_poles"]))
    put(28, int(s["brake_on_stop"]))
    put(29, int(s["stall_protection"]))
    put(30, int(s["beep_volume"]))
    put(31, int(s["telemetry_on_interval"]))
    put(32, encode_servo_low_us(int(s["pwm_input_min_us"])))
    put(33, encode_servo_high_us(int(s["pwm_input_max_us"])))
    put(34, encode_servo_neutral_us(int(s["pwm_input_neutral_us"])))
    put(35, int(s["pwm_input_deadband"]))
    put(36, int(s["low_voltage_cut_off"]))
    put(37, int(s["low_cell_volt_cutoff_offset"]))
    put(40, int(s["sine_mode_changeover_throttle_level"]))
    put(41, int(s["drag_brake_strength"]))
    put(42, int(s["driving_brake_strength"]))
    put(43, int(s["temperature_limit"]))
    put(44, int(s["current_limit"]))
    put(45, int(s["sine_mode_power"]))
    put(46, int(s["input_type"]))
    put(47, int(s["auto_advance"]))

    return bytes(buf)


def decode_summary(page: bytes) -> list[str]:
    """Human-readable check of the first settings bytes."""
    eeprom_ver = page[1]
    maj, minor = page[3], page[4]
    max_ramp = page[5]
    ramp = f"{max_ramp / 10.0:g} %/ms" if max_ramp >= 10 else f"fine raw {max_ramp}"
    kv = page[26] * 40 + 20
    adv_temp = page[23] - 10 if page[23] >= 10 else page[23]
    adv_deg = adv_temp * 60 / 64.0
    servo_lo = page[32] * 2 + 750
    servo_hi = page[33] * 2 + 1750
    return [
        f"eeprom_version={eeprom_ver} fw={maj}.{minor}",
        f"max_ramp={max_ramp} ({ramp})",
        f"motor_kv_byte={page[26]} -> {kv} kV, poles={page[27]}",
        f"variable_pwm={page[21]} (1=by RPM), auto_advance={page[47]}",
        f"advance_level={page[23]} -> ~{adv_deg:.1f}° fixed",
        f"pwm_input min={servo_lo} us max={servo_hi} us",
        f"input_type={page[46]} pwm_freq={page[24]} kHz",
    ]


def place_region(image: bytearray, offset: int, region_size: int, data: bytes,
                 name: str) -> None:
    if len(data) > region_size:
        raise FactoryImageError(
            f"{name}: {len(data)} bytes exceeds region size {region_size} "
            f"(offset 0x{offset:04X})"
        )
    image[offset:offset + len(data)] = data
    # remainder of region already 0xFF


def build_full_image(bootloader: bytes, app: bytes, eeprom_page: bytes) -> bytes:
    image = bytearray([0xFF] * FLASH_SIZE)
    place_region(image, BL_OFFSET, BL_REGION_SIZE, bootloader, "bootloader")
    place_region(image, APP_OFFSET, APP_REGION_SIZE, app, "application")
    place_region(image, EEPROM_OFFSET, EEPROM_PAGE_SIZE, eeprom_page, "eeprom")
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
                   help="obj/AM32_ARK_4IN1_F051_*.bin application image")
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

    eeprom_page = build_eeprom_page(defaults, ver_maj, ver_min, eeprom_ver)
    bootloader = args.bootloader.read_bytes()
    app = args.app.read_bytes()

    image = build_full_image(bootloader, app, eeprom_page)

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
        f"regions: bl={len(bootloader)}B app={len(app)}B "
        f"eeprom_settings={EEPROM_SETTINGS_SIZE}B"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
