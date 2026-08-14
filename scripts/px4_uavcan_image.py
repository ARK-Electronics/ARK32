#!/usr/bin/env python3
"""PX4 SD-card DroneCAN image: sign APDescriptor and emit the UAVCAN filename.

PX4's UAVCAN firmware updater (src/drivers/uavcan/uavcan_servers.cpp):

  * Accepts a .bin on the SD-card root (or ufw_staging/) only if the first
    1 KiB contains an APDescriptor whose 8-byte signature is
    {0x40,0xa2,0xe4,0xf1,0x64,0x68,0x91,0x06} and image_crc != 0.
  * Copies it to /ufw/<board_id>.bin, where
    board_id = (hardware_version.major << 8) | hardware_version.minor.
  * Flashes any node whose GetNodeInfo hw version matches and whose
    reported image_crc differs (or is 0).

The filename uses the numeric ship version from Inc/version.h
(MAJOR.MINOR[.PATCH]), not the -ark artifact suffix and not the git hash
PX4 cannode builds put in the third field:

    <board_id>-<MAJOR.MINOR[.PATCH]>.uavcan.bin

e.g. 71-3.0.2.uavcan.bin  (tag v3.0.2-ark)

PX4 matches the file by the APDescriptor board_id, not by parsing the
name. The ship version is for humans and for FW.db.

Sign *before* Src/DroneCAN/set_app_signature.py. The AM32 bootloader CRC
covers the whole image except its own 44-byte block; filling the PX4 CRC
fields after that would invalidate the AM32 signature. PX4 itself only
compares the stored image_crc to GetNodeInfo — it does not recompute it —
so AM32-signing last (which changes bytes inside the PX4 CRC coverage) is
safe for the SD-card workflow.

Usage:
  scripts/px4_uavcan_image.py sign  <bin> <elf>
  scripts/px4_uavcan_image.py emit  <bin>
  scripts/px4_uavcan_image.py check <bin>
  scripts/px4_uavcan_image.py --selftest
"""

from __future__ import annotations

import argparse
import struct
import subprocess
import sys
import re
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
VERSION_H = REPO_ROOT / "Inc" / "version.h"

SIGNATURE = bytes((0x40, 0xA2, 0xE4, 0xF1, 0x64, 0x68, 0x91, 0x06))
RESERVED = b"\xFF" * 8
# signature[8] + crc1 + crc2 + size + git + maj + min + board + reserved[8]
DESC_LEN = 36
# crc32_block1, crc32_block2, image_size, git_hash
CRC_FIELDS_OFF = 8
CRC_FIELDS_LEN = 16
# PX4 migrateFWFromRoot only scans this many bytes of an SD-card-root .bin.
PX4_SCAN_LIMIT = 1024

# Same table as PX4 src/drivers/bootloaders/make_can_boot_descriptor.py
_CRCTAB = (
    0x00000000,
    0x77073096,
    0xEE0E612C,
    0x990951BA,
    0x076DC419,
    0x706AF48F,
    0xE963A535,
    0x9E6495A3,
    0x0EDB8832,
    0x79DCB8A4,
    0xE0D5E91E,
    0x97D2D988,
    0x09B64C2B,
    0x7EB17CBD,
    0xE7B82D07,
    0x90BF1D91,
    0x1DB71064,
    0x6AB020F2,
    0xF3B97148,
    0x84BE41DE,
    0x1ADAD47D,
    0x6DDDE4EB,
    0xF4D4B551,
    0x83D385C7,
    0x136C9856,
    0x646BA8C0,
    0xFD62F97A,
    0x8A65C9EC,
    0x14015C4F,
    0x63066CD9,
    0xFA0F3D63,
    0x8D080DF5,
    0x3B6E20C8,
    0x4C69105E,
    0xD56041E4,
    0xA2677172,
    0x3C03E4D1,
    0x4B04D447,
    0xD20D85FD,
    0xA50AB56B,
    0x35B5A8FA,
    0x42B2986C,
    0xDBBBC9D6,
    0xACBCF940,
    0x32D86CE3,
    0x45DF5C75,
    0xDCD60DCF,
    0xABD13D59,
    0x26D930AC,
    0x51DE003A,
    0xC8D75180,
    0xBFD06116,
    0x21B4F4B5,
    0x56B3C423,
    0xCFBA9599,
    0xB8BDA50F,
    0x2802B89E,
    0x5F058808,
    0xC60CD9B2,
    0xB10BE924,
    0x2F6F7C87,
    0x58684C11,
    0xC1611DAB,
    0xB6662D3D,
    0x76DC4190,
    0x01DB7106,
    0x98D220BC,
    0xEFD5102A,
    0x71B18589,
    0x06B6B51F,
    0x9FBFE4A5,
    0xE8B8D433,
    0x7807C9A2,
    0x0F00F934,
    0x9609A88E,
    0xE10E9818,
    0x7F6A0DBB,
    0x086D3D2D,
    0x91646C97,
    0xE6635C01,
    0x6B6B51F4,
    0x1C6C6162,
    0x856530D8,
    0xF262004E,
    0x6C0695ED,
    0x1B01A57B,
    0x8208F4C1,
    0xF50FC457,
    0x65B0D9C6,
    0x12B7E950,
    0x8BBEB8EA,
    0xFCB9887C,
    0x62DD1DDF,
    0x15DA2D49,
    0x8CD37CF3,
    0xFBD44C65,
    0x4DB26158,
    0x3AB551CE,
    0xA3BC0074,
    0xD4BB30E2,
    0x4ADFA541,
    0x3DD895D7,
    0xA4D1C46D,
    0xD3D6F4FB,
    0x4369E96A,
    0x346ED9FC,
    0xAD678846,
    0xDA60B8D0,
    0x44042D73,
    0x33031DE5,
    0xAA0A4C5F,
    0xDD0D7CC9,
    0x5005713C,
    0x270241AA,
    0xBE0B1010,
    0xC90C2086,
    0x5768B525,
    0x206F85B3,
    0xB966D409,
    0xCE61E49F,
    0x5EDEF90E,
    0x29D9C998,
    0xB0D09822,
    0xC7D7A8B4,
    0x59B33D17,
    0x2EB40D81,
    0xB7BD5C3B,
    0xC0BA6CAD,
    0xEDB88320,
    0x9ABFB3B6,
    0x03B6E20C,
    0x74B1D29A,
    0xEAD54739,
    0x9DD277AF,
    0x04DB2615,
    0x73DC1683,
    0xE3630B12,
    0x94643B84,
    0x0D6D6A3E,
    0x7A6A5AA8,
    0xE40ECF0B,
    0x9309FF9D,
    0x0A00AE27,
    0x7D079EB1,
    0xF00F9344,
    0x8708A3D2,
    0x1E01F268,
    0x6906C2FE,
    0xF762575D,
    0x806567CB,
    0x196C3671,
    0x6E6B06E7,
    0xFED41B76,
    0x89D32BE0,
    0x10DA7A5A,
    0x67DD4ACC,
    0xF9B9DF6F,
    0x8EBEEFF9,
    0x17B7BE43,
    0x60B08ED5,
    0xD6D6A3E8,
    0xA1D1937E,
    0x38D8C2C4,
    0x4FDFF252,
    0xD1BB67F1,
    0xA6BC5767,
    0x3FB506DD,
    0x48B2364B,
    0xD80D2BDA,
    0xAF0A1B4C,
    0x36034AF6,
    0x41047A60,
    0xDF60EFC3,
    0xA867DF55,
    0x316E8EEF,
    0x4669BE79,
    0xCB61B38C,
    0xBC66831A,
    0x256FD2A0,
    0x5268E236,
    0xCC0C7795,
    0xBB0B4703,
    0x220216B9,
    0x5505262F,
    0xC5BA3BBE,
    0xB2BD0B28,
    0x2BB45A92,
    0x5CB36A04,
    0xC2D7FFA7,
    0xB5D0CF31,
    0x2CD99E8B,
    0x5BDEAE1D,
    0x9B64C2B0,
    0xEC63F226,
    0x756AA39C,
    0x026D930A,
    0x9C0906A9,
    0xEB0E363F,
    0x72076785,
    0x05005713,
    0x95BF4A82,
    0xE2B87A14,
    0x7BB12BAE,
    0x0CB61B38,
    0x92D28E9B,
    0xE5D5BE0D,
    0x7CDCEFB7,
    0x0BDBDF21,
    0x86D3D2D4,
    0xF1D4E242,
    0x68DDB3F8,
    0x1FDA836E,
    0x81BE16CD,
    0xF6B9265B,
    0x6FB077E1,
    0x18B74777,
    0x88085AE6,
    0xFF0F6A70,
    0x66063BCA,
    0x11010B5C,
    0x8F659EFF,
    0xF862AE69,
    0x616BFFD3,
    0x166CCF45,
    0xA00AE278,
    0xD70DD2EE,
    0x4E048354,
    0x3903B3C2,
    0xA7672661,
    0xD06016F7,
    0x4969474D,
    0x3E6E77DB,
    0xAED16A4A,
    0xD9D65ADC,
    0x40DF0B66,
    0x37D83BF0,
    0xA9BCAE53,
    0xDEBB9EC5,
    0x47B2CF7F,
    0x30B5FFE9,
    0xBDBDF21C,
    0xCABAC28A,
    0x53B39330,
    0x24B4A3A6,
    0xBAD03605,
    0xCDD70693,
    0x54DE5729,
    0x23D967BF,
    0xB3667A2E,
    0xC4614AB8,
    0x5D681B02,
    0x2A6F2B94,
    0xB40BBE37,
    0xC30C8EA1,
    0x5A05DF1B,
    0x2D02EF8D,
)


def crc32_px4(data: bytes, crc: int = 0) -> int:
    for byte in data:
        crc = _CRCTAB[(crc ^ byte) & 0xFF] ^ (crc >> 8)
    return crc & 0xFFFFFFFF


def unpack_desc(blob: bytes) -> dict:
    if len(blob) != DESC_LEN:
        raise ValueError("descriptor length")
    sig, c1, c2, size, git, maj, mn, board, reserved = struct.unpack("<8sIIIIBBH8s", blob)
    return {
        "signature": sig,
        "crc32_block1": c1,
        "crc32_block2": c2,
        "image_crc": c1 | (c2 << 32),
        "image_size": size,
        "git_hash": git,
        "major_version": maj,
        "minor_version": mn,
        "board_id": board,
        "reserved": reserved,
    }


def pack_desc(d: dict) -> bytes:
    return struct.pack(
        "<8sIIIIBBH8s",
        d["signature"],
        d["crc32_block1"],
        d["crc32_block2"],
        d["image_size"],
        d["git_hash"],
        d["major_version"],
        d["minor_version"],
        d["board_id"],
        d["reserved"],
    )


def find_descriptor(img: bytes, limit: int | None = None) -> int:
    """Return 8-byte-aligned offset of APDesc00, or -1."""
    end = len(img) if limit is None else min(len(img), limit)
    # Signature is 8 bytes; search 8-aligned so we match PX4's uint64 scan.
    last = end - DESC_LEN
    off = 0
    while off <= last:
        if img[off : off + 8] == SIGNATURE:
            return off
        off += 8
    return -1


def firmware_version(version_h: Path = VERSION_H) -> str:
    """MAJOR.MINOR[.PATCH] — numeric ship version, no -ark suffix."""
    text = version_h.read_text(encoding="utf-8")

    def num(name: str) -> str | None:
        m = re.search(rf"^\s*#define\s+{name}\s+(\d+)\s*$", text, re.M)
        return m.group(1) if m else None

    major = num("VERSION_MAJOR")
    minor = num("VERSION_MINOR")
    if major is None or minor is None:
        raise SystemExit(f"{version_h}: missing VERSION_MAJOR / VERSION_MINOR")
    ver = f"{major}.{minor}"
    patch = num("VERSION_PATCH")
    if patch is not None:
        ver += f".{patch}"
    return ver


def uavcan_name(d: dict, version: str | None = None) -> str:
    ver = version if version is not None else firmware_version()
    return f"{d['board_id']}-{ver}.uavcan.bin"


def git_abbrev8(cwd: str | None = None) -> int:
    try:
        out = subprocess.check_output(
            ["git", "rev-list", "HEAD", "--max-count=1", "--abbrev=8", "--abbrev-commit"],
            cwd=cwd,
            stderr=subprocess.DEVNULL,
            text=True,
        ).strip()
        if out:
            return int(out, 16)
    except (subprocess.CalledProcessError, FileNotFoundError, ValueError):
        pass
    return 0


def apply_descriptor(img: bytearray, off: int, git_hash: int) -> dict:
    d = unpack_desc(bytes(img[off : off + DESC_LEN]))
    if d["signature"] != SIGNATURE:
        raise ValueError("bad APDescriptor signature")
    if d["reserved"] != RESERVED:
        raise ValueError("APDescriptor reserved bytes must be 0xFF")
    if d["board_id"] == 0:
        raise ValueError("APDescriptor board_id is 0")

    d["git_hash"] = git_hash
    d["image_size"] = len(img)
    d["crc32_block1"] = 0
    d["crc32_block2"] = 0
    img[off : off + DESC_LEN] = pack_desc(d)

    d["crc32_block1"] = crc32_px4(bytes(img[: off + 8]))
    d["crc32_block2"] = crc32_px4(bytes(img[off + CRC_FIELDS_OFF + CRC_FIELDS_LEN :]))
    d["image_crc"] = d["crc32_block1"] | (d["crc32_block2"] << 32)
    img[off : off + DESC_LEN] = pack_desc(d)
    return d


def patch_elf(elf_path: Path, img: bytes, off: int) -> None:
    """Copy the APDescriptor from the signed bin into the ELF."""
    elf = bytearray(elf_path.read_bytes())
    elf_off = find_descriptor(elf)
    if elf_off < 0:
        raise SystemExit(f"No APDescriptor found in {elf_path}")
    elf[elf_off : elf_off + DESC_LEN] = img[off : off + DESC_LEN]
    elf_path.write_bytes(elf)


def cmd_sign(bin_path: Path, elf_path: Path, git_hash: int | None) -> int:
    img = bytearray(bin_path.read_bytes())
    off = find_descriptor(img)
    if off < 0:
        return 0
    if off + DESC_LEN > PX4_SCAN_LIMIT:
        raise SystemExit(
            f"APDescriptor at 0x{off:x} is past the {PX4_SCAN_LIMIT}-byte "
            "window PX4 scans on the SD-card root"
        )
    if git_hash is None:
        git_hash = git_abbrev8()
    d = apply_descriptor(img, off, git_hash)
    bin_path.write_bytes(img)
    patch_elf(elf_path, img, off)
    print(
        f"Signed PX4 APDescriptor board_id={d['board_id']} "
        f"sw={d['major_version']}.{d['minor_version']} "
        f"git={d['git_hash']:08x} crc={d['image_crc']:016x} "
        f"at 0x{off:x} in {bin_path}"
    )
    return 0


def cmd_emit(bin_path: Path, version: str | None = None) -> int:
    img = bin_path.read_bytes()
    off = find_descriptor(img)
    if off < 0:
        return 0
    d = unpack_desc(img[off : off + DESC_LEN])
    if d["image_crc"] == 0:
        raise SystemExit(f"{bin_path}: APDescriptor image_crc is 0 (not signed)")
    dest = bin_path.parent / uavcan_name(d, version)
    dest.write_bytes(img)
    print(f"PX4 SD-card image {dest}")
    return 0


def cmd_check(bin_path: Path, version: str | None = None) -> int:
    img = bin_path.read_bytes()
    off = find_descriptor(img, PX4_SCAN_LIMIT)
    if off < 0:
        raise SystemExit(f"{bin_path}: no APDescriptor in first {PX4_SCAN_LIMIT} bytes")
    d = unpack_desc(img[off : off + DESC_LEN])
    errors: list[str] = []
    if d["image_crc"] == 0:
        errors.append("image_crc is 0; PX4 will ignore the file")
    if d["board_id"] == 0:
        errors.append("board_id is 0")
    if d["reserved"] != RESERVED:
        errors.append("reserved bytes are not 0xFF")
    if off % 8:
        errors.append(f"descriptor at 0x{off:x} is not 8-byte aligned")
    # Must sit entirely inside one 512-byte PX4 read chunk.
    chunk = off & ~511
    if off + DESC_LEN > chunk + 512:
        errors.append(f"descriptor at 0x{off:x} straddles a 512-byte PX4 read")

    expected = bin_path.parent / uavcan_name(d, version)
    if not expected.is_file():
        errors.append(f"missing {expected.name}")
    else:
        other = expected.read_bytes()
        if other != img:
            errors.append(f"{expected.name} does not match {bin_path.name}")

    if errors:
        print(f"PX4 UAVCAN image check FAILED ({bin_path}):", file=sys.stderr)
        for e in errors:
            print(f"  - {e}", file=sys.stderr)
        return 1
    print(
        f"OK {expected.name} board_id={d['board_id']} "
        f"sw={d['major_version']}.{d['minor_version']} "
        f"git={d['git_hash']:08x} off=0x{off:x}"
    )
    return 0


def selftest() -> int:
    # 600-byte fake image: descriptor at 520 (second 512-byte chunk, first 1 KiB).
    img = bytearray(600)
    off = 520
    proto = {
        "signature": SIGNATURE,
        "crc32_block1": 0,
        "crc32_block2": 0,
        "image_size": 0,
        "git_hash": 0,
        "major_version": 3,
        "minor_version": 0,
        "board_id": 71,
        "reserved": RESERVED,
    }
    img[off : off + DESC_LEN] = pack_desc(proto)
    d = apply_descriptor(img, off, 0x59EFC137)
    assert d["git_hash"] == 0x59EFC137
    assert d["board_id"] == 71
    assert d["image_crc"] != 0
    assert d["image_size"] == 600
    assert uavcan_name(d, "3.0.2") == "71-3.0.2.uavcan.bin"
    assert firmware_version() == "3.0.2"
    assert find_descriptor(bytes(img), PX4_SCAN_LIMIT) == off
    # Signature at a non-8-aligned offset must not be found (PX4 uint64 scan).
    bad = bytearray(64)
    bad[3 : 11] = SIGNATURE
    assert find_descriptor(bytes(bad)) < 0
    print("px4_uavcan_image selftest OK")
    return 0


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--selftest", action="store_true")
    p.add_argument("--git-hash", dest="git_hash", default=None, help="8-hex-digit VCS commit (default: git)")
    p.add_argument(
        "--version",
        dest="version",
        default=None,
        help="numeric ship version for the .uavcan.bin name (default: MAJOR.MINOR[.PATCH] from Inc/version.h)",
    )
    p.add_argument("cmd", nargs="?", choices=("sign", "emit", "check"))
    p.add_argument("binfile", nargs="?", type=Path)
    p.add_argument("elffile", nargs="?", type=Path)
    args = p.parse_args(argv)

    if args.selftest:
        return selftest()
    if not args.cmd or not args.binfile:
        p.error("cmd and binfile required (or --selftest)")
    if args.cmd == "sign":
        if not args.elffile:
            p.error("sign requires the elf file")
        git_hash = int(args.git_hash, 16) if args.git_hash else None
        return cmd_sign(args.binfile, args.elffile, git_hash)
    if args.cmd == "emit":
        return cmd_emit(args.binfile, args.version)
    return cmd_check(args.binfile, args.version)


if __name__ == "__main__":
    sys.exit(main())
