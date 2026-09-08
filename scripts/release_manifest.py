#!/usr/bin/env python3
"""Package the supported ARK 4IN1 release with source/toolchain provenance."""

import argparse
import hashlib
import json
import re
import subprocess
from pathlib import Path


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def package(root, compiler, obj):
    defines = dict(re.findall(r'^\s*#\s*define\s+(VERSION_MAJOR|VERSION_MINOR|VERSION_PATCH|EEPROM_VERSION)\s+(\d+)',
                              (root / 'Inc/version.h').read_text(), re.MULTILINE))
    version = '.'.join(defines[k] for k in ('VERSION_MAJOR', 'VERSION_MINOR', 'VERSION_PATCH') if k in defines)
    tag = re.search(r'^\s*#\s*define\s+VERSION_TAG\s+"([^"\n]+)"',
                    (root / 'Inc/version.h').read_text(), re.MULTILINE)
    if tag:
        version += '-' + tag.group(1)
    target = 'ARK_4IN1_F051'
    prefix = 'ARK32_' + target + '_' + version
    assets = [obj / (prefix + suffix) for suffix in
              ('.hex', '.bin', '.factory.hex', '.factory.bin', '.eeprom.bin')]
    for asset in assets:
        if not asset.is_file() or asset.stat().st_size == 0:
            raise ValueError('missing or empty release asset: ' + str(asset))
    if (obj / (prefix + '.factory.bin')).stat().st_size != 32768:
        raise ValueError('factory image must cover exactly 32 KiB')
    if (obj / (prefix + '.eeprom.bin')).stat().st_size != 1024:
        raise ValueError('EEPROM image must cover exactly 1 KiB')
    # Reject stale binaries instead of silently publishing an unrecorded file.
    actual = set(obj.glob('*.hex')) | set(obj.glob('*.bin'))
    if actual != set(assets):
        raise ValueError('unexpected binary assets: ' + ', '.join(sorted(str(p) for p in actual - set(assets))))
    commit = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=root, text=True).strip()
    compiler_version = subprocess.check_output([str(compiler), '--version'], text=True).splitlines()[0]
    bootloader = root / 'Bootloaders/AM32_F051_BOOTLOADER_ARK4IN1_V18.bin'
    manifest = {
        'manifest_version': 1, 'commit': commit, 'target': target, 'firmware_version': version,
        'compiler': compiler_version, 'eeprom_layout': int(defines['EEPROM_VERSION']),
        'bootloader': {'path': str(bootloader.relative_to(root)), 'sha256': sha256(bootloader)},
        'factory_defaults_sha256': sha256(root / 'factory/ARK_4IN1_F051_eeprom_defaults.json'),
        'flash_addresses': {'factory': '0x08000000', 'application': '0x08001000', 'eeprom': '0x08007C00'},
        'assets': [{'name': p.name, 'size': p.stat().st_size, 'sha256': sha256(p)} for p in assets],
    }
    manifest_path = obj / 'release-manifest.json'
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + '\n')
    checksummed = sorted(assets + [manifest_path])
    (obj / 'SHA256SUMS').write_text(''.join(sha256(p) + '  ' + p.name + '\n' for p in checksummed))
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compiler', required=True)
    parser.add_argument('--obj', default='obj')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    package(root, Path(args.compiler).resolve(), (root / args.obj).resolve())


if __name__ == '__main__':
    main()
