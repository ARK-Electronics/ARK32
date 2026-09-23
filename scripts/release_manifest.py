#!/usr/bin/env python3
"""Package the supported ARK 4IN1 release with source/toolchain provenance.

Run through `make release-manifest`, which passes the compiler, version,
bootloader and defaults the build used.
"""

import argparse
import hashlib
import json
import re
import shutil
import subprocess
from pathlib import Path


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def package(root, obj, compiler, version, target, bootloader, defaults, schema):
    eeprom_layout = re.search(r'^\s*#\s*define\s+EEPROM_VERSION\s+(\d+)',
                              (root / 'Inc/version.h').read_text(), re.MULTILINE)
    prefix = 'ARK32_' + target + '_' + version
    binaries = [obj / (prefix + suffix) for suffix in
                ('.hex', '.bin', '.factory.hex', '.factory.bin', '.eeprom.bin')]
    for asset in binaries:
        if not asset.is_file() or asset.stat().st_size == 0:
            raise ValueError('missing or empty release asset: ' + str(asset))
    factory = (obj / (prefix + '.factory.bin')).read_bytes()
    if len(factory) != 32768:
        raise ValueError('factory image must cover exactly 32 KiB')
    if (obj / (prefix + '.eeprom.bin')).stat().st_size != 1024:
        raise ValueError('EEPROM image must cover exactly 1 KiB')
    # The manifest names this bootloader; prove it is the one inside the image.
    if not factory.startswith(bootloader.read_bytes()):
        raise ValueError('factory image does not start with ' + str(bootloader))
    # Reject stale binaries instead of silently publishing an unrecorded file.
    actual = set(obj.glob('*.hex')) | set(obj.glob('*.bin'))
    if actual != set(binaries):
        raise ValueError('unexpected binary assets: ' + ', '.join(sorted(str(p) for p in actual - set(binaries))))
    # Published under the asset name the configurator fetches.
    eeprom_json = obj / 'eeprom.json'
    shutil.copyfile(schema, eeprom_json)
    assets = binaries + [eeprom_json]
    commit = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=root, text=True).strip()
    compiler_version = subprocess.check_output([str(compiler), '--version'], text=True).splitlines()[0]
    manifest = {
        'manifest_version': 1, 'commit': commit, 'target': target, 'firmware_version': version,
        'compiler': compiler_version, 'eeprom_layout': int(eeprom_layout.group(1)),
        'bootloader': {'path': str(bootloader.relative_to(root)), 'sha256': sha256(bootloader)},
        'factory_defaults_sha256': sha256(defaults),
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
    for name in ('compiler', 'version', 'target', 'bootloader', 'defaults', 'schema'):
        parser.add_argument('--' + name, required=True)
    parser.add_argument('--obj', default='obj')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    package(root, root / args.obj, Path(args.compiler), args.version, args.target,
            root / args.bootloader, root / args.defaults, root / args.schema)


if __name__ == '__main__':
    main()
