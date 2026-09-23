import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location('release_manifest', Path(__file__).resolve().parents[1] / 'release_manifest.py')
release = importlib.util.module_from_spec(spec)
spec.loader.exec_module(release)


class ReleaseManifestTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        for name in ('Inc', 'Bootloaders', 'factory', 'schema', 'obj'):
            (self.root / name).mkdir()
        (self.root / 'Inc/version.h').write_text('#define VERSION_MAJOR 32\n#define VERSION_MINOR 0\n#define EEPROM_VERSION 3\n')
        self.bootloader = self.root / 'Bootloaders/bootloader.bin'
        self.bootloader.write_bytes(b'bootloader')
        (self.root / 'factory/defaults.json').write_text('{}')
        (self.root / 'schema/eeprom.json').write_text('{"fields": {}}')
        self.obj = self.root / 'obj'
        self.prefix = 'ARK32_ARK_4IN1_F051_32.0'
        for suffix, size in (('.hex', 10), ('.bin', 20), ('.factory.hex', 30), ('.eeprom.bin', 1024)):
            (self.obj / (self.prefix + suffix)).write_bytes(bytes(size))
        (self.obj / (self.prefix + '.factory.bin')).write_bytes(b'bootloader'.ljust(32768, b'\xff'))

    def package(self):
        with patch.object(release.subprocess, 'check_output', side_effect=['a' * 40 + '\n', 'arm gcc 15.2.1\n']):
            return release.package(self.root, self.obj, Path('/compiler'), '32.0', 'ARK_4IN1_F051', self.bootloader,
                                   self.root / 'factory/defaults.json', self.root / 'schema/eeprom.json')

    def test_complete_release_hashes_and_provenance(self):
        manifest = self.package()
        self.assertEqual(manifest['commit'], 'a' * 40)
        self.assertEqual(manifest['firmware_version'], '32.0')
        self.assertEqual(manifest['eeprom_layout'], 3)
        self.assertEqual(manifest['bootloader']['path'], 'Bootloaders/bootloader.bin')
        self.assertEqual(len(manifest['assets']), 6)
        self.assertEqual((self.obj / 'eeprom.json').read_text(), '{"fields": {}}')
        names = set()
        for line in (self.obj / 'SHA256SUMS').read_text().splitlines():
            digest, name = line.split('  ')
            self.assertEqual(digest, release.sha256(self.obj / name))
            names.add(name)
        self.assertIn('eeprom.json', names)
        self.assertEqual(json.loads((self.obj / 'release-manifest.json').read_text()), manifest)

    def test_missing_factory_image_rejected(self):
        (self.obj / (self.prefix + '.factory.bin')).unlink()
        with self.assertRaisesRegex(ValueError, 'missing or empty'):
            self.package()

    def test_truncated_factory_image_rejected(self):
        (self.obj / (self.prefix + '.factory.bin')).write_bytes(b'bad')
        with self.assertRaisesRegex(ValueError, '32 KiB'):
            self.package()

    def test_other_bootloader_rejected(self):
        self.bootloader.write_bytes(b'newer bootloader')
        with self.assertRaisesRegex(ValueError, 'does not start with'):
            self.package()

    def test_stale_asset_rejected(self):
        (self.obj / 'ARK32_ARK_4IN1_F051_31.0.hex').write_bytes(b'old')
        with self.assertRaisesRegex(ValueError, 'unexpected binary'):
            self.package()


if __name__ == '__main__':
    unittest.main()
