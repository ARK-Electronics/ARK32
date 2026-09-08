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
        for name in ('Inc', 'Bootloaders', 'factory', 'obj'):
            (self.root / name).mkdir()
        (self.root / 'Inc/version.h').write_text('#define VERSION_MAJOR 3\n#define VERSION_MINOR 0\n#define VERSION_PATCH 3\n#define EEPROM_VERSION 3\n')
        (self.root / 'Bootloaders/AM32_F051_BOOTLOADER_ARK4IN1_V18.bin').write_bytes(b'bootloader')
        (self.root / 'factory/ARK_4IN1_F051_eeprom_defaults.json').write_text('{}')
        self.obj = self.root / 'obj'
        self.prefix = 'ARK32_ARK_4IN1_F051_3.0.3'
        for suffix, size in (('.hex', 10), ('.bin', 20), ('.factory.hex', 30), ('.factory.bin', 32768), ('.eeprom.bin', 1024)):
            (self.obj / (self.prefix + suffix)).write_bytes(bytes(size))

    def package(self):
        with patch.object(release.subprocess, 'check_output', side_effect=['a' * 40 + '\n', 'arm gcc 15.2.1\n']):
            return release.package(self.root, Path('/compiler'), self.obj)

    def test_complete_release_hashes_and_provenance(self):
        manifest = self.package()
        self.assertEqual(manifest['commit'], 'a' * 40)
        self.assertEqual(manifest['eeprom_layout'], 3)
        self.assertEqual(len(manifest['assets']), 5)
        for line in (self.obj / 'SHA256SUMS').read_text().splitlines():
            digest, name = line.split('  ')
            self.assertEqual(digest, release.sha256(self.obj / name))
        self.assertEqual(json.loads((self.obj / 'release-manifest.json').read_text()), manifest)

    def test_missing_factory_image_rejected(self):
        (self.obj / (self.prefix + '.factory.bin')).unlink()
        with self.assertRaisesRegex(ValueError, 'missing or empty'):
            self.package()

    def test_truncated_factory_image_rejected(self):
        (self.obj / (self.prefix + '.factory.bin')).write_bytes(b'bad')
        with self.assertRaisesRegex(ValueError, '32 KiB'):
            self.package()

    def test_stale_asset_rejected(self):
        (self.obj / 'ARK32_ARK_4IN1_F051_2.0.hex').write_bytes(b'old')
        with self.assertRaisesRegex(ValueError, 'unexpected binary'):
            self.package()


if __name__ == '__main__':
    unittest.main()
