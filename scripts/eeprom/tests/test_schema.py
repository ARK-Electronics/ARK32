"""Language/compatibility tests: histories, rejected edits and exact factory IO."""
import copy
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

SCRIPTS = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SCRIPTS / 'eeprom'))
sys.path.insert(0, str(SCRIPTS))
from schema import (REPO_ROOT, default_bytes, from_raw, load_schema, resolve_field,
                    resolve_schema, to_raw, validate_json_schema, validate_layout)
from build_factory_image import FactoryImageError, build_eeprom_page, encode_advance_degrees, encode_motor_kv


class SchemaTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.document = load_schema()

    def test_firmware_tuple_and_enum_history(self):
        old = resolve_schema(self.document, 3, '2.16')['fields']
        new = resolve_schema(self.document, 3, '32.0')['fields']
        self.assertEqual(old['motorPoles']['raw']['max'], 64)
        self.assertEqual(new['motorPoles']['raw'], {'min': 2, 'max': 128})
        self.assertEqual([v['raw'] for v in old['variablePwmFreq']['values']], [0, 1])
        self.assertEqual([v['raw'] for v in new['variablePwmFreq']['values']], [0, 1, 2])
        self.assertIn('canNode', old)
        self.assertNotIn('maxRampSpeed', resolve_schema(self.document, 2, '2.16')['fields'])
        self.assertIsNone(resolve_field({'minFirmwareVersion': '32.1'}, 3, '32.0'))
        self.assertIsNotNone(resolve_field({'minFirmwareVersion': '2.9'}, 3, '2.10'))

    def test_overlay_order_and_one_level_merge(self):
        f = {'raw': {'min': 0, 'max': 1}, 'ui': {'widget': 'slider'}, 'versions': {
            'firmware:32.0+': {'raw': {'max': 128}, 'values': [{'raw': 2, 'name': 'latest'}]},
            'eeprom:3+': {'raw': {'max': 100}},
            'firmware:2.18+': {'raw': {'max': 64}},
            'default': {'ui': {'disabledWhen': {'field': 'x', 'op': 'eq', 'raw': 0}}}}}
        r = resolve_field(f, 3, '32.0')
        self.assertEqual(r['raw'], {'min': 0, 'max': 128})
        self.assertEqual(r['ui']['widget'], 'slider')
        self.assertIn('disabledWhen', r['ui'])
        self.assertEqual(r['values'], [{'raw': 2, 'name': 'latest'}])

    def test_unknown_overlay_keys_do_not_change_field_identity(self):
        field = {'alias': ['ORIGINAL'], 'default': {'raw': 1}, 'versions': {
            'default': {'alias': ['CHANGED'], 'default': {'raw': 2}, 'unknown': True}}}
        resolved = resolve_field(field)
        self.assertEqual(resolved['alias'], ['ORIGINAL'])
        self.assertEqual(resolved['default'], {'raw': 1})
        self.assertNotIn('unknown', resolved)

    def test_disjoint_storage_reinterpretation(self):
        doc = copy.deepcopy(self.document)
        old = copy.deepcopy(doc['fields']['reserved0'])
        old['maxFirmwareVersion'] = '31.255'
        doc['fields']['legacyReserved'] = old
        doc['fields']['reserved0']['minFirmwareVersion'] = '32.0'
        validate_layout(doc)
        doc['fields']['legacyReserved']['maxFirmwareVersion'] = '32.0'
        with self.assertRaisesRegex(ValueError, 'overlaps'):
            validate_layout(doc)

    def test_disjoint_layout_reinterpretation(self):
        doc = copy.deepcopy(self.document)
        old = copy.deepcopy(doc['fields']['reserved0'])
        old['maxEepromVersion'] = 2
        doc['fields']['legacyReserved'] = old
        doc['fields']['reserved0']['minEepromVersion'] = 3
        validate_layout(doc)

    def test_gap_overlap_alias_and_group_rejected(self):
        cases = [('gap', lambda d: d['fields'].pop('reserved0')),
                 ('overlap', lambda d: d['fields']['motorPoles'].update(offset=26)),
                 ('alias', lambda d: d['fields']['motorPoles'].update(alias=['MOTOR_KV'])),
                 ('group', lambda d: d['groups']['can'].update(minEepromVersion=4)),
                 ('membership', lambda d: d['groups']['motor']['fields'].append('currentLimit')),
                 ('width', lambda d: d['fields']['motorPoles'].update(size=2))]
        for name, mutate in cases:
            with self.subTest(name=name):
                doc = copy.deepcopy(self.document)
                mutate(doc)
                with self.assertRaises(ValueError):
                    validate_layout(doc)

    def test_invalid_language_storage_overlay_and_predicate_rejected(self):
        for mutate in [lambda d: d.update(version='2.0.0'),
                       lambda d: d['fields']['motorPoles']['versions']['firmware:32.0+'].update(offset=22),
                       lambda d: d['fields']['motorPoles'].update(cName='motor_poles'),
                       lambda d: d['fields']['motorPoles'].update(ui={'visibleWhen': {'all': []}})]:
            doc = copy.deepcopy(self.document)
            mutate(doc)
            with self.assertRaises(ValueError):
                validate_json_schema(doc)

    def test_unknown_same_major_metadata_is_accepted(self):
        doc = copy.deepcopy(self.document)
        doc['fields']['motorKv']['futureConsumer'] = {'help': 'ignored'}
        validate_json_schema(doc)
        validate_layout(doc)

    def test_default_blob_and_erased_tail(self):
        golden = bytes.fromhex((REPO_ROOT / 'schema/eeprom-defaults.hex').read_text())
        self.assertEqual(default_bytes(self.document), golden)
        self.assertEqual(len(golden), 48)
        self.assertEqual(golden[3:5], bytes([1, 35]))
        self.assertEqual(golden[43:45], bytes([141, 102]))

    def test_exact_factory_conversion_rejects_quantization(self):
        self.assertEqual(encode_motor_kv(1020), 25)
        self.assertEqual(encode_advance_degrees(15), 26)
        with self.assertRaisesRegex(FactoryImageError, 'not representable exactly'):
            encode_motor_kv(1000)
        with self.assertRaisesRegex(FactoryImageError, 'not representable exactly'):
            encode_advance_degrees(16)
        field = resolve_schema(self.document)['fields']['motorKv']
        self.assertEqual(to_raw(field, 1000), 25)
        self.assertEqual(from_raw(field, 25), 1020)

    def test_factory_page_keeps_historical_bytes(self):
        product = json.loads((REPO_ROOT / 'factory/ARK_4IN1_F051_eeprom_defaults.json').read_text())
        page = build_eeprom_page(product, 32, 0, 3)
        import hashlib
        # Captured from the pre-schema production encoder and product JSON.
        self.assertEqual(hashlib.sha256(page).hexdigest(), '0ddd802a96d82220bef23ea7901b560caf2e393c947194befddf1ad17ffdb864')
        self.assertEqual(page[48:], b'\xff' * (1024 - 48))

    def test_generation_is_deterministic(self):
        with tempfile.TemporaryDirectory() as directory:
            cmd = [sys.executable, str(SCRIPTS / 'eeprom/eeprom_tool.py'), 'generate', '--out', directory]
            subprocess.run(cmd, check=True)
            before = {p.name: p.read_bytes() for p in Path(directory).iterdir()}
            subprocess.run(cmd, check=True)
            self.assertEqual(before, {p.name: p.read_bytes() for p in Path(directory).iterdir()})

    def test_parallel_generation(self):
        with tempfile.TemporaryDirectory() as directory:
            cmd = [sys.executable, str(SCRIPTS / 'eeprom/eeprom_tool.py'), 'generate', '--out', directory]
            processes = [subprocess.Popen(cmd) for _ in range(3)]
            self.assertEqual([p.wait() for p in processes], [0, 0, 0])
            self.assertEqual(len(list(Path(directory).iterdir())), 4)

    def test_frozen_gui_schema_assets(self):
        import shutil
        with tempfile.TemporaryDirectory() as directory:
            base = Path(directory) / 'eeprom-schema'
            for relative in ['schema/eeprom.json', 'schema/eeprom.schema.json', 'Inc/version.h']:
                destination = base / relative
                destination.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(REPO_ROOT / relative, destination)
            script = (f'import sys; sys.path.insert(0, {str(SCRIPTS / "eeprom")!r}); '
                      f'sys.frozen=True; sys._MEIPASS={directory!r}; '
                      'import schema; assert schema.load_schema()["bufferSize"] == 192')
            subprocess.run([sys.executable, '-c', script], check=True)


if __name__ == '__main__':
    unittest.main()
