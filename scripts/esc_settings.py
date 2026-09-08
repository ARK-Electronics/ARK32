"""Decode capture EEPROM images through the firmware's living schema.

Keep the raw bytes in records alongside the resolved display value. Field
names use the schema's stable identity translated to snake_case.
"""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent / 'eeprom'))
from schema import from_raw, load_schema, resolve_schema, snake_case

_SCHEMA = load_schema()
EEPROM_SIZE = _SCHEMA['bufferSize']


def decode(eeprom):
    """Raw bytes -> {name: {offset, raw, value, unit}} for present scalars."""
    if len(eeprom) < 5:
        return {}
    fields = resolve_schema(_SCHEMA, eeprom[1], (eeprom[3], eeprom[4]))['fields']
    out = {}
    for key, field in fields.items():
        off, size = field['offset'], field['size']
        if field.get('hidden') or field['type'] in ('reserved', 'bluejay') or off + size > len(eeprom):
            continue
        raw = int.from_bytes(eeprom[off:off + size], 'little', signed=field['type'].startswith('int'))
        out[snake_case(key)] = {'offset': off, 'raw': raw, 'value': from_raw(field, raw), 'unit': field.get('unit', '')}
    return out


def as_record(eeprom):
    vals = decode(eeprom)
    return {'eeprom_raw_hex': bytes(eeprom[:EEPROM_SIZE]).hex(),
            'settings': {k: v['value'] for k, v in vals.items()},
            'units': {k: v['unit'] for k, v in vals.items() if v['unit']},
            'raw_bytes': {k: v['raw'] for k, v in vals.items()}}


def format_text(eeprom):
    lines = []
    for name, value in decode(eeprom).items():
        shown = '%g' % value['value']
        suffix = (' %s' % value['unit']) if value['unit'] else ''
        raw_note = '' if value['raw'] == value['value'] else '   (raw %u)' % value['raw']
        lines.append('  %-26s %8s%-6s%s' % (name, shown, suffix, raw_note))
    return '\n'.join(lines)
