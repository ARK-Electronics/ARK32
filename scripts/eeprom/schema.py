"""Local EEPROM schema loading, overlay resolution, validation and conversions.

Layout width/coverage rules originate in AM32 PR 295's generator. Unlike that
checker the JSON is the source, and no build operation downloads schema data.
"""
from __future__ import annotations

import copy
import json
import math
import re
import sys
from decimal import Decimal
from pathlib import Path

REPO_ROOT = (Path(sys._MEIPASS) / 'eeprom-schema' if getattr(sys, 'frozen', False)
             else Path(__file__).resolve().parents[2])
SCHEMA_PATH = REPO_ROOT / 'schema/eeprom.json'
TYPE_WIDTH = {'uint8': 1, 'int8': 1, 'bool': 1, 'enum': 1, 'number': 1,
              'uint16': 2, 'int16': 2, 'uint32': 4, 'int32': 4}


def firmware_tuple(value):
    if isinstance(value, (tuple, list)) and len(value) == 2:
        return tuple(int(v) for v in value)
    if not re.fullmatch(r'\d+\.\d+', str(value)):
        raise ValueError(f'invalid firmware version: {value!r}')
    return tuple(int(v) for v in str(value).split('.'))


def build_context(path=REPO_ROOT / 'Inc/version.h'):
    text = Path(path).read_text()
    values = {k: int(v) for k, v in re.findall(
        r'^\s*#\s*define\s+(VERSION_MAJOR|VERSION_MINOR|EEPROM_VERSION)\s+(\d+)', text, re.M)}
    return values['EEPROM_VERSION'], (values['VERSION_MAJOR'], values['VERSION_MINOR'])


def snake_case(key):
    return re.sub(r'[A-Z]', lambda m: '_' + m[0].lower(), key)


def _merge(base, overlay):
    """One level object merge; arrays replace, including enum values."""
    for key, value in overlay.items():
        if key not in ('raw', 'display', 'values', 'disabledValue', 'ui'):
            continue
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            base[key] = {**base[key], **copy.deepcopy(value)}
        else:
            base[key] = copy.deepcopy(value)
    return base


def field_present(field, layout, firmware):
    fw = firmware_tuple(firmware)
    return (field.get('minEepromVersion', 0) <= layout <= field.get('maxEepromVersion', math.inf)
            and firmware_tuple(field.get('minFirmwareVersion', '0.0')) <= fw
            and ('maxFirmwareVersion' not in field or fw <= firmware_tuple(field['maxFirmwareVersion'])))


def resolve_field(field, layout=3, firmware='32.0'):
    if not field_present(field, layout, firmware):
        return None
    result = copy.deepcopy(field)
    versions = result.pop('versions', {})
    _merge(result, versions.get('default', {}))
    eeprom, firmwares = [], []
    for key, overlay in versions.items():
        if key == 'default':
            continue
        match = re.fullmatch(r'eeprom:(\d+)\+', key)
        if match:
            if int(match[1]) <= layout:
                eeprom.append((int(match[1]), overlay))
            continue
        match = re.fullmatch(r'firmware:(\d+\.\d+)\+', key)
        if not match:
            raise ValueError(f'invalid overlay selector: {key}')
        if firmware_tuple(match[1]) <= firmware_tuple(firmware):
            firmwares.append((firmware_tuple(match[1]), overlay))
    for _, overlay in sorted(eeprom) + sorted(firmwares):
        _merge(result, overlay)
    return result


def resolve_schema(document, layout=3, firmware='32.0'):
    result = copy.deepcopy(document)
    result['fields'] = {key: resolved for key, field in document['fields'].items()
                        if (resolved := resolve_field(field, layout, firmware)) is not None}
    result['groups'] = {key: {**group, 'fields': [f for f in group['fields'] if f in result['fields']]}
                        for key, group in result['groups'].items()
                        if field_present(group, layout, firmware)}
    return result


def raw_range(field):
    values = [v['raw'] for v in field.get('values', [])]
    bits = field['size'] * 8
    signed = field['type'].startswith('int')
    minimum = -(1 << (bits - 1)) if signed else 0
    maximum = (1 << (bits - int(signed))) - 1
    if field['type'] == 'bool':
        maximum = 1
    return (field.get('raw', {}).get('min', min(values) if values else minimum),
            field.get('raw', {}).get('max', max(values) if values else maximum))


def from_raw(field, raw):
    display = field.get('display', {})
    value = Decimal(str(raw)) * Decimal(str(display.get('factor', 1))) + Decimal(str(display.get('offset', 0)))
    return int(value) if value == value.to_integral_value() else float(value)


def to_raw(field, value, exact=False):
    display = field.get('display', {})
    original = Decimal(str(value))
    converted = (original - Decimal(str(display.get('offset', 0)))) / Decimal(str(display.get('factor', 1)))
    # Match Math.round for the UI, with deterministic decimal affine arithmetic.
    raw = math.floor(converted + Decimal('0.5'))
    minimum, maximum = raw_range(field)
    if not minimum <= raw <= maximum:
        raise ValueError(f'{field.get("name", "field")}: {value} out of range')
    if exact and Decimal(str(from_raw(field, raw))) != original:
        raise ValueError(f'{field.get("name", "field")}: {value} is not representable exactly')
    return raw


def default_bytes(document, layout=3, firmware='32.0'):
    fields = resolve_schema(document, layout, firmware)['fields']
    end = max((f['offset'] + f['size'] for f in fields.values() if 'default' in f), default=0)
    image = bytearray(b'\xff' * end)
    for field in fields.values():
        if 'default' not in field:
            continue
        raw = field['default']['raw']
        data = bytes(raw) if isinstance(raw, list) else int(raw).to_bytes(field['size'], 'little', signed=field['type'].startswith('int'))
        image[field['offset']:field['offset'] + field['size']] = data
    return bytes(image)


def _rectangles_overlap(a, b):
    if max(a.get('minEepromVersion', 0), b.get('minEepromVersion', 0)) > min(a.get('maxEepromVersion', math.inf), b.get('maxEepromVersion', math.inf)):
        return False
    lo = max(firmware_tuple(a.get('minFirmwareVersion', '0.0')), firmware_tuple(b.get('minFirmwareVersion', '0.0')))
    hi = min(firmware_tuple(a.get('maxFirmwareVersion', '2147483647.2147483647')), firmware_tuple(b.get('maxFirmwareVersion', '2147483647.2147483647')))
    return lo <= hi


def _validate_predicate(predicate, fields):
    if 'all' in predicate or 'any' in predicate:
        key = 'all' if 'all' in predicate else 'any'
        if set(predicate) != {key} or not predicate[key]:
            raise ValueError('predicate must be a nonempty all/any or a field comparison')
        for child in predicate[key]:
            _validate_predicate(child, fields)
    elif (set(predicate) != {'field', 'op', 'raw'} or predicate['field'] not in fields
          or predicate['op'] not in ('eq', 'ne', 'gt', 'lt') or not isinstance(predicate['raw'], int)):
        raise ValueError(f'invalid UI predicate: {predicate}')


def validate_layout(document, layout=3, firmware='32.0'):
    if str(document.get('version', '')).split('.')[0] != '1':
        raise ValueError('unsupported EEPROM schema language major')
    fields = document['fields']
    buffer_size = document['bufferSize']
    aliases, members, dronecan_names, dronecan_orders = set(), set(), set(), set()
    for key, field in fields.items():
        if not re.fullmatch(r'[a-z][A-Za-z0-9]*', key):
            raise ValueError(f'invalid field key: {key}')
        member = snake_case(key)
        if member in members:
            raise ValueError(f'duplicate C member: {member}')
        members.add(member)
        offset, size, kind = field['offset'], field['size'], field['type']
        if not isinstance(offset, int) or not isinstance(size, int) or offset < 0 or size < 1 or offset + size > buffer_size:
            raise ValueError(f'{key}: invalid byte extent')
        if kind not in ('reserved', 'bluejay') and (kind not in TYPE_WIDTH or size != TYPE_WIDTH[kind]):
            raise ValueError(f'{key}: type width does not match size')
        if field.get('minEepromVersion', 0) > field.get('maxEepromVersion', math.inf):
            raise ValueError(f'{key}: inverted EEPROM interval')
        if firmware_tuple(field.get('minFirmwareVersion', '0.0')) > firmware_tuple(field.get('maxFirmwareVersion', '2147483647.2147483647')):
            raise ValueError(f'{key}: inverted firmware interval')
        if not field.get('hidden') and kind != 'reserved' and not key.startswith('can') and not field.get('alias'):
            raise ValueError(f'{key}: missing alias')
        for alias in field.get('alias', []):
            if alias in aliases:
                raise ValueError(f'duplicate alias: {alias}')
            aliases.add(alias)
        for selector, overlay in field.get('versions', {}).items():
            if not re.fullmatch(r'default|eeprom:\d+\+|firmware:\d+\.\d+\+', selector):
                raise ValueError(f'{key}: invalid overlay selector {selector}')
            if any(k in overlay for k in ('offset', 'size', 'type')):
                raise ValueError(f'{key}: storage may not be overlaid')
        if 'default' in field:
            raw = field['default']['raw']
            if isinstance(raw, list):
                if len(raw) != size or any(not isinstance(x, int) or not 0 <= x <= 255 for x in raw):
                    raise ValueError(f'{key}: invalid default byte array')
            else:
                try:
                    int(raw).to_bytes(size, 'little', signed=kind.startswith('int'))
                except (OverflowError, TypeError, ValueError) as exc:
                    raise ValueError(f'{key}: default does not fit storage') from exc
        dc = field.get('dronecan')
        if dc:
            if dc['name'] in dronecan_names or dc['order'] in dronecan_orders:
                raise ValueError(f'{key}: duplicate DroneCAN name or order')
            dronecan_names.add(dc['name']); dronecan_orders.add(dc['order'])
            if dc.get('ptr', 'eeprom') not in ('eeprom', 'runtime:motor_kv', 'runtime:low_cell_volt_cutoff'):
                raise ValueError(f'{key}: unsupported DroneCAN runtime pointer')
        for candidate in [field, *field.get('versions', {}).values()]:
            for predicate in ('visibleWhen', 'disabledWhen'):
                if predicate in candidate.get('ui', {}):
                    _validate_predicate(candidate['ui'][predicate], fields)
    ordered = sorted(fields.items(), key=lambda item: item[1]['offset'])
    for i, (a_key, a) in enumerate(ordered):
        for b_key, b in ordered[i + 1:]:
            if b['offset'] >= a['offset'] + a['size']:
                break
            if _rectangles_overlap(a, b):
                raise ValueError(f'{a_key} overlaps {b_key} in a firmware/layout rectangle')
    memberships = {key: 0 for key in fields}
    for group_key, group in document['groups'].items():
        for key in group['fields']:
            if key not in fields:
                raise ValueError(f'{group_key}: unknown field {key}')
            memberships[key] += 1
        if group['fields'] and group.get('minEepromVersion', 0) > min(fields[k].get('minEepromVersion', 0) for k in group['fields']):
            raise ValueError(f'{group_key}: group layout gate hides a member')
        for predicate in ('visibleWhen', 'disabledWhen'):
            if predicate in group.get('ui', {}):
                _validate_predicate(group['ui'][predicate], fields)
    for key, field in fields.items():
        if not field.get('hidden') and field['type'] != 'reserved' and memberships[key] != 1:
            raise ValueError(f'{key}: visible field must occur in exactly one group')
    coverage = set()
    for key, field in resolve_schema(document, layout, firmware)['fields'].items():
        coverage.update(range(field['offset'], field['offset'] + field['size']))
        minimum, maximum = raw_range(field)
        if minimum > maximum:
            raise ValueError(f'{key}: inverted raw range')
        if field.get('display', {}).get('factor', 1) == 0:
            raise ValueError(f'{key}: display factor must be nonzero')
        if field.get('onLoad') == 'clamp' and 'onLoadFallbackRaw' not in field:
            raise ValueError(f'{key}: clamp needs onLoadFallbackRaw')
        if field.get('onLoad') == 'disable' and 'disabledValue' not in field:
            raise ValueError(f'{key}: disable needs disabledValue')
    if coverage != set(range(buffer_size)):
        raise ValueError(f'current layout has unmapped bytes: {sorted(set(range(buffer_size)) - coverage)}')


def validate_json_schema(document):
    try:
        import jsonschema
    except ImportError as exc:
        raise ValueError('EEPROM tools require jsonschema: pip install -r scripts/eeprom/requirements.txt') from exc
    definition = json.loads((REPO_ROOT / 'schema/eeprom.schema.json').read_text())
    jsonschema.Draft7Validator.check_schema(definition)
    try:
        jsonschema.Draft7Validator(definition).validate(document)
    except jsonschema.ValidationError as exc:
        raise ValueError(f'JSON Schema: {exc.message} at {list(exc.path)}') from exc


def load_schema(path=SCHEMA_PATH):
    document = json.loads(Path(path).read_text())
    validate_json_schema(document)
    layout, firmware = build_context()
    validate_layout(document, layout, firmware)
    return document
