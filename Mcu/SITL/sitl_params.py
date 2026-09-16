#!/usr/bin/env python3
'''
AM32 eeprom parameter table for the SITL GUI parameter editor

Loads the layout from schema/eeprom.json (offsets are byte positions in the
192 byte eeprom image). Reserved slots and the 128 byte startup tune
are not listed.

Each entry is (offset, name, minimum, maximum, default, description).
Values are single bytes as stored; where the firmware scales a stored
byte (MOTOR_KV) the conversion lives in MODEL_CHECKS so the editor can
show what the simulated motor needs.
'''

import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'scripts/eeprom'))
from schema import build_context, default_bytes, from_raw, load_schema, raw_range, resolve_schema, to_raw

_SCHEMA = load_schema()
_LAYOUT, _FIRMWARE = build_context()
_FIELDS = resolve_schema(_SCHEMA, _LAYOUT, _FIRMWARE)['fields']
_VER = {'EEPROM_VERSION': _LAYOUT, 'VERSION_MAJOR': _FIRMWARE[0], 'VERSION_MINOR': _FIRMWARE[1]}
_NAMED = {field.get('sitl', {}).get('name', field.get('alias', [key])[0]): field
          for key, field in _FIELDS.items()
          if field['size'] == 1 and not field.get('hidden') and field['type'] != 'reserved'}


def _default(name, field):
    return _VER.get(name, field.get('default', {}).get('raw', field.get('dronecan', {}).get('default', 255)))


# Public table shape is retained for the GUI and existing sitl.param files;
# offsets, ranges and defaults now come exclusively from the schema.
PARAMS = [(field['offset'], name, *raw_range(field), _default(name, field),
           field.get('description', field['name'])) for name, field in _NAMED.items()]
PARAMS_BY_NAME = {p[1]: p for p in PARAMS}


def calc_value(name, byte):
    field = _NAMED.get(name)
    if field is None:
        return ''
    byte = int(byte)
    minimum, maximum = raw_range(field)
    disabled = field.get('disabledValue')
    if disabled and (byte == disabled['raw'] or not minimum <= byte <= maximum):
        return str(disabled['display'])
    for value in field.get('values', []):
        if byte == value['raw']:
            return value['name']
    if field['type'] == 'bool' or field.get('ui', {}).get('widget') == 'checkbox':
        return 'on' if byte else 'off'
    return ('%g %s' % (from_raw(field, byte), field.get('unit', ''))).strip()


def kv_to_byte(kv):
    field = _FIELDS['motorKv']
    display = field['display']
    return to_raw(field, max(display['min'], min(display['max'], float(kv))))


def byte_to_kv(b):
    return from_raw(_FIELDS['motorKv'], int(b))


def model_checks(model):
    '''parameters whose value has to agree with the simulated motor,
    as {name: (required_byte, description)}. model is the sim's motor
    dict (kv, poles); the comparison is on the STORED byte so the
    coarse Kv quantisation does not read as a mismatch'''
    out = {}
    if not model:
        return out
    kv = model.get('kv')
    if kv:
        want = kv_to_byte(kv)
        out['MOTOR_KV'] = (
            want,
            'the simulated motor is %.0f Kv (stored as %d = %d Kv).\n'
            'AM32 scales low rpm power protection from MOTOR_KV, so a\n'
            'mismatch clamps duty and the motor sticks at low rpm.'
            % (kv, want, byte_to_kv(want)))
    poles = model.get('poles')
    if poles:
        out['MOTOR_POLES'] = (
            int(poles),
            'the simulated motor has %d poles; a mismatch scales every\n'
            'reported rpm and the commutation timing.' % int(poles))
    return out


def mismatches(image, model):
    '''names of model-critical parameters whose stored value disagrees'''
    bad = []
    for name, (want, _help) in model_checks(model).items():
        off = PARAMS_BY_NAME[name][0]
        if off < len(image) and image[off] != want:
            bad.append(name)
    return bad


# ---------------------------------------------------------------- images
#
# eeprom images are generated from text parameter files (sitl.param)
# rather than committed as binaries: a binary pins one snapshot of the
# layout, and a later change to eeprom.h or to the firmware defaults
# leaves it silently meaning something else.

EEPROM_SIZE = _SCHEMA['bufferSize']


def _firmware_defaults():
    return default_bytes(_SCHEMA, _LAYOUT, _FIRMWARE)


def base_image():
    '''an eeprom image with no per-target settings: firmware defaults,
    erased (0xff) past the end of them, and the version fields taken
    from Inc/version.h so they track this tree'''
    img = bytearray(b'\xff' * EEPROM_SIZE)
    d = _firmware_defaults()
    img[:len(d)] = d
    for name in ('EEPROM_VERSION', 'VERSION_MAJOR', 'VERSION_MINOR'):
        p = PARAMS_BY_NAME[name]
        img[p[0]] = p[4]
    return img


def build_image(overrides):
    '''eeprom image for {name: stored byte}, on top of base_image()'''
    img = base_image()
    for name, value in overrides.items():
        p = PARAMS_BY_NAME.get(name)
        if p is None:
            raise KeyError('unknown parameter %s' % name)
        value = int(value)
        if not 0 <= value <= 255:
            raise ValueError('%s=%d is not a byte' % (name, value))
        img[p[0]] = value
    return bytes(img)


def parse_param_file(path):
    '''read a sitl.param file: "NAME value" or "NAME, value" lines with
    # comments. Values are eeprom bytes as stored'''
    out = {}
    for lineno, line in enumerate(open(path), 1):
        line = line.split('#')[0].replace(',', ' ').strip()
        if not line:
            continue
        f = line.split()
        if len(f) != 2:
            raise ValueError('%s:%d: expected "NAME value"' % (path, lineno))
        name = f[0].upper()
        if name not in PARAMS_BY_NAME:
            raise KeyError('%s:%d: unknown parameter %s' % (path, lineno, name))
        out[name] = int(float(f[1]))
    return out


def image_from_param_file(path):
    return build_image(parse_param_file(path))


def write_eeprom(param_path, dest):
    '''generate the eeprom image for a parameter file, for a SITL run'''
    image = image_from_param_file(param_path)
    with open(dest, 'wb') as f:
        f.write(image)
    return dest


def image_overrides(image):
    '''(overrides, unmapped) for an eeprom image: the named parameters
    that differ from base_image(), and the offsets that differ but have
    no name in PARAMS (the startup tune, reserved bytes)'''
    base = base_image()
    named = dict((p[0], p[1]) for p in PARAMS)
    overrides, unmapped = {}, []
    for off in range(min(len(image), EEPROM_SIZE)):
        if image[off] == base[off]:
            continue
        if off in named:
            overrides[named[off]] = image[off]
        else:
            unmapped.append(off)
    return overrides, unmapped


def format_param_file(image, header=''):
    '''sitl.param text for an eeprom image, listing only what differs
    from the firmware defaults'''
    overrides, unmapped = image_overrides(image)
    lines = []
    for line in header.splitlines():
        lines.append(('# ' + line).rstrip())
    lines += [
        '#',
        '# eeprom parameters as stored (bytes). Anything not listed keeps',
        '# the firmware default from default_settings[] in',
        '# schema/eeprom.json; the image is generated by',
        '# Mcu/SITL/sitl_params.py, never committed as a binary.',
        '']
    if unmapped:
        lines += ['# NOTE: unnamed bytes also differ from the defaults at '
                  'offsets %s' % ', '.join(str(o) for o in unmapped), '']
    for off, name, _mn, _mx, _dflt, _help in PARAMS:
        if name not in overrides:
            continue
        calc = calc_value(name, overrides[name])
        lines.append('%-24s %3d%s' % (name, overrides[name],
                                      '   # %s' % calc if calc else ''))
    return '\n'.join(lines) + '\n'


def main():
    import argparse
    ap = argparse.ArgumentParser(description=__doc__)
    sub = ap.add_subparsers(dest='cmd', required=True)
    b = sub.add_parser('build', help='generate an eeprom image')
    b.add_argument('param')
    b.add_argument('out')
    d = sub.add_parser('dump', help='print an eeprom image as parameters')
    d.add_argument('eeprom')
    args = ap.parse_args()
    if args.cmd == 'build':
        write_eeprom(args.param, args.out)
        print('wrote %s from %s' % (args.out, args.param))
    else:
        print(format_param_file(open(args.eeprom, 'rb').read(),
                                'dumped from %s' % os.path.basename(args.eeprom)),
              end='')


if __name__ == '__main__':
    main()
