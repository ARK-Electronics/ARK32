"""Generate byte rewrite policies, preserving the existing reset semantics."""
from schema import raw_range, snake_case


def generate(document):
    lines = ['/* Generated from schema/eeprom.json. Do not edit. */', '#pragma once', '#include "eeprom.h"',
             'static inline void eeprom_apply_on_load(EEprom_t *settings)', '{']
    for key, field in document['fields'].items():
        if field.get('onLoad', 'keep') == 'keep':
            continue
        minimum, maximum = raw_range(field)
        value = field['disabledValue']['raw'] if field['onLoad'] == 'disable' else field['onLoadFallbackRaw']
        member = 'settings->' + snake_case(key)
        lines += [f'    if ({member} < {minimum} || {member} > {maximum}) {{', f'        {member} = {value};', '    }']
    return '\n'.join(lines + ['}', ''])
