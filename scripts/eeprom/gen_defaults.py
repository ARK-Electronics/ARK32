"""Generate the unchanged erase-image prefix; the remainder stays erased."""
from schema import default_bytes


def generate(document, layout, firmware):
    data = default_bytes(document, layout, firmware)
    lines = ['/* Generated from schema/eeprom.json. Do not edit. */', '#pragma once', '#include <stdint.h>',
             f'enum {{ EEPROM_DEFAULT_SETTINGS_LEN = {len(data)} }};',
             'static const uint8_t default_settings[EEPROM_DEFAULT_SETTINGS_LEN] = {']
    for start in range(0, len(data), 12):
        lines.append('    ' + ', '.join(f'0x{x:02x}' for x in data[start:start + 12]) + ',')
    return '\n'.join(lines + ['};', ''])
