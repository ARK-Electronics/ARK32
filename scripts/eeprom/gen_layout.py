"""Generate the flat packed C storage view."""
from schema import raw_range, snake_case


def generate(document):
    minimum, maximum = raw_range(document['fields']['motorPoles'])
    size = document['bufferSize']
    lines = ['/* Generated from schema/eeprom.json. Do not edit. */', '#pragma once', '#include <stdint.h>',
             f'#define EEPROM_SIZE {size}', f'#define MOTOR_POLES_MIN {minimum}', f'#define MOTOR_POLES_MAX {maximum}',
             '', 'typedef union EEprom_u {', '    struct __attribute__((packed)) {']
    cursor = 0
    for key, field in sorted(document['fields'].items(), key=lambda item: item[1]['offset']):
        offset, width = field['offset'], field['size']
        if offset > cursor:
            # Unnamed bitfields reserve actual storage without exporting names.
            lines += ['        uint8_t : 8;' for _ in range(offset - cursor)]
        kind = field['type']
        ctype = f'{kind}_t' if kind in ('uint8', 'int8', 'uint16', 'int16', 'uint32', 'int32') else 'uint8_t'
        array = f'[{width}]' if kind in ('reserved', 'bluejay') and width > 1 else ''
        lines.append(f'        {ctype} {snake_case(key)}{array}; /* {offset} */')
        cursor = offset + width
    lines += ['        uint8_t : 8;' for _ in range(size - cursor)]
    lines += ['    };', f'    uint8_t buffer[{size}];', '} EEprom_t;', f'_Static_assert(sizeof(EEprom_t) == {size}, "EEPROM size");', '']
    return '\n'.join(lines)
