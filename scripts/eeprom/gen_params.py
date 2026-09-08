"""Generate DroneCAN enumeration; wire conversion code remains in DroneCAN.c."""
from schema import raw_range, snake_case


def generate(document):
    lines = ['/* Generated from schema/eeprom.json. Included by DroneCAN.c. */',
             'static const struct parameter parameters[] = {']
    fields = [(k, f) for k, f in document['fields'].items() if 'dronecan' in f]
    for key, field in sorted(fields, key=lambda item: item[1]['dronecan']['order']):
        dc = field['dronecan']
        minimum, maximum = raw_range(field)
        minimum, maximum = dc.get('min', minimum), dc.get('max', maximum)
        ptr = dc.get('ptr', 'eeprom')
        ptr = 'eepromBuffer.' + snake_case(key) if ptr == 'eeprom' else ptr.split(':', 1)[1]
        if 'ifdef' in dc:
            lines.append('#ifdef ' + dc['ifdef'])
        lines.append(f'    {{"{dc["name"]}", T_{dc["vtype"].upper()}, {minimum}, {maximum}, {dc["default"]}, &{ptr}}},')
        if 'ifdef' in dc:
            lines.append('#endif')
    return '\n'.join(lines + ['};', ''])
