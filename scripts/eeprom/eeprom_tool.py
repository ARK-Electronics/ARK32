#!/usr/bin/env python3
"""Generate build artifacts from the local EEPROM schema, or check its language."""
import argparse
from pathlib import Path
import sys
import tempfile

import gen_defaults
import gen_layout
import gen_onload
import gen_params
from schema import SCHEMA_PATH, build_context, load_schema, resolve_schema, validate_json_schema


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=['generate', 'check'])
    parser.add_argument('--schema', type=Path, default=SCHEMA_PATH)
    parser.add_argument('--out', type=Path, default=Path('obj/generated'))
    args = parser.parse_args()
    try:
        document = load_schema(args.schema)
        layout, firmware = build_context()
        if args.command == 'check':
            print(f'EEPROM schema {document["version"]}: valid (layout {layout}, firmware {firmware[0]}.{firmware[1]})')
            return 0
        resolved = resolve_schema(document, layout, firmware)
        outputs = {'eeprom_layout.h': gen_layout.generate(resolved),
                   'eeprom_defaults.h': gen_defaults.generate(document, layout, firmware),
                   'eeprom_params.c': gen_params.generate(resolved),
                   'eeprom_onload.h': gen_onload.generate(resolved)}
        args.out.mkdir(parents=True, exist_ok=True)
        for name, content in outputs.items():
            path = args.out / name
            # Atomic replace prevents consumers seeing a partial generated file.
            with tempfile.NamedTemporaryFile(mode='w', dir=args.out, prefix=name + '.', delete=False) as handle:
                handle.write(content)
                tmp = Path(handle.name)
            tmp.replace(path)
        return 0
    except (OSError, ValueError, KeyError, TypeError) as exc:
        print(f'EEPROM schema: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
