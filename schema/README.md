# EEPROM schema

`eeprom.json` is the living layout and metadata source for ARK32. Releases
attach this exact document as `eeprom.json`. The schema language is 1.1.0;
EEPROM layout 3 and firmware 32.0 are independent version numbers.

Install the build dependency in your Python environment:

```sh
python3 -m pip install -r scripts/eeprom/requirements.txt
make eeprom-check
make AM32_SITL_CAN
```

Every firmware/SITL build validates the document with `eeprom.schema.json`,
resolves this tree's `Inc/version.h`, and generates the flat packed union,
48-byte defaults, DroneCAN table and load policies under `obj/generated/`.
Python 3.9+ and GNU make 4.3+ are required. Builds never fetch a schema.
CubeIDE builds need an explicit pre-build generation step and generated
include directory; they are currently unsupported.

Field identity is the JSON key. C members are derived by inserting an
underscore before each capital and lowercasing. Aliases name configurator
settings. `sitl.name` preserves existing parameter-file identifiers;
`factory.key` maps the product's display-unit settings to their fields.
Neither consumer maintains another offset/conversion table.

`versions` applies `default`, ascending `eeprom:N+`, then ascending
`firmware:X.Y+` overlays. Firmware comparison uses integer major/minor
pairs. Objects merge one level and arrays replace. Storage changes require
a new field key with disjoint firmware/layout version bounds. Unknown
same-major metadata remains compatible; another language major is rejected.

`onLoad: clamp` uses `onLoadFallbackRaw`, the existing firmware reset value
when a byte is outside its valid interval. This preserves the reset-to-5 or
reset-to-10 behavior; it does not choose the nearest endpoint. `disable`
uses `disabledValue.raw`. Old-format timing translation remains in C.

`dronecan.order` preserves parameter enumeration indices. Its `default`
keeps the runtime/CAN fallback (distinct from erase defaults); optional
`min`/`max` describe wire ranges differing from storage. Runtime pointers,
wire units and truncate-on-store conversions retain their existing behavior.

`default.raw` transcribes the historical erase prefix, independently of UI
ranges: firmware 1.35, temperature 141 and current 102 are intentional.
`eeprom-defaults.hex` is the independent committed compatibility fixture.
The remainder of an erase image is 0xFF. Configurator resets preserve the
identity/CAN fields declared by `preserveOnDefaults`.

The product factory JSON uses display units, with two explicit legacy
exceptions: temperature 141 and current 102 represent the schema's raw
disabled sentinels. Its encoder accepts those exact sentinels and otherwise
requires an exact display-to-raw round trip. The resulting 1 KiB production
page is unchanged. Factory test SHA256 is pinned to the pre-schema encoder.

Run language/history/factory checks with:

```sh
python3 -m unittest discover -s scripts/eeprom/tests -v
make factory-image-check
```

Firmware UDP and DroneCAN save/restart coverage lives in
`Mcu/SITL/tests/test_eeprom_schema.py` and runs with the regular SITL suite.

The companion configurator checkout also provides a hardware-free cross-repo
test. After building this firmware, run from the configurator repository:

```sh
python3 scripts/sitl/run_integration.py --firmware ../AM32
```

It uses the actual TypeScript codec against firmware UDP readback and verifies
DroneCAN SAVE across a process restart. It requires Node 22, installed
configurator dependencies, and the Python SITL CI dependencies. Run it separately
from other CAN tests because the harness shares a finite set of multicast buses.
