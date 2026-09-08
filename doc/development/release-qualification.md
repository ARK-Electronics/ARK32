# Qualifying firmware integrations

Build success establishes that the firmware links; a product release also
needs evidence that configuration migration and hardware behavior match the
intended change. Record the evidence below against the final candidate SHA.
If the candidate changes, identify which results need to be repeated.

## Configuration and version changes

For changes to EEPROM layout, conversions, defaults or firmware identity:

- Record firmware SHA, configurator SHA/version, board revision, bootloader
  identity, and the original firmware version and EEPROM layout.
- Save the original EEPROM bytes before migration. Test factory defaults,
  representative field settings, erased/invalid fields, and boundary values.
- Verify the expected byte differences after first boot. Unrelated settings
  and reserved/tune bytes must remain unchanged unless migration specifies
  otherwise. Verify a second boot does not repeat or drift the migration.
- Round-trip settings through every supported configuration transport, then
  save, restart and reread. Include invalid pole counts, enum limits, and
  numeric values outside a byte's range. Runtime division denominators must
  remain valid even before settings are saved.
- Verify the configurator selects the correct target artifact and reports
  the version actually carried by the EEPROM/protocol. Exercise update from
  the previous supported release and document rollback behavior explicitly.
- Compare the full factory EEPROM page with its expected fixture. Record
  intentional differences instead of replacing a golden fixture blindly.

The active schema work is [#107](https://github.com/ARK-Electronics/ARK32/pull/107),
stacked on the version change [#106](https://github.com/ARK-Electronics/ARK32/pull/106),
with [configurator #16](https://github.com/ARK-Electronics/ark32-configurator/pull/16).
Review the firmware/configurator pair together. For the 3.0.3 to 32.0 transition,
verify EEPROM version bytes 3-4, on-wire version display, and artifact matching.
The schema PR must retain motor-pole validation from
[#110](https://github.com/ARK-Electronics/ARK32/pull/110).
These links identify pending work; this document does not certify it as shipped.

## Gate-driver and new-target changes

Keep hardware CI manually dispatched after the rig is prepared. Attach the
actual bench report and traces; unchecked test-plan boxes are not evidence.

For G431 CAN target [#79](https://github.com/ARK-Electronics/ARK32/pull/79) and
DRV8350 fault handling [#104](https://github.com/ARK-Electronics/ARK32/pull/104):

| Scenario | Evidence to capture |
|---|---|
| Factory flash and first boot | Correct bootloader/app/EEPROM regions; board identity; CAN configuration |
| Idle sleep and wake | ENABLE waveform, idle current, clean startup and shutdown |
| Overtemperature warning | nFAULT, current and PWM show continued conduction; warning telemetry and thermal derating |
| VDS retry and persistent fault | Pulse timing, retry count, and bounded transition to the intended latched state |
| UVLO and thermal shutdown | PWM behavior and recovery after voltage/temperature returns to the specified range |
| Gate-driver fault | Latch behavior and zero-throttle/reset recovery |
| F051 regression | DRV8328 fault still cuts drive and clears through its documented sleep/reset sequence |

State how each fault condition was produced and observed; do not infer the
physical fault class from the firmware log label alone. Record motor/prop,
pack voltage, current limits, ambient temperature, board revision, and any
test limitations. Link traces to exact firmware and bootloader SHAs.

## Candidate record

Attach a short Markdown record to the release PR or release artifact:

```text
Firmware SHA:
Bootloader SHA / image hash:
Configurator SHA / version:
Board / revision:
Motor / prop / supply / current limits:
Source release and EEPROM layout:
Migration fixture and expected byte differences:
CI build / SITL / calibration / size / codegen / factory result URLs:
Bench report and trace URLs:
Cases executed, outcomes and omissions:
Rollback behavior:
Reviewer and date:
```

Publish immutable versioned artifacts after the candidate's required results
are available. Verify its release manifest and checksums, and include known
limitations and configuration changes in the release notes. A nightly build
alone does not establish hardware qualification.
