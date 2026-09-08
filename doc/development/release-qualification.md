# Qualifying firmware integrations

Record the evidence below against the final candidate SHA. A new candidate repeats whatever it invalidates.

## Configuration and version changes

For changes to EEPROM layout, conversions, defaults or firmware identity:

- Record firmware SHA, configurator SHA/version, board revision, bootloader identity, and the original firmware version and EEPROM layout.
- Save the original EEPROM bytes before migration. Test factory defaults, representative field settings, erased/invalid fields, and boundary values.
- Verify the expected byte differences after first boot. Unrelated settings and reserved/tune bytes stay unchanged unless the migration says otherwise. A second boot must not repeat or drift the migration.
- Round-trip settings through every supported transport, then save, restart and reread. Include invalid pole counts, enum limits, and values outside a byte's range. Runtime division denominators must stay valid before settings are saved.
- Verify the configurator selects the correct target artifact and reports the version the EEPROM/protocol carries. Update from the previous supported release and record rollback behavior.
- Compare the full factory EEPROM page with its fixture. Record intentional differences instead of replacing the fixture.

Active work: [#107](https://github.com/ARK-Electronics/ARK32/pull/107) (schema) stacked on [#106](https://github.com/ARK-Electronics/ARK32/pull/106) (version change), paired with [configurator #16](https://github.com/ARK-Electronics/ark32-configurator/pull/16). Review the pair together: EEPROM version bytes 3-4, on-wire version display, artifact matching, and the motor-pole validation from [#110](https://github.com/ARK-Electronics/ARK32/pull/110).

## Gate-driver and new-target changes

Hardware CI stays manually dispatched once the rig is prepared. Attach the bench report and traces.

For the G431 CAN target [#79](https://github.com/ARK-Electronics/ARK32/pull/79) and DRV8350 fault handling [#104](https://github.com/ARK-Electronics/ARK32/pull/104):

| Scenario | Evidence to capture |
|---|---|
| Factory flash and first boot | Correct bootloader/app/EEPROM regions; board identity; CAN configuration |
| Idle sleep and wake | ENABLE waveform, idle current, clean startup and shutdown |
| Overtemperature warning | nFAULT, current and PWM show continued conduction; warning telemetry and thermal derating |
| VDS retry and persistent fault | Pulse timing, retry count, and bounded transition to the intended latched state |
| UVLO and thermal shutdown | PWM behavior and recovery after voltage/temperature returns to range |
| Gate-driver fault | Latch behavior and zero-throttle/reset recovery |
| F051 regression | DRV8328 fault still cuts drive and clears through its documented sleep/reset sequence |

State how each fault was produced and observed, not just the firmware log label. Record motor/prop, pack voltage, current limits, ambient temperature, board revision, and test limitations. Link traces to exact firmware and bootloader SHAs.

## Candidate record

Attach to the release PR or release artifact:

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

Publish versioned artifacts only after the candidate's required results exist. Verify the release manifest and checksums (added by [#112](https://github.com/ARK-Electronics/ARK32/pull/112)) and list known limitations and configuration changes in the release notes.
