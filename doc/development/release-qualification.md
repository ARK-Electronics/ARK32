# Qualifying firmware integrations

Record this evidence against the final candidate SHA. A new candidate repeats whatever it invalidates.

## Configuration and version changes

For changes to EEPROM layout, conversions, defaults or firmware identity:

- Save the EEPROM bytes before migration. Cover factory defaults, representative field settings, erased or invalid fields, and boundary values.
- After first boot only the bytes the migration names change; reserved and tune bytes stay put. A second boot changes nothing.
- Round-trip settings through every transport, then save, restart and reread. Include invalid pole counts, enum limits and values outside a byte.
- The configurator picks the right target artifact and shows the version the EEPROM carries. Update from the previous release and record rollback.
- Compare the full factory EEPROM page with its fixture. Record intentional differences instead of replacing the fixture.

## Gate-driver and new-target changes

Hardware CI is dispatched manually. Attach the bench report and traces, linked to the exact firmware and bootloader SHAs.

| Scenario | Evidence |
|---|---|
| Factory flash and first boot | Bootloader/app/EEPROM regions; board identity; CAN configuration |
| Idle sleep and wake | ENABLE waveform, idle current, clean start and stop |
| Overtemperature warning | nFAULT, current and PWM show continued conduction; warning telemetry and derating |
| VDS retry and persistent fault | Pulse timing, retry count, bounded transition to the latched state |
| UVLO and thermal shutdown | PWM behavior and recovery once voltage or temperature returns |
| Gate-driver fault | Latch, and recovery on zero throttle or reset |
| F051 regression | DRV8328 fault still cuts drive and clears through its sleep/reset sequence |

For each fault, record how it was produced and observed, and the ambient temperature.

## Candidate record

Attach to the release PR or release:

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

Publish only once the candidate's required results exist. Verify the release manifest and checksums ([#112](https://github.com/ARK-Electronics/ARK32/pull/112)), and list known limitations and configuration changes in the release notes.
