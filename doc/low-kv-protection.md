# Low-kV RPM-dependent duty limit

The stored 20 kV setting (EEPROM byte 26 equal to zero) disables the low-RPM
duty ceiling, as in upstream AM32. Other stored values enable it, including
60, 100, 140, 180, 220 and 260 kV, which previously bypassed it. The opt-out
uses the stored byte so target-specific kV scaling cannot turn another
setting into an opt-out. RC-car mode forces the limiter on with a 1000/2000
low-speed ceiling.

Only an explicit 20 kV selects the opt-out over DroneCAN: `MOTOR_KV` writes
of 21–59 kV are stored as 60 kV, and writes outside 20–10220 kV are ignored
rather than wrapped into the byte (10260 kV used to store zero). The
configurator rounds to the nearest 40 kV step, so enter 60 for a motor
under 60 kV to keep the limiter.

ARK32 keeps its existing envelope calculation, including support for pole
counts above 32. At low speed the nominal ceiling is 400/2000 duty (20%);
it rises to 2000/2000 (100%) through the configured electrical-RPM range.
Running-brake dead-time compensation raises the low-speed ceiling by the
added dead time, and stall protection can add duty after the clamp. This is
separate from the first-commutations startup cap and the current limit.

The low-speed ceiling is never below the startup floor, `min_startup_duty`
(Minimum duty cycle × 10 + Startup power, plus any dead-time compensation).
The startup code clamps its floor to the ceiling, so a lower ceiling would
override the configured minimum duty and startup power. For example,
Minimum duty cycle 30 and Startup power 150 give a 450/2000 ceiling.

Every settings load starts from the power-on values (limiter on, 400/2000
ceiling, no current limit), so a reload through the SITL EEPROM editor or a
DroneCAN reset to defaults matches a boot with the same EEPROM. The PWM
dead-time register is only ever ORed, so a Running brake level change
still needs a restart to reach the timer.

For a 160 kV, 42-pole motor configured with byte 26 equal to 4, the stored
value is **180 kV**. On a target without kV scaling, its envelope is
**2,000–13,000 electrical RPM**, or approximately **95–619 shaft RPM**.
Use the encoded value when comparing
tunes: a DroneCAN write of 160 kV truncates to byte 3 and reloads as 140 kV,
giving a different envelope. Save and restart after changing motor kV or
pole count through DroneCAN so the derived limits are recomputed.

The limit applies during six-step operation, including after sine startup
hands off. Sine stepping itself is unchanged. Automatic timing retains its
separate 300 effective-kV threshold (after target scaling) and uses the
duty-based schedule below it.

Existing integer resolution still matters: very small kV/pole products
(for example 60 kV with two poles) produce a zero upper RPM threshold,
which leaves the ceiling unrestricted. Invalid pole counts retain the
existing zero-envelope fallback.

## Hardware validation before release

SITL regression tests check the loaded policy, electrical-RPM thresholds,
stopped duty ceiling and settings reloads. They do not establish loaded
starting torque or sine-handoff performance on a physical low-kV motor.

Compare the base and changed firmware with the same saved settings and
bench setup:

1. On the 160 kV/42-pole motor, verify the saved 180 kV encoding and repeat
   cold starts and low-speed throttle steps at the intended load and supply.
   Compare starting reliability, spool time, current and desync/stuck events.
2. Repeat with sine startup enabled, observing the transition into six-step
   operation for hesitation, stalls or repeated reacquisition.
3. Run the existing normal-kV baseline with its usual settings to check that
   its startup and throttle response remain consistent.

Record the firmware commits, stored kV/poles, supply, load and traces with
the results. The 20 kV bypass is a policy regression case, not a substitute
for testing the motor's configured envelope.
