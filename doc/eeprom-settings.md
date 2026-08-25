# EEPROM settings cheat sheet

User-facing reference for every field in the 192-byte `EEprom_t` (`Inc/eeprom.h`), in layout order. Units are what [ARK32 Configurator](https://github.com/ARK-Electronics/ark32-configurator) shows, not the raw byte. **ARK default** is the ARK 4IN1 factory image (`factory/ARK_4IN1_F051_eeprom_defaults.json`).

The configurator serves the same text as **Settings guide** (always available) and as a slide-over from the settings tab. Keep this file in lockstep with `packages/am32-core/src/eeprom/guide.ts` in that repo.

| Setting | EEPROM name | Range | ARK default | What it does |
|---------|-------------|-------|-------------|--------------|
| Boot byte | `BOOT_BYTE` | 0x00 / 0x01 / 0xFF | 0x01 | Bootloader jump flag. 0x01 or 0xFF runs the application. Owned by the flash/boot path, not a user setting. |
| EEPROM layout | `LAYOUT_REVISION` | integer (current is 3) | 3 | Which EEPROM layout the firmware expects. The firmware migrates older layouts on boot. Do not write this by hand. |
| Bootloader revision | `BOOT_LOADER_REVISION` | integer | stamped by the bootloader | Bootloader version. The bootloader overwrites this on every settings-page write, so changing it does nothing. |
| Firmware major | `MAIN_REVISION` | integer | from the running firmware | Firmware major version, rewritten on boot. Not a setting. |
| Firmware minor | `SUB_REVISION` | integer | from the running firmware | Firmware minor version, rewritten on boot. Not a setting. |
| Ramp rate | `MAX_RAMP` | 0.1–20 % duty / ms | 2.0 %/ms | How fast duty may change, as a percent of full throttle per millisecond. 2 %/ms means 0→100% takes 50 ms; lower is safer on large props, and values under 1.0 use a finer 0.1 %/ms step. Firmware caps each rpm band (startup 2, low-rpm 6, high-rpm 16 %/ms) and this setting only lowers those caps, so the factory 2.0 %/ms flattens all three — raise toward 16 %/ms for high-kV 5–7 inch punch. |
| Minimum duty cycle | `MINIMUM_DUTY_CYCLE` | 0–25% | 2% | Lowest motor duty the ESC will apply once spinning, on DShot and servo alike. Raise if the motor will not start or growls at idle. Too high wastes hover power. |
| Disable stick calibration | `DISABLE_STICK_CALIBRATION` | Off / On | Off | Skips the PWM high/low stick calibration on boot. Irrelevant for DShot. |
| Absolute voltage cutoff | `ABSOLUTE_VOLTAGE_CUTOFF` | 0.5–50 V (0.5 V steps) | 5.0 V | Pack voltage that Absolute LVC treats as empty. Unused unless Low voltage cut off is Absolute. |
| Current P | `CURRENT_P` | 0–255 | 100 | Proportional gain of the current limiter. Only runs when a current limit is set. Leave the factory value unless you are tuning the limiter. |
| Current I | `CURRENT_I` | 0–255 | 0 | Integral gain of the current limiter. Leave at 0 unless the limit is oscillating or sagging. |
| Current D | `CURRENT_D` | 0–255 | 50 | Derivative gain of the current limiter. Damps overshoot when current hits the cap. |
| Active brake power | `ACTIVE_BRAKE_POWER` | 0–5% duty (0 is off) | 2% | Duty applied in Active brake mode. Unused unless Brake on stop is Active brake. |
| Reversed | `MOTOR_DIRECTION` | Off / On | Off | Swaps rotation direction. Same as the Reversed checkbox on each ESC card. |
| 3D mode | `BIDIRECTIONAL_MODE` | Off / On | Off | Center throttle is stop; above is forward, below is reverse. For 3D planes, not multirotors. |
| Sinusoidal startup | `SINUSOIDAL_STARTUP` | Off / On | Off | Open-loop sine drive at the start of spool-up, smoother on large, high-inertia motors. Requires Complementary PWM. ARK 4IN1 ships this off; the feature is still available. |
| Complementary PWM | `COMPLEMENTARY_PWM` | Off / On | On | Drives the low-side FETs during the off time (active freewheeling). More efficient, and required for sine start and braking. Leave on for multirotors. |
| PWM type | `VARIABLE_PWM_FREQUENCY` | Fixed / Variable / By RPM | Variable | Fixed uses the kHz setting. Variable sweeps that setting up to 2× as rpm rises. By RPM picks frequency from commutation speed and ignores the kHz slider. |
| Stuck rotor protection | `STUCK_ROTOR_PROTECTION` | Off / On | On | Cuts drive if the motor never produces BEMF (jammed or missing prop). Leave on. |
| Timing advance | `TIMING_ADVANCE` | 0–30° (0.9375° steps) | 15° | How early the ESC commutates relative to the BEMF zero-cross. More advance can make more power and heat; too much desyncs. Ignored while Auto timing advance is on. |
| PWM frequency | `PWM_FREQUENCY` | 8–144 kHz | 24 kHz | FET switching rate when PWM Type is Fixed or Variable. Higher is quieter and costs more heat. Ignored when PWM Type is By RPM. |
| Startup power | `STARTUP_POWER` | 50–150% | 100% | Extra duty added on top of Minimum duty cycle during startup. Raise if the motor will not break out; lower if it lurches or overshoots. |
| Motor KV | `MOTOR_KV` | 20–10220 (40 kV steps) | 1020 | Nameplate kV. Used for the low-rpm throttle envelope and for auto timing. Wrong kV can cap top end or make timing too aggressive. |
| Motor poles | `MOTOR_POLES` | 2–128 | 14 | Magnet count, usually 14. Scales rpm telemetry and the same envelope/timing math as Motor KV. The ceiling is 128 for high-pole outrunners and hub motors. |
| Brake on stop | `BRAKE_ON_STOP` | Off / Brake on stop / Active brake | Off | Off coasts at zero throttle; Brake on stop shorts the windings (drag brake). Active brake applies a small reverse duty and only runs while armed. Leave off for multirotors. |
| Stall protection | `STALL_PROTECTION` | Off / On | Off | Boosts throttle as rpm falls, for crawlers and RC cars. Do not use on multirotors. |
| Beeper volume | `BEEP_VOLUME` | 0–11 | 5 | Loudness of motor-as-speaker beeps. 0 is effectively silent. Beeps only play when the motor is not spinning. |
| 30 ms interval telemetry | `INTERVAL_TELEMETRY` | Off / On (values >1 stagger the interval) | Off | Sends KISS telemetry on a ~30 ms cadence over the signal wire. Values above 1 offset the interval so several ESCs can share one wire. |
| Low threshold | `SERVO_LOW_THRESHOLD` | 750–1250 µs | 1020 µs | PWM pulse treated as zero throttle. Ignored under DShot. |
| High threshold | `SERVO_HIGH_THRESHOLD` | 1750–2250 µs | 1980 µs | PWM pulse treated as full throttle. Ignored under DShot. |
| Neutral | `SERVO_NEUTRAL` | 1374–1630 µs | 1500 µs | Center pulse for bidirectional / 3D PWM. Ignored under DShot and in unidirectional mode. |
| Dead band | `SERVO_DEAD_BAND` | 0–100 | 50 | PWM counts around Neutral treated as stop, so stick jitter does not spin the motor. |
| Low voltage cut off | `LOW_VOLTAGE_CUTOFF` | Off / Cell based / Absolute | Off | Off does nothing. Cell based estimates cell count and cuts at the per-cell threshold; Absolute cuts at the voltage set below. Prefer the flight controller for LVC on a multirotor. |
| Low voltage cut off threshold | `LOW_VOLTAGE_THRESHOLD` | 2.50–3.50 V/cell | 3.00 V/cell | Per-cell voltage that Cell based LVC treats as empty. Unused when LVC is Off or Absolute. |
| Car type reverse braking | `RC_CAR_REVERSING` | Off / On | Off | Car-style brake-then-reverse on the same stick. Not for multirotors. |
| Use hall sensors | `USE_HALL_SENSORS` | Off / On | Off | Sensored commutation. Only works on firmware built with hall-sensor support. ARK 4IN1 has none, so this is forced off. |
| Sine mode range | `SINE_MODE_RANGE` | 5–25% throttle | 15% | Throttle percent where sine mode hands off to normal BEMF commutation. |
| Brake strength | `BRAKE_STRENGTH` | 1–10 | 10 | How hard the stopped drag brake holds. Only used when Brake on stop is on. |
| Running brake level | `RUNNING_BRAKE_LEVEL` | 1–10 | 10 | Regen / dead-time while spinning. Lower is more braking and more FET heat. 10 is the least brake. |
| Temperature limit | `TEMPERATURE_LIMIT` | 70–140 °C (141+ is off) | Off | Starts folding duty back as FET temperature approaches this. The top of the slider disables it. |
| Current limit | `CURRENT_LIMIT` | 0–200 A (top of slider is off) | Off | Caps phase current using the onboard shunt and the Current P/I/D terms. The top of the slider disables it. ARK 4IN1 shares one shunt across four ESCs, so this is not a per-motor limiter. |
| Sine mode power | `SINE_MODE_POWER` | 1–10 | 6 | How hard sine mode drives. Raise if the motor will not break out in sine; lower if it jerks at the handoff. |
| Protocol | `ESC_PROTOCOL` | Auto, DShot, Servo, Serial, EDT ARM | DShot | Which throttle input to accept. Auto detects DShot vs servo PWM; DShot is what ARK ships. EDT ARM requires an Extended DShot Telemetry arm handshake before it will spin. |
| Auto timing advance | `AUTO_ADVANCE` | Off / On | Off | Picks timing from rpm (using Motor KV and poles) as the motor spins up. Overrides the Timing advance slider. |
| Startup melody | `STARTUP_MELODY` | Bluejay RTTTL, 128 bytes | ARK tune (erased / 0xFF) | Tune played on the motor at boot. All 0xFF plays the ARK “ARK” Morse tune. Only plays while the motor is not spinning. |
| CAN settings | `CAN_SETTINGS` | 16 bytes at offset 176 | zeros (node 0, index 0) | DroneCAN identity and options: node ID, ESC index, require-arming, telemetry rate, require-zero-throttle, input filter, debug rate, bus terminator. The configurator carries this block through every save and does not edit it. Reset-to-defaults also leaves it alone so four ESCs do not collapse onto one node ID. |
