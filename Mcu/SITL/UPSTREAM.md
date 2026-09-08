# Upstream SITL alignment

Baseline: [am32-firmware/AM32@79af09b00c52021895cddff4080a983aa1b1279c](https://github.com/am32-firmware/AM32/commit/79af09b00c52021895cddff4080a983aa1b1279c),
the merged SITL implementation from PR #394 and its follow-up fixes, reviewed
on 2026-09-08. This is a pinned comparison, not a claim to track future changes.

The MCU simulator scheduler, input/ADC/peripheral emulation and motor physics
already match this upstream baseline semantically apart from ARK extensions
and formatting. Preserve the ARK zero-cross fault injector, extended state
telemetry, product control hooks, models and pytest suite when syncing.

`run_upstream_tests.py` imports upstream's complete behavioral runner, including
simulation-time waits, paced DroneCAN tests for busy hosts, repeated EDT
commands until acknowledged, audio/tone tests, direction and 3D behavior,
stuck-rotor recovery, debugger pause, parameter round trips and FC capture.
Upstream reference model/parameter pairs are also included and validated.
The large upstream raw bench captures are not needed by this behavioral suite;
ARK calibration continues to use its own committed captures and tolerances.

Local adaptations to the upstream runner are deliberately small:

- ARK32 artifact prefix and ports below the Windows ephemeral range.
- ARK Morse startup signature and 1047 Hz physics-audio expectation, from
  `Src/sounds.c`, replacing upstream AM32's three-note tune.
- Existing ARK calibration pairs plus the three upstream reference pairs.
- Required CAN prerequisites in dedicated CI and fatal sanitizer diagnostics.
- A missing binary produces a useful error instead of a `None` path exception.

CI runs the upstream suite on Linux, Linux ASan/UBSan, and macOS with its
multicast loopback route. Existing ARK pytest, GUI and Windows smoke jobs remain.

```sh
make AM32_SITL_CAN
python3 -m pip install -r Mcu/SITL/requirements-ci.txt
mkdir -p upstream_test_run
cd upstream_test_run
SITL_REQUIRE_CAN=1 python3 ../Mcu/SITL/run_upstream_tests.py --sitl ../obj/ARK32_AM32_SITL_CAN_*.elf
```

For the next sync, diff against the SHA above, port new shared simulator
behavior and tests, then run both suites and the ARK calibration gates.
Do not replace ARK control-path tests or change their expected behavior merely
to match an upstream default. Update this baseline and enumerate local deltas.
