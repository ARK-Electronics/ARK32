# ARK 4IN1 F051 + 900KV + 10″ prop — HWCI-derived SITL reference

Steady-state RPM map extracted from powered HWCI on the Tyto Flight Stand
(SETUP A, uni DShot), not a free-run noprop cal.

| Item | Value |
|------|--------|
| ESC | ARK 4IN1 F051 ch1 |
| Motor | 900 KV, 14 poles |
| Prop | HQProp 10×5×3 |
| Supply | 6S ≈ 24.9 V |
| Source run | `hwci/runs/pr65-aa42860-900kv-20260729_111732/suite50` |
| SITL model | `Mcu/SITL/models/ark_900kv_10inch.json` |

## Files

| File | Role |
|------|------|
| `expected.json` | Steady RPM + `sitl_gate` for CI |
| `suite50_ref.jsonl` | Subsampled hold/snap status from the suite |
| `capture_meta.json` | Machine-readable setup |

## SITL gate

```bash
# from repo root after make AM32_SITL_CAN
pytest Mcu/SITL/tests/test_bench_models.py -k 10inch -v
```

Only throttle levels listed under `sitl_gate.levels` are asserted (0.10–0.30).
Higher stick on the first-order plant still desyncs under CI load.
