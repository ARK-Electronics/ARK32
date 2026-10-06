"""Summarize an interleaved two-firmware A/B (e.g. upstream AM32 vs ARK32).

Each arm is a set of run directories (``hwci run --out ...``) of the SAME
profile on the SAME rig, captured alternately (see
``hwci/docs/UPSTREAM_AB.md``). For every steady-point metric, run summary
metric and snap/step spool-up time this reports each arm's n, median and range,
then applies the protocol's decision rule mechanically:

* fewer than ``min_n`` runs on either arm -> ``n<min`` (no call);
* the two arms' ranges overlap               -> ``within spread`` (no call);
* the ranges are disjoint                    -> ``B higher`` / ``B lower``.

Disjoint ranges is deliberately strict: it is the exact permutation-test
outcome "every B run beat every A run", whose two-sided p-value is
2 / C(nA + nB, nA) - 0.10 at 3 v 3, 0.029 at 4 v 4, 0.008 at 5 v 5. Whether
"higher" is better depends on the metric (current, CPU load and jitter are
lower-is-better; thrust and efficiency higher-is-better); the table states
direction only.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from . import metrics as metricsmod
from .config import Profile, load_profile, profile_from_dict
from .model import RunResult

# Per-steady-point metrics compared across arms (keys of metrics.compute()
# steady_points entries).
STEADY_KEYS = (
    "rpm", "thrust_gf", "current_a", "elec_power_w", "eff_gf_per_w",
    "cpu_load_pct", "ctrl_exec_us_max", "main_loop_us_max",
    "zc_jitter_pct", "zc_jitter_max_pct", "confirm_rejects_per_zc",
    "fet_temp_c", "motor_temp_c",
)

# Whole-run summary metrics compared across arms.
SUMMARY_KEYS = (
    "max_current_a", "max_cpu_load_pct", "worst_main_loop_us_steady",
    "worst_ctrl_exec_us_steady", "demag_events", "bemf_timeout_samples",
    "zc_blind_steps_total", "start_failures", "start_time_to_run_ms_max",
    "max_fet_temp_c", "max_motor_temp_c",
)

# A step/snap segment: commanded throttle rises at least this much over the
# previous segment without a ramp (demag_step_stress "step*", ramp_snap "snap*").
STEP_MIN_RISE = 0.2


def _profile_for_run(run: RunResult) -> Profile:
    pd = run.meta.get("profile_def")
    if pd:
        return profile_from_dict(pd)
    return load_profile(run.meta["profile"])


def _col(rows: list[dict], key: str) -> np.ndarray:
    out = np.full(len(rows), np.nan)
    for i, r in enumerate(rows):
        v = r.get(key, "")
        if v not in ("", None):
            try:
                out[i] = float(v)
            except (TypeError, ValueError):
                pass
    return out


def spool_times_ms(run: RunResult, profile: Profile, frac: float = 0.9) -> dict:
    """Time from each step/snap segment's start to ``frac`` of its settled rpm.

    Settled rpm is the mean over the segment's last ``steady_tail_fraction``
    (the same tail the steady metrics use). Stand optical rpm is preferred;
    without it, SWD eRPM / pole pairs. A segment that never reaches the
    threshold reports ``None`` - a desync or stall, not a slow spool.
    """
    rows = run.rows
    seg = np.array([r.get("segment") for r in rows], dtype=object)
    t = _col(rows, "t")
    rpm = _col(rows, "stand_rpm")
    if np.all(np.isnan(rpm)):
        pp = int(run.meta.get("pole_pairs") or profile.pole_pairs or 7)
        rpm = _col(rows, "perf_e_rpm") / pp
    out: dict[str, float | None] = {}
    prev = 0.0
    for s in profile.segments:
        rise = s.throttle - prev
        prev = s.throttle
        if s.ramp or rise < STEP_MIN_RISE:
            continue
        idx = np.where(seg == s.label)[0]
        if idx.size < 4:
            continue
        n_tail = max(1, int(round(idx.size * profile.steady_tail_fraction)))
        target = np.nanmean(rpm[idx[-n_tail:]])
        if not np.isfinite(target) or target <= 0:
            out[s.label] = None
            continue
        hit = idx[rpm[idx] >= frac * target]
        out[s.label] = (round(float((t[hit[0]] - t[idx[0]]) * 1000.0), 1)
                        if hit.size else None)
    return out


def run_values(run_dir: str | Path) -> tuple[dict, dict]:
    """Flat ``{metric_key: value}`` for one run dir, plus its meta.

    Uses ``metrics.json`` when ``hwci analyze`` already wrote it, otherwise
    computes it from the samples (same code path as ``hwci analyze``).
    """
    run_dir = Path(run_dir)
    run = RunResult.load(run_dir)
    profile = _profile_for_run(run)
    mpath = run_dir / "metrics.json"
    m = (json.loads(mpath.read_text()) if mpath.exists()
         else metricsmod.compute(run, profile))
    vals: dict = {}
    for p in m.get("steady_points", []):
        for k in STEADY_KEYS:
            vals[f"{p['segment']}.{k}"] = p.get(k)
    for k in SUMMARY_KEYS:
        vals[f"summary.{k}"] = m.get("summary", {}).get(k)
    for label, ms in spool_times_ms(run, profile).items():
        vals[f"{label}.spool_t90_ms"] = ms
    # A safety abort (host desync/current/thrust cutoff) is itself an A/B
    # outcome: count it, and the truncated run's later metrics read as missing.
    vals["summary.aborted"] = 1 if run.meta.get("aborted") else 0
    return vals, run.meta


def _finite(v) -> bool:
    return v is not None and not (isinstance(v, float) and math.isnan(v))


@dataclass
class ArmStats:
    n: int
    median: float | None
    lo: float | None
    hi: float | None


def _stats(values: list) -> ArmStats:
    xs = sorted(float(v) for v in values if _finite(v))
    if not xs:
        return ArmStats(0, None, None, None)
    return ArmStats(len(xs), float(np.median(xs)), xs[0], xs[-1])


@dataclass
class Row:
    key: str
    a: ArmStats
    b: ArmStats
    delta_pct: float | None
    verdict: str


def compare(a_runs: list[dict], b_runs: list[dict], min_n: int = 3) -> list[Row]:
    """Apply the decision rule to every metric key present in either arm."""
    keys: list[str] = []
    for r in a_runs + b_runs:
        for k in r:
            if k not in keys:
                keys.append(k)
    rows = []
    for k in keys:
        a = _stats([r.get(k) for r in a_runs])
        b = _stats([r.get(k) for r in b_runs])
        delta = None
        if a.median not in (None, 0.0) and b.median is not None:
            delta = round((b.median - a.median) / abs(a.median) * 100.0, 1)
        if a.n < min_n or b.n < min_n:
            verdict = "n<min"
        elif b.lo > a.hi:
            verdict = "B higher"
        elif b.hi < a.lo:
            verdict = "B lower"
        else:
            verdict = "within spread"
        rows.append(Row(k, a, b, delta, verdict))
    return rows


def _fmt(v: float | None) -> str:
    if v is None:
        return "-"
    return f"{v:.4g}"


def to_markdown(rows: list[Row], a_label: str = "A", b_label: str = "B",
                header: str = "") -> str:
    out = []
    if header:
        out += [header, ""]
    out += [f"| metric | {a_label} n | {a_label} median [range] | "
            f"{b_label} n | {b_label} median [range] | delta | verdict |",
            "| --- | ---: | --- | ---: | --- | ---: | --- |"]
    for r in rows:
        verdict = r.verdict.replace("B ", f"{b_label} ")
        out.append(
            f"| {r.key} | {r.a.n} | {_fmt(r.a.median)} [{_fmt(r.a.lo)}..{_fmt(r.a.hi)}] | "
            f"{r.b.n} | {_fmt(r.b.median)} [{_fmt(r.b.lo)}..{_fmt(r.b.hi)}] | "
            f"{'-' if r.delta_pct is None else f'{r.delta_pct:+.1f}%'} | {verdict} |")
    return "\n".join(out) + "\n"


def summarize_dirs(a_dirs: list[str], b_dirs: list[str], *, min_n: int = 3,
                   a_label: str = "A", b_label: str = "B") -> tuple[str, list[Row]]:
    """Load both arms' run dirs, check they are comparable, and tabulate."""
    a_vals, b_vals, metas = [], [], []
    for dirs, sink in ((a_dirs, a_vals), (b_dirs, b_vals)):
        for d in dirs:
            v, meta = run_values(d)
            sink.append(v)
            metas.append(meta)
    profiles = {m.get("profile") for m in metas}
    if len(profiles) != 1:
        raise ValueError(f"A/B runs mix profiles {sorted(map(str, profiles))}; "
                         "compare one profile at a time")
    a_sha = {m.get("git_sha") for m in metas[:len(a_dirs)]}
    b_sha = {m.get("git_sha") for m in metas[len(a_dirs):]}
    notes = [f"profile `{profiles.pop()}`; {a_label} git_sha {sorted(map(str, a_sha))}, "
             f"{b_label} git_sha {sorted(map(str, b_sha))}; decision rule: "
             f"n >= {min_n} per arm and disjoint ranges"]
    if a_sha & b_sha:
        notes.append("WARNING: both arms report the same git_sha - check each "
                     "arm's rig config repo_root points at its own checkout")
    aborted = [f"{d}: {m['aborted']}" for d, m in zip(a_dirs + b_dirs, metas)
               if m.get("aborted")]
    if aborted:
        notes.append("Aborted runs:\n" + "\n".join(f"- {a}" for a in aborted))
    rows = compare(a_vals, b_vals, min_n=min_n)
    return to_markdown(rows, a_label, b_label, header="\n\n".join(notes)), rows
