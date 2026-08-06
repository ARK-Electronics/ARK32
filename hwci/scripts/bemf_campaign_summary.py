#!/usr/bin/env python3
"""Summarize free-run BEMF / startup HWCI runs into one markdown table set.

Usage:
  python scripts/bemf_campaign_summary.py runs/g491-bemf-* runs/g491-1800kv-*
  python scripts/bemf_campaign_summary.py --out runs/bemf_summary.md runs/...
"""
from __future__ import annotations

import argparse
import bisect
import json
import re
import sys
from collections import Counter, defaultdict
from pathlib import Path

# Allow running from hwci/ without install
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from hwci import metrics as metricsmod  # noqa: E402
from hwci.config import profile_from_dict  # noqa: E402
from hwci.model import RunResult  # noqa: E402


def _profile(result: RunResult):
    pd = result.meta.get("profile_def")
    if pd:
        return profile_from_dict(pd)
    from hwci.config import load_profile
    return load_profile(result.meta.get("profile", "ci_smoke"))


# esc_state_t values (Inc/esc_state.h). Only the two drive states matter here.
ESC_OPEN_LOOP = 4
ESC_CLOSED_LOOP = 5

_FAULT_RE = re.compile(r"fault:\s*(\S+)")
_TS_RE = re.compile(r"^(\d+\.\d+)\t(.*)$")


def _tier_label(throttle) -> str:
    """Group start* segments by commanded throttle, e.g. 0.08 -> '8%'."""
    try:
        return f"{round(float(throttle) * 100)}%"
    except (TypeError, ValueError):
        return "?"


def residency_by_tier(rows: list[dict]) -> list[dict]:
    """Open- vs closed-loop sample residency within each start* throttle tier.

    This is the metric that actually tracks sync quality. Raw desync-line
    counts do not: three G431 tuning iterations differed 22/12/12 on desync
    lines while their open-loop residency was identical to within counting
    noise (2026-08). Reported per tier because the deficit vs the F051
    reference is concentrated at the low-BEMF tiers.
    """
    agg: dict[str, Counter] = defaultdict(Counter)
    segs: dict[str, set] = defaultdict(set)
    for r in rows:
        seg = str(r.get("segment") or "")
        if not seg.startswith("start"):
            continue
        state = r.get("perf_esc_state")
        if state is None:
            continue
        tier = _tier_label(r.get("throttle_cmd"))
        segs[tier].add(seg)
        if int(state) == ESC_OPEN_LOOP:
            agg[tier]["open"] += 1
        elif int(state) == ESC_CLOSED_LOOP:
            agg[tier]["closed"] += 1
    out = []
    for tier, c in agg.items():
        drive = c["open"] + c["closed"]
        out.append({
            "tier": tier,
            "segments": len(segs[tier]),
            "open": c["open"],
            "closed": c["closed"],
            "pct_open": (100.0 * c["open"] / drive) if drive else None,
        })
    out.sort(key=lambda d: float(d["tier"].rstrip("%")) if d["tier"] != "?" else 1e9)
    return out


def faults_by_tier(rows: list[dict], debug_text: str | None) -> list[dict]:
    """Attribute each debug-UART ``fault:`` line to the segment it landed in.

    ``perf_host_t`` in samples.csv and the debug_uart.log timestamps are the
    same host monotonic clock, so this is an exact lookup rather than a
    reconstruction from nominal segment timing.
    """
    if not debug_text:
        return []
    stamps: list[float] = []
    labels: list[str] = []
    for r in rows:
        ht = r.get("perf_host_t")
        if ht is None:
            continue
        stamps.append(float(ht))
        seg = str(r.get("segment") or "")
        labels.append(_tier_label(r.get("throttle_cmd"))
                      if seg.startswith("start") else "coast/idle")
    if not stamps:
        return []

    counts: dict[str, Counter] = defaultdict(Counter)
    kinds: set[str] = set()
    for line in debug_text.splitlines():
        m = _TS_RE.match(line)
        if not m:
            continue
        ts, body = float(m.group(1)), m.group(2)
        fm = _FAULT_RE.search(body)
        if not fm:
            continue
        kind = fm.group(1)
        # Ignore the idle signal_lost reboot loop between flash and run start.
        if kind == "signal_lost":
            continue
        i = bisect.bisect_left(stamps, ts)
        if i >= len(stamps):
            i = len(stamps) - 1
        if i > 0 and abs(stamps[i - 1] - ts) < abs(stamps[i] - ts):
            i -= 1
        # Outside the sampled window entirely (pre-run backlog).
        if not (stamps[0] - 1.0 <= ts <= stamps[-1] + 1.0):
            continue
        counts[labels[i]][kind] += 1
        kinds.add(kind)

    order = sorted(counts, key=lambda t: (t == "coast/idle",
                                          float(t.rstrip("%")) if t != "coast/idle" else 0))
    return [{"tier": t, "kinds": dict(counts[t]),
             "total": sum(counts[t].values())} for t in order]


def summarize_run(run_dir: Path) -> dict | None:
    if not (run_dir / "samples.csv").exists():
        return None
    result = RunResult.load(run_dir)
    profile = _profile(result)
    m = metricsmod.compute(result, profile)
    log_path = run_dir / "debug_uart.log"
    debug_text = log_path.read_text(errors="replace") if log_path.exists() else None
    summary = m.get("summary") or {}
    starts = m.get("startup") or {}
    steady = m.get("steady_points") or []
    demag = m.get("demag") or {}
    return {
        "dir": str(run_dir),
        "name": run_dir.name,
        "profile": result.meta.get("profile"),
        "target": result.meta.get("target"),
        "motor": result.meta.get("motor"),
        "aborted": result.meta.get("aborted"),
        "n_samples": result.meta.get("n_samples"),
        "summary": summary,
        "startup": starts,
        "steady": steady,
        "demag": demag,
        "debug_faults": result.meta.get("debug_uart_faults"),
        "debug_lines": result.meta.get("debug_uart_line_count"),
        "residency": residency_by_tier(result.rows),
        "faults_by_tier": faults_by_tier(result.rows, debug_text),
    }


def md_escape(s) -> str:
    return str(s).replace("|", "\\|")


def render(runs: list[dict]) -> str:
    lines = [
        "# BEMF / startup free-run data collection",
        "",
        f"Runs: **{len(runs)}**",
        "",
        "## Run index",
        "",
        "| run | profile | aborted | demag | blind_zc | max_rpm | max_I | starts fail/att |",
        "|---|---|---|---|---|---|---|---|",
    ]
    for r in runs:
        s = r["summary"]
        st = r["startup"]
        d = r["demag"]
        att = st.get("attempts") or 0
        fail = st.get("failures") or 0
        lines.append(
            f"| `{r['name']}` | {r['profile']} | {r['aborted'] or '—'} | "
            f"{d.get('event_count', s.get('demag_events', '—'))} | "
            f"{s.get('zc_blind_steps_total', '—')} | "
            f"{_rpm(r)} | {s.get('max_current_a', '—')} | "
            f"{fail}/{att} |"
        )

    lines += ["", "## Startup reliability (start* segments)", ""]
    for r in runs:
        st = r["startup"]
        if not st.get("attempts"):
            continue
        lines += [
            f"### `{r['name']}` — {st.get('failures', 0)} fail / "
            f"{st.get('attempts', 0)} att "
            f"(mean ttr {st.get('time_to_run_ms_mean')} ms, "
            f"max {st.get('time_to_run_ms_max')} ms)",
            "",
            "| segment | ok | time_to_run_ms |",
            "|---|---|---|",
        ]
        for a in st.get("per_attempt") or []:
            lines.append(
                f"| {a['segment']} | {'✅' if a['success'] else '❌'} | "
                f"{a.get('time_to_run_ms') if a['success'] else '—'} |"
            )
        lines.append("")

    lines += [
        "", "## Closed-loop retention by start tier", "",
        "Share of *driving* samples spent in OPEN_LOOP within each start\\* "
        "tier (lower is better). Reference: ARK 4IN1 F051, JS2306 1800 KV, "
        "no prop, 8% tier = **0.77%** "
        "(`ark-release-1800kv-startup-reliability-20260727_185930`).",
        "",
    ]
    for r in runs:
        if not r["residency"]:
            continue
        lines += [
            f"### `{r['name']}` ({r['target']})",
            "",
            "| tier | segs | open | closed | % open |",
            "|---|---|---|---|---|",
        ]
        for d in r["residency"]:
            lines.append(
                f"| {d['tier']} | {d['segments']} | {d['open']} | {d['closed']} | "
                f"{_fmt(d['pct_open'], 2)}% |"
            )
        lines.append("")

    lines += [
        "", "## Faults by start tier", "",
        "Each `fault:` console line mapped to the segment it occurred in via "
        "the shared `perf_host_t` clock. Pre-run `signal_lost` reboot backlog "
        "is excluded.",
        "",
    ]
    for r in runs:
        if not r["faults_by_tier"]:
            continue
        kinds = sorted({k for d in r["faults_by_tier"] for k in d["kinds"]})
        lines += [
            f"### `{r['name']}`",
            "",
            "| tier | " + " | ".join(kinds) + " | total |",
            "|---" * (len(kinds) + 2) + "|",
        ]
        for d in r["faults_by_tier"]:
            cells = " | ".join(str(d["kinds"].get(k, 0)) for k in kinds)
            lines.append(f"| {d['tier']} | {cells} | {d['total']} |")
        lines.append("")

    lines += ["", "## Steady-state ZC / filter metrics", ""]
    for r in runs:
        if not r["steady"]:
            continue
        lines += [
            f"### `{r['name']}` ({r['profile']})",
            "",
            "| seg | thr | rpm | eRPM~ | I_A | zc_jit% | zc_jit_max% | "
            "confirm/zc | phase_peak | phase_bin |",
            "|---|---|---|---|---|---|---|---|---|---|",
        ]
        for p in r["steady"]:
            rpm = p.get("rpm") or 0
            erpm = rpm * 7  # 14-pole free-run article
            lines.append(
                f"| {p.get('segment')} | {p.get('throttle')} | "
                f"{_fmt(rpm, 0)} | {_fmt(erpm, 0)} | {_fmt(p.get('current_a'), 3)} | "
                f"{_fmt(p.get('zc_jitter_pct'), 3)} | "
                f"{_fmt(p.get('zc_jitter_max_pct'), 2)} | "
                f"{_fmt(p.get('confirm_rejects_per_zc'), 2)} | "
                f"{_fmt(p.get('zc_phase_peak_ratio'), 2)} | "
                f"{p.get('zc_phase_peak_bin')} |"
            )
        lines.append("")

    lines += [
        "## Tuning notes (fill from data)",
        "",
        "- **Reliable min start throttle:** (lowest start* level with 0 failures)",
        "- **Weak-BEMF floor:** (lowest crawl hold that stays running, jitter bound)",
        "- **High-eRPM jitter hump:** (throttle/RPM where zc_jitter_pct peaks)",
        "- **Confirm reject pressure:** (where confirm_rejects_per_zc rises)",
        "- **Startup power / min duty candidates:** …",
        "- **PWM frequency / advance trade:** …",
        "",
    ]
    return "\n".join(lines)


def _fmt(v, nd=2):
    if v is None:
        return "—"
    try:
        return f"{float(v):.{nd}f}"
    except (TypeError, ValueError):
        return str(v)


def _rpm(r: dict):
    s = r["summary"]
    # prefer max from steady
    pts = r["steady"]
    if pts:
        return max((p.get("rpm") or 0) for p in pts)
    return s.get("max_rpm")  # may be absent


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("runs", nargs="+", type=Path, help="run directories")
    ap.add_argument("--out", type=Path, help="write markdown here")
    args = ap.parse_args()

    runs = []
    for p in args.runs:
        if p.is_dir():
            s = summarize_run(p)
            if s:
                runs.append(s)
        else:
            print(f"skip missing {p}", file=sys.stderr)

    text = render(runs)
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text)
        print(f"wrote {args.out}")
    else:
        print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
