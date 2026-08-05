#!/usr/bin/env python3
"""Summarize free-run BEMF / startup HWCI runs into one markdown table set.

Usage:
  python scripts/bemf_campaign_summary.py runs/g491-bemf-* runs/g491-1800kv-*
  python scripts/bemf_campaign_summary.py --out runs/bemf_summary.md runs/...
"""
from __future__ import annotations

import argparse
import json
import sys
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


def summarize_run(run_dir: Path) -> dict | None:
    if not (run_dir / "samples.csv").exists():
        return None
    result = RunResult.load(run_dir)
    profile = _profile(result)
    m = metricsmod.compute(result, profile)
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
