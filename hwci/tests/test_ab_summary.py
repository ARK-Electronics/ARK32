"""Interleaved firmware A/B summary (hwci ab-summary, hwci/docs/UPSTREAM_AB.md)."""
import pytest

from hwci import ab
from hwci.cli import main
from hwci.config import Profile, Segment, profile_to_dict
from hwci.model import RunResult


def _profile(name="ab_synth"):
    return Profile(name=name, sample_rate_hz=100.0, steady_tail_fraction=0.5,
                   segments=[Segment("idle", 0.0, 0.5),
                             Segment("ramp", 0.3, 0.5, ramp=True),
                             Segment("hold", 0.3, 1.0, steady=True),
                             Segment("snap", 0.9, 1.0),
                             Segment("dud", 0.0, 0.5),
                             Segment("stall", 0.9, 1.0)])


def _run(profile, hold_rpm, snap_rise_s, sha="aaaaaaa", stall=True):
    """Rows at 100 Hz: hold at hold_rpm, snap rises linearly to 2x over
    snap_rise_s then holds, 'stall' segment never spins up."""
    rows, t = [], 0.0
    for s in profile.segments:
        n = int(round(s.duration_s * 100))
        for i in range(n):
            if s.label == "hold":
                rpm = hold_rpm
            elif s.label == "snap":
                ts = i / 100.0
                rpm = hold_rpm * (1.0 + min(ts / snap_rise_s, 1.0))
            elif s.label == "stall":
                rpm = 0.0 if stall else hold_rpm
            else:
                rpm = 0.0
            rows.append({"t": round(t, 6), "segment": s.label,
                         "throttle_cmd": s.throttle, "stand_rpm": rpm})
            t += 0.01
    meta = {"profile": profile.name, "profile_def": profile_to_dict(profile),
            "pole_pairs": 7, "git_sha": sha, "target": "ARK_4IN1_F051"}
    return RunResult(meta=meta, rows=rows)


def test_compare_decision_rule():
    a = [{"x": 1.0, "y": 1.0, "z": 5.0}, {"x": 2.0, "y": 3.0, "z": None},
         {"x": 3.0, "y": 5.0, "z": 5.0}]
    b = [{"x": 4.0, "y": 2.0, "z": 6.0}, {"x": 5.0, "y": 4.0, "z": 6.0},
         {"x": 6.0, "y": 6.0, "z": 6.0}]
    rows = {r.key: r for r in ab.compare(a, b, min_n=3)}
    assert rows["x"].verdict == "B higher"          # ranges [1..3] vs [4..6]
    assert rows["x"].delta_pct == pytest.approx(150.0)
    assert rows["y"].verdict == "within spread"     # overlapping ranges
    assert rows["z"].verdict == "n<min"             # one A run missing z
    assert rows["z"].a.n == 2
    lower = {r.key: r for r in ab.compare(b, a, min_n=3)}
    assert lower["x"].verdict == "B lower"


def test_spool_t90_steps_only():
    p = _profile()
    # rise 0.5 s from hold to 2x hold: 90% of 2x = 1.8x -> 0.8 of the rise
    sp = ab.spool_times_ms(_run(p, 10000.0, 0.5), p)
    assert set(sp) == {"snap", "stall"}            # ramp/hold/dud are not steps
    assert sp["snap"] == pytest.approx(400.0, abs=15.0)
    assert sp["stall"] is None                     # never spun: not a time


def test_summarize_dirs_end_to_end(tmp_path):
    p = _profile()
    a_dirs, b_dirs = [], []
    for i in range(3):
        a_dirs.append(str(_run(p, 10000.0 + 10 * i, 0.5, sha="up1").save(tmp_path / f"a{i}")))
        b_dirs.append(str(_run(p, 12000.0 + 10 * i, 0.2, sha="ark1").save(tmp_path / f"b{i}")))
    md, rows = ab.summarize_dirs(a_dirs, b_dirs, a_label="upstream", b_label="ark32")
    by = {r.key: r for r in rows}
    assert by["hold.rpm"].verdict == "B higher"
    assert by["snap.spool_t90_ms"].verdict == "B lower"
    assert "| hold.rpm | 3 |" in md
    assert "ark32 higher" in md
    assert "WARNING" not in md

    out = tmp_path / "ab.md"
    assert main(["ab-summary", "--a", *a_dirs, "--b", *b_dirs,
                 "--out", str(out)]) == 0
    assert "hold.rpm" in out.read_text()


def test_summarize_rejects_mixed_profiles_and_flags_same_sha(tmp_path):
    p1, p2 = _profile("one"), _profile("two")
    a = [str(_run(p1, 10000.0, 0.5).save(tmp_path / "a"))]
    b = [str(_run(p2, 10000.0, 0.5).save(tmp_path / "b"))]
    with pytest.raises(ValueError):
        ab.summarize_dirs(a, b)
    assert main(["ab-summary", "--a", *a, "--b", *b]) == 2

    b_same = [str(_run(p1, 11000.0, 0.5).save(tmp_path / "b_same"))]
    md, _ = ab.summarize_dirs(a, b_same, min_n=1)
    assert "WARNING" in md                          # same git_sha on both arms


def test_aborted_runs_are_counted_and_listed(tmp_path):
    p = _profile()
    a_dirs, b_dirs = [], []
    for i in range(3):
        a_dirs.append(str(_run(p, 10000.0, 0.5, sha="up1").save(tmp_path / f"a{i}")))
        r = _run(p, 10000.0, 0.5, sha="ark1")
        r.meta["aborted"] = "desync: test"
        b_dirs.append(str(r.save(tmp_path / f"b{i}")))
    md, rows = ab.summarize_dirs(a_dirs, b_dirs)
    by = {r.key: r for r in rows}
    assert by["summary.aborted"].a.median == 0
    assert by["summary.aborted"].b.median == 1
    assert by["summary.aborted"].verdict == "B higher"
    assert "Aborted runs:" in md and "desync: test" in md
