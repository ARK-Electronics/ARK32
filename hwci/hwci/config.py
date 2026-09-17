"""Rig and test-profile configuration (YAML-backed).

Rig configs from files are validated STRICTLY: unknown keys and unknown
backend values are errors, and simulator backends are rejected in a rig file.
A typo must never silently turn a hardware run into a simulated one (or drop a
channel) while CI reports green - use the explicit ``--sim`` flag to simulate.
"""
from __future__ import annotations

import dataclasses
from dataclasses import dataclass, field
from pathlib import Path

import yaml

from .build import find_artifact
from .debugger.openocd import (APP_LOAD_ADDR, DEFAULT_CONFIGS, G4_APP_LOAD_ADDR,
                               G4_CONFIGS)
from .flightstand.base import SafetyLimits

PROFILES_DIR = Path(__file__).parent / "profiles"
DEFAULT_TARGET = "ARK_4IN1_F051"

# Allowed backend values. "sim" entries are valid only for the built-in
# default RigConfig (offline runs); load_rig() rejects them in a rig file.
BACKEND_CHOICES: dict[str, set[str]] = {
    "debugger_backend": {"openocd", "sim", "none"},
    "telem_backend": {"serial", "sim", "none"},
    # Text console on ARK_G431_CAN (USART2 TX PB3 → ST-Link VCP). Not KISS.
    "debug_uart_backend": {"serial", "sim", "none"},
    # "none" = ESC signal driven outside the harness (e.g. ARK FPV BDShot);
    # use only with setup B — see docs/BENCH_SETUPS.md.
    "throttle_backend": {"flightstand", "external", "none", "sim"},
    "stand_backend": {"grpc", "sim", "none"},
}
_SIM_ONLY = {"sim"}


@dataclass(frozen=True)
class TargetPreset:
    """MCU-family defaults applied when a rig omits openocd / load-address keys."""
    openocd_configs: tuple[str, ...]
    app_load_addr: int
    adapter_speed_khz: int | None = None


# Keys that load_rig() fills from TARGET_PRESETS when absent from the YAML.
_PRESET_FILL_KEYS = ("openocd_configs", "app_load_addr", "adapter_speed_khz")

TARGET_PRESETS: dict[str, TargetPreset] = {
    "ARK_4IN1_F051": TargetPreset(
        openocd_configs=tuple(DEFAULT_CONFIGS),
        app_load_addr=APP_LOAD_ADDR,
        adapter_speed_khz=2000,
    ),
    "ARK_G431_CAN": TargetPreset(
        openocd_configs=tuple(G4_CONFIGS),
        app_load_addr=G4_APP_LOAD_ADDR,
        # G4 SWD is solid well above F0 rates; 8 MHz cuts perf-poll latency.
        adapter_speed_khz=8000,
    ),
}

@dataclass
class Segment:
    """One phase of a test profile."""
    label: str
    throttle: float           # target throttle, 0..1
    duration_s: float
    ramp: bool = False        # ramp linearly from the previous throttle
    steady: bool = False      # use this segment for steady-state metrics


@dataclass
class SmokeGates:
    """Absolute pass/fail checks for a free-run / smoke profile.

    Independent of baseline regression compare. When enabled, a run that
    never spun (stuck-rotor latch, no running flag, etc.) fails CI even
    if the host did not abort on current/safety limits.

    Optional fields left None are skipped (so older firmware without
    ``perf_esc_state`` can still gate on running/RPM/BEMF).
    """
    enabled: bool = False
    # Steady segments at/above this throttle must look like "driving".
    min_throttle: float = 0.55
    # Fraction of samples in those segments with perf_running == 1.
    min_running_fraction: float = 0.90
    # Fail if any sample has stuck-rotor latch (firmware uses 102).
    max_bemf_latch_samples: int = 0
    # When perf_esc_state is present: high-throttle steady must be OPEN(4)
    # or CLOSED(5) for this fraction of samples.
    require_esc_drive_states: bool = True
    min_esc_drive_fraction: float = 0.90
    # Max final perf_esc_illegal_edge_count (0 = no illegal named edges).
    max_illegal_edges: int | None = 0
    # min_rpm keyed by throttle level (exact segment throttle match or
    # nearest segment at/above key). Example: {1.0: 30000} for full stick.
    min_rpm: dict = field(default_factory=dict)


@dataclass
class Profile:
    name: str
    description: str = ""
    sample_rate_hz: float = 100.0
    arm_settle_s: float = 2.0
    segments: list[Segment] = field(default_factory=list)
    safety: SafetyLimits = field(default_factory=SafetyLimits)
    # analysis knobs
    steady_tail_fraction: float = 0.5   # use last half of a steady segment
    demag_commutation_spike: float = 3.0  # x median commutation interval
    demag_rpm_drop_fraction: float = 0.25  # rpm fell >25% while throttle high
    # Offline fallback only: on a rig the motor's pole pairs come from
    # RigConfig (recorded into the run meta), not from the test profile.
    pole_pairs: int = 7
    smoke_gates: SmokeGates | None = None
    # Live EEPROM bytes that must match before the run starts. Empty = no
    # check. A mismatch aborts rather than producing a plausible wrong
    # number (e.g. brake_on_stop=1 makes a coast look like HIZ allOff()).
    require_eeprom: dict[str, int] = field(default_factory=dict)

    @property
    def duration_s(self) -> float:
        return sum(s.duration_s for s in self.segments)


def _safety_from(d: dict) -> SafetyLimits:
    d = d or {}
    return SafetyLimits(
        max_thrust_n=d.get("max_thrust_n"),
        max_current_a=d.get("max_current_a"),
        max_rpm=d.get("max_rpm"),
        max_voltage_v=d.get("max_voltage_v"),
        max_motor_temp_c=d.get("max_motor_temp_c"),
    )


def _smoke_gates_from(d: dict | None) -> SmokeGates | None:
    if not d:
        return None
    min_rpm = d.get("min_rpm") or {}
    # YAML may use float keys; normalize to float.
    min_rpm_f = {float(k): float(v) for k, v in min_rpm.items()}
    return SmokeGates(
        enabled=bool(d.get("enabled", True)),
        min_throttle=float(d.get("min_throttle", 0.55)),
        min_running_fraction=float(d.get("min_running_fraction", 0.90)),
        max_bemf_latch_samples=int(d.get("max_bemf_latch_samples", 0)),
        require_esc_drive_states=bool(d.get("require_esc_drive_states", True)),
        min_esc_drive_fraction=float(d.get("min_esc_drive_fraction", 0.90)),
        max_illegal_edges=(None if d.get("max_illegal_edges") is None
                           else int(d["max_illegal_edges"])),
        min_rpm=min_rpm_f,
    )


def profile_from_dict(d: dict) -> Profile:
    segs = [Segment(label=s.get("label", f"seg{i}"),
                    throttle=float(s["throttle"]),
                    duration_s=float(s["duration_s"]),
                    ramp=bool(s.get("ramp", False)),
                    steady=bool(s.get("steady", False)))
            for i, s in enumerate(d.get("segments", []))]
    return Profile(
        name=d["name"],
        description=d.get("description", ""),
        sample_rate_hz=float(d.get("sample_rate_hz", 100.0)),
        arm_settle_s=float(d.get("arm_settle_s", 2.0)),
        segments=segs,
        safety=_safety_from(d.get("safety")),
        steady_tail_fraction=float(d.get("steady_tail_fraction", 0.5)),
        demag_commutation_spike=float(d.get("demag_commutation_spike", 3.0)),
        demag_rpm_drop_fraction=float(d.get("demag_rpm_drop_fraction", 0.25)),
        pole_pairs=int(d.get("pole_pairs", 7)),
        smoke_gates=_smoke_gates_from(d.get("smoke_gates")),
        require_eeprom={str(k): int(v)
                        for k, v in (d.get("require_eeprom") or {}).items()},
    )


def profile_to_dict(profile: Profile) -> dict:
    """Serialize a Profile (inverse of :func:`profile_from_dict`).

    Stored in each run's ``meta.json`` so a run directory is self-describing:
    analysis re-uses the exact profile the run was made with, even if the
    profile YAML changed since (or was a custom file path).
    """
    return dataclasses.asdict(profile)


def load_profile(name_or_path: str) -> Profile:
    """Load a profile by built-in name (in profiles/) or by file path."""
    p = Path(name_or_path)
    if not p.exists():
        cand = PROFILES_DIR / f"{name_or_path}.yaml"
        if cand.exists():
            p = cand
        else:
            raise FileNotFoundError(
                f"profile {name_or_path!r} not found "
                f"(looked in {PROFILES_DIR} and as a path)")
    return profile_from_dict(yaml.safe_load(p.read_text()))


def list_profiles() -> list[str]:
    return sorted(p.stem for p in PROFILES_DIR.glob("*.yaml"))


@dataclass
class RigConfig:
    """How the host reaches the hardware. Defaults are the offline simulator."""
    target: str = DEFAULT_TARGET
    repo_root: str = str(Path(__file__).resolve().parents[2])
    obj_dir: str | None = None              # default <repo_root>/obj
    elf_path: str | None = None             # default: glob obj for the target

    debugger_backend: str = "sim"           # "openocd" | "none" (| "sim" offline)
    openocd_configs: list[str] = field(
        default_factory=lambda: list(DEFAULT_CONFIGS))
    openocd_search_dirs: list[str] = field(default_factory=list)
    openocd_bin: str = "openocd"
    app_load_addr: int = APP_LOAD_ADDR
    # None = OpenOCD default for the target cfg; set via target preset or YAML.
    adapter_speed_khz: int | None = None

    telem_backend: str = "sim"              # "serial" | "none" (| "sim" offline)
    telem_port: str = "/dev/ttyUSB0"
    telem_baud: int = 115200

    # ARK_G431_CAN debug console (USART2 TX PB3 @ 115200 via ST-Link VCP).
    # Not KISS telemetry — state transitions and fault events as text lines.
    debug_uart_backend: str = "none"        # "serial" | "none" (| "sim" offline)
    debug_uart_port: str = "/dev/esc-debug-uart"
    debug_uart_baud: int = 115200

    # flightstand = SETUP A; external = serial bridge; none = SETUP B (PX4/BDShot owns pin)
    throttle_backend: str = "sim"
    throttle_port: str = "/dev/ttyACM0"
    throttle_baud: int = 115200

    stand_backend: str = "sim"              # "grpc" | "none" (| "sim" offline)
    stand_host: str = "127.0.0.1"
    stand_port: int = 50051
    stand_signals: dict = field(default_factory=dict)

    motor_name: str = "sim-motor"
    pole_pairs: int = 7                     # the motor under test (authoritative)
    prop: str = "sim-prop"

    def resolved_obj_dir(self) -> Path:
        return Path(self.obj_dir) if self.obj_dir else Path(self.repo_root) / "obj"

    def resolved_elf(self) -> Path | None:
        if self.elf_path:
            return Path(self.elf_path)
        return find_artifact(self.resolved_obj_dir(), self.target, "elf")

    def validate(self, *, allow_sim_backends: bool = True) -> None:
        for key, choices in BACKEND_CHOICES.items():
            value = getattr(self, key)
            if value not in choices:
                raise ValueError(
                    f"rig config: {key} = {value!r} is not one of "
                    f"{sorted(choices)}")
            if not allow_sim_backends and value in _SIM_ONLY:
                raise ValueError(
                    f"rig config: {key} = 'sim' is not allowed in a rig file; "
                    "use a real backend or 'none' (or run with --sim for a "
                    "fully simulated run)")
        if self.throttle_backend == "flightstand" and self.stand_backend == "none":
            raise ValueError(
                "rig config: throttle_backend 'flightstand' needs a stand "
                "(stand_backend 'grpc'); use throttle_backend 'external' on a "
                "stand-less bench")


def apply_target_preset(cfg: RigConfig, *, explicit_keys: set[str] | None = None
                        ) -> RigConfig:
    """Fill openocd/app_load/adapter defaults from :data:`TARGET_PRESETS`.

    Only keys *not* listed in ``explicit_keys`` are overwritten, so a rig YAML
    can pin any of them. Call with ``explicit_keys=set(yaml)`` after load.
    """
    preset = TARGET_PRESETS.get(cfg.target)
    if preset is None:
        return cfg
    explicit = explicit_keys or set()
    if "openocd_configs" not in explicit:
        cfg.openocd_configs = list(preset.openocd_configs)
    if "app_load_addr" not in explicit:
        cfg.app_load_addr = preset.app_load_addr
    if "adapter_speed_khz" not in explicit:
        cfg.adapter_speed_khz = preset.adapter_speed_khz
    return cfg


def load_rig(path: str | None) -> RigConfig:
    if not path:
        return RigConfig()
    data = yaml.safe_load(Path(path).read_text()) or {}
    known = {f.name for f in dataclasses.fields(RigConfig)}
    unknown = sorted(set(data) - known)
    if unknown:
        raise ValueError(
            f"rig config {path}: unknown key(s) {unknown}; "
            f"valid keys: {sorted(known)}")
    cfg = RigConfig()
    for key, value in data.items():
        setattr(cfg, key, value)
    # Target-family SWD defaults (G4 app @ 0x08004000, stm32g4x, …) unless
    # the YAML pinned those keys explicitly.
    apply_target_preset(cfg, explicit_keys=set(data.keys()) & set(_PRESET_FILL_KEYS))
    # A rig FILE describes hardware: simulator backends in it are almost
    # certainly a typo'd or half-edited config, and silently simulating a
    # "hardware" run is the worst possible failure mode.
    cfg.validate(allow_sim_backends=False)
    return cfg
