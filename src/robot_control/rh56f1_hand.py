"""RH56F1 canonical hand control: per-finger admittance (reference implementation) and its parameters.

The parameters are the contract in components/rh56f1.yaml (control.admittance). The sim2real EtherCAT master
runs the same law in C at 500 Hz (tools/ethercat/rh56f1_admittance.h) on the axes commanded on
/hand_<side>/angle_target; a sim2real test checks that the C and this module give the same numbers. Teleop
(motion_acq) and policies (sim2real) send targets only; a simulator ports this module so a policy meets
the same controller in training and on the hand.

레지스터는 닫을수록 작아진다. 축마다 매 주기:
    f   = max(force - bias - deadband, 0)   (손끝 촉각이 조용하면 / proximal_scale)
    y  += a (f / k + max(f - f_max, 0) / k_over - y),   a = dt / (tau + dt)
    cmd = target + y;  접촉 중 cmd >= actual - lead (1 - f / f_max)
"""
from __future__ import annotations

from dataclasses import dataclass, field, fields
from pathlib import Path

import yaml

N = 6
COMPONENT = Path(__file__).resolve().parents[2] / "components" / "rh56f1.yaml"


@dataclass(frozen=True)
class AdmParams:
    k_g_per_reg: float = 3.6
    deadband_g: float = 40.0
    tau_contact_s: float = 1.0
    tau_release_s: float = 0.15
    f_max_g: float = 800.0
    k_over_g_per_reg: float = 0.36
    lead_reg: float = 11.0
    max_offset_reg: float = 880.0
    proximal_scale: float = 1.0
    tip_on_counts: float = 20.0
    hold_band_g: float = 100.0
    joints: tuple[int, ...] = (1, 1, 1, 1, 1, 0)

    def __post_init__(self) -> None:
        if min(self.k_g_per_reg, self.k_over_g_per_reg, self.f_max_g, self.max_offset_reg) <= 0:
            raise ValueError("admittance: k, k_over, f_max, max_offset > 0")
        if min(self.deadband_g, self.tau_contact_s, self.tau_release_s, self.lead_reg, self.tip_on_counts,
               self.hold_band_g) < 0:
            raise ValueError("admittance: deadband, tau, lead, tip_on >= 0")
        if not 0 < self.proximal_scale <= 1 or len(self.joints) != N or any(j not in (0, 1) for j in self.joints):
            raise ValueError("admittance: proximal_scale in (0, 1], joints = 6 x 0/1")

    def argv(self) -> str:
        """마스터 --adm 값: 숫자 11 개 + joints 6 개, 쉼표."""
        vals = [getattr(self, f.name) for f in fields(self) if f.name != "joints"]
        return ",".join(f"{v:g}" for v in vals) + "," + ",".join(str(j) for j in self.joints)

    @classmethod
    def from_cfg(cls, raw: dict | None) -> "AdmParams":
        raw = dict(raw or {})
        known = {f.name for f in fields(cls)}
        unknown = sorted(set(raw) - known - {"enabled"})
        if unknown:
            raise ValueError(f"admittance: unknown keys {unknown}")
        raw.pop("enabled", None)
        if "joints" in raw:
            raw["joints"] = tuple(int(v) for v in raw["joints"])
        return cls(**{k: (v if k == "joints" else float(v)) for k, v in raw.items()})


@dataclass
class AdmState:
    y: list[float] = field(default_factory=lambda: [0.0] * N)
    f: list[float] = field(default_factory=lambda: [0.0] * N)
    bias: list[float] = field(default_factory=lambda: [0.0] * N)


def adm_step(p: AdmParams, s: AdmState, i: int, dt: float, target: float, actual: float, force: float,
             tip: float) -> float:
    """한 축 한 주기 — rh56f1_admittance.h adm_step 과 같다."""
    f = force - s.bias[i] - p.deadband_g
    if f < 0:
        f = 0.0
    if f > 0 and tip >= 0 and tip < p.tip_on_counts:
        f /= p.proximal_scale
    goal = f / p.k_g_per_reg + ((f - p.f_max_g) / p.k_over_g_per_reg if f > p.f_max_g else 0.0)
    goal = min(goal, p.max_offset_reg)
    if f > 0 and abs(f - p.k_g_per_reg * s.y[i]) < p.hold_band_g and f <= p.f_max_g:
        goal = s.y[i]   # inside the hold band: keep (the hand moves 3-5 registers at a time, 10.06)
    tau = p.tau_contact_s if f > 0 else p.tau_release_s
    a = 1.0 if tau <= 0 else dt / (tau + dt)
    s.y[i] += a * (goal - s.y[i])
    if s.y[i] < 0:
        s.y[i] = 0.0
    s.f[i] = f
    cmd = target + s.y[i]
    if f > 0:
        lead = max(p.lead_reg * (1.0 - f / p.f_max_g), 0.0)
        cmd = max(cmd, actual - lead)
    return cmd


def load_admittance(path: Path = COMPONENT) -> AdmParams:
    """control.admittance of the RH56F1 component contract."""
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    return AdmParams.from_cfg(((raw.get("control") or {}).get("admittance")))


def load_protection(path: Path = COMPONENT) -> dict:
    """protection block (SDO at driver start)."""
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    return dict(raw.get("protection") or {})

