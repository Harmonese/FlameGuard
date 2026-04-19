from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Sequence, Tuple


@dataclass(frozen=True)
class FurnaceObservation:
    time_s: float
    T_avg_C: float
    T_stack_C: float
    v_stack_mps: float


@dataclass(frozen=True)
class FeedObservation:
    time_s: float
    composition: Sequence[float]
    normalize: bool = False


@dataclass(frozen=True)
class WeightedMoistureEstimate:
    omega_from_Tavg: float
    omega_from_Tstack: float
    omega_from_vstack: float
    omega_fused: float


@dataclass(frozen=True)
class ControllerOutput:
    time_s: float
    omega_est: float
    omega_req: float
    error_C: float
    T_set_C: float


@dataclass(frozen=True)
class GovernorDecision:
    time_s: float
    omega_est: float
    omega_req: float
    omega_tar: float
    dyn_band: Tuple[float, float]
    Tavg_pred_C: float
    note: str = ""


@dataclass(frozen=True)
class SlotState:
    time_s: float
    omega0: float
    tref_min: float
    slope_min_per_c: float


@dataclass(frozen=True)
class OptimizerResponseLite:
    time_s: float
    Tg_star_C: float
    vg_star_mps: float
    Tm_star_C: float
    omega_opt: float
    power_kW: float
    success: bool
    diagnostics: Dict[str, float] = field(default_factory=dict)


@dataclass(frozen=True)
class ActuatorCommand:
    time_s: float
    Tg_cmd_C: float
    vg_cmd_mps: float
    heater_enable: bool
    Q_heat_deficit_kW: float
    resource_limited: bool
    Tg_limited_by_stack: bool
    vg_limited_by_stack: bool


@dataclass(frozen=True)
class PreheaterLinearizedGains:
    K_wT: float
    K_wv: float
    K_wx: float
    reference_note: str = "Derived from local regression on optimizer lookup table"


DEFAULT_PREHEATER_GAINS = PreheaterLinearizedGains(
    K_wT=-4.85e-4,
    K_wv=-2.62e-2,
    K_wx=6.45e-1,
)
