from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Sequence, Tuple, Optional


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
    mdot_preheater_kgps: float = 0.0
    mdot_stack_cap_kgps: float = 0.0
    mdot_aux_flow_kgps: float = 0.0
    fan_circulation_power_kW: float = 0.0


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


# =========================================================
# New dynamic proxy / MPC data contracts
# =========================================================

@dataclass(frozen=True)
class PreheaterCellState:
    """One axial cell in the preheater digital twin.

    omega is the current wet-basis moisture of the material in the cell.
    omega0/tref/slope are the original equivalent properties carried by that
    material parcel; they are not re-identified during transport.
    """

    index: int
    z_center_m: float
    residence_left_s: float
    omega: float
    T_solid_C: float
    omega0: float
    tref_min: float
    slope_min_per_c: float


@dataclass(frozen=True)
class PreheaterState:
    time_s: float
    cells: Tuple[PreheaterCellState, ...]
    omega_out: float
    T_solid_out_C: float
    Tg_profile_C: Tuple[float, ...] = field(default_factory=tuple)


@dataclass(frozen=True)
class SlotControlQuery:
    """Local inverse-model query used by the dynamic lookup table."""

    time_s: float
    cell: PreheaterCellState
    omega_target: float
    T_stack_cap_C: float
    v_stack_cap_mps: float


@dataclass(frozen=True)
class DynamicSlotLookupResult:
    time_s: float
    Tg_star_C: float
    vg_star_mps: float
    omega_target: float
    omega_reachable: float
    feasible: bool
    power_kW: float
    Qsup_kW: float = 0.0
    Qreq_kW: float = 0.0
    mdot_stack_cap_kgps: float = float('inf')
    mdot_preheater_kgps: float = 0.0
    source: str = "dynamic_lookup"


@dataclass(frozen=True)
class ControlSetpoint:
    time_s: float
    Tg_ref_C: float
    vg_ref_mps: float
    source: str = "mpc"
    omega_target: float = 0.3218
    omega_reachable: float = 0.3218
    power_kW: float = 0.0
    Qreq_kW: float = 0.0
    Qsup_kW: float = 0.0
    mdot_stack_cap_kgps: float = float('inf')
    mdot_preheater_kgps: float = 0.0
    # Dynamic resource diagnostics. Temperature can be raised above
    # T_stack_available_C by auxiliary heat; mass flow cannot.
    T_stack_available_C: float = 930.0
    v_stack_available_mps: float = 18.0


@dataclass(frozen=True)
class MPCDecision:
    time_s: float
    Tg_ref_C: float
    vg_ref_mps: float
    omega_target: float
    omega_reachable: float
    predicted_Tavg_C: float
    predicted_omega_out: float
    cost: float
    feasible: bool
    source: str = "lookup_mpc"
    note: str = ""
