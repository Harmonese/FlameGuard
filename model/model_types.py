from __future__ import annotations

"""Canonical model-layer dataclasses and configuration.

These types used to live in ``optimizer/optimizer.py`` because the first
implementation centered on a static SLSQP optimizer.  They are now owned by the
model layer so that Python proxy models, future COMSOL co-simulation adapters,
and hardware adapters can share the same contracts without importing an
optimizer module.
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class EquivalentProperties:
    """Equivalent material properties carried by a waste parcel or cell."""

    omega0: float
    tref_min: float
    slope_min_per_c: float


@dataclass(frozen=True)
class ResourceBoundary:
    """Natural flue-gas resource boundary at the current operating point."""

    T_stack_cap_C: float
    v_stack_cap_mps: float


@dataclass(frozen=True)
class DynamicTargetBand:
    """Legacy/static optimizer target band in wet-basis moisture fraction."""

    omega_min: float
    omega_max: float


@dataclass(frozen=True)
class OptimizerRequest:
    """Legacy static optimizer request.

    Kept in the model-layer type module because some lookup/fallback tools still
    use this compact request format.  The online NMPC path does not depend on it.
    """

    props: EquivalentProperties
    omega_tar: float
    resource: ResourceBoundary
    dyn_band: DynamicTargetBand
    slot_id: str = ""
    burn_policy: str = "advisory"


@dataclass
class OptimizerResult:
    """Legacy static optimizer result schema."""

    success: bool
    slot_id: str
    burn_policy: str
    omega_tar_requested: float
    omega_tar_projected: float
    omega_band_applied_min: float
    omega_band_applied_max: float
    Tg_star_C: float
    vg_star_mps: float
    Tm_star_C: float
    omega_opt: float
    w_opt_percent: float
    d_omega_minus: float
    d_omega_plus: float
    tau20_min: float
    tau_target_min: float
    tau_r_min: float
    Qsup_kW: float
    Qreq_kW: float
    power_kW: float
    mdot_stack_cap_kgps: float
    mdot_preheater_kgps: float
    Tavg_burn_C: float
    Tmin_burn_C: float
    Tmax_burn_C: float
    sigma_burn_C: float
    steady_band_violation_percent: float
    feasible_under_strict_burn: bool
    stage1_objective: float
    stage2_objective: float
    stage3_objective: float
    max_constraint_violation: float
    message: str = ""


@dataclass
class Config:
    """Shared engineering constants for proxy models and optimizers."""

    # Reference and ambient temperatures
    T_REF: float = 175.0
    T0: float = 20.0
    T_AMB: float = 20.0

    # Throughput and geometry
    MDOT_W: float = 20000.0 / 86400.0  # kg/s
    D: float = 1.2
    L: float = 3.2
    PHI: float = 0.14
    DTUBE: float = 0.168
    N_TUBES: int = 6
    R_DUCT: float = 0.4
    R_STACK: float = 0.3
    RHO_BULK: float = 450.0

    # Heat-transfer model
    U0: float = 18.97
    K_U: float = 20.09
    N_U: float = 0.65

    # Material/flue-gas properties
    CPG: float = 1.05
    CS: float = 1.70
    CW: float = 4.1844
    LAMBDA: float = 2257.9
    RHO_G_REF: float = 0.78
    T_G_REF_K: float = 450.0

    # Equipment limits
    VG_MIN: float = 3.0
    VG_MAX: float = 12.0
    TG_MIN: float = 100.0
    TG_SAFE_MAX: float = 2000.0
    TM_MAX: float = 250.0
    DELTA_T_MIN: float = 0.0

    # Experimental model validity range
    OMEGA_MODEL_MIN: float = 0.20
    OMEGA_MODEL_MAX: float = 0.98

    # Furnace steady proxy coefficients
    TAVG_A: float = -13.109632
    TAVG_B: float = 1294.871365
    TMIN_A: float = -13.422320
    TMIN_B: float = 1330.692379
    TMAX_A: float = -16.072025
    TMAX_B: float = 1589.019616
    SIGMA_A: float = -0.189997
    SIGMA_B: float = 18.331239

    # Steady burn hard bounds for legacy strict optimizer mode
    TMIN_BURN_MIN: float = 850.0
    TAVG_BURN_MIN: float = 850.0
    TAVG_BURN_MAX: float = 1100.0
    TMAX_BURN_MAX: float = 1100.0

    # Steady reference moisture band
    OMEGA_STEADY_MIN: float = 0.3042675804697915
    OMEGA_STEADY_MAX: float = 0.3393469511577442

    # Legacy static SLSQP options
    MAXITER: int = 120
    STAGE_TOL: float = 1e-6
    FEAS_TOL: float = 1e-2
    BIG_SLACK: float = 1e6

    def __post_init__(self) -> None:
        self.A_D = math.pi * self.R_DUCT ** 2
        self.A_S = math.pi * self.R_STACK ** 2
        self.A = math.pi * self.D * self.L + self.N_TUBES * math.pi * self.DTUBE * self.L
        self.M_H = self.RHO_BULK * self.PHI * (math.pi * self.D ** 2 / 4.0) * self.L
        self.TAU_R_SEC = self.M_H / self.MDOT_W
        self.TAU_R_MIN = self.TAU_R_SEC / 60.0


__all__ = [
    "Config",
    "EquivalentProperties",
    "ResourceBoundary",
    "DynamicTargetBand",
    "OptimizerRequest",
    "OptimizerResult",
]
