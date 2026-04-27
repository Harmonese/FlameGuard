from __future__ import annotations

"""Compatibility facade for the legacy static optimizer.

Historically, shared physical types/functions and the static SLSQP solver lived
in this single module.  Step-3 refactoring moved canonical model types and
thermal equations into ``models.model_types`` and ``models.thermal_core``.

New model/control/runtime code should import from ``models.*``.  This module is
kept only so old scripts and legacy lookup-table utilities still run.
"""

from model.model_types import (  # noqa: F401
    Config,
    DynamicTargetBand,
    EquivalentProperties,
    OptimizerRequest,
    OptimizerResult,
    ResourceBoundary,
)
from model.thermal_core import (  # noqa: F401
    T_avg_proxy,
    T_max_proxy,
    T_min_proxy,
    ceq_from_props,
    evap_water_per_kg_wet,
    mdot_preheater,
    mdot_stack_cap,
    power_kW,
    q_req_kW,
    q_sup_kW,
    rho_g,
    sigma_proxy,
    steady_band_violation_percent,
    strict_burn_feasible,
    tau20,
    tau_target,
    validate_dynamic_target_band,
    validate_equivalent_properties,
    validate_resource_boundary,
)
from controller.optimizer.static_optimizer import build_lookup_table, optimize_static_slot, result_to_row  # noqa: F401

__all__ = [
    "Config",
    "EquivalentProperties",
    "ResourceBoundary",
    "DynamicTargetBand",
    "OptimizerRequest",
    "OptimizerResult",
    "optimize_static_slot",
    "build_lookup_table",
    "result_to_row",
    "strict_burn_feasible",
    "steady_band_violation_percent",
    "T_avg_proxy",
    "T_min_proxy",
    "T_max_proxy",
    "sigma_proxy",
    "ceq_from_props",
    "rho_g",
    "q_sup_kW",
    "q_req_kW",
    "tau20",
    "tau_target",
    "power_kW",
    "mdot_stack_cap",
    "mdot_preheater",
]
