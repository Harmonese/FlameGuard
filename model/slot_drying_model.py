from __future__ import annotations

"""Isolated slot drying model used by the dynamic inverse optimizer.

This module intentionally does *not* advect material or mix in new feed.  It
answers a local question: if the material parcel currently represented by one
preheater cell keeps seeing a constant inlet gas condition (Tg, vg) for its
remaining residence time, what final moisture and solid temperature are
predicted by the same low-order thermal/drying laws used in the forward proxy?
"""

from dataclasses import dataclass
import math
import numpy as np

from model.control_types import PreheaterCellState
from model.model_types import Config, EquivalentProperties
from model.thermal_core import ceq_from_props, rho_g, power_kW, mdot_preheater


@dataclass(frozen=True)
class IsolatedSlotRolloutResult:
    omega_end: float
    T_solid_end_C: float
    Qsup_avg_kW: float
    Qsup_total_kJ: float
    Qevap_total_kJ: float
    sensible_total_kJ: float
    power_kW: float
    mdot_preheater_kgps: float
    steps: int


def _kinetic_limit(
    *,
    omega: float,
    T_solid_C: float,
    props: EquivalentProperties,
    dt_s: float,
    cfg: Config,
    omega_min: float,
    min_tau20_min: float,
) -> float:
    if omega <= omega_min:
        return 0.0
    tau20_min = props.tref_min + props.slope_min_per_c * (T_solid_C - cfg.T_REF)
    tau20_s = max(min_tau20_min, tau20_min) * 60.0
    return max(0.0, (omega - omega_min) * dt_s / max(tau20_s, 1e-9))


def simulate_isolated_slot(
    cell: PreheaterCellState,
    *,
    Tg_C: float,
    vg_mps: float,
    tau_remaining_s: float | None = None,
    dt_s: float = 10.0,
    cfg: Config | None = None,
    omega_min: float = 0.20,
    omega_max: float = 0.98,
    T_amb_C: float = 20.0,
    T_evap_C: float = 100.0,
    T_solid_max_C: float = 250.0,
    heat_transfer_factor: float = 1.0,
    min_tau20_min: float = 1.0,
) -> IsolatedSlotRolloutResult:
    """Roll out one material parcel without axial advection or fresh-feed mixing.

    The heat-transfer area and hold-up mass are both taken from the full
    preheater geometry.  Their ratio A/M_H is therefore identical to the
    distributed model's per-cell ratio (A/n)/(M_H/n).  This keeps the local
    parcel heating/drying rate consistent with the forward proxy while avoiding
    the erroneous feed mixing that occurs if a one-cell full preheater model is
    stepped directly.
    """

    cfg = cfg or Config()
    horizon = max(0.0, float(cell.residence_left_s if tau_remaining_s is None else tau_remaining_s))
    dt_s = max(float(dt_s), 1e-9)
    steps = max(1, int(math.ceil(horizon / dt_s))) if horizon > 0.0 else 1

    props = EquivalentProperties(
        omega0=float(cell.omega0),
        tref_min=float(cell.tref_min),
        slope_min_per_c=float(cell.slope_min_per_c),
    )
    omega = float(np.clip(cell.omega, omega_min, omega_max))
    T_solid = float(np.clip(cell.T_solid_C, T_amb_C, T_solid_max_C))

    U = cfg.U0 + cfg.K_U * (max(float(vg_mps), 0.0) ** cfg.N_U)
    area = max(cfg.A, 1e-9)
    mass = max(cfg.M_H, 1e-9)
    ceq = ceq_from_props(props, cfg)
    cap_kJ_per_K = max(mass * ceq, 1e-9)

    total_E_kJ = 0.0
    evap_E_kJ = 0.0
    sensible_E_kJ = 0.0

    elapsed = 0.0
    for k in range(steps):
        dt_step = min(dt_s, max(horizon - elapsed, 0.0)) if horizon > 0.0 else dt_s
        if dt_step <= 0.0:
            break
        elapsed += dt_step

        deltaT = max(float(Tg_C) - T_solid, 0.0)
        Q_kW = heat_transfer_factor * U * area * deltaT / 1000.0
        E_kJ = max(Q_kW * dt_step, 0.0)
        total_E_kJ += E_kJ

        if E_kJ <= 0.0:
            continue

        # Sensible heating to the evaporation threshold first.
        if T_solid < T_evap_C:
            need = (T_evap_C - T_solid) * cap_kJ_per_K
            used = min(E_kJ, need)
            T_solid += used / cap_kJ_per_K
            sensible_E_kJ += used
            E_kJ -= used

        # Above the threshold, keep the same low-order split used by the forward
        # proxy: most high-temperature heat is available for evaporation, with a
        # kinetic cap from tau20 and a residual sensible heating term.
        if E_kJ > 0.0 and omega > omega_min:
            E_evap_available = 0.85 * E_kJ if T_solid >= T_evap_C else 0.0
            dm_evap = E_evap_available / max(cfg.LAMBDA, 1e-9)
            d_omega_energy = dm_evap * max(1.0 - omega, 0.05) / mass
            d_omega_kin = _kinetic_limit(
                omega=omega,
                T_solid_C=T_solid,
                props=props,
                dt_s=dt_step,
                cfg=cfg,
                omega_min=omega_min,
                min_tau20_min=min_tau20_min,
            )
            d_omega = min(d_omega_energy, d_omega_kin, max(omega - omega_min, 0.0))
            d_omega = max(d_omega, 0.0)
            omega -= d_omega

            # Account diagnostics for the actual evaporated moisture.  The
            # wet-basis conversion mirrors the forward proxy's d_omega_energy.
            evap_used = d_omega * mass * cfg.LAMBDA / max(1.0 - (omega + d_omega), 0.05)
            evap_used = min(max(evap_used, 0.0), E_evap_available)
            evap_E_kJ += evap_used

            E_left = max(E_kJ - evap_used, 0.0)
            T_solid += E_left / cap_kJ_per_K
            sensible_E_kJ += E_left
        elif E_kJ > 0.0:
            T_solid += E_kJ / cap_kJ_per_K
            sensible_E_kJ += E_kJ

        T_solid = float(np.clip(T_solid, T_amb_C, T_solid_max_C))
        omega = float(np.clip(omega, omega_min, omega_max))

    duration = max(elapsed, dt_s if horizon <= 0.0 else horizon, 1e-9)
    return IsolatedSlotRolloutResult(
        omega_end=float(omega),
        T_solid_end_C=float(T_solid),
        Qsup_avg_kW=float(total_E_kJ / duration),
        Qsup_total_kJ=float(total_E_kJ),
        Qevap_total_kJ=float(evap_E_kJ),
        sensible_total_kJ=float(sensible_E_kJ),
        power_kW=float(power_kW(float(Tg_C), float(vg_mps), cfg)),
        mdot_preheater_kgps=float(mdot_preheater(float(Tg_C), float(vg_mps), cfg)),
        steps=int(steps),
    )
