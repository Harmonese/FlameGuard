from __future__ import annotations

from dataclasses import dataclass, asdict
import math
import numpy as np
from scipy.optimize import minimize

from model.control_types import PreheaterCellState, DynamicSlotLookupResult
from model.model_types import Config, ResourceBoundary
from model.thermal_core import mdot_preheater, mdot_stack_cap, power_kW
from model.slot_drying_model import simulate_isolated_slot


@dataclass(frozen=True)
class DynamicSlotRequest:
    cell: PreheaterCellState
    omega_target: float
    resource: ResourceBoundary
    time_s: float = 0.0
    slot_id: str = "dynamic_slot"


@dataclass
class DynamicSlotSLSQPConfig:
    maxiter: int = 80
    q_omega: float = 2.0e5
    q_power: float = 1.0e-2
    q_action: float = 1.0e-3
    Tg_nom_C: float = 500.0
    vg_nom_mps: float = 7.0
    rollout_dt_s: float = 10.0
    feasibility_tol: float = 0.015


class DynamicSlotSLSQPOptimizer:
    """Local inverse optimizer for one slot and its remaining time.

    It uses the same forward proxy for evaluation, but keeps the decision vector
    simple: a constant Tg/vg pair over the slot's remaining residence time.
    """

    def __init__(self, cfg: DynamicSlotSLSQPConfig | None = None, opt_cfg: Config | None = None):
        self.cfg = cfg or DynamicSlotSLSQPConfig()
        self.opt_cfg = opt_cfg or Config()

    def _simulate(self, req: DynamicSlotRequest, Tg: float, vg: float) -> tuple[float, float, float, float]:
        """Simulate one material parcel over its remaining residence time.

        This intentionally does not call PreheaterForwardModel.step(), because
        the full forward step includes axial advection and fresh-feed mixing.
        The inverse optimizer needs a local parcel/slot model: current slot
        state + constant Tg/vg + remaining time -> final moisture.
        """
        horizon = max(req.cell.residence_left_s, self.cfg.rollout_dt_s)
        rollout = simulate_isolated_slot(
            req.cell,
            Tg_C=float(Tg),
            vg_mps=float(vg),
            tau_remaining_s=horizon,
            dt_s=self.cfg.rollout_dt_s,
            cfg=self.opt_cfg,
        )
        qevap_avg = rollout.Qevap_total_kJ / max(horizon, 1e-9)
        return rollout.omega_end, rollout.T_solid_end_C, rollout.Qsup_avg_kW, qevap_avg

    def optimize(self, req: DynamicSlotRequest) -> DynamicSlotLookupResult:
        r = req.resource
        Tg_hi = max(self.opt_cfg.TG_MIN, min(r.T_stack_cap_C, self.opt_cfg.TG_SAFE_MAX))
        bounds = [(self.opt_cfg.TG_MIN, Tg_hi), (self.opt_cfg.VG_MIN, self.opt_cfg.VG_MAX)]
        mdot_cap = mdot_stack_cap(r, self.opt_cfg)

        target = float(np.clip(req.omega_target, self.opt_cfg.OMEGA_MODEL_MIN, min(req.cell.omega, self.opt_cfg.OMEGA_MODEL_MAX)))

        def obj(z):
            Tg, vg = float(z[0]), float(z[1])
            omega_end, _, _, _ = self._simulate(req, Tg, vg)
            e = omega_end - target
            p = power_kW(Tg, vg, self.opt_cfg)
            act = ((Tg - self.cfg.Tg_nom_C) / 500.0) ** 2 + ((vg - self.cfg.vg_nom_mps) / 6.0) ** 2
            return self.cfg.q_omega * e * e + self.cfg.q_power * p + self.cfg.q_action * act

        cons = [{"type": "ineq", "fun": lambda z: mdot_cap - mdot_preheater(float(z[0]), float(z[1]), self.opt_cfg)}]
        seeds = [
            [min(max(self.cfg.Tg_nom_C, bounds[0][0]), bounds[0][1]), self.cfg.vg_nom_mps],
            [bounds[0][1], 11.0],
            [max(bounds[0][0], 250.0), 5.0],
            [bounds[0][1], 3.5],
        ]
        best = None
        best_fun = float('inf')
        for x0 in seeds:
            try:
                res = minimize(obj, x0=np.array(x0, dtype=float), method='SLSQP', bounds=bounds, constraints=cons,
                               options={'maxiter': self.cfg.maxiter, 'ftol': 1e-8, 'disp': False})
                val = obj(res.x)
                if val < best_fun:
                    best_fun = val
                    best = res
            except Exception:
                continue
        if best is None:
            Tg, vg = bounds[0][0], bounds[1][0]
            success = False
        else:
            Tg, vg = float(best.x[0]), float(best.x[1])
            success = bool(best.success)
        omega_end, _, qsup_avg, qevap_avg = self._simulate(req, Tg, vg)
        p = power_kW(Tg, vg, self.opt_cfg)
        feasible = bool(success and abs(omega_end - target) <= self.cfg.feasibility_tol)
        return DynamicSlotLookupResult(
            time_s=req.time_s,
            Tg_star_C=float(Tg),
            vg_star_mps=float(vg),
            omega_target=float(target),
            omega_reachable=float(omega_end),
            feasible=feasible,
            power_kW=float(p),
            Qsup_kW=float(qsup_avg),
            Qreq_kW=float(qevap_avg),
            mdot_stack_cap_kgps=float(mdot_cap),
            mdot_preheater_kgps=float(mdot_preheater(Tg, vg, self.opt_cfg)),
            source='dynamic_slot_slsqp',
        )


def result_to_row(req: DynamicSlotRequest, res: DynamicSlotLookupResult) -> dict:
    row = asdict(res)
    row.update({
        'omega0': req.cell.omega0,
        'tref_min': req.cell.tref_min,
        'slope_min_per_c': req.cell.slope_min_per_c,
        'omega_current': req.cell.omega,
        'T_solid_current_C': req.cell.T_solid_C,
        'tau_remaining_s': req.cell.residence_left_s,
        'T_stack_cap_C': req.resource.T_stack_cap_C,
        'v_stack_cap_mps': req.resource.v_stack_cap_mps,
        'slot_id': req.slot_id,
    })
    return row
