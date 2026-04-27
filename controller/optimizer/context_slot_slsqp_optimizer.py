from __future__ import annotations

from dataclasses import dataclass, asdict
import math
import numpy as np
from scipy.optimize import minimize

from model.control_types import DynamicSlotLookupResult, FeedObservation, PreheaterCellState, PreheaterState
from model.preheater_forward_model import PreheaterForwardModel, PreheaterForwardConfig
from model.model_types import Config, ResourceBoundary
from model.thermal_core import mdot_preheater, mdot_stack_cap, power_kW


@dataclass
class ContextSlotRequest:
    """Context-aware inverse request.

    Unlike DynamicSlotRequest, this request carries the complete distributed
    preheater state through a PreheaterForwardModel.  The inverse optimizer
    evaluates candidate inlet Tg/vg by rolling out the full 20-cell model, so gas
    cooling by upstream material is included automatically.
    """

    preheater: PreheaterForwardModel
    feed: FeedObservation
    omega_target: float
    resource: ResourceBoundary
    horizon_s: float = 600.0
    time_s: float = 0.0
    slot_id: str = "context_slot"
    representative_remaining_s: float = 360.0


@dataclass
class ContextSlotSLSQPConfig:
    maxiter: int = 80
    rollout_dt_s: float = 20.0
    q_omega: float = 2.0e5
    q_power: float = 1.0e-2
    q_action: float = 2.0e-3
    Tg_nom_C: float = 800.0
    vg_nom_mps: float = 12.0
    feasibility_tol: float = 0.02


@dataclass(frozen=True)
class ContextFeatureSummary:
    slot_index: int
    residence_left_s: float
    omega_current: float
    T_solid_current_C: float
    omega0: float
    tref_min: float
    slope_min_per_c: float
    upstream_omega_mean: float
    upstream_T_solid_mean_C: float
    upstream_load_proxy: float
    local_Tg_nominal_C: float


def summarize_context(state: PreheaterState, *, representative_remaining_s: float = 360.0) -> ContextFeatureSummary:
    cell = min(state.cells, key=lambda c: abs(c.residence_left_s - representative_remaining_s))
    upstream = [c for c in state.cells if c.index < cell.index]
    if upstream:
        up_omega = float(np.mean([c.omega for c in upstream]))
        up_T = float(np.mean([c.T_solid_C for c in upstream]))
    else:
        up_omega = float(cell.omega)
        up_T = float(cell.T_solid_C)
    # A compact proxy for how much heat upstream cells will consume before the
    # target cell sees the gas.  Higher moisture and lower solid temperature both
    # increase the proxy.
    load = 0.0
    for c in upstream:
        load += max(c.omega - 0.20, 0.0) + 0.004 * max(100.0 - c.T_solid_C, 0.0)
    tg_prof = list(state.Tg_profile_C or [])
    local_Tg = float(tg_prof[min(cell.index, len(tg_prof) - 1)]) if tg_prof else 800.0
    return ContextFeatureSummary(
        slot_index=int(cell.index),
        residence_left_s=float(cell.residence_left_s),
        omega_current=float(cell.omega),
        T_solid_current_C=float(cell.T_solid_C),
        omega0=float(cell.omega0),
        tref_min=float(cell.tref_min),
        slope_min_per_c=float(cell.slope_min_per_c),
        upstream_omega_mean=up_omega,
        upstream_T_solid_mean_C=up_T,
        upstream_load_proxy=float(load),
        local_Tg_nominal_C=local_Tg,
    )


class ContextSlotSLSQPOptimizer:
    """Full-context inverse optimizer used for table generation and fallback.

    It answers: for the *current whole preheater inventory*, what constant inlet
    Tg/vg would make the outlet moisture after horizon_s close to omega_target?
    This keeps inverse evaluation physically consistent with the forward model,
    including gas cooling along the 20 cells.
    """

    def __init__(self, cfg: ContextSlotSLSQPConfig | None = None, opt_cfg: Config | None = None):
        self.cfg = cfg or ContextSlotSLSQPConfig()
        self.opt_cfg = opt_cfg or Config()

    def _simulate(self, req: ContextSlotRequest, Tg: float, vg: float) -> tuple[float, float, float]:
        model = req.preheater.clone()
        steps = max(1, int(math.ceil(req.horizon_s / max(self.cfg.rollout_dt_s, 1e-9))))
        omega_out = model.state().omega_out
        q_power = power_kW(float(Tg), float(vg), model.opt_cfg)
        t0 = req.feed.time_s
        for k in range(steps):
            ff = FeedObservation(
                time_s=t0 + (k + 1) * self.cfg.rollout_dt_s,
                composition=req.feed.composition,
                normalize=req.feed.normalize,
            )
            st = model.step(ff, float(Tg), float(vg), self.cfg.rollout_dt_s)
            omega_out = st.omega_out
        return float(omega_out), float(st.T_solid_out_C), float(q_power)

    def optimize(self, req: ContextSlotRequest) -> DynamicSlotLookupResult:
        mdot_cap = mdot_stack_cap(req.resource, self.opt_cfg)
        Tg_hi = max(self.opt_cfg.TG_MIN, min(req.resource.T_stack_cap_C, self.opt_cfg.TG_SAFE_MAX))
        vg_hi = max(self.opt_cfg.VG_MIN, min(req.resource.v_stack_cap_mps, self.opt_cfg.VG_MAX))
        bounds = [(self.opt_cfg.TG_MIN, Tg_hi), (self.opt_cfg.VG_MIN, vg_hi)]
        target = float(np.clip(req.omega_target, self.opt_cfg.OMEGA_MODEL_MIN, self.opt_cfg.OMEGA_MODEL_MAX))

        def obj(z):
            Tg, vg = float(z[0]), float(z[1])
            omega_end, _, _ = self._simulate(req, Tg, vg)
            e = omega_end - target
            p = power_kW(Tg, vg, self.opt_cfg)
            act = ((Tg - self.cfg.Tg_nom_C) / 500.0) ** 2 + ((vg - self.cfg.vg_nom_mps) / 6.0) ** 2
            return self.cfg.q_omega * e * e + self.cfg.q_power * p + self.cfg.q_action * act

        cons = [{"type": "ineq", "fun": lambda z: mdot_cap - mdot_preheater(float(z[0]), float(z[1]), self.opt_cfg)}]
        seeds = [
            [min(max(self.cfg.Tg_nom_C, bounds[0][0]), bounds[0][1]), min(max(self.cfg.vg_nom_mps, bounds[1][0]), bounds[1][1])],
            [bounds[0][1], bounds[1][1]],
            [800.0, bounds[1][1]],
            [bounds[0][1], max(bounds[1][0], 6.0)],
        ]
        best = None
        best_fun = float('inf')
        for x0 in seeds:
            try:
                x0 = np.asarray([min(max(x0[0], bounds[0][0]), bounds[0][1]), min(max(x0[1], bounds[1][0]), bounds[1][1])], dtype=float)
                res = minimize(obj, x0=x0, method='SLSQP', bounds=bounds, constraints=cons,
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
        omega_end, _, p = self._simulate(req, Tg, vg)
        feasible = bool(success and abs(omega_end - target) <= self.cfg.feasibility_tol)
        return DynamicSlotLookupResult(
            time_s=req.time_s,
            Tg_star_C=float(Tg),
            vg_star_mps=float(vg),
            omega_target=float(target),
            omega_reachable=float(omega_end),
            feasible=feasible,
            power_kW=float(p),
            Qsup_kW=0.0,
            Qreq_kW=0.0,
            mdot_stack_cap_kgps=float(mdot_cap),
            mdot_preheater_kgps=float(mdot_preheater(Tg, vg, self.opt_cfg)),
            source='context_slot_slsqp_full_forward',
        )


def result_to_context_row(req: ContextSlotRequest, res: DynamicSlotLookupResult) -> dict:
    row = asdict(res)
    feat = summarize_context(req.preheater.state(), representative_remaining_s=req.representative_remaining_s)
    row.update(asdict(feat))
    row.update({
        'omega_target': res.omega_target,
        'T_stack_cap_C': req.resource.T_stack_cap_C,
        'v_stack_cap_mps': req.resource.v_stack_cap_mps,
        'horizon_s': req.horizon_s,
        'slot_id': req.slot_id,
    })
    return row
