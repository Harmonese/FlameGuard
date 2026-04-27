from __future__ import annotations

from dataclasses import dataclass
import math
import numpy as np

from model.control_types import FeedObservation, FurnaceObservation, MPCDecision, PreheaterCellState
from model.preheater_forward_model import PreheaterForwardModel
from model.furnace_dynamic import FurnaceDyn
from model.model_types import ResourceBoundary
from model.thermal_core import power_kW
from controller.optimizer.dynamic_slot_lookup import DynamicSlotLookup


@dataclass
class MPCConfig:
    dt_mpc_s: float = 20.0
    horizon_s: float = 600.0
    T_set_C: float = 872.99
    omega_ref: float = 0.3218
    omega_min: float = 0.20
    omega_max: float = 0.60
    target_gain_per_C: float = 5.0e-4
    candidate_offsets: tuple[float, ...] = (-0.06, -0.03, 0.0, 0.03, 0.06)
    representative_remaining_s: float = 360.0
    q_Tavg: float = 1.0
    q_omega: float = 250.0
    r_energy: float = 1.0e-4
    r_move_Tg: float = 1.0e-4
    r_move_vg: float = 0.05
    q_safety: float = 50.0
    safe_low_C: float = 845.0
    safe_high_C: float = 1105.0
    # Direct full-furnace candidates are evaluated by the same 20-cell rollout
    # as lookup candidates. They prevent sparse/single-slot lookup from trapping
    # the MPC in low-flow solutions that are locally feasible but too weak for
    # the full furnace because of gas cooling along the preheater.
    direct_control_candidates: tuple[tuple[float, float], ...] = (
        (800.0, 12.0),
        (930.0, 12.0),
        (930.0, 8.0),
    )
    infeasible_lookup_penalty: float = 1.0e5


class MPCController:
    """Lookup-assisted MPC controller.

    For each target-moisture candidate, the dynamic slot lookup gives a Tg/vg
    baseline. Additional direct full-furnace candidates are also evaluated. All
    candidates are scored by rolling out the distributed preheater plus furnace
    model; only the first selected command is executed.
    """

    def __init__(self, lookup: DynamicSlotLookup, cfg: MPCConfig | None = None):
        self.lookup = lookup
        self.cfg = cfg or MPCConfig()
        self.prev_Tg_ref_C = 300.0
        self.prev_vg_ref_mps = 6.0
        self.last_decision: MPCDecision | None = None

    def _target_center(self, obs: FurnaceObservation) -> float:
        error = self.cfg.T_set_C - obs.T_avg_C
        # Cold furnace: error > 0 => request drier future outlet moisture.
        center = self.cfg.omega_ref - self.cfg.target_gain_per_C * error
        return float(np.clip(center, self.cfg.omega_min, self.cfg.omega_max))

    def _representative_cell(self, preheater: PreheaterForwardModel) -> PreheaterCellState:
        return preheater.representative_cell(self.cfg.representative_remaining_s)

    def _rollout_cost(self, *, preheater: PreheaterForwardModel, furnace: FurnaceDyn, feed: FeedObservation,
                      Tg: float, vg: float, omega_target: float, disturbance=None) -> tuple[float, float, float]:
        p = preheater.clone()
        f = furnace.clone()
        steps = max(1, int(math.ceil(self.cfg.horizon_s / self.cfg.dt_mpc_s)))
        cost = 0.0
        pred_T = self.cfg.T_set_C
        pred_w = preheater.state().omega_out
        for k in range(steps):
            ff = FeedObservation(time_s=feed.time_s + (k + 1) * self.cfg.dt_mpc_s, composition=feed.composition, normalize=feed.normalize)
            st = p.step(ff, Tg, vg, self.cfg.dt_mpc_s)
            pred_w = st.omega_out
            pred_T, _, _ = f.step(pred_w, dt_s=self.cfg.dt_mpc_s, disturbance=disturbance)
            eT = pred_T - self.cfg.T_set_C
            ew = pred_w - omega_target
            safety = max(self.cfg.safe_low_C - pred_T, 0.0) ** 2 + max(pred_T - self.cfg.safe_high_C, 0.0) ** 2
            cost += self.cfg.q_Tavg * eT * eT + self.cfg.q_omega * ew * ew + self.cfg.q_safety * safety
        move = self.cfg.r_move_Tg * (Tg - self.prev_Tg_ref_C) ** 2 + self.cfg.r_move_vg * (vg - self.prev_vg_ref_mps) ** 2
        try:
            energy = power_kW(Tg, vg, preheater.opt_cfg)
        except Exception:
            energy = 0.0
        cost += move + self.cfg.r_energy * energy * steps
        return float(cost), float(pred_T), float(pred_w)

    def _append_candidate(self, candidates, *, preheater, furnace, feed, Tg, vg, omega_target, disturbance,
                          feasible=True, omega_reachable=None, source='') -> None:
        Tg = float(np.clip(Tg, 100.0, 2000.0))
        vg = float(np.clip(vg, 3.0, 12.0))
        cost, pred_T, pred_w = self._rollout_cost(
            preheater=preheater,
            furnace=furnace,
            feed=feed,
            Tg=Tg,
            vg=vg,
            omega_target=omega_target,
            disturbance=disturbance,
        )
        if not feasible:
            cost += self.cfg.infeasible_lookup_penalty
        if omega_reachable is None:
            omega_reachable = pred_w
        candidates.append((cost, omega_target, Tg, vg, pred_T, pred_w, feasible, float(omega_reachable), source))

    def step(self, *, obs: FurnaceObservation, preheater: PreheaterForwardModel, furnace: FurnaceDyn,
             feed: FeedObservation, resource: ResourceBoundary, disturbance=None) -> MPCDecision:
        center = self._target_center(obs)
        cell = self._representative_cell(preheater)
        candidates = []

        # 1) Lookup inverse candidates: target moisture -> Tg/vg baseline.
        for off in self.cfg.candidate_offsets:
            omega_target = float(np.clip(center + off, self.cfg.omega_min, min(self.cfg.omega_max, cell.omega)))
            inv = self.lookup.query(cell=cell, omega_target=omega_target, resource=resource, time_s=obs.time_s)
            Tg = float(np.clip(inv.Tg_star_C, 100.0, resource.T_stack_cap_C))
            vg = float(np.clip(inv.vg_star_mps, 3.0, 12.0))
            self._append_candidate(
                candidates,
                preheater=preheater,
                furnace=furnace,
                feed=feed,
                Tg=Tg,
                vg=vg,
                omega_target=omega_target,
                disturbance=disturbance,
                feasible=bool(inv.feasible),
                omega_reachable=inv.omega_reachable,
                source=inv.source,
            )

        # 2) Direct full-furnace candidates: not produced by lookup, only scored
        # by the full forward model. Include previous command and high-flow cases.
        direct_targets = {
            float(np.clip(center, self.cfg.omega_min, min(self.cfg.omega_max, cell.omega))),
            float(np.clip(self.cfg.omega_ref, self.cfg.omega_min, min(self.cfg.omega_max, cell.omega))),
        }
        direct_controls = [(self.prev_Tg_ref_C, self.prev_vg_ref_mps)]
        direct_controls.extend(self.cfg.direct_control_candidates)
        direct_controls.append((resource.T_stack_cap_C, 12.0))
        seen = set()
        for Tg_raw, vg_raw in direct_controls:
            Tg = float(np.clip(Tg_raw, 100.0, resource.T_stack_cap_C))
            vg = float(np.clip(vg_raw, 3.0, 12.0))
            key = (round(Tg, 6), round(vg, 6))
            if key in seen:
                continue
            seen.add(key)
            for omega_target in direct_targets:
                self._append_candidate(
                    candidates,
                    preheater=preheater,
                    furnace=furnace,
                    feed=feed,
                    Tg=Tg,
                    vg=vg,
                    omega_target=omega_target,
                    disturbance=disturbance,
                    feasible=True,
                    omega_reachable=None,
                    source='direct_full_rollout',
                )

        cost, omega_target, Tg, vg, pred_T, pred_w, feasible, omega_reach, source = min(candidates, key=lambda x: x[0])
        self.prev_Tg_ref_C = Tg
        self.prev_vg_ref_mps = vg
        self.last_decision = MPCDecision(
            time_s=obs.time_s,
            Tg_ref_C=Tg,
            vg_ref_mps=vg,
            omega_target=omega_target,
            omega_reachable=omega_reach,
            predicted_Tavg_C=pred_T,
            predicted_omega_out=pred_w,
            cost=cost,
            feasible=feasible,
            source='lookup_mpc',
            note=f'inverse={source}; center={center:.4f}',
        )
        return self.last_decision
