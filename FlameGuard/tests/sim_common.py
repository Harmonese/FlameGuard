from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from types import SimpleNamespace
from typing import Deque, List
import csv

import matplotlib.pyplot as plt
import numpy as np

from control_types import DEFAULT_PREHEATER_GAINS, ActuatorCommand, ControllerOutput, FeedObservation, FurnaceObservation, GovernorDecision
from controller.controller import PIDConfig, PIDController
from governor.governor import DynamicGovernor, DynamicGovernorConfig
from optimizer.optimizer import DynamicTargetBand, EquivalentProperties, OptimizerRequest, ResourceBoundary, optimize_static_slot
from transcriber.transcriber_a import TranscriberA, TranscriberAConfig
from transcriber.transcriber_b import TranscriberB


def furnace_outputs_from_omega(omega: float):
    return (
        -13.109632 * (100.0 * omega) + 1294.871365,
        -14.412237 * (100.0 * omega) + 1423.472316,
        -0.215310 * (100.0 * omega) + 25.332842,
    )


@dataclass
class SimConfig:
    dt_meas_s: float = 0.1
    dt_ctrl_s: float = 1.0
    dt_opt_s: float = 2.0
    total_time_s: float = 1200.0
    omega_ref: float = 0.3218
    pre_dead_s: float = 5.0
    pre_tau_s: float = 985.0
    furnace_dead_s: float = 5.0
    furnace_tau1_s: float = 0.223
    furnace_tau2_s: float = 75.412
    resource_T_stack_cap_C: float = 930.0
    resource_v_stack_cap_mps: float = 18.0
    use_lookup_table: bool = True
    lookup_k: int = 12

    @property
    def T_set_C(self) -> float:
        return furnace_outputs_from_omega(self.omega_ref)[0]


@dataclass
class History:
    t: List[float] = field(default_factory=list)
    T_avg: List[float] = field(default_factory=list)
    T_stack: List[float] = field(default_factory=list)
    v_stack: List[float] = field(default_factory=list)
    T_set: List[float] = field(default_factory=list)
    omega_est: List[float] = field(default_factory=list)
    omega_req: List[float] = field(default_factory=list)
    omega_tar: List[float] = field(default_factory=list)
    dyn_lo: List[float] = field(default_factory=list)
    dyn_hi: List[float] = field(default_factory=list)
    slot_omega0: List[float] = field(default_factory=list)
    omega_opt: List[float] = field(default_factory=list)
    omega_out: List[float] = field(default_factory=list)
    Tg_cmd: List[float] = field(default_factory=list)
    vg_cmd: List[float] = field(default_factory=list)
    heater_deficit: List[float] = field(default_factory=list)
    Tavg_pred: List[float] = field(default_factory=list)


class LookupOptimizer:
    def __init__(self, csv_path: str | Path, *, k_neighbors: int = 12):
        self.rows = []
        self.k_neighbors = max(3, k_neighbors)
        path = Path(csv_path)
        with path.open(encoding='utf-8-sig', newline='') as f:
            reader = csv.DictReader(f)
            for row in reader:
                parsed = {k: row[k] for k in row}
                for key, val in row.items():
                    if val in ('True', 'False'):
                        parsed[key] = (val == 'True')
                    else:
                        try:
                            parsed[key] = float(val)
                        except Exception:
                            parsed[key] = val
                if parsed.get('success', True):
                    self.rows.append(parsed)
        if not self.rows:
            raise RuntimeError('Lookup table is empty or has no successful rows.')
        self._omega0 = np.array([r['omega0'] for r in self.rows], dtype=float)
        self._tref = np.array([r['tref_min'] for r in self.rows], dtype=float)
        self._slope = np.array([r['slope_min_per_c'] for r in self.rows], dtype=float)
        self._otar = np.array([r['omega_tar_requested'] for r in self.rows], dtype=float)
        self._scales = np.array([
            max(self._omega0.max() - self._omega0.min(), 1e-6),
            max(self._tref.max() - self._tref.min(), 1e-6),
            max(self._slope.max() - self._slope.min(), 1e-6),
            max(self._otar.max() - self._otar.min(), 1e-6),
        ])
        self._fields_float = [
            'Tg_star_C', 'vg_star_mps', 'Tm_star_C', 'omega_opt', 'power_kW',
            'Qreq_kW', 'Qsup_kW', 'mdot_stack_cap_kgps', 'mdot_preheater_kgps',
            'omega_tar_projected'
        ]

    def query(self, req: OptimizerRequest):
        q = np.array([
            req.props.omega0,
            req.props.tref_min,
            req.props.slope_min_per_c,
            req.omega_tar,
        ])
        data = np.vstack([self._omega0, self._tref, self._slope, self._otar]).T
        d = (data - q) / self._scales
        dsq = np.sum(d * d, axis=1)
        idx = np.argsort(dsq)[: self.k_neighbors]
        if dsq[idx[0]] < 1e-14:
            row = self.rows[int(idx[0])]
            return SimpleNamespace(**row)
        weights = 1.0 / np.maximum(np.sqrt(dsq[idx]), 1e-9)
        weights = weights / np.sum(weights)
        out = {}
        base = self.rows[int(idx[0])]
        for field in base.keys():
            if field in self._fields_float:
                out[field] = float(sum(weights[j] * self.rows[int(i)][field] for j, i in enumerate(idx)))
            else:
                out[field] = base[field]
        out['success'] = True
        return SimpleNamespace(**out)


class PreheaterDyn:
    def __init__(self, cfg: SimConfig, w_init: float):
        self.cfg = cfg
        self.gains = DEFAULT_PREHEATER_GAINS
        self.delay_steps = max(1, round(cfg.pre_dead_s / cfg.dt_meas_s))
        self.cmd_queue: Deque[tuple[float, float, float, float]] = deque(maxlen=self.delay_steps)
        self.w_out = w_init
        self.shape_state = 0.0

    def initialize(self, Tg0: float, vg0: float, omega0_slot0: float, omega_opt0: float):
        for _ in range(self.delay_steps):
            self.cmd_queue.append((Tg0, vg0, omega0_slot0, omega_opt0))
        self.shape_state = 0.0

    def step(self, cmd: ActuatorCommand, omega0_slot: float, omega_opt_target: float) -> float:
        self.cmd_queue.append((cmd.Tg_cmd_C, cmd.vg_cmd_mps, omega0_slot, omega_opt_target))
        Tg_d, vg_d, omega0_d, omega_opt_d = self.cmd_queue[0]
        Tg_prev, vg_prev, omega0_prev, _ = self.cmd_queue[-2] if len(self.cmd_queue) >= 2 else self.cmd_queue[0]
        g = self.gains
        transient_target = (
            g.K_wT * (Tg_d - Tg_prev) +
            g.K_wv * (vg_d - vg_prev) +
            g.K_wx * (omega0_d - omega0_prev)
        )
        alpha_shape = np.exp(-self.cfg.dt_meas_s / max(0.35 * self.cfg.pre_tau_s, 1e-6))
        self.shape_state = alpha_shape * self.shape_state + (1.0 - alpha_shape) * transient_target
        w_target = np.clip(omega_opt_d + self.shape_state, 0.20, 0.95)
        alpha = np.exp(-self.cfg.dt_meas_s / self.cfg.pre_tau_s)
        self.w_out = alpha * self.w_out + (1.0 - alpha) * w_target
        return self.w_out


class FurnaceDyn:
    def __init__(self, cfg: SimConfig):
        self.cfg = cfg
        self.delay_steps = max(1, round(cfg.furnace_dead_s / cfg.dt_meas_s))
        self.queue: Deque[float] = deque([cfg.omega_ref] * self.delay_steps, maxlen=self.delay_steps)
        t_ref, ts_ref, vs_ref = furnace_outputs_from_omega(cfg.omega_ref)
        self.refs = {'T_avg': t_ref, 'T_stack': ts_ref, 'v_stack': vs_ref}
        self.states = {'T_avg': (0.0, 0.0), 'T_stack': (0.0, 0.0), 'v_stack': (0.0, 0.0)}
        self.gains = {'T_avg': -13.109632, 'T_stack': -14.412237, 'v_stack': -0.215310}

    def step(self, w_in: float, disturbance=None):
        self.queue.append(w_in)
        w_d = self.queue[0]
        dt = self.cfg.dt_meas_s
        outputs = {}
        delta_w_pct = 100.0 * (w_d - self.cfg.omega_ref)
        for key in ['T_avg', 'T_stack', 'v_stack']:
            x1, x2 = self.states[key]
            u = self.gains[key] * delta_w_pct
            a1 = np.exp(-dt / self.cfg.furnace_tau1_s)
            x1 = a1 * x1 + (1.0 - a1) * u
            a2 = np.exp(-dt / self.cfg.furnace_tau2_s)
            x2 = a2 * x2 + (1.0 - a2) * x1
            self.states[key] = (x1, x2)
            outputs[key] = self.refs[key] + x2
        if disturbance is None:
            d_avg, d_stack, d_v = 0.0, 0.0, 0.0
        elif isinstance(disturbance, (tuple, list)) and len(disturbance) == 3:
            d_avg, d_stack, d_v = float(disturbance[0]), float(disturbance[1]), float(disturbance[2])
        else:
            d_avg, d_stack, d_v = float(disturbance), 0.0, 0.0
        outputs['T_avg'] += d_avg
        outputs['T_stack'] += d_stack
        outputs['v_stack'] += d_v
        return outputs['T_avg'], outputs['T_stack'], outputs['v_stack']


def _initial_result(initial_comp, cfg: SimConfig, lookup: LookupOptimizer | None):
    from cleanser.cleanser import composition_to_equivalent_properties
    eq = composition_to_equivalent_properties(initial_comp).equivalent
    req = OptimizerRequest(
        props=EquivalentProperties(eq.omega0, eq.tref_min, eq.slope_min_per_c),
        omega_tar=cfg.omega_ref,
        resource=ResourceBoundary(cfg.resource_T_stack_cap_C, cfg.resource_v_stack_cap_mps),
        dyn_band=DynamicTargetBand(omega_min=0.30, omega_max=0.35),
        slot_id='initial',
        burn_policy='advisory',
    )
    if lookup is not None:
        return lookup.query(req)
    return optimize_static_slot(req)


def run_case(name: str, composition_schedule, disturbance_schedule=None, cfg: SimConfig | None = None):
    cfg = cfg or SimConfig()
    controller = PIDController(PIDConfig(dt_controller_s=cfg.dt_ctrl_s, T_set_C=cfg.T_set_C))
    governor = DynamicGovernor(DynamicGovernorConfig(dt_governor_s=cfg.dt_ctrl_s))
    ta = TranscriberA(
        TranscriberAConfig(
            dt_update_s=cfg.dt_opt_s,
            resource_T_stack_default_C=cfg.resource_T_stack_cap_C,
            resource_v_stack_default_mps=cfg.resource_v_stack_cap_mps,
            burn_policy='advisory',
        )
    )
    tb = TranscriberB()
    hist = History()

    lookup = None
    if cfg.use_lookup_table:
        lookup = LookupOptimizer(Path(__file__).resolve().parent.parent / 'optimizer' / 'optimizer_lookup_table.csv', k_neighbors=cfg.lookup_k)

    initial_comp = composition_schedule(0.0)
    ta.initialize(initial_comp)
    init_res = _initial_result(initial_comp, cfg, lookup)
    _, init_cmd = tb.translate(init_res, time_s=0.0)

    pre = PreheaterDyn(cfg, w_init=init_res.omega_opt)
    pre.initialize(init_cmd.Tg_cmd_C, init_cmd.vg_cmd_mps, ta.slot_state.omega0, init_res.omega_opt)
    fur = FurnaceDyn(cfg)

    current_cmd = init_cmd
    current_opt_omega = init_res.omega_opt
    current_ctrl: ControllerOutput | None = None
    current_gov: GovernorDecision | None = None

    ctrl_elapsed = 0.0
    opt_elapsed = 0.0
    t = 0.0
    while t <= cfg.total_time_s + 1e-9:
        comp = composition_schedule(t)
        disturbance = None if disturbance_schedule is None else disturbance_schedule(t)
        feed = FeedObservation(time_s=t, composition=comp)
        if opt_elapsed <= 1e-12:
            slot = ta.update_slot_state(feed)
        else:
            slot = ta.slot_state

        w_for_furnace = pre.step(current_cmd, slot.omega0, current_opt_omega)
        Tavg, Tstack, vstack = fur.step(w_for_furnace, disturbance=disturbance)
        obs = FurnaceObservation(time_s=t, T_avg_C=Tavg, T_stack_C=Tstack, v_stack_mps=vstack)

        if ctrl_elapsed <= 1e-12:
            current_ctrl = controller.step(obs)
            current_gov = governor.decide(obs, current_ctrl)
        if opt_elapsed <= 1e-12 and current_gov is not None:
            req = ta.build_request(
                slot_time_s=t,
                governor_decision=current_gov,
                resource=ResourceBoundary(cfg.resource_T_stack_cap_C, cfg.resource_v_stack_cap_mps),
                slot_id=name,
            )
            res = lookup.query(req) if lookup is not None else optimize_static_slot(req)
            _, current_cmd = tb.translate(res, time_s=t)
            current_opt_omega = res.omega_opt

        hist.t.append(t)
        hist.T_avg.append(Tavg)
        hist.T_stack.append(Tstack)
        hist.v_stack.append(vstack)
        hist.T_set.append(cfg.T_set_C)
        hist.slot_omega0.append(slot.omega0)
        hist.omega_opt.append(current_opt_omega)
        hist.omega_out.append(w_for_furnace)
        hist.Tg_cmd.append(current_cmd.Tg_cmd_C)
        hist.vg_cmd.append(current_cmd.vg_cmd_mps)
        hist.heater_deficit.append(current_cmd.Q_heat_deficit_kW)
        if current_ctrl is None or current_gov is None:
            hist.omega_est.append(np.nan)
            hist.omega_req.append(np.nan)
            hist.omega_tar.append(np.nan)
            hist.dyn_lo.append(np.nan)
            hist.dyn_hi.append(np.nan)
            hist.Tavg_pred.append(np.nan)
        else:
            hist.omega_est.append(current_ctrl.omega_est)
            hist.omega_req.append(current_ctrl.omega_req)
            hist.omega_tar.append(current_gov.omega_tar)
            hist.dyn_lo.append(current_gov.dyn_band[0])
            hist.dyn_hi.append(current_gov.dyn_band[1])
            hist.Tavg_pred.append(current_gov.Tavg_pred_C)

        ctrl_elapsed += cfg.dt_meas_s
        opt_elapsed += cfg.dt_meas_s
        if ctrl_elapsed >= cfg.dt_ctrl_s - 1e-12:
            ctrl_elapsed = 0.0
        if opt_elapsed >= cfg.dt_opt_s - 1e-12:
            opt_elapsed = 0.0
        t += cfg.dt_meas_s
    return hist


def plot_history(hist: History, title: str, out_png: str):
    fig, axes = plt.subplots(4, 1, figsize=(12, 12), sharex=True)
    axes[0].plot(hist.t, hist.T_avg, label='T_avg (C)')
    axes[0].plot(hist.t, hist.T_stack, label='T_stack (C)')
    axes[0].plot(hist.t, hist.v_stack, label='v_stack (m/s)')
    axes[0].plot(hist.t, hist.T_set, '--', label='T_set (C)')
    axes[0].legend()
    axes[0].set_ylabel('Furnace outputs')

    axes[1].plot(hist.t, hist.omega_est, label='omega_est')
    axes[1].plot(hist.t, hist.omega_req, label='omega_req')
    axes[1].plot(hist.t, hist.omega_tar, label='omega_tar')
    axes[1].plot(hist.t, hist.dyn_lo, '--', label='Wdyn low')
    axes[1].plot(hist.t, hist.dyn_hi, '--', label='Wdyn high')
    axes[1].legend()
    axes[1].set_ylabel('Target chain')
    axes[1].set_ylim(0.18, 0.62)

    axes[2].plot(hist.t, hist.slot_omega0, label='slot omega0')
    axes[2].plot(hist.t, hist.omega_opt, label='lookup omega_opt')
    axes[2].plot(hist.t, hist.omega_out, label='actual omega_out')
    axes[2].legend()
    axes[2].set_ylabel('Slot / outlet')
    axes[2].set_ylim(0.18, 0.95)

    axes[3].plot(hist.t, hist.Tg_cmd, label='Tg_cmd')
    axes[3].plot(hist.t, hist.vg_cmd, label='vg_cmd')
    axes[3].plot(hist.t, hist.heater_deficit, label='Q_heat_deficit')
    axes[3].legend()
    axes[3].set_ylabel('Commands')
    axes[3].set_xlabel('time (s)')

    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(out_png, dpi=150)
    plt.close(fig)
