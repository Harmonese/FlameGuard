from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from types import SimpleNamespace
from typing import Callable, Deque, Iterable, List, Sequence
import csv
import math

import matplotlib.pyplot as plt
import numpy as np

from control_types import DEFAULT_PREHEATER_GAINS, ActuatorCommand, ControllerOutput, FeedObservation, FurnaceObservation, GovernorDecision
from controller.controller import PIDConfig, PIDController
from governor.governor import DynamicGovernor, DynamicGovernorConfig
from optimizer.optimizer import DynamicTargetBand, EquivalentProperties, OptimizerRequest, ResourceBoundary, optimize_static_slot
from transcriber.transcriber_a import TranscriberA, TranscriberAConfig
from transcriber.transcriber_b import TranscriberB


REFERENCE_TAVG_BAND = (850.0, 895.99)
SAFE_TAVG_BAND = (850.0, 1100.0)
SUPERVISION_TAVG_BAND = (845.0, 1105.0)
REFERENCE_OMEGA_BAND = (0.3042675804697915, 0.3393469511577442)


def furnace_outputs_from_omega(omega: float):
    return (
        -13.109632 * (100.0 * omega) + 1294.871365,
        -14.412237 * (100.0 * omega) + 1423.472316,
        -0.215310 * (100.0 * omega) + 25.332842,
    )


@dataclass
class EventWindow:
    start_s: float
    end_s: float
    label: str


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

    case_name: str = "case"
    out_dir: str = "tests/results"
    notes: str = ""
    event_start_s: float | None = None
    event_end_s: float | None = None
    tail_window_s: float = 1200.0
    settle_hold_safe_s: float = 300.0
    settle_hold_ref_s: float = 600.0
    overshoot_deadband_C: float = 2.0

    # More realistic initialization support than the ad-hoc cold-start note.
    furnace_init_mode: str = "warm"   # "warm" or "custom"
    T_avg_init_C: float | None = None
    T_stack_init_C: float | None = None
    v_stack_init_mps: float | None = None
    omega_out_init: float | None = None

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
    error_C: List[float] = field(default_factory=list)
    governor_clipped: List[float] = field(default_factory=list)
    fast_mode: List[float] = field(default_factory=list)
    disturbance_Tavg: List[float] = field(default_factory=list)
    disturbance_Tstack: List[float] = field(default_factory=list)
    disturbance_vstack: List[float] = field(default_factory=list)
    feed_x1: List[float] = field(default_factory=list)
    feed_x2: List[float] = field(default_factory=list)
    feed_x3: List[float] = field(default_factory=list)
    feed_x4: List[float] = field(default_factory=list)
    feed_x5: List[float] = field(default_factory=list)
    feed_x6: List[float] = field(default_factory=list)


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

    def initialize_outputs(self, T_avg_init_C: float, T_stack_init_C: float, v_stack_init_mps: float):
        targets = {
            'T_avg': float(T_avg_init_C),
            'T_stack': float(T_stack_init_C),
            'v_stack': float(v_stack_init_mps),
        }
        for key in ['T_avg', 'T_stack', 'v_stack']:
            delta = targets[key] - self.refs[key]
            self.states[key] = (delta, delta)

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


def _normalize_disturbance(disturbance):
    if disturbance is None:
        return 0.0, 0.0, 0.0
    if isinstance(disturbance, (tuple, list)) and len(disturbance) == 3:
        return float(disturbance[0]), float(disturbance[1]), float(disturbance[2])
    return float(disturbance), 0.0, 0.0


def run_case(name: str, composition_schedule, disturbance_schedule=None, cfg: SimConfig | None = None):
    cfg = cfg or SimConfig(case_name=name)
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

    init_omega_out = init_res.omega_opt if cfg.omega_out_init is None else float(cfg.omega_out_init)
    pre = PreheaterDyn(cfg, w_init=init_omega_out)
    pre.initialize(init_cmd.Tg_cmd_C, init_cmd.vg_cmd_mps, ta.slot_state.omega0, init_res.omega_opt)
    fur = FurnaceDyn(cfg)
    if cfg.furnace_init_mode == 'custom':
        fur.initialize_outputs(
            cfg.T_avg_init_C if cfg.T_avg_init_C is not None else cfg.T_set_C,
            cfg.T_stack_init_C if cfg.T_stack_init_C is not None else furnace_outputs_from_omega(cfg.omega_ref)[1],
            cfg.v_stack_init_mps if cfg.v_stack_init_mps is not None else furnace_outputs_from_omega(cfg.omega_ref)[2],
        )

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
        d_avg, d_stack, d_v = _normalize_disturbance(disturbance)
        feed = FeedObservation(time_s=t, composition=comp)

        if opt_elapsed <= 1e-12:
            slot = ta.update_slot_state(feed)
        else:
            slot = ta.slot_state

        w_for_furnace = pre.step(current_cmd, slot.omega0, current_opt_omega)
        Tavg, Tstack, vstack = fur.step(w_for_furnace, disturbance=(d_avg, d_stack, d_v))
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
        hist.disturbance_Tavg.append(d_avg)
        hist.disturbance_Tstack.append(d_stack)
        hist.disturbance_vstack.append(d_v)
        feed_vals = list(comp) + [0.0] * max(0, 6 - len(comp))
        hist.feed_x1.append(feed_vals[0])
        hist.feed_x2.append(feed_vals[1])
        hist.feed_x3.append(feed_vals[2])
        hist.feed_x4.append(feed_vals[3])
        hist.feed_x5.append(feed_vals[4])
        hist.feed_x6.append(feed_vals[5])
        if current_ctrl is None or current_gov is None:
            hist.omega_est.append(np.nan)
            hist.omega_req.append(np.nan)
            hist.omega_tar.append(np.nan)
            hist.dyn_lo.append(np.nan)
            hist.dyn_hi.append(np.nan)
            hist.Tavg_pred.append(np.nan)
            hist.error_C.append(np.nan)
            hist.governor_clipped.append(np.nan)
            hist.fast_mode.append(np.nan)
        else:
            hist.omega_est.append(current_ctrl.omega_est)
            hist.omega_req.append(current_ctrl.omega_req)
            hist.omega_tar.append(current_gov.omega_tar)
            hist.dyn_lo.append(current_gov.dyn_band[0])
            hist.dyn_hi.append(current_gov.dyn_band[1])
            hist.Tavg_pred.append(current_gov.Tavg_pred_C)
            hist.error_C.append(current_ctrl.error_C)
            hist.governor_clipped.append(float(abs(current_gov.omega_tar - current_ctrl.omega_req) > 1e-9))
            hist.fast_mode.append(float(getattr(controller, 'fast_mode', False)))

        ctrl_elapsed += cfg.dt_meas_s
        opt_elapsed += cfg.dt_meas_s
        if ctrl_elapsed >= cfg.dt_ctrl_s - 1e-12:
            ctrl_elapsed = 0.0
        if opt_elapsed >= cfg.dt_opt_s - 1e-12:
            opt_elapsed = 0.0
        t += cfg.dt_meas_s
    return hist


def _bool_ratio(mask: np.ndarray) -> float:
    return float(np.mean(mask.astype(float))) if mask.size else float('nan')


def _rms(x: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.square(x)))) if x.size else float('nan')


def _first_sustained_time(t: np.ndarray, mask: np.ndarray, dt: float, hold_s: float, start_index: int = 0) -> float:
    if t.size == 0 or mask.size == 0:
        return float('nan')
    hold_n = max(1, int(round(hold_s / max(dt, 1e-9))))
    for i in range(max(0, start_index), len(mask) - hold_n + 1):
        if np.all(mask[i:i + hold_n]):
            return float(t[i])
    return float('nan')


def _count_zero_crossings_with_deadband(x: np.ndarray, deadband: float) -> int:
    if x.size == 0:
        return 0
    sign = np.zeros_like(x, dtype=int)
    sign[x > deadband] = 1
    sign[x < -deadband] = -1
    last = 0
    count = 0
    for s in sign:
        if s == 0:
            continue
        if last != 0 and s != last:
            count += 1
        last = s
    return int(count)


def _segment_rows(hist: History, cfg: SimConfig) -> list[dict[str, float | str]]:
    t = np.asarray(hist.t, dtype=float)
    T_avg = np.asarray(hist.T_avg, dtype=float)
    T_set = np.asarray(hist.T_set, dtype=float)
    omega_req = np.asarray(hist.omega_req, dtype=float)
    omega_tar = np.asarray(hist.omega_tar, dtype=float)
    omega_est = np.asarray(hist.omega_est, dtype=float)
    heater = np.asarray(hist.heater_deficit, dtype=float)
    Tg_cmd = np.asarray(hist.Tg_cmd, dtype=float)
    omega_opt = np.asarray(hist.omega_opt, dtype=float)
    clipped = np.asarray(hist.governor_clipped, dtype=float)
    fast_mode = np.asarray(hist.fast_mode, dtype=float)
    dt = cfg.dt_meas_s

    event_start = cfg.event_start_s
    event_end = cfg.event_end_s if cfg.event_end_s is not None else cfg.event_start_s
    tail_start = max(0.0, cfg.total_time_s - cfg.tail_window_s)

    segments: list[tuple[str, np.ndarray]] = [('full', np.ones_like(t, dtype=bool))]
    if event_start is not None:
        segments.append(('pre_event', t < event_start))
        if event_end is not None and event_end > event_start:
            segments.append(('event_window', (t >= event_start) & (t < event_end)))
            segments.append(('post_event', t >= event_end))
        else:
            segments.append(('post_event', t >= event_start))
    segments.append(('tail', t >= tail_start))

    rows: list[dict[str, float | str]] = []
    for segment_name, mask in segments:
        if not np.any(mask):
            continue
        Ts = T_avg[mask]
        Es = T_set[mask] - Ts
        om_req = omega_req[mask]
        om_tar = omega_tar[mask]
        om_est = omega_est[mask]
        heater_s = heater[mask]
        tg_s = Tg_cmd[mask]
        om_opt_s = omega_opt[mask]
        clip_s = clipped[mask]
        fast_s = fast_mode[mask]
        rows.append({
            'case_name': cfg.case_name,
            'segment': segment_name,
            't_start_s': float(t[mask][0]),
            't_end_s': float(t[mask][-1]),
            'duration_s': float(mask.sum() * dt),
            'Tavg_mean_C': float(np.mean(Ts)),
            'Tavg_min_C': float(np.min(Ts)),
            'Tavg_max_C': float(np.max(Ts)),
            'Tavg_MAE_C': float(np.mean(np.abs(Es))),
            'Tavg_RMSE_C': _rms(Es),
            'Tavg_pp_C': float(np.max(Ts) - np.min(Ts)),
            'ratio_in_ref_band': _bool_ratio((Ts >= REFERENCE_TAVG_BAND[0]) & (Ts <= REFERENCE_TAVG_BAND[1])),
            'ratio_in_safe_band': _bool_ratio((Ts >= SAFE_TAVG_BAND[0]) & (Ts <= SAFE_TAVG_BAND[1])),
            'ratio_in_supervision_band': _bool_ratio((Ts >= SUPERVISION_TAVG_BAND[0]) & (Ts <= SUPERVISION_TAVG_BAND[1])),
            'omega_req_mean': float(np.nanmean(om_req)),
            'omega_tar_mean': float(np.nanmean(om_tar)),
            'omega_est_mean': float(np.nanmean(om_est)),
            'omega_opt_mean': float(np.nanmean(om_opt_s)),
            'omega_req_tv': float(np.nansum(np.abs(np.diff(om_req)))),
            'Tg_cmd_tv': float(np.nansum(np.abs(np.diff(tg_s)))),
            'governor_clip_ratio': float(np.nanmean(clip_s)),
            'fast_mode_ratio': float(np.nanmean(fast_s)),
            'heater_deficit_energy_kJ': float(np.nansum(heater_s) * dt),
        })

    post_mask = segments[-2][1] if any(name == 'post_event' for name, _ in segments) else segments[-1][1]
    if np.any(post_mask):
        Ts = T_avg[post_mask]
        Es = T_set[post_mask] - Ts
        t_post = t[post_mask]
        start_idx = np.where(post_mask)[0][0]
        safe_mask = (T_avg >= SAFE_TAVG_BAND[0]) & (T_avg <= SAFE_TAVG_BAND[1])
        ref_mask = (T_avg >= REFERENCE_TAVG_BAND[0]) & (T_avg <= REFERENCE_TAVG_BAND[1])
        rows.append({
            'case_name': cfg.case_name,
            'segment': 'recovery',
            't_start_s': float(t_post[0]),
            't_end_s': float(t_post[-1]),
            'duration_s': float(len(t_post) * dt),
            'Tavg_mean_C': float(np.mean(Ts)),
            'Tavg_min_C': float(np.min(Ts)),
            'Tavg_max_C': float(np.max(Ts)),
            'Tavg_MAE_C': float(np.mean(np.abs(Es))),
            'Tavg_RMSE_C': _rms(Es),
            'Tavg_pp_C': float(np.max(Ts) - np.min(Ts)),
            'ratio_in_ref_band': _bool_ratio((Ts >= REFERENCE_TAVG_BAND[0]) & (Ts <= REFERENCE_TAVG_BAND[1])),
            'ratio_in_safe_band': _bool_ratio((Ts >= SAFE_TAVG_BAND[0]) & (Ts <= SAFE_TAVG_BAND[1])),
            'ratio_in_supervision_band': _bool_ratio((Ts >= SUPERVISION_TAVG_BAND[0]) & (Ts <= SUPERVISION_TAVG_BAND[1])),
            'recovery_to_safe_s': _first_sustained_time(t, safe_mask, dt, cfg.settle_hold_safe_s, start_idx),
            'recovery_to_ref_s': _first_sustained_time(t, ref_mask, dt, cfg.settle_hold_ref_s, start_idx),
            'overshoot_crossings': _count_zero_crossings_with_deadband(Ts - T_set[post_mask], cfg.overshoot_deadband_C),
            'governor_clip_ratio': float(np.nanmean(clipped[post_mask])),
            'fast_mode_ratio': float(np.nanmean(fast_mode[post_mask])),
            'heater_deficit_energy_kJ': float(np.nansum(heater[post_mask]) * dt),
            'omega_req_tv': float(np.nansum(np.abs(np.diff(omega_req[post_mask])))),
            'Tg_cmd_tv': float(np.nansum(np.abs(np.diff(Tg_cmd[post_mask])))),
        })

    return rows


def history_to_csv_rows(hist: History) -> list[dict[str, float]]:
    rows = []
    n = len(hist.t)
    for i in range(n):
        rows.append({
            'time_s': hist.t[i],
            'T_avg_C': hist.T_avg[i],
            'T_stack_C': hist.T_stack[i],
            'v_stack_mps': hist.v_stack[i],
            'T_set_C': hist.T_set[i],
            'error_C': hist.error_C[i],
            'omega_est': hist.omega_est[i],
            'omega_req': hist.omega_req[i],
            'omega_tar': hist.omega_tar[i],
            'dyn_lo': hist.dyn_lo[i],
            'dyn_hi': hist.dyn_hi[i],
            'slot_omega0': hist.slot_omega0[i],
            'omega_opt': hist.omega_opt[i],
            'omega_out': hist.omega_out[i],
            'Tg_cmd_C': hist.Tg_cmd[i],
            'vg_cmd_mps': hist.vg_cmd[i],
            'heater_deficit_kW': hist.heater_deficit[i],
            'Tavg_pred_C': hist.Tavg_pred[i],
            'governor_clipped': hist.governor_clipped[i],
            'fast_mode': hist.fast_mode[i],
            'disturbance_Tavg_C': hist.disturbance_Tavg[i],
            'disturbance_Tstack_C': hist.disturbance_Tstack[i],
            'disturbance_vstack_mps': hist.disturbance_vstack[i],
            'feed_x1': hist.feed_x1[i],
            'feed_x2': hist.feed_x2[i],
            'feed_x3': hist.feed_x3[i],
            'feed_x4': hist.feed_x4[i],
            'feed_x5': hist.feed_x5[i],
            'feed_x6': hist.feed_x6[i],
        })
    return rows


def _write_csv(path: Path, rows: Sequence[dict]):
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        return
    fieldnames: list[str] = []
    seen = set()
    for row in rows:
        for key in row.keys():
            if key not in seen:
                seen.add(key)
                fieldnames.append(key)
    with path.open('w', encoding='utf-8', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def plot_history(hist: History, out_png: str | Path, title: str, *, event_windows: Sequence[EventWindow] | None = None):
    fig, axes = plt.subplots(5, 1, figsize=(13, 15), sharex=True)

    for ev in event_windows or []:
        for ax in axes:
            ax.axvspan(ev.start_s, ev.end_s, alpha=0.12)

    axes[0].plot(hist.t, hist.T_avg, label='T_avg (C)')
    axes[0].plot(hist.t, hist.T_stack, label='T_stack (C)')
    axes[0].plot(hist.t, hist.T_set, '--', label='T_set (C)')
    axes[0].axhline(REFERENCE_TAVG_BAND[0], linestyle=':', label='Tavg ref low')
    axes[0].axhline(REFERENCE_TAVG_BAND[1], linestyle=':', label='Tavg ref high')
    axes[0].axhline(SAFE_TAVG_BAND[0], linestyle='-.', label='Tavg safe low')
    axes[0].axhline(SAFE_TAVG_BAND[1], linestyle='-.', label='Tavg safe high')
    axes[0].legend(ncol=3, fontsize=8)
    axes[0].set_ylabel('Furnace')

    axes[1].plot(hist.t, hist.omega_est, label='omega_est')
    axes[1].plot(hist.t, hist.omega_req, label='omega_req')
    axes[1].plot(hist.t, hist.omega_tar, label='omega_tar')
    axes[1].plot(hist.t, hist.dyn_lo, '--', label='Wdyn low')
    axes[1].plot(hist.t, hist.dyn_hi, '--', label='Wdyn high')
    axes[1].axhline(REFERENCE_OMEGA_BAND[0], linestyle=':', label='Wss low')
    axes[1].axhline(REFERENCE_OMEGA_BAND[1], linestyle=':', label='Wss high')
    axes[1].legend(ncol=4, fontsize=8)
    axes[1].set_ylabel('Target chain')
    axes[1].set_ylim(0.18, 0.62)

    axes[2].plot(hist.t, hist.slot_omega0, label='slot omega0')
    axes[2].plot(hist.t, hist.omega_opt, label='lookup omega_opt')
    axes[2].plot(hist.t, hist.omega_out, label='actual omega_out')
    axes[2].legend(ncol=3, fontsize=8)
    axes[2].set_ylabel('Slot / outlet')
    axes[2].set_ylim(0.18, 0.95)

    axes[3].plot(hist.t, hist.Tg_cmd, label='Tg_cmd')
    axes[3].plot(hist.t, hist.vg_cmd, label='vg_cmd')
    axes[3].plot(hist.t, hist.heater_deficit, label='Q_heat_deficit')
    axes[3].legend(ncol=3, fontsize=8)
    axes[3].set_ylabel('Commands')

    axes[4].plot(hist.t, hist.error_C, label='error_C')
    axes[4].plot(hist.t, hist.disturbance_Tavg, label='disturbance_Tavg')
    axes[4].plot(hist.t, hist.governor_clipped, label='governor_clipped')
    axes[4].plot(hist.t, hist.fast_mode, label='fast_mode')
    axes[4].legend(ncol=4, fontsize=8)
    axes[4].set_ylabel('Diagnostics')
    axes[4].set_xlabel('time (s)')

    fig.suptitle(title)
    fig.tight_layout()
    out_path = Path(out_png)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def save_case_artifacts(hist: History, cfg: SimConfig, title: str, *, event_windows: Sequence[EventWindow] | None = None) -> dict[str, Path]:
    out_dir = Path(cfg.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    plot_path = out_dir / f'{cfg.case_name}.png'
    timeseries_path = out_dir / f'{cfg.case_name}_timeseries.csv'
    metrics_path = out_dir / f'{cfg.case_name}_metrics.csv'

    plot_history(hist, plot_path, title, event_windows=event_windows)
    _write_csv(timeseries_path, history_to_csv_rows(hist))
    _write_csv(metrics_path, _segment_rows(hist, cfg))
    return {
        'plot': plot_path,
        'timeseries': timeseries_path,
        'metrics': metrics_path,
    }


def print_metrics_table(metrics_csv: Path):
    with metrics_csv.open(encoding='utf-8', newline='') as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    print('segment,duration_s,Tavg_MAE_C,Tavg_RMSE_C,ratio_in_ref_band,ratio_in_safe_band,governor_clip_ratio,omega_req_tv,Tg_cmd_tv,recovery_to_safe_s,recovery_to_ref_s,overshoot_crossings')
    for row in rows:
        print(','.join([
            row.get('segment', ''),
            row.get('duration_s', ''),
            row.get('Tavg_MAE_C', ''),
            row.get('Tavg_RMSE_C', ''),
            row.get('ratio_in_ref_band', ''),
            row.get('ratio_in_safe_band', ''),
            row.get('governor_clip_ratio', ''),
            row.get('omega_req_tv', ''),
            row.get('Tg_cmd_tv', ''),
            row.get('recovery_to_safe_s', ''),
            row.get('recovery_to_ref_s', ''),
            row.get('overshoot_crossings', ''),
        ]))
