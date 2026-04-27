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

from model.control_types import DEFAULT_PREHEATER_GAINS, ActuatorCommand, FeedObservation, FurnaceObservation, ControlSetpoint
from model.model_types import DynamicTargetBand, EquivalentProperties, OptimizerRequest, ResourceBoundary
from controller.optimizer.optimizer import optimize_static_slot
from runtime.execution_adapter import TranscriberB

from controller.mpc_controller import MPCConfig, MPCController
from controller.nmpc_controller import NMPCConfig, NonlinearMPCController
from model.resource_model import ResourceModel, ResourceModelConfig
from model.feed_preview import KnownScheduleFeedPreview
from controller.estimators.furnace_disturbance_observer import FurnaceDisturbanceObserver
from model.preheater_forward_model import PreheaterForwardConfig, PreheaterForwardModel
from model.furnace_dynamic import FurnaceDyn as NewFurnaceDyn, FurnaceDynConfig
from controller.optimizer.dynamic_slot_lookup import DynamicSlotLookup


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
    control_mode: str = "lookup_mpc"
    dynamic_lookup_table: str = "controller/optimizer/generated_tables/dynamic_slot_lookup_table_dev.csv"
    mpc_dt_s: float = 20.0
    mpc_horizon_s: float = 600.0
    mpc_representative_remaining_s: float = 360.0
    preheater_n_cells: int = 20

    # NMPC / initialization options. The nominal command is the full-preheater
    # steady-hold point used to synchronize preheater inventory, actuator limiter,
    # and controller warm start.
    nominal_Tg_C: float = 800.0
    nominal_vg_mps: float = 12.0
    preheater_warmup_s: float = 6.0 * 985.0
    preheater_warmup_dt_s: float = 20.0
    nmpc_reoptimize_s: float = 60.0
    # NMPC can keep its 20 s decision grid, while internally integrating
    # the distributed preheater with a smaller substep for prediction/plant
    # consistency.
    nmpc_rollout_dt_s: float = 5.0
    nmpc_maxiter: int = 20

    # Dynamic resource / auxiliary heat model. Natural resources come from
    # predicted/measured T_stack and v_stack; auxiliary heat can raise Tg up to
    # aux_Tg_max_C but cannot create extra mass flow.
    stack_to_preheater_loss_C: float = 0.0
    extractable_velocity_fraction: float = 1.0
    aux_Tg_max_C: float = 930.0

    # Disturbance observer: NMPC receives estimated additive disturbance, not
    # the scenario's ground-truth disturbance.
    disturbance_observer_alpha: float = 0.05

    case_name: str = "case"
    out_dir: str = "runtime/results"
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
    control_source: List[str] = field(default_factory=list)
    control_note: List[str] = field(default_factory=list)
    T_stack_available: List[float] = field(default_factory=list)
    v_stack_available: List[float] = field(default_factory=list)
    mdot_stack_available: List[float] = field(default_factory=list)
    mdot_preheater_cmd: List[float] = field(default_factory=list)
    resource_limited: List[float] = field(default_factory=list)
    aux_heat_enable: List[float] = field(default_factory=list)
    mdot_aux_flow: List[float] = field(default_factory=list)
    fan_circulation_power: List[float] = field(default_factory=list)
    disturbance_est_Tavg: List[float] = field(default_factory=list)
    disturbance_est_Tstack: List[float] = field(default_factory=list)
    disturbance_est_vstack: List[float] = field(default_factory=list)


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
    from model.material_model import composition_to_equivalent_properties
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


def _make_command(time_s: float, Tg: float, vg: float) -> ActuatorCommand:
    return ActuatorCommand(
        time_s=float(time_s),
        Tg_cmd_C=float(Tg),
        vg_cmd_mps=float(vg),
        heater_enable=False,
        Q_heat_deficit_kW=0.0,
        resource_limited=False,
        Tg_limited_by_stack=False,
        vg_limited_by_stack=False,
    )


def _warm_start_preheater(preheater: PreheaterForwardModel, *, composition, cfg: SimConfig) -> None:
    """Bring the distributed preheater inventory close to the nominal hold state.

    The previous version initialized omega/T_solid directly while actuator and
    controller started from unrelated values. This artificial inconsistency was
    responsible for the large initial excursion. Here the inventory is pre-rolled
    under the same nominal command that initializes the actuator and NMPC.
    """
    warm_s = max(0.0, float(cfg.preheater_warmup_s))
    if warm_s <= 0.0:
        return
    dt = max(1e-6, float(cfg.preheater_warmup_dt_s))
    n_steps = max(1, int(math.ceil(warm_s / dt)))
    for k in range(n_steps):
        t_warm = -warm_s + (k + 1) * dt
        feed = FeedObservation(time_s=t_warm, composition=composition)
        preheater.step(feed, cfg.nominal_Tg_C, cfg.nominal_vg_mps, dt)
    # Reset the public clock so the test artifacts still start at t=0.
    preheater.time_s = 0.0


def _normalize_disturbance(disturbance):
    if disturbance is None:
        return 0.0, 0.0, 0.0
    if isinstance(disturbance, (tuple, list)) and len(disturbance) == 3:
        return float(disturbance[0]), float(disturbance[1]), float(disturbance[2])
    return float(disturbance), 0.0, 0.0


def run_case(name: str, composition_schedule, disturbance_schedule=None, cfg: SimConfig | None = None):
    """Run a scenario using the refactored lookup-assisted MPC path.

    The legacy classes remain in this file for comparison, but the default path
    now uses:
      distributed preheater forward model -> dynamic slot lookup -> MPC -> actuator limiter.
    """
    cfg = cfg or SimConfig(case_name=name)
    hist = History()
    tb = TranscriberB()
    resource_model = ResourceModel(ResourceModelConfig(
        stack_to_preheater_loss_C=cfg.stack_to_preheater_loss_C,
        extractable_velocity_fraction=cfg.extractable_velocity_fraction,
        aux_Tg_max_C=cfg.aux_Tg_max_C,
        vg_cmd_max_mps=12.0,
    ))
    # Initial effective resource; updated dynamically after the first furnace observation.
    resource = ResourceBoundary(cfg.aux_Tg_max_C, min(cfg.resource_v_stack_cap_mps, 12.0))
    feed_preview = KnownScheduleFeedPreview(composition_schedule)

    initial_comp = composition_schedule(0.0)
    init_omega = cfg.omega_out_init if cfg.omega_out_init is not None else cfg.omega_ref
    preheater = PreheaterForwardModel(PreheaterForwardConfig(
        n_cells=cfg.preheater_n_cells,
        tau_residence_s=cfg.pre_tau_s,
        feed_delay_s=cfg.pre_dead_s,
    ))
    preheater.initialize(initial_comp, omega_init=init_omega, T_solid_init_C=120.0, time_s=0.0)
    if cfg.furnace_init_mode != 'custom':
        _warm_start_preheater(preheater, composition=initial_comp, cfg=cfg)

    furnace = NewFurnaceDyn(FurnaceDynConfig(
        omega_ref=cfg.omega_ref,
        dead_s=cfg.furnace_dead_s,
        tau1_s=cfg.furnace_tau1_s,
        tau2_s=cfg.furnace_tau2_s,
        dt_s=cfg.dt_meas_s,
    ))
    if cfg.furnace_init_mode == 'custom':
        furnace.initialize_outputs(
            cfg.T_avg_init_C if cfg.T_avg_init_C is not None else cfg.T_set_C,
            cfg.T_stack_init_C if cfg.T_stack_init_C is not None else furnace_outputs_from_omega(cfg.omega_ref)[1],
            cfg.v_stack_init_mps if cfg.v_stack_init_mps is not None else furnace_outputs_from_omega(cfg.omega_ref)[2],
        )

    # Parallel disturbance-free furnace model for residual-based disturbance estimation.
    furnace_nominal = furnace.clone()
    disturbance_observer = FurnaceDisturbanceObserver()
    disturbance_observer.cfg = type(disturbance_observer.cfg)(alpha=cfg.disturbance_observer_alpha)

    lookup_path = Path(__file__).resolve().parent.parent / cfg.dynamic_lookup_table
    dyn_lookup = DynamicSlotLookup(lookup_path, k_neighbors=cfg.lookup_k, fallback_slsqp=True)
    fallback_mpc = MPCController(dyn_lookup, MPCConfig(
        dt_mpc_s=cfg.mpc_dt_s,
        horizon_s=cfg.mpc_horizon_s,
        T_set_C=cfg.T_set_C,
        omega_ref=cfg.omega_ref,
        representative_remaining_s=cfg.mpc_representative_remaining_s,
    ))
    fallback_mpc.prev_Tg_ref_C = cfg.nominal_Tg_C
    fallback_mpc.prev_vg_ref_mps = cfg.nominal_vg_mps
    mpc = NonlinearMPCController(NMPCConfig(
        dt_pred_s=cfg.mpc_dt_s,
        horizon_s=cfg.mpc_horizon_s,
        reoptimize_s=cfg.nmpc_reoptimize_s,
        rollout_dt_s=cfg.nmpc_rollout_dt_s,
        T_set_C=cfg.T_set_C,
        omega_ref=cfg.omega_ref,
        nominal_Tg_C=cfg.nominal_Tg_C,
        nominal_vg_mps=cfg.nominal_vg_mps,
        maxiter=cfg.nmpc_maxiter,
    ), fallback_controller=fallback_mpc, resource_model=resource_model)
    mpc.initialize(Tg_ref_C=cfg.nominal_Tg_C, vg_ref_mps=cfg.nominal_vg_mps, time_s=0.0)

    # Consistent warm initialization: actuator limiter, current command, and NMPC
    # warm start all begin from the same nominal hold point. This removes the
    # artificial initial kick caused by starting the plant near omega_ref while
    # commanding only ~215-300 C gas.
    if hasattr(tb, 'initialize_previous'):
        tb.initialize_previous(cfg.nominal_Tg_C, cfg.nominal_vg_mps)
    current_cmd = _make_command(0.0, cfg.nominal_Tg_C, cfg.nominal_vg_mps)
    current_decision = None

    opt_elapsed = 0.0
    t = 0.0
    while t <= cfg.total_time_s + 1e-9:
        comp = composition_schedule(t)
        disturbance = None if disturbance_schedule is None else disturbance_schedule(t)
        d_avg, d_stack, d_v = _normalize_disturbance(disturbance)
        feed = FeedObservation(time_s=t, composition=comp)

        pre_state = preheater.step(feed, current_cmd.Tg_cmd_C, current_cmd.vg_cmd_mps, cfg.dt_meas_s)
        omega_out = pre_state.omega_out
        Tavg, Tstack, vstack = furnace.step(omega_out, dt_s=cfg.dt_meas_s, disturbance=(d_avg, d_stack, d_v))
        obs = FurnaceObservation(time_s=t, T_avg_C=Tavg, T_stack_C=Tstack, v_stack_mps=vstack)
        nom_Tavg, nom_Tstack, nom_vstack = furnace_nominal.step(omega_out, dt_s=cfg.dt_meas_s, disturbance=None)
        nom_obs = FurnaceObservation(time_s=t, T_avg_C=nom_Tavg, T_stack_C=nom_Tstack, v_stack_mps=nom_vstack)
        disturbance_est = disturbance_observer.update(obs, nom_obs)
        resource_state = resource_model.from_observation(obs)
        resource = resource_state.effective_resource

        if opt_elapsed <= 1e-12:
            current_decision = mpc.step(
                obs=obs,
                preheater=preheater,
                furnace=furnace_nominal,
                feed=feed,
                resource=resource,
                prev_cmd=current_cmd,
                disturbance=disturbance_est.as_tuple(),
                feed_preview=feed_preview,
            )
            setpoint = ControlSetpoint(
                time_s=t,
                Tg_ref_C=current_decision.Tg_ref_C,
                vg_ref_mps=current_decision.vg_ref_mps,
                source=current_decision.source,
                omega_target=current_decision.omega_target,
                omega_reachable=current_decision.omega_reachable,
                mdot_stack_cap_kgps=resource_state.mdot_stack_available_kgps,
                T_stack_available_C=resource_state.T_stack_available_C,
                v_stack_available_mps=resource_state.v_stack_available_mps,
            )
            current_cmd = tb.translate_setpoint(setpoint)

        # Lightweight equivalent moisture estimate for legacy plots/metrics.
        omega_est = float(np.clip((Tavg - 1294.871365) / (100.0 * -13.109632), 0.05, 0.95))
        omega_target = current_decision.omega_target if current_decision is not None else cfg.omega_ref
        pred_T = current_decision.predicted_Tavg_C if current_decision is not None else np.nan

        hist.t.append(t)
        hist.T_avg.append(Tavg)
        hist.T_stack.append(Tstack)
        hist.v_stack.append(vstack)
        hist.T_set.append(cfg.T_set_C)
        hist.slot_omega0.append(float(pre_state.cells[-1].omega0))
        hist.omega_opt.append(float(omega_target))
        hist.omega_out.append(float(omega_out))
        hist.Tg_cmd.append(current_cmd.Tg_cmd_C)
        hist.vg_cmd.append(current_cmd.vg_cmd_mps)
        hist.heater_deficit.append(current_cmd.Q_heat_deficit_kW)
        hist.disturbance_Tavg.append(d_avg)
        hist.disturbance_Tstack.append(d_stack)
        hist.disturbance_vstack.append(d_v)
        feed_vals = list(comp) + [0.0] * max(0, 6 - len(comp))
        hist.feed_x1.append(feed_vals[0]); hist.feed_x2.append(feed_vals[1]); hist.feed_x3.append(feed_vals[2])
        hist.feed_x4.append(feed_vals[3]); hist.feed_x5.append(feed_vals[4]); hist.feed_x6.append(feed_vals[5])
        hist.omega_est.append(omega_est)
        hist.omega_req.append(float(omega_target))
        hist.omega_tar.append(float(omega_target))
        hist.dyn_lo.append(0.20)
        hist.dyn_hi.append(0.60)
        hist.Tavg_pred.append(float(pred_T))
        hist.error_C.append(cfg.T_set_C - Tavg)
        hist.governor_clipped.append(0.0)
        hist.fast_mode.append(0.0)
        hist.control_source.append(current_decision.source if current_decision is not None else '')
        hist.control_note.append(current_decision.note if current_decision is not None else '')
        hist.T_stack_available.append(float(resource_state.T_stack_available_C))
        hist.v_stack_available.append(float(resource_state.v_stack_available_mps))
        hist.mdot_stack_available.append(float(resource_state.mdot_stack_available_kgps))
        try:
            from model.thermal_core import mdot_preheater
            hist.mdot_preheater_cmd.append(float(mdot_preheater(current_cmd.Tg_cmd_C, current_cmd.vg_cmd_mps, preheater.opt_cfg)))
        except Exception:
            hist.mdot_preheater_cmd.append(float('nan'))
        hist.resource_limited.append(1.0 if current_cmd.resource_limited else 0.0)
        hist.aux_heat_enable.append(1.0 if current_cmd.heater_enable else 0.0)
        hist.mdot_aux_flow.append(float(getattr(current_cmd, 'mdot_aux_flow_kgps', 0.0)))
        hist.fan_circulation_power.append(float(getattr(current_cmd, 'fan_circulation_power_kW', 0.0)))
        hist.disturbance_est_Tavg.append(float(disturbance_est.d_Tavg_C))
        hist.disturbance_est_Tstack.append(float(disturbance_est.d_Tstack_C))
        hist.disturbance_est_vstack.append(float(disturbance_est.d_vstack_mps))

        opt_elapsed += cfg.dt_meas_s
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
    aux_flow = np.asarray(hist.mdot_aux_flow, dtype=float)
    fan_power = np.asarray(hist.fan_circulation_power, dtype=float)
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
            'aux_flow_mean_kgps': float(np.nanmean(aux_flow[mask])),
            'aux_flow_max_kgps': float(np.nanmax(aux_flow[mask])),
            'fan_power_mean_kW': float(np.nanmean(fan_power[mask])),
            'fan_energy_kJ': float(np.nansum(fan_power[mask]) * dt),
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
            'aux_flow_mean_kgps': float(np.nanmean(aux_flow[post_mask])),
            'aux_flow_max_kgps': float(np.nanmax(aux_flow[post_mask])),
            'fan_power_mean_kW': float(np.nanmean(fan_power[post_mask])),
            'fan_energy_kJ': float(np.nansum(fan_power[post_mask]) * dt),
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
            'control_source': hist.control_source[i],
            'control_note': hist.control_note[i],
            'T_stack_available_C': hist.T_stack_available[i],
            'v_stack_available_mps': hist.v_stack_available[i],
            'mdot_stack_available_kgps': hist.mdot_stack_available[i],
            'mdot_preheater_cmd_kgps': hist.mdot_preheater_cmd[i],
            'resource_limited': hist.resource_limited[i],
            'aux_heat_enable': hist.aux_heat_enable[i],
            'mdot_aux_flow_kgps': hist.mdot_aux_flow[i],
            'fan_circulation_power_kW': hist.fan_circulation_power[i],
            'disturbance_est_Tavg_C': hist.disturbance_est_Tavg[i],
            'disturbance_est_Tstack_C': hist.disturbance_est_Tstack[i],
            'disturbance_est_vstack_mps': hist.disturbance_est_vstack[i],
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
    fig, axes = plt.subplots(6, 1, figsize=(13, 18), sharex=True)

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
    axes[3].legend(ncol=2, fontsize=8)
    axes[3].set_ylabel('Commands')

    axes[4].plot(hist.t, hist.error_C, label='error_C')
    axes[4].plot(hist.t, hist.disturbance_Tavg, label='disturbance_Tavg')
    axes[4].plot(hist.t, hist.governor_clipped, label='governor_clipped')
    axes[4].plot(hist.t, hist.fast_mode, label='fast_mode')
    axes[4].legend(ncol=4, fontsize=8)
    axes[4].set_ylabel('Diagnostics')

    # Resource / auxiliary diagnostics are separated from Tg/vg commands so
    # auxiliary heat and circulation demand are visible instead of being hidden
    # by the command scale.  Left axis: kW; right axis: kg/s.
    axes[5].plot(hist.t, hist.heater_deficit, label='Q_aux_heat (kW)')
    axes[5].plot(hist.t, hist.fan_circulation_power, label='fan/circ power (kW)')
    axes[5].set_ylabel('Aux / fan kW')
    ax5b = axes[5].twinx()
    ax5b.plot(hist.t, hist.mdot_aux_flow, linestyle='--', label='aux/circ flow (kg/s)')
    ax5b.set_ylabel('Aux flow kg/s')
    lines, labels = axes[5].get_legend_handles_labels()
    lines2, labels2 = ax5b.get_legend_handles_labels()
    axes[5].legend(lines + lines2, labels + labels2, ncol=3, fontsize=8)
    axes[5].set_xlabel('time (s)')

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
