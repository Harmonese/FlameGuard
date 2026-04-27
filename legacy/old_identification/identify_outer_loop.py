from __future__ import annotations

import argparse
import csv
import math
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Iterable

import numpy as np

ROOT = os.path.dirname(os.path.dirname(__file__))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from cleanser.cleanser import composition_to_equivalent_properties
from control_types import FeedObservation, FurnaceObservation, GovernorDecision
from controller.controller import PIDController, PIDConfig
from optimizer.optimizer import DynamicTargetBand, EquivalentProperties, OptimizerRequest, ResourceBoundary, optimize_static_slot
from tests.sim_common import FurnaceDyn, LookupOptimizer, PreheaterDyn, SimConfig
from transcriber.transcriber_a import TranscriberA, TranscriberAConfig
from transcriber.transcriber_b import TranscriberB


DEFAULT_COMPOSITION = [0.20, 0.15, 0.15, 0.10, 0.20, 0.20]


@dataclass
class OpenLoopHistory:
    t: list[float]
    omega_req: list[float]
    omega_tar: list[float]
    omega_est: list[float]
    omega_opt: list[float]
    omega_out: list[float]
    T_avg: list[float]
    T_stack: list[float]
    v_stack: list[float]
    Tg_cmd: list[float]
    vg_cmd: list[float]


@dataclass
class StepMetrics:
    delta_u: float
    y0: float
    yss: float
    delta_y: float
    gain: float
    dead_time_s: float | None
    t50_s: float | None
    t63_s: float | None
    t90_s: float | None
    t95_s: float | None


def fixed_composition_schedule(_: float) -> list[float]:
    return list(DEFAULT_COMPOSITION)


def build_initial_result(cfg: SimConfig, composition: list[float], lookup: LookupOptimizer | None):
    eq = composition_to_equivalent_properties(composition).equivalent
    req = OptimizerRequest(
        props=EquivalentProperties(eq.omega0, eq.tref_min, eq.slope_min_per_c),
        omega_tar=cfg.omega_ref,
        resource=ResourceBoundary(cfg.resource_T_stack_cap_C, cfg.resource_v_stack_cap_mps),
        dyn_band=DynamicTargetBand(omega_min=0.20, omega_max=0.60),
        slot_id='identify_init',
        burn_policy='advisory',
    )
    if lookup is not None:
        return lookup.query(req)
    return optimize_static_slot(req)


def run_open_loop_case(
    omega_schedule: Callable[[float], float],
    *,
    composition_schedule: Callable[[float], list[float]] = fixed_composition_schedule,
    cfg: SimConfig | None = None,
) -> OpenLoopHistory:
    cfg = cfg or SimConfig()
    lookup = None
    if cfg.use_lookup_table:
        lookup = LookupOptimizer(Path(__file__).resolve().parent.parent / 'optimizer' / 'optimizer_lookup_table.csv', k_neighbors=cfg.lookup_k)

    estimator = PIDController(PIDConfig(dt_controller_s=cfg.dt_ctrl_s, T_set_C=cfg.T_set_C))
    ta = TranscriberA(
        TranscriberAConfig(
            dt_update_s=cfg.dt_opt_s,
            resource_T_stack_default_C=cfg.resource_T_stack_cap_C,
            resource_v_stack_default_mps=cfg.resource_v_stack_cap_mps,
            burn_policy='advisory',
        )
    )
    tb = TranscriberB()

    initial_comp = composition_schedule(0.0)
    ta.initialize(initial_comp)
    init_res = build_initial_result(cfg, initial_comp, lookup)
    _, init_cmd = tb.translate(init_res, time_s=0.0)

    pre = PreheaterDyn(cfg, w_init=init_res.omega_opt)
    pre.initialize(init_cmd.Tg_cmd_C, init_cmd.vg_cmd_mps, ta.slot_state.omega0, init_res.omega_opt)
    fur = FurnaceDyn(cfg)

    hist = OpenLoopHistory([], [], [], [], [], [], [], [], [], [], [])

    current_cmd = init_cmd
    current_opt_omega = init_res.omega_opt
    current_omega_req = cfg.omega_ref
    current_omega_tar = cfg.omega_ref

    ctrl_elapsed = 0.0
    opt_elapsed = 0.0
    t = 0.0
    while t <= cfg.total_time_s + 1e-9:
        comp = composition_schedule(t)
        feed = FeedObservation(time_s=t, composition=comp)
        if opt_elapsed <= 1e-12:
            slot = ta.update_slot_state(feed)
        else:
            slot = ta.slot_state

        w_for_furnace = pre.step(current_cmd, slot.omega0, current_opt_omega)
        Tavg, Tstack, vstack = fur.step(w_for_furnace, disturbance=None)
        obs = FurnaceObservation(time_s=t, T_avg_C=Tavg, T_stack_C=Tstack, v_stack_mps=vstack)
        moist = estimator.invert_outputs_to_moisture(obs)

        if ctrl_elapsed <= 1e-12:
            current_omega_req = float(np.clip(omega_schedule(t), 0.20, 0.60))
            current_omega_tar = current_omega_req

        if opt_elapsed <= 1e-12:
            fake_gov = GovernorDecision(
                time_s=t,
                omega_est=moist.omega_fused,
                omega_req=current_omega_req,
                omega_tar=current_omega_tar,
                dyn_band=(0.20, 0.60),
                Tavg_pred_C=Tavg,
                note='open-loop identification bypass governor',
            )
            req = ta.build_request(
                slot_time_s=t,
                governor_decision=fake_gov,
                resource=ResourceBoundary(cfg.resource_T_stack_cap_C, cfg.resource_v_stack_cap_mps),
                slot_id='identify_outer_loop',
            )
            res = lookup.query(req) if lookup is not None else optimize_static_slot(req)
            _, current_cmd = tb.translate(res, time_s=t)
            current_opt_omega = res.omega_opt

        hist.t.append(t)
        hist.omega_req.append(current_omega_req)
        hist.omega_tar.append(current_omega_tar)
        hist.omega_est.append(moist.omega_fused)
        hist.omega_opt.append(current_opt_omega)
        hist.omega_out.append(w_for_furnace)
        hist.T_avg.append(Tavg)
        hist.T_stack.append(Tstack)
        hist.v_stack.append(vstack)
        hist.Tg_cmd.append(current_cmd.Tg_cmd_C)
        hist.vg_cmd.append(current_cmd.vg_cmd_mps)

        ctrl_elapsed += cfg.dt_meas_s
        opt_elapsed += cfg.dt_meas_s
        if ctrl_elapsed >= cfg.dt_ctrl_s - 1e-12:
            ctrl_elapsed = 0.0
        if opt_elapsed >= cfg.dt_opt_s - 1e-12:
            opt_elapsed = 0.0
        t += cfg.dt_meas_s

    return hist


def resample_to_dt(hist: OpenLoopHistory, dt_s: float) -> dict[str, np.ndarray]:
    t = np.asarray(hist.t)
    t_grid = np.arange(0.0, t[-1] + 1e-9, dt_s)
    out: dict[str, np.ndarray] = {'t': t_grid}
    for key in ['omega_req', 'omega_est', 'T_avg', 'omega_opt', 'omega_out', 'T_stack', 'v_stack', 'Tg_cmd', 'vg_cmd']:
        out[key] = np.interp(t_grid, t, np.asarray(getattr(hist, key), dtype=float))
    return out


def settle_value(signal: np.ndarray, start_idx: int, end_idx: int) -> float:
    lo = max(0, start_idx)
    hi = min(len(signal), end_idx)
    if hi <= lo:
        return float(signal[-1])
    return float(np.mean(signal[lo:hi]))


def compute_step_metrics(t: np.ndarray, u: np.ndarray, y: np.ndarray, t_step: float, settle_window_s: float = 300.0) -> StepMetrics:
    dt = float(t[1] - t[0])
    k_step = int(round(t_step / dt))
    n_settle = max(10, int(round(settle_window_s / dt)))
    y0 = settle_value(y, max(0, k_step - n_settle), k_step)
    yss = settle_value(y, len(y) - n_settle, len(y))
    u0 = settle_value(u, max(0, k_step - n_settle), k_step)
    uss = settle_value(u, len(u) - n_settle, len(u))
    delta_u = uss - u0
    delta_y = yss - y0
    gain = delta_y / delta_u if abs(delta_u) > 1e-12 else float('nan')

    thresholds = {
        'dead_time_s': 0.05,
        't50_s': 0.50,
        't63_s': 0.632,
        't90_s': 0.90,
        't95_s': 0.95,
    }

    times: dict[str, float | None] = {k: None for k in thresholds}
    if abs(delta_y) > 1e-9:
        for idx in range(k_step, len(y)):
            frac = (y[idx] - y0) / delta_y
            for name, level in thresholds.items():
                if times[name] is None and frac >= level:
                    times[name] = float(t[idx] - t_step)
    return StepMetrics(delta_u, y0, yss, delta_y, gain, **times)


def print_step_summary(label: str, metrics: StepMetrics) -> None:
    print(f'[{label}]')
    print(f'  delta_u      = {metrics.delta_u:+.6f}')
    print(f'  y0           = {metrics.y0:+.6f}')
    print(f'  yss          = {metrics.yss:+.6f}')
    print(f'  delta_y      = {metrics.delta_y:+.6f}')
    print(f'  gain         = {metrics.gain:+.6f}')
    print(f'  dead_time_s  = {metrics.dead_time_s}')
    print(f'  t50_s        = {metrics.t50_s}')
    print(f'  t63_s        = {metrics.t63_s}')
    print(f'  t90_s        = {metrics.t90_s}')
    print(f'  t95_s        = {metrics.t95_s}')


def build_multistep_schedule(base: float = 0.3218) -> tuple[Callable[[float], float], list[tuple[float, float]]]:
    # Each tuple is (time_s, absolute omega_req value)
    sequence = [
        (0.0, base),
        (300.0, base + 0.005),
        (1200.0, base),
        (2100.0, base - 0.005),
        (3000.0, base),
        (3900.0, base + 0.010),
        (5100.0, base),
        (6300.0, base - 0.010),
        (7500.0, base),
    ]

    def schedule(t: float) -> float:
        current = sequence[0][1]
        for ts, val in sequence:
            if t >= ts:
                current = val
            else:
                break
        return current

    return schedule, sequence


def identify_first_order_delay(u: np.ndarray, y: np.ndarray, max_delay_steps: int = 200) -> dict[str, float]:
    best: dict[str, float] | None = None
    n = len(y)
    for d in range(max_delay_steps + 1):
        rows = []
        target = []
        for k in range(d, n - 1):
            rows.append([y[k], u[k - d]])
            target.append(y[k + 1])
        X = np.asarray(rows, dtype=float)
        Y = np.asarray(target, dtype=float)
        theta, *_ = np.linalg.lstsq(X, Y, rcond=None)
        pred = X @ theta
        sse = float(np.sum((Y - pred) ** 2))
        sst = float(np.sum((Y - np.mean(Y)) ** 2))
        fit = 1.0 - sse / max(sst, 1e-12)
        cand = {'a': float(theta[0]), 'b': float(theta[1]), 'delay_steps': float(d), 'fit': fit, 'sse': sse}
        if best is None or cand['fit'] > best['fit']:
            best = cand
    assert best is not None
    return best


def identify_second_order_delay(u: np.ndarray, y: np.ndarray, max_delay_steps: int = 200) -> dict[str, float]:
    best: dict[str, float] | None = None
    n = len(y)
    for d in range(max_delay_steps + 1):
        rows = []
        target = []
        start = max(1, d)
        for k in range(start, n - 1):
            rows.append([y[k], y[k - 1], u[k - d]])
            target.append(y[k + 1])
        X = np.asarray(rows, dtype=float)
        Y = np.asarray(target, dtype=float)
        theta, *_ = np.linalg.lstsq(X, Y, rcond=None)
        pred = X @ theta
        sse = float(np.sum((Y - pred) ** 2))
        sst = float(np.sum((Y - np.mean(Y)) ** 2))
        fit = 1.0 - sse / max(sst, 1e-12)
        cand = {
            'a1': float(theta[0]),
            'a2': float(theta[1]),
            'b': float(theta[2]),
            'delay_steps': float(d),
            'fit': fit,
            'sse': sse,
        }
        if best is None or cand['fit'] > best['fit']:
            best = cand
    assert best is not None
    return best


def estimate_observer_gain_from_poles(a_poly_roots: Iterable[complex], speed_factor: float = 2.5) -> list[float]:
    scaled = []
    for p in a_poly_roots:
        magnitude = abs(p)
        angle = math.atan2(p.imag, p.real)
        faster_mag = max(min(magnitude ** speed_factor, 0.95), 0.05)
        scaled.append(complex(faster_mag * math.cos(angle), faster_mag * math.sin(angle)))
    coeffs = np.poly(scaled)
    coeffs = np.real_if_close(coeffs, tol=1000)
    return [float(c) for c in coeffs]


def save_csv(path: Path, data: dict[str, np.ndarray]) -> None:
    with path.open('w', encoding='utf-8', newline='') as f:
        writer = csv.writer(f)
        headers = list(data.keys())
        writer.writerow(headers)
        n = len(next(iter(data.values())))
        for i in range(n):
            writer.writerow([data[h][i] for h in headers])


def main() -> None:
    parser = argparse.ArgumentParser(description='Identify outer-loop model from omega_req to T_avg / omega_est using the existing FlameGuard simulation chain.')
    parser.add_argument('--no-lookup', action='store_true', help='Use online optimizer instead of lookup table.')
    parser.add_argument('--out-dir', type=str, default='tests/identify_outer_loop_results', help='Directory to write CSV results.')
    parser.add_argument('--step-size', type=float, default=0.01, help='Step size for single-step experiment around omega_ref.')
    parser.add_argument('--mode', choices=['step', 'multi', 'both'], default='both', help='Run only the single-step test, only the multi-step fit, or both.')
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    omega_ref = SimConfig().omega_ref

    if args.mode in ('step', 'both'):
        cfg_step = SimConfig(total_time_s=3000.0, use_lookup_table=(not args.no_lookup))
        omega_ref = cfg_step.omega_ref
        t_step = 300.0
        step_size = float(args.step_size)

        def step_schedule(t: float) -> float:
            return omega_ref if t < t_step else float(np.clip(omega_ref + step_size, 0.20, 0.60))

        print('Running single-step experiment...', flush=True)
        hist_step = run_open_loop_case(step_schedule, cfg=cfg_step)
        data_step = resample_to_dt(hist_step, 1.0)
        save_csv(out_dir / 'single_step_1s.csv', data_step)

        print('=== SINGLE-STEP OUTER-LOOP IDENTIFICATION ===', flush=True)
        print(f'omega_ref                = {omega_ref:.6f}', flush=True)
        print(f'step_time_s              = {t_step:.1f}', flush=True)
        print(f'step_size                = {step_size:+.6f}', flush=True)
        print(f'data_saved               = {out_dir / "single_step_1s.csv"}', flush=True)

        m_tavg = compute_step_metrics(data_step['t'], data_step['omega_req'], data_step['T_avg'], t_step)
        m_omega = compute_step_metrics(data_step['t'], data_step['omega_req'], data_step['omega_est'], t_step)
        m_omega_out = compute_step_metrics(data_step['t'], data_step['omega_req'], data_step['omega_out'], t_step)

        print_step_summary('T_avg_from_omega_req', m_tavg)
        print_step_summary('omega_est_from_omega_req', m_omega)
        print_step_summary('omega_out_from_omega_req', m_omega_out)

    if args.mode in ('multi', 'both'):
        cfg_multi = SimConfig(total_time_s=8400.0, use_lookup_table=(not args.no_lookup))
        multi_schedule, sequence = build_multistep_schedule(base=cfg_multi.omega_ref)
        print('Running multi-step experiment and fitting models...', flush=True)
        hist_multi = run_open_loop_case(multi_schedule, cfg=cfg_multi)
        data_multi = resample_to_dt(hist_multi, 1.0)
        save_csv(out_dir / 'multistep_1s.csv', data_multi)

        print('\n=== MULTI-STEP EXPERIMENT USED FOR MODEL FITTING ===', flush=True)
        for ts, val in sequence:
            print(f'  t={ts:7.1f} s -> omega_req={val:.6f}', flush=True)
        print(f'data_saved               = {out_dir / "multistep_1s.csv"}', flush=True)

        u = data_multi['omega_req'] - cfg_multi.omega_ref
        y_tavg = data_multi['T_avg'] - cfg_multi.T_set_C
        y_omega = data_multi['omega_est'] - cfg_multi.omega_ref

        model1_tavg = identify_first_order_delay(u, y_tavg, max_delay_steps=180)
        model1_omega = identify_first_order_delay(u, y_omega, max_delay_steps=180)
        model2_tavg = identify_second_order_delay(u, y_tavg, max_delay_steps=180)
        model2_omega = identify_second_order_delay(u, y_omega, max_delay_steps=180)

        print('\n=== DISCRETE-TIME FITS AT 1.0 s SAMPLE TIME ===', flush=True)
        print('Model form 1st order: y[k+1] = a*y[k] + b*u[k-d]', flush=True)
        print(f'T_avg   : a={model1_tavg["a"]:+.6f}, b={model1_tavg["b"]:+.6f}, d={int(model1_tavg["delay_steps"])} steps, fit={model1_tavg["fit"]:.4f}', flush=True)
        print(f'omega_f : a={model1_omega["a"]:+.6f}, b={model1_omega["b"]:+.6f}, d={int(model1_omega["delay_steps"])} steps, fit={model1_omega["fit"]:.4f}', flush=True)
        print('Model form 2nd order: y[k+1] = a1*y[k] + a2*y[k-1] + b*u[k-d]', flush=True)
        print(f'T_avg   : a1={model2_tavg["a1"]:+.6f}, a2={model2_tavg["a2"]:+.6f}, b={model2_tavg["b"]:+.6f}, d={int(model2_tavg["delay_steps"])} steps, fit={model2_tavg["fit"]:.4f}', flush=True)
        print(f'omega_f : a1={model2_omega["a1"]:+.6f}, a2={model2_omega["a2"]:+.6f}, b={model2_omega["b"]:+.6f}, d={int(model2_omega["delay_steps"])} steps, fit={model2_omega["fit"]:.4f}', flush=True)

        roots_tavg = np.roots([1.0, -model2_tavg['a1'], -model2_tavg['a2']])
        obs_poly = estimate_observer_gain_from_poles(roots_tavg, speed_factor=2.5)
        print('\n=== RECOMMENDED STARTING POINT FOR OUTER LQI DESIGN ===', flush=True)
        print('Suggested plant for T_avg deviation (pick the 2nd-order one first):', flush=True)
        print(f'  y[k+1] = {model2_tavg["a1"]:+.6f}*y[k] + {model2_tavg["a2"]:+.6f}*y[k-1] + {model2_tavg["b"]:+.6f}*du[k-{int(model2_tavg["delay_steps"])}]', flush=True)
        print('Suggested auxiliary plant for omega_est deviation:', flush=True)
        print(f'  w[k+1] = {model1_omega["a"]:+.6f}*w[k] + {model1_omega["b"]:+.6f}*du[k-{int(model1_omega["delay_steps"])}]', flush=True)
        print('Suggested observer characteristic polynomial coefficients (faster than plant, for reference):', flush=True)
        print(f'  {obs_poly}', flush=True)
        print('Paste the full terminal output back to me, and I can turn it into the first LQI controller.py.', flush=True)


if __name__ == '__main__':
    main()
