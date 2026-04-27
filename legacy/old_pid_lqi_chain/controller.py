
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Deque, Tuple

import numpy as np

from control_types import ControllerOutput, FurnaceObservation, WeightedMoistureEstimate


@dataclass
class PIDConfig:
    dt_controller_s: float = 1.0
    T_set_C: float = 872.99
    omega_ref: float = 0.3218

    mu1: float = 0.6
    mu2: float = 0.3
    mu3: float = 0.1

    omega_est_min: float = 0.05
    omega_est_max: float = 0.95
    omega_req_min: float = 0.20
    omega_req_max: float = 0.60

    # Dual-mode output shaping
    max_delta_omega_req_per_step_normal: float = 0.004
    max_delta_omega_req_per_step_fast: float = 0.008
    max_delta_omega_req_per_step_fast_after_hold: float = 0.010
    req_filter_beta_normal: float = 0.95
    req_filter_beta_fast: float = 0.88
    fast_mode_max_hold_s: float = 60.0

    # Emergency mode logic
    Tavg_fast_enter_low_C: float = 850.0
    Tavg_fast_enter_high_C: float = 1100.0
    Tavg_fast_exit_low_C: float = 860.0
    Tavg_fast_exit_high_C: float = 900.0
    fast_exit_hold_s: float = 60.0

    # Integral handling
    enable_integral_separation: bool = True
    integral_deadband_C: float = 10.0
    integral_state_min: float = -4000.0
    integral_state_max: float = 4000.0

    # Identified outer-loop model (Ts = 1 s)
    A1: float = 1.991110
    A2: float = -0.991119
    B_E: float = 0.010255
    B_W: float = 0.000008
    input_delay_steps: int = 11

    # Luenberger observer poles ~ [0.95, 0.95]
    observer_l1: float = 0.09111
    observer_l2: float = 0.0894130775416474

    # Normal-mode LQI gain, designed on normalized command v in [-1, 1]
    # State order: [e_k, e_{k-1}, w_k, w_{k-1}, z_k, q0..q10]
    K_normal: Tuple[float, ...] = (
        8.52688590, -8.44825753,
        2.11166618e-09, -2.09156871e-09,
        9.99656054e-05,
        8.71114654e-02, 8.71417659e-02, 8.71720382e-02,
        8.72022822e-02, 8.72324979e-02, 8.72626852e-02,
        8.72928440e-02, 8.73229743e-02, 8.73530760e-02,
        8.73831490e-02, 8.74131933e-02,
    )

    # Fast-recovery mode gain on normalized command v in [-1, 1]
    K_fast: Tuple[float, ...] = (
        1.65714564e+01, -1.64040989e+01,
        1.08851255e-08, -1.07438032e-08,
        3.15830466e-04,
        1.67634880e-01, 1.67845169e-01, 1.68055318e-01,
        1.68265327e-01, 1.68475196e-01, 1.68684924e-01,
        1.68894511e-01, 1.69103956e-01, 1.69313260e-01,
        1.69522422e-01, 1.69731441e-01,
    )

    TAVG_A: float = -13.109632
    TAVG_B: float = 1294.871365
    TSTACK_A: float = -14.412237
    TSTACK_B: float = 1423.472316
    VSTACK_A: float = -0.215310
    VSTACK_B: float = 25.332842


class PIDController:
    """
    Backward-compatible outer controller interface.

    Despite the historical class name, this controller implements a dual-mode
    low-order optimal feedback law:
      - normal mode: smooth LQI-like feedback
      - fast mode: stronger LQI-like feedback once T_avg leaves [850, 1100] C

    It keeps the original output contract:
      FurnaceObservation -> ControllerOutput(omega_est, omega_req, ...)
    so governor/transcriber/optimizer do not need to change.
    """

    def __init__(self, cfg: PIDConfig | None = None):
        self.cfg = cfg or PIDConfig()

        # observer states: [e_k, e_{k-1}, w_k, w_{k-1}]
        self.xhat = np.zeros(4, dtype=float)

        # integral of temperature error
        self.integral = 0.0

        # command history for rate limiting and delay handling
        self.prev_omega_req = float(self.cfg.omega_ref)
        self.req_filtered = float(self.cfg.omega_ref)

        # q0..q10 = realized delta-omega-request command history
        self.delay_line: Deque[float] = deque([0.0] * self.cfg.input_delay_steps, maxlen=self.cfg.input_delay_steps)

        # mode management
        self.fast_mode = False
        self.fast_mode_elapsed_s = 0.0
        self.fast_exit_hold_elapsed_s = 0.0

    def _clip_omega(self, omega: float) -> float:
        return min(max(omega, self.cfg.omega_est_min), self.cfg.omega_est_max)

    def invert_outputs_to_moisture(self, obs: FurnaceObservation) -> WeightedMoistureEstimate:
        c = self.cfg
        # Static regression inverse uses percent moisture in the proxy models
        w1 = (obs.T_avg_C - c.TAVG_B) / (100.0 * c.TAVG_A)
        w2 = (obs.T_stack_C - c.TSTACK_B) / (100.0 * c.TSTACK_A)
        w3 = (obs.v_stack_mps - c.VSTACK_B) / (100.0 * c.VSTACK_A)
        w1 = self._clip_omega(w1)
        w2 = self._clip_omega(w2)
        w3 = self._clip_omega(w3)
        fused = self._clip_omega(c.mu1 * w1 + c.mu2 * w2 + c.mu3 * w3)
        return WeightedMoistureEstimate(w1, w2, w3, fused)

    def _select_mode(self, obs: FurnaceObservation) -> None:
        c = self.cfg
        if obs.T_avg_C < c.Tavg_fast_enter_low_C or obs.T_avg_C > c.Tavg_fast_enter_high_C:
            self.fast_mode = True
            self.fast_mode_elapsed_s = 0.0
            self.fast_exit_hold_elapsed_s = 0.0
            return

        if self.fast_mode:
            in_exit_band = c.Tavg_fast_exit_low_C <= obs.T_avg_C <= c.Tavg_fast_exit_high_C
            if in_exit_band:
                self.fast_exit_hold_elapsed_s += c.dt_controller_s
            else:
                self.fast_exit_hold_elapsed_s = 0.0

            self.fast_mode_elapsed_s += c.dt_controller_s
            if self.fast_exit_hold_elapsed_s >= c.fast_exit_hold_s:
                self.fast_mode = False
                self.fast_mode_elapsed_s = 0.0
                self.fast_exit_hold_elapsed_s = 0.0

    def _observer_predict(self) -> None:
        c = self.cfg
        u_del = self.delay_line[-1] if len(self.delay_line) > 0 else 0.0

        e1, e2, w1, w2 = self.xhat
        self.xhat[0] = c.A1 * e1 + c.A2 * e2 + c.B_E * u_del
        self.xhat[1] = e1
        self.xhat[2] = c.A1 * w1 + c.A2 * w2 + c.B_W * u_del
        self.xhat[3] = w1

    def _observer_correct(self, error_C: float, omega_dev: float) -> None:
        c = self.cfg
        ye = error_C - self.xhat[0]
        yw = omega_dev - self.xhat[2]

        # Two independent 2nd-order Luenberger corrections
        self.xhat[0] += c.observer_l1 * ye
        self.xhat[1] += c.observer_l2 * ye
        self.xhat[2] += c.observer_l1 * yw
        self.xhat[3] += c.observer_l2 * yw

    def _update_integral(self, error_C: float) -> None:
        c = self.cfg
        integrate_enabled = True
        if c.enable_integral_separation and abs(error_C) <= c.integral_deadband_C:
            integrate_enabled = False
        if integrate_enabled:
            self.integral += error_C * c.dt_controller_s
        self.integral = min(max(self.integral, c.integral_state_min), c.integral_state_max)

    def _current_gain(self) -> np.ndarray:
        return np.asarray(self.cfg.K_fast if self.fast_mode else self.cfg.K_normal, dtype=float)

    def _current_step_limit(self) -> float:
        c = self.cfg
        if self.fast_mode:
            if self.fast_mode_elapsed_s <= c.fast_mode_max_hold_s:
                return c.max_delta_omega_req_per_step_fast
            return c.max_delta_omega_req_per_step_fast_after_hold
        return c.max_delta_omega_req_per_step_normal

    def _current_filter_beta(self) -> float:
        return self.cfg.req_filter_beta_fast if self.fast_mode else self.cfg.req_filter_beta_normal

    def _rate_limit(self, omega_req_raw: float) -> float:
        step = self._current_step_limit()
        lo = self.prev_omega_req - step
        hi = self.prev_omega_req + step
        limited = min(max(omega_req_raw, lo), hi)
        self.prev_omega_req = limited
        return limited

    def step(self, obs: FurnaceObservation) -> ControllerOutput:
        c = self.cfg
        moist = self.invert_outputs_to_moisture(obs)
        omega_est = moist.omega_fused

        error_C = c.T_set_C - obs.T_avg_C
        omega_dev = omega_est - c.omega_ref

        # Mode selection uses the raw measurable output
        self._select_mode(obs)

        # Predict + correct observer
        self._observer_predict()
        self._observer_correct(error_C, omega_dev)

        # Integral on main objective only
        self._update_integral(error_C)

        # Augmented state for optimal feedback
        x_aug = np.concatenate([
            self.xhat,
            np.asarray([self.integral], dtype=float),
            np.asarray(self.delay_line, dtype=float),
        ])

        K = self._current_gain()
        v = -float(K @ x_aug)

        # Normalized command v -> physical delta omega request
        v = min(max(v, -1.0), 1.0)
        prev_filtered = self.req_filtered
        du_limit = self._current_step_limit()
        delta_omega = du_limit * v

        # Temperature too cold => error_C > 0 => delta_omega should be negative (drier)
        omega_req_raw = min(max(prev_filtered + delta_omega, c.omega_req_min), c.omega_req_max)
        omega_req_limited = self._rate_limit(omega_req_raw)

        beta = self._current_filter_beta()
        self.req_filtered = beta * prev_filtered + (1.0 - beta) * omega_req_limited
        omega_req = min(max(self.req_filtered, c.omega_req_min), c.omega_req_max)

        # Realized increment seen by the identified outer-loop model
        du_realized = omega_req - prev_filtered
        self.delay_line.appendleft(du_realized)

        return ControllerOutput(
            time_s=obs.time_s,
            omega_est=omega_est,
            omega_req=omega_req,
            error_C=error_C,
            T_set_C=c.T_set_C,
        )
