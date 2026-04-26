from __future__ import annotations

from dataclasses import dataclass

from control_types import ControllerOutput, FurnaceObservation, WeightedMoistureEstimate


@dataclass
class PIDConfig:
    dt_controller_s: float = 1.0
    T_set_C: float = 872.99

    # 先保留当前主参数，只在 I 项上做保守化
    kp: float = 0.008
    ki: float = 0.00005
    kd: float = 0.1

    antiwindup_min: float = -0.12
    antiwindup_max: float = 0.12

    mu1: float = 0.6
    mu2: float = 0.3
    mu3: float = 0.1

    omega_est_min: float = 0.05
    omega_est_max: float = 0.95
    omega_req_min: float = 0.20
    omega_req_max: float = 0.60
    max_delta_omega_req_per_step: float = 0.008

    # 保留原有输出低通
    req_filter_beta: float = 0.88

    # 新增：积分分离
    enable_integral_separation: bool = True
    integral_deadband_C: float = 0

    TAVG_A: float = -13.109632
    TAVG_B: float = 1294.871365
    TSTACK_A: float = -14.412237
    TSTACK_B: float = 1423.472316
    VSTACK_A: float = -0.215310
    VSTACK_B: float = 25.332842


class PIDController:
    def __init__(self, cfg: PIDConfig | None = None):
        self.cfg = cfg or PIDConfig()
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_omega_req = 0.5 * (self.cfg.omega_req_min + self.cfg.omega_req_max)
        self.req_filtered = self.prev_omega_req

    def _clip_omega(self, omega: float) -> float:
        return min(max(omega, self.cfg.omega_est_min), self.cfg.omega_est_max)

    def invert_outputs_to_moisture(self, obs: FurnaceObservation) -> WeightedMoistureEstimate:
        c = self.cfg
        # 静态回归的自变量是百分数含水率，因此反解后要除以 100
        w1 = (obs.T_avg_C - c.TAVG_B) / (100.0 * c.TAVG_A)
        w2 = (obs.T_stack_C - c.TSTACK_B) / (100.0 * c.TSTACK_A)
        w3 = (obs.v_stack_mps - c.VSTACK_B) / (100.0 * c.VSTACK_A)
        w1 = self._clip_omega(w1)
        w2 = self._clip_omega(w2)
        w3 = self._clip_omega(w3)
        fused = self._clip_omega(c.mu1 * w1 + c.mu2 * w2 + c.mu3 * w3)
        return WeightedMoistureEstimate(w1, w2, w3, fused)

    def _rate_limit(self, omega_req_raw: float) -> float:
        c = self.cfg
        step = c.max_delta_omega_req_per_step
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

        # 积分分离：误差进入小范围时暂停积分，减少慢对象低频 hunting
        integrate_enabled = True
        if c.enable_integral_separation and abs(error_C) <= c.integral_deadband_C:
            integrate_enabled = False

        if integrate_enabled:
            self.integral += error_C * c.dt_controller_s

        # anti-windup 仍然保留
        i_min = c.antiwindup_min / max(c.ki, 1e-12)
        i_max = c.antiwindup_max / max(c.ki, 1e-12)
        self.integral = min(max(self.integral, i_min), i_max)

        derivative = (error_C - self.prev_error) / max(c.dt_controller_s, 1e-9)
        self.prev_error = error_C

        delta_omega = c.kp * error_C + c.ki * self.integral + c.kd * derivative

        # 温度偏低 -> 需要更干 -> omega_req 下降
        omega_req_raw = min(max(omega_est - delta_omega, c.omega_req_min), c.omega_req_max)
        omega_req_limited = self._rate_limit(omega_req_raw)

        beta = c.req_filter_beta
        self.req_filtered = beta * self.req_filtered + (1.0 - beta) * omega_req_limited
        omega_req = min(max(self.req_filtered, c.omega_req_min), c.omega_req_max)

        return ControllerOutput(
            time_s=obs.time_s,
            omega_est=omega_est,
            omega_req=omega_req,
            error_C=error_C,
            T_set_C=c.T_set_C,
        )
