from __future__ import annotations

from dataclasses import dataclass
import math

from control_types import ControllerOutput, FurnaceObservation, GovernorDecision


@dataclass
class DynamicGovernorConfig:
    dt_governor_s: float = 1.0
    omega_ss_min: float = 0.3042675804697915
    omega_ss_max: float = 0.3393469511577442
    omega_dyn_min_cap: float = 0.20
    omega_dyn_max_cap: float = 0.60
    # dynamic output supervision based on measurable outputs
    Tavg_dyn_min_C: float = 845.0
    Tavg_dyn_max_C: float = 1105.0
    horizon_s: float = 600.0
    tau_eff_s: float = 1060.0  # preheater + furnace dominant response
    beta_dyn_smoothing: float = 0.80
    TAVG_A: float = -13.109632
    TAVG_B: float = 1294.871365


class DynamicGovernor:
    def __init__(self, cfg: DynamicGovernorConfig | None = None):
        self.cfg = cfg or DynamicGovernorConfig()
        self.prev_band = (self.cfg.omega_ss_min, self.cfg.omega_ss_max)

    def _project(self, value: float, band: tuple[float, float]) -> float:
        return min(max(value, band[0]), band[1])

    def _phi(self) -> float:
        c = self.cfg
        return 1.0 - math.exp(-c.horizon_s / max(c.tau_eff_s, 1e-9))

    def _tavg_band_from_measurement(self, obs: FurnaceObservation, omega_est: float) -> tuple[float, float]:
        c = self.cfg
        phi = max(self._phi(), 1e-6)
        k = 100.0 * c.TAVG_A * phi
        # y + k*(omega - omega_est) in [min,max], k<0
        lo1 = omega_est + (c.Tavg_dyn_max_C - obs.T_avg_C) / k
        hi1 = omega_est + (c.Tavg_dyn_min_C - obs.T_avg_C) / k
        lo = min(lo1, hi1)
        hi = max(lo1, hi1)
        lo = max(lo, c.omega_dyn_min_cap)
        hi = min(hi, c.omega_dyn_max_cap)
        if lo > hi:
            center = min(max(omega_est, c.omega_dyn_min_cap), c.omega_dyn_max_cap)
            return center, center
        return lo, hi

    def decide(self, obs: FurnaceObservation, ctrl: ControllerOutput) -> GovernorDecision:
        c = self.cfg
        tavg_lo, tavg_hi = self._tavg_band_from_measurement(obs, ctrl.omega_est)
        # Always include steady-state preferred band, but do not force it as hard constraint.
        raw_lo = max(c.omega_dyn_min_cap, min(tavg_lo, c.omega_ss_min))
        raw_hi = min(c.omega_dyn_max_cap, max(tavg_hi, c.omega_ss_max))
        beta = c.beta_dyn_smoothing
        band = (
            beta * self.prev_band[0] + (1.0 - beta) * raw_lo,
            beta * self.prev_band[1] + (1.0 - beta) * raw_hi,
        )
        self.prev_band = band
        omega_tar = self._project(ctrl.omega_req, band)
        phi = self._phi()
        Tavg_pred = obs.T_avg_C + 100.0 * c.TAVG_A * (omega_tar - ctrl.omega_est) * phi
        return GovernorDecision(
            time_s=ctrl.time_s,
            omega_est=ctrl.omega_est,
            omega_req=ctrl.omega_req,
            omega_tar=omega_tar,
            dyn_band=band,
            Tavg_pred_C=Tavg_pred,
            note="dynamic Tavg governor",
        )
