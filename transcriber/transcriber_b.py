from __future__ import annotations

from dataclasses import dataclass

from control_types import ActuatorCommand, OptimizerResponseLite
from optimizer.optimizer import Config, mdot_preheater, rho_g


@dataclass
class TranscriberBConfig:
    Tg_min_C: float = 100.0
    vg_min_mps: float = 3.0
    Tg_max_C: float = 2000.0
    vg_max_mps: float = 12.0
    cp_g_kJ_per_kgK: float = 1.05
    T_amb_C: float = 20.0
    max_delta_Tg_cmd_C: float = 40.0
    max_delta_vg_cmd_mps: float = 0.8


class TranscriberB:
    def __init__(self, cfg: TranscriberBConfig | None = None, opt_cfg: Config | None = None):
        self.cfg = cfg or TranscriberBConfig()
        self.opt_cfg = opt_cfg or Config()
        self._prev_Tg_cmd = 175.0
        self._prev_vg_cmd = 6.0

    @staticmethod
    def _rate_limit(target: float, prev: float, max_step: float) -> float:
        return min(max(target, prev - max_step), prev + max_step)

    def translate(self, result, *, time_s: float) -> tuple[OptimizerResponseLite, ActuatorCommand]:
        lite = OptimizerResponseLite(
            time_s=time_s,
            Tg_star_C=result.Tg_star_C,
            vg_star_mps=result.vg_star_mps,
            Tm_star_C=result.Tm_star_C,
            omega_opt=result.omega_opt,
            power_kW=result.power_kW,
            success=result.success,
            diagnostics={
                'Qreq_kW': result.Qreq_kW,
                'Qsup_kW': result.Qsup_kW,
                'mdot_stack_cap_kgps': result.mdot_stack_cap_kgps,
                'mdot_preheater_kgps': result.mdot_preheater_kgps,
                'omega_tar_projected': result.omega_tar_projected,
            },
        )
        Tg_cmd = self._rate_limit(result.Tg_star_C, self._prev_Tg_cmd, self.cfg.max_delta_Tg_cmd_C)
        vg_cmd = self._rate_limit(result.vg_star_mps, self._prev_vg_cmd, self.cfg.max_delta_vg_cmd_mps)
        resource_limited = False
        Tg_limited = False
        vg_limited = False

        if Tg_cmd < self.cfg.Tg_min_C:
            Tg_cmd = self.cfg.Tg_min_C
            resource_limited = True
            Tg_limited = True
        if Tg_cmd > self.cfg.Tg_max_C:
            Tg_cmd = self.cfg.Tg_max_C
            resource_limited = True
            Tg_limited = True
        if vg_cmd < self.cfg.vg_min_mps:
            vg_cmd = self.cfg.vg_min_mps
            resource_limited = True
            vg_limited = True
        if vg_cmd > self.cfg.vg_max_mps:
            vg_cmd = self.cfg.vg_max_mps
            resource_limited = True
            vg_limited = True

        mdot_need = mdot_preheater(Tg_cmd, vg_cmd, self.opt_cfg)
        mdot_cap = result.mdot_stack_cap_kgps
        rho = rho_g(Tg_cmd, self.opt_cfg)
        if mdot_need > mdot_cap + 1e-9:
            resource_limited = True
            vg_limited = True
            vg_cap = mdot_cap / max(rho * self.opt_cfg.A_D, 1e-9)
            vg_cmd = min(vg_cmd, vg_cap)
            vg_cmd = min(max(vg_cmd, self.cfg.vg_min_mps), self.cfg.vg_max_mps)

        mdot_actual = mdot_preheater(Tg_cmd, vg_cmd, self.opt_cfg)
        Q_available = mdot_actual * self.cfg.cp_g_kJ_per_kgK * max(Tg_cmd - self.cfg.T_amb_C, 0.0)
        deficit = max(result.Qreq_kW - Q_available, 0.0)
        heater_enable = deficit > 1e-6
        cmd = ActuatorCommand(
            time_s=time_s,
            Tg_cmd_C=Tg_cmd,
            vg_cmd_mps=vg_cmd,
            heater_enable=heater_enable,
            Q_heat_deficit_kW=deficit,
            resource_limited=resource_limited,
            Tg_limited_by_stack=Tg_limited,
            vg_limited_by_stack=vg_limited,
        )
        self._prev_Tg_cmd = Tg_cmd
        self._prev_vg_cmd = vg_cmd
        return lite, cmd
