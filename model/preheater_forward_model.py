from __future__ import annotations

from dataclasses import dataclass
import copy
import math
from typing import Tuple
import numpy as np

from model.material_model import composition_to_equivalent_properties
from model.control_types import FeedObservation, PreheaterCellState, PreheaterState
from model.model_types import Config, EquivalentProperties, ResourceBoundary
from model.thermal_core import ceq_from_props, rho_g, power_kW, mdot_preheater, mdot_stack_cap


@dataclass
class PreheaterForwardConfig:
    n_cells: int = 20
    length_m: float = 3.2
    tau_residence_s: float = 985.0
    feed_delay_s: float = 5.0
    omega_min: float = 0.20
    omega_max: float = 0.98
    T_amb_C: float = 20.0
    T_evap_C: float = 100.0
    T_solid_max_C: float = 250.0
    opt_cfg: Config | None = None

    # Transfer model tuning. Values preserve the original U(v)*A law but add a
    # modest calibration factor for distributed-cell numerical stability.
    heat_transfer_factor: float = 1.0
    min_tau20_min: float = 1.0


class PreheaterForwardModel:
    """Distributed preheater digital twin.

    The model is deliberately low-order: plug-flow-like axial transport with
    numerical mixing, gas-side heat transfer, sensible heating, and drying limited
    by both available energy and the experimental tau20 kinetics.
    """

    def __init__(self, cfg: PreheaterForwardConfig | None = None):
        self.cfg = cfg or PreheaterForwardConfig()
        self.opt_cfg = self.cfg.opt_cfg or Config()
        self.time_s = 0.0
        n = self.cfg.n_cells
        self.omega = np.full(n, 0.3218, dtype=float)
        self.T_solid = np.full(n, 120.0, dtype=float)
        self.omega0 = np.full(n, 0.65, dtype=float)
        self.tref = np.full(n, 15.0, dtype=float)
        self.slope = np.full(n, -0.21, dtype=float)
        self._last_Tg_profile = tuple([175.0] * (n + 1))
        # Time-based feed delay buffer: list of (arrival_time_s, equivalent_properties).
        # This replaces the older cell-bucket delay, which tied a 5 s feed delay
        # to a ~49 s axial cell residence bucket.
        self._feed_delay: list[tuple[float, EquivalentProperties]] = []
        self._feed_delay_time_s = 0.0

    def clone(self) -> "PreheaterForwardModel":
        return copy.deepcopy(self)

    @property
    def tau_cell_s(self) -> float:
        return self.cfg.tau_residence_s / max(self.cfg.n_cells, 1)

    @property
    def cell_mass_kg(self) -> float:
        return self.opt_cfg.M_H / max(self.cfg.n_cells, 1)

    @property
    def cell_area_m2(self) -> float:
        return self.opt_cfg.A / max(self.cfg.n_cells, 1)

    def initialize(
        self,
        composition,
        *,
        normalize: bool = False,
        omega_init: float | None = None,
        T_solid_init_C: float = 120.0,
        time_s: float = 0.0,
    ) -> None:
        eq = composition_to_equivalent_properties(composition, normalize=normalize).equivalent
        self.time_s = float(time_s)
        self.omega0[:] = eq.omega0
        self.tref[:] = eq.tref_min
        self.slope[:] = eq.slope_min_per_c
        self.omega[:] = float(eq.omega0 if omega_init is None else omega_init)
        self.omega[:] = np.clip(self.omega, self.cfg.omega_min, self.cfg.omega_max)
        self.T_solid[:] = float(T_solid_init_C)
        self._feed_delay = [
            (self.time_s - max(self.cfg.feed_delay_s, 0.0) - 1e-6, eq),
            (self.time_s, eq),
        ]
        self._feed_delay_time_s = 0.0

    def _equivalent_from_feed(self, feed: FeedObservation) -> EquivalentProperties:
        return composition_to_equivalent_properties(feed.composition, normalize=feed.normalize).equivalent

    @staticmethod
    def _interp_equivalent(a: EquivalentProperties, b: EquivalentProperties, w: float) -> EquivalentProperties:
        w = min(max(float(w), 0.0), 1.0)
        return EquivalentProperties(
            omega0=(1.0 - w) * a.omega0 + w * b.omega0,
            tref_min=(1.0 - w) * a.tref_min + w * b.tref_min,
            slope_min_per_c=(1.0 - w) * a.slope_min_per_c + w * b.slope_min_per_c,
        )

    def _delayed_feed_props(self, feed: FeedObservation, dt_s: float) -> EquivalentProperties:
        # Feed delay is now represented in real time, independent of axial-cell
        # residence time and independent of the controller/MPC update rate.
        eq = self._equivalent_from_feed(feed)
        now = float(feed.time_s)
        if not self._feed_delay:
            self._feed_delay = [(now - max(self.cfg.feed_delay_s, 0.0) - 1e-6, eq), (now, eq)]
        else:
            if now >= self._feed_delay[-1][0] - 1e-12:
                self._feed_delay.append((now, eq))
            else:
                # Warm-start or cloned rollouts should be monotone; if not, reset safely.
                self._feed_delay = [(now - max(self.cfg.feed_delay_s, 0.0) - 1e-6, eq), (now, eq)]

        target_t = now - max(self.cfg.feed_delay_s, 0.0)
        # Drop points far older than the interpolation target but keep one guard.
        while len(self._feed_delay) > 2 and self._feed_delay[1][0] <= target_t:
            self._feed_delay.pop(0)

        if target_t <= self._feed_delay[0][0]:
            return self._feed_delay[0][1]
        for (t0, e0), (t1, e1) in zip(self._feed_delay[:-1], self._feed_delay[1:]):
            if t0 <= target_t <= t1:
                if abs(t1 - t0) < 1e-12:
                    return e1
                return self._interp_equivalent(e0, e1, (target_t - t0) / (t1 - t0))
        return self._feed_delay[-1][1]

    def _advect(self, feed_props: EquivalentProperties, dt_s: float) -> None:
        gamma = min(max(dt_s / max(self.tau_cell_s, 1e-9), 0.0), 1.0)
        # State variables: upstream material moves to downstream cells.
        old = (self.omega.copy(), self.T_solid.copy(), self.omega0.copy(), self.tref.copy(), self.slope.copy())
        omega_old, T_old, omega0_old, tref_old, slope_old = old
        for i in range(self.cfg.n_cells - 1, 0, -1):
            self.omega[i] = (1.0 - gamma) * omega_old[i] + gamma * omega_old[i - 1]
            self.T_solid[i] = (1.0 - gamma) * T_old[i] + gamma * T_old[i - 1]
            self.omega0[i] = (1.0 - gamma) * omega0_old[i] + gamma * omega0_old[i - 1]
            self.tref[i] = (1.0 - gamma) * tref_old[i] + gamma * tref_old[i - 1]
            self.slope[i] = (1.0 - gamma) * slope_old[i] + gamma * slope_old[i - 1]
        self.omega[0] = (1.0 - gamma) * omega_old[0] + gamma * feed_props.omega0
        self.T_solid[0] = (1.0 - gamma) * T_old[0] + gamma * self.cfg.T_amb_C
        self.omega0[0] = (1.0 - gamma) * omega0_old[0] + gamma * feed_props.omega0
        self.tref[0] = (1.0 - gamma) * tref_old[0] + gamma * feed_props.tref_min
        self.slope[0] = (1.0 - gamma) * slope_old[0] + gamma * feed_props.slope_min_per_c

    def _cell_props(self, i: int) -> EquivalentProperties:
        return EquivalentProperties(float(self.omega0[i]), float(self.tref[i]), float(self.slope[i]))

    def _drying_kinetic_limit(self, i: int, dt_s: float) -> float:
        omega = float(self.omega[i])
        if omega <= self.cfg.omega_min:
            return 0.0
        props = self._cell_props(i)
        tau20_min = props.tref_min + props.slope_min_per_c * (float(self.T_solid[i]) - self.opt_cfg.T_REF)
        tau20_s = max(self.cfg.min_tau20_min, tau20_min) * 60.0
        return max(0.0, (omega - self.cfg.omega_min) * dt_s / tau20_s)

    def _apply_heat_and_drying(self, Tg_in_C: float, vg_mps: float, dt_s: float) -> None:
        n = self.cfg.n_cells
        Tg_profile = [float(Tg_in_C)]
        mdot_g = max(rho_g(Tg_in_C, self.opt_cfg) * self.opt_cfg.A_D * max(vg_mps, 0.0), 1e-9)
        U = self.opt_cfg.U0 + self.opt_cfg.K_U * (max(vg_mps, 0.0) ** self.opt_cfg.N_U)
        mass = max(self.cell_mass_kg, 1e-9)
        area = self.cell_area_m2
        for i in range(n):
            Tg_cell = Tg_profile[-1]
            deltaT = max(Tg_cell - float(self.T_solid[i]), 0.0)
            Q_kW = self.cfg.heat_transfer_factor * U * area * deltaT / 1000.0
            E_kJ = max(Q_kW * dt_s, 0.0)
            props = self._cell_props(i)
            ceq = ceq_from_props(props, self.opt_cfg)
            cap_kJ_per_K = max(mass * ceq, 1e-9)

            # sensible heating toward evaporation temperature first
            if self.T_solid[i] < self.cfg.T_evap_C:
                need = (self.cfg.T_evap_C - self.T_solid[i]) * cap_kJ_per_K
                used = min(E_kJ, need)
                self.T_solid[i] += used / cap_kJ_per_K
                E_kJ -= used

            # above evaporation threshold, split heat into evaporation and modest further heating
            if E_kJ > 0.0 and self.omega[i] > self.cfg.omega_min:
                # use most high-temperature heat for evaporation; remaining heat raises solid temperature slowly
                E_evap = 0.85 * E_kJ if self.T_solid[i] >= self.cfg.T_evap_C else 0.0
                dm_evap = E_evap / max(self.opt_cfg.LAMBDA, 1e-9)
                d_omega_energy = dm_evap * max(1.0 - self.omega[i], 0.05) / mass
                d_omega = min(d_omega_energy, self._drying_kinetic_limit(i, dt_s), self.omega[i] - self.cfg.omega_min)
                self.omega[i] -= max(d_omega, 0.0)
                E_left = E_kJ - E_evap
                self.T_solid[i] += max(E_left, 0.0) / cap_kJ_per_K
            elif E_kJ > 0.0:
                self.T_solid[i] += E_kJ / cap_kJ_per_K

            self.T_solid[i] = float(np.clip(self.T_solid[i], self.cfg.T_amb_C, self.cfg.T_solid_max_C))
            self.omega[i] = float(np.clip(self.omega[i], self.cfg.omega_min, self.cfg.omega_max))
            # Gas cooling by transferred heat, capped to avoid nonphysical inversion.
            dTg = Q_kW / max(mdot_g * self.opt_cfg.CPG, 1e-9)
            Tg_next = max(self.T_solid[i] + 1e-3, Tg_cell - dTg)
            Tg_profile.append(float(Tg_next))
        self._last_Tg_profile = tuple(Tg_profile)

    def step(self, feed: FeedObservation, Tg_in_C: float, vg_mps: float, dt_s: float) -> PreheaterState:
        feed_props = self._delayed_feed_props(feed, dt_s)
        self._advect(feed_props, dt_s)
        self._apply_heat_and_drying(Tg_in_C, vg_mps, dt_s)
        self.time_s = float(feed.time_s)
        return self.state(time_s=feed.time_s)

    def state(self, *, time_s: float | None = None) -> PreheaterState:
        n = self.cfg.n_cells
        dz = self.cfg.length_m / max(n, 1)
        cells = []
        for i in range(n):
            remaining = max(0.0, (n - 1 - i + 0.5) * self.tau_cell_s)
            cells.append(PreheaterCellState(
                index=i,
                z_center_m=(i + 0.5) * dz,
                residence_left_s=remaining,
                omega=float(self.omega[i]),
                T_solid_C=float(self.T_solid[i]),
                omega0=float(self.omega0[i]),
                tref_min=float(self.tref[i]),
                slope_min_per_c=float(self.slope[i]),
            ))
        return PreheaterState(
            time_s=float(self.time_s if time_s is None else time_s),
            cells=tuple(cells),
            omega_out=float(self.omega[-1]),
            T_solid_out_C=float(self.T_solid[-1]),
            Tg_profile_C=tuple(self._last_Tg_profile),
        )

    def representative_cell(self, target_remaining_s: float) -> PreheaterCellState:
        st = self.state()
        return min(st.cells, key=lambda c: abs(c.residence_left_s - target_remaining_s))

    def rollout_constant(self, feed: FeedObservation, Tg_C: float, vg_mps: float, horizon_s: float, dt_s: float):
        states = []
        t0 = self.time_s
        steps = max(1, int(math.ceil(horizon_s / max(dt_s, 1e-9))))
        model = self.clone()
        for k in range(steps):
            f = FeedObservation(time_s=t0 + (k + 1) * dt_s, composition=feed.composition, normalize=feed.normalize)
            states.append(model.step(f, Tg_C, vg_mps, dt_s))
        return states
