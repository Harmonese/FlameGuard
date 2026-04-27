from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Deque
import copy
import numpy as np


def furnace_outputs_from_omega(omega: float):
    return (
        -13.109632 * (100.0 * omega) + 1294.871365,
        -14.412237 * (100.0 * omega) + 1423.472316,
        -0.215310 * (100.0 * omega) + 25.332842,
    )


@dataclass
class FurnaceDynConfig:
    omega_ref: float = 0.3218
    dead_s: float = 5.0
    tau1_s: float = 0.223
    tau2_s: float = 75.412
    dt_s: float = 0.1


class FurnaceDyn:
    """Two-lag + dead-time furnace dynamic proxy.

    The physical dead time is specified in seconds. The plant normally advances
    with cfg.dt_s, but MPC rollouts may use a much coarser dt_s. In that case we
    must not reuse the plant queue length, otherwise a 5 s delay can be inflated
    to hundreds of seconds in prediction.
    """

    def __init__(self, cfg: FurnaceDynConfig | None = None):
        self.cfg = cfg or FurnaceDynConfig()
        self.delay_steps = max(1, round(self.cfg.dead_s / self.cfg.dt_s))
        self.queue: Deque[float] = deque([self.cfg.omega_ref] * self.delay_steps, maxlen=self.delay_steps)
        t_ref, ts_ref, vs_ref = furnace_outputs_from_omega(self.cfg.omega_ref)
        self.refs = {'T_avg': t_ref, 'T_stack': ts_ref, 'v_stack': vs_ref}
        self.states = {'T_avg': (0.0, 0.0), 'T_stack': (0.0, 0.0), 'v_stack': (0.0, 0.0)}
        self.gains = {'T_avg': -13.109632, 'T_stack': -14.412237, 'v_stack': -0.215310}

    def clone(self) -> "FurnaceDyn":
        return copy.deepcopy(self)

    def initialize_outputs(self, T_avg_init_C: float, T_stack_init_C: float, v_stack_init_mps: float):
        targets = {'T_avg': float(T_avg_init_C), 'T_stack': float(T_stack_init_C), 'v_stack': float(v_stack_init_mps)}
        for key in ['T_avg', 'T_stack', 'v_stack']:
            delta = targets[key] - self.refs[key]
            self.states[key] = (delta, delta)

    def _delayed_omega(self, omega_in: float, dt: float) -> float:
        """Return delayed moisture for this integration step.

        For plant integration (dt == cfg.dt_s), use the persistent queue. For
        coarse MPC rollout steps, approximate the physical dead time in seconds,
        not in the plant number of 0.1 s samples.
        """
        omega_in = float(omega_in)
        if abs(dt - self.cfg.dt_s) <= 1e-12:
            self.queue.append(omega_in)
            return float(self.queue[0])

        # Coarse MPC step longer than dead time: the dead-time transient happens
        # inside this single prediction interval, so use the current input.
        if dt >= self.cfg.dead_s:
            return omega_in

        # Intermediate case: create a local queue of the proper coarse length.
        desired_len = max(2, int(np.ceil(self.cfg.dead_s / max(dt, 1e-9))) + 1)
        recent = list(self.queue)[-desired_len:] if self.queue else [self.cfg.omega_ref]
        while len(recent) < desired_len:
            recent.insert(0, recent[0])
        local_queue: Deque[float] = deque(recent, maxlen=desired_len)
        local_queue.append(omega_in)
        omega_d = float(local_queue[0])

        # Preserve a consistent recent history for possible subsequent calls.
        preserved = list(local_queue)[-self.delay_steps:]
        if len(preserved) < self.delay_steps:
            preserved = [preserved[0]] * (self.delay_steps - len(preserved)) + preserved
        self.queue = deque(preserved, maxlen=self.delay_steps)
        return omega_d

    def step(self, omega_in: float, *, dt_s: float | None = None, disturbance=None):
        dt = float(dt_s if dt_s is not None else self.cfg.dt_s)
        omega_d = self._delayed_omega(float(omega_in), dt)
        outputs = {}
        delta_w_pct = 100.0 * (omega_d - self.cfg.omega_ref)
        for key in ['T_avg', 'T_stack', 'v_stack']:
            x1, x2 = self.states[key]
            u = self.gains[key] * delta_w_pct
            a1 = np.exp(-dt / max(self.cfg.tau1_s, 1e-9))
            x1 = a1 * x1 + (1.0 - a1) * u
            a2 = np.exp(-dt / max(self.cfg.tau2_s, 1e-9))
            x2 = a2 * x2 + (1.0 - a2) * x1
            self.states[key] = (x1, x2)
            outputs[key] = self.refs[key] + x2
        if disturbance is None:
            d_avg, d_stack, d_v = 0.0, 0.0, 0.0
        elif isinstance(disturbance, (tuple, list)) and len(disturbance) == 3:
            d_avg, d_stack, d_v = float(disturbance[0]), float(disturbance[1]), float(disturbance[2])
        else:
            d_avg, d_stack, d_v = float(disturbance), 0.0, 0.0
        return outputs['T_avg'] + d_avg, outputs['T_stack'] + d_stack, outputs['v_stack'] + d_v
