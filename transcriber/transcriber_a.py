from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Deque, Sequence
import math

from cleanser.cleanser import composition_to_equivalent_properties
from control_types import FeedObservation, GovernorDecision, SlotState
from optimizer.optimizer import DynamicTargetBand, EquivalentProperties, OptimizerRequest, ResourceBoundary


@dataclass
class TranscriberAConfig:
    dt_update_s: float = 2.0
    feed_delay_s: float = 5.0
    tau_mix_s: float = 985.0
    resource_T_stack_default_C: float = 930.0
    resource_v_stack_default_mps: float = 18.0
    burn_policy: str = "advisory"


class TranscriberA:
    def __init__(self, cfg: TranscriberAConfig | None = None):
        self.cfg = cfg or TranscriberAConfig()
        self.delay_steps = max(1, round(self.cfg.feed_delay_s / self.cfg.dt_update_s))
        self.queue: Deque[EquivalentProperties] = deque(maxlen=self.delay_steps)
        self.slot_state = SlotState(time_s=0.0, omega0=0.40, tref_min=15.0, slope_min_per_c=-0.19)

    def initialize(self, composition: Sequence[float], *, normalize: bool = False) -> None:
        eq = composition_to_equivalent_properties(composition, normalize=normalize).equivalent
        for _ in range(self.delay_steps):
            self.queue.append(eq)
        self.slot_state = SlotState(time_s=0.0, omega0=eq.omega0, tref_min=eq.tref_min, slope_min_per_c=eq.slope_min_per_c)

    def update_slot_state(self, feed: FeedObservation) -> SlotState:
        eq = composition_to_equivalent_properties(feed.composition, normalize=feed.normalize).equivalent
        if len(self.queue) < self.delay_steps:
            for _ in range(self.delay_steps - len(self.queue)):
                self.queue.append(eq)
        delayed = self.queue[0] if len(self.queue) == self.delay_steps else eq
        self.queue.append(eq)

        alpha = math.exp(-self.cfg.dt_update_s / self.cfg.tau_mix_s)
        slot = self.slot_state
        new_state = SlotState(
            time_s=feed.time_s,
            omega0=alpha * slot.omega0 + (1.0 - alpha) * delayed.omega0,
            tref_min=alpha * slot.tref_min + (1.0 - alpha) * delayed.tref_min,
            slope_min_per_c=alpha * slot.slope_min_per_c + (1.0 - alpha) * delayed.slope_min_per_c,
        )
        self.slot_state = new_state
        return new_state

    def build_request(
        self,
        *,
        slot_time_s: float,
        governor_decision: GovernorDecision,
        resource: ResourceBoundary | None = None,
        slot_id: str = "slot",
    ) -> OptimizerRequest:
        resource = resource or ResourceBoundary(
            T_stack_cap_C=self.cfg.resource_T_stack_default_C,
            v_stack_cap_mps=self.cfg.resource_v_stack_default_mps,
        )
        slot = self.slot_state
        props = EquivalentProperties(
            omega0=slot.omega0,
            tref_min=slot.tref_min,
            slope_min_per_c=slot.slope_min_per_c,
        )
        band = DynamicTargetBand(
            omega_min=governor_decision.dyn_band[0],
            omega_max=governor_decision.dyn_band[1],
        )
        return OptimizerRequest(
            props=props,
            omega_tar=governor_decision.omega_tar,
            resource=resource,
            dyn_band=band,
            slot_id=f"{slot_id}_{int(slot_time_s)}",
            burn_policy=self.cfg.burn_policy,
        )
