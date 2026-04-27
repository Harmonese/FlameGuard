from __future__ import annotations

"""Replaceable proxy-model interfaces.

Control code should depend on these behavioral contracts rather than concrete
Python proxy implementations.  A future COMSOL co-simulation backend or hardware
backend can be adapted by implementing the same methods.
"""

from typing import Any, Protocol, runtime_checkable

from model.control_types import ActuatorCommand, FeedObservation, FurnaceObservation, PreheaterState, ControlSetpoint
from model.model_types import ResourceBoundary


@runtime_checkable
class CloneableModel(Protocol):
    def clone(self) -> Any:
        """Return an independent copy suitable for NMPC rollout."""
        ...


@runtime_checkable
class PreheaterModelProtocol(CloneableModel, Protocol):
    def step(self, feed: FeedObservation, Tg_in_C: float, vg_mps: float, dt_s: float) -> PreheaterState:
        """Advance the preheater model and return its distributed state/output."""
        ...

    def state(self) -> PreheaterState:
        """Return the current distributed preheater state without advancing."""
        ...


@runtime_checkable
class FurnaceModelProtocol(CloneableModel, Protocol):
    def step(self, omega_in: float, dt_s: float, disturbance: Any | None = None) -> tuple[float, float, float]:
        """Advance furnace dynamics and return (T_avg, T_stack, v_stack)."""
        ...

    def observation(self, time_s: float) -> FurnaceObservation:
        """Return current observable furnace outputs."""
        ...


@runtime_checkable
class ActuatorModelProtocol(CloneableModel, Protocol):
    def step(
        self,
        time_s: float,
        setpoint: ControlSetpoint,
        dt_s: float,
        *,
        T_stack_available_C: float,
        v_stack_available_mps: float,
        mdot_stack_cap_kgps: float,
    ) -> ActuatorCommand:
        """Advance actuator dynamics and resource projection/diagnostics."""
        ...


@runtime_checkable
class ResourceModelProtocol(Protocol):
    def boundary_from_observation(self, obs: FurnaceObservation) -> ResourceBoundary:
        """Compute natural flue-gas resource boundary from furnace observation."""
        ...


@runtime_checkable
class FeedPreviewProviderProtocol(Protocol):
    def get(self, t_now: float, horizon_s: float, dt_s: float) -> list[FeedObservation]:
        """Return future feed observations over the prediction horizon."""
        ...


__all__ = [
    "CloneableModel",
    "PreheaterModelProtocol",
    "FurnaceModelProtocol",
    "ActuatorModelProtocol",
    "ResourceModelProtocol",
    "FeedPreviewProviderProtocol",
]
