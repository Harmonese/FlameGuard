from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Sequence

from model.control_types import FeedObservation


class FeedPreviewProvider:
    def get(self, time_s: float, *, horizon_s: float, dt_s: float) -> list[FeedObservation]:
        raise NotImplementedError


@dataclass
class ConstantFeedPreview(FeedPreviewProvider):
    composition: Sequence[float]
    normalize: bool = False

    def get(self, time_s: float, *, horizon_s: float, dt_s: float) -> list[FeedObservation]:
        n = max(1, int(round(horizon_s / max(dt_s, 1e-9))))
        return [FeedObservation(time_s=time_s + (k + 1) * dt_s, composition=self.composition, normalize=self.normalize) for k in range(n)]


@dataclass
class KnownScheduleFeedPreview(FeedPreviewProvider):
    composition_schedule: Callable[[float], Sequence[float]]
    normalize: bool = False

    def get(self, time_s: float, *, horizon_s: float, dt_s: float) -> list[FeedObservation]:
        n = max(1, int(round(horizon_s / max(dt_s, 1e-9))))
        return [
            FeedObservation(
                time_s=time_s + (k + 1) * dt_s,
                composition=self.composition_schedule(time_s + (k + 1) * dt_s),
                normalize=self.normalize,
            )
            for k in range(n)
        ]
