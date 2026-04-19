from __future__ import annotations

import os
import sys

ROOT = os.path.dirname(os.path.dirname(__file__))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from tests.sim_common import SimConfig, plot_history, run_case


def composition_schedule(t: float):
    if t < 300.0:
        return [0.20, 0.15, 0.15, 0.10, 0.20, 0.20]
    return [0.25, 0.25, 0.20, 0.05, 0.15, 0.10]


if __name__ == '__main__':
    cfg = SimConfig(total_time_s=3000.0, use_lookup_table=True)
    hist = run_case('feed_step_change', composition_schedule, disturbance_schedule=None, cfg=cfg)
    out = os.path.join(os.path.dirname(__file__), 'feed_step_change.png')
    plot_history(hist, 'Closed loop test: feed composition step change without furnace bias', out)
    print(f'Saved plot to {out}')
