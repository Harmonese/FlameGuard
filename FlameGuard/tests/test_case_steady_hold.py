from __future__ import annotations

import os
import sys

ROOT = os.path.dirname(os.path.dirname(__file__))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from tests.sim_common import SimConfig, plot_history, run_case


def composition_schedule(t: float):
    return [0.20, 0.15, 0.15, 0.10, 0.20, 0.20]


if __name__ == '__main__':
    cfg = SimConfig(total_time_s=1200.0, use_lookup_table=True)
    hist = run_case('steady_hold', composition_schedule, disturbance_schedule=None, cfg=cfg)
    out = os.path.join(os.path.dirname(__file__), 'steady_hold.png')
    plot_history(hist, 'Closed loop test: steady hold without external disturbance', out)
    print(f'Saved plot to {out}')
