from __future__ import annotations

import csv
from itertools import product
from pathlib import Path

from model.control_types import FeedObservation
from model.preheater_forward_model import PreheaterForwardModel, PreheaterForwardConfig
from model.model_types import ResourceBoundary
from controller.optimizer.context_slot_slsqp_optimizer import ContextSlotRequest, ContextSlotSLSQPOptimizer, result_to_context_row


def _make_synthetic_preheater(*, omega_base: float, T_base: float, upstream_hot: bool) -> PreheaterForwardModel:
    p = PreheaterForwardModel(PreheaterForwardConfig(n_cells=20, tau_residence_s=985.0, feed_delay_s=5.0))
    comp = [0.2, 0.15, 0.15, 0.15, 0.2, 0.15]
    p.initialize(comp, omega_init=omega_base, T_solid_init_C=T_base, time_s=0.0)
    for i in range(p.cfg.n_cells):
        frac = i / max(p.cfg.n_cells - 1, 1)
        # Create simple but diverse upstream/downstream context profiles.
        p.omega[i] = max(0.20, min(0.95, omega_base + (0.08 if frac < 0.5 else -0.04) * (1.0 - frac)))
        p.T_solid[i] = max(20.0, min(250.0, T_base + (60.0 if upstream_hot else -20.0) * (1.0 - frac)))
    # Establish a gas profile diagnostic under a nominal command.
    p.step(FeedObservation(time_s=20.0, composition=comp), 800.0, 12.0, 20.0)
    return p


def iter_dev_requests(resource: ResourceBoundary | None = None):
    resource = resource or ResourceBoundary(930.0, 12.0)
    omega_vals = [0.32, 0.45, 0.60]
    T_vals = [90.0, 150.0]
    upstream_hot_vals = [False, True]
    target_vals = [0.25, 0.32, 0.40]
    idx = 0
    for omega, T, upstream_hot, target in product(omega_vals, T_vals, upstream_hot_vals, target_vals):
        p = _make_synthetic_preheater(omega_base=omega, T_base=T, upstream_hot=upstream_hot)
        yield ContextSlotRequest(
            preheater=p,
            feed=FeedObservation(time_s=20.0, composition=[0.2, 0.15, 0.15, 0.15, 0.2, 0.15]),
            omega_target=target,
            resource=resource,
            horizon_s=600.0,
            time_s=20.0,
            slot_id=f'context_dev_{idx}',
        )
        idx += 1


def build_dev_table(csv_path: str | Path, *, limit: int | None = None) -> list[dict]:
    opt = ContextSlotSLSQPOptimizer()
    rows = []
    for n, req in enumerate(iter_dev_requests()):
        if limit is not None and n >= limit:
            break
        try:
            res = opt.optimize(req)
            rows.append(result_to_context_row(req, res))
        except Exception as exc:
            rows.append({'slot_id': req.slot_id, 'feasible': False, 'message': f'{type(exc).__name__}: {exc}'})
    path = Path(csv_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if rows:
        fields = sorted({k for r in rows for k in r.keys()})
        with path.open('w', encoding='utf-8-sig', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader(); writer.writerows(rows)
    return rows


if __name__ == '__main__':
    out = Path(__file__).resolve().parent / 'generated_tables' / 'context_slot_lookup_table_dev.csv'
    rows = build_dev_table(out)
    print(f'wrote {len(rows)} rows -> {out}')
