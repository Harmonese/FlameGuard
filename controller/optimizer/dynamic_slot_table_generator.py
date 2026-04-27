from __future__ import annotations

import csv
from pathlib import Path
from itertools import product

from model.control_types import PreheaterCellState
from model.model_types import ResourceBoundary
from controller.optimizer.dynamic_slot_slsqp_optimizer import DynamicSlotRequest, DynamicSlotSLSQPOptimizer, result_to_row


def iter_dev_requests(resource: ResourceBoundary | None = None):
    resource = resource or ResourceBoundary(930.0, 18.0)
    # Small development table, intentionally sparse. It is enough for interface
    # validation and smoke tests; production coverage should be generated later.
    omega0_vals = [0.65, 0.80]
    tref_vals = [14.5, 16.0]
    slope_vals = [-0.21]
    omega_current_vals = [0.32, 0.45, 0.60]
    T_vals = [70.0, 130.0]
    tau_vals = [120.0, 360.0, 720.0]
    target_vals = [0.25, 0.32, 0.40]
    idx = 0
    for omega0, tref, slope, omega_current, T, tau, target in product(
        omega0_vals, tref_vals, slope_vals, omega_current_vals, T_vals, tau_vals, target_vals
    ):
        cell = PreheaterCellState(
            index=0,
            z_center_m=0.0,
            residence_left_s=tau,
            omega=omega_current,
            T_solid_C=T,
            omega0=omega0,
            tref_min=tref,
            slope_min_per_c=slope,
        )
        yield DynamicSlotRequest(cell=cell, omega_target=target, resource=resource, slot_id=f'dev_{idx}')
        idx += 1


def build_dev_table(csv_path: str | Path, *, limit: int | None = None) -> list[dict]:
    opt = DynamicSlotSLSQPOptimizer()
    rows = []
    for n, req in enumerate(iter_dev_requests()):
        if limit is not None and n >= limit:
            break
        try:
            res = opt.optimize(req)
            rows.append(result_to_row(req, res))
        except Exception as exc:
            rows.append({
                'slot_id': req.slot_id,
                'omega0': req.cell.omega0,
                'tref_min': req.cell.tref_min,
                'slope_min_per_c': req.cell.slope_min_per_c,
                'omega_current': req.cell.omega,
                'T_solid_current_C': req.cell.T_solid_C,
                'tau_remaining_s': req.cell.residence_left_s,
                'omega_target': req.omega_target,
                'T_stack_cap_C': req.resource.T_stack_cap_C,
                'v_stack_cap_mps': req.resource.v_stack_cap_mps,
                'feasible': False,
                'message': f'{type(exc).__name__}: {exc}',
            })
    path = Path(csv_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if rows:
        fields = sorted({k for r in rows for k in r.keys()})
        with path.open('w', encoding='utf-8-sig', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)
    return rows


if __name__ == '__main__':
    out = Path(__file__).resolve().parent / 'generated_tables' / 'dynamic_slot_lookup_table_dev.csv'
    rows = build_dev_table(out)
    print(f'wrote {len(rows)} rows -> {out}')
