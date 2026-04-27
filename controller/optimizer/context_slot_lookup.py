from __future__ import annotations

import csv
from pathlib import Path
import numpy as np

from model.control_types import DynamicSlotLookupResult
from model.preheater_forward_model import PreheaterForwardModel
from controller.optimizer.context_slot_slsqp_optimizer import summarize_context
from model.model_types import ResourceBoundary


class ContextSlotLookup:
    """kNN lookup for context-aware inverse rows.

    The table features include the representative slot plus compact upstream
    load statistics, so it is closer to the full forward model than the older
    isolated-slot table.
    """

    fields = [
        'slot_index', 'residence_left_s', 'omega_current', 'T_solid_current_C',
        'omega0', 'tref_min', 'slope_min_per_c', 'upstream_omega_mean',
        'upstream_T_solid_mean_C', 'upstream_load_proxy', 'local_Tg_nominal_C',
        'omega_target', 'T_stack_cap_C', 'v_stack_cap_mps'
    ]
    scales = {
        'slot_index': 10.0,
        'residence_left_s': 500.0,
        'omega_current': 0.25,
        'T_solid_current_C': 120.0,
        'omega0': 0.25,
        'tref_min': 5.0,
        'slope_min_per_c': 0.08,
        'upstream_omega_mean': 0.25,
        'upstream_T_solid_mean_C': 120.0,
        'upstream_load_proxy': 5.0,
        'local_Tg_nominal_C': 300.0,
        'omega_target': 0.2,
        'T_stack_cap_C': 400.0,
        'v_stack_cap_mps': 10.0,
    }

    def __init__(self, csv_path: str | Path, *, k_neighbors: int = 12):
        self.path = Path(csv_path)
        self.k = max(1, int(k_neighbors))
        self.rows = []
        with self.path.open(encoding='utf-8-sig', newline='') as f:
            for row in csv.DictReader(f):
                parsed = {}
                for k, v in row.items():
                    if v in ('True', 'False'):
                        parsed[k] = v == 'True'
                    else:
                        try:
                            parsed[k] = float(v)
                        except Exception:
                            parsed[k] = v
                self.rows.append(parsed)
        if not self.rows:
            raise ValueError(f'empty context lookup table: {self.path}')
        self.X = np.asarray([[float(r.get(f, 0.0)) / self.scales[f] for f in self.fields] for r in self.rows], dtype=float)

    def query(self, *, preheater: PreheaterForwardModel, omega_target: float, resource: ResourceBoundary,
              representative_remaining_s: float = 360.0, time_s: float = 0.0) -> DynamicSlotLookupResult:
        feat = summarize_context(preheater.state(), representative_remaining_s=representative_remaining_s)
        qdict = dict(feat.__dict__)
        qdict.update({'omega_target': float(omega_target), 'T_stack_cap_C': resource.T_stack_cap_C, 'v_stack_cap_mps': resource.v_stack_cap_mps})
        q = np.asarray([float(qdict.get(f, 0.0)) / self.scales[f] for f in self.fields], dtype=float)
        d = np.linalg.norm(self.X - q[None, :], axis=1)
        idx = np.argsort(d)[:self.k]
        w = 1.0 / np.maximum(d[idx], 1e-9)
        w = w / np.sum(w)
        def avg(name, default=0.0):
            return float(sum(wi * float(self.rows[ii].get(name, default)) for wi, ii in zip(w, idx)))
        feasible = any(bool(self.rows[ii].get('feasible', False)) for ii in idx)
        return DynamicSlotLookupResult(
            time_s=float(time_s),
            Tg_star_C=avg('Tg_star_C', 800.0),
            vg_star_mps=avg('vg_star_mps', 12.0),
            omega_target=float(omega_target),
            omega_reachable=avg('omega_reachable', omega_target),
            feasible=bool(feasible),
            power_kW=avg('power_kW', 0.0),
            Qsup_kW=avg('Qsup_kW', 0.0),
            Qreq_kW=avg('Qreq_kW', 0.0),
            mdot_stack_cap_kgps=avg('mdot_stack_cap_kgps', float('inf')),
            mdot_preheater_kgps=avg('mdot_preheater_kgps', 0.0),
            source='context_slot_knn_lookup',
        )
