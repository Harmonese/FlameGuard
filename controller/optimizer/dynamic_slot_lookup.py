from __future__ import annotations

import csv
from pathlib import Path
import numpy as np

from model.control_types import DynamicSlotLookupResult, PreheaterCellState
from model.model_types import Config, ResourceBoundary


class DynamicSlotLookup:
    """kNN interpolated lookup for dynamic single-slot inverse control."""

    def __init__(self, csv_path: str | Path, *, k_neighbors: int = 8, fallback_slsqp: bool = True):
        self.path = Path(csv_path)
        self.k_neighbors = max(1, k_neighbors)
        self.fallback_slsqp = fallback_slsqp
        self.rows: list[dict] = []
        if self.path.exists():
            with self.path.open(encoding='utf-8-sig', newline='') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    parsed = {}
                    for k, v in row.items():
                        if v in ('True', 'False'):
                            parsed[k] = (v == 'True')
                        else:
                            try:
                                parsed[k] = float(v)
                            except Exception:
                                parsed[k] = v
                    self.rows.append(parsed)
        self._slsqp = None
        if self.rows:
            self._build_arrays()
        else:
            self._x = None

    def _build_arrays(self) -> None:
        fields = ['omega0', 'tref_min', 'slope_min_per_c', 'omega_current', 'T_solid_current_C', 'tau_remaining_s', 'omega_target']
        self._fields = fields
        self._x = np.array([[float(r[f]) for f in fields] for r in self.rows], dtype=float)
        span = np.ptp(self._x, axis=0)
        self._scale = np.where(span > 1e-9, span, 1.0)

    def _fallback(self, cell: PreheaterCellState, omega_target: float, resource: ResourceBoundary, time_s: float) -> DynamicSlotLookupResult:
        from controller.optimizer.dynamic_slot_slsqp_optimizer import DynamicSlotRequest, DynamicSlotSLSQPOptimizer
        if self._slsqp is None:
            self._slsqp = DynamicSlotSLSQPOptimizer()
        req = DynamicSlotRequest(cell=cell, omega_target=omega_target, resource=resource, time_s=time_s)
        res = self._slsqp.optimize(req)
        return DynamicSlotLookupResult(**{**res.__dict__, 'source': 'dynamic_slot_slsqp_fallback'})

    def query(self, *, cell: PreheaterCellState, omega_target: float, resource: ResourceBoundary, time_s: float = 0.0) -> DynamicSlotLookupResult:
        if not self.rows or self._x is None:
            if self.fallback_slsqp:
                return self._fallback(cell, omega_target, resource, time_s)
            raise RuntimeError(f'Dynamic slot lookup table not found or empty: {self.path}')
        q = np.array([
            cell.omega0,
            cell.tref_min,
            cell.slope_min_per_c,
            cell.omega,
            cell.T_solid_C,
            cell.residence_left_s,
            omega_target,
        ], dtype=float)
        d = (self._x - q) / self._scale
        dsq = np.sum(d * d, axis=1)
        idx = np.argsort(dsq)[: self.k_neighbors]
        if dsq[idx[0]] < 1e-14:
            r = self.rows[int(idx[0])]
            return DynamicSlotLookupResult(
                time_s=time_s,
                Tg_star_C=float(r['Tg_star_C']),
                vg_star_mps=float(r['vg_star_mps']),
                omega_target=float(r.get('omega_target', omega_target)),
                omega_reachable=float(r.get('omega_reachable', omega_target)),
                feasible=bool(r.get('feasible', True)),
                power_kW=float(r.get('power_kW', 0.0)),
                Qsup_kW=float(r.get('Qsup_kW', 0.0)),
                Qreq_kW=float(r.get('Qreq_kW', 0.0)),
                mdot_stack_cap_kgps=float(r.get('mdot_stack_cap_kgps', float('inf'))),
                mdot_preheater_kgps=float(r.get('mdot_preheater_kgps', 0.0)),
                source='dynamic_slot_lookup_exact',
            )
        weights = 1.0 / np.maximum(np.sqrt(dsq[idx]), 1e-9)
        weights = weights / np.sum(weights)
        def interp(field: str, default: float = 0.0) -> float:
            return float(sum(weights[j] * float(self.rows[int(i)].get(field, default)) for j, i in enumerate(idx)))
        feasible = bool(sum(weights[j] * float(bool(self.rows[int(i)].get('feasible', True))) for j, i in enumerate(idx)) >= 0.5)
        return DynamicSlotLookupResult(
            time_s=time_s,
            Tg_star_C=interp('Tg_star_C'),
            vg_star_mps=interp('vg_star_mps'),
            omega_target=float(omega_target),
            omega_reachable=interp('omega_reachable', omega_target),
            feasible=feasible,
            power_kW=interp('power_kW'),
            Qsup_kW=interp('Qsup_kW'),
            Qreq_kW=interp('Qreq_kW'),
            mdot_stack_cap_kgps=interp('mdot_stack_cap_kgps', float('inf')),
            mdot_preheater_kgps=interp('mdot_preheater_kgps'),
            source='dynamic_slot_lookup_knn',
        )
