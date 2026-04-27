from __future__ import annotations

import csv
from pathlib import Path
from types import SimpleNamespace
import numpy as np

from optimizer.optimizer import OptimizerRequest


class LookupOptimizer:
    """Legacy static lookup optimizer extracted from tests/sim_common.py."""

    def __init__(self, csv_path: str | Path, *, k_neighbors: int = 12):
        self.rows = []
        self.k_neighbors = max(3, k_neighbors)
        path = Path(csv_path)
        with path.open(encoding='utf-8-sig', newline='') as f:
            reader = csv.DictReader(f)
            for row in reader:
                parsed = {k: row[k] for k in row}
                for key, val in row.items():
                    if val in ('True', 'False'):
                        parsed[key] = (val == 'True')
                    else:
                        try:
                            parsed[key] = float(val)
                        except Exception:
                            parsed[key] = val
                if parsed.get('success', True):
                    self.rows.append(parsed)
        if not self.rows:
            raise RuntimeError('Lookup table is empty or has no successful rows.')
        self._omega0 = np.array([r['omega0'] for r in self.rows], dtype=float)
        self._tref = np.array([r['tref_min'] for r in self.rows], dtype=float)
        self._slope = np.array([r['slope_min_per_c'] for r in self.rows], dtype=float)
        self._otar = np.array([r['omega_tar_requested'] for r in self.rows], dtype=float)
        self._scales = np.array([
            max(self._omega0.max() - self._omega0.min(), 1e-6),
            max(self._tref.max() - self._tref.min(), 1e-6),
            max(self._slope.max() - self._slope.min(), 1e-6),
            max(self._otar.max() - self._otar.min(), 1e-6),
        ])
        self._fields_float = [
            'Tg_star_C', 'vg_star_mps', 'Tm_star_C', 'omega_opt', 'power_kW',
            'Qreq_kW', 'Qsup_kW', 'mdot_stack_cap_kgps', 'mdot_preheater_kgps',
            'omega_tar_projected'
        ]

    def query(self, req: OptimizerRequest):
        q = np.array([req.props.omega0, req.props.tref_min, req.props.slope_min_per_c, req.omega_tar])
        data = np.vstack([self._omega0, self._tref, self._slope, self._otar]).T
        d = (data - q) / self._scales
        dsq = np.sum(d * d, axis=1)
        idx = np.argsort(dsq)[: self.k_neighbors]
        if dsq[idx[0]] < 1e-14:
            return SimpleNamespace(**self.rows[int(idx[0])])
        weights = 1.0 / np.maximum(np.sqrt(dsq[idx]), 1e-9)
        weights = weights / np.sum(weights)
        out = {}
        base = self.rows[int(idx[0])]
        for field in base.keys():
            if field in self._fields_float:
                out[field] = float(sum(weights[j] * self.rows[int(i)][field] for j, i in enumerate(idx)))
            else:
                out[field] = base[field]
        out['success'] = True
        return SimpleNamespace(**out)
