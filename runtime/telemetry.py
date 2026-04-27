from __future__ import annotations

"""Telemetry helpers re-exported from runtime.simulator.

This small module marks the intended split point for Step-3.  The implementation
currently lives in ``runtime.simulator`` to avoid changing behavior; future work
can move the function bodies here without changing imports.
"""

from .simulator import History, history_to_csv_rows, print_metrics_table, save_case_artifacts

__all__ = ["History", "history_to_csv_rows", "save_case_artifacts", "print_metrics_table"]
