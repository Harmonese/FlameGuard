from __future__ import annotations

"""Compatibility wrapper for test scenarios.

Runtime orchestration was moved to ``runtime.simulator`` in Step-3 so tests can
remain thin scenario/verification entry points.  Existing case scripts still
import ``tests.sim_common``; this wrapper preserves that public import path.
"""

from runtime.simulator import *  # noqa: F401,F403
