# Legacy code archive

This folder stores modules from earlier FlameGuard control-chain iterations. They are kept for traceability and documentation continuity, but they are not the current online path.

## Archived groups

- `old_pid_lqi_chain/`: PID/LQI-style controller code from the old controller-governor-static-optimizer chain.
- `old_governor/`: old dynamic moisture-target clipping governor.
- `old_slot_transcriber/`: old Transcriber A future-slot request builder.
- `old_static_optimizer/`: static optimizer helpers/tests that are no longer part of the main NMPC runtime.
- `old_identification/`: outer-loop identification experiments and result CSV files.
- `experimental_slot_lookup/`: experimental single-slot dynamic inverse lookup code if later moved here.

## Important note

`optimizer/optimizer.py` remains in the main tree for now because active modules still import shared types and thermal utility functions from it, such as `ResourceBoundary`, `Config`, `rho_g`, `power_kW`, and mass-flow helpers. A later cleanup should split this file into:

- shared thermal types/utilities, and
- legacy static SLSQP optimizer.
