# FlameGuard MPC Refactor Notes

This branch implements the refactor through the requested **step4** level:

1. Directory/model structure was organized without deleting the legacy code.
2. A shared proxy-model layer was added under `models/`.
3. A distributed preheater forward model was added.
4. A dynamic single-slot inverse optimizer and lookup layer were added.
5. A small development lookup table was generated under `optimizer/generated_tables/`.
6. The existing test scenario files are preserved, but `tests/sim_common.py` now routes them through the new lookup-assisted MPC path by default.

## New model layer

- `models/thermal_core.py` re-exports the validated thermal constants/functions from the original SLSQP optimizer so the forward model and inverse optimizer share the same physical basis.
- `models/preheater_forward_model.py` implements a low-order distributed preheater digital twin with axial cells, material transport, gas-side cooling, sensible heating, and energy/kinetic-limited drying.
- `models/furnace_dynamic.py` extracts the furnace dynamic proxy into a reusable model with `clone()` support for MPC rollout.

## New optimizer/lookup layer

- `optimizer/lookup_optimizer.py` contains the legacy static lookup optimizer extracted from `tests/sim_common.py`.
- `optimizer/dynamic_slot_slsqp_optimizer.py` adds the dynamic single-slot inverse SLSQP optimizer.
- `optimizer/dynamic_slot_table_generator.py` can generate a development dynamic-slot lookup table.
- `optimizer/dynamic_slot_lookup.py` provides a kNN lookup service used by MPC.
- `optimizer/generated_tables/dynamic_slot_lookup_table_dev.csv` is the current small development table. It is intentionally sparse and currently seeded as a lightweight development table for interface/smoke testing; `optimizer/dynamic_slot_table_generator.py` and `optimizer/dynamic_slot_slsqp_optimizer.py` are included so it can be regenerated with SLSQP and later expanded into a production table after the new control loop is tuned.

## New controller path

- `controller/mpc_controller.py` implements lookup-assisted MPC. It evaluates several candidate target outlet-moisture values, queries the dynamic-slot lookup for Tg/vg baselines, rolls out the preheater and furnace models, and selects the lowest-cost first action.
- The legacy `controller/controller.py` remains available but is not used by the default `tests/sim_common.py` path.

## Test path

The existing case entry points remain unchanged:

- `tests/test_case_steady_hold.py`
- `tests/test_case_feed_step_change.py`
- `tests/test_case_furnace_temp_temporary_disturbance.py`
- `tests/test_case_furnace_temp_permanent_disturbance.py`
- `tests/test_case_cold_start.py`

They now test the refactored MPC workflow through `tests/sim_common.py`.

## Smoke tests performed

A short 5 s steady smoke and a short 30 s feed-step smoke were run in the working environment through `run_case(...)`. Full long-horizon scenario runs were not executed here because they are intentionally long; they should be run by the user during tuning.

## 2026-04-26: block-SLSQP NMPC upgrade

This revision implements the requested priority-1/2/3 fixes:

1. **Consistent warm initialization**
   - `tests/sim_common.py` now pre-rolls the distributed preheater under the nominal hold command.
   - `TranscriberB.initialize_previous(...)`, `current_cmd`, fallback MPC, and NMPC warm start are synchronized to the same nominal hold point (`800 C`, `12 m/s` by default).
   - This removes the artificial initial mismatch where the plant started near `omega_ref` while the actuator limiter started near 175–300 C.

2. **Full-model NMPC**
   - Added `controller/nmpc_controller.py`.
   - The controller optimizes a control-blocked sequence of future `Tg/vg` values with SLSQP.
   - The objective is evaluated by rolling out the full distributed 20-cell preheater plus furnace dynamics.
   - The existing lookup-assisted MPC remains as a fallback if SLSQP is unavailable or fails.

3. **No single representative slot in the main decision**
   - Main NMPC scoring no longer queries one representative cell.
   - Candidate decisions are judged through complete 20-cell preheater rollout, so all cell inventories and outlet dynamics participate in the decision.
   - The dynamic slot lookup table is retained only as fallback infrastructure.

Diagnostics:
- Timeseries CSV now includes `control_source` and `control_note` so NMPC/fallback behavior can be inspected after each test.

## 2026-04 dynamic-resource / actuator / observer / context-inverse update

Implemented in this package:

1. **Actuator dynamics**: `models/actuator_dynamic.py` implements rate limit + first-order lag. `TranscriberB` now uses this model for NMPC commands.
2. **Dynamic resource model**: `models/resource_model.py` derives natural stack resources from measured/predicted `T_stack` and `v_stack`. Temperature shortfall can be supplied by auxiliary heat; mass-flow shortfall clamps `vg`.
3. **Auxiliary heat accounting**: `Q_heat_deficit_kW` now represents auxiliary heat required to raise natural stack gas temperature to the commanded preheater inlet temperature.
4. **Disturbance observer**: `estimators/furnace_disturbance_observer.py` estimates additive furnace disturbances from residuals between measured outputs and a disturbance-free nominal furnace twin. NMPC no longer receives the ground-truth scenario disturbance directly.
5. **Feed preview**: `models/feed_preview.py` provides `KnownScheduleFeedPreview`; NMPC rollout can use future feed composition instead of assuming current feed is constant.
6. **Feed delay**: `PreheaterForwardModel` now uses a time-based feed delay buffer instead of tying the 5 s feed delay to the axial cell residence bucket.
7. **Context-aware inverse model**: `optimizer/context_slot_slsqp_optimizer.py` evaluates inverse candidates by rolling out the complete distributed preheater model, so gas cooling along upstream cells is included. `optimizer/context_slot_lookup.py` and `optimizer/context_slot_table_generator.py` define the next-generation table/lookup path. This is not yet the primary NMPC controller path; NMPC still uses full forward rollout directly.
