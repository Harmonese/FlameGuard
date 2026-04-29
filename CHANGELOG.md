# Changelog

## Runtime cleanup, factories, and interface tightening

- Added `plant/factory.py` so runtime obtains the active `PlantBackend` through a single factory instead of manually constructing Python plant internals.
- Added `controller/factory.py` to assemble predictor resource model, state estimator, operator, and executor from runtime configuration.
- Removed legacy target-moisture/governor telemetry fields and old override helper definitions from `runtime/simulator.py` (`omega_req`, `omega_tar`, `dyn_lo`, `dyn_hi`, `governor_clipped`, `fast_mode`, lookup-table config remnants).
- Tightened `PredictorBundle` with structural predictor protocols instead of untyped `Any` for preheater/furnace/resource predictors.
- Updated async NMPC plan acceptance: finite bounded best-effort SLSQP plans are accepted even when SciPy reports `success=False` due to iteration limits; optimizer success remains visible as telemetry.
- Added recovery-guard hysteresis and minimum hold time (`recovery_guard_exit_C`, `recovery_guard_min_hold_s`) plus requested/active/reason telemetry.
- Updated latency-aware metrics to use recorded sample durations instead of row-count times nominal `dt`, which keeps metrics meaningful when simulated compute latency advances plant time irregularly.
- Removed generated `runtime/results/` artifacts from the source package and added them to `.gitignore`.

## Deep-drying moisture floor update

- Lowered the default preheater residual wet-basis moisture floor from `0.20` to `0.05` in both `plant/python_model` and `controller/predictor`. This keeps the model numerically bounded above zero while allowing deep drying under high auxiliary flue-gas temperature.
- Updated the shared low-order `Config.OMEGA_MODEL_MIN` defaults and validation messages to remove the old hard-coded 20% assumption.
- Updated README configuration notes to document the residual moisture floor and its role in permanent-disturbance safety reachability.

## Aggressive safety-recovery control speed update

- Added state-dependent NMPC recovery weights: when predicted furnace temperature approaches the 850 degC compliance floor, auxiliary heat, auxiliary circulation, fan power, and high-vg economic penalties are strongly reduced while safety penalties are increased.
- Added explicit max-recovery SLSQP seeds (`Tg = Tg_max`, `vg = vg_max`) when the current furnace temperature is in the recovery region.
- Added an executor-level safety recovery guard. If measured `T_avg` is below the compliance floor, or the active NMPC plan predicts safety is not reachable while temperature is still below target, executor requests a prompt `1100 degC / 12 m/s` recovery command and marks `recovery_guard_active`.
- Added `recovery_guard_active` to `ActuatorCommand`, time-series CSVs, control-event CSVs, and segment metrics.
- Fixed two runtime telemetry duplication leftovers: duplicate feed composition appends and duplicate command-apply-delay appends.

## Compute-latency aware runtime diagnostics

- Added optional latency-aware runtime simulation via `SimConfig.compute_latency_mode`: `none` preserves ideal simulation, `profile` advances plant time by measured operator wall-clock solve time, and `fixed` advances by `fixed_compute_latency_s` for reproducible what-if tests.
- During simulated compute latency, the plant continues to run under the previous `ActuatorCommand`; the newly solved command is applied only after the simulated delay.
- Added telemetry columns and metrics for `operator_compute_wall_s`, `simulated_compute_latency_s`, `command_apply_delay_s`, async job submissions, accepted/discarded plans, and async result state age.
- Updated `AsyncNonlinearMPCController` to count submitted/accepted/discarded background plans and discard background results whose source state is older than the stale-plan timeout.
- Added `runtime.tests.test_compute_latency_effect` to compare closed-loop recovery under fixed compute delays such as 0, 2, 5, 10, and 30 seconds.

## Safety recovery and auxiliary heat limit update

- Raised the effective auxiliary flue-gas temperature ceiling from 930 degC to 1100 degC across runtime, plant resource model, predictor resource model, and NMPC bounds. This reflects that actual incinerator flue gas / auxiliary heating can be substantially hotter than the previous conservative cap.
- Made NMPC moisture targets disturbance-aware: `omega_target` now inverts the furnace surrogate against `T_target_C - disturbance_est_Tavg_C`, so permanent negative furnace disturbances demand lower outlet moisture.
- Added safety feasibility diagnostics to `MPCDecision`, time-series CSVs, control-event CSVs, and metrics: `safety_reachable`, `safety_margin_C`, `omega_max_for_safety`, `nmpc_pred_min_Tavg_C`, and `nmpc_pred_max_Tavg_C`.
- Added emergency re-optimization logic: when measured `T_avg` is below the safety/compliance floor, cached plans are shortened by `emergency_reoptimize_interval_s` instead of being held for the full nominal reoptimization period.

## NMPC first-stage SLSQP speed cleanup

- Added a lightweight `PreheaterForwardModel.step_output()` / `step_fast()` path for controller rollout. NMPC objective evaluations now use `PreheaterOutput` instead of constructing a full 20-cell `PreheaterState` at every internal prediction step.
- Replaced predictor preheater `copy.deepcopy()` cloning with a manual structural clone that copies only mutable numeric state and inlet-delay history.
- Moved preheater moisture synchronization out of the per-cell energy loop; the predictor now synchronizes moisture once per heat/drying step instead of once per cell.
- Cached inverse furnace-surrogate moisture targets in `NonlinearMPCController._omega_target_for_temperature()` to avoid repeated 40-step bisections during SLSQP finite-difference evaluations.
- Precomputed rollout feeds, step sizes, elapsed times, and block indices once per NMPC solve and reused them across seed evaluation, SLSQP objective calls, and final rollout.

## COMSOL furnace static surrogate update

- Added `scripts/fit_furnace_static_surrogate.py` to fit a reproducible polynomial regression from the COMSOL batch-run CSV tables.
- Stored the COMSOL exports under `scripts/data/furnace_static_comsol/` and the current degree-3 fit report at `scripts/furnace_static_surrogate_fit.json`.
- Replaced the old one-dimensional linear furnace steady proxy with a two-input COMSOL surrogate using wet-basis moisture `omega = w_b / 100` and dry-basis ratio `rd`.
- Converted COMSOL temperature outputs from K to degC during fitting; runtime furnace outputs remain in degC/m/s.
- Kept plant and controller predictor implementations as separate files. Both now carry independent copies of the fitted static surrogate and the same two-lag/dead-time dynamic wrapper.
- Existing scenarios continue to default to `rd=3.0`; future feed characterization can pass `dry_basis_ratio`, `rd`, `r_d`, or `dry_ratio` through `FeedstockObservation.raw`.

## Historical root-layout cleanup

- Earlier revisions moved the project away from the old target-moisture/governor/transcriber layout toward the current `domain/`, `plant/`, `controller/`, and `runtime/` packages.
- The active tree no longer contains `model/`, `legacy/`, or `controller/optimizer/`; old compatibility paths have been removed from the delivery package.

## Current main commands

```bash
python3.14 -m runtime.tests.test_preheater_timestep_invariance
python3.14 -m runtime.tests.run_all_cases
```

Generated plots, CSVs, and metrics are written to `runtime/results/` by default.
