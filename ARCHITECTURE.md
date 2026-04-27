# FlameGuard Architecture

This document describes the current architecture after the NMPC and project-layout refactor. The root package is intentionally small:

```text
./model       replaceable proxy / plant models and shared physical contracts
./controller  control algorithms, observers, policies, and optimizers
./runtime     simulation, execution adaptation, telemetry, plotting, and tests
./legacy      archived pre-NMPC code and historical experiments
```

The main architectural rule is: **the model layer and the control layer are separate**. A controller may call a model through a stable interface, but it should not own the physical model. Likewise, a model should not know whether it is being used by NMPC, a future COMSOL co-simulation adapter, or a hardware runtime.

---

## 1. High-level flow

```text
Feed schedule / composition
        ↓
model.material_model
        ↓
model.preheater_forward_model + model.furnace_dynamic
        ↓
controller.estimators.furnace_disturbance_observer
        ↓
controller.nmpc_controller
        ↓
controller.optimizer: block-SLSQP / auxiliary inverse tools
        ↓
runtime.execution_adapter
        ↓
model.actuator_dynamic + model.resource_model
        ↓
model.preheater_forward_model
        ↓
model.furnace_dynamic
        ↓
runtime telemetry / plots / metrics / feedback
```

The main online decision is no longer made by a single-slot static optimizer. The current controller is a full-state nonlinear MPC that predicts the 20-cell preheater state and furnace response over a finite horizon.

---

## 2. Model layer: `model/`

The `model/` package owns mathematical and proxy descriptions of the plant.

```text
model/model_types.py              canonical dataclasses and engineering constants
model/control_types.py            runtime/control-facing observations and commands
model/thermal_core.py             shared thermal formulas independent of optimizers
model/interfaces.py               replaceable model backend protocols
model/material_model.py           composition → equivalent material properties
model/preheater_forward_model.py  20-cell preheater forward proxy
model/furnace_dynamic.py          furnace static/dynamic proxy
model/actuator_dynamic.py         actuator rate limit + first-order dynamics
model/resource_model.py           flue-gas resource and auxiliary diagnostics
model/feed_preview.py             future feed schedule provider
model/proxy_data/                 COMSOL/regression data used by proxy models
```

The model layer should remain replaceable. The present backend is a Python proxy model; later backends can be added as:

```text
Python proxy model  →  COMSOL co-simulation  →  hardware/plant interface
```

The lightweight contracts are defined in `model/interfaces.py`:

```text
PreheaterModelProtocol
FurnaceModelProtocol
ActuatorModelProtocol
ResourceModelProtocol
FeedPreviewProviderProtocol
```

---

## 3. Controller layer: `controller/`

The `controller/` package owns decision-making. It may use model interfaces for prediction, but it should not contain plant physics.

```text
controller/nmpc_controller.py          main full-state NMPC controller
controller/mpc_controller.py           lookup-assisted fallback controller
controller/estimators/                 disturbance/state observers
controller/optimizer/                  online/fallback/inverse optimizer utilities
```

### Controller

The controller is no longer a PID/LQI block that outputs only a moisture request. The main controller optimizes a future flue-gas command trajectory:

```text
[Tg_1, vg_1, Tg_2, vg_2, ...]
```

It uses control blocking so that several future time intervals share one pair of control variables.

### Governor / operating policy

The old standalone governor clipped `omega_req` into `omega_tar`. In the current code, governor-like responsibilities are part of the NMPC problem formulation:

```text
reference-band penalty
safety-band penalty
resource/economy penalty
high-flow emergency behavior
overshoot/undershoot management
```

The name “governor” may still be used in reports for this policy layer, but there is no active root-level `governor/` package.

### Optimizer

There are two optimizer roles:

1. **Online NMPC optimizer**: the block-SLSQP solve inside `controller/nmpc_controller.py`.
2. **Auxiliary inverse / lookup tools**: `controller/optimizer/context_*` and `controller/optimizer/dynamic_slot_*`, used for table generation, warm starts, fallback, or engineering interpretation.

The old static SLSQP optimizer is kept as a compatibility facade under `controller/optimizer/optimizer.py`; shared model dataclasses and thermal formulas live in `model/`, not inside the optimizer.

---

## 4. Runtime layer: `runtime/`

The `runtime/` package connects models and controllers into executable scenarios.

```text
runtime/simulator.py          scenario loop and main run_case entry
runtime/execution_adapter.py  control command → actuator/resource execution
runtime/telemetry.py          telemetry/metrics helpers
runtime/plotting.py           plotting helpers
runtime/tests/                runnable scenario tests and smoke tests
```

Typical test commands are now:

```bash
python3.14 -m runtime.tests.test_preheater_timestep_invariance
python3.14 -m runtime.tests.run_all_cases
```

The default output directory is `runtime/results/`.

---

## 5. Legacy layer: `legacy/`

`legacy/` stores historical code that is useful for comparison but is not part of the current main path:

```text
legacy/old_pid_lqi_chain/
legacy/old_governor/
legacy/old_slot_transcriber/
legacy/old_static_optimizer/
legacy/old_identification/
legacy/notes/
```

Legacy modules may not import cleanly after the layout refactor; they are archived for documentation, comparison, and possible manual recovery, not for active execution.

---

## 6. Relation to the old chain

The old engineering narrative was:

```text
controller → governor → transcriber A → optimizer → transcriber B
```

The new architecture should be presented as an upgrade rather than a forced one-to-one mapping:

| Old concept | Old role | Current representation |
|---|---|---|
| controller | PID/LQI moisture request | `controller.nmpc_controller`: full-state NMPC over Tg/vg sequence |
| governor | moisture-target clipping | operating policy embedded in NMPC objectives and constraints |
| transcriber A | future-slot request builder | replaced by model state + feed preview + NMPC problem construction |
| optimizer | static SLSQP for one target slot | online block-SLSQP plus auxiliary inverse tools |
| transcriber B | ideal setpoint to executable command | `runtime.execution_adapter` with actuator/resource diagnostics |

The old names may be referenced in documentation for continuity, but the implementation is now layer-based.

---

## 7. Open modeling tasks

The architecture is now prepared for cleaner documentation and model replacement, but several modeling tasks remain open:

1. Replace the one-pass flue-gas resource assumption with a fresh-gas plus recirculation-loop energy model.
2. Recalibrate the furnace proxy model if COMSOL re-runs provide updated `T_stack`, `v_stack`, stack mass flow, and stack enthalpy relations.
3. Add concrete COMSOL co-simulation and hardware backend adapters against `model/interfaces.py`.
4. Split `runtime/simulator.py` into smaller internal modules once behavior is frozen.
5. Revisit auxiliary inverse/table generation after the flue-gas loop model is finalized.

---

## 8. Recommended README direction

Keep the original two-section logic:

```text
I. Mathematical modeling and proxy model layer
II. Control algorithm and closed-loop architecture
```

The model section should describe material, furnace, preheater, actuator, and flue-gas resource models. The control section should describe disturbance observation, operating policy, NMPC, optimizer, runtime execution, and test scenarios.
