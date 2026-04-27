# Changelog

## Layered root-layout cleanup

- Collapsed the project root into four active top-level packages: `model/`, `controller/`, `runtime/`, and `legacy/`.
- Moved shared physical dataclasses and thermal equations into `model/`.
- Moved control optimizers and inverse lookup tools into `controller/optimizer/`.
- Moved the disturbance observer into `controller/estimators/`.
- Moved execution adaptation into `runtime/execution_adapter.py`.
- Moved scenario tests into `runtime/tests/`.
- Moved COMSOL/regression proxy data into `model/proxy_data/`.
- Removed the empty root-level `governor/`, `optimizer/`, `transcriber/`, `tests/`, `models/`, `cleanser/`, `estimators/`, and `proxy_model/` packages.
- Updated imports to the new layer-oriented paths.

## Current main commands

```bash
python3.14 -m runtime.tests.test_preheater_timestep_invariance
python3.14 -m runtime.tests.run_all_cases
```

Generated plots, CSVs, and metrics are written to `runtime/results/` by default.
