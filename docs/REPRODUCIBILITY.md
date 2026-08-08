# Reproducibility

The release is a clean-room export from the frozen source commit recorded in `reproducibility/release_manifest.json`. The export intentionally excludes the source repository's Git history, development task cards, failed-method pipelines, and large raw-run artifacts.

## Fresh installation

1. Clone with `--recurse-submodules`.
2. Install `requirements-lock.txt`.
3. Install the package with `python -m pip install -e .`.
4. Run `python scripts/reproduce_quick.py`.

## Full S5B reproduction

`scripts/run_s5b_holdout.py` regenerates the deterministic random wind profiles using the frozen `numpy.PCG64` contract, runs LQR, LS-PMPC, and the secondary observer ablation, and recomputes metrics directly from newly written CSV files. No historical raw results are used as input.

The full run is hardware dependent in wall-clock time. Acceptance is based on the frozen paired count, safety gate, and metric tolerances described in `reproducibility/expected/s5b_expected.json`.

## Task-LQR reproduction

`scripts/reproduce_task_lqr.py` runs old LQR and the frozen `lqr_011` Task-LQR on the calm and crosswind setpoint scenes, computes task metrics from the generated runs, and checks the frozen tolerances in `reproducibility/expected/task_lqr_expected.json`.
