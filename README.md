# UAV Multi-Link Anti-Sway Control

Reproducible MuJoCo simulation of a 6-DoF UAV carrying five passive rigid links and a 2.5 kg cutter under distributed wind disturbances.

## Validated Method

The primary validated controller is LQR-stabilized preview MPC (LS-PMPC). The release also exposes the PID and frozen full-state LQR baselines, plus the selected S6T2 Task-LQR baseline.

## Main Results

The frozen S5B benchmark contains 60 paired evaluations:

- tip RMS improvement: **13.060586%**
- UAV x-RMSE improvement: **19.859454%**
- tip wins: **59/60**
- primary safety failures: **0**
- historical solve-time P95: **1.0731 ms**

Solve timing is hardware dependent and is recorded for reference only.

## Task-Space Extension

The selected Task-LQR `lqr_011` baseline reports:

- calm position RMSE improvement: **17.407892%**
- calm orientation RMSE improvement: **51.721920%**
- calm acquisition: **2.455 s**
- crosswind position: **6.328867% worse**
- crosswind orientation improvement: **41.985099%**
- crosswind acquisition: **not achieved**

No final robust task-space method was established. Task-LQR is published as a frozen baseline and not as a robust crosswind solution.

## Quick Start

```bash
git clone --recurse-submodules https://github.com/canimiliya/uav_multilink_antisway_control_final.git
cd uav_multilink_antisway_control_final
python -m pip install -r requirements-lock.txt
python -m pip install -e .
python scripts/reproduce_quick.py
```

The lock file also installs the pinned `third_party/udaan` submodule so the shared geometric inner loop is importable in a fresh environment.

The quick result is written to `outputs/quick/result.json`. Generated outputs are ignored by Git.

## Reproduce Results

```bash
python scripts/reproduce_task_lqr.py
python scripts/run_s5b_holdout.py --model-config configs/model_5link.yaml --da-config configs/da_pmpc.yaml --headless
```

The S5B script regenerates the 20 seeded random wind series and runs the frozen 60-pair primary comparison. It does not read historical raw run CSV files.

## Repository Structure

- `src/uav_sway/`: simulation, disturbances, controllers, metrics, and task-space state extraction
- `configs/`: frozen model, controller, scenario, and wind contracts
- `scripts/`: quick, Task-LQR, and full S5B reproduction entry points
- `reproducibility/frozen/`: small model, linear-model, controller, and task-space references
- `tests/`: release-surface tests
- `third_party/udaan/`: pinned Git submodule

## Environment

The verified lock uses Python 3.11.15, MuJoCo 3.0.1, NumPy 2.4.6, SciPy 1.17.1, OSQP 1.1.3, PyYAML 6.0.3, matplotlib 3.11.1, imageio 2.37.4, and pytest 9.1.1. `environment.yml` provides the corresponding Conda environment.

## Limitations

This is a simulation benchmark. It is not a PX4, ROS 2, hardware, vision, cutting-contact, or real-flight validation. The plant uses a planar suspended-chain abstraction, distributed aerodynamic force proxies, and explicitly documented engineering estimates for some geometry and inertia parameters.

## License

The main project is MIT licensed. See `LICENSE` and `THIRD_PARTY_NOTICES.md` for the pinned Udaan dependency and its separate BSD 3-Clause license.

## Citation

If you use this release, please cite the metadata in `CITATION.cff` and preserve the frozen provenance in `reproducibility/release_manifest.json`.
