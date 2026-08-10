# Native-Stack Benchmark v1

## Research position

This is Benchmark B, a new system-level protocol. Benchmark A remains the frozen V1-V10 task-level comparison at 20 Hz with a common `[ax, ay, az]` interface, including the valid V5 SATC Holdout result. Benchmark B neither invalidates V5 nor rescues any V6-V10 paper result.

Benchmark B standardizes the unchanged five-link MuJoCo plant, physical actuator authority, causal sensor envelope, tasks, wind realizations, safety, and metrics. Controllers may own their model, observer, solver, hierarchy, inner loop, and preregistered supported rate. Every stack must end at the same `WrenchCommand[T,tau_x,tau_y,tau_z]` interface.

## Frozen assets

- Plant: `reproducibility/frozen/model/model_5link_controlled.xml`, unchanged from V10.
- Physics: 1000 Hz RK4.
- Physical limits: thrust `[0, 285.74568] N`; body torques `±25`, `±25`, `±12 N m`.
- Supported controller rates: 20, 50, 100, 200, 500, and 1000 Hz.
- Native Development: 200 cases, executable.
- Native Holdout: 140 cases, `execution_allowed=false`, never executed in P2-R0.

## Safety v2

Legacy finite, height, joint, attitude, and actuator checks remain. Native-only runaway diagnostics freeze horizontal displacement limits of 8 m for the UAV and 10 m for the cutter tip. The task bank is confined to targets within 2 m of trim; these limits reserve at least four times the target radius plus suspended-tool reach. They were frozen before any future native-controller performance run and were not fitted to V10.

## Claim boundary

The P2-R0 controller smoke has `NO_SELECTION_AUTHORITY` and `NO_SCIENTIFIC_CLAIM`. No new controller was developed, selected, tuned, or performance-evaluated.
