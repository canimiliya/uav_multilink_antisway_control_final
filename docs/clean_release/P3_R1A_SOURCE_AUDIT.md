# P3-R1A Source Audit

**Task:** `P3-R1A-ACCELERATION-MAINLINE-INVENTORY-AND-THREE-TASK-CAPABILITY-FREEZE-R1`
**Audit type:** read-only source audit and contract freeze
**Audit branch:** `release/p3-r1a-mainline-audit`
**Start head:** `150d6c125b790563c94a48cdb596f06ee12ad102`
**V5 source tag:** `v5-research-final-2026-08-09` (`60a6350d6875d4654ab881fdf8d96e749226133c`)
**Controller/model/config modifications:** none
**Performance experiment:** not executed
**Holdout:** not accessed or executed

## Formal acceleration-level mainline

The identifiable public mainline is:

```text
V3Reference / V3StateReader
    -> V3CascadedTaskPID, V3FullStateLQR, or V3TaskWeightedLQR
    -> V3AccelerationLimiter
    -> finite world-frame [ax, ay, az]
    -> GeometricInnerLoop (shared Udaan SO(3) attitude layer)
    -> desired force, thrust, body torque
    -> frozen MuJoCo UAV + five-link cutter
```

This is distinct from the later `native_stack/` family. `native_stack/` is not part of the clean mainline export.

## Component inventory

| component | source_path | source_commit/tag | status | notes |
|---|---|---|---|---|
| three-axis PID family | `src/uav_sway/v3/controllers.py` | implementation freeze `d59827abdfb89a0b1c06a0e5a54ad90795ee5d3c`; present at V5 tag | IDENTIFIED | `V3CascadedTaskPID` is the V5 Traditional `hybrid_x007_y041_z041`; `V3TaskPID` is the direct 3-D PID implementation. |
| Full-LQR | `src/uav_sway/v3/controllers.py` | implementation freeze `36fb57a92bb0d7b1a6b99680dcf4b791d54298cb`; present at V5 tag | IDENTIFIED | `V3FullStateLQR`, common 3-D limiter, frozen candidate `full_lqr_048`. |
| Task-LQR | `src/uav_sway/v3/controllers.py` | implementation freeze `36fb57a92bb0d7b1a6b99680dcf4b791d54298cb`; present at V5 tag | IDENTIFIED / AUDIT-ONLY | `V3TaskWeightedLQR`, frozen candidate `task_lqr_009`; no new selection or run in this task. |
| V3 command contract | `src/uav_sway/v3/contracts.py` | `f27a79d24689d2fd9d8444df58d3d7dae0d6ee64`; present at V5 tag | IDENTIFIED | Shared amplitude and slew limits for all three axes. |
| V3 observation/reference | `src/uav_sway/v3/observation.py` | `36fb57a92bb0d7b1a6b99680dcf4b791d54298cb`; present at V5 tag | IDENTIFIED | Causal UAV XYZ, cutter-tip XYZ, velocity, five joint states, and orientation measurement. |
| shared geometric inner loop | `src/uav_sway/control/geometric_inner_loop.py` | `606453f308bd349412d6a910b6d77de4c49bbe45`; present at V5 tag | IDENTIFIED | Converts world-frame acceleration to force and Udaan geometric thrust/body torque. |
| V5 SATC | `src/uav_sway/v5/satc_ofmpc.py` | protocol freeze `a728f4566b3965a6fb73531e9c81b8a63b3ac6ac`; V5 frozen candidate `satc_b_027` | IDENTIFIED | Depends on frozen CART-OFMPC and frozen Task-LQR/Full-LQR backbone gains. |
| SATC/CART dependency | `src/uav_sway/v4/cart_ofmpc.py`, `src/uav_sway/v3/dr_tsrmpc.py`, `src/uav_sway/mpc/osqp_solver.py`, `src/uav_sway/mpc/qp_builder.py` | V4 freeze `ff5c6ac1a02524705ff53ac106196798a6b1ca72`; present at V5 tag | IDENTIFIED | Required only when exporting SATC; not a new algorithm decision here. |
| five-link MuJoCo model | `reproducibility/frozen/model/model_5link_controlled.xml` | V5 tag; blob identical to current tree | VERIFIED | SHA-256 `19105873c0fcc891ebb85efe6c20c378d5b77b6bf9003559e43ae47ca03d153d`. |
| model/config loader | `src/uav_sway/models/model_config.py`, `src/uav_sway/control/runtime_model.py` | V1.0.0/V5 tree | IDENTIFIED | Runtime model uses the frozen XML; no model generation or mutation performed. |
| wind implementation | `src/uav_sway/disturbances/wind_profiles.py`, `wind_applier.py`, `aerodynamics.py` | V1.0.0 source plus later read-only-compatible wind extension | IDENTIFIED | Constant, one-cosine gust, low-pass stochastic profile, and distributed per-body force application are present. |
| reference/task implementation | `src/uav_sway/v3/observation.py`, `src/uav_sway/task_space/reference.py`, `src/uav_sway/task_space/state.py` | V3/V2 freezes; present at V5 tag | IDENTIFIED | Arbitrary cutter-tip XYZ target maps to UAV XYZ reference; tip position/velocity and cutter orientation are read from MuJoCo. |
| metrics | `src/uav_sway/v3/metrics.py`, `src/uav_sway/evaluation/task_space_metrics.py` | V3/V1.0.0 tree | IDENTIFIED | Research metrics and task-space metric helpers exist; no metrics run was requested here. |
| plotting utilities | project `src/`/`scripts/` search | none in clean core | NOT FOUND | Existing plots are research/evidence scripts and are excluded from Clean Export. |
| viewer/render utilities | project `src/`/`scripts/` search | none in clean core | NOT FOUND | Viewer/render is deferred to P3-R1C/P3-R1F; no demo was created. |

## Frozen controller identifiers

| method | implementation | frozen identifier | parameter evidence |
|---|---|---|---|
| PID | `V3CascadedTaskPID` | `hybrid_x007_y041_z041` | `reproducibility/v5/holdout/holdout_controller_summary.json` and V5 `method_status.json`; used only as provenance, not rerun. |
| Full-LQR | `V3FullStateLQR` | `full_lqr_048` | `reproducibility/v3/r1/full_lqr_freeze.json`; no gain modification. |
| Task-LQR | `V3TaskWeightedLQR` | `task_lqr_009` | `reproducibility/v3/r1/task_lqr_freeze.json`; audit-only in this task. |
| SATC | `SATCOFMPC` | `satc_b_027` | `reproducibility/v5/self/self_freeze.json`; V5 frozen implementation and parameter record. |

## Model provenance

- Path: `reproducibility/frozen/model/model_5link_controlled.xml`
- SHA-256: `19105873c0fcc891ebb85efe6c20c378d5b77b6bf9003559e43ae47ca03d153d`
- Bodies: UAV mass `9.74 kg`, five dynamic links of `0.20 kg` each, cutter `2.50 kg`; total `13.24 kg`.
- Joints: `joint_1` through `joint_5`, each hinge, with qpos addresses `[7, 8, 9, 10, 11]` and qdot addresses `[6, 7, 8, 9, 10]`.
- Comparison against the V5 tag is byte/blob-identical; `MODEL_PROVENANCE_MATCH=true`.

## Static capability conclusion

- `MOVE_TO_POSITION_AND_SETTLE`: **SUPPORTED at code-contract level**. `reference_for_target()` accepts arbitrary XYZ, the V3 outer controllers emit a finite three-vector, and the cutter-tip position/velocity are observable. No performance claim is made.
- `MOVE_TO_POSITION_ORIENTATION`: **NOT_SUPPORTED as an independent control channel**. Orientation is measured (`cutter_axis_world`, `cutter_rotation_world`) but no orientation command is present.
- Orientation class: **A — `CUTTER_XYZ_ONLY`**.
- `INITIAL_SWAY_RECOVERY`: **SUPPORTED for initialization/smoke**. Non-zero qpos and qdot are addressable for all five joints; no performance experiment was run.
- `WIND_HOVER_RECOVERY`: **SUPPORTED for callback/data availability**. Constant wind, one-cosine gust, low-pass stochastic wind, and distributed body forces are present; no recovery benchmark was run.

## Scope and non-actions

`controller` parameters, model XML, configs, `native_stack/`, historical evidence, paper files, and holdout data were not modified. No file was deleted, no public repository was created, and no performance/holdout run was executed.
