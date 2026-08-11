# Task Capability Contract

**Contract:** P3-R1A code-level capability freeze
**Performance execution:** `false`
**Orientation class:** `CUTTER_XYZ_ONLY`

| field | frozen value |
|---|---|
| `MOVE_TASK_POSITION` | `SUPPORTED` (code-contract level; no performance claim) |
| `MOVE_TASK_ORIENTATION` | `NOT_SUPPORTED` as an independent command/control channel |
| `SUPPORTED_POSITION_DIMENSIONS` | `[x, y, z]` |
| `SUPPORTED_ORIENTATION` | `CUTTER_XYZ_ONLY`; orientation is measured but not independently controlled (`MEASURED_BUT_NOT_INDEPENDENTLY_CONTROLLED`) |
| `INITIAL_SWAY_INITIALIZATION` | `SUPPORTED` for five non-zero joint qpos values and qdot readback; smoke passed |
| `WIND_CONSTANT` | `SUPPORTED`; `constant_crosswind`, onset `4.0 s` |
| `WIND_GUST` | `SUPPORTED`; `one_cosine_gust`, onset `5.0 s`, duration `2.0 s` |
| `WIND_STOCHASTIC` | `SUPPORTED`; `low_frequency_random` low-pass Gaussian profile with explicit seed |
| `CUTTER_TIP_POSITION_MEASUREMENT` | `SUPPORTED`; MuJoCo `cutter_tip` site |
| `CUTTER_TIP_VELOCITY_MEASUREMENT` | `SUPPORTED`; site Jacobian, not finite-difference logging |
| `JOINT_STATE_MEASUREMENT` | `SUPPORTED`; `q1..q5` and `qdot1..qdot5` |
| `REFERENCE_GENERATOR` | `SUPPORTED` for arbitrary XYZ target mapping through `reference_for_target()`; existing scenario trajectory helper remains research-specific |
| `SHARED_INNER_LOOP` | `GeometricInnerLoop` with Udaan SO(3), desired force, thrust, body torque |
| `OUTER_OUTPUT` | finite world-frame `[ax, ay, az]`, amplitude and slew limited |
| `KNOWN_LIMITATIONS` | no independent orientation/yaw command; no viewer/render utility in clean core; no performance qualification in this task; Udaan must be packaged/installed in the next export task |

The contract deliberately does not claim full 6-DoF cutter pose control.
