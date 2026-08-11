# Import Dependency Audit

This is an import inventory only. No import was rewritten in P3-R1A.

| module | imports | clean_export_safe | problem |
|---|---|---|---|
| `src/uav_sway/v3/controllers.py` | numpy; local `v3.contracts`, `v3.observation` | yes | none |
| `src/uav_sway/v3/contracts.py` | numpy; stdlib typing/dataclasses | yes | none |
| `src/uav_sway/v3/observation.py` | mujoco, numpy, `task_space.state` | conditional | requires MuJoCo and the local task-space state module |
| `src/uav_sway/v3/metrics.py` | mujoco, numpy, scipy | yes | scipy is declared in `pyproject.toml` |
| `src/uav_sway/v4/cart_ofmpc.py` | numpy; local `mpc`, `v3` modules | conditional | SATC export must include the OSQP/QP dependency closure |
| `src/uav_sway/v5/satc_ofmpc.py` | numpy; local `v3`/`v4` modules | conditional | requires CART-OFMPC and its QP dependency closure |
| `src/uav_sway/control/geometric_inner_loop.py` | numpy; `udaan.control.quadrotor`; `udaan.manif`; local task-space modules | conditional | `udaan` is a checked-out git submodule but is not installed by the current environment and is not listed in the root `pyproject.toml`; P3-R1B must package/install it. |
| `src/uav_sway/disturbances/aerodynamics.py` | mujoco, numpy, pyyaml | yes | all root dependencies are declared |
| `src/uav_sway/disturbances/wind_profiles.py` | numpy, pyyaml | yes | none |
| `src/uav_sway/task_space/state.py` | mujoco, numpy | yes | none |
| `src/uav_sway/evaluation/task_space_metrics.py` | numpy; local metrics | yes | none |
| `scripts/run_v5_self_development.py` | research runners and `reproducibility/v3..v5` artifacts | no | research-only runner; do not export as clean core |
| `src/uav_sway/native_stack/**` | governance/native-stack modules | no | explicitly excluded from Clean Export |

### Blocking-dependency interpretation

The only packaging issue found is the known `udaan` third-party dependency. It is not a controller ambiguity and does not require parameter changes. It is recorded for P3-R1B; no import repair was performed here.

No clean-core import resolves an absolute local path, old experiment folder, or native-stack artifact.
