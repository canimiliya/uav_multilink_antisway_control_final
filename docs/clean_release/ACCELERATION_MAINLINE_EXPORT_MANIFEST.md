# Acceleration Mainline Clean Export Manifest

This manifest is a **plan only**. P3-R1A does not copy files, create a new repository, or rewrite imports.

## Required core

| source_path | destination_path | required/optional | source_sha256 | reason |
|---|---|---|---|---|
| `src/uav_sway/v3/controllers.py` | `src/uav_sway/controllers/three_axis_traditional.py` | required | `45b61d298fe1653cb8dd294bd6928b4c53df4e3002186609d5ec3844613ab1d4` | V3CascadedTaskPID, V3FullStateLQR, V3TaskWeightedLQR |
| `src/uav_sway/v3/contracts.py` | `src/uav_sway/controllers/contracts.py` | required | `d974d6c0d937e9fcaf1301d6facd7c88648ffde75783d83a57ba801dbeb29fb4` | finite `[ax,ay,az]` contract and common limiter |
| `src/uav_sway/v3/observation.py` | `src/uav_sway/controllers/observation.py` | required | `87dbd34d914aa2319eabc20199cd2e4f4d299581c6ce3af81b17e7d90ffbd675` | causal UAV/cutter/joint state and XYZ reference mapping |
| `src/uav_sway/v5/satc_ofmpc.py` | `src/uav_sway/controllers/satc_ofmpc.py` | required | `ec95cf0820d3c386b6a02bf2ab77b011a9aae34e733aee408f871e07967e370b` | V5 frozen SATC implementation |
| `src/uav_sway/v4/cart_ofmpc.py` | `src/uav_sway/controllers/cart_ofmpc.py` | required | `385510d44e050632c63071a738d7c76d595d1434f818dd9652cc5a4a27412bff` | SATC backbone dependency |
| `src/uav_sway/v3/dr_tsrmpc.py` | `src/uav_sway/controllers/dr_tsrmpc.py` | required | `711b296cd14f968d08ac504e1553b50097692dc309631cb84079862e7458b6ed` | CART controllability-basis dependency |
| `src/uav_sway/mpc/osqp_solver.py` | `src/uav_sway/mpc/osqp_solver.py` | required | `9397a321745816526cd210734efd71bd681b34f25b1eed8b015c509eded7a4d3` | SATC QP solver dependency |
| `src/uav_sway/mpc/qp_builder.py` | `src/uav_sway/mpc/qp_builder.py` | required | `2a0547645bc1cd23baa3085eb5ccf4530f3590cb1daf90b6d47be6215d0636d6` | SATC QP data dependency |
| `src/uav_sway/control/geometric_inner_loop.py` | `src/uav_sway/controllers/geometric_inner_loop.py` | required | `bb6402b2529f438b065641a3b45519be46f634bff23d36d06ccd6c7b4d6d645f` | shared SO(3) force/thrust/torque layer |
| `src/uav_sway/control/base.py` | `src/uav_sway/controllers/base.py` | required | `4a63da3f615cc8da7efd541cd9cefba3d9541d1b97799c6b74446c179b4e37c7` | shared state/reference types |
| `src/uav_sway/control/state_reader.py` | `src/uav_sway/simulation/state_reader.py` | required | `b138c599c63240fa0672c897ea732290b49c665512e71cc736f884d467bae8d9` | MuJoCo UAV/joint state readback |
| `src/uav_sway/control/runtime_model.py` | `src/uav_sway/simulation/runtime_model.py` | required | `a15c280a04edd3435c93526080082681416dd4b54dc522fe0f5abc353f677204` | runtime model provenance/actuator audit |
| `src/uav_sway/task_space/state.py` | `src/uav_sway/tasks/task_state.py` | required | `0a2dfcd19ad72a61dff7875b805ed553fe980f810a5c42b55b171cb91e5771ad` | cutter-tip XYZ/velocity/orientation measurement |
| `src/uav_sway/task_space/reference.py` | `src/uav_sway/tasks/reference.py` | required | `6a249406f64379d1df82432ef870a65421d8b522c3caf41a877f4ee8b80a79d9` | equilibrium-derived cutter XYZ target mapping |
| `src/uav_sway/disturbances/aerodynamics.py` | `src/uav_sway/wind/aerodynamics.py` | required | `1683355121cfff6b45064f557564cb90ff54b20460b30be12f14c23b2490ce53` | distributed body-wind force model |
| `src/uav_sway/disturbances/wind_applier.py` | `src/uav_sway/wind/applier.py` | required | `c46226616f2c28dbab8fa6a0fcb667e8fbe856da8734f5f54c8ad7e5c153641a` | clears/applies per-body wind wrench |
| `src/uav_sway/disturbances/wind_profiles.py` | `src/uav_sway/wind/profiles.py` | required | `45050e94b93e56e395735d1d19455e71b99642d28dc608dc78f27e9b5fc31f64` | constant, gust, stochastic profiles |
| `src/uav_sway/disturbances/wind_io.py` | `src/uav_sway/wind/io.py` | required | `e91e4b447027a9e05529b5ca97957d18f6a8cf7c756509eb9fbe97c52d806483` | wind-series serialization |
| `src/uav_sway/models/model_config.py` | `src/uav_sway/simulation/model_config.py` | required | `61e6a9f5f11a3ff0960965b17e59972207441782fe90f61d94f0e378d3cad728` | five-link model metadata |
| `src/uav_sway/v3/metrics.py` | `src/uav_sway/metrics/lqr_metrics.py` | required | `f858b9351ae41e15dd92dc79b979d05431fd174f54f41164c0b4ac9893722642` | frozen A/B and task metric alignment helpers |
| `src/uav_sway/evaluation/task_space_metrics.py` | `src/uav_sway/metrics/task_space.py` | required | `40b90fe4e00458d113a41de8be4331a26906265577888ffb99ed446a48ebf88d` | task acquisition and task-space metrics |
| `reproducibility/frozen/model/model_5link_controlled.xml` | `models/model_5link_controlled.xml` | required | `19105873c0fcc891ebb85efe6c20c378d5b77b6bf9003559e43ae47ca03d153d` | byte-identical frozen V5 plant |
| `reproducibility/v3/r0/linear_model_audit.json` | `configs/frozen/linear_model_audit.json` | required | `47e26957c833c51e086362136dd40d61d1c991caa6b46e9e8d9e8a810afa24e5` | frozen A/B evidence for LQR/SATC |
| `reproducibility/v3/r1/task_metric_alignment_audit.json` | `configs/frozen/task_metric_alignment.json` | required | `16a155a04432667f2152823e1c5b919010da19f52557db03064ebd9dadf2fdb3` | frozen task output maps |
| `reproducibility/v3/r1r1/pid_freeze.json` | `configs/controllers/pid_freeze.json` | required | `759561998a44f41f5688dab1199daa6c47f6c4ae3586e96070910625b2f5db64` | frozen PID evidence |
| `reproducibility/v3/r1/full_lqr_freeze.json` | `configs/controllers/full_lqr_freeze.json` | required | `d98d569200dad66c0afb88042a93632c2318c1a62b92b286ba9e0bffdd8b9fa6` | frozen Full-LQR gain/evidence |
| `reproducibility/v3/r1/task_lqr_freeze.json` | `configs/controllers/task_lqr_freeze.json` | required | `0399190be129f34a1061dbee0258f160268def76b9fdc032b98b395d3a4589d9` | frozen Task-LQR gain/evidence |
| `reproducibility/v5/self/self_freeze.json` | `configs/controllers/satc_freeze.json` | required | `613e48ec12d7aaede8171ce9fdf173e5b673befb1295f4b0045126dca04ae36b` | frozen SATC `satc_b_027` parameters |
| `configs/model_5link.yaml` | `configs/model_5link.yaml` | required | `e2eaafd1a1b0066816c2150bbe42c466393cb871bfbb61366b7fa2765d730c5d` | link count/geometry metadata |
| `configs/s3_pid.yaml` | `configs/inner_loop.yaml` | required | `43fd2d9cd931e49ce6080b08b4892728f19939ca234b0e6482973c07d2ec530f` | frozen rates, limits, and inner gains |
| `configs/aerodynamics.yaml` | `configs/aerodynamics.yaml` | required | `485c399bcddbd6ba363fdc3d9ab8eb3fd0e9afbefe51a0825ef2559db6bd1a9` | wind force parameters |
| `third_party/udaan` | `third_party/udaan` | required | gitlink `9eb1a2dcfe438ce7b4c4cd119072e4f3d8a6a816` | required external SO(3) implementation; install/packaging is P3-R1B |

## Required demo support

| source_path | destination_path | required/optional | source_sha256 | reason |
|---|---|---|---|---|
| `scripts/run_v3_r1_baselines.py` | `examples/run_three_axis_baselines.py` | optional | audit at V3 freeze `759831afa0a043729a30eb79b349fedb5f829372` | reproducible headless baseline runner; not needed for core imports |
| `scripts/run_v5_self_development.py` | `examples/run_satc_development.py` | optional | V5 protocol `a728f4566b3965a6fb73531e9c81b8a63b3ac6ac` | research runner only; do not expose holdout path |
| `src/uav_sway/evaluation/task_space_metrics.py` | `src/uav_sway/metrics/task_space.py` | optional | `40b90fe4e00458d113a41de8be4331a26906265577888ffb99ed446a48ebf88d` | demo logging/metric helper |
| plotting utility | `src/uav_sway/visualization/` | optional | not present | must be implemented in P3-R1F; no existing clean utility identified |
| viewer/render utility | `examples/viewer.py` | optional | not present | must be implemented in P3-R1C; no existing clean utility identified |

## Tests

| source_path | destination_path | required/optional | source_sha256 | reason |
|---|---|---|---|---|
| `tests/v3/test_r1_protocol.py` | `tests/test_three_axis_contract.py` | required | `ee38b99f6ec22177d244577e277938a1119670ecb2387595969cc5c2a70679da` | three-axis limiter/controller contract |
| `tests/v3/test_r1r1_controller.py` | `tests/test_pid_contract.py` | required | `1a5086d3d26bfd6d1c10f93d13ea481bbc5ec38753d8ec544a6c9e1aa9a779be` | PID anti-windup and direction checks |
| `tests/v5/test_satc_controller.py` | `tests/test_satc_contract.py` | required | `d8f7c5ec97801886697d857feb770bb93a4547c984dc347b6b846bbef33e2236` | SATC frozen behavior checks |
| `tests/release/test_release_surface.py` | `tests/test_release_surface.py` | required | `033942226bb298163987cf673219373ad9dfd0990a735ea38bb58560a989d9a9` | release-surface regression checks |

## Do not export

The following are explicitly excluded (`do_not_export_count = 12` categories):

1. `src/uav_sway/native_stack/` and all native-stack evidence.
2. `src/uav_sway/v2/` through `src/uav_sway/v10/` research-only methods, except the explicitly listed V3/V4/V5 dependency files.
3. `reproducibility/v2/` through `reproducibility/v10/` raw evidence and candidate grids.
4. `reproducibility/*/holdout/` raw data and execution manifests.
5. paper PDFs, paper adaptation files, and source-audit downloads.
6. old reports, task cards, and historical project-management documents.
7. `.benchmarks/`, `*.log`, and large generated outputs.
8. debug scripts and failed-run logs.
9. V5 ablation-only scripts and traces.
10. random/20-seed evaluation runners and any Holdout entry point.
11. existing research plotting scripts until they are replaced by a clean visualization module.
12. existing viewer/render assets and third-party documentation/examples; retain only the importable `udaan` package needed by the inner loop.

No file was copied or deleted during P3-R1A.
