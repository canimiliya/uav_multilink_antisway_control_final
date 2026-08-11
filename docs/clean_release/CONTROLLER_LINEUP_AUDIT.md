# Public Controller Lineup Audit

Task: `P3-R1G-VISUAL-POLISH-AND-CONTROLLER-LINEUP-AUDIT-R1`.

The audit is presentation-only. Frozen controllers, model, scenarios, and metrics were not changed; no Holdout or Native-Stack execution was used.

## Frozen public showcase scenarios

- T1: large-sway transfer, 5.0 s move, zero wind.
- T2: the same transfer with world +X wind at 3.0 m/s, onset 3.0 s, ramp 3.0--4.0 s.
- T3 remains historical only: LQR 3 m/s and SATC 5 m/s.

## Controller decisions

| Controller | Class | ID | Source | Config/evidence | Runnable on frozen T1 | Included |
|---|---|---|---|---|---:|---:|
| PID | `V3CascadedTaskPID` | `hybrid_x007_y041_z041` | `src/uav_sway/v3/controllers.py` | `configs/s3_pid.yaml` / `reproducibility/v3/r1r1/pid_freeze.json` | True | True |
| Full-LQR | `V3FullStateLQR` | `full_lqr_048` | `src/uav_sway/v3/controllers.py` | `configs/lqr.yaml` / `reproducibility/v3/r1/full_lqr_freeze.json` | True | True |
| SATC | `SATC-OFMPC` | `satc_b_027` | `src/uav_sway/v5/satc_ofmpc.py` | `reproducibility/v5/self/self_freeze.json` / `reproducibility/v5/self/self_freeze.json` | True | True |

All included controllers satisfy the acceleration-mainline, identifiable frozen provenance, frozen-scenario runnable, and non-debug/non-Native-Stack rules. The PID video is a supplementary runnable showcase; its T1 stability status is reported from the frozen run and is not converted into a new science conclusion.

`PUBLIC_SHOWCASE_SET` is intentionally limited to the three audited candidates above; no extra controller was claimed.
