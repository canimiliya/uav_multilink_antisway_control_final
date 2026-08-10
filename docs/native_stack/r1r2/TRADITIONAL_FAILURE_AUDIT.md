# P2-R1R2 Traditional failure audit

This audit uses only frozen P2-R1R1 Development results; it executes no new controller cases.

| Family | Selected | Safety | Success | Position-pass/speed-fail | Position-fail/speed-pass | Both fail |
|---|---|---:|---:|---:|---:|---:|
| native_pid | `native_pid_001` | 100.0% | 27.5% | 10.0% | 34.0% | 28.5% |
| native_full_lqr | `native_full_lqr_006` | 96.0% | 12.0% | 1.5% | 44.0% | 42.5% |
| native_task_lqr | `native_task_lqr_004` | 90.0% | 13.0% | 0.5% | 58.5% | 28.0% |

## Mechanism conclusion

Native PID is already safe and meets the aggregate RMSE gates, but most non-successes are endpoint-regulation failures. The recovery therefore needs stronger causal steady-state/servo action and reference feedforward, not a relaxed success definition.

The physical Full-LQR model is controllable and stabilizable; its loss is a reference-servo problem, not an inability of LQR to stabilize. Full-LQR recovery must add equilibrium shifting, feedforward, and integral augmentation while retaining linear-quadratic identity.

Native Task-LQR traded the historical adapter's unsafe tracking behavior for safety, but lacks sufficient task endpoint regulation and trajectory feedforward. Task-LQR recovery must preserve the native safety gain while adding LQT/LQI task tracking.

Detailed task, wind, direction, target-bin, and failure-class counts are frozen in `traditional_failure_mechanism_audit.json`.
