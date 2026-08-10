# Native Full-LQR recovery

A physical 12-state/4-input LQR with three LQI servo states was evaluated over 96 unique configurations. The augmented linear model is controllable/stabilizable, but that property did not imply nonlinear mission competence.

Selected R1R2 recovery candidate: `r1r2_full_lqr_c_0_04`.

- Safety: 36.5%
- Success: 16.5%
- Setpoint RMSE: 13.935711 m
- Trajectory RMSE: 1.660154 m
- Strong P90: 21.172407 m
- Catastrophic count: 127
- Deadline miss rate: 0.000033

Result: **FAIL competence**. The dominant issue is physical-wrench servo/model mismatch and unsafe nonlinear tracking, not lack of linear controllability.
