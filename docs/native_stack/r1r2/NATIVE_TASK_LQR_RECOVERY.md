# Native Task-LQR recovery

An output-weighted task LQT/LQI form was evaluated over 96 unique configurations.

Selected R1R2 recovery candidate: `r1r2_task_lqr_c_0_04`.

- Safety: 0.0%
- Success: 0.0%
- Setpoint RMSE: 4.892444 m
- Trajectory RMSE: 4.140531 m
- Strong P90: 8.039782 m
- Catastrophic count: 200
- Deadline miss rate: 0.000036

Result: **FAIL competence**. Feeding cutter-tip output errors into the hover-model servo did not preserve the R1R1 native safety improvement; the output/model mismatch was catastrophic across the full bank.
