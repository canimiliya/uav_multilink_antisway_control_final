# Native PID recovery

Two PID/PD servo forms and 96 unique configurations were evaluated.

Selected R1R2 recovery candidate: `r1r2_pid_c_0_01`.

- Safety: 93.5%
- Success: 13.0%
- Setpoint RMSE: 1.259619 m
- Trajectory RMSE: 0.535861 m
- Strong P90: 1.602494 m
- Catastrophic count: 13
- Deadline miss rate: 0.000236

Result: **FAIL competence**. Stage C improved safety over the broad initial search, but did not restore endpoint success; the frozen R1R1 Native PID remains stronger.
