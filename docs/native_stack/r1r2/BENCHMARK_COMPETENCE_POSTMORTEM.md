# Benchmark competence postmortem

## Formal result

`P2_NATIVE_TRADITIONAL_RECOVERY_FAILED`

Each Traditional family evaluated 96 unique configurations under the frozen Benchmark v1.1 Development bank and unchanged competence gate. Zero families passed; therefore the failure is now eligible for owner-level review, but this task does not change the gate.

## Mechanisms

- Native PID: the R1R1 controller was safe and met RMSE gates, but endpoint regulation dominated failure. Broader gains reduced safety; bounded local contraction recovered some safety but not the 70% endpoint success requirement.
- Native Full-LQR: 12-state controllability and 15-state LQI stabilizability were verified, yet direct physical-wrench tracking produced nonlinear safety loss and large setpoint tails. Linear stabilizability is not task competence.
- Native Task-LQR: task-output errors were not dynamically consistent with the hover-model position channels. Integral augmentation amplified this mismatch and failed safety across the full Development bank.
- Smooth trajectories mainly require causal velocity/acceleration feedforward; setpoints mainly require trim-correct steady-state endpoint regulation. A single direct hover-servo formulation did not satisfy both under the frozen mission envelope.

## Governance conclusion

The 70% success gate, plant, runner, metrics, and case semantics were not changed. SATC, Equal-Rate, Holdout, and Paper were not run. Any reconsideration of mission-envelope/gate structural compatibility is a separate owner-authorized task.
