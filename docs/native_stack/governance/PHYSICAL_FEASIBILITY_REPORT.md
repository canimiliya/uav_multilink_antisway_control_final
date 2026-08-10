# Physical feasibility report

## Two complementary checks

The analytical authority audit uses frozen model and manifest values only. It shows substantial thrust, force, torque, distance, and duration reserve.

The certifying diagnostic then solves a bounded nonlinear endpoint equilibrium for each of the 200 Development identities. Five joint angles, roll, pitch, and the canonical four-axis wrench are bounded by the unchanged SafetyV2/actuator envelope. The free base is translated so the actual MuJoCo cutter-tip site equals the exact frozen target. The maximum generalized-acceleration residual was `4.90e-13`.

All 200 cases passed endpoint geometry, actuator, equilibrium, and conservative transfer-time checks. Nominal and challenge cohorts, and setpoint and smooth-trajectory families, each achieved 100% certification.

## Failed dynamic attempt retained

The earlier fixed-gain noncausal dynamic attempt is preserved as `ORACLE_IMPLEMENTATION_NOT_CERTIFYING`. It had only 2.5% safety and hit torque limits, so its failures diagnose that oracle law, not physical infeasibility. No parameter retuning or rerun was used to overwrite it.

The equilibrium certificate does not claim trajectory-level optimal control, causal realizability, or Traditional competence. It answers only whether the endpoint and mission time remain physically available under the frozen authority.
