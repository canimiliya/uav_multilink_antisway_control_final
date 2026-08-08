# V2 Research Contract

Status: `V2_RESEARCH_CONTRACT_FROZEN`

This contract is based on the public `v1.0.0` release and applies only to the
`research-v2` branch. `main` and `v1.0.0` are read-only benchmark baselines.
No controller implementation, controller tuning, large raw run, or paper
selection is part of V2-R0.

## Research questions

1. `CALM_3D_SETPOINT`: from the frozen equilibrium, reach a nearby 3-D
   cutter-tip position with zero target tip velocity and zero target cutter
   angular velocity while retaining the equilibrium cutter direction.
2. `WIND_3D_SETPOINT`: perform the same nearby 3-D setpoint task under a
   frozen distributed wind, including constant development wind and unseen
   stochastic holdout wind. A controller may not read future wind truth.
3. `RAMP_WIND_EQUILIBRIUM_HOLD`: start at the equilibrium target and increase
   wind from zero; keep cutter-tip position and equilibrium orientation close
   to their targets rather than moving to a new target.

The plant is the frozen MuJoCo 6-DoF UAV plus 5-link suspended chain. The
nearby target is a terminal setpoint, not a pre-generated flight trajectory.
The controller receives the endpoint and current causal state only.

## Local 3-D justification

V2-R0 evaluates the world-frame position map of the `cutter_tip` site at the
frozen equilibrium. The full MuJoCo site Jacobian is recorded in
`reproducibility/v2/controllability_audit.json`. The first three columns are
the UAV translational coordinates and are an identity matrix at this
equilibrium; therefore the position-output rank is 3. UAV attitude columns
are also recorded. This justifies a local 3-D position task, not arbitrary
global reachability, arbitrary orientation control, or dynamic replanning.

## Terminal acquisition

`task_acquired=true` only when all conditions hold continuously for at least
1.0 s:

- 3-D cutter-tip position error `<= 0.05 m`;
- cutter-tip speed `<= 0.10 m/s`;
- cutter direction error from the equilibrium direction `<= 5 deg`;
- cutter angular speed `<= 0.10 rad/s`.

These thresholds are fixed before V2 controller development and may not be
relaxed after a method fails.

## Fair comparison

The mandatory traditional baselines are Cascaded PID/PD, Full-State LQR, and
Task-Space LQR. Each method must use the same plant, actuator limits, wind,
task samples, terminal gate, and safety gate, and must be tuned seriously on
development data. LS-PMPC-v1 may be retained only as a legacy advanced
reference.

Later work must include one self-developed advanced method and one fair
adaptation of a 2024--2026 paper method. V2-R0 deliberately does not select
the paper or tune any advanced method.

## Development and holdout

Development samples are the only samples allowed for controller and parameter
selection. Holdout samples are locked before execution but are not run during
V2-R0. The holdout changes target direction, stochastic wind seed, and part
of the wind-strength set. Every method uses exactly the same holdout samples;
no method may tune on holdout results.

The machine-readable split, sample families, and seed namespaces are in
`reproducibility/v2/data_split_contract.json`.

## Advanced-method win rule

The advanced method is compared with the strongest traditional baseline, not
only with PID. On common paired holdout samples it must have safety no worse,
task success rate no lower, and a clear acquisition-time improvement among
common successes. It must also show a clear 3-D cutter-position improvement
and improve at least one peak or steady-state task error in the ramp-wind
hold task. Paired holdout statistics are required. No percentage threshold is
invented in V2-R0; it is fixed only after V2-R1 baseline statistics exist and
before advanced holdout evaluation.
