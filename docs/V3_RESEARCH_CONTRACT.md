# V3 Research Contract

V3-R0 freezes a fair full-3D benchmark contract from `v2-research-final-2026-08-09`. It does not implement, tune, or benchmark a controller.

## Frozen boundary

The MuJoCo five-link plant is unchanged and has SHA-256 `19105873c0fcc891ebb85efe6c20c378d5b77b6bf9003559e43ae47ca03d153d`. All five passive joints use a y-axis hinge, so suspended-chain swing is primarily in the x-z plane; this is not a universal-joint spatial cable model. The cutter-tip translation task remains three-dimensional.

## Common authority

Every V3 method must return the same world-frame acceleration command `[ax, ay, az]` through `V3AccelerationCommand`, with per-axis amplitude limit 2.0 m/s² and per-update slew limit 0.25 m/s² at a 0.05 s outer period. The same Udaan geometric inner loop and plant safety limits are used for every method.

## Local audit

The frozen-equilibrium finite-difference audit uses a 20-state model with yaw/yaw-rate decoupling checked before omission. It produces `A3 in R^(20x20)`, `B3 in R^(20x3)`, rank(B3)=3, task-position rank 3, and a PBH stabilizability check for all modes with `|lambda| >= 1`. Full-state controllability is diagnostic only, not a hard gate.

## Samples and evaluation

Development has 18 multi-axis targets, wind speeds 1.5 and 3.0 m/s, seeds 2000--2019, and a 0--3.0 m/s ramp. A new deterministic 12-direction spherical holdout uses PCG64 seed 20260809, wind speeds 2.0 and 3.5 m/s, seeds 3000--3019, and a 0--3.5 m/s ramp. Holdout execution is forbidden until all methods are frozen. V2 holdout is archival and is not reused.

The terminal gate remains position <=0.05 m, tip speed <=0.10 m/s, orientation <=5 degrees, angular speed <=0.10 rad/s, and continuous hold >=1.0 s. V3-R1 will establish the three 3D traditional baselines; Self and Paper methods are deliberately not selected in R0.

See the machine-readable files in `reproducibility/v3/r0/` for the complete contract and audit matrices.
