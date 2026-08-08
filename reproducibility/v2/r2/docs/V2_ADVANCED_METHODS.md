# V2 Advanced Methods Contract

This document freezes the method choices before any Advanced performance run.

## Frozen traditional boundary

`reproducibility/v2/r1r1/**` is read-only. The primary traditional reference remains PID `pid_005`; the Task-LQR reference remains `task_lqr_001`. The R1R1 sample bank, holdout protocol, shared Y/Z controller, terminal gate, safety gate, and advanced win rule are unchanged.

## Advanced-Self: OF-TSRMPC

OF-TSRMPC means Offset-Free Task-Space Residual Model Predictive Control. It competes only in the x anti-sway channel. Y/Z continue to use the frozen R1R1 cutter-tip task PD.

The frozen 16-state model is `x[k+1] = A x[k] + B u[k]`. The nominal stabilizing backbone is the frozen Task-LQR `task_lqr_001`. The registered control law is:

`a_x = u_ss_hat - K_task x + v_mpc`.

The bias model is an output disturbance, not a physical matched-wind model:

`d[k+1] = d[k]`, `y_task[k] = C_task x[k] + d[k]`.

The internal steady-state optimizer solves for `x_s, u_s` under `|u_s| <= 2 m/s^2` while preserving the external cutter target. The residual MPC uses `dt=0.05 s`, `H=20`, and no future wind or target information. Its cost includes tip-x position/velocity, cutter orientation and angular velocity, residual acceleration, acceleration rate, and a terminal penalty from the frozen Task-LQR Riccati matrix.

The only registered self grid is 36 candidates: beta `[0.05, 0.15, 0.30]`, position weight `[40, 80, 160]`, orientation weight `[5, 20]`, residual `R` `[0.5, 2.0]`, with horizon fixed at 20. It may not be expanded after this commit.

## Advanced-Paper: LV2026-CASCADE-ADAPTED

The selected primary source is [Lv et al. 2026](https://arxiv.org/html/2601.03386). The paper's full text was reviewed: the off-center model, control-oriented equations, inner attitude controller, middle suspension-point acceleration swing controller, decoupler, outer load-velocity controller, local exponential-stability arguments, and simulation/ground/flight evidence are present.

The adaptation is explicitly not an exact reproduction. It constructs a causal equivalent swing vector from the suspension point to the cutter CoM:

`r_eq = p_cutterCOM - p_suspension`,

then defines `alpha_eq`, `beta_eq`, and their causal rates. The preserved core is the paper's idea that suspension-point acceleration regulates load swing. The adaptation does not add a camera, tension sensor, slack-mode oracle, obstacle map, future wind, or future target.

ES-HPC-MPC and SEP-NMPC were reviewed as alternatives but not selected because their core guarantees depend on hybrid slack/taut plus camera FoV constraints, or point-mass/massless-cable plus obstacle HOCBF assumptions, respectively.

The paper method starts from its original parameters and may use at most 0.5x/1x/2x adaptation values per major gain, with at most 27 candidates. Any future parameter grid must be frozen before V2-R4 performance.

## Frozen order and decision rule

V2-R3 runs Advanced-Self development only. V2-R4 runs Advanced-Paper development only after Self is frozen. V2-R5 is the single final Holdout after both methods are frozen. Neither method may be retuned after seeing the other's results.

The final win rule is inherited unchanged from R1R1: 100% safety, at least 71.9298% success, at least 5% 3D RMSE improvement over 0.113688 m, at least 5% acquisition improvement over 2.30 s, and at least one 5% ramp improvement. Final PASS is decided only by frozen-Holdout paired statistics with 95% bootstrap confidence.
