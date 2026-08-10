# V10 final technical report

## Outcome

V10 completed the final preregistered external-paper route and **failed** the frozen Development gate. No V10 Holdout run was authorized or executed. The project therefore does not satisfy `Recent Paper > Traditional`, and `PROJECT_RESEARCH_COMPLETE` remains false. The external-paper search is permanently closed and V11 must not start automatically.

## Paper and implementation

The sole paper was Xu et al., *Fixed-Time Disturbance Observer-Based MPC Robust Trajectory Tracking Control of Quadrotor* (arXiv:2408.15019v2). V10 preserved the multivariable bi-homogeneous FxTDO, causal lumped-disturbance estimate, constant disturbance injection across each horizon, and constrained receding-horizon MPC. It adapted the paper to the frozen 20D five-link model and task output, mass-normalized force to world acceleration, and ran at the fair 20 Hz outer rate. Torque FxTDO/INDI were omitted because the common benchmark does not grant independent torque authority. Fidelity is `ADAPTED_NOT_EXACT_REPRODUCTION`.

## Observer diagnostic

Offline replay used 48 frozen V9 validation trajectories and 10,957 valid safe steps. MuJoCo-reconstructed force truth was used only after execution for audit. The estimate was finite and bounded, with all-axis acceleration RMSE `0.296522 m/s^2`, axis RMSE `[0.10570312076794246, 0.4870596771400852, 0.12399502393715892]`, post-1 s RMSE `0.309140 m/s^2`, and maximum absolute estimate `1.000 m/s^2`. The empirical fixed-time settling claim was not validated.

## Development evidence

The search evaluated 64 unique configurations: 768 Stage-A core runs, then 2,304 full-Development runs for the preregistered top 16. There were no performance-driven retries. The best formal candidate was `fxtdo_v10_020`.

| Metric | FxTDO-MPC | Full-LQR | SATC |
|---|---:|---:|---:|
| Safety | 1.0000 | 1.0000 | 1.0000 |
| Success | 0.0000 | 0.2500 | 0.6042 |
| Position mean (m) | 15.445775 | 0.147030 | 0.102094 |
| Position P90 (m) | 29.392419 | 0.212558 | 0.145779 |
| Orientation mean (deg) | 5.657011 | 0.921252 | 0.747164 |
| Effort | 40.299640 | 1.212330 | 1.091294 |
| Solve P95 (ms) | 1.1611 | 0.0372 | 2.8391 |

The candidate passed only safety and the advanced runtime option (2/9 gates). Success was zero. Position improvement versus Full-LQR was `-10405.17%`; this negative number means severe degradation. The paired 10,000-resample bootstrap mean delta was `-15.298745 m` with 95% CI `[-16.581971383704783, -14.000863820273086]` and positive-pair fraction `0.000`. All 72 strong cases were catastrophic by the frozen pair rule.

The likely mechanism is under-authoritative finite-horizon correction for the frozen discrete model: the selected Q/R/horizon combinations generated insufficient or poorly scaled action under wind, while the disturbance estimate did not demonstrate the paper's empirical fixed-time settling. This is an evidence-based diagnosis, not a new gate or permission to retune.

## Final boundary

V10 did not unlock Paper freeze, Paper-vs-SATC formal comparison, or the one-shot 112-case Holdout. No completed-version resume text was generated. The highest supported project claim remains the frozen V5 `V5_SELF_OVERALL_HOLDOUT_WIN`. Continued paper replacement would violate the final-route contract and reduce research credibility.
