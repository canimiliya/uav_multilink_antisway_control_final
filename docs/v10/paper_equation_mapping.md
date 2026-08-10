# V10 Xu FxTDO-MPC equation and adaptation mapping

Primary source: Liwen Xu et al., *Fixed-Time Disturbance Observer-Based MPC Robust Trajectory Tracking Control of Quadrotor*, arXiv:2408.15019v2 (30 August 2024). The open PDF and TeX bundle were both audited. This is an `ADAPTED_NOT_EXACT_REPRODUCTION`.

## Source equations

- Eq. (1b): `v_dot = known thrust/gravity + f_d / m`; `f_d` includes aerodynamic drag, wind and unknown payload effects.
- Eq. (5): `z1_dot = T + f_d`, with `z1 = m v`.
- Eq. (6): `zhat1_dot = fhat_d + T + L1 phi1(e1)` and `fhat_d_dot = L2 phi2(e1)`.
- Eq. (7): `phi1` combines signed powers `1/2`, `1`, and `1/(1-d_inf)`; `phi2` combines signed powers `0`, `1`, and `(1+d_inf)/(1-d_inf)`.
- Eqs. (9), (20)-(22): positive gains, `0 < d_inf < 1`, `L2 > delta_dot_bar/k2`, and a sufficiently large `L1` yield a fixed convergence-time bound independent of initial error.
- Eq. (23): the current disturbance estimate is injected into the prediction model and held constant across the current MPC horizon.
- Eqs. (24)-(27): state/input tracking cost, terminal cost, state/input constraints and receding-horizon solution.

## Fair five-link mapping

The benchmark acceleration interface already absorbs the common thrust/gravity conversion. Therefore V10 uses the mass-normalized world-frame form `v_dot = a_actual + d`, with `z1 = v`, `T = previous_actual_limited_acceleration`, and `fhat_d/m = dhat`. The same causal three-axis FxTDO equations are integrated at the 20 Hz information rate with declared internal numerical substeps.

The paper's 10D `[p,v,q]` model becomes the frozen 20D five-link error model. Since `dhat` and the command are both world accelerations, the discrete prediction is `x[k+1] = A x[k] + B (u[k] + compensation_scale*dhat)`. The estimate is constant over each horizon. The cost uses `C_task_v3 x` for cutter-tip position/velocity/orientation/angular-rate errors plus absolute control, control-rate and terminal costs. Every horizon move obeys `|u_i| <= 2.0 m/s^2` and `|Delta u_i| <= 0.25 m/s^2/update`.

No single/double-pendulum or modal reduction is introduced. Wind, five-link reaction and unmodelled nonlinear coupling remain one translational lumped disturbance.

## Adaptation distance

| Component | Status | V10 treatment |
|---|---|---|
| Multivariable FxTDO | PRESERVED | Three coupled-vector equations evaluated componentwise with vector errors |
| Bi-homogeneous `phi1/phi2` | PRESERVED | All three signed-power terms retained |
| Causal disturbance estimate | PRESERVED | Current/past velocity and previous applied command only |
| Disturbance injection into MPC | PRESERVED | Constant over each prediction horizon |
| Receding-horizon constraints | PRESERVED | Absolute acceleration and slew constraints at every step |
| Paper `[p,v,q]` state and tracking output | ADAPTED | Frozen 20D five-link state and `C_task_v3` task output |
| Source 100 Hz MPC | ADAPTED | Fair 20 Hz outer rate; numerical substeps do not add sensor information |
| Force units | ADAPTED | Mass-normalized acceleration disturbance |
| Torque FxTDO and INDI | OMITTED | Common benchmark has no independent torque authority; common geometric inner loop remains frozen |

## Runtime information barrier

Allowed: current/past UAV position and velocity, measured current plant state, previous actual limited acceleration, and current/past reference. Forbidden: true wind, MuJoCo hidden force, future state/reference/wind, or SATC internal variables. Ground-truth equivalent disturbance may only be reconstructed offline for RMSE/settling/bias diagnostics.
