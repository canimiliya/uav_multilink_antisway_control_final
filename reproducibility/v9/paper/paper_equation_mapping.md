# Jin et al. (2025) Equation Mapping

## Source dynamics and labels

- Equation (1) defines quadrotor translation and rotation with external
  payload/residual force f_e and torque tau_e. V9 reconstructs these labels
  offline from measured finite-difference acceleration, actual applied
  thrust/torque, and known nominal rigid-body dynamics. Simulator wind truth is
  excluded.
- Equation (8) treats chi = [f_e, tau_e] as an unknown dynamical state driven
  by a measured flight substate zeta.

## Lifted linear predictor

- Equations (3)-(7) map nonlinear dynamics into z_(k+1) = A z_k + B zeta_k,
  where z = Phi(chi).
- Equation (9) defines the ReLU neural embedding. V9 retains a feed-forward
  neural embedding; no recurrent, Transformer, diffusion, or SATC component is
  introduced.
- Algorithm 1 and Equation (10) define offline embedding/LLS learning.
  Equations (11)-(12) supply forward/backward multi-step and reconstruction
  losses. V9 selects predictors only on trajectory-held-out prediction data.
- Equations (13)-(16) bound accumulated prediction error under stable lifted
  dynamics and bounded local error.
- Equations (17)-(20) motivate per-layer spectral normalization and the
  embedding Lipschitz bound. V9 applies spectral normalization to every hidden
  and embedding linear layer and records the learned lifted spectral radius.

## NP-MPC integration

- Equation (21) is the controlling identity: predicted state equals nominal
  dynamics plus a mapped learned force/torque prediction, inside a constrained
  receding-horizon optimization.
- V9 uses the frozen 20-state discrete nominal plant, maps predicted
  translational residual acceleration into that model, and optimizes the
  physical acceleration sequence subject to absolute 2.0 m/s2 authority and
  0.25 m/s2/update slew. The predictor is propagated across the horizon rather
  than added after an LQR command.
- The paper's learned torque state is trained, validated, and available to the
  internal attitude-model diagnostic. It receives no extra torque actuator
  authority; the final command remains [ax, ay, az] at 20 Hz through the common
  geometric inner loop.
- The paper estimates A and B online from a rolling 40-sample sequence. V9
  freezes the neural embedding offline and permits rolling least-squares
  updates of lifted matrices using only causal measured history.

This is ADAPTED_NOT_EXACT_REPRODUCTION: the Neural Predictor/LLS and
learned-dynamics-in-MPC identity are retained, while thrust/torque actuation is
restricted to the benchmark's common acceleration interface.
