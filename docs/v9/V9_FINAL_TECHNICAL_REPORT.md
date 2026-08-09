# V9 Final Technical Report

## Outcome

V9 is closed at the preregistered predictor competence gate with `BLOCKED_V9_NEURAL_PREDICTOR_NOT_VALIDATED`. The route did not enter NP-MPC implementation, the inherited 144-sample Development bank, or the locked 112-sample Holdout. Consequently, `PAPER_GT_TRADITIONAL=false` and `PROJECT_RESEARCH_COMPLETE=false`.

## Official Method Reproduction

The fixed source is Jin et al. (2025), *Neural Predictor for Flight Control with Payload*, IEEE Robotics and Automation Letters, DOI `10.1109/LRA.2025.3573624`, official repository commit `e2ababf519e7b7cca8e23f42dcb8c34fae927037`. The planned five-link adaptation is `JIN2025-NP-MPC-ADAPTED-5LINK`.

In an isolated CUDA environment, the official dataset loading, LLS training, spectral normalization, multi-step prediction, serialization, pretrained wrapper loading, numerical evaluation, and two-epoch training smoke all ran. Two minimal compatibility repairs were documented: bypassing an unused truncated `lifting_func.pth` while retaining the complete wrapper used by evaluation, and changing TensorBoard scalar logging for Windows path compatibility. The complete wrapper's recomputed mean errors (`0.07492 N` force and `0.00623 Nm` torque) did not exactly reproduce the separately published CSV (`0.12362 N` and `0.00808 Nm`). This upstream inconsistency is preserved rather than hidden.

## Frozen Identification Evidence

Before model fitting, V9 froze 240 Full-LQR trajectories (57,840 timesteps), split by whole trajectory into 192 TRAIN and 48 VALIDATION trajectories. The bank uses only measured state, finite differences, applied commands, and known nominal dynamics to reconstruct residual force/torque labels. Wind truth is neither stored as a model input nor available at runtime. Unsafe tails remain in the audit bank but are excluded from fitting through the frozen `label_valid AND safe_step` mask.

## Predictor Gate

Twenty-four preregistered Neural Predictor configurations were evaluated without SATC, Development, or Holdout performance. The diagnostic best model, `np_v9_14`, uses an 18-dimensional embedding, three 96-unit hidden layers, a stable lifted `A/B` model, reconstruction, forward/backward objectives, spectral normalization, and the paper's causal 40-sample online update concept.

On the frozen validation split it achieved force RMSE `0.87555`, torque RMSE `0.19361`, all-channel RMSE `0.89670`, and multi-step RMSE `1.78086`; its lifted spectral radius was `0.99282`. Against the best simple low-pass estimator, force RMSE was 10.50% worse and all-channel RMSE was 8.53% worse, although torque RMSE improved by 17.30%. The contract required both force and torque RMSE to improve by at least 5%, with stable lifted dynamics. The model therefore failed competence.

## Scientific Boundary

The failure is specific to this frozen five-link, 20 Hz adaptation and does not establish that the original method is ineffective on its source platform. No NP-MPC controller exists, no Paper controller configuration was evaluated, and no Paper-vs-Traditional or Paper-vs-SATC performance claim is permitted. The previously validated Traditional controllers and `satc_b_027` are unchanged. The highest valid project claim remains `V5_SELF_OVERALL_HOLDOUT_WIN`, within a pure-simulation boundary.
