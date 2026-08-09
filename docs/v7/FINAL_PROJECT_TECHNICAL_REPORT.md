# V7 Final Project Technical Report

## Final status

V7 is closed with **`KANG2026_ADAPTATION_NOT_STRONGER_THAN_TRADITIONAL`**. The Kang2026 adaptation did not qualify on the frozen Development split; therefore, the 112-sample V7 Holdout remained locked and unexecuted. The larger project is not yet scientifically complete under the recalibrated requirement `Traditional < Recent Paper Advanced`.

## Source and adaptation

The complete nine-page publisher PDF by Kang and Shan (2026), DOI `10.1016/j.conengprac.2026.106837`, was audited before performance. The implementation preserved the reduced-order fully actuated formulation, virtual-constraint swing treatment, cascade outer-loop structure, and causal disturbance observer. It mapped the original point-payload cable direction to the suspension-point-to-cutter equivalent direction and extended it causally to 3D. The original thrust/body-torque inner loop was omitted so every method retained the common 20 Hz world-acceleration interface, ±2.0 m/s² axis limit, 0.25 m/s²/update slew limit, and common geometric inner loop. This is an adaptation, not an exact reproduction.

## Preregistered Development

The V7 contract froze 144 new Development samples, 112 disjoint locked Holdout samples, 96 unique Paper configurations, a fixed Full-LQR primary comparator, seven qualification gates, and a 10,000-resample paired bootstrap before Paper performance. Frozen PID, Full-LQR, Task-LQR, and SATC were rerun without retuning. Total authoritative Development work was 11,520 runs.

The best candidate, `kang_b_001`, achieved 100% safety, 0.69% success, 0.37311 m mean position RMSE, 3.915 s median acquisition, 2.1186° orientation RMSE, 0.42774 m strong-wind mean, 0.55126 m strong-wind P90, effort 6.22186, and 0.087 ms solve-time P95. Full-LQR achieved 0.14703 m position RMSE. Thus the adaptation's position metric was 153.76% worse, not at least 5% better. The exact-pair bootstrap produced mean delta -0.22608 m, median -0.22646 m, positive-pair fraction 0, and 95% CI [-0.24431, -0.20810] m. It also contained 43 catastrophic pairs. Only safety and the additional acquisition criterion passed: 2/7 gates.

## Interpretation and boundary

The method remained safe and was 7.61% faster in acquisition where acquisition existed, but the cutter-tip regulation deficit was systematic. A plausible adaptation limitation is that one equivalent payload direction cannot represent all internal modes of five massive links; additionally, the fairness interface excludes the paper's original torque inner loop. These observations do not refute the original paper controller on its published plant or hardware.

No Paper was frozen, no Paper-versus-SATC formal trade-off was opened, no final resume package was generated, and no V8 route was started. The frozen V5 claim `V5_SELF_OVERALL_HOLDOUT_WIN` remains the project's highest validated result.
