# V4 CART-OFMPC Development Report

## Result

V4-R1 is closed with **no qualified CART-OFMPC Development candidate**. The four frozen comparators were evaluated once on all 94 Development samples (376 authoritative runs). The preregistered CART search then used 8 Stage-A structural configurations, 18 Stage-B full-bank configurations, and six unchanged Stage-B configurations for Stage-C confirmation. No V4 Holdout sample was executed.

## Frozen protocol and implementation

The performance-before-data checkpoint is `ff5c6ac1a02524705ff53ac106196798a6b1ca72`. CART-OFMPC uses the immutable `task_lqr_009` gain, a causal one-step residual estimator, a bounded steady-target active-set QP, residual trust, bounded anti-windup debt, and a finite-horizon task-space QP that directly constrains physical acceleration and slew. It never reads true or future wind. The steady solver records requested and feasible inputs separately and never silently scales an infeasible equilibrium.

## Comparator evidence

The frozen Full-LQR achieved overall mean position RMSE `0.131385 m`. The legacy `self_a_034` preserved 100% normal-regime success but retained its previously identified sustained-strong-wind failure. All 376 comparator runs were safe.

## Best near-miss

`cart_b_001` passed 12 of 15 atomic gates. Overall position RMSE was `0.105271 m`, a `19.88%` improvement over Full-LQR. The paired 10,000-resample bootstrap CI was `[0.009070, 0.041823] m`, so its lower bound was positive. Normal-regime success stayed at `100.0%`; position changed by `1.38%` and acquisition by `0.00%` relative to legacy Self, both within contract.

The candidate nevertheless failed three strong-wind gates: success was `10.81%`, below the best Traditional; P90 position changed by `94.04%` versus Full-LQR, above the +10% limit; and `7` exact pairs exceeded twice the Full-LQR error. Thus the favorable strong-wind mean (`0.160055 m`, `16.82%` better than Full-LQR) cannot qualify the method.

## Mechanism conclusion

The original V3 amplification chain was materially interrupted: in the best near-miss strong cohort, residual clipping averaged `3.53%`, amplitude saturation `0.99%`, and slew activity `9.71%`, versus the R0 legacy findings of roughly 69.74%, 27.77%, and 62.55%. Requested steady input remained bounded in practice rather than reaching the legacy 5-11 m/s2 range. However, abrupt simultaneous wind onset produced a different tail: trust dropped while backbone/QP cancellation and slew activity remained concentrated in several directions. The mechanism repair improved the mean but did not satisfy tail robustness or strong-cohort success.

## Closure

No candidate is frozen and no ablation was run, because the frozen contract permits ablation only after a Development-qualified freeze. There was no 27th/33rd configuration, no comparator retuning, no Paper work, no contract change, and no Holdout access. Final status: `CLOSED_WITH_NO_V4_CART_OFMPC_DEVELOPMENT_WIN`.
