# V5 Final Technical Report

## Final result

V5 is permanently closed as **V5_SELF_OVERALL_HOLDOUT_WIN**. The frozen SATC-OFMPC candidate `satc_b_027` passed all five Overall Holdout gates and 19/20 total frozen gates. It is not a Strict Holdout winner because normal-regime acquisition was 11.27% slower than legacy `self_a_034`, exceeding the preregistered 10% allowance.

## Frozen research design

V5 branched from V4 tag `v4-research-final-2026-08-09` at `8e3d7ad08747b00d46b9bdfc3cc72451491567db`. Before performance, it froze a 120-sample Development bank, a disjoint 96-sample Holdout bank, relative aligned/opposed/cross direction strata, 64-Self-configuration maximum, 10,000-resample paired bootstrap (seed 20260816), physical acceleration ±2.0 m/s², slew 0.25 m/s²/update, and the unchanged 20 Hz interface.

## Self Development

SATC-OFMPC combines CART's constraint-feasible offset model with causal shock detection, rate-limited offset engagement, final physical-input coordination, slew reserve, and a geometric disturbance-task conflict index. Three of 36 full-bank Stage-B configurations passed 20/20 gates and all three repeated that result in independent Stage C. `satc_b_027` was frozen with Development position RMSE 0.097127 m, success 93.33%, strong P90 0.144855 m, and zero catastrophic pairs.

Post-freeze ablation was explanatory only. Shock-only slightly outperformed full SATC on strong mean/P90, so the added conflict/headroom layers are not claimed to provide monotonic gains. No ablation result was used for retuning.

## Paper-Advanced

The sole selected source was Jiroušek, Báča, and Saska, *Towards Fully Onboard State Estimation and Trajectory Tracking for UAVs with Suspended Payloads* (ICINCO 2025, DOI 10.5220/0013789200003982; primary full text: https://arxiv.org/html/2508.11547v2). The adaptation preserved the paper's augmented incremental-MPC equations and frozen input/increment constraints. All 24 full-bank configurations were ineligible; best `paper_b_002` passed 2/5 Overall gates. Paper was not run on Holdout. This does not compare SATC with the original authors' system.

## One-shot Holdout

The five eligible frozen controllers ran the same 96 samples for 480 authoritative runs, with no retry or compromise. SATC achieved 100% safety, 55.21% success, 0.096827 m mean position RMSE, 2.415 s median acquisition, and 0.149591 m strong P90. Relative to `full_lqr_048`, position improved 29.10%; the overall paired-bootstrap 95% CI was [0.033754, 0.045756] m. Strong position improved 34.41% versus Full-LQR and 80.11% versus legacy Self. Aligned, opposed, and cross directional P90 gates all passed, with zero catastrophic pairs.

## Scope and closure

The evidence supports an Overall Holdout win and unseen strong-transient tail robustness in this frozen MuJoCo benchmark. It does not support a Strict win, Paper-on-Holdout comparison, original-paper superiority, hardware flight, or real-world generalization. V1–V4 evidence trees remain byte-identical to the V4 source tag. V5 must not be tuned or rerun; any new work requires a new research version and new unseen Holdout.
