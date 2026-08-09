# Final Project Technical Report

## Final scientific status

The project closes with **V5_SELF_OVERALL_HOLDOUT_WIN** as its highest positive result and **V6_NO_QUALIFIED_RECENT_PAPER_BASELINE** as the final V6 outcome. V6 did not alter or retune SATC-OFMPC, the Traditional controllers, or any V1-V5 evidence.

## V5 retained result

Frozen `satc_b_027` remains the formal unseen-Holdout Self result: 96 samples, 100% safety, 55.21% success, 0.09683 m position RMSE, and a positive 10,000-resample paired-bootstrap interval versus `full_lqr_048`. It is an Overall, not Strict, Holdout win.

## V6 preregistered design

V6 froze a new 120-sample Development bank and a disjoint 96-sample locked Holdout bank before Paper performance. The suite was fixed to Yu et al. (2026) and SEP-NMPC (2026). Kang and Shan (2026) was excluded before performance because only abstract-level primary content was retrievable. The Paper search budget was 32 unique configurations per method: eight 24-sample smoke configurations and 24 full 120-sample configurations.

## V6 Development result

The best Yu adaptation `yu_b_007` passed 0/8 gates: safety 40.83%, success 0.00%, position RMSE 5.6509 m, and 54 strong catastrophic pairs. The best SEP adaptation `sep_b_002` passed 3/8 gates: safety 100.00%, success 1.67%, position RMSE 0.4564 m, and 54 strong catastrophic pairs. Its overall paired-bootstrap interval versus Full-LQR was [-0.3424, -0.2925] m, entirely favoring Full-LQR.

Because no Paper qualified, V6 Holdout was not executed. This is the preregistered stopping outcome, not an infrastructure block.

## Scope

Evidence supports SATC's V5 simulated Holdout result and the V6 adaptation-level negative results. It does not establish superiority over the original published systems, a Paper-Advanced win, real-flight behavior, hardware feasibility, or a Strict V5 win.
