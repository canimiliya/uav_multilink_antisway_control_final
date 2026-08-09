# V8 Final Technical Report

## Outcome

V8 is closed with `V8_NO_QUALIFIED_DOUBLE_PENDULUM_PAPER_BASELINE`. The recent-Paper strong-baseline objective was not achieved, so the inherited 112-sample Holdout was not executed and `PROJECT_RESEARCH_COMPLETE=false`.

## Source and Model Reduction

Priority A (Yan 2026) and fallback B (Yan 2024) had no complete legally open primary equations. The first eligible preregistered source was Xu et al. (2025), *Actuators* 14(7):335, DOI `10.3390/act14070335`, implemented as `XU2025-CBS-FTDO-ADAPTED-5LINK`.

The frozen five-link plant produced five passive modes. Modes 1 and 2 (0.3194 and 1.6054 Hz) were selected using plant-only observability, control-participation, energy, and bandwidth criteria. Their base-pulse tip-response normalized RMSE was `2.93e-5`, so the modal reduction passed its preregistered parity check.

## Faithful Adaptation

The controller retained Xu's four cascaded backstepping loops, two command filters, parallel/perpendicular force synthesis, and finite-time force/angular disturbance observers. The point-load direction was adapted from the two frozen modal coordinates. The output was restricted to the common 20 Hz world-frame acceleration interface with the same amplitude, slew, and geometric-inner-loop authority as every comparator.

## Development Evidence

The 144-sample V7 Development bank and comparator evidence were carried forward byte-identically. Stage A ran 16 candidates on 24 non-selection samples. Stage B ran all 128 unique configurations on all 144 samples: 18,432 authoritative Paper runs. No configuration passed all gates.

The diagnostic best-ranked candidate, `xu_v8_077`, achieved 47.92% safety, 0.69% success, and 2.1982 m mean position RMSE. It was 1,395% worse than Full-LQR by the signed improvement definition, had 72 catastrophic strong pairs, and failed the paired bootstrap (`95% CI [-2.1717, -1.9332] m`). Its 1.16 s acquisition median came from only one successful sample and is not a valid Advanced result.

## Interpretation and Final Boundary

Small-signal modal parity did not translate to closed-loop qualification. The source assumes a point load, direct thrust-vector synthesis, a smooth `C5` target, and 100 Hz execution. The fair adaptation used a five-link task-tip benchmark, step targets, a 20 Hz rate-limited acceleration outer loop, and a shared inner loop. Saturation and unsafe attitude, height, and joint excursions dominated.

No Paper method was frozen; no Paper-vs-SATC trade-off or Holdout claim is allowed. V1-V7, all Traditional controllers, and `satc_b_027` remain unchanged. The highest valid claim remains `V5_SELF_OVERALL_HOLDOUT_WIN`, within a pure-simulation boundary.
