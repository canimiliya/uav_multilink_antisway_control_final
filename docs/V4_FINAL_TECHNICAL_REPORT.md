# V4 Final Technical Report

## Final status

V4 is permanently closed as **mechanism improvement without a qualified Self method**. CART-OFMPC corrected the dominant V3 persistent-strong-wind infeasibility chain, but `cart_b_001` passed only 12/15 frozen Development gates. No candidate was frozen, no V4 Holdout sample was run, and no V5 work was started.

## What CART fixed

The V3 strong-wind diagnostic recorded residual clipping at 69.74%, slew activity at 62.55%, and requested steady input averaging 5.19 m/s² with an 11.34 m/s² peak. For `cart_b_001`, the corresponding frozen strong-cohort values were about 3.53%, 9.71%, and 0.0078 m/s² mean. Overall position improved 19.88% versus Full-LQR; strong mean position improved 16.82% versus Full-LQR and 74.31% versus legacy Self. The paired bootstrap and normal non-regression gates passed.

This supports a narrow positive result: CART materially interrupted the old persistent-equilibrium amplification mechanism. It does not establish a V4 controller win.

## Why qualification still failed

Strong success was 10.81%. Strong position P90 was 0.415972 m, 94.04% worse than Full-LQR, and seven simultaneous-onset pairs were catastrophic: `dev_strong_simultaneous_00, dev_strong_simultaneous_01, dev_strong_simultaneous_02, dev_strong_simultaneous_05, dev_strong_simultaneous_06, dev_strong_simultaneous_09, dev_strong_simultaneous_11`. The favorable mean therefore cannot override the frozen tail gates.

## Simultaneous-onset mechanism audit

Within the 12 frozen simultaneous directions, all seven negative-x targets were catastrophic and all five positive-x targets were non-catastrophic. Catastrophic mean position RMSE was 0.419872 m versus 0.085726 m; orientation RMSE was 5.178° versus 0.858°. Trust fell to 0.558 versus 0.941; residual clipping rose to 18.56% versus 0.01%; slew activity rose to 39.66% versus 2.90%. QP-correction/backbone norm ratio averaged 1.688 versus 0.071.

**Confirmed:** the frozen cohort has an exact x-sign partition and a low-trust/high-correction/high-slew catastrophic regime; all 18 full-bank Stage-B candidates failed both tail gates.

**Strongly supported:** simultaneous onset creates a new direction-dependent transient-coordination problem under slew-limited recovery, distinct from the repaired V3 steady infeasibility chain.

**Hypothesized:** fixed +x wind combined with negative-x target demand aligns the innovation against the target transient. This needs a new preregistered experiment. R1 did not preserve time-series traces, so tail-onset time and the backbone/QP vector angle are not observable and are not claimed as confirmed.

## Pareto and V5 decision evidence

Stage A's eight candidates used a 16-sample smoke bank and cannot share a formal frontier with Stage B. Across the 18 comparable full-bank candidates, none passed P90 or catastrophic-pair gates. `cart_b_014` increased strong success to 89.19% and reduced the catastrophic count to 4, but failed normal position/acquisition and still failed both tail gates. This is an architecture-level transient-robustness limitation expressed through a parameter trade-off, not evidence for a 27th CART configuration.

The evidence supports `RECOMMEND_NEW_RESEARCH_VERSION`, centered on abrupt simultaneous onset and frozen before any performance run. This recommendation does not authorize or start V5.

## Claims and closure

V3 Self remains a Development win not confirmed on V3 Holdout. V4 CART shows mean improvement and mechanism repair but fails Development qualification. V4 Holdout remains `UNUSED`; it must not retrospectively rescue CART. Paper was not part of V4. Final result: `V4_CLOSED_WITH_MECHANISM_IMPROVEMENT_BUT_NO_QUALIFIED_SELF`.
