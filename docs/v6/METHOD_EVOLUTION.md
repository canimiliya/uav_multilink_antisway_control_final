# Method Evolution

## LS-PMPC

Established the predictive-control and reproducibility foundation, but did not yield a final robust task-space winner.

## OF/DR-TSRMPC

Added offset-free and disturbance-rejection mechanisms. Formal gates exposed persistent acquisition and robustness limitations; the route was closed without hiding failures.

## CART-OFMPC

Introduced constraint-feasible steady targets, explicit unrepresented residuals, trust, and anti-windup debt. It clarified the saturation/estimation mechanism but did not achieve a V4 Development win.

## SATC-OFMPC

Added causal shock detection, bumpless offset engagement, physical-input coordination, slew headroom, and a geometric disturbance-task conflict index. The frozen `satc_b_027` passed Development and achieved the project's V5 Overall Holdout win.

## Recent-Paper suite

V6 froze Yu 2026 and SEP-NMPC 2026 adaptations before performance. Neither qualified on new Development, so the project closed without a Paper Holdout or replacement search.
