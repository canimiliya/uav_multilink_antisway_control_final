# Interview Q&A

## What is the strongest result?

The strongest result is the V5 SATC-OFMPC Overall Holdout win: lower position RMSE and higher success than frozen Full-LQR on 96 unseen samples, 100% safety, and a positive paired-bootstrap confidence interval.

## Why is it not a Strict win?

Normal-regime acquisition degraded 11.27% versus the frozen legacy Self reference, exceeding the preregistered 10% allowance.

## Why did V6 not run Holdout?

Both recent-Paper adaptations failed the preregistered Development gate. The contract required zero-qualified Paper suites to stop before Holdout, preventing test-set fishing.

## Why not implement Kang and Shan from the abstract?

The full primary equations were not legally and reliably available at suite freeze. Guessing formulas would have produced an unverifiable method, so a complete predeclared 2026 fallback was used instead.

## What did the Paper failures teach you?

Cross-model adaptation is not a neutral implementation detail. A controller derived for a point load or obstacle-aware single cable can lose stability or task accuracy when mapped to a five-link cutter and a strict acceleration/slew interface. The negative results quantify that adaptation gap.

## What remains before deployment?

Hardware-in-the-loop timing, actuator and sensor calibration, model mismatch, cable/link flexibility, estimator noise, embedded computation, and real-flight safety validation remain outside this project.
