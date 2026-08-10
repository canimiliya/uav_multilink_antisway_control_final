# Competence Governance v2

## Scope

Governance v2 retains Native Benchmark v1.1 physics and data byte-for-byte. It changes only how a Traditional method becomes eligible for a future comparison. It does not qualify any existing method and cannot relabel P2-R1R1 or P2-R1R2.

## Traditional eligibility

A future frozen method must satisfy all of the following on the unchanged 200-case Development bank:

1. All-case SafetyV2 rate at least 98%, no more than four catastrophic cases, and deadline miss rate at most 1%.
2. Nominal-cohort success at least 70%.
3. Success at least 50% within each nominal wind kind: calm, moderate, and stochastic.
4. Nominal success at least 60% in each task family: setpoint and smooth trajectory.
5. Nominal setpoint position RMSE at most 1.25 m and nominal smooth-trajectory tracking RMSE at most 1.50 m.

The 70% nominal threshold remains a methodological gate, not a physical constant. It demands a clear operational majority without imposing the near-operational reliability implied by 80%. The per-wind and per-task floors prevent aggregation from hiding a method that only works in calm conditions or only for one task family.

## Challenge robustness

Strong sustained, strong transient, and ramp cases remain unchanged and mandatory. Their success, RMSE, endpoint error, saturation, and strong-error statistics are reported as Advanced-method differentiation. They do not decide Traditional eligibility except through the all-case safety/catastrophic requirements.

## Integrity rules

The thresholds were frozen before computing `WOULD_EXISTING_METHOD_PASS`. No threshold may be adjusted after that diagnostic. Any future qualification must use a new explicitly authorized task, protocol freeze, and run; old competence-v1 failures remain failures.
