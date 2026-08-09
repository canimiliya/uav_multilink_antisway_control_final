# V7 Negative Results

## Outcome

`KANG2026-FAS-DOB-ADAPTED-5LINK` produced zero Development-qualified candidates within the frozen 96-configuration budget. The selected near-miss `kang_b_001` passed 2/7 gates.

## Gate audit

| Gate | Result |
|---|---|
| Safety no worse than best Traditional | PASS |
| Success no worse than best Traditional | FAIL |
| Position improvement vs Full-LQR ≥5% | FAIL |
| Paired-bootstrap lower bound >0 | FAIL |
| Strong-wind P90 non-regression | FAIL |
| Zero catastrophic pairs | FAIL |
| At least one meaningful extra metric | PASS (acquisition) |

The Paper adaptation's 0.37311 m Development position RMSE exceeded Full-LQR's 0.14703 m. Its bootstrap interval was entirely negative, and all 144 paired position deltas favored Full-LQR. The negative result is therefore not explained by sampling uncertainty.

## Stopping decision

The Paper was not frozen, the V7 Holdout remained unexecuted, no parameter refinement or replacement paper was attempted after seeing results, and SATC/Traditional controllers were not changed.
