# V3 Final Technical Report

## Outcome

The unique V3 Holdout completed without compromise. The final Self claim is `V3_SELF_DEVELOPMENT_WIN_NOT_CONFIRMED_ON_HOLDOUT`. No controller, parameter, metric, safety rule, plant, or win contract was changed after protocol freeze.

## Holdout protocol

- 57 unseen samples: 12 calm spherical targets, 12 at 2.0 m/s, 12 at 3.5 m/s, 20 stochastic samples with seeds 3000--3019, and one 0--3.5 m/s ramp equilibrium hold.
- Four frozen participants used the same manifest: corrected PID, Full-State LQR, Task-Weighted LQR, and `self_a_034` / 3D-DR-TSRMPC.
- Primary Traditional remained `full_lqr_048`.
- Paper status remained `NOT_RUN_DEVELOPMENT_INELIGIBLE`.

## Controller summary

| Controller | Safety | Success | Position RMSE (m) | Acquisition median (s) | Orientation RMSE (deg) | Ramp peak (m) | Ramp steady (m) | Effort | Solve p95 (ms) |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Corrected PID | 1.000000 | 0.385965 | 0.274936746 | 3.177500 | 0.695855952 | 0.581439668 | 0.876586664 | 5.901354733 | 0.194900 |
| Full-LQR | 1.000000 | 0.596491 | 0.106566812 | 3.755000 | 0.795738623 | 0.427940252 | 0.206872256 | 0.809616166 | 0.116300 |
| Task-LQR | 1.000000 | 0.543860 | 0.124226134 | 2.100000 | 0.600998288 | 0.503370853 | 0.302647851 | 1.418547865 | 0.119200 |
| Self | 1.000000 | 0.789474 | 0.200428612 | 2.245000 | 2.427884208 | 1.373841379 | 0.785216428 | 6.099010953 | 4.024000 |


## Self versus frozen Primary

- Position improvement: -88.078%
- Acquisition improvement: 40.213%
- Ramp peak improvement: -221.036%
- Ramp steady improvement: -279.566%
- Exact-pair bootstrap: mean delta -0.093861800 m, median 0.005101541 m, positive fraction 0.719298, 95% CI [-0.149708225, -0.041287377] m.

## Failure diagnostic

The negative result is concentrated in the new constant-3.5-m/s cohort. Self had lower mean position RMSE than Full-LQR for calm (`0.064782` vs `0.071726` m), constant 2.0 m/s (`0.066786` vs `0.095067` m), stochastic (`0.064036` vs `0.070041` m), and the ramp equilibrium hold (`0.028342` vs `0.124266` m). In contrast, all 12 constant-3.5-m/s pairs favored Full-LQR: Self averaged `0.711379` m versus `0.212308` m, and neither controller acquired those targets.

This cohort explains why Self won 41/57 position pairs and had a positive median paired delta, yet still failed the mean position gate and formal bootstrap. The cohort analysis is explanatory only and does not replace the frozen aggregate metrics or gates.

## Frozen gates

- Overall Holdout win: `False`
- Strict all-metric Holdout win: `False`
- Final status: `V3_SELF_DEVELOPMENT_WIN_NOT_CONFIRMED_ON_HOLDOUT`

The formal acquisition gate uses the frozen aggregate definition. Common-success paired acquisition is retained only as a diagnostic. Ramp summary values preserve the inherited R1 aggregation semantics: the maximum per-sample peak/steady metric across the exact bank.

## Paper route and final boundary

The V3 adaptation of Lv et al. did not satisfy Development qualification and therefore was not evaluated on Holdout. This report does not claim that Self outperformed the original paper or a Paper controller on Holdout. V3 is permanently closed after this evidence freeze; no Self retuning or second V3 Holdout is authorized.
