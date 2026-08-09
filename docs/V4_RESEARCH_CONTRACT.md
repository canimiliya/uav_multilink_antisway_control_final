# V4 Strong-Wind Research Contract

## Scope and status

V3 remains permanently frozen at `1f6ef12bc657724024f4e281c6b0534ca1d76fad` with final status
`V3_SELF_DEVELOPMENT_WIN_NOT_CONFIRMED_ON_HOLDOUT`. The V3 Holdout is now V4 prior
evidence, not a V4 Holdout. This R0 task ran no new-controller performance and did
not execute the newly frozen V4 Holdout.

## Reproduced failure

Instrument-only replay reproduced all 12 sustained 3.5 m/s pairs exactly. Mean
position RMSE was `0.711379 m` for `self_a_034`
and `0.212308 m` for `full_lqr_048`. The frozen
ramp replay remained `0.028342 m` for Self.

## Root-cause hierarchy

The primary, strongly supported mechanism is a constraint-unaware residual-to-
equilibrium map entering a saturated feedback regime. In constant 3.5 m/s wind,
raw residual clipping occurred on `69.7%`
of updates, chiefly `qdot1`, `qdot2`, and `qdot3`. The unscaled steady command
required `5.190 m/s^2` on
average (peak `11.342`), so
the internal solution was scaled on `69.1%`
of updates. The resulting loop showed `27.8%`
amplitude activity and `62.6%` slew activity.

The ramp case had no residual clipping, steady scaling, amplitude saturation, or
slew activity. Failure without any reference step (`0.689 m`)
shows that target-step interaction is not necessary. Projection rejection was
small and every QP solved, so neither is the primary cause.

## Frozen V4 hypothesis

The sole V4 Self hypothesis is **CART-OFMPC**: causal residual estimation plus a
constraint-feasible steady target, residual-confidence scheduling, and explicit
anti-windup accounting for unrepresented disturbance. It may use measured state,
previous actual command, innovation, estimated disturbance, and constraint
activity, but never true or future wind.

## Frozen evaluation

Development contains `94` samples, including
`37` sustained strong-wind cases and all
three onset relationships. Holdout contains `94` wholly new
samples generated with seed `20260810`, stochastic seeds `4000..4019`, and
`48` sustained strong-wind cases. Holdout
execution remains forbidden until every participant and selection outcome is
frozen.

Formal success requires safety/success non-regression, at least 5% overall and
strong-wind position improvement versus Full-LQR, at least 30% strong-wind
improvement versus legacy Self, no catastrophic strong-wind pair, normal-regime
position within 5% of legacy Self, and the frozen paired bootstrap gate.

## Frozen boundaries

The plant, task, safety definition, geometric inner loop, 20 Hz world-frame
`[ax, ay, az]` interface, `+/-2.0 m/s^2` per-axis authority, and `0.25 m/s^2` per-
update slew limit are unchanged. Traditional controllers and `self_a_034` remain
byte-frozen baselines. Paper search is outside this task.
