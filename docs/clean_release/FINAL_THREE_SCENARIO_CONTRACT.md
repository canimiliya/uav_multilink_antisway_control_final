# Final three scenario contract

Frozen by P3-R1F fixed-grid capability-boundary characterization.

## T1
- Initial sway: `[20.0, -16.0, 12.0, -8.0, 4.0]` deg
- Target delta: `[2.0, 1.7, 4.5]` m
- Move duration: `5 s` (COMMON_FASTEST_STABLE_MOVE_S)
- Wind: 0 m/s

## T2
- Initial sway: `[20.0, -16.0, 12.0, -8.0, 4.0]` deg
- Target delta: `[2.0, 1.7, 4.5]` m
- Move duration: `5 s` (inherited from T1 common boundary)
- Wind: `3 m/s`, world +X
- Onset: 3 s, half-cosine ramp 3--4 s

## T3 (archived, not rerun)
- Hover, zero initial sway, world +X
- Historical tested envelope: 3--10 m/s
- LQR max recoverable: 3 m/s
- SATC max recoverable: 5 m/s

All primary meeting comparisons use COMMON_STABLE cases. Results are functional capability-boundary evidence only; no tuning, model change, holdout, paper, or native-research claim is made.
