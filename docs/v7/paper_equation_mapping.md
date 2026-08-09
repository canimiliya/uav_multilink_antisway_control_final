# Kang 2026 Equation Mapping

This implementation is a faithful adaptation, not an exact reproduction. The primary source is Kang and Shan, *Control Engineering Practice* 170 (2026) 106837, DOI `10.1016/j.conengprac.2026.106837`.

## Original to V7 mapping

| Paper element | Original formulation | V7 adaptation |
|---|---|---|
| Suspended state | Fixed-length point payload represented by two cable swing angles | Causal 3D equivalent swing displacement: cutter-tip error minus UAV position error; its derivative is formed from measured velocities |
| Virtual constraint | Equations (10), (18), and (19) couple translation and cable swing | The same composite-coordinate structure couples UAV translation to the equivalent five-link swing state |
| Reduced-order FAS | Equations (11)-(17) stabilize an auxiliary translational coordinate | A normalized acceleration-channel FAS coordinate is used under the common `[ax, ay, az]` authority |
| Disturbance observer | Finite-time observer in equation (16) | Causal discrete super-twisting realization with fixed internal substeps, bounded estimate, and no wind truth |
| Outer command | Equation (20) produces a desired force | Mass-normalized world acceleration, limited to `2.0 m/s²` per axis and `0.25 m/s²` slew per 20 Hz update |
| Inner control | Paper FAS attitude controller and torque DOB, equations (29)-(44) | Omitted because every V7 method must use the frozen common geometric inner loop |

## Preserved scientific core

The adaptation preserves the reduced-order FAS coordinate, virtual-constraint coupling, cascade interpretation, swing suppression, and disturbance-observer compensation. It does not use true wind, future references, SATC state, motor commands, direct thrust, or torque authority. The physical five-link MuJoCo plant and cutter-tip metrics remain unchanged.

## Assumptions and validation

The UAV-to-cutter displacement is treated as the equivalent suspended direction of the massive five-link chain. At the frozen equilibrium, its swing displacement is zero. The 3D extension applies the paper's outer-loop structure componentwise with preregistered axis scaling; all controller outputs pass through the same benchmark limiter. Unit tests verify causality, reset determinism, finite commands, and common authority limits before any Development performance is run.

