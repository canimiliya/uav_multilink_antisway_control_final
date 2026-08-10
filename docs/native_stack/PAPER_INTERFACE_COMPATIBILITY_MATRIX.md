# Future paper interface compatibility matrix

| Paper-native output | Native-stack mapping | Required controller-owned layer |
|---|---|---|
| Acceleration command | `AccelerationOuterStackAdapter` then a preregistered inner loop | Attitude/torque loop |
| Thrust + attitude | Attitude target converted to canonical torque | Attitude loop |
| Thrust + body rate | Body-rate target converted to canonical torque | Body-rate loop |
| Thrust + torque | Direct `WrenchCommand` | None |
| Hierarchical MPC + inner loop | High-level and inner components scheduled independently, ending at `WrenchCommand` | Paper hierarchy |
| Direct wrench | Direct `WrenchCommand` | None |

Compatibility does not qualify a paper. A future selection must separately audit actuator assumptions, sensor needs, reference preview, rate, plant model, disturbance truth, and reproducible source availability. Any need for rotor-specific authority, hidden truth, future state, or plant modification is incompatible with Native-Stack Benchmark v1.
