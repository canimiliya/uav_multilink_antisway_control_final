# Native physical control interface

`WrenchCommand` contains scalar body-z thrust and three body-frame torques. The canonical actuator validates the exact frozen ctrlranges, clips requests to those ranges, writes only the four audited direct-wrench actuators, and records requested, clipped, and actual commands.

The plant exposes four rotor motors as well, but all formal historical runners used `thrust_motor`, `mx_motor`, `my_motor`, and `mz_motor`. Native v1 therefore standardizes that lowest historically exercised direct-wrench path. A paper-specific actuator is forbidden.

`NativeStackController` supports observation, optional high-level updates, optional inner updates, physical command output, and diagnostics. This accommodates single-rate, hierarchical, observer-based, MPC, direct-wrench, and wrapped legacy acceleration controllers. `AccelerationOuterStackAdapter` passes an unchanged legacy acceleration through the existing `GeometricInnerLoop`; it does not retune the legacy method.

Command logs include component update timestamps, requested/clipped/actual wrench, saturation, command rate, outer time, inner time, total time, and deadline status.
