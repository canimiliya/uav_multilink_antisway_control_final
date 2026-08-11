# P3-R1B heading extension gate

The requested `XYZ + cutter heading` extension is blocked at P0.  The
vendored source `third_party/udaan/udaan/control/quadrotor/geometric_attitude.py`
defines `GeometricAttitudeController.compute(self, t, state, thrust_force,
desired_att=None)`.  Its first runtime argument is `t` (time), and the
implementation derives the desired attitude from `thrust_force`; it does not
consume a yaw/heading command.

The task card requires an immediate stop when this interface is not a yaw
channel.  Therefore no wrapper, yaw smoke, heading preset, Task-1 upgrade,
Task-4 run, animation, or `TASK_CAPABILITY_CONTRACT_V2.md` was created.  The
frozen `GeometricInnerLoop`, controller parameters, and model remain
untouched.  The machine-readable evidence is
`artifacts/meeting_demo/yaw_interface_audit.json`.

This is a capability/interface block, not evidence that cutter heading is
impossible in the model.  A future implementation requires an explicitly
authorized Udaan interface change or a separate attitude-command API.
