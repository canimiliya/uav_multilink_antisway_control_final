# Sensor envelope and truth isolation

Every runtime controller receives only `SensorPacket`: current time/tick, UAV position and velocity, rotation, body angular velocity, five joint positions and velocities, cutter-tip position and velocity, the current reference sample, and the previous applied physical command. Exact simulator state is allowed because this benchmark studies control rather than estimation.

Runtime packets exclude true/future wind, future reference, future state, hidden MuJoCo forces, and Holdout metadata. Reference preview is disabled in v1. Any future preview protocol must be frozen separately and grant identical preview information to every eligible method.

`DiagnosticTruthPacket` is a distinct offline-only type for observer and physics audits. The runtime controller base rejects ownership of that type, and the runtime sensor reader cannot construct or return it.
