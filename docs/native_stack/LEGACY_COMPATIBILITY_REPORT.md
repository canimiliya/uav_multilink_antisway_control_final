# Legacy compatibility report

The V1-V10 runner and controller sources were not modified. Ten protected evidence/document trees match the V10 source tag by Git tree object, and all historical research tags retain their original commit targets.

Three deterministic `full_lqr_048` cases from the frozen V3 Development split were rerun through the old pipeline. Command-path maximum error was 0, metric maximum error was 0, and safety, task success, and acquisition identities were unchanged. No old Holdout was read or executed.

For 64 deterministic state/reference/acceleration inputs, the new `AccelerationOuterStackAdapter` and old `GeometricInnerLoop` produced exactly identical desired force, thrust, torque, clipping, and applied wrench (all maximum errors 0). Therefore the new API does not alter frozen legacy behavior.
