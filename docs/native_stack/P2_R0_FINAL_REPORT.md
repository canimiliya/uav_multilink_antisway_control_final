# P2-R0 Native-Stack Benchmark v1 final report

## Outcome

`P2_NATIVE_STACK_BENCHMARK_READY`. All mandatory gates passed. This task upgraded and froze the comparison platform only; it selected no paper, introduced no new controller performance claim, retuned neither SATC nor Traditional, and executed no old or native Holdout.

## Audited actuation and timing

The unchanged plant exposes the canonical direct physical interface `WrenchCommand[T,tau_x,tau_y,tau_z]` through `thrust_motor`, `mx_motor`, `my_motor`, and `mz_motor`. Limits are thrust `[0, 285.74568] N` and body torques `±25/±25/±12 N m`. Physics is 1000 Hz; legacy inner and outer rates are 200 and 20 Hz. There are no motor dynamics, actuator lag, or actuator rate limits in the frozen model.

The integer-tick scheduler supports `[20, 50, 100, 200, 500, 1000]`. Its 1000-second logical audit had exact update counts and zero accumulated drift.

## Compatibility and isolation

Three frozen V3 Development cases reproduced `full_lqr_048` with command and metric maximum error 0. Across 64 deterministic inputs, desired force, thrust, torque, clipping, and applied wrench matched exactly. Runtime controllers cannot own the offline truth packet, future preview is disabled, and true wind/hidden force/Holdout metadata are absent from the sensor API.

## New banks

Native Development contains 200 cases (SHA-256 `0f03df8fe11310f6357197a9d03b605476831c76db58f64d04e92535e6df9473`). Native Holdout contains 140 cases (SHA-256 `63e6192faf992494f5a78f4c008d844564b0015fc26b43a94a0c98da659b2538`), remains `execution_allowed=false`, and has zero executions. All target, wind, trajectory, and timing identities are disjoint.

## Test evidence

All 11 isolated test-directory processes passed: 248 passed, 0 failed. The execution audit preserves three monolithic timeout attempts and one same-process module-name collection conflict; no scientific protocol changed. V1-V10 protected trees and tags remain unchanged.

## Research boundary

Benchmark A and its V5 SATC Holdout win remain valid and read-only. Benchmark B is a separate system-level protocol. P2-R1 is not started by this result; it requires explicit authorization and must develop native baselines on Development without opening Holdout.
