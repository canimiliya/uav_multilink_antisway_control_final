# Native-Stack Benchmark v1 to v1.1 migration

## Why v1 is retained

`native-stack-benchmark-v1` correctly froze the plant, physical wrench interface,
sensor envelope, scheduler, safety rules, and the ordered identities of 200
Development and 140 unopened Holdout cases. It did not freeze the mapping from
those identities and seeds to a complete physical experiment. The v1 tag and the
blocked P2-R1 history therefore remain immutable research evidence.

## What v1.1 may change

v1.1 may only add a pure, deterministic, versioned case resolver, resolved
manifests, semantic signal fingerprints, and an authoritative runner that owns
reference and wind construction. The original case count, order, IDs, seeds,
categories, issue offsets, durations, split flags, plant, physical actuator
contract, sensor contract, and safety contract cannot change.

## Authority boundary

The generic `NativeStackRunner.run(controller, reference, ..., disturbance)` API
is retained for diagnostics and is explicitly non-authoritative. Formal benchmark
execution must use a resolved case through the authoritative runner. This patch
runs no PID, LQR, SATC, Paper, Development-performance, or Holdout-performance
experiment. The 38 provisional R1 smoke runs have no selection authority and are
not inputs to semantic design.
