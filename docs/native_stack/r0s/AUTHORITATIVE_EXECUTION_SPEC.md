# Authoritative execution specification

Formal v1.1 execution is only `AuthoritativeNativeCaseRunner.run_case(controller, resolved_case)`. The runner verifies the semantic fingerprint and internally constructs the causal reference, wind timeline, and distributed body-force callback. It rejects Holdout and any mutated case before physics execution.

The older generic runner remains available for diagnostics but is labeled `DIAGNOSTIC_NON_AUTHORITATIVE`; callers cannot turn custom reference or disturbance injection into benchmark evidence. This semantic patch performs no controller performance run.
