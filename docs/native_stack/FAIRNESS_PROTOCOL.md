# Native benchmark fairness protocol

All compared stacks must use the same physical plant and its unchanged mass, inertia, links, cutter, hinges, damping, gravity, collision, aerodynamics, and wind; the same actuator limits, sensor envelope, task case, safety definition, and wind realization; and the same reference-preview rule.

Internal architecture, observer, model, solver, inner loop, and controller rate may differ. Rate differences are legal only in explicitly labeled Native-Rate results. Equal-Rate results must share a rate.

Compute budgets are reported rather than artificially equalized. Every component records mean, P95, P99, maximum execution time, and deadline misses. A component is real-time only if execution time does not exceed its update period.

No future method may use the native Holdout for debugging, parameter selection, architecture choice, or retries. Selection uses Development only, then the exact frozen implementation and protocol may receive one authorized Holdout execution.
