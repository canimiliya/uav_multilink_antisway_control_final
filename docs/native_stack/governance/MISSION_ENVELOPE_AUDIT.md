# Mission envelope audit

## Conclusion

Native Benchmark v1.1 is demanding but physically reasonable. The audit found no basis to regenerate Development or Holdout and no basis to change the plant, actuator limits, wind signals, task identities, or 12 s duration.

The frozen system has 13.24 kg total mass, requires 129.884 N to hover, and has 285.746 N maximum thrust. A conservative 3 m/s static-drag estimate is 3.785 N, only 4.16% of the horizontal force available at 35-degree tilted hover. Smooth references peak at 0.427 m/s and 0.417 m/s²; targets span 0.559-1.773 m with vertical displacement from -0.100 to 0.549 m.

The endpoint equilibrium certificate independently covered all 200 Development cases. Every final target admitted a safe nonlinear MuJoCo equilibrium under its final frozen wind and canonical wrench limits. The largest required wrench was 129.967 N thrust and approximately 0.546/0.033/0.004 N m torque. A conservative transfer certificate required at most 9.008 s versus at least 11.25 s available.

This is an endpoint and authority result, not evidence that a causal controller can track every reference or reject every transient.

## Classification

- Static force and hover authority: `PHYSICALLY_REASONABLE`.
- Reference motion and linked-load burden: `AGGRESSIVE_BUT_FEASIBLE`.
- Mission structural problem: `false`.
