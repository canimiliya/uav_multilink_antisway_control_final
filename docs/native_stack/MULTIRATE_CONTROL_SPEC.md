# Deterministic multi-rate control

The scheduler uses integer 1000 Hz physics ticks; floating-point time never decides whether a component is due. Supported component rates are exact integer divisors: 20, 50, 100, 200, 500, and 1000 Hz.

At each tick the order is:

1. Read the causal state and current reference.
2. Run every component due at that tick in registration order.
3. Apply the resulting canonical physical command.
4. Advance MuJoCo by one 1 ms physics step.

Each component uses zero-order hold between updates. Phase offsets must be integer ticks within one component period. A 1000-second logical validation produced the exact expected count at every supported rate with zero accumulated tick drift.

Equal-Rate mode gives all compared methods one preregistered common rate. Native-Rate mode gives each method its preregistered supported native rate. Results from the two modes must be labeled and must not be silently pooled.
