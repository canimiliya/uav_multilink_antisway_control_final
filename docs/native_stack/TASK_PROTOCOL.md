# Native task protocol

Two task families are mandatory.

- Cutter setpoint/acquisition retains deterministic 3D target steps, calm operation, and aligned, opposed, and cross-wind onset.
- Smooth cutter trajectory tracking includes minimum-jerk/quintic transitions, smooth approach-stop, and smooth 3D waypoint paths.

Every smooth reference deterministically returns position, velocity, acceleration, and jerk. Endpoint velocity and acceleration are zero. Controllers receive only the current sample; no method receives private future preview.

The 200-case Development and 140-case Holdout banks each balance both task families across calm, moderate, strong sustained, strong transient, stochastic, and ramp wind, with aligned/opposed/cross directions. Holdout uses disjoint target identities, target/wind/trajectory seeds, and timing identities.
