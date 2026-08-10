# Platform limitations

- The plant uses direct body wrench actuators. It has no motor, propeller, ESC, actuator lag, actuator rate limit, or actuator-dynamics model. Rotor graphics and four rotor actuator definitions do not change the formal direct-wrench execution path.
- Native v1 is simulation-only and does not validate onboard sensing, state estimation, communication latency, hardware timing, aerodynamic model fidelity, or real flight.
- Exact simulator state is intentionally available within the causal sensor envelope.
- Controller CPU timing is host- and load-dependent; deadline metrics are evidence for the recorded environment, not a hardware guarantee.
- The new workspace bounds are protocol gates only for future native studies. They do not reclassify V1-V10 trials.
- A controller needing rotor-level allocation or actuator dynamics requires a separately authorized new plant version; v1 must not be silently extended.
