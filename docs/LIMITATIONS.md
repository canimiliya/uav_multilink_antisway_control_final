# Limitations

- The benchmark is simulation-only and does not establish PX4, ROS 2, hardware, vision, cutting-contact, or real-flight readiness.
- The suspended device is represented by a planar five-link rigid-chain abstraction.
- Wind is applied through deterministic distributed aerodynamic force proxies.
- Some aircraft inertia, link mass, and cutter geometry values are engineering estimates and are labeled in the model configuration and provenance files.
- Task-LQR is a frozen baseline. No robust task-space controller passed the complete calm-plus-crosswind competence contract.
