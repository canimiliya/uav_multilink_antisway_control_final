# Xu 2025 Equation Mapping

Primary source: J. Xu, D. Lin, J. Ye, and T. Jiang, *Actuators* 14(7):335 (2025), DOI `10.3390/act14070335`.

This is a faithful advanced-controller adaptation, not a claim of reproducing the authors' plant.

| Source equations | V8 realization | Classification |
|---|---|---|
| (1)-(2), point-load cable direction and angular velocity | The frozen five-link joint state is projected onto plant modes 1 and 2, reconstructed, and mapped to a causal suspension-to-cutter-center unit direction and angular velocity. | ADAPTED |
| (8)-(13), point-load dynamics | The MuJoCo five-link plant remains unchanged. Source states are used only inside the outer controller. | ADAPTED |
| (14)-(19), multivariable finite-time observer template | Vector signed-square-root injection, smoothed sign injection, bounded disturbance estimates, and causal discrete integration at 20 Hz. | PRESERVED/ADAPTED |
| (20)-(22), load-position loop | Actual cutter-tip error forms the paper virtual load-velocity command. | PRESERVED/ADAPTED |
| (23)-(31), load-velocity loop and force observer | Actual tip velocity, nominal mass-normalized force, gravity compensation, and finite-time force-disturbance compensation. | PRESERVED/ADAPTED |
| (27)-(30), desired direction and command filter | Desired load direction is computed from virtual force, normalized safely, and filtered on the sphere. | PRESERVED |
| (32)-(35), direction loop | Cross-product direction error and filtered virtual angular-rate command. | PRESERVED |
| (36)-(40), angular-rate loop and observer | Modal load angular rate, finite-time angular disturbance observer, and paper-native angular feedback. | PRESERVED/ADAPTED |
| (41), parallel/perpendicular force synthesis | Source force vector is mass-normalized and converted to the common world-frame acceleration command. | PRESERVED/ADAPTED |
| Source thrust magnitude and attitude command | Omitted; all methods use the same frozen geometric inner loop. | OMITTED_FOR_FAIR_AUTHORITY |
| True wind, future target, SATC state/modules | Not available to the controller. | UNAVAILABLE/PROHIBITED |

The implementation retains all four cascaded loops, both command filters, both disturbance-observer channels, and parallel/perpendicular force synthesis. It is therefore not a `PD + sign()` surrogate.
