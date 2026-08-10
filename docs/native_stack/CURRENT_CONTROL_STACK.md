# Current frozen control stack

The audited V1-V10 execution path uses the byte-frozen MuJoCo model `reproducibility/frozen/model/model_5link_controlled.xml` (SHA-256 `19105873c0fcc891ebb85efe6c20c378d5b77b6bf9003559e43ae47ca03d153d`). Physics integrates at 1000 Hz with RK4. Wind and the shared geometric inner loop update at 200 Hz, and the legacy acceleration outer loop updates at 20 Hz. Both controller levels use zero-order hold.

The formal runners write `thrust_motor`, `mx_motor`, `my_motor`, and `mz_motor`: direct body-z thrust plus body torque with limits `[0, 285.74568] N`, `±25`, `±25`, and `±12 N m`. Four rotor actuators exist in the XML but are not the historical formal execution path. No actuator dynamics, lag, rate limit, or motor model is present. Native v1 therefore standardizes the audited direct wrench and does not alter the plant.
