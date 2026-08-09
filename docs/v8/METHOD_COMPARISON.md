# V8 Method Comparison

All values below are from the inherited 144-sample Development bank. `xu_v8_077` is diagnostic only and is not a frozen Paper baseline.

| Method | Safety | Success | Position RMSE (m) | Acquisition median (s) | Orientation (deg) | Effort | Runtime p95 (ms) |
|---|---:|---:|---:|---:|---:|---:|---:|
| Corrected PID | 100% | 13.19% | 0.3051 | 3.195 | 0.9056 | 5.3418 | 0.0708 |
| Full-LQR | 100% | 25.00% | 0.1470 | 4.2375 | 0.9213 | 1.2123 | 0.0372 |
| Task-LQR | 100% | 25.00% | 0.1869 | 2.1375 | 0.6470 | 1.4282 | 0.0387 |
| SATC (`satc_b_027`) | 100% | 60.42% | 0.1021 | 2.795 | 0.7472 | 1.0913 | 2.8391 |
| Xu2025 diagnostic best | 47.92% | 0.69% | 2.1982 | 1.16* | 21.4043 | 26.0732 | 0.3511 |

`*` Acquisition is based on one successful sample. It cannot support a speed claim.

The Paper candidate failed safety, success, position, bootstrap, strong-mean, strong-tail, and catastrophic-pair gates. Consequently, there is no formal `Paper > Traditional` result and no valid Paper-vs-SATC trade-off comparison.
