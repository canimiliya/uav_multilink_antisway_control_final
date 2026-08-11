# P3-R1C realistic stress demo

This package is a `FUNCTIONAL_STRESS_DEMO` on the frozen MuJoCo model. It
contains ten fixed 40-second headless jobs (Full-LQR `full_lqr_048` and SATC
`satc_b_027`), distributed X/Y/XY30 wind, CSV/metrics, full-qpos render state
snapshots, native MuJoCo videos and meeting summaries.

The frozen model SHA256 is checked before the run. Controllers, gains, model
XML, Heading, Holdout, Paper and Governance are not modified or executed.
Stress performance is reported as observed; it is not a new Holdout or Paper
claim.
