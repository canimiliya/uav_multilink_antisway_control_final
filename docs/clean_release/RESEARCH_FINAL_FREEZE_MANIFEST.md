# Research Final Freeze Manifest

Task: `P3-R1H-RESEARCH-FINAL-FREEZE-AND-STORAGE-AUDIT-RESUME-R2`

Project: UAV multi-link anti-sway control

Freeze date: `2026-08-12`

Scientific base HEAD: `d45e1ab7e1f340d138d870560d7915777b6ce2ef`

Scientific branch: `release/p3-r1g-visual-polish-and-controller-lineup`

Scientific remote branch: `origin/release/p3-r1g-visual-polish-and-controller-lineup`

P3-R1G local/remote synchronization: `true`

Freeze commit HEAD: the commit that adds this manifest and `LOCAL_STORAGE_AUDIT.md`; the exact SHA is recorded in the final task report.

Model:

- Path: `reproducibility/frozen/model/model_5link_controlled.xml`
- SHA256: `19105873c0fcc891ebb85efe6c20c378d5b77b6bf9003559e43ae47ca03d153d`

PID:

- Class: `V3CascadedTaskPID`
- Config/id: `hybrid_x007_y041_z041`
- Source: `src/uav_sway/v3/controllers.py`
- Evidence: `reproducibility/v3/r1r1/pid_freeze.json`

Full-LQR:

- Class: `V3FullStateLQR`
- Config/id: `full_lqr_048`
- Source: `src/uav_sway/v3/controllers.py`
- Evidence: `reproducibility/v3/r1/full_lqr_freeze.json`

SATC:

- Class: `SATC-OFMPC`
- Config/id: `satc_b_027`
- Source: `src/uav_sway/v5/satc_ofmpc.py`
- Evidence: `reproducibility/v5/self/self_freeze.json`

Shared control limits:

- Acceleration: `2.0 m/s^2`
- Slew: `0.25 m/s^2/update`
- Outer period: `0.05 s`

T1 frozen scenario:

- Initial sway: `[20, -16, 12, -8, 4]` deg
- Target delta: `[2.0, 1.7, 4.5]` m
- Move duration: `5.0 s`
- Wind: `0 m/s`

T2 frozen scenario:

- Initial sway: `[20, -16, 12, -8, 4]` deg
- Target delta: `[2.0, 1.7, 4.5]` m
- Move duration: `5.0 s`
- Wind: world `+X`, `3.0 m/s`
- Onset: `3.0 s`
- Ramp: half-cosine, `3.0--4.0 s`

T3 archived boundary:

- Hover, zero initial sway, world `+X`
- Historical LQR maximum recoverable wind: `3 m/s`
- Historical SATC maximum recoverable wind: `5 m/s`
- Rerun in this task: `false`

Stable definition source: `src/uav_sway/demo/recoverable_runner.py`, `_run_job`; no safety violations, final tip error `<=0.15 m`, final tip speed `<=0.20 m/s`, final 5 s tip RMS `<=0.20 m`, final 5 s joint RMS `<=1.0 deg`, and a continuous qualifying hold of at least `1.0 s`.

Holdout executed in this task: `false`

Controller retuned: `false`

Model modified: `false`

Scientific results modified: `false`

THIS FREEZE IS THE SCIENTIFIC SOURCE OF TRUTH FOR THE FUTURE CLEAN REPOSITORY.

## Known local files intentionally excluded from scientific freeze

`.vscode/`

- Reason: local VS Code configuration only; `IGNORE_LOCAL_ONLY`.

`scripts/meeting_demo/live_viewer.py`

- SHA256: `E6D1828A14604E3BF33640B935685C5E4FE5E7F736D1245B2B7FD8A174BBD928`
- Reason: optional interactive MuJoCo viewer; not used by frozen T1/T2/T3 evidence; candidate for later clean-repo migration; not part of this scientific freeze.
