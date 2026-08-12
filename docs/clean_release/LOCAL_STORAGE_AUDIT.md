# Local Storage Audit

Task: `P3-R1H-RESEARCH-FINAL-FREEZE-AND-STORAGE-AUDIT-RESUME-R2`

Repository audited: `D:\Desktop\my_project\uav_multilink_antisway_control_final`

Audit date: `2026-08-12`

Measurement basis: PowerShell recursive file sizes, reported in GiB using the Windows `1GB` divisor. The audit is read-only; no file was deleted or moved. The values below were measured before the Freeze commit was created.

## Repository totals

| Area | Size (GiB) |
|---|---:|
| Total project directory | 1.785518 |
| `.git` | 0.449653 |
| `outputs` | 0.541646 |
| `artifacts` | 0.002108 |
| `reproducibility` | 0.750533 |
| `src` | 0.002202 |
| `tests` | 0.002837 |
| `docs` | 0.000116 |
| `configs` | 0.000007 |
| `third_party` checkout | 0.024807 |
| `.papers` | absent |

Single files at least 100 MB: `.git/objects/pack/pack-ffc7485d90247a81e856b50a9d8711cfeb8e958a.pack` (`0.385813 GiB`).

Major file-type totals:

- `.csv`: `1.068720 GiB` across 1,238 files
- `.mp4`: `0.109330 GiB` across 65 files
- `.json`: `0.059418 GiB` across 47,041 files
- `.npz`: `0.027595 GiB` across 91 files
- `.gif`: `0.025515 GiB` across 11 files
- `.log`: `0.000011 GiB` across 38 files
- `.npy`: `0.000008 GiB` across 8 files

## Storage classification

The categories below are disjoint for the local-storage estimate. Tracked files are counted in A even when they are historical; the later clean-repository estimate separately excludes old run banks and generated outputs.

### A. TRACKED_AND_REMOTE

`1.091021 GiB` total:

- Tracked superproject files outside `.git`: `1.066213 GiB`
- `third_party/udaan` submodule checkout, restorable from its remote: `0.024807 GiB`

These are covered by the scientific branch/freeze commit or by the existing submodule reference once the remote backup is available.

### B. UNTRACKED_BUT_MUST_PRESERVE

`0.020582 GiB` total, 42 files:

- P3-R1G final boundary videos, final figures, final metrics/manifests, and boundary audit artifacts under `outputs/meeting_demo_boundary_v5/` and `artifacts/meeting_demo_boundary_v5/`: `0.020575 GiB`
- `scripts/meeting_demo/live_viewer.py`: `0.000006 GiB`

`scripts/meeting_demo/live_viewer.py` is a future clean-repository candidate, not part of the scientific Freeze. The final T1/T2 media and key boundary metrics are local preservation candidates. No new untracked T3 final video was identified; historical T3 media already tracked in prior release outputs is counted in A.

### C. REGENERABLE_LARGE_DATA

`0.219469 GiB` total, 47,034 files:

- Ignored reproducibility caches: `0.044053 GiB`
- Boundary sweep `render_states.npz`, `run.csv`, and non-final metrics: included in the remaining boundary-output portion
- `.benchmarks/`: `0.008841 GiB`
- Python/pytest caches and bytecode: included in this category

These are regenerable from the frozen code or are test/runtime caches. They were not removed in this task.

### D. HISTORICAL_DEBUG_OR_FAILED_EXPERIMENT

`0.004793 GiB` total, 34 files:

- Non-final/edge boundary media and plots under `outputs/meeting_demo_boundary_v5/`: approximately `0.004785 GiB`
- Root historical debug logs: approximately `0.000008 GiB`

These remain preserved as historical local evidence. The category is a later cleanup candidate only; this task performed no cleanup.

`.vscode/settings.json` is separately classified as `IGNORE_LOCAL_ONLY` and is not included in B, C, or D.

## Largest directories

Top 10 recursive directories:

1. `reproducibility/` — `0.750533 GiB`
2. `reproducibility/v2/` — `0.627408 GiB`
3. `outputs/` — `0.541646 GiB`
4. `.git/` — `0.449653 GiB`
5. `.git/objects/` — `0.393789 GiB`
6. `.git/objects/pack/` — `0.393776 GiB`
7. `reproducibility/v2/r1r1/` — `0.326217 GiB`
8. `reproducibility/v2/r1r1/runs/` — `0.325939 GiB`
9. `reproducibility/v2/r1/` — `0.300758 GiB`
10. `reproducibility/v2/r1/runs/` — `0.300550 GiB`

All recursive directories at least 100 MB additionally include:

- `outputs/meeting_demo_boundary_v5/` — `0.183734 GiB`
- `outputs/meeting_demo_recoverable_v4/` — `0.151054 GiB`
- `outputs/meeting_demo_extreme_v3/` — `0.130181 GiB`
- `outputs/meeting_demo_recoverable_v4/T3/` — `0.116390 GiB`
- `reproducibility/v2/r1r1/runs/task_lqr/` — `0.110624 GiB`
- `reproducibility/v2/r1r1/runs/lqr/` — `0.110297 GiB`
- `reproducibility/v2/r1r1/runs/pid/` — `0.105018 GiB`
- `outputs/meeting_demo_boundary_v5/T2/` — `0.102245 GiB`
- `reproducibility/v2/r1/runs/task_lqr/` — `0.101843 GiB`
- `reproducibility/v2/r1/runs/lqr/` — `0.101599 GiB`

## Top 20 files

1. `.git/objects/pack/pack-ffc7485d90247a81e856b50a9d8711cfeb8e958a.pack` — `0.385813 GiB`
2. `.git/modules/third_party/udaan/objects/pack/pack-2c981ff265b965c7c8f4273e1127ccc26c06df7d.pack` — `0.055314 GiB`
3. `reproducibility/v9/training/training_bank.npz` — `0.018002 GiB`
4. `outputs/meeting_demo_extreme_v3/T1/T1_AGGRESSIVE_LQR_vs_SATC.mp4` — `0.009189 GiB`
5. `.git/objects/pack/pack-131c5fe472fe46a49f7d21525ac0dea89505d7d6.pack` — `0.007348 GiB`
6. `outputs/meeting_demo_extreme_v3/T3/X/T3_X_LQR_vs_SATC.mp4` — `0.006636 GiB`
7. `outputs/meeting_demo_boundary_v5/T1/6.0s/satc_b_027/run.csv` — `0.006521 GiB`
8. `outputs/meeting_demo_recoverable_v4/T1/satc_b_027/run.csv` — `0.006521 GiB`
9. `outputs/meeting_demo_boundary_v5/T1/5.5s/satc_b_027/run.csv` — `0.006500 GiB`
10. `outputs/meeting_demo_boundary_v5/T1/5.0s/satc_b_027/run.csv` — `0.006498 GiB`
11. `outputs/meeting_demo_boundary_v5/T2/0mps/satc_b_027/run.csv` — `0.006498 GiB`
12. `outputs/meeting_demo_recoverable_v4/T3/03mps/satc_b_027/run.csv` — `0.006491 GiB`
13. `outputs/meeting_demo_recoverable_v4/T3/04mps/satc_b_027/run.csv` — `0.006476 GiB`
14. `outputs/meeting_demo_boundary_v5/T2/1mps/satc_b_027/run.csv` — `0.006459 GiB`
15. `outputs/meeting_demo_recoverable_v4/T3/05mps/satc_b_027/run.csv` — `0.006453 GiB`
16. `outputs/meeting_demo_recoverable_v4/T1/full_lqr_048/run.csv` — `0.006432 GiB`
17. `outputs/meeting_demo_boundary_v5/T1/6.0s/full_lqr_048/run.csv` — `0.006432 GiB`
18. `outputs/meeting_demo_boundary_v5/T2/2mps/satc_b_027/run.csv` — `0.006432 GiB`
19. `outputs/meeting_demo_boundary_v5/T1/5.5s/full_lqr_048/run.csv` — `0.006432 GiB`
20. `outputs/meeting_demo_recoverable_v4/T3/03mps/full_lqr_048/run.csv` — `0.006411 GiB`

## Space estimate

The expected clean-repository source payload is estimated at `0.112663 GiB`: source, scripts, tests, configs, docs, current frozen evidence, and the Udaan submodule, excluding Git history, old V2 run banks, generated outputs, caches, and historical local logs. Adding the approved B preservation set gives an operational clean-export footprint of:

`ESTIMATED_CLEAN_REPO_SIZE_GB = 0.133244`

The estimated reclaimable local space after preserving B outside the old repository is:

`ESTIMATED_RECLAIMABLE_SIZE_GB = 1.652274`

This is an estimate only. No deletion or migration was performed.

## New clean-repository target

- Local target exists: `true`
- Path: `D:\Desktop\my_project\uav-multilink-antisway-control`
- Remote target: `https://github.com/canimiliya/uav_multilink_antisway_control.git`
- `NEW_REMOTE_MODIFIED = false`
- No clone, init, copy, or push was performed.
