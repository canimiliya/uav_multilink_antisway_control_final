# V2 Final Claim Matrix

| Claim | Status | Boundary / evidence |
|---|---|---|
| PID, LQR, and Task-LQR are frozen traditional baselines | SUPPORTED | `reproducibility/v2/r1/*_freeze.json`, `r1r1/gate.json` |
| OF-TSRMPC is closed negative | SUPPORTED | `reproducibility/v2/r3/gate.json`, `r3r1/gate.json` |
| DR-TSRMPC improves development-set success, 3D positioning, and ramp rejection | SUPPORTED WITH SCOPE | `r3r2/development_summary.json`; development only |
| DR-TSRMPC is an overall advanced winner | NOT ESTABLISHED | Global and paired acquisition gates failed |
| LV2026-CASCADE-ADAPTED wins development | NOT ESTABLISHED | `lv2026_000` failed five eligibility gates |
| V2 established Advanced > Traditional | NOT ESTABLISHED | No method passed the full preregistered envelope |
| V2 Holdout was measured | FALSE | Holdout execution remained disabled; 23 samples preserved unused |
| V2 has an x-only advanced-authority/full-3D-metric limitation | SUPPORTED | `reproducibility/v2/final/v2_limitations.json` |

The following claims are explicitly prohibited by the evidence: “Advanced > Traditional”, “LV2026 paper is invalid”, and any statement implying measured Holdout performance.
