# V2 Final Technical Report

## Final result

V2 is closed with partial scientific success. The frozen traditional baselines remain PID, LQR, and Task-LQR. OF-TSRMPC is closed negative. DR-TSRMPC produced substantial development-set improvements in task success, 3D positioning, and ramp-wind disturbance rejection, but it did not improve the acquisition-speed gates and is therefore not an overall winner. LV2026-CASCADE-ADAPTED did not win development. No Holdout was executed.

The precise final status is recorded in [`final_gate.json`](../reproducibility/v2/final/final_gate.json). Existing R3R2 and R4 gate files remain preserved; their explanatory closure clarifications do not rewrite the original gate results.

## Frozen methods and evidence

| Route | Final status | Evidence-backed interpretation |
|---|---|---|
| PID / LQR / Task-LQR | Traditional baselines frozen | `pid_005`, `lqr_041`, and `task_lqr_001` remain the comparison envelope. |
| OF-TSRMPC | `CLOSED_NEGATIVE` | Structural failure was audited and the route was not reopened. |
| DR-TSRMPC | `CLOSED_PARTIAL_SCIENTIFIC_SUCCESS` | Development-set gains in success, 3D position, and ramp rejection; no overall win because acquisition requirements failed. |
| LV2026-CASCADE-ADAPTED | `CLOSED_NEGATIVE` | Best candidate `lv2026_000` failed success, position, acquisition, paired acquisition, and ramp gates. |
| Holdout | `NEVER_EXECUTED` | The 23 frozen samples remain unused. |

## DR-TSRMPC development result

Across the six Stage-2 candidates, success was 100%, 3D position RMSE was 0.08215–0.08238 m, ramp peak error was 0.08850–0.09080 m, ramp steady-state error was 0.07302–0.07713 m, and solver P95 was 1.72–2.41 ms. The global acquisition median was 2.43–2.50 s and every paired acquisition improvement was non-positive. This is a real development-set trade-off, not an overall domination result.

## Paper route result

The best LV2026-CASCADE-ADAPTED candidate was `lv2026_000`: 56.1404% success, 0.1421516575 m 3D position RMSE, 2.3725 s acquisition median, −0.1860% paired acquisition improvement, 0.1462048303 m ramp peak error, and 0.1959644033 m ramp steady-state error. It was safe on 57/57 development samples and passed runtime-limit checks, but failed the remaining frozen eligibility gates. No candidate was selected and no Holdout was opened.

## What V2 does and does not establish

V2 did not establish an advanced controller that dominates the frozen traditional metric envelope under all preregistered gates. The safe scientific statement is that DR-TSRMPC showed substantial development-set improvements in task success, 3D positioning, and ramp-wind disturbance rejection, while sacrificing acquisition speed. It is not valid to state “Advanced > Traditional” or to call the LV2026 paper invalid.

## Experimental-design limitation

Advanced controller-specific authority was restricted to the x anti-sway channel, while shared y/z control was identical across methods and the formal metric evaluated the full 3D cutter-tip task. Thus advanced methods could not independently improve the shared y/z channels. This is a V2 experimental-design limitation, not a reason to relax the frozen gate or alter the negative result.

## Holdout and future work

The Holdout contains eight diagonal targets, random-wind seeds 1000–1019, constant winds at 2.25 and 3.75 m/s, and a 0→3.75 m/s ramp. It was excluded from tuning, model selection, and final testing. A future V3 may define a new fair contract in which every method receives the same `[a_x, a_y, a_z]` interface, but this task does not create V3 or reuse the V2 Holdout automatically.
