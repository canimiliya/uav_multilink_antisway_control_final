# Paper-Ready Results

## Benchmark and protocol

All controllers used the same MuJoCo five-link plant, cutter-tip task metrics, geometric inner loop, ±2 m/s² world-frame acceleration authority, and 0.25 m/s²/update slew limit. V6 Paper adaptation used only the new 120-sample Development split. The 96-sample V6 Holdout remained locked because no Paper qualified.

## Main comparator table

Use `FINAL_COMPARISON_TABLE.csv` for exact Development values. The frozen SATC reference achieved 100.00% safety, 90.00% success, 0.09687 m mean position RMSE, and 3.7225 s acquisition on V6 Development without retuning. These are supporting cross-version Development results; the formal SATC claim remains its V5 unseen Holdout win.

## Recent-Paper adaptations

YU2026-FT-CFO preserved the compensation-function observer, finite-time signed-power feedback, and an equivalent multi-link swing-energy term. SEP2026 preserved finite-horizon constrained optimization, storage shaping, and strict passivity filtering; obstacle HOCBFs were inapplicable because the frozen benchmark has no obstacle task. Neither method met Development qualification. Therefore no Paper was evaluated on V6 Holdout, and no Paper-versus-SATC Holdout statement is valid.

## Recommended result sentence

"A preregistered adaptation study of two recent suspended-payload controllers produced no Development-qualified external advanced baseline under the frozen five-link acceleration interface; consequently, the unseen V6 Holdout remained unopened, while the previously established V5 SATC Overall Holdout win was retained unchanged."
