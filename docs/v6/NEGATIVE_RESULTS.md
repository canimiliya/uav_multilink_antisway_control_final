# Negative Results

## V6 Yu 2026 adaptation

Best candidate `yu_b_007` passed 0/8 gates. It had 40.83% safety, zero success, 5.6509 m mean position RMSE, and 54 catastrophic strong pairs. The failure indicates that the point-load finite-time/CFO structure did not remain stable under this five-link task mapping and frozen slew authority.

## V6 SEP-NMPC adaptation

Best candidate `sep_b_002` passed 3/8 gates. It maintained 100% safety but achieved only 1.67% success and 0.4564 m position RMSE. Its paired-bootstrap interval versus Full-LQR was wholly negative, and it produced 54 catastrophic strong pairs.

## Interpretation boundaries

These are failures of frozen five-link adaptations, not evidence that the original published controllers fail on their own systems. No parameters were added after the budget, no Paper was replaced after suite freeze, and V6 Holdout was not opened.
