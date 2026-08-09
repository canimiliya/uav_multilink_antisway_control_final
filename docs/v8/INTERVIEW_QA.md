# V8 Interview Q&A

## Did the recent-Paper baseline succeed?

No. We implemented and tuned a faithful Xu 2025 composite backstepping plus finite-time disturbance-observer adaptation, but none of 128 configurations passed the preregistered Development gate. We preserved the negative result and did not open Holdout.

## Why was Xu 2025 selected instead of Yan 2026?

The selection order was frozen before performance. Yan 2026 and Yan 2024 did not have complete legally obtainable primary equations in the source audit. Xu 2025 was the first complete CC BY primary source, so it was selected without performance-based paper switching.

## Why use two five-link modes if Xu models one cable?

Earlier single-direction adaptations discarded all internal-chain structure. V8 first identified all five passive modes and retained the two low-frequency, observable, controllable modes. Their plant-only small-signal tip response matched the full linear chain, but this did not guarantee nonlinear closed-loop success.

## Why did the adaptation fail despite modal parity?

Model-reduction parity and controller qualification are different gates. Xu's method assumes a point load, direct thrust-vector synthesis, smooth references, and 100 Hz control. Our fair benchmark uses five massive links, task-tip steps, a 20 Hz acceleration interface, slew limits, and a shared inner loop. The adapted angular cascade saturated and produced attitude, height, and joint safety failures.

## Is SATC proven better than Xu 2025?

Not as a formal Paper comparison. No V8 Paper candidate qualified or froze, and it never entered Holdout. We can report that the V8 adaptation failed Development; we cannot claim SATC beat the original paper or a validated Paper baseline.

## Why was Holdout not used?

The Holdout was frozen before V8 and remained unseen. The contract permitted execution only after a Paper candidate passed Development and was frozen. That condition failed, so opening even one Holdout sample would have violated the protocol.

## What remains valid?

The three Traditional baselines and SATC remain frozen. The highest validated claim is still `V5_SELF_OVERALL_HOLDOUT_WIN`. The project remains pure simulation and the broader goal requiring a recent Paper baseline is incomplete.
