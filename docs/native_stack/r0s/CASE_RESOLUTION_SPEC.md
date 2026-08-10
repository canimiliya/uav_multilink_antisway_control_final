# Native case resolution specification

`native-case-semantics-v1` is a pure deterministic mapping from an unchanged v1 identity record to one complete physical experiment. The Development and Holdout generators are identical; only their original disjoint seed namespaces differ. Every resolved case records its initial condition, exact target, trajectory geometry and timing, wind waveform and direction, distributed body application, execution sampling, six canonical signal hashes, and one semantic fingerprint.

The original identity hashes remain `0f03df8fe11310f6357197a9d03b605476831c76db58f64d04e92535e6df9473` and `63e6192faf992494f5a78f4c008d844564b0015fc26b43a94a0c98da659b2538`. Targets use PCG64 and remain inside the 2 m trim-centered mission envelope. No controller output or provisional R1 performance was used.
