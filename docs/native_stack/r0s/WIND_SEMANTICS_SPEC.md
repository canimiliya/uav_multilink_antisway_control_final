# Wind semantics specification

Wind is sampled at 200 Hz and held to the 1000 Hz physics ticks. Calm is zero; moderate and strong sustained are 1.5 and 3.0 m/s constants after 4 s; strong transient is a 3.0 m/s one-cosine gust from 5 s to 7 s; stochastic wind is a seeded PCG64 first-order low-pass Gaussian process (sigma 0.8 m/s, tau 1 s, clip 3 m/s); ramp reaches 3.0 m/s in 3 s after 4 s and holds.

Aligned/opposed/cross are defined from the horizontal task displacement, with a seeded horizontal fallback for degenerate displacement. The existing distributed quadratic-drag model is applied independently at the COM of the quadrotor, five links, and cutter. Existing Cd, projected-area proxies, air density, and the no-aerodynamic-torque rule are unchanged.
