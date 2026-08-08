"""Analytic, reference-only x trajectories for the three S2 scenarios."""

from __future__ import annotations

import numpy as np


def smoothstep(tau: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    tau = np.asarray(tau, dtype=float)
    s = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
    ds = 30 * tau**2 - 60 * tau**3 + 30 * tau**4
    dds = 60 * tau - 180 * tau**2 + 120 * tau**3
    return s, ds, dds


def quintic_boundary_segment(
    time: np.ndarray,
    t0: float,
    t1: float,
    x0: float,
    v0: float,
    a0: float,
    x1: float,
    v1: float,
    a1: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Evaluate a quintic segment satisfying position, velocity, and acceleration endpoints."""
    t = np.asarray(time, dtype=float)
    duration = float(t1 - t0)
    if duration <= 0.0:
        raise ValueError("quintic segment requires t1 > t0")
    tau = np.clip((t - t0) / duration, 0.0, 1.0)

    c0 = float(x0)
    c1 = duration * float(v0)
    c2 = 0.5 * duration**2 * float(a0)
    r0 = float(x1) - (c0 + c1 + c2)
    r1 = duration * float(v1) - (c1 + 2.0 * c2)
    r2 = duration**2 * float(a1) - 2.0 * c2
    c3 = 10.0 * r0 - 4.0 * r1 + 0.5 * r2
    c4 = -15.0 * r0 + 7.0 * r1 - r2
    c5 = 6.0 * r0 - 3.0 * r1 + 0.5 * r2

    position = c0 + tau * (c1 + tau * (c2 + tau * (c3 + tau * (c4 + tau * c5))))
    d_tau = c1 + tau * (2.0 * c2 + tau * (3.0 * c3 + tau * (4.0 * c4 + tau * 5.0 * c5)))
    dd_tau = 2.0 * c2 + tau * (6.0 * c3 + tau * (12.0 * c4 + tau * 20.0 * c5))
    velocity = d_tau / duration
    acceleration = dd_tau / duration**2
    return position, velocity, acceleration


def _segment_move(t: np.ndarray, start: float, end: float, distance: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    duration = end - start
    tau = np.clip((t - start) / duration, 0.0, 1.0)
    s, ds, dds = smoothstep(tau)
    return distance * s, distance / duration * ds, distance / duration**2 * dds


def generate_reference(scenario: str, time: np.ndarray, config: dict) -> dict[str, np.ndarray]:
    t = np.asarray(time, dtype=float)
    if not np.isfinite(t).all():
        raise ValueError("time contains non-finite values")
    x = np.zeros_like(t)
    vx = np.zeros_like(t)
    ax = np.zeros_like(t)
    event = np.full(t.shape, "hover", dtype=object)
    if scenario == "approach_stop":
        m = (t >= 1.0) & (t < 2.0)
        x[m], vx[m], ax[m] = quintic_boundary_segment(t[m], 1.0, 2.0, 0.0, 0.0, 0.0, 0.375, 0.75, 0.0)
        m = (t >= 2.0) & (t < 5.0)
        x[m], vx[m], ax[m] = 0.375 + 0.75 * (t[m] - 2.0), 0.75, 0.0
        m = (t >= 5.0) & (t < 6.0)
        x[m], vx[m], ax[m] = quintic_boundary_segment(t[m], 5.0, 6.0, 2.625, 0.75, 0.0, 3.0, 0.0, 0.0)
        x[t >= 6.0], vx[t >= 6.0], ax[t >= 6.0] = 3.0, 0.0, 0.0
        event[(t >= 1.0) & (t < 2.0)] = "accelerate"
        event[(t >= 2.0) & (t < 5.0)] = "cruise"
        event[(t >= 5.0) & (t < 6.0)] = "decelerate"
        event[t >= 6.0] = "hold"
    elif scenario == "crosswind_hover":
        event[t >= 4.0] = "wind_onset"
    elif scenario == "gust_micro_adjust":
        m = (t >= 3.0) & (t < 5.0)
        dx, dv, da = _segment_move(t[m], 3.0, 5.0, 0.30)
        x[m], vx[m], ax[m] = dx, dv, da
        x[t >= 5.0], vx[t >= 5.0], ax[t >= 5.0] = 0.30, 0.0, 0.0
        event[(t >= 3.0) & (t < 5.0)] = "micro_adjust"
        event[(t >= 5.0) & (t <= 7.0)] = "gust"
        event[t > 7.0] = "hold"
    else:
        raise KeyError(scenario)
    y = np.full_like(t, float(config[scenario]["y_ref"]))
    z = np.full_like(t, float(config[scenario]["z_ref"]))
    yaw = np.full_like(t, float(config[scenario]["yaw_ref"]))
    return {"time": t, "x_ref": x, "vx_ref": vx, "ax_ref": ax, "y_ref": y, "z_ref": z, "yaw_ref": yaw, "event": event}
