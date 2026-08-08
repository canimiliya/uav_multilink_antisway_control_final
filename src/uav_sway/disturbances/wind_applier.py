"""Apply one frozen wind sample to every configured rigid body."""

from __future__ import annotations

import mujoco

from .aerodynamics import AerodynamicConfig, compute_body_wind_forces


def clear_and_apply_wind(model, data, model_config, aerodynamic_config: AerodynamicConfig, wind_x: float) -> dict[str, float]:
    """Clear all prior wrench state, then apply independent COM drag forces."""
    data.xfrc_applied[:] = 0.0
    return compute_body_wind_forces(model, data, aerodynamic_config, model_config, wind_x)


def body_ids(model, n_links: int) -> dict[str, int]:
    names = ["quadrotor", *[f"link_{i}" for i in range(1, n_links + 1)], "cutter"]
    result = {}
    for name in names:
        result[name] = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name))
        if result[name] < 0:
            raise KeyError(name)
    return result
