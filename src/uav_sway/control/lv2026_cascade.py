"""Causal five-link adaptation of the Lv et al. suspension-point cascade.

This module is deliberately named ``LV2026-CASCADE-ADAPTED``.  It is not an
exact reproduction of the paper controller: the five-link benchmark exposes
an equivalent planar swing state and uses the frozen PID-005 x bridge.
"""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np

from .acceleration_limiter import AccelerationLimiter
from .base import ControlState, ReferenceState
from .position_pid import PositionPID


GRAVITY = 9.81
AX_MIN = -2.0
AX_MAX = 2.0
AX_SLEW = 0.25
PID_005 = {"kp": 0.8, "kd": 4.0, "ki": 0.1, "integral_limit": 1.0}
PAPER_K_BETA = 3.2
PAPER_KP_BETA = 3.2
GAIN_MULTIPLIERS = (0.5, 1.0, 2.0)


def _id(model, object_type, name: str) -> int:
    value = int(mujoco.mj_name2id(model, object_type, name))
    if value < 0:
        raise KeyError(name)
    return value


@dataclass(frozen=True)
class EquivalentSwingState:
    suspension_point_world: np.ndarray
    cutter_com_world: np.ndarray
    relative_vector_world: np.ndarray
    relative_velocity_world: np.ndarray
    equivalent_length_m: float
    alpha_eq_rad: float
    beta_eq_rad: float
    alpha_dot_eq_rad_s: float
    beta_dot_eq_rad_s: float


@dataclass(frozen=True)
class LV2026Diagnostics:
    beta_eq_rad: float
    beta_dot_eq_rad_s: float
    alpha_eq_rad: float
    alpha_dot_eq_rad_s: float
    equivalent_length_m: float
    beta_error_rad: float
    shaped_beta_error_rad_s: float
    paper_beta_acceleration_m_s2: float
    paper_swing_correction_ax: float
    nominal_pid_ax: float
    ax_cmd_raw: float
    ax_cmd_amplitude_limited: float
    ax_cmd_limited: float
    ax_saturated: bool
    ax_slew_limited: bool
    swing_correction_saturated: bool


def _angle_and_rate(rx: float, rz: float, vx: float, vz: float) -> tuple[float, float]:
    denominator = float(rx * rx + rz * rz)
    if denominator <= 1.0e-14:
        raise ValueError("equivalent swing vector is too short")
    # MuJoCo is z-up.  The equilibrium vector points along -z, so beta is
    # positive when the cutter COM is perturbed toward +x:
    # beta = atan2(r_x, -r_z).
    angle = float(np.arctan2(rx, -rz))
    rate = float((-rz * vx + rx * vz) / denominator)
    return angle, rate


def equivalent_swing_from_vectors(relative_vector_world, relative_velocity_world) -> EquivalentSwingState:
    """Compute both planar equivalent angles from causal relative state."""

    r = np.asarray(relative_vector_world, dtype=float).reshape(3)
    v = np.asarray(relative_velocity_world, dtype=float).reshape(3)
    if not np.isfinite(r).all() or not np.isfinite(v).all():
        raise ValueError("equivalent swing vectors must be finite")
    length = float(np.linalg.norm(r))
    if length <= 1.0e-12:
        raise ValueError("equivalent swing length must be positive")
    alpha, alpha_dot = _angle_and_rate(float(r[1]), float(r[2]), float(v[1]), float(v[2]))
    beta, beta_dot = _angle_and_rate(float(r[0]), float(r[2]), float(v[0]), float(v[2]))
    return EquivalentSwingState(
        np.zeros(3), np.zeros(3), r.copy(), v.copy(), length,
        alpha, beta, alpha_dot, beta_dot,
    )


class EquivalentSwingReader:
    """Read the upper suspension point, cutter COM, and causal Jacobian rate."""

    def __init__(self, model) -> None:
        self.suspension_body_id = _id(model, mujoco.mjtObj.mjOBJ_BODY, "link_1")
        self.cutter_body_id = _id(model, mujoco.mjtObj.mjOBJ_BODY, "cutter")

    def read(self, model, data) -> EquivalentSwingState:
        suspension = np.asarray(data.xpos[self.suspension_body_id], dtype=float).copy()
        cutter_com = np.asarray(data.xipos[self.cutter_body_id], dtype=float).copy()
        suspension_jacp = np.zeros((3, model.nv), dtype=float)
        suspension_jacr = np.zeros((3, model.nv), dtype=float)
        cutter_jacp = np.zeros((3, model.nv), dtype=float)
        cutter_jacr = np.zeros((3, model.nv), dtype=float)
        mujoco.mj_jacBody(model, data, suspension_jacp, suspension_jacr, self.suspension_body_id)
        mujoco.mj_jacBodyCom(model, data, cutter_jacp, cutter_jacr, self.cutter_body_id)
        relative = cutter_com - suspension
        relative_velocity = (cutter_jacp - suspension_jacp) @ np.asarray(data.qvel, dtype=float)
        state = equivalent_swing_from_vectors(relative, relative_velocity)
        return EquivalentSwingState(
            suspension, cutter_com, state.relative_vector_world, state.relative_velocity_world,
            state.equivalent_length_m, state.alpha_eq_rad, state.beta_eq_rad,
            state.alpha_dot_eq_rad_s, state.beta_dot_eq_rad_s,
        )


def paper_middle_loop_beta_acceleration(beta_eq: float, beta_dot_eq: float,
                                        k_beta: float, k_p_beta: float) -> tuple[float, float, float]:
    """Return Eq.18's shaped beta acceleration and its two causal errors."""

    if k_beta <= 0.0 or k_p_beta <= 0.0:
        raise ValueError("paper gains must be positive")
    beta_error = -float(beta_eq)  # beta_d = 0
    shaped_error_rate = -float(beta_dot_eq) + float(k_beta) * beta_error
    beta_acceleration = (1.0 - float(k_beta) ** 2) * beta_error + (float(k_beta) + float(k_p_beta)) * shaped_error_rate
    return float(beta_acceleration), float(beta_error), float(shaped_error_rate)


def paper_suspension_acceleration_correction(beta_eq: float, beta_dot_eq: float,
                                             equivalent_length_m: float,
                                             k_beta: float, k_p_beta: float,
                                             gravity: float = GRAVITY) -> tuple[float, float, float, float]:
    """Map the Eq.18 virtual input through the equivalent pendulum bridge.

    For ``r=[l sin(beta), 0, -l cos(beta)]`` in a z-up world, the planar
    pendulum equation is ``beta_ddot = -(a_x + g sin(beta))/l``.  Therefore
    the desired suspension-point correction is ``-l*beta_ddot-g*sin(beta)``.
    """

    length = float(equivalent_length_m)
    if length <= 1.0e-12 or not np.isfinite(length):
        raise ValueError("equivalent length must be positive")
    beta_accel, beta_error, shaped_error_rate = paper_middle_loop_beta_acceleration(beta_eq, beta_dot_eq, k_beta, k_p_beta)
    correction = -length * beta_accel - float(gravity) * np.sin(float(beta_eq))
    return float(correction), float(beta_accel), beta_error, shaped_error_rate


def enumerate_grid() -> list[dict]:
    rows = []
    index = 0
    for k_beta_multiplier in GAIN_MULTIPLIERS:
        for k_p_beta_multiplier in GAIN_MULTIPLIERS:
            rows.append({
                "candidate_id": f"lv2026_{index:03d}",
                "k_beta_multiplier": k_beta_multiplier,
                "k_p_beta_multiplier": k_p_beta_multiplier,
                "k_beta": PAPER_K_BETA * k_beta_multiplier,
                "k_p_beta": PAPER_KP_BETA * k_p_beta_multiplier,
            })
            index += 1
    return rows


class LV2026CascadeAdapted:
    """PID-005 x bridge plus the causal Lv-inspired swing correction."""

    def __init__(self, k_beta: float, k_p_beta: float):
        self.k_beta = float(k_beta)
        self.k_p_beta = float(k_p_beta)
        allowed = {PAPER_K_BETA * m for m in GAIN_MULTIPLIERS}
        allowed_p = {PAPER_KP_BETA * m for m in GAIN_MULTIPLIERS}
        if self.k_beta not in allowed or self.k_p_beta not in allowed_p:
            raise ValueError("LV2026 gains are outside the frozen nine-case grid")
        self.pid = PositionPID(PID_005["kp"], PID_005["kd"], PID_005["ki"], AX_MIN, AX_MAX, AX_SLEW, PID_005["integral_limit"])
        self.limiter = AccelerationLimiter(AX_MIN, AX_MAX, AX_SLEW)
        self.swing_reader = None
        self.diagnostics = LV2026Diagnostics(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, False, False, False)

    def reset(self, state: ControlState | None = None, reference: ReferenceState | None = None) -> None:
        self.pid.reset(state, reference)
        self.limiter.reset(0.0)
        self.swing_reader = None

    def command(self, state: ControlState, reference: ReferenceState, dt: float, model, data) -> float:
        if self.swing_reader is None:
            self.swing_reader = EquivalentSwingReader(model)
        nominal = float(self.pid.command(state, reference, dt))
        swing = self.swing_reader.read(model, data)
        correction, beta_accel, beta_error, shaped_rate = paper_suspension_acceleration_correction(
            swing.beta_eq_rad, swing.beta_dot_eq_rad_s, swing.equivalent_length_m,
            self.k_beta, self.k_p_beta,
        )
        raw = nominal + correction
        limited = float(self.limiter.limit(raw))
        limit_diag = self.limiter.diagnostics
        self.diagnostics = LV2026Diagnostics(
            swing.beta_eq_rad, swing.beta_dot_eq_rad_s, swing.alpha_eq_rad, swing.alpha_dot_eq_rad_s,
            swing.equivalent_length_m, beta_error, shaped_rate, beta_accel, correction, nominal,
            raw, limit_diag.amplitude_limited, limited, limit_diag.saturated,
            limit_diag.slew_limited, bool(abs(correction) > AX_MAX),
        )
        return limited


def synthetic_equivalent_swing_audit() -> dict:
    equilibrium = equivalent_swing_from_vectors([0.0, 0.0, -2.5], [0.0, 0.0, 0.0])
    positive_x = equivalent_swing_from_vectors([0.01, 0.0, -2.5], [0.0, 0.0, 0.0])
    correction, beta_accel, _, _ = paper_suspension_acceleration_correction(
        positive_x.beta_eq_rad, positive_x.beta_dot_eq_rad_s, positive_x.equivalent_length_m,
        PAPER_K_BETA, PAPER_KP_BETA,
    )
    return {
        "coordinate_system": "MuJoCo world z-up, gravity [0, 0, -9.81]",
        "r_eq": "p_cutter_com - p_susp",
        "beta_eq": "atan2(r_eq_x, -r_eq_z), positive toward +x",
        "beta_dot_eq": "(-r_eq_z*v_rel_x + r_eq_x*v_rel_z) / (r_eq_x^2 + r_eq_z^2)",
        "equilibrium_beta_rad": equilibrium.beta_eq_rad,
        "positive_x_beta_rad": positive_x.beta_eq_rad,
        "positive_x_correction_ax_m_s2": correction,
        "positive_x_target_beta_acceleration_m_s2": beta_accel,
        "positive_x_sign_pass": bool(positive_x.beta_eq_rad > 0.0 and correction > 0.0 and beta_accel < 0.0),
        "causal": True,
    }
