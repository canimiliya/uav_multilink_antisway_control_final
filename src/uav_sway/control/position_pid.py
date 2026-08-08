"""Position-only outer PID controller with deterministic limits and anti-windup."""

from __future__ import annotations

from dataclasses import dataclass

from .base import ControlState, ReferenceState


@dataclass
class PIDDiagnostics:
    position_error_x: float = 0.0
    velocity_error_x: float = 0.0
    pid_integral_x: float = 0.0
    ax_pid_feedback: float = 0.0
    ax_reference_feedforward: float = 0.0
    ax_cmd_raw: float = 0.0
    ax_cmd_amplitude_limited: float = 0.0
    ax_cmd_limited: float = 0.0
    ax_saturated: bool = False
    ax_slew_limited: bool = False


class PositionPID:
    """PID reads only UAV x position/velocity and reference x derivatives."""

    def __init__(self, kp: float, kd: float, ki: float, ax_min: float = -2.0,
                 ax_max: float = 2.0, slew_limit: float = 0.25,
                 integral_limit: float = 1.0):
        self.kp = float(kp)
        self.kd = float(kd)
        self.ki = float(ki)
        self.ax_min = float(ax_min)
        self.ax_max = float(ax_max)
        self.slew_limit = float(slew_limit)
        self.integral_limit = float(integral_limit)
        self.integral = 0.0
        self.previous_command = 0.0
        self.diagnostics = PIDDiagnostics()

    def reset(self, state: ControlState, reference: ReferenceState) -> None:
        del state, reference
        self.integral = 0.0
        self.previous_command = 0.0
        self.diagnostics = PIDDiagnostics()

    def command(self, state: ControlState, reference: ReferenceState, dt: float) -> float:
        if dt <= 0.0:
            raise ValueError("PID dt must be positive")
        ex = float(state.uav_x - reference.x_ref)
        ev = float(state.uav_vx - reference.vx_ref)
        feedback = -self.kp * ex - self.kd * ev - self.ki * self.integral
        raw = float(reference.ax_ref + feedback)
        amplitude = float(min(self.ax_max, max(self.ax_min, raw)))
        amplitude_saturated = amplitude != raw

        # Conditional integration uses the direction of the unsaturated
        # feedback. Integral state is bounded before it is used again.
        candidate_integral = self.integral + ex * dt
        candidate_integral = min(self.integral_limit, max(-self.integral_limit, candidate_integral))
        pushing_high = amplitude_saturated and raw > self.ax_max and ex < 0.0
        pushing_low = amplitude_saturated and raw < self.ax_min and ex > 0.0
        if not (pushing_high or pushing_low):
            self.integral = candidate_integral
        self.integral = min(self.integral_limit, max(-self.integral_limit, self.integral))

        # Recompute feedback after the conditional integral update, then apply
        # the specified absolute limit followed by a per-update slew limit.
        feedback = -self.kp * ex - self.kd * ev - self.ki * self.integral
        raw = float(reference.ax_ref + feedback)
        amplitude = float(min(self.ax_max, max(self.ax_min, raw)))
        delta = amplitude - self.previous_command
        limited = self.previous_command + min(self.slew_limit, max(-self.slew_limit, delta))
        limited = float(min(self.ax_max, max(self.ax_min, limited)))
        slew_limited = limited != amplitude
        saturated = amplitude != raw or slew_limited
        self.previous_command = limited
        self.diagnostics = PIDDiagnostics(
            position_error_x=ex, velocity_error_x=ev, pid_integral_x=float(self.integral),
            ax_pid_feedback=float(feedback), ax_reference_feedforward=float(reference.ax_ref),
            ax_cmd_raw=float(raw), ax_cmd_amplitude_limited=float(amplitude),
            ax_cmd_limited=limited, ax_saturated=bool(saturated), ax_slew_limited=bool(slew_limited),
        )
        return limited
