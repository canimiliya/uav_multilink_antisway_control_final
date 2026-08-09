"""Shock-aware transient coordination around the frozen CART-OFMPC model."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from uav_sway.v3.controllers import V3ControllerDiagnostics, _V3ControllerBase
from uav_sway.v3.observation import V3Observation, V3Reference
from uav_sway.v4.cart_ofmpc import CARTOFMPC


@dataclass(frozen=True)
class SATCOFMPCDiagnostics(V3ControllerDiagnostics):
    shock_score: float
    reference_shock: float
    innovation_shock: float
    innovation_norm: float
    innovation_rate: float
    disturbance_equivalent_input: np.ndarray
    desired_task_motion: np.ndarray
    conflict_index: float
    cancellation_index: float
    offset_engagement: float
    coordination_weight: float
    slew_headroom: float
    amplitude_headroom: float
    trust: float
    residual_clip_fraction: float
    residual_unrepresented_norm: float
    residual_debt_norm: float
    steady_equality_residual: float
    steady_task_residual: float
    steady_feasible: bool
    steady_constraint_active: bool
    reachable_constraint_active: bool
    steady_target_state: np.ndarray
    requested_steady_input: np.ndarray
    feasible_steady_input: np.ndarray
    steady_command: np.ndarray
    backbone_command: np.ndarray
    qp_correction: np.ndarray
    mpc_command: np.ndarray
    robust_command: np.ndarray
    final_physical_command: np.ndarray
    constraint_activity: bool
    qp_status: str
    qp_iterations: int
    solve_time_ms: float
    limiter_mismatch: float


class SATCOFMPC(_V3ControllerBase):
    """Causal CART plus bumpless final-input transient coordination.

    The controller never receives wind truth.  A reference jump and the rate
    of the one-step model innovation create a hysteretic shock score.  During
    a shock, CART's offset command is blended bumplessly with the immutable
    Full-LQR stabilizer.  Conflict is geometric: opposition between the
    innovation-equivalent acceleration and desired task motion.
    """

    def __init__(
        self,
        a: np.ndarray,
        b: np.ndarray,
        c_task: np.ndarray,
        cart_gain: np.ndarray,
        robust_gain: np.ndarray,
        parameters: dict,
    ) -> None:
        super().__init__()
        self.a = np.asarray(a, dtype=float).reshape(20, 20)
        self.b = np.asarray(b, dtype=float).reshape(20, 3)
        self.b_pinv = np.linalg.pinv(self.b)
        self.cart_gain = np.asarray(cart_gain, dtype=float).reshape(3, 20)
        self.robust_gain = np.asarray(robust_gain, dtype=float).reshape(3, 20)
        self.parameters = dict(parameters)
        self.cart = CARTOFMPC(self.a, self.b, c_task, self.cart_gain, parameters)
        self.reset()

    def reset(self) -> None:
        super().reset()
        self.cart.reset()
        self.previous_reference: np.ndarray | None = None
        self.previous_innovation = np.zeros(20)
        self.shock_score = 0.0
        self.conflict_memory = 0.0
        self.offset_engagement = 1.0
        self._set_empty_diagnostics()

    def _set_empty_diagnostics(self) -> None:
        z3 = np.zeros(3); z20 = np.zeros(20)
        self.diagnostics = SATCOFMPCDiagnostics(
            z3, z3, z3, np.zeros(3, dtype=bool), np.zeros(3, dtype=bool), 0.0, z3,
            0.0, 0.0, 0.0, 0.0, 0.0, z3, z3, 0.0, 0.0, 1.0, 0.0, 0.25, 2.0,
            1.0, 0.0, 0.0, 0.0, 0.0, 0.0, True, False, False, z20, z3, z3, z3, z3, z3, z3, z3, z3,
            False, "not_run", 0, 0.0, 0.0,
        )

    @staticmethod
    def _opposition(first: np.ndarray, second: np.ndarray) -> float:
        denominator = float(np.linalg.norm(first) * np.linalg.norm(second))
        if denominator <= 1.0e-10:
            return 0.0
        return float(np.clip(-(first @ second) / denominator, 0.0, 1.0))

    def command(self, observation: V3Observation, reference: V3Reference, dt: float = 0.05) -> np.ndarray:
        if abs(float(dt) - 0.05) > 1.0e-12:
            raise ValueError("SATC-OFMPC requires the frozen 0.05 s outer period")
        state = np.asarray(observation.full_state_error, dtype=float).reshape(20)
        reference_position = np.asarray(reference.tip_position_world, dtype=float).reshape(3)
        previous_actual = self.limiter.previous.copy()

        # CART sees the same previous physical command and is corrected to the
        # final applied command below, keeping its estimator causally honest.
        self.cart.limiter.previous = previous_actual.copy()
        cart_command = self.cart.command(observation, reference, dt)
        cart_diag = self.cart.diagnostics
        innovation = np.asarray(cart_diag.residual_raw, dtype=float)
        innovation_rate = float(np.linalg.norm(innovation - self.previous_innovation))
        innovation_norm = float(np.linalg.norm(innovation))
        self.previous_innovation = innovation.copy()

        if self.previous_reference is None:
            reference_jump = 0.0
        else:
            reference_jump = float(np.linalg.norm(reference_position - self.previous_reference))
        self.previous_reference = reference_position.copy()
        reference_shock = float(1.0 - np.exp(-float(self.parameters["shock_reference_gain"]) * reference_jump))
        innovation_shock = float(np.tanh(innovation_rate / float(self.parameters["shock_innovation_scale"])))
        trigger = max(reference_shock, innovation_shock)
        self.shock_score = float(max(trigger, float(self.parameters["shock_decay"]) * self.shock_score))

        represented = np.asarray(cart_diag.residual_represented, dtype=float)
        disturbance_input = self.b_pinv @ represented
        desired_motion = reference_position - np.asarray(observation.task_state.tip_position_world, dtype=float)
        conflict = self._opposition(disturbance_input, desired_motion)
        beta = float(self.parameters["conflict_beta"])
        self.conflict_memory = float((1.0 - beta) * self.conflict_memory + beta * conflict)

        backbone = np.asarray(cart_diag.backbone_command, dtype=float)
        correction = np.asarray(cart_diag.qp_correction, dtype=float)
        cancellation = self._opposition(backbone, correction)
        cancellation *= float(np.clip(min(np.linalg.norm(backbone), np.linalg.norm(correction)) / float(self.parameters["cancellation_norm_scale"]), 0.0, 1.0))
        coordination_drive = self.shock_score * (
            float(self.parameters["shock_base_weight"])
            + float(self.parameters["conflict_gain"]) * self.conflict_memory
            + float(self.parameters["cancellation_gain"]) * cancellation
        )
        coordination_target = float(self.parameters["robust_blend_max"]) * float(np.clip(coordination_drive, 0.0, 1.0))
        engagement_target = 1.0 - coordination_target
        rate = float(self.parameters["offset_disengage_rate"] if engagement_target < self.offset_engagement else self.parameters["offset_engage_rate"])
        self.offset_engagement += float(np.clip(engagement_target - self.offset_engagement, -rate, rate))
        coordination_weight = 1.0 - self.offset_engagement

        robust_raw = -float(self.parameters["robust_gain_scale"]) * self.robust_gain @ state
        robust_command = np.clip(robust_raw, -2.0, 2.0)
        coordinated = self.offset_engagement * cart_command + coordination_weight * robust_command
        amplitude_limit = 2.0 - float(self.parameters["amplitude_reserve_fraction"]) * self.shock_score * 2.0
        amplitude_limit = float(np.clip(amplitude_limit, 1.0, 2.0))
        coordinated = np.clip(coordinated, -amplitude_limit, amplitude_limit)
        slew_limit = 0.25 * (1.0 - float(self.parameters["slew_reserve_fraction"]) * self.shock_score)
        slew_limit = float(np.clip(slew_limit, 0.125, 0.25))
        delta = np.clip(coordinated - previous_actual, -slew_limit, slew_limit)
        final = previous_actual + delta
        # Keep the inherited limiter state as the single authoritative actual command.
        self.limiter.previous = final.copy()
        self.cart.limiter.previous = final.copy()
        self.cart.estimator.accept(final)

        saturated = np.abs(coordinated) >= amplitude_limit - 1.0e-9
        slew_limited = np.abs(coordinated - previous_actual) > slew_limit + 1.0e-9
        mismatch = float(np.max(np.abs(final - coordinated)))
        self.diagnostics = SATCOFMPCDiagnostics(
            coordinated.copy(), np.clip(coordinated, -2.0, 2.0), final.copy(), saturated, slew_limited,
            float(np.linalg.norm(state)), np.asarray(cart_diag.residual_debt[:3], dtype=float).copy(),
            self.shock_score, reference_shock, innovation_shock, innovation_norm, innovation_rate,
            disturbance_input.copy(), desired_motion.copy(), self.conflict_memory, cancellation,
            self.offset_engagement, coordination_weight, 0.25 - slew_limit, 2.0 - amplitude_limit,
            float(cart_diag.trust), float(cart_diag.residual_clip_fraction), float(cart_diag.residual_unrepresented_norm),
            float(cart_diag.residual_debt_norm), float(cart_diag.steady_equality_residual), float(cart_diag.steady_task_residual),
            bool(cart_diag.steady_feasible), bool(cart_diag.steady_constraint_active), bool(cart_diag.reachable_constraint_active),
            np.asarray(self.cart.steady_solver.solve(self.cart.trust * represented, previous_actual).state, dtype=float),
            np.asarray(cart_diag.requested_steady_input, dtype=float).copy(), np.asarray(cart_diag.feasible_steady_input, dtype=float).copy(),
            np.asarray(cart_diag.feasible_steady_input, dtype=float).copy(), backbone.copy(), correction.copy(), correction.copy(), robust_command.copy(), final.copy(),
            bool(np.any(saturated) or np.any(slew_limited)), str(cart_diag.qp_status), int(cart_diag.qp_iterations),
            float(cart_diag.solve_time_ms), mismatch,
        )
        return final.copy()
