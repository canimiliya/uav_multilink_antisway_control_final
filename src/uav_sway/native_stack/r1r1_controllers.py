"""Native physical-wrench controllers for the frozen P2-R1R1 study."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
from pathlib import Path
from typing import Any

import numpy as np
from scipy.linalg import solve_continuous_are
from udaan.control.quadrotor import GeometricAttitudeController
from udaan.manif import SO3, TSO3

from uav_sway.control.geometric_inner_loop import GeometricInnerLoop
from uav_sway.task_space.state import CutterTaskState
from uav_sway.v3.controllers import V3CascadedTaskPID, V3FullStateLQR, V3TaskWeightedLQR
from uav_sway.v3.metrics import load_r0_linear_matrices
from uav_sway.v3.observation import V3Observation, V3Reference
from uav_sway.v5.satc_ofmpc import SATCOFMPC
from .api import SensorPacket, WrenchCommand
from .controller import AccelerationOuterStackAdapter, NativeStackController

MASS_KG = 13.24
INERTIA = np.array([0.655826666667, 0.966532666667, 1.24834333333])
GRAVITY = 9.81
UAV_MINUS_TIP_TRIM = np.array([-0.225, 0.0, 2.81])


@dataclass(frozen=True)
class NativeGains:
    kp: tuple[float, float, float]
    kd: tuple[float, float, float]
    ki: tuple[float, float, float] = (0.0, 0.0, 0.0)
    attitude_wn: float = 4.0
    attitude_zeta: float = 0.95
    integral_limit: float = 0.8
    acceleration_limit: float = 5.0
    acceleration_slew_per_outer_update: float = 0.10
    swing_angle_gain: float = 0.0
    swing_rate_gain: float = 0.0
    jerk_feedforward: float = 0.0
    transient_coordination: float = 0.0
    constraint_margin: float = 0.85
    tip_correction_kp: float = 0.05
    tip_correction_kd: float = 0.02
    tip_correction_limit_m: float = 0.15


def _vee(matrix: np.ndarray) -> np.ndarray:
    return np.array([matrix[2, 1], matrix[0, 2], matrix[1, 0]])


def _desired_rotation(force_world: np.ndarray) -> np.ndarray:
    z = force_world / max(float(np.linalg.norm(force_world)), 1.0e-9)
    heading = np.array([1.0, 0.0, 0.0]); y = np.cross(z, heading)
    if np.linalg.norm(y) < 1.0e-6:
        heading = np.array([0.0, 1.0, 0.0]); y = np.cross(z, heading)
    y /= np.linalg.norm(y); x = np.cross(y, z)
    return np.column_stack((x, y, z))


class NativeWrenchController(NativeStackController):
    """Causal task-space outer loop plus native SO(3) physical-wrench layer."""

    architecture = "cascaded_task_feedback_to_SO3_physical_wrench"

    def __init__(self, gains: NativeGains, method_id: str, outer_dt: float = 0.01) -> None:
        self.gains = gains; self.method_id = method_id; self.outer_dt = float(outer_dt)
        self._sensor_packet: SensorPacket | None = None
        self._integral = np.zeros(3); self._desired_force = np.array([0.0, 0.0, MASS_KG * GRAVITY])
        self._acceleration = np.zeros(3)
        self._wrench = WrenchCommand(MASS_KG * GRAVITY, np.zeros(3))
        self._attitude = GeometricAttitudeController(inertia=np.diag(INERTIA))
        self._attitude._gains.kp = INERTIA * gains.attitude_wn**2
        self._attitude._gains.kd = 2.0 * gains.attitude_zeta * INERTIA * gains.attitude_wn

    def reset(self) -> None:
        self._sensor_packet = None; self._integral[:] = 0.0
        self._acceleration[:] = 0.0
        self._desired_force = np.array([0.0, 0.0, MASS_KG * GRAVITY])
        self._wrench = WrenchCommand(MASS_KG * GRAVITY, np.zeros(3))

    def _task_acceleration(self, packet: SensorPacket) -> np.ndarray:
        tip_error = packet.reference.position_world - packet.cutter_tip_position_world
        relative_tip_velocity = packet.cutter_tip_velocity_world - packet.uav_velocity_world
        correction = np.clip(
            self.gains.tip_correction_kp * tip_error - self.gains.tip_correction_kd * relative_tip_velocity,
            -self.gains.tip_correction_limit_m, self.gains.tip_correction_limit_m,
        )
        uav_target = packet.reference.position_world + UAV_MINUS_TIP_TRIM + correction
        error = uav_target - packet.uav_position_world
        velocity_error = packet.reference.velocity_world - packet.uav_velocity_world
        proposed = np.clip(self._integral + error * self.outer_dt, -self.gains.integral_limit, self.gains.integral_limit)
        acceleration = (packet.reference.acceleration_world + np.asarray(self.gains.kp) * error
                        + np.asarray(self.gains.kd) * velocity_error + np.asarray(self.gains.ki) * proposed)
        swing = float(np.sum(packet.joint_position)); swing_rate = float(np.sum(packet.joint_velocity))
        acceleration[0] -= self.gains.swing_angle_gain * swing + self.gains.swing_rate_gain * swing_rate
        acceleration += self.gains.jerk_feedforward * packet.reference.jerk_world
        norm = float(np.linalg.norm(acceleration))
        constrained = norm > self.gains.acceleration_limit
        if not constrained:
            self._integral = proposed
        elif norm > 0.0:
            acceleration *= self.gains.acceleration_limit / norm
        return acceleration

    def update_high_level(self) -> None:
        if self._sensor_packet is None: raise RuntimeError("observe must precede update")
        acceleration = self._task_acceleration(self._sensor_packet)
        delta = np.clip(acceleration - self._acceleration, -self.gains.acceleration_slew_per_outer_update, self.gains.acceleration_slew_per_outer_update)
        self._acceleration = self._acceleration + delta
        self._desired_force = MASS_KG * (self._acceleration + np.array([0.0, 0.0, GRAVITY]))

    def update_inner(self) -> None:
        if self._sensor_packet is None: raise RuntimeError("observe must precede update")
        packet = self._sensor_packet
        thrust, torque = self._attitude.compute(
            0.0, (SO3(packet.rotation_world_from_body), TSO3(packet.body_angular_velocity)), self._desired_force,
        )
        self._wrench = WrenchCommand(thrust, torque)

    def physical_command(self) -> WrenchCommand:
        return self._wrench

    def diagnostics(self) -> dict[str, Any]:
        return {"method_id": self.method_id, "architecture": self.architecture, "gains": asdict(self.gains), "integral": self._integral.copy()}


class NativePID(NativeWrenchController):
    architecture = "native_cascaded_PID_PD_SO3_wrench"


class _LegacyReset:
    def reset(self) -> None:
        return None


class LegacyTaskLevelAdapter(AccelerationOuterStackAdapter):
    """Frozen acceleration-output incumbent routed through the audited adapter."""
    architecture = "frozen_legacy_acceleration_plus_LegacyTaskLevelAdapter"

    def __init__(self, method_id: str, kp: float, kd: float, tip_kp: float = .10, tip_kd: float = .05, historical_id: str | None = None) -> None:
        super().__init__(_LegacyReset(), GeometricInnerLoop(MASS_KG, INERTIA, 4.0, .9), np.array([.225, 0., -2.81]))
        self.method_id = method_id; self.kp = float(kp); self.kd = float(kd)
        self.tip_kp = float(tip_kp); self.tip_kd = float(tip_kd); self._previous = np.zeros(3)
        self.historical_id = historical_id; self._historical = self._load_historical(historical_id)

    @staticmethod
    def _load_historical(historical_id: str | None):
        root = Path(__file__).resolve().parents[3]
        if historical_id == "corrected_pid":
            p = json.loads((root/"reproducibility/v3/r1r1/pid_freeze.json").read_text(encoding="utf-8"))["parameters"]
            return V3CascadedTaskPID(np.array(p["uav_kp"]),np.array(p["uav_kd"]),np.array(p["uav_ki"]),np.array(p["tip_kp"]),np.array(p["tip_kd"]),np.array(p["correction_limit_m"]),p["correction_slew_m_per_update"],p["integral_limit"],p["tip_velocity_mode"])
        if historical_id in {"full_lqr_048","task_lqr_009"}:
            name="full_lqr" if historical_id.startswith("full") else "task_lqr"; p=json.loads((root/f"reproducibility/v3/r1/{name}_freeze.json").read_text(encoding="utf-8"))["parameters"]
            cls=V3FullStateLQR if name=="full_lqr" else V3TaskWeightedLQR; return cls(np.array(p["K"]))
        if historical_id == "satc_b_027":
            a,b=load_r0_linear_matrices(root); metric=json.loads((root/"reproducibility/v3/r1/task_metric_alignment_audit.json").read_text(encoding="utf-8")); c=np.vstack([metric[x] for x in ("C_pos","C_vel","C_dir","C_omega_perp")])
            task=np.array(json.loads((root/"reproducibility/v3/r1/task_lqr_freeze.json").read_text(encoding="utf-8"))["parameters"]["K"]); full=np.array(json.loads((root/"reproducibility/v3/r1/full_lqr_freeze.json").read_text(encoding="utf-8"))["parameters"]["K"]); params=json.loads((root/"reproducibility/v5/self/self_freeze.json").read_text(encoding="utf-8"))["parameters"]
            return SATCOFMPC(a,b,c,task,full,params)
        return None

    def reset(self) -> None:
        super().reset(); self._previous[:] = 0.0
        if self._historical is not None: self._historical.reset()

    def _historical_observation(self, p: SensorPacket) -> tuple[V3Observation,V3Reference]:
        target=p.reference.position_world+UAV_MINUS_TIP_TRIM; r=p.rotation_world_from_body
        state=np.zeros(20); state[0]=p.uav_position_world[0]-target[0]; state[1]=p.uav_velocity_world[0]-p.reference.velocity_world[0]
        state[2]=p.uav_position_world[1]-target[1]; state[3]=p.uav_velocity_world[1]-p.reference.velocity_world[1]
        state[4]=p.uav_position_world[2]-target[2]; state[5]=p.uav_velocity_world[2]-p.reference.velocity_world[2]
        state[6]=np.arctan2(r[2,1],r[2,2]); state[7]=p.body_angular_velocity[0]; state[8]=np.arcsin(np.clip(-r[2,0],-1.,1.)); state[9]=p.body_angular_velocity[1]
        state[10:15]=p.joint_position; state[15:20]=p.joint_velocity
        angle=float(np.sum(p.joint_position)); ca,sa=np.cos(angle),np.sin(angle); ry=np.array([[ca,0,sa],[0,1,0],[-sa,0,ca]]); cutter_rotation=r@ry
        task=CutterTaskState(p.cutter_tip_position_world,p.cutter_tip_velocity_world,r@(p.body_angular_velocity+np.array([0.,float(np.sum(p.joint_velocity)),0.])),cutter_rotation@np.array([1.,0.,0.]),cutter_rotation)
        obs=V3Observation(state,p.uav_position_world,p.uav_velocity_world,task); ref=V3Reference(target,p.reference.velocity_world,p.reference.position_world,p.time_s)
        return obs,ref

    def update_high_level(self) -> None:
        if self._sensor_packet is None: raise RuntimeError("observe must precede update")
        p = self._sensor_packet
        if self._historical is not None:
            obs,ref=self._historical_observation(p); self.set_legacy_acceleration(self._historical.command(obs,ref,.05)); return
        tip_error = p.reference.position_world - p.cutter_tip_position_world
        relative_velocity = p.cutter_tip_velocity_world - p.uav_velocity_world
        correction = np.clip(self.tip_kp*tip_error - self.tip_kd*relative_velocity, -.15, .15)
        target = p.reference.position_world + UAV_MINUS_TIP_TRIM + correction
        raw = p.reference.acceleration_world + self.kp*(target-p.uav_position_world) + self.kd*(p.reference.velocity_world-p.uav_velocity_world)
        raw = np.clip(raw, -2., 2.); self._previous += np.clip(raw-self._previous, -.25, .25)
        self.set_legacy_acceleration(self._previous)

    def diagnostics(self) -> dict[str, Any]:
        return {"method_id": self.method_id, "architecture": self.architecture, "historical_id":self.historical_id,"kp": self.kp, "kd": self.kd}


def physical_hover_model() -> tuple[np.ndarray, np.ndarray]:
    """Continuous hover model with true [delta-T, tau-x, tau-y, tau-z] input."""
    a = np.zeros((12, 12)); b = np.zeros((12, 4))
    a[0:3, 3:6] = np.eye(3); a[6:9, 9:12] = np.eye(3)
    a[3, 7] = GRAVITY; a[4, 6] = -GRAVITY
    b[5, 0] = 1.0 / MASS_KG; b[9:12, 1:4] = np.diag(1.0 / INERTIA)
    return a, b


def lqr_audit(q_position: float, q_velocity: float, q_attitude: float, q_rate: float, r: float) -> dict[str, Any]:
    a, b = physical_hover_model()
    q = np.diag([q_position]*3 + [q_velocity]*3 + [q_attitude]*3 + [q_rate]*3)
    rr = np.diag([r]*4); p = solve_continuous_are(a, b, q, rr); k = np.linalg.solve(rr, b.T @ p)
    ctrb = np.hstack([np.linalg.matrix_power(a, i) @ b for i in range(12)])
    poles = np.linalg.eigvals(a - b @ k)
    return {"state_schema": ["position_xyz", "velocity_xyz", "roll_pitch_yaw", "body_rate_xyz"], "state_dimension": 12,
        "physical_input_schema": ["delta_thrust_N", "tau_x_Nm", "tau_y_Nm", "tau_z_Nm"], "physical_input_dimension": 4,
        "linearization_point": "level hover at trim wrench [mass*g,0,0,0]", "A": a.tolist(), "B": b.tolist(), "K": k.tolist(),
        "rank_B": int(np.linalg.matrix_rank(b)), "controllability_rank": int(np.linalg.matrix_rank(ctrb)), "controllable": bool(np.linalg.matrix_rank(ctrb) == 12),
        "stabilizable": bool(np.all(np.real(poles) < 0.0)), "closed_loop_poles": [[float(x.real), float(x.imag)] for x in poles]}


class NativeFullLQR(NativeWrenchController):
    architecture = "12_state_hover_LQR_physical_wrench_with_swing_damping"
    def __init__(self, gains: NativeGains, method_id: str, q: tuple[float, float, float, float, float]) -> None:
        super().__init__(gains, method_id); self.audit = lqr_audit(*q); self._gain = np.asarray(self.audit["K"])
    def _position_error(self, packet: SensorPacket) -> np.ndarray:
        target = packet.reference.position_world + UAV_MINUS_TIP_TRIM
        return packet.uav_position_world - target
    def update_high_level(self) -> None:
        if self._sensor_packet is None: raise RuntimeError("observe must precede update")
        p = self._sensor_packet; r = p.rotation_world_from_body
        angles = np.array([np.arctan2(r[2,1],r[2,2]), np.arcsin(np.clip(-r[2,0],-1.,1.)), np.arctan2(r[1,0],r[0,0])])
        state = np.r_[self._position_error(p), p.uav_velocity_world-p.reference.velocity_world, angles, p.body_angular_velocity]
        delta = -self._gain @ state
        delta[0] += MASS_KG * p.reference.acceleration_world[2]
        delta[2] -= self.gains.swing_angle_gain*float(np.sum(p.joint_position)) + self.gains.swing_rate_gain*float(np.sum(p.joint_velocity))
        self._wrench = WrenchCommand(MASS_KG*GRAVITY + float(delta[0]), delta[1:])
    def update_inner(self) -> None:
        return None
    def diagnostics(self) -> dict[str, Any]:
        return {**super().diagnostics(), "linear_model_audit": self.audit}


class NativeTaskLQR(NativeFullLQR):
    architecture = "output_weighted_tip_LQT_physical_wrench_with_joint_stabilization"
    def _position_error(self, packet: SensorPacket) -> np.ndarray:
        return packet.cutter_tip_position_world - packet.reference.position_world


class SATCNative(NativeWrenchController):
    architecture = "shock_aware_constraint_predictive_disturbance_task_coordination_native_wrench"
    def _task_acceleration(self, packet: SensorPacket) -> np.ndarray:
        acceleration = super()._task_acceleration(packet)
        conflict = packet.reference.velocity_world - packet.cutter_tip_velocity_world
        shock = np.tanh(np.linalg.norm(packet.reference.jerk_world) + 0.5*np.linalg.norm(conflict))
        acceleration += self.gains.transient_coordination * shock * conflict
        limit = self.gains.acceleration_limit * self.gains.constraint_margin
        norm = float(np.linalg.norm(acceleration))
        if norm > limit: acceleration *= limit / norm
        return acceleration


def candidate_registries() -> dict[str, list[dict[str, Any]]]:
    pid = []
    pid_values = [(.10,.45,.02,.00),(.12,.55,.03,.01),(.15,.60,.04,.02),(.18,.70,.05,.03),
        (.20,.80,.06,.04),(.22,.90,.08,.05),(.25,.75,.06,.04),(.28,.85,.08,.05),
        (.30,.95,.10,.06),(.32,1.05,.10,.07),(.35,1.10,.12,.08),(.16,.90,.08,.06)]
    for i, (kp, kd, ki, swing) in enumerate(pid_values):
        pid.append({"method_id": f"native_pid_{i:03d}", "gains": NativeGains((kp,)*3,(kd,)*3,(ki,)*3, swing_angle_gain=swing, swing_rate_gain=.04+swing)})
    full_values = [(.12,.55,.01,.03,2,20,50),(.15,.65,.02,.04,3,30,50),(.18,.75,.03,.05,3,40,75),(.20,.85,.04,.06,4,40,75),(.22,.95,.05,.07,4,50,100),(.25,1.05,.06,.08,5,60,100),(.28,.90,.05,.09,3,30,50),(.32,1.10,.07,.10,5,70,125)]
    full = [{"method_id": f"native_full_lqr_{i:03d}", "gains": NativeGains((kp,)*3,(kd,)*3, swing_angle_gain=s, swing_rate_gain=s), "q": (qp,qv,qa,8.0,r)} for i,(kp,kd,s,qp,qv,qa,r) in enumerate(full_values)]
    task_values = [(.14,.60,.03,.01,.005,2,20,50),(.17,.70,.04,.02,.01,3,30,50),(.20,.80,.05,.03,.02,3,30,50),(.23,.90,.06,.04,.03,4,40,75),(.26,1.00,.08,.05,.04,4,50,75),(.30,1.10,.10,.06,.05,5,60,100),(.32,.95,.08,.05,.06,3,40,75),(.36,1.15,.12,.07,.08,5,70,125)]
    task = [{"method_id": f"native_task_lqr_{i:03d}", "gains": NativeGains((kp,)*3,(kd,)*3,(ki,)*3, swing_angle_gain=s, swing_rate_gain=s), "q": (qp,qv,qa,8.0,r)} for i,(kp,kd,ki,s,qp,qv,qa,r) in enumerate(task_values)]
    satc = []
    for i in range(16):
        kp = .18 + .035*(i%4); kd = .65 + .10*((i//4)%2); coord = .04 + .03*(i%4); margin = .78 + .04*(i//4)
        satc.append({"method_id": f"satc_native_{i:03d}", "gains": NativeGains((kp,)*3,(kd,)*3,(.06,)*3, attitude_wn=4.0, swing_angle_gain=.03, swing_rate_gain=.04, jerk_feedforward=.005, transient_coordination=coord, constraint_margin=min(margin,.90))})
    return {"native_pid": pid, "native_full_lqr": full, "native_task_lqr": task, "satc_native": satc}
