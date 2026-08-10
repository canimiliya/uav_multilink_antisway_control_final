from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np

from uav_sway.native_stack.actuation import CanonicalWrenchActuator
from uav_sway.native_stack.api import WrenchCommand
from uav_sway.native_stack.references import ApproachStopReference, MinimumJerkReference, WaypointReference

ROOT = Path(__file__).resolve().parents[2]


def test_canonical_wrench_exact_limits_and_saturation():
    model = mujoco.MjModel.from_xml_path(str(ROOT / "reproducibility/frozen/model/model_5link_controlled.xml"))
    data = mujoco.MjData(model)
    actuator = CanonicalWrenchActuator(model)
    result = actuator.apply(data, WrenchCommand(300, np.array([30, -30, 20])), 4, 0.001)
    np.testing.assert_array_equal(result.actual.as_array(), [285.74568, 25, -25, 12])
    assert result.thrust_saturated and result.torque_saturated.all()
    assert result.application_time_s == 0.004


def test_minimum_jerk_boundary_conditions_and_determinism():
    generator = MinimumJerkReference(np.zeros(3), np.array([1, -2, 3]), 1.0, 2.0)
    before = generator.sample(0.5); after = generator.sample(4.0); middle = generator.sample(2.0)
    np.testing.assert_array_equal(before.position_world, np.zeros(3))
    np.testing.assert_array_equal(after.position_world, [1, -2, 3])
    np.testing.assert_array_equal(before.velocity_world, np.zeros(3))
    np.testing.assert_array_equal(after.acceleration_world, np.zeros(3))
    np.testing.assert_array_equal(middle.position_world, generator.sample(2.0).position_world)


def test_approach_stop_and_waypoints_are_smooth_at_stops():
    approach = ApproachStopReference(np.zeros(3), np.ones(3), 0.0, 2.0)
    np.testing.assert_allclose(approach.sample(2.0).velocity_world, 0.0)
    waypoint = WaypointReference(np.array([[0, 0, 0], [1, 0, 1], [1, 1, 1]]), np.array([0, 2, 5]))
    np.testing.assert_allclose(waypoint.sample(2.0).position_world, [1, 0, 1])
    np.testing.assert_allclose(waypoint.sample(2.0).velocity_world, 0.0)
