from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np

from uav_sway.native_stack.api import WrenchCommand
from uav_sway.native_stack.controller import NativeStackController
from uav_sway.native_stack.references import MinimumJerkReference
from uav_sway.native_stack.runner import NativeStackRunner

ROOT = Path(__file__).resolve().parents[2]
MODEL = ROOT / "reproducibility/frozen/model/model_5link_controlled.xml"


class HoverController(NativeStackController):
    def __init__(self, thrust): self.thrust = thrust
    def reset(self): self.outer = self.inner = 0
    def update_high_level(self): self.outer += 1
    def update_inner(self): self.inner += 1
    def physical_command(self): return WrenchCommand(self.thrust, np.zeros(3))


def test_native_runner_multirate_zoh_and_logging():
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    controller = HoverController(float(np.sum(model.body_mass) * 9.81))
    reference = MinimumJerkReference(np.array([0.225, 0, 0.39]), np.array([0.225, 0, 0.39]), 0, 1)
    result = NativeStackRunner(MODEL).run(controller, reference, 0.02, 50, 200)
    assert result.ticks == 20
    assert result.outer_update_ticks == (0,)
    assert result.inner_update_ticks == (0, 5, 10, 15)
    assert len(result.command_records) == 20
    assert all(record.actual == result.command_records[0].actual for record in result.command_records)
    assert result.safe
