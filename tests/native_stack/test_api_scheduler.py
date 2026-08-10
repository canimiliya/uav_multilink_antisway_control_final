from __future__ import annotations

from dataclasses import FrozenInstanceError

import numpy as np
import pytest

from uav_sway.native_stack.api import DiagnosticTruthPacket, ReferenceSample, SensorPacket, WrenchCommand
from uav_sway.native_stack.controller import NativeStackController
from uav_sway.native_stack.scheduler import DeterministicMultiRateScheduler, SUPPORTED_RATES_HZ


class Controller(NativeStackController):
    def reset(self): pass
    def physical_command(self): return WrenchCommand(0, np.zeros(3))


def test_sensor_envelope_has_no_truth_or_future_fields():
    names = set(SensorPacket.field_names())
    assert not names & {"wind", "true_wind", "future_reference", "future_state", "external_force", "holdout_metadata"}


def test_truth_packet_cannot_be_owned_by_runtime_controller():
    truth = DiagnosticTruthPacket(0.0, np.zeros(3), np.zeros(3), np.zeros(3))
    controller = Controller()
    with pytest.raises(TypeError, match="cannot own"):
        controller.truth = truth


def test_packets_and_arrays_are_immutable():
    command = WrenchCommand(1, np.zeros(3))
    with pytest.raises(FrozenInstanceError):
        command.thrust_N = 2
    with pytest.raises(ValueError):
        command.torque_Nm[0] = 1


def test_scheduler_exact_rates_and_order():
    scheduler = DeterministicMultiRateScheduler()
    for rate in SUPPORTED_RATES_HZ:
        scheduler.register(str(rate), rate)
    assert scheduler.due_components(0) == tuple(str(rate) for rate in SUPPORTED_RATES_HZ)
    assert scheduler.due_components(1) == ("1000",)
    assert scheduler.due_components(5) == ("200", "1000")
    for rate in SUPPORTED_RATES_HZ:
        stride = 1000 // rate
        assert scheduler.due(str(rate), 1000 - stride)
        assert not scheduler.due(str(rate), 1000 - stride + 1) if stride > 1 else True


def test_scheduler_rejects_noninteger_or_unfrozen_rate():
    scheduler = DeterministicMultiRateScheduler()
    with pytest.raises(ValueError): scheduler.register("bad", 333)
    with pytest.raises(ValueError): DeterministicMultiRateScheduler(2000)
