from dataclasses import replace

import numpy as np
import pytest

from balancer.control import PidController
from balancer.model import load_model
from balancer.scenarios import CROUCH, DRIVE, KNOCKDOWN, PUSH, STAND, Push, run_scenario
from balancer.sim import Mode


def test_model_loads() -> None:
    model = load_model()
    assert model.nu == 8


def test_stands_for_thirty_seconds() -> None:
    result = run_scenario(STAND, PidController())
    assert not result.fell
    assert result.samples[-1].time >= 30.0
    assert abs(result.samples[-1].forward_velocity) < 0.02


def test_recovers_from_push() -> None:
    result = run_scenario(PUSH, PidController())
    assert not result.fell
    assert abs(result.samples[-1].forward_velocity) < 0.05


def test_drives_at_commanded_speed() -> None:
    result = run_scenario(DRIVE, PidController())
    cruise = [sample for sample in result.samples if 6.0 <= sample.time < 10.0]
    velocities = np.array([sample.forward_velocity for sample in cruise])
    target = cruise[0].command_velocity
    assert not result.fell
    assert abs(velocities.mean() - target) < 0.1 * target
    assert result.samples[-1].position_x - result.samples[0].position_x > 3.0


def test_crouches_and_stands_up() -> None:
    result = run_scenario(CROUCH, PidController())
    heights = [sample.height for sample in result.samples]
    assert not result.fell
    assert min(heights) < 0.115
    assert max(heights) > 0.215


def test_fall_disables_wheels() -> None:
    result = run_scenario(KNOCKDOWN, PidController())
    lying = [sample for sample in result.samples if sample.mode is Mode.FALLEN]
    assert lying
    assert all(sample.left_torque == 0.0 and sample.right_torque == 0.0 for sample in lying)


@pytest.mark.parametrize("force", [50.0, 60.0, 80.0, 100.0, 120.0, -50.0, -120.0])
def test_gets_up_after_fall(force: float) -> None:
    scenario = replace(KNOCKDOWN, duration=16.0, pushes=(Push(time=1.0, force=force),))
    result = run_scenario(scenario, PidController())
    assert result.fell
    assert all(sample.mode is Mode.BALANCING for sample in result.samples[-1500:])


def test_gets_up_after_repeated_falls() -> None:
    pushes = (Push(1.0, 80.0), Push(8.0, -120.0), Push(15.0, 60.0), Push(22.0, 150.0))
    scenario = replace(KNOCKDOWN, duration=30.0, pushes=pushes)
    result = run_scenario(scenario, PidController())
    assert all(sample.mode is Mode.BALANCING for sample in result.samples[-1500:])
