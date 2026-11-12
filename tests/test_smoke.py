import numpy as np

from balancer.control import PidController
from balancer.model import load_model
from balancer.scenarios import DRIVE, PUSH, STAND, run_scenario


def test_model_loads() -> None:
    model = load_model()
    assert model.nu == 2


def test_stands_for_thirty_seconds() -> None:
    result = run_scenario(STAND, PidController())
    assert not result.fell
    assert result.samples[-1].time >= 30.0
    assert abs(result.samples[-1].pitch) < 0.02


def test_recovers_from_push() -> None:
    result = run_scenario(PUSH, PidController())
    assert not result.fell
    assert abs(result.samples[-1].pitch) < 0.02
    assert abs(result.samples[-1].forward_velocity) < 0.05


def test_drives_at_commanded_speed() -> None:
    result = run_scenario(DRIVE, PidController())
    cruise = [sample for sample in result.samples if 6.0 <= sample.time < 10.0]
    velocities = np.array([sample.forward_velocity for sample in cruise])
    target = cruise[0].command_velocity
    assert not result.fell
    assert abs(velocities.mean() - target) < 0.1 * target
    assert result.samples[-1].position_x > 3.0
