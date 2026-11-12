import math
from dataclasses import dataclass

import mujoco
import numpy as np

from balancer.model import SIDES
from balancer.params import RobotParams


@dataclass(frozen=True)
class RobotState:
    pitch: float
    pitch_rate: float
    yaw_rate: float
    forward_velocity: float


def pitch_from_quaternion(quaternion: np.ndarray) -> float:
    w, x, y, z = quaternion
    return math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))


def _sensor(data: mujoco.MjData, name: str) -> float:
    return float(data.sensor(name).data[0])


def _wheel_ground_speed(data: mujoco.MjData, pitch_rate: float, side_index: int) -> float:
    side = SIDES[side_index]
    shin_pitch_rate = (
        pitch_rate + _sensor(data, f"{side.hip}_velocity") + _sensor(data, f"{side.knee}_velocity")
    )
    return _sensor(data, f"{side.wheel}_velocity") + shin_pitch_rate


def read_state(data: mujoco.MjData, params: RobotParams) -> RobotState:
    gyro = data.sensor("gyro").data
    pitch_rate, yaw_rate = float(gyro[1]), float(gyro[2])
    mean_speed = sum(_wheel_ground_speed(data, pitch_rate, index) for index in (0, 1)) / 2
    return RobotState(
        pitch=pitch_from_quaternion(data.sensor("orientation").data),
        pitch_rate=pitch_rate,
        yaw_rate=yaw_rate,
        forward_velocity=params.wheel.radius * mean_speed,
    )
