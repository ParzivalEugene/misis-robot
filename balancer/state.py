import math
from dataclasses import dataclass

import mujoco
import numpy as np

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


def read_state(data: mujoco.MjData, params: RobotParams) -> RobotState:
    pitch_rate, yaw_rate = float(data.sensor("gyro").data[1]), float(data.sensor("gyro").data[2])
    mean_wheel_velocity = (
        float(data.sensor("left_wheel_velocity").data[0])
        + float(data.sensor("right_wheel_velocity").data[0])
    ) / 2
    return RobotState(
        pitch=pitch_from_quaternion(data.sensor("orientation").data),
        pitch_rate=pitch_rate,
        yaw_rate=yaw_rate,
        forward_velocity=params.wheel.radius * (mean_wheel_velocity + pitch_rate),
    )
