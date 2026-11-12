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
    lean: float
    lean_rate: float
    motion: float


def body_up_axis(quaternion: np.ndarray) -> np.ndarray:
    w, x, y, z = quaternion
    return np.array([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)])


def pitch_from_quaternion(quaternion: np.ndarray, heading: np.ndarray) -> float:
    up = body_up_axis(quaternion)
    return math.atan2(float(up @ heading), float(up[2]))


def _sensor(data: mujoco.MjData, name: str) -> float:
    return float(data.sensor(name).data[0])


def _wheel_ground_speed(data: mujoco.MjData, pitch_rate: float, side_index: int) -> float:
    side = SIDES[side_index]
    shin_pitch_rate = (
        pitch_rate + _sensor(data, f"{side.hip}_velocity") + _sensor(data, f"{side.knee}_velocity")
    )
    return _sensor(data, f"{side.wheel}_velocity") + shin_pitch_rate


def _vector(data: mujoco.MjData, name: str) -> np.ndarray:
    return np.asarray(data.sensor(name).data, dtype=float)


def _heading(data: mujoco.MjData) -> np.ndarray:
    left, right = SIDES
    axle = _vector(data, f"{left.wheel}_position") - _vector(data, f"{right.wheel}_position")
    heading = np.cross(axle, np.array([0.0, 0.0, 1.0]))
    return heading / max(float(np.linalg.norm(heading)), 1e-9)


def _midpoint(data: mujoco.MjData, suffix: str) -> np.ndarray:
    left, right = SIDES
    return (_vector(data, f"{left.name}{suffix}") + _vector(data, f"{right.name}{suffix}")) / 2


def com_relative_to_hip(data: mujoco.MjData) -> tuple[float, float]:
    hip = _midpoint(data, "_hip_position")
    return float((_vector(data, "com") - hip) @ _heading(data)), float(hip[2])


def _lean(data: mujoco.MjData) -> tuple[float, float]:
    left, right = SIDES
    heading = _heading(data)
    wheel_position = (
        _vector(data, f"{left.wheel}_position") + _vector(data, f"{right.wheel}_position")
    ) / 2
    wheel_velocity = (
        _vector(data, f"{left.wheel}_linear_velocity")
        + _vector(data, f"{right.wheel}_linear_velocity")
    ) / 2
    offset = _vector(data, "com") - wheel_position
    relative_velocity = _vector(data, "com_velocity") - wheel_velocity
    dx, dz = float(offset @ heading), float(offset[2])
    vx, vz = float(relative_velocity @ heading), float(relative_velocity[2])
    return math.atan2(dx, dz), (dz * vx - dx * vz) / max(dx * dx + dz * dz, 1e-9)


def read_state(data: mujoco.MjData, params: RobotParams) -> RobotState:
    gyro = data.sensor("gyro").data
    pitch_rate, yaw_rate = float(gyro[1]), float(gyro[2])
    mean_speed = sum(_wheel_ground_speed(data, pitch_rate, index) for index in (0, 1)) / 2
    lean, lean_rate = _lean(data)
    return RobotState(
        pitch=pitch_from_quaternion(data.sensor("orientation").data, _heading(data)),
        pitch_rate=pitch_rate,
        yaw_rate=yaw_rate,
        forward_velocity=params.wheel.radius * mean_speed,
        lean=lean,
        lean_rate=lean_rate,
        motion=float(np.linalg.norm(gyro)) + float(np.linalg.norm(_vector(data, "com_velocity"))),
    )
