from dataclasses import dataclass
from typing import Protocol

import numpy as np

from balancer.state import RobotState


@dataclass(frozen=True)
class Command:
    forward_velocity: float = 0.0
    yaw_rate: float = 0.0


@dataclass(frozen=True)
class WheelTorques:
    left: float
    right: float


class Controller(Protocol):
    def reset(self) -> None: ...

    def compute(self, state: RobotState, command: Command, dt: float) -> WheelTorques: ...


@dataclass(frozen=True)
class PidGains:
    pitch_kp: float = 6.0
    pitch_kd: float = 0.4
    velocity_kp: float = 0.15
    velocity_ki: float = 0.05
    yaw_kp: float = 0.3
    max_pitch_reference: float = 0.2
    max_velocity_integral: float = 2.0


class PidController:
    def __init__(self, gains: PidGains | None = None) -> None:
        self.gains = gains or PidGains()
        self.velocity_integral = 0.0

    def reset(self) -> None:
        self.velocity_integral = 0.0

    def compute(self, state: RobotState, command: Command, dt: float) -> WheelTorques:
        pitch_reference = self._pitch_reference(state, command, dt)
        balance = self.gains.pitch_kp * (state.pitch - pitch_reference)
        balance += self.gains.pitch_kd * state.pitch_rate
        turn = self.gains.yaw_kp * (command.yaw_rate - state.yaw_rate)
        return WheelTorques(left=balance - turn, right=balance + turn)

    def _pitch_reference(self, state: RobotState, command: Command, dt: float) -> float:
        velocity_error = command.forward_velocity - state.forward_velocity
        limit = self.gains.max_velocity_integral
        self.velocity_integral = float(
            np.clip(self.velocity_integral + velocity_error * dt, -limit, limit)
        )
        reference = (
            self.gains.velocity_kp * velocity_error
            + self.gains.velocity_ki * self.velocity_integral
        )
        return float(
            np.clip(reference, -self.gains.max_pitch_reference, self.gains.max_pitch_reference)
        )
