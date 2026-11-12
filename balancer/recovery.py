import math
from dataclasses import dataclass

from balancer.legs import LegAngles, leg_angles_toward
from balancer.params import RobotParams


@dataclass(frozen=True)
class GetUpParams:
    settle_time: float = 1.0
    settled_motion: float = 0.2
    reach_speed: float = 3.0
    push_speed: float = 1.0
    push_end: float = 0.25
    max_wheel_shift: float = 0.1
    min_tuck_depth: float = 0.03
    max_tuck_depth: float = 0.06
    leg_speed: float = 3.0
    wheel_brake: float = 0.01
    handover_lean: float = 0.25
    min_get_up_time: float = 0.2
    stand_speed: float = 2.0
    arm_retract_speed: float = 2.0
    stance_height: float = 0.14
    timeout: float = 8.0


@dataclass(frozen=True)
class GetUpInputs:
    pitch: float
    lean: float
    com_ahead_of_hip: float
    hip_height: float


@dataclass(frozen=True)
class GetUpOutputs:
    legs: LegAngles
    shoulder: float
    wheel_brake: float


def approach(current: float, target: float, max_step: float) -> float:
    return current + max(-max_step, min(max_step, target - current))


class PushUp:
    def __init__(
        self, fell_forward: bool, legs: LegAngles, shoulder: float, params: GetUpParams
    ) -> None:
        self.turn = 1 if fell_forward else -1
        self.params = params
        self.legs = legs
        self.shoulder = shoulder
        self.reached_floor = False

    @property
    def _reach_angle(self) -> float:
        return -self.turn * math.pi / 2

    @property
    def _push_end(self) -> float:
        return -self.turn * self.params.push_end

    def step(self, inputs: GetUpInputs, robot: RobotParams, dt: float) -> GetUpOutputs:
        self._move_arm(dt)
        if not self.reached_floor:
            return GetUpOutputs(self.legs, self.shoulder, self.params.wheel_brake)
        target = self._legs_under_com(inputs, robot)
        max_step = self.params.leg_speed * dt
        self.legs = LegAngles(
            hip=approach(self.legs.hip, target.hip, max_step),
            knee=approach(self.legs.knee, target.knee, max_step),
        )
        return GetUpOutputs(self.legs, self.shoulder, self.params.wheel_brake)

    def _move_arm(self, dt: float) -> None:
        if not self.reached_floor:
            self.shoulder = approach(self.shoulder, self._reach_angle, self.params.reach_speed * dt)
            self.reached_floor = self.shoulder == self._reach_angle
        else:
            self.shoulder = approach(self.shoulder, self._push_end, self.params.push_speed * dt)

    def _legs_under_com(self, inputs: GetUpInputs, robot: RobotParams) -> LegAngles:
        limit = self.params.max_wheel_shift
        world_x = max(-limit, min(limit, inputs.com_ahead_of_hip))
        depth = inputs.hip_height - robot.wheel.radius
        world_z = -max(self.params.min_tuck_depth, min(self.params.max_tuck_depth, depth))
        cos_pitch, sin_pitch = math.cos(inputs.pitch), math.sin(inputs.pitch)
        body_x = world_x * cos_pitch - world_z * sin_pitch
        body_z = world_x * sin_pitch + world_z * cos_pitch
        return leg_angles_toward(body_x, body_z, robot.leg, near_hip=self.legs.hip)


@dataclass(frozen=True)
class Waypoint:
    angles: LegAngles
    speed: float


class GetUpTrajectory:
    def __init__(self, start: LegAngles, goal: LegAngles, speed: float) -> None:
        self.target = start
        self.goal = goal
        self.speed = speed

    @property
    def done(self) -> bool:
        return self.target == self.goal

    def advance(self, dt: float) -> LegAngles:
        hip_error = self.goal.hip - self.target.hip
        knee_error = self.goal.knee - self.target.knee
        distance = max(abs(hip_error), abs(knee_error))
        step = self.speed * dt
        if distance <= step:
            self.target = self.goal
        else:
            ratio = step / distance
            self.target = LegAngles(
                hip=self.target.hip + hip_error * ratio,
                knee=self.target.knee + knee_error * ratio,
            )
        return self.target
