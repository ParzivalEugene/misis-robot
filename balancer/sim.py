import math
from dataclasses import dataclass

import mujoco

from balancer.control import Command, Controller, WheelTorques
from balancer.eyes import RED, EyeThresholds, apply_eye_color, eye_color
from balancer.legs import clamp_height, leg_angles
from balancer.model import CHASSIS, SIDES, Side, load_model
from balancer.params import RobotParams, SimParams
from balancer.state import RobotState, read_state

FALLEN_PITCH = 0.6


@dataclass(frozen=True)
class Sample:
    time: float
    pitch: float
    forward_velocity: float
    command_velocity: float
    height: float
    left_torque: float
    right_torque: float
    position_x: float
    fallen: bool


class Simulation:
    def __init__(
        self,
        controller: Controller,
        params: RobotParams | None = None,
        sim: SimParams | None = None,
        initial_pitch: float = 0.0,
    ) -> None:
        self.params = params or RobotParams()
        self.model = load_model(self.params, sim)
        self.data = mujoco.MjData(self.model)
        self.controller = controller
        self.eye_thresholds = EyeThresholds()
        self._chassis_id = self.model.body(CHASSIS).id
        self._push_force = 0.0
        self._push_until = 0.0
        self.height = self.params.leg.default_height
        self.fallen = False
        self.reset(initial_pitch)

    def reset(self, initial_pitch: float = 0.0) -> None:
        mujoco.mj_resetData(self.model, self.data)
        self.height = self.params.leg.default_height
        self.fallen = False
        self._push_until = 0.0
        self.data.qpos[2] = self.params.wheel.radius + self.height * math.cos(initial_pitch)
        self.data.qpos[3:7] = (math.cos(initial_pitch / 2), 0.0, math.sin(initial_pitch / 2), 0.0)
        angles = leg_angles(self.height, self.params.leg)
        for side in SIDES:
            self.data.qpos[self.model.joint(side.hip).qposadr[0]] = angles.hip
            self.data.qpos[self.model.joint(side.knee).qposadr[0]] = angles.knee
        self._apply_leg_targets()
        mujoco.mj_forward(self.model, self.data)
        self.controller.reset()

    @property
    def time(self) -> float:
        return float(self.data.time)

    @property
    def position_x(self) -> float:
        return float(self.data.xpos[self._chassis_id, 0])

    def push(self, force: float, duration: float) -> None:
        self._push_force = force
        self._push_until = self.time + duration

    def step(self, command: Command) -> Sample:
        state = read_state(self.data, self.params)
        self.fallen = self.fallen or abs(state.pitch) > FALLEN_PITCH
        self._track_height(command.height)
        self._apply_leg_targets()
        torques = self._wheel_torques(state, command)
        self._set_wheel_torques(torques)
        pushing = self.time < self._push_until
        self.data.xfrc_applied[self._chassis_id, 0] = self._push_force if pushing else 0.0
        mujoco.mj_step(self.model, self.data)
        apply_eye_color(self.model, self._eye_color(state))
        return Sample(
            time=self.time,
            pitch=state.pitch,
            forward_velocity=state.forward_velocity,
            command_velocity=command.forward_velocity,
            height=self.height,
            left_torque=torques.left,
            right_torque=torques.right,
            position_x=self.position_x,
            fallen=self.fallen,
        )

    def _wheel_torques(self, state: RobotState, command: Command) -> WheelTorques:
        if self.fallen:
            return WheelTorques(left=0.0, right=0.0)
        torques = self.controller.compute(state, command, self.model.opt.timestep)
        limit = self.params.wheel.max_torque
        return WheelTorques(
            left=max(-limit, min(limit, torques.left)),
            right=max(-limit, min(limit, torques.right)),
        )

    def _set_wheel_torques(self, torques: WheelTorques) -> None:
        self._set_control(SIDES[0], torques.left)
        self._set_control(SIDES[1], torques.right)

    def _set_control(self, side: Side, torque: float) -> None:
        self.data.ctrl[self.model.actuator(side.wheel_motor).id] = torque

    def _track_height(self, target: float) -> None:
        max_change = self.params.leg.max_height_rate * self.model.opt.timestep
        error = clamp_height(target, self.params.leg) - self.height
        self.height += max(-max_change, min(max_change, error))

    def _apply_leg_targets(self) -> None:
        angles = leg_angles(self.height, self.params.leg)
        for side in SIDES:
            self.data.ctrl[self.model.actuator(side.hip_servo).id] = angles.hip
            self.data.ctrl[self.model.actuator(side.knee_servo).id] = angles.knee

    def _eye_color(self, state: RobotState) -> tuple[float, float, float]:
        return RED if self.fallen else eye_color(state.pitch, self.eye_thresholds)
