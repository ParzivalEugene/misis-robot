import math
from dataclasses import dataclass

import mujoco

from balancer.control import Command, Controller
from balancer.eyes import EyeThresholds, apply_eye_color, eye_color
from balancer.model import CHASSIS, LEFT_MOTOR, RIGHT_MOTOR, load_model
from balancer.params import RobotParams, SimParams
from balancer.state import read_state

FALLEN_PITCH = 0.7


@dataclass(frozen=True)
class Sample:
    time: float
    pitch: float
    forward_velocity: float
    command_velocity: float
    left_torque: float
    right_torque: float
    position_x: float


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
        self._chassis_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, CHASSIS)
        self._left_motor = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, LEFT_MOTOR)
        self._right_motor = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, RIGHT_MOTOR)
        self._push_force = 0.0
        self._push_until = 0.0
        self.reset(initial_pitch)

    def reset(self, initial_pitch: float = 0.0) -> None:
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[3:7] = (math.cos(initial_pitch / 2), 0.0, math.sin(initial_pitch / 2), 0.0)
        mujoco.mj_forward(self.model, self.data)
        self.controller.reset()
        self._push_until = 0.0

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
        torques = self.controller.compute(state, command, self.model.opt.timestep)
        self.data.ctrl[self._left_motor] = torques.left
        self.data.ctrl[self._right_motor] = torques.right
        pushing = self.time < self._push_until
        self.data.xfrc_applied[self._chassis_id, 0] = self._push_force if pushing else 0.0
        mujoco.mj_step(self.model, self.data)
        apply_eye_color(self.model, eye_color(state.pitch, self.eye_thresholds))
        return Sample(
            time=self.time,
            pitch=state.pitch,
            forward_velocity=state.forward_velocity,
            command_velocity=command.forward_velocity,
            left_torque=float(self.data.ctrl[self._left_motor]),
            right_torque=float(self.data.ctrl[self._right_motor]),
            position_x=self.position_x,
        )
