import math
from dataclasses import dataclass
from enum import Enum

import mujoco

from balancer.control import Command, Controller, WheelTorques
from balancer.eyes import RED, YELLOW, Color, EyeThresholds, apply_eye_color, eye_color
from balancer.legs import LegAngles, clamp_height, leg_angles
from balancer.model import CHASSIS, SIDES, load_model
from balancer.params import RobotParams, SimParams
from balancer.recovery import GetUpInputs, GetUpParams, GetUpTrajectory, PushUp, approach
from balancer.state import RobotState, com_relative_to_hip, read_state

FALLEN_LEAN = 0.6
STANDING_UP_FALLEN_LEAN = 1.3


class Mode(Enum):
    BALANCING = "balancing"
    FALLEN = "fallen"
    GETTING_UP = "getting_up"


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
    mode: Mode

    @property
    def fallen(self) -> bool:
        return self.mode is not Mode.BALANCING


class Simulation:
    def __init__(
        self,
        controller: Controller,
        params: RobotParams | None = None,
        sim: SimParams | None = None,
        initial_pitch: float = 0.0,
        auto_recover: bool = True,
    ) -> None:
        self.params = params or RobotParams()
        self.model = load_model(self.params, sim)
        self.data = mujoco.MjData(self.model)
        self.controller = controller
        self.auto_recover = auto_recover
        self.eye_thresholds = EyeThresholds()
        self.get_up = GetUpParams()
        self._chassis_id = self.model.body(CHASSIS).id
        self._push_force = 0.0
        self._push_until = 0.0
        self.height = self.params.leg.default_height
        self.mode = Mode.BALANCING
        self._mode_since = 0.0
        self._hip_offset = 0.0
        self._leg_targets = leg_angles(self.height, self.params.leg)
        self._shoulder_target = 0.0
        self._push_up: PushUp | None = None
        self._stand_up: GetUpTrajectory | None = None
        self.reset(initial_pitch)

    def reset(self, initial_pitch: float = 0.0) -> None:
        mujoco.mj_resetData(self.model, self.data)
        self.height = self.params.leg.default_height
        self._set_mode(Mode.BALANCING)
        self._hip_offset = 0.0
        self._shoulder_target = 0.0
        self._push_up = None
        self._stand_up = None
        self._push_until = 0.0
        self.data.qpos[2] = self.params.wheel.radius + self.height * math.cos(initial_pitch)
        self.data.qpos[3:7] = (math.cos(initial_pitch / 2), 0.0, math.sin(initial_pitch / 2), 0.0)
        self._leg_targets = leg_angles(self.height, self.params.leg)
        for side in SIDES:
            self.data.qpos[self.model.joint(side.hip).qposadr[0]] = self._leg_targets.hip
            self.data.qpos[self.model.joint(side.knee).qposadr[0]] = self._leg_targets.knee
        self._apply_joint_targets()
        mujoco.mj_forward(self.model, self.data)
        self.controller.reset()

    @property
    def time(self) -> float:
        return float(self.data.time)

    @property
    def position_x(self) -> float:
        return float(self.data.xpos[self._chassis_id, 0])

    @property
    def fallen(self) -> bool:
        return self.mode is not Mode.BALANCING

    def push(self, force: float, duration: float) -> None:
        self._push_force = force
        self._push_until = self.time + duration

    def step(self, command: Command) -> Sample:
        state = read_state(self.data, self.params)
        self._update_mode(state)
        brake = self._update_joint_targets(state, command)
        self._apply_joint_targets()
        torques = self._wheel_torques(state, command, brake)
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
            mode=self.mode,
        )

    def _set_mode(self, mode: Mode) -> None:
        self.mode = mode
        self._mode_since = self.time

    def _time_in_mode(self) -> float:
        return self.time - self._mode_since

    def _update_mode(self, state: RobotState) -> None:
        if self.mode is Mode.BALANCING and abs(state.lean) > self._fallen_lean():
            self._stand_up = None
            self._set_mode(Mode.FALLEN)
        elif self.mode is Mode.FALLEN and self._ready_to_get_up(state):
            self._push_up = PushUp(
                state.pitch > 0, self._leg_targets, self._shoulder_target, self.get_up
            )
            self._set_mode(Mode.GETTING_UP)
        elif self.mode is Mode.GETTING_UP:
            self._check_get_up(state)

    def _fallen_lean(self) -> float:
        return FALLEN_LEAN if self._stand_up is None else STANDING_UP_FALLEN_LEAN

    def _ready_to_get_up(self, state: RobotState) -> bool:
        settled = state.motion < self.get_up.settled_motion
        return self.auto_recover and settled and self._time_in_mode() >= self.get_up.settle_time

    def _check_get_up(self, state: RobotState) -> None:
        if self._push_up is None:
            return
        started = self._time_in_mode() > self.get_up.min_get_up_time
        if started and abs(state.lean) < self.get_up.handover_lean:
            self._hand_over_to_balance()
        elif self._time_in_mode() > self.get_up.timeout:
            self._push_up = None
            self._set_mode(Mode.FALLEN)

    def _hand_over_to_balance(self) -> None:
        self.height = self.get_up.stance_height
        stance = leg_angles(self.height, self.params.leg)
        turns = round((self._leg_targets.hip - stance.hip) / (2 * math.pi))
        self._hip_offset = 2 * math.pi * turns
        goal = LegAngles(hip=stance.hip + self._hip_offset, knee=stance.knee)
        self._stand_up = GetUpTrajectory(self._leg_targets, goal, self.get_up.stand_speed)
        self._push_up = None
        self.controller.reset()
        self._set_mode(Mode.BALANCING)

    def _update_joint_targets(self, state: RobotState, command: Command) -> float:
        dt = self.model.opt.timestep
        if self.mode is Mode.GETTING_UP and self._push_up is not None:
            ahead, hip_height = com_relative_to_hip(self.data)
            inputs = GetUpInputs(state.pitch, state.lean, ahead, hip_height)
            outputs = self._push_up.step(inputs, self.params, dt)
            self._leg_targets, self._shoulder_target = outputs.legs, outputs.shoulder
            return outputs.wheel_brake
        if self.mode is Mode.BALANCING:
            retract = self.get_up.arm_retract_speed * dt
            self._shoulder_target = approach(self._shoulder_target, 0.0, retract)
            self._leg_targets = self._balancing_leg_targets(command, dt)
        return 0.0

    def _balancing_leg_targets(self, command: Command, dt: float) -> LegAngles:
        if self._stand_up is not None:
            targets = self._stand_up.advance(dt)
            if self._stand_up.done:
                self._stand_up = None
            return targets
        self._track_height(command.height)
        stance = leg_angles(self.height, self.params.leg)
        return LegAngles(hip=stance.hip + self._hip_offset, knee=stance.knee)

    def _wheel_torques(self, state: RobotState, command: Command, brake: float) -> WheelTorques:
        if self.mode is Mode.FALLEN:
            return WheelTorques(left=0.0, right=0.0)
        if self.mode is Mode.GETTING_UP:
            return WheelTorques(left=self._brake(0, brake), right=self._brake(1, brake))
        torques = self.controller.compute(state, command, self.model.opt.timestep)
        limit = self.params.wheel.max_torque
        return WheelTorques(
            left=max(-limit, min(limit, torques.left)),
            right=max(-limit, min(limit, torques.right)),
        )

    def _brake(self, side_index: int, gain: float) -> float:
        velocity = float(self.data.sensor(f"{SIDES[side_index].wheel}_velocity").data[0])
        return -gain * velocity

    def _set_wheel_torques(self, torques: WheelTorques) -> None:
        self.data.ctrl[self.model.actuator(SIDES[0].wheel_motor).id] = torques.left
        self.data.ctrl[self.model.actuator(SIDES[1].wheel_motor).id] = torques.right

    def _track_height(self, target: float) -> None:
        max_change = self.params.leg.max_height_rate * self.model.opt.timestep
        error = clamp_height(target, self.params.leg) - self.height
        self.height += max(-max_change, min(max_change, error))

    def _apply_joint_targets(self) -> None:
        for side in SIDES:
            self.data.ctrl[self.model.actuator(side.hip_servo).id] = self._leg_targets.hip
            self.data.ctrl[self.model.actuator(side.knee_servo).id] = self._leg_targets.knee
            self.data.ctrl[self.model.actuator(side.shoulder_servo).id] = self._shoulder_target

    def _eye_color(self, state: RobotState) -> Color:
        if self.mode is Mode.FALLEN:
            return RED
        if self.mode is Mode.GETTING_UP:
            return YELLOW
        return eye_color(state.lean, self.eye_thresholds)
