import time
from dataclasses import dataclass, field

import mujoco
import mujoco.viewer

from balancer.control import Command, PidController
from balancer.model import CHASSIS
from balancer.params import LegParams
from balancer.sim import Simulation

KEY_UP = 265
KEY_DOWN = 264
KEY_LEFT = 263
KEY_RIGHT = 262
KEY_PAGE_UP = 266
KEY_PAGE_DOWN = 267
KEY_HOME = 268
KEY_END = 269
KEY_COMMA = 44
KEY_PERIOD = 46

VELOCITY_STEP = 0.1
YAW_RATE_STEP = 0.5
HEIGHT_STEP = 0.02
MAX_VELOCITY = 1.0
MAX_YAW_RATE = 3.0
PUSH_FORCE = 10.0
PUSH_DURATION = 0.1
FRAME_TIME = 1 / 60


@dataclass
class Teleop:
    legs: LegParams = field(default_factory=LegParams)
    velocity: float = 0.0
    yaw_rate: float = 0.0
    height: float = LegParams().default_height
    pending_push: float = 0.0
    reset_requested: bool = False

    def handle_key(self, key: int) -> None:
        if key == KEY_UP:
            self.velocity = min(MAX_VELOCITY, self.velocity + VELOCITY_STEP)
        elif key == KEY_DOWN:
            self.velocity = max(-MAX_VELOCITY, self.velocity - VELOCITY_STEP)
        elif key == KEY_LEFT:
            self.yaw_rate = min(MAX_YAW_RATE, self.yaw_rate + YAW_RATE_STEP)
        elif key == KEY_RIGHT:
            self.yaw_rate = max(-MAX_YAW_RATE, self.yaw_rate - YAW_RATE_STEP)
        elif key == KEY_PAGE_UP:
            self.height = min(self.legs.max_height, self.height + HEIGHT_STEP)
        elif key == KEY_PAGE_DOWN:
            self.height = max(self.legs.min_height, self.height - HEIGHT_STEP)
        elif key == KEY_HOME:
            self.velocity, self.yaw_rate = 0.0, 0.0
        elif key == KEY_PERIOD:
            self.pending_push = PUSH_FORCE
        elif key == KEY_COMMA:
            self.pending_push = -PUSH_FORCE
        elif key == KEY_END:
            self.reset_requested = True

    def restart(self) -> None:
        self.velocity, self.yaw_rate = 0.0, 0.0
        self.height = self.legs.default_height
        self.reset_requested = False

    @property
    def command(self) -> Command:
        return Command(forward_velocity=self.velocity, yaw_rate=self.yaw_rate, height=self.height)


def follow_robot(viewer: mujoco.viewer.Handle, model: mujoco.MjModel) -> None:
    viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    viewer.cam.trackbodyid = model.body(CHASSIS).id
    viewer.cam.distance = 1.3
    viewer.cam.elevation = -15.0
    viewer.cam.azimuth = 135.0


def print_controls() -> None:
    print(
        "Arrows: speed and turn | fn+Up/Down: height | fn+Left: stop | fn+Right: reset | "
        "comma/period: push back/forward"
    )


def main() -> None:
    simulation = Simulation(PidController(), initial_pitch=0.05)
    teleop = Teleop()
    print_controls()
    with mujoco.viewer.launch_passive(
        simulation.model, simulation.data, key_callback=teleop.handle_key
    ) as viewer:
        follow_robot(viewer, simulation.model)
        while viewer.is_running():
            frame_start = time.perf_counter()
            if teleop.reset_requested:
                with viewer.lock():
                    simulation.reset()
                teleop.restart()
            if teleop.pending_push:
                simulation.push(teleop.pending_push, PUSH_DURATION)
                teleop.pending_push = 0.0
            frame_end = simulation.time + FRAME_TIME
            with viewer.lock():
                while simulation.time < frame_end:
                    simulation.step(teleop.command)
            viewer.sync()
            time.sleep(max(0.0, FRAME_TIME - (time.perf_counter() - frame_start)))


if __name__ == "__main__":
    main()
