import time
from dataclasses import dataclass

import mujoco
import mujoco.viewer

from balancer.control import Command, PidController
from balancer.model import CHASSIS
from balancer.sim import Simulation

KEY_UP = 265
KEY_DOWN = 264
KEY_LEFT = 263
KEY_RIGHT = 262
KEY_PAGE_UP = 266
KEY_PAGE_DOWN = 267
KEY_HOME = 268

VELOCITY_STEP = 0.1
YAW_RATE_STEP = 0.5
MAX_VELOCITY = 1.0
MAX_YAW_RATE = 3.0
PUSH_FORCE = 8.0
PUSH_DURATION = 0.1


@dataclass
class Teleop:
    velocity: float = 0.0
    yaw_rate: float = 0.0
    pending_push: float = 0.0

    def handle_key(self, key: int) -> None:
        if key == KEY_UP:
            self.velocity = min(MAX_VELOCITY, self.velocity + VELOCITY_STEP)
        elif key == KEY_DOWN:
            self.velocity = max(-MAX_VELOCITY, self.velocity - VELOCITY_STEP)
        elif key == KEY_LEFT:
            self.yaw_rate = min(MAX_YAW_RATE, self.yaw_rate + YAW_RATE_STEP)
        elif key == KEY_RIGHT:
            self.yaw_rate = max(-MAX_YAW_RATE, self.yaw_rate - YAW_RATE_STEP)
        elif key == KEY_HOME:
            self.velocity, self.yaw_rate = 0.0, 0.0
        elif key == KEY_PAGE_UP:
            self.pending_push = PUSH_FORCE
        elif key == KEY_PAGE_DOWN:
            self.pending_push = -PUSH_FORCE

    @property
    def command(self) -> Command:
        return Command(forward_velocity=self.velocity, yaw_rate=self.yaw_rate)


def follow_robot(viewer: mujoco.viewer.Handle, model: mujoco.MjModel) -> None:
    viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    viewer.cam.trackbodyid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, CHASSIS)
    viewer.cam.distance = 1.2
    viewer.cam.elevation = -15.0
    viewer.cam.azimuth = 135.0


def main() -> None:
    simulation = Simulation(PidController(), initial_pitch=0.05)
    teleop = Teleop()
    with mujoco.viewer.launch_passive(
        simulation.model, simulation.data, key_callback=teleop.handle_key
    ) as viewer:
        follow_robot(viewer, simulation.model)
        while viewer.is_running():
            frame_start = time.perf_counter()
            if teleop.pending_push:
                simulation.push(teleop.pending_push, PUSH_DURATION)
                teleop.pending_push = 0.0
            frame_end = simulation.time + 1 / 60
            while simulation.time < frame_end:
                simulation.step(teleop.command)
            viewer.sync()
            time.sleep(max(0.0, 1 / 60 - (time.perf_counter() - frame_start)))


if __name__ == "__main__":
    main()
