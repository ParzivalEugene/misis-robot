from dataclasses import dataclass

import mujoco
import numpy as np

from balancer.model import EYE_MATERIAL

Color = tuple[float, float, float]

GREEN: Color = (0.15, 0.95, 0.35)
YELLOW: Color = (1.0, 0.85, 0.1)
RED: Color = (1.0, 0.12, 0.1)


@dataclass(frozen=True)
class EyeThresholds:
    warning_pitch: float = 0.12
    falling_pitch: float = 0.35


def eye_color(pitch: float, thresholds: EyeThresholds) -> Color:
    tilt = abs(pitch)
    if tilt <= thresholds.warning_pitch:
        return _blend(GREEN, YELLOW, tilt / thresholds.warning_pitch)
    span = thresholds.falling_pitch - thresholds.warning_pitch
    return _blend(YELLOW, RED, min(1.0, (tilt - thresholds.warning_pitch) / span))


def _blend(start: Color, end: Color, ratio: float) -> Color:
    mixed = (1 - ratio) * np.asarray(start) + ratio * np.asarray(end)
    return (float(mixed[0]), float(mixed[1]), float(mixed[2]))


def apply_eye_color(model: mujoco.MjModel, color: Color) -> None:
    material_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_MATERIAL, EYE_MATERIAL)
    model.mat_rgba[material_id, :3] = color
