import math
from dataclasses import dataclass

from balancer.params import LegParams


@dataclass(frozen=True)
class LegAngles:
    hip: float
    knee: float


def clamp_height(height: float, params: LegParams) -> float:
    return max(params.min_height, min(params.max_height, height))


def leg_angles(height: float, params: LegParams) -> LegAngles:
    reach = clamp_height(height, params)
    thigh, shin = params.thigh_length, params.shin_length
    cos_thigh = (reach**2 + thigh**2 - shin**2) / (2 * reach * thigh)
    cos_shin = (reach**2 + shin**2 - thigh**2) / (2 * reach * shin)
    thigh_angle = math.acos(max(-1.0, min(1.0, cos_thigh)))
    shin_angle = math.acos(max(-1.0, min(1.0, cos_shin)))
    return LegAngles(hip=thigh_angle, knee=-(thigh_angle + shin_angle))
