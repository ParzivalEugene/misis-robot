import math
from dataclasses import dataclass

from balancer.params import LegParams

KNEE_LIMIT = 2.8


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


def leg_angles_toward(x: float, z: float, params: LegParams, near_hip: float) -> LegAngles:
    thigh, shin = params.thigh_length, params.shin_length
    min_reach = math.sqrt(thigh**2 + shin**2 - 2 * thigh * shin * math.cos(math.pi - KNEE_LIMIT))
    reach = max(min_reach, min(0.999 * (thigh + shin), math.hypot(x, z)))
    direction = math.atan2(-x, -z)
    cos_thigh = (reach**2 + thigh**2 - shin**2) / (2 * reach * thigh)
    cos_knee = (thigh**2 + shin**2 - reach**2) / (2 * thigh * shin)
    thigh_offset = math.acos(max(-1.0, min(1.0, cos_thigh)))
    knee = -(math.pi - math.acos(max(-1.0, min(1.0, cos_knee))))
    hip = direction + thigh_offset
    return LegAngles(hip=hip + 2 * math.pi * round((near_hip - hip) / (2 * math.pi)), knee=knee)
