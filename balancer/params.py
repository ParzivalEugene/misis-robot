from dataclasses import dataclass


@dataclass(frozen=True)
class BodyParams:
    width: float = 0.14
    depth: float = 0.08
    height: float = 0.20
    hip_above_bottom: float = 0.03
    mass: float = 1.0


@dataclass(frozen=True)
class LegParams:
    thigh_length: float = 0.12
    shin_length: float = 0.12
    link_radius: float = 0.012
    link_mass: float = 0.05
    gap: float = 0.006
    servo_kp: float = 60.0
    servo_kv: float = 1.5
    servo_max_force: float = 8.0
    min_height: float = 0.10
    max_height: float = 0.22
    default_height: float = 0.18
    max_height_rate: float = 0.25


@dataclass(frozen=True)
class WheelParams:
    radius: float = 0.05
    width: float = 0.025
    gap: float = 0.004
    mass: float = 0.1
    max_torque: float = 0.6


@dataclass(frozen=True)
class EyeParams:
    radius: float = 0.018
    thickness: float = 0.004
    spacing: float = 0.06
    height_from_top: float = 0.06


@dataclass(frozen=True)
class AntennaParams:
    length: float = 0.06
    rod_radius: float = 0.003
    ball_radius: float = 0.012


@dataclass(frozen=True)
class RobotParams:
    body: BodyParams = BodyParams()
    leg: LegParams = LegParams()
    wheel: WheelParams = WheelParams()
    eye: EyeParams = EyeParams()
    antenna: AntennaParams = AntennaParams()

    @property
    def body_center_above_hip(self) -> float:
        return self.body.height / 2 - self.body.hip_above_bottom

    @property
    def body_top_above_hip(self) -> float:
        return self.body.height - self.body.hip_above_bottom

    @property
    def leg_offset(self) -> float:
        return self.body.width / 2 + self.leg.gap + self.leg.link_radius

    @property
    def wheel_offset_from_leg(self) -> float:
        return self.leg.link_radius + self.wheel.gap + self.wheel.width / 2


@dataclass(frozen=True)
class SimParams:
    timestep: float = 0.002
