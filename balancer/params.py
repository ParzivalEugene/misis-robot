from dataclasses import dataclass


@dataclass(frozen=True)
class BodyParams:
    width: float = 0.14
    depth: float = 0.08
    height: float = 0.30
    bottom_below_axle: float = 0.03
    mass: float = 1.0


@dataclass(frozen=True)
class WheelParams:
    radius: float = 0.05
    width: float = 0.025
    gap: float = 0.005
    mass: float = 0.1
    max_torque: float = 0.6


@dataclass(frozen=True)
class EyeParams:
    radius: float = 0.018
    thickness: float = 0.004
    spacing: float = 0.06
    height_from_top: float = 0.08


@dataclass(frozen=True)
class AntennaParams:
    length: float = 0.06
    rod_radius: float = 0.003
    ball_radius: float = 0.012


@dataclass(frozen=True)
class RobotParams:
    body: BodyParams = BodyParams()
    wheel: WheelParams = WheelParams()
    eye: EyeParams = EyeParams()
    antenna: AntennaParams = AntennaParams()

    @property
    def axle_height(self) -> float:
        return self.wheel.radius

    @property
    def body_center_above_axle(self) -> float:
        return self.body.height / 2 - self.body.bottom_below_axle

    @property
    def wheel_offset(self) -> float:
        return self.body.width / 2 + self.wheel.gap + self.wheel.width / 2


@dataclass(frozen=True)
class SimParams:
    timestep: float = 0.002
