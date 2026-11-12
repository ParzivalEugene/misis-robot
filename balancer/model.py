import math
from dataclasses import dataclass

import mujoco

from balancer.params import RobotParams, SimParams

CHASSIS = "chassis"
EYE_MATERIAL = "eye"
TRACKING_CAMERA = "track"
QUARTER_TURN = f"{math.pi / 2:.6f}"

FLOOR_CONTACT = 'contype="1" conaffinity="6"'
BODY_CONTACT = 'contype="2" conaffinity="1"'
LIMB_CONTACT = 'contype="4" conaffinity="1"'


@dataclass(frozen=True)
class Side:
    name: str
    sign: int

    @property
    def hip(self) -> str:
        return f"{self.name}_hip"

    @property
    def knee(self) -> str:
        return f"{self.name}_knee"

    @property
    def wheel(self) -> str:
        return f"{self.name}_wheel"

    @property
    def wheel_motor(self) -> str:
        return f"{self.name}_wheel_motor"

    @property
    def hip_servo(self) -> str:
        return f"{self.name}_hip_servo"

    @property
    def knee_servo(self) -> str:
        return f"{self.name}_knee_servo"


LEFT = Side("left", 1)
RIGHT = Side("right", -1)
SIDES = (LEFT, RIGHT)


def _assets_xml() -> str:
    return """
  <asset>
    <texture name="grid" type="2d" builtin="checker" width="512" height="512"
             rgb1="0.82 0.84 0.86" rgb2="0.74 0.76 0.78"/>
    <texture name="sky" type="skybox" builtin="gradient" width="512" height="512"
             rgb1="0.95 0.96 0.98" rgb2="0.70 0.74 0.80"/>
    <material name="floor" texture="grid" texrepeat="8 8" reflectance="0.02"/>
    <material name="shell" rgba="0.92 0.92 0.90 1" specular="0.2" shininess="0.2"/>
    <material name="tire" rgba="0.12 0.12 0.13 1" specular="0.1"/>
    <material name="eye" rgba="0.05 0.05 0.05 1" emission="1"/>
    <material name="accent" rgba="0.30 0.32 0.36 1"/>
  </asset>"""


def _leg_xml(side: Side, params: RobotParams) -> str:
    leg, wheel = params.leg, params.wheel
    return f"""
      <body name="{side.hip}" pos="0 {side.sign * params.leg_offset:.4f} 0">
        <joint name="{side.hip}" type="hinge" axis="0 1 0" range="-0.3 1.6" damping="0.05"/>
        <geom type="capsule" fromto="0 0 0 0 0 {-leg.thigh_length}" size="{leg.link_radius}"
              mass="{leg.link_mass}" material="accent" {LIMB_CONTACT}/>
        <body name="{side.knee}" pos="0 0 {-leg.thigh_length}">
          <joint name="{side.knee}" type="hinge" axis="0 1 0" range="-2.8 0" damping="0.05"/>
          <geom type="capsule" fromto="0 0 0 0 0 {-leg.shin_length}" size="{leg.link_radius}"
                mass="{leg.link_mass}" material="accent" {LIMB_CONTACT}/>
          <body name="{side.wheel}"
                pos="0 {side.sign * params.wheel_offset_from_leg:.4f} {-leg.shin_length}">
            <joint name="{side.wheel}" type="hinge" axis="0 1 0" damping="0.001"
                   armature="0.0005"/>
            <geom type="cylinder" size="{wheel.radius} {wheel.width / 2}" euler="{QUARTER_TURN} 0 0"
                  mass="{wheel.mass}" material="tire" friction="1.2 0.01 0.001"
                  {LIMB_CONTACT}/>
          </body>
        </body>
      </body>"""


def _face_xml(params: RobotParams) -> str:
    body, eye, antenna = params.body, params.eye, params.antenna
    top = params.body_top_above_hip
    eye_x = body.depth / 2 + eye.thickness / 2 - 0.001
    eye_z = top - eye.height_from_top
    ball_z = top + antenna.length
    eyes = "".join(
        f"""
      <geom class="visual" type="cylinder" size="{eye.radius} {eye.thickness / 2}"
            pos="{eye_x:.4f} {side * eye.spacing / 2:.4f} {eye_z:.4f}" euler="0 {QUARTER_TURN} 0"
            material="{EYE_MATERIAL}"/>"""
        for side in (1, -1)
    )
    return f"""{eyes}
      <geom class="visual" type="capsule" fromto="0 0 {top:.4f} 0 0 {ball_z:.4f}"
            size="{antenna.rod_radius}" material="accent"/>
      <geom class="visual" type="sphere" size="{antenna.ball_radius}"
            pos="0 0 {ball_z:.4f}" material="accent"/>"""


def _actuators_xml(params: RobotParams) -> str:
    leg, torque = params.leg, params.wheel.max_torque
    servo = (
        f'kp="{leg.servo_kp}" kv="{leg.servo_kv}" forcelimited="true" '
        f'forcerange="{-leg.servo_max_force} {leg.servo_max_force}"'
    )
    return "".join(
        f"""
    <motor name="{side.wheel_motor}" joint="{side.wheel}" ctrllimited="true"
           ctrlrange="{-torque} {torque}"/>
    <position name="{side.hip_servo}" joint="{side.hip}" {servo}/>
    <position name="{side.knee_servo}" joint="{side.knee}" {servo}/>"""
        for side in SIDES
    )


def _sensors_xml() -> str:
    joints = "".join(
        f"""
    <jointvel name="{joint}_velocity" joint="{joint}"/>"""
        for side in SIDES
        for joint in (side.hip, side.knee, side.wheel)
    )
    return f"""
    <framequat name="orientation" objtype="site" objname="imu"/>
    <gyro name="gyro" site="imu"/>{joints}"""


def build_mjcf(params: RobotParams, sim: SimParams) -> str:
    body = params.body
    standing_height = params.wheel.radius + params.leg.default_height
    return f"""<mujoco model="robot_balancer">
  <compiler angle="radian"/>
  <option timestep="{sim.timestep}" integrator="implicitfast"/>
  <visual>
    <global offwidth="1280" offheight="720"/>
    <headlight ambient="0.35 0.35 0.35" diffuse="0.5 0.5 0.5"/>
  </visual>
  <default>
    <default class="visual">
      <geom contype="0" conaffinity="0" mass="0" group="1"/>
    </default>
  </default>
{_assets_xml()}
  <worldbody>
    <light pos="1 -1 3" dir="-0.3 0.3 -1" directional="true" castshadow="true"/>
    <geom name="floor" type="plane" size="20 20 0.1" material="floor" {FLOOR_CONTACT}/>
    <body name="{CHASSIS}" pos="0 0 {standing_height:.4f}">
      <freejoint/>
      <camera name="{TRACKING_CAMERA}" mode="trackcom" pos="0.8 -0.8 0.15"
              xyaxes="1 1 0 -0.2 0.2 1.4"/>
      <site name="imu" pos="0 0 {params.body_center_above_hip:.4f}"/>
      <geom type="box" size="{body.depth / 2} {body.width / 2} {body.height / 2}"
            pos="0 0 {params.body_center_above_hip:.4f}" mass="{body.mass}" material="shell"
            {BODY_CONTACT}/>
{_face_xml(params)}
{"".join(_leg_xml(side, params) for side in SIDES)}
    </body>
  </worldbody>
  <actuator>{_actuators_xml(params)}
  </actuator>
  <sensor>{_sensors_xml()}
  </sensor>
</mujoco>
"""


def load_model(params: RobotParams | None = None, sim: SimParams | None = None) -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_string(build_mjcf(params or RobotParams(), sim or SimParams()))
