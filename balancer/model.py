import mujoco

from balancer.params import RobotParams, SimParams

CHASSIS = "chassis"
LEFT_WHEEL = "left_wheel"
RIGHT_WHEEL = "right_wheel"
LEFT_MOTOR = "left_motor"
RIGHT_MOTOR = "right_motor"
EYE_MATERIAL = "eye"
TRACKING_CAMERA = "track"


def _assets_xml() -> str:
    return """
  <asset>
    <texture name="grid" type="2d" builtin="checker" width="512" height="512"
             rgb1="0.82 0.84 0.86" rgb2="0.74 0.76 0.78"/>
    <texture name="sky" type="skybox" builtin="gradient" width="512" height="512"
             rgb1="0.95 0.96 0.98" rgb2="0.70 0.74 0.80"/>
    <material name="floor" texture="grid" texrepeat="8 8" reflectance="0.05"/>
    <material name="shell" rgba="0.92 0.92 0.90 1" specular="0.2" shininess="0.2"/>
    <material name="tire" rgba="0.12 0.12 0.13 1" specular="0.1"/>
    <material name="eye" rgba="0.05 0.05 0.05 1" emission="1"/>
    <material name="accent" rgba="0.30 0.32 0.36 1"/>
  </asset>"""


def _wheel_xml(name: str, side: float, params: RobotParams) -> str:
    wheel = params.wheel
    return f"""
      <body name="{name}" pos="0 {side * params.wheel_offset:.4f} 0">
        <joint name="{name}" type="hinge" axis="0 1 0" damping="0.001" armature="0.0005"/>
        <geom type="cylinder" size="{wheel.radius} {wheel.width / 2}" euler="90 0 0"
              mass="{wheel.mass}" material="tire" friction="1.2 0.01 0.001"/>
      </body>"""


def _face_xml(params: RobotParams) -> str:
    body, eye, antenna = params.body, params.eye, params.antenna
    top = body.height - body.bottom_below_axle
    eye_x = body.depth / 2 + eye.thickness / 2 - 0.001
    eye_z = top - eye.height_from_top
    ball_z = top + antenna.length
    eyes = "".join(
        f"""
      <geom class="visual" type="cylinder" size="{eye.radius} {eye.thickness / 2}"
            pos="{eye_x:.4f} {side * eye.spacing / 2:.4f} {eye_z:.4f}" euler="0 90 0"
            material="{EYE_MATERIAL}"/>"""
        for side in (1, -1)
    )
    return f"""{eyes}
      <geom class="visual" type="capsule" fromto="0 0 {top:.4f} 0 0 {ball_z:.4f}"
            size="{antenna.rod_radius}" material="accent"/>
      <geom class="visual" type="sphere" size="{antenna.ball_radius}"
            pos="0 0 {ball_z:.4f}" material="accent"/>"""


def build_mjcf(params: RobotParams, sim: SimParams) -> str:
    body = params.body
    return f"""<mujoco model="robot_balancer">
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
    <geom name="floor" type="plane" size="20 20 0.1" material="floor"/>
    <body name="{CHASSIS}" pos="0 0 {params.axle_height:.4f}">
      <freejoint/>
      <camera name="{TRACKING_CAMERA}" mode="trackcom" pos="0.7 -0.7 0.25"
              xyaxes="1 1 0 -0.25 0.25 1.4"/>
      <site name="imu" pos="0 0 {params.body_center_above_axle:.4f}"/>
      <geom type="box" size="{body.depth / 2} {body.width / 2} {body.height / 2}"
            pos="0 0 {params.body_center_above_axle:.4f}" mass="{body.mass}" material="shell"/>
{_face_xml(params)}
{_wheel_xml(LEFT_WHEEL, 1, params)}
{_wheel_xml(RIGHT_WHEEL, -1, params)}
    </body>
  </worldbody>
  <actuator>
    <motor name="{LEFT_MOTOR}" joint="{LEFT_WHEEL}" ctrllimited="true"
           ctrlrange="{-params.wheel.max_torque} {params.wheel.max_torque}"/>
    <motor name="{RIGHT_MOTOR}" joint="{RIGHT_WHEEL}" ctrllimited="true"
           ctrlrange="{-params.wheel.max_torque} {params.wheel.max_torque}"/>
  </actuator>
  <sensor>
    <framequat name="orientation" objtype="site" objname="imu"/>
    <gyro name="gyro" site="imu"/>
    <jointvel name="left_wheel_velocity" joint="{LEFT_WHEEL}"/>
    <jointvel name="right_wheel_velocity" joint="{RIGHT_WHEEL}"/>
  </sensor>
</mujoco>
"""


def load_model(params: RobotParams | None = None, sim: SimParams | None = None) -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_string(build_mjcf(params or RobotParams(), sim or SimParams()))
