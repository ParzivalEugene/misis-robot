# Robot Balancer: Specification

## Goal

Design and program a two-wheeled self-balancing robot with jointed legs in simulation to close the
robotics course. The project covers both course parts: 3D modelling (robot appearance in Blender)
and programming (physics model and control in MuJoCo with Python).

## Robot

- Upright rounded body, about 20 cm tall, total mass about 1.4 kg.
- Two legs, each with a hip and a knee joint driven by position servos, links 12 cm long.
- A wheel at the end of each leg, 10 cm in diameter, driven by a torque-controlled motor with a
  torque limit.
- Ride height adjustable from 10 cm to 22 cm, measured from wheel axle to hip.
- Two round glowing eyes on the front face, a short antenna with a ball on top. No mouth.
- Colors: matte white or light grey body, dark wheels, eyes dark at rest and lit by state color.
- Ideal sensors: body orientation, angular rate, joint positions and velocities. No noise.

## Scope

Required:

1. MuJoCo model built from primitives for physics, with Blender meshes as visuals.
2. PID balance controller.
3. Leg inverse kinematics that keeps each wheel under its hip for a commanded ride height.
4. Robot stands still without falling for at least 30 s.
5. Robot recovers from an external push.
6. Robot drives forward at a commanded speed while staying balanced.
7. Robot crouches and stands up while balancing.
8. Fall detection: once the robot falls, wheel motors are disabled until reset.
9. Eyes change color smoothly with tilt angle: green upright, yellow at large tilt, red when fallen.
10. Keyboard teleop in the viewer: speed, turn rate, ride height, push, reset.
11. Scripted scenarios: stand, push, drive, crouch.
12. Plots of pitch angle and control signal, mp4 videos of the scenarios.
13. README in Russian as the course report.

Bonus:

- Body roll into turns by commanding different leg heights.
- Jump.
- LQR controller with linearization from MuJoCo, compared to PID on the same scenarios.

## Out of scope

- Sensor noise, state estimation, filtering.
- Real hardware, ROS.
- SLAM, navigation, path planning.
- Rough terrain or slopes.

## Done criteria

- Smoke tests pass for required items 4 to 8.
- `ruff` and `pyright` report no errors.
- README contains run instructions, plots, screenshots and links to videos in `media/`.

## Deliverables

| Path | Content |
|---|---|
| `balancer/` | Model loading, controllers, scenarios, eye coloring |
| `scripts/` | Entry points: viewer, video recording, plotting |
| `assets/` | MJCF model and exported meshes |
| `blender/` | Blender generation script and `.blend` file |
| `tests/` | Smoke tests |
| `media/` | Plots, screenshots, videos |
| `docs/adr/` | Architecture decision records |
