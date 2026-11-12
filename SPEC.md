# Robot Balancer: Specification

## Goal

Design and program a two-wheeled self-balancing robot in simulation to close the robotics course.
The project covers both course parts: 3D modelling (robot appearance in Blender) and programming
(physics model and control in MuJoCo with Python).

## Robot

- Upright rounded body, about 30 cm tall, total mass about 1.2 kg.
- Two side wheels, 10 cm in diameter, each driven by a torque-controlled motor with a torque limit.
- Two round glowing eyes on the front face, a short antenna with a ball on top. No mouth.
- Colors: matte white or light grey body, dark wheels, eyes dark at rest and lit by state color.
- Ideal sensors: body pitch angle, pitch rate, wheel angles and wheel velocities. No noise.

## Scope

Required:

1. MuJoCo model built from primitives for physics, with Blender meshes as visuals.
2. PID balance controller.
3. Robot stands still without falling for at least 30 s.
4. Robot recovers from an external push.
5. Robot drives forward at a commanded speed while staying balanced.
6. Eyes change color smoothly with tilt angle: green upright, yellow at large tilt, red when falling.
7. Keyboard teleop in the viewer: arrow keys set forward speed and turn rate.
8. Scripted scenarios: stand, push, drive.
9. Plots of pitch angle and control signal, mp4 videos of the scenarios.
10. README in Russian as the course report.

Bonus:

- LQR controller with linearization from MuJoCo, compared to PID on the same scenarios.

## Out of scope

- Sensor noise, state estimation, filtering.
- Real hardware, ROS.
- SLAM, navigation, path planning.
- Rough terrain or slopes.

## Done criteria

- Smoke tests pass for required items 3, 4 and 5.
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
