# 0001. MuJoCo and Python for simulation

Status: accepted

## Context

The project must run on macOS on Apple Silicon. Isaac Sim needs an NVIDIA GPU. Gazebo with ROS 2
runs on macOS only through Docker or a VM. The robot is small and the focus is control, not
navigation.

## Decision

Use MuJoCo through its official Python bindings. The model is described in MJCF. The viewer runs
with `mjpython`, as required on macOS.

## Consequences

- Native install with `pip`, no ROS stack.
- Accurate contact physics and built-in finite-difference linearization for LQR.
- Few ready-made sensors and robots, so the model is written by hand.
