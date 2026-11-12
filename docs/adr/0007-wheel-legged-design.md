# 0007. Wheel-legged design

Status: accepted

## Context

A plain two-wheeled balancer was judged too simple for the course. Adding articulated legs makes
the robot more interesting to design and control while keeping the balancing work already done.

## Decision

Each side has a two-link leg (hip and knee, pitch axis) with a wheel at the end. Leg joints use
MuJoCo position actuators acting as servos. Leg inverse kinematics keeps the wheel directly under
the hip, so the body pitch stays a valid balance signal and the PID balance controller is reused
unchanged. Ride height is part of the command, and leg control lives outside the balance
controller. A fall is latched when body pitch exceeds a threshold, after which wheel torques are
zero until reset.

## Consequences

- Center of mass height changes with ride height, so PID gains must work across the whole range.
- Wheel reaction torque passes through the leg servos, which must be stiff enough.
- Leg and wheel geoms collide only with the floor, not with the body or each other.
