# 0008. Arms for self-righting, balance on center of mass lean

Status: accepted

## Context

The legs attach to the bottom of the body, so they can lift only the hip end. Scripted leg motions
and a random search over leg trajectories did not find a reliable way to stand up after a fall.
Balancing on body pitch also stopped being valid once legs fold, because the wheels are no longer
under the hip.

## Decision

- Add one arm per side with a shoulder servo near the top of the body. After a fall the arms push
  the top of the body up, as in a push-up, while leg inverse kinematics in the world frame moves the
  wheels under the center of mass. Leg and arm targets are rate limited.
- Balance on the lean angle between the wheel axle and the whole-robot center of mass, computed
  from MuJoCo subtree and frame sensors. The same signal is used for fall detection, the handover
  from getting up to balancing, and eye color.
- The robot is considered settled after a fall only when both angular and linear motion are small.
- Hip joints are unlimited, and the hip offset is unwrapped by whole turns after each recovery.

## Consequences

- The robot recovers from forward and backward falls and from repeated falls.
- The robot no longer drifts while standing, since the lean has no constant offset.
- Lean and center of mass are taken from the simulator. On real hardware they would be estimated
  from the IMU, joint encoders and the kinematic model.
- Recovery from a sideways fall stays out of scope.
