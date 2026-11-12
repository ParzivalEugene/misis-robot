# 0006. PID as primary controller, LQR as bonus

Status: accepted

## Context

The required behaviors can be met with a cascaded PID controller. LQR gives better behavior and a
stronger report, but adds linearization and tuning work.

## Decision

Implement PID first: an inner loop on pitch angle and an outer loop on wheel velocity that sets the
target pitch. Implement LQR afterwards if time permits, using MuJoCo finite-difference
linearization and `scipy` to solve the Riccati equation. Both controllers share one interface so
scenarios and tests run with either.

## Consequences

- The required scope does not depend on LQR.
- The shared controller interface must be defined before scenarios are written.
