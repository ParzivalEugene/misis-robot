# 0005. Smoke tests only

Status: accepted

## Context

Unit testing a simulation-driven study project gives little value per line. What matters is whether
the robot meets the required behaviors.

## Decision

Test only the done criteria from the specification: the model loads, the robot stands for 30 s,
recovers from a push, and drives at a commanded speed. Tests run headless without the viewer.

## Consequences

- Tests double as an automatic check of the done criteria.
- Internal functions are not unit tested.
