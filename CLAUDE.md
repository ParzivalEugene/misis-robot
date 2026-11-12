# Robot Balancer

Two-wheeled self-balancing robot in MuJoCo with Blender visuals. Scope and done criteria are in
`SPEC.md`, decisions in `docs/adr/`.

## Code rules

- No comments, no docstrings.
- Type hints on every signature.
- Small single-purpose functions, readable names.
- Robot parameters in frozen dataclasses, no magic numbers in logic.
- English for code and technical docs (`SPEC.md`, ADR, this file).

## Russian-language deliverables

`README.md` and anything shown at the course review are in Russian. Style: formal, concise,
readable. No emoji, no em dashes, no GIFs, no filler. Images are PNG only, videos are mp4 files in
`media/` referenced by plain links.

## Layout

- `balancer/`: library code (model, controllers, scenarios, eyes).
- `scripts/`: thin entry points.
- `assets/`: MJCF and meshes. `blender/`: generation script and `.blend`.
- `tests/`: smoke tests for the done criteria only.

## Commands

- Setup: `uv sync`
- Viewer: `uv run mjpython scripts/<script>.py`
- Checks: `uv run ruff check . && uv run ruff format --check . && uv run pyright && uv run pytest`
- Blender model: `blender -b -P blender/make_robot.py`

## Physics model

Collisions and inertia come from primitive geoms. Mesh geoms are visual only (`contype="0"`,
`conaffinity="0"`, `mass="0"`). Blender units are meters, part origins sit on joint axes.
