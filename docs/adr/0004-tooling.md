# 0004. Tooling

Status: accepted

## Context

MuJoCo wheels may lag behind the newest Python release. The project needs reproducible setup with
minimal effort.

## Decision

- `uv` with `pyproject.toml` and a lock file, Python 3.12.
- `ruff` for linting and formatting.
- `pyright` for type checking.
- `pytest` for tests.

## Consequences

- One command recreates the environment: `uv sync`.
- Lint, format and type checks run before each commit.
