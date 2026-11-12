# 0003. Code style

Status: accepted

## Context

The codebase is small and must stay easy to read and defend at the course review.

## Decision

- No comments and no docstrings. Names and structure carry the meaning.
- Type hints on every function signature and on non-obvious variables.
- Small functions with one responsibility.
- Robot parameters (dimensions, masses, gains, limits) live in frozen dataclasses, not as magic
  numbers in logic.
- English for code, identifiers and technical docs.

## Consequences

- Code that needs a comment to be understood gets renamed or split instead.
- Tuning parameters are found in one place.
