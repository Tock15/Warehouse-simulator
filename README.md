# Warehouse Robot Solver

SWI-Prolog engine (all search/reasoning) + Pygame GUI via PySwip. See `NewPlan.md`.

## Setup
1. Install SWI-Prolog (64-bit) and make sure `swipl` is on PATH (PySwip needs it).
2. `uv sync`

## Tests
- `uv run pytest`
- `swipl -g run_tests -t halt tests/test_*.pl`

## Layout
- `prolog/` engine: `level.pl`, `rules.pl`, `heuristic.pl`, `search.pl`, `solve.pl` (plan section 6)
- `src/warehouse_robot/bridge.py` PySwip bridge; runs Prolog on one background thread (plan section 5)
