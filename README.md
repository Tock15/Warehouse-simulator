# Warehouse Robot Solver

SWI-Prolog engine (all search/reasoning) + Pygame GUI via PySwip. See `NewPlan.md`.

## Setup
1. Install SWI-Prolog (64-bit) and make sure `swipl` is on PATH (PySwip needs it).
2. `uv sync`

## Tests
- `uv run pytest`
- `swipl -g run_tests -t halt tests/test_*.pl`

## Benchmark (report numbers)
- `swipl -g main -t halt bench/bench.pl`: cost, expanded nodes and time for UCS, A*, greedy on a fixed level set (UCS gives up after 120 s per level)

## Layout
- `prolog/` engine: `level.pl`, `rules.pl`, `heuristic.pl`, `search.pl`, `solve.pl` (plan section 6)
- `src/warehouse_robot/bridge.py` PySwip bridge; runs Prolog on one background thread (plan section 5)
