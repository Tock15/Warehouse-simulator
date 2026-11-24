# NewPlan: Warehouse Robot Solver

Revamp after the prof's feedback. The push rule made the original Sokoban too hard for the search we were showing, so we're switching to a **warehouse robot**. The robot picks boxes up and carries them; it never pushes. Users build their own levels, we validate them, and then the AI solves them.

**This plan covers the Prolog engine and the Python bridge only.** The UI (editor, rendering, animation) is the teammate's design. The **bridge contract** (section 5) is where the two sides meet.

---

## 1. What changed and why

| Old | New | Reason |
|---|---|---|
| Player pushes boxes | Robot picks up, carries, drops | Prof: no push rule, simpler search |
| Fixed level files | User-built levels: editor → submit → validate → solve | Prof: custom levels plus a solvability check |
| Identical boxes | Weights 1, 2, 3 | Different costs give A* something to work with |
| h_sum, h_match (h_match dominates, so max was useless) | max(h_match, h_far), where neither dominates | Prof: combine two heuristics with max |
| BFS, IDS, UCS, greedy, A*, pruning toggle | **UCS, greedy, A*** only | Prof: drop the options. UCS is the optimal reference |
| "Pruning" | Just the legal-move rules, always on | Prof: it's only which transitions are allowed |
| Node/time limits, stats, verify reports | **Removed** | Prof doesn't care about stats, and they complicated the code |
| Player-vs-AI mode | **Removed** | Prof: irrelevant to the course |

---

## 2. Game rules

### Board
- Fixed **10×10** grid, border always walls (8×8 playable).
- **1–4 boxes.** The box count drives the state space more than the grid size does. If UCS is too slow, cut to 3 boxes before shrinking the grid.
- Caps are starting values. Adjust them after measuring runtimes.

### Tiles
| Tile | Rule |
|---|---|
| Wall | Never passable |
| Floor | Passable |
| Goal | Passable. A box finishes here |
| **Fragile floor** (`max_weight` 1 or 2) | The robot can stand on it empty, or holding a box with weight ≤ `max_weight` |
| **Conveyor belt** (straight run of cells, entry → exit) | A one-way "slow teleporter". Stepping onto the entry carries the robot (and its box, if any) to the exit in one move, and the UI plays a slide animation. The robot can't stop, turn, pick or drop on a belt, and nothing can be placed on one. Cost = (1 + w) × belt length, the same as walking it, so it's never a cheap shortcut; it's just one-way. A belt can't run through walls or doors |
| **Plate** | A box can be dropped on it. The robot standing on it does nothing |
| **Door** | Open while **any box is on any plate**, otherwise a wall. No box may be dropped on it |

### Robot actions
| Action | When legal | Cost |
|---|---|---|
| Step (empty) | Target not a wall, box, closed door or belt cell (belts are entered only through their entry) | 1 |
| Step (carrying box of weight w) | Same, plus the target tile can hold w | 1 + w |
| Ride belt | Robot steps onto a belt's entry and comes out at its exit; the exit cell must be free (and hold w if carrying) | (1 + w) × length (w = 0 if empty) |
| PICK box | Robot empty, box on a neighboring cell (4 directions), robot's own tile can hold w, and if the box is on a plate the robot isn't standing on a door | 1 |
| DROP box | Neighboring cell is floor/goal/plate (not belt, door or wall), is empty, and can hold w | 1 |

- **Capacity: one box at a time.** There is no facing direction (it would multiply the state count by 4).
- **Boxes are obstacles** to the robot. That's what makes narrow aisles require planning: clear the blocker first.
- **A box can be dropped anywhere legal, not just on goals.** This is required for moving blockers out of the way and for parking a box on a plate.

### Goals: two modes, set per box
- `goal: null` means the box may finish on **any** goal.
- `goal: "g1"` means the box must finish on **that** goal.
- A level may mix both.

### Win condition
Every box is on a goal it accepts, and the robot holds nothing.

### Deadlocks are back, on purpose
Conveyors and plates/doors make some moves irreversible. Example: the last box lifted off a plate can never pass the door, so its goal must be reachable without the door. The validator catches these cases. Nobody needs to reason about them by hand.

---

## 3. Search design

### State
```prolog
state(Robot, Held, Boxes)
%  Robot : X-Y
%  Held  : none | BoxId
%  Boxes : list of Id-(X-Y), sorted by Id, for boxes that are not held
```
Weights, goals and tiles are fixed facts and are **not** stored in the state. Door status isn't stored either. It's computed from `Boxes`.

### Walking is folded into one action (macro actions)
If every single step were a node, the state count would grow by about 40× (one copy per robot cell). Instead, the search uses two kinds of action:

| Search action | Meaning | Cost |
|---|---|---|
| `fetch(Id, Stand)` | Walk empty to cell `Stand` next to box `Id`, then PICK | shortest walk + 1 |
| `deliver(Id, Stand, Target)` | Carry `Id` to `Stand`, then DROP onto neighboring `Target` | (1 + w) × shortest carry + 1 |

- "Shortest walk" and "shortest carry" come from a shortest-path search inside the move rules (belt edges, fragile tiles, current door status and current box obstacles all apply). Belt edges cost more than 1, so this is a small Dijkstra rather than a plain BFS. Nothing moves during a walk, so door status is constant for that search.
- **One Dijkstra per node, not one per child.** A deliver can have ~100 children (every legal drop cell). Generate them all from **one** search: one Dijkstra from the robot (empty) gives the cost of every `fetch`; one Dijkstra from the robot (carrying) gives the cost of every `deliver`. Then `findall` over that distance table.
- **Drop targets are never restricted** (e.g. "only goals, plates or parking spots"). Restricting them could make solvable levels look unsolvable and would break the optimality proof.
- Costs are exact, so **UCS and A* stay optimal**.
- `solve` expands the macro plan back into single steps for the UI (the old `walk/4` pattern).

### Algorithms
**Three separate algorithms** sharing one best-first loop. Only the priority changes. They're never combined. Only the *heuristics* are combined (the max), and that h feeds greedy and A*. UCS ignores h.
```prolog
priority(ucs,    G, _H, G).
priority(greedy, _G, H, H).
priority(astar,  G, H, F-NegG) :- F is G + H, NegG is -G.   % tie on f: prefer larger g
```
- **A\* tie-breaking:** costs are small integers, so many nodes tie on f. Preferring the larger g commits to one promising path instead of fanning out over all equal ones. Still optimal.
- **Where each one runs:**
  - Validator and normal solving use **A\***.
  - The UI may offer greedy and UCS for comparison.
  - **UCS only has to finish on the fixed benchmark levels** (the report's optimal reference). It has no heuristic, so it isn't expected to handle every user-built level quickly.
- **Branching:** A* still *generates* ~100 children per deliver, but only *expands* promising ones. Bad parking moves stay in the queue. So the heuristic must be cheap to evaluate: table lookups and ≤24 permutations, no path search inside h.
- **Keep a best-g table, not a closed set.** A state is skipped only if it was already reached with a g that's ≤ the new one. A* with an admissible (but possibly inconsistent) heuristic then stays optimal. The old closed-on-first-pop approach could lose optimality, since a max of heuristics isn't guaranteed to be consistent.
- No limits. The state space is finite (≤4 boxes on a 10×10 grid), so search always ends with a plan or `no_solution`.
- Count expanded nodes. That single number is the only statistic kept (A* vs greedy vs UCS).

### Heuristics (DRAFT: to be redesigned in a dedicated session)
> The prof grades the **quality of the heuristic** and the **technique of combining them with max**. Everything below is a first draft. Revisit before implementing step 4.

**Distance tables**, precomputed once per level at load:
`dist(W, From, To, D)` is the shortest path over cells that can hold weight W (W = 0 for empty), counting walls only.
Belt cells are treated as plain floor (in both directions), and doors and boxes are **ignored**. A belt costs the same per cell as walking, so treating it as floor never overestimates. Dropping restrictions can only make paths shorter, so these distances stay lower bounds.

**Box cost lower bound** for box `b` (weight w) to goal `g`:
- `b` already on `g`: 0
- `b` held by the robot at R: (1 + w) × max(0, dist(w, R, g) − 1) + 1
- otherwise, at P: 2 + (1 + w) × max(0, dist(w, P, g) − 2)
  (2 for PICK + DROP. The box moves one cell on PICK, onto the robot, and one cell on DROP, so carrying covers at least d − 2.)

**h_match**: the cheapest assignment of boxes to goals they accept, summing box cost lower bounds.
With ≤4 boxes that's at most 24 permutations, so **use plain permutations, not the Hungarian algorithm.**
Strong when many boxes remain. Ignores the robot's walking.

**h_far**: the worst single box.
`max over unplaced boxes b of [ walk_lb(Robot, b) + cost_lb(b, nearest accepted goal) ]`
where `walk_lb = max(0, dist(0, Robot, P) − 1)` (0 for the held box).
Strong when one box is far from the robot. Weak when many boxes remain.

**h = max(h_match, h_far).**
Neither dominates. Example: box A is next to the robot and 1 step from a goal; box B is 30 steps from the robot and 1 step from a goal. h_far ≈ 31, while h_match ≈ 4. With many light boxes near the robot, h_match wins instead.

### Proofs to write (report)
1. `dist` is a lower bound (it's a relaxation: fewer restrictions than the real moves).
2. Box cost lower bound ≤ the true cost to place that box.
3. h_match is admissible (each box needs its own goal, and each box's cost is a lower bound).
4. h_far is admissible (the robot has to reach that box and deliver it at some point).
5. The max of admissible heuristics is admissible.
6. Neither heuristic dominates (the counterexample above).
7. A* with a best-g table (reopening) is optimal with an admissible heuristic.

---

## 4. Validator

`validate(Level)` runs cheap checks first, then the full search.

**Cheap checks** (each failure returns a human-readable reason):
- Exactly one robot. 1–4 boxes. Weights in {1, 2, 3}.
- Enough goals: every box has an accepted goal, and no goal is required by two boxes.
- Every `goal` id a box names exists.
- Starting positions are legal: no box on a wall, belt or door, or on a fragile tile too weak for it. The robot is not on a wall, box or belt cell.
- Plates and doors come together (doors without plates are permanent walls, which is allowed but probably a mistake; reject for clarity).
- A box-to-goal assignment exists where every pair has a finite `dist` (otherwise: "box a can't reach any goal").

**Full check:** run A*. A plan found means valid. `no_solution` means "Level has no solution" (e.g. a doorstop or conveyor trap).
The space is finite and there are no limits, so this always finishes.

---

## 5. Bridge contract (between the UI and the engine)

### Level (Python dict; the UI may save it as JSON)
```python
{
  "walls":     [[x, y], ...],
  "goals":     [{"id": "g1", "pos": [x, y]}, ...],
  "boxes":     [{"id": "a", "pos": [x, y], "weight": 2, "goal": "g1"},   # or "goal": None
                ...],
  "robot":     [x, y],
  "fragile":   [{"pos": [x, y], "max_weight": 1}, ...],
  "belts":     [{"entry": [x, y], "exit": [x, y]}, ...],   # straight line; cells between are belt
  "plates":    [[x, y], ...],
  "doors":     [[x, y], ...]
}
```

### Calls
| Call | Returns |
|---|---|
| `validate(level)` | `{"ok": True}` or `{"ok": False, "reason": "3 boxes but 2 goals"}` |
| `solve(level, algo)` with `algo` ∈ `"astar" \| "greedy" \| "ucs"` | `{"plan": [...], "cost": 17, "nodes": 812}`, or `None` if unsolvable |

Plan steps for the UI to animate:
- `"u" | "d" | "l" | "r"`: one robot step
- `("ride", [x, y])`: slide along a belt to its exit at `[x, y]`
- `("pick", "a")`
- `("drop", "a", [x, y])`

### Trust boundary
The bridge checks the dict before building any Prolog text: ids match `[a-z][a-z0-9_]*`, coordinates are integers in 0–9, weights are in 1–3, and each belt's entry and exit lie on one straight line. Bad input raises `ValueError`. **Nothing from the UI is pasted into a query unchecked.**

### How the level reaches Prolog
The bridge turns the dict into **one** term and calls `load_level/1`, which asserts the facts. **Prolog does no text parsing.**
```prolog
level(Walls, Goals, Boxes, Robot, Fragile, Belts, Plates, Doors)
```
The calls still run on the existing single Prolog worker thread (`submit`), so the GUI never freezes.

---

## 6. Prolog layout

One idea per file:

| File | Contents |
|---|---|
| `level.pl` | `load_level/1`. Dynamic facts: `wall/1`, `goal/2`, `box_start/2`, `weight/2`, `target/2`, `fragile/2`, `belt/3` (entry, exit, length), `belt_cell/1`, `plate/1`, `door/1`, `robot_start/1` |
| `rules.pl` | `can_hold/2`, `door_open/1`, `step/4` (a single step or a belt ride), shortest path, `action/4` (fetch/deliver with cost), `goal_state/1` |
| `heuristic.pl` | `dist/4` table built at load, `h_match/2`, `h_far/2`, `h/2` = max |
| `search.pl` | The best-first loop, `priority/4` (with A* tie-breaking), the best-g table, node count |
| `solve.pl` | Entry points: `validate/2`, `solve/4`, macro plan → step plan |

### Style rules (to keep it easy to trace)
- **At most ~4 arguments per predicate.** Bundle values into terms like `node(State, G, Path)`. (The old `bf/8` and `expand/12` are what we're avoiding.)
- **One `%` comment per predicate**, saying what it means, in the form `% name(+In, -Out): meaning`.
- **One rule per clause where possible.** Clauses should read like the rules table in section 2.
- **No `nb_setval`, no global counters, no cuts** unless truly needed. Pass the node count as an argument.
- Use library `heaps` for the frontier and `assoc` for the best-g table, as now.

Example of the intended style:
```prolog
% door_open(+Boxes): some box sits on a plate.
door_open(Boxes) :- member(_-P, Boxes), plate(P), !.

% can_hold(+Cell, +W): Cell can hold the robot carrying weight W (0 = empty).
can_hold(C, _) :- \+ fragile(C, _).
can_hold(C, W) :- fragile(C, Max), W =< Max.
```

---

## 7. What moves to the new repo

**Port (copy, then adapt)**
- `state.pl`: `delta/3`, the BFS flood/walk pattern → `rules.pl`.
- `heuristic.pl`: permutation matching pattern → `h_match`.
- `level.pl`: dynamic-fact pattern only. Parsing is removed.
- `bridge.py`: worker thread, `submit`, `shutdown` as-is. Rewrite the query functions to match section 5.
- Test setup: plunit (`tests/*.pl`) + pytest.

**Delete**
- `deadlock.pl`, `verify.pl`, `experiments.py`, `results/`
- `solver.pl` limits, timing, stats tuple
- `bridge.py` `set_limits`, `verify`, `dead_boxes`
- Old `levels/*.txt` (the format doesn't fit the new rules)

**Write fresh**
- `search.pl`: a new best-first loop with a best-g table (reopening). Only the idea of `priority/4` carries over, not the old code
- `rules.pl`, `heuristic.pl` (h_far, dist tables), `solve.pl` (validator, plan expansion)
- Bridge input checks + level → term conversion

UI parts (`gui.py`, `theme.py`, assets, editor) are the teammate's call.

---

## 8. Tests

**plunit, one or more per rule** (these double as worked examples for the report):
- A weight-3 box can't be carried onto a `max_weight` 2 tile, and can't be dropped there.
- A belt only carries the robot from entry to exit, never back. Its cells can't be stood on or dropped on, and the ride costs (1 + w) × length.
- A door is a wall with no box on a plate, and open with one.
- PICK of a plate box from a door cell is illegal.
- A box with a pinned goal doesn't count as done on a different goal.
- h_match and h_far ≤ the UCS cost on a set of small levels (admissibility check).
- The doorstop level (2 boxes) is solvable. The same level with 1 box is reported unsolvable.
- UCS and A* return the same cost. Greedy returns a cost ≥ that.

**pytest (bridge):**
- A bad id, out-of-range coordinate or weight raises `ValueError`.
- `validate` returns the expected reasons for broken levels.
- `solve` returns a plan whose steps, when replayed, end in the win condition.

---

## 9. Build order
1. `level.pl` + bridge level → term, with tests.
2. `rules.pl` single steps and actions, with tests for every rule in section 2.
3. `search.pl` with UCS (h = 0). Solve tiny levels.
4. `heuristic.pl`: dist tables, h_match, h_far, max. Admissibility tests against UCS.
5. Greedy and A* priorities.
6. `solve.pl`: validator + plan expansion.
7. Bridge `validate`/`solve` final. Hand the contract to the teammate.
8. Measure runtimes on 4-box levels and adjust the caps if UCS is too slow.

---

## 10. Open items
- **Heuristic design (section 3) is a draft.** Redo it in a dedicated session. It's graded on quality and on the max-combination technique.
- Grid/box caps are provisional until step 8.
- **Fallbacks if step 8 shows UCS/A* too slow, in this order:**
  1. Treat identical-weight boxes in any-goal mode as interchangeable (store them sorted by position, not id). Up to ×24 fewer states with 4 identical boxes, and still optimal.
  2. Lower the box cap to 3.
- The teammate should confirm the plan-step format before animating it.
