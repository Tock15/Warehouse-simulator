"""Bridge validate()/solve() end to end. Plans are replayed by an independent Python version of
the rules (NewPlan.md section 2), so a wrong engine move or cost fails here."""
import pytest

from warehouse_robot import bridge

MOVES = {"u": (0, -1), "d": (0, 1), "l": (-1, 0), "r": (1, 0)}


def room(mx, my):
    """Walls on every inner cell outside 1..mx x 1..my."""
    return [[x, y] for x in range(1, 9) for y in range(1, 9) if x > mx or y > my]


def level(walls=(), goals=(), boxes=(), robot=(1, 1), fragile=(), belts=(), plates=(), doors=()):
    return {"walls": [list(w) for w in walls],
            "goals": [{"id": g, "pos": list(p)} for g, p in goals],
            "boxes": [{"id": b, "pos": list(p), "weight": w, "goal": t} for b, p, w, t in boxes],
            "robot": list(robot),
            "fragile": [{"pos": list(p), "max_weight": m} for p, m in fragile],
            "belts": [{"entry": list(e), "exit": list(x)} for e, x in belts],
            "plates": [list(p) for p in plates], "doors": [list(d) for d in doors]}


BASE = level(goals=[("g1", (6, 1))], boxes=[("a", (3, 1), 1, None)])
BELT = level(room(6, 3), [("g1", (6, 2))], [("a", (1, 3), 2, None)], belts=[((2, 2), (5, 2))])
FRAGILE = level(room(5, 3), [("g1", (5, 2))], [("a", (1, 2), 2, None)],
                fragile=[((3, 1), 1), ((3, 2), 1)])
DOOR_WALLS = room(7, 4) + [[4, y] for y in (1, 3, 4)]
DOORSTOP = level(DOOR_WALLS, [("g1", (7, 4)), ("g2", (1, 1))],
                 [("a", (2, 3), 1, "g1"), ("b", (1, 3), 1, "g2")], robot=(3, 4), plates=[(2, 1)], doors=[(4, 2)])
DOORSTOP_ALONE = level(DOOR_WALLS, [("g1", (7, 4))], [("a", (2, 3), 1, "g1")],
                       robot=(3, 4), plates=[(2, 1)], doors=[(4, 2)])


def replay(lv, plan):
    """Apply UI steps under the game rules, asserting each is legal. Returns (cost, won)."""
    walls = {tuple(w) for w in lv["walls"]}
    walls |= {(x, y) for x in range(10) for y in range(10) if x in (0, 9) or y in (0, 9)}
    fragile = {tuple(f["pos"]): f["max_weight"] for f in lv["fragile"]}
    plates, doors = {tuple(p) for p in lv["plates"]}, {tuple(d) for d in lv["doors"]}
    belts, belt_cells = {}, set()
    for b in lv["belts"]:
        (ex, ey), (xx, xy) = b["entry"], b["exit"]
        n = abs(xx - ex) + abs(xy - ey)
        dx, dy = (xx > ex) - (xx < ex), (xy > ey) - (xy < ey)
        belts[(xx, xy)] = ((ex, ey), n + 1)
        belt_cells |= {(ex + k * dx, ey + k * dy) for k in range(n)}
    goals = {g["id"]: tuple(g["pos"]) for g in lv["goals"]}
    weight = {b["id"]: b["weight"] for b in lv["boxes"]}
    target = {b["id"]: b["goal"] for b in lv["boxes"]}
    boxes = {b["id"]: tuple(b["pos"]) for b in lv["boxes"]}
    robot, held, cost = tuple(lv["robot"]), None, 0

    def holds(c, w):
        return w <= fragile.get(c, 3)

    def door_open():
        return any(p in plates for p in boxes.values())

    def standable(c, w):
        return (c not in walls and c not in boxes.values() and c not in belt_cells
                and not (c in doors and not door_open()) and holds(c, w))

    def adjacent(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1]) == 1

    for s in plan:
        w = weight[held] if held else 0
        if isinstance(s, str):
            nxt = (robot[0] + MOVES[s][0], robot[1] + MOVES[s][1])
            assert standable(nxt, w), (s, robot)
            robot, cost = nxt, cost + 1 + w
        elif s[0] == "ride":
            exit_ = tuple(s[1])
            entry, length = belts[exit_]
            assert adjacent(robot, entry) and standable(exit_, w), (s, robot)
            robot, cost = exit_, cost + (1 + w) * length
        elif s[0] == "pick":
            b = s[1]
            assert held is None and adjacent(robot, boxes[b]) and holds(robot, weight[b]), (s, robot)
            del boxes[b]
            assert robot not in doors or door_open(), "lift closed the door under the robot"
            held, cost = b, cost + 1 + weight[b]
        else:
            b, cell = s[1], tuple(s[2])
            assert held == b and adjacent(robot, cell) and holds(cell, w), (s, robot)
            assert cell not in walls | belt_cells | doors and cell not in boxes.values(), (s, cell)
            boxes[b], held, cost = cell, None, cost + 1 + w
    won = held is None and all(p == goals.get(target[b], p) and p in goals.values()
                               for b, p in boxes.items())
    return cost, won


def call(fn, *args):
    return bridge.submit(fn, *args).result(60)


@pytest.mark.parametrize("lv", [BASE, BELT, FRAGILE, DOORSTOP], ids=["base", "belt", "fragile", "doorstop"])
def test_plans_replay_to_a_win(lv):
    costs = {}
    for algo in ("ucs", "astar", "greedy"):
        res = call(bridge.solve, lv, algo)
        cost, won = replay(lv, res["plan"])
        assert won and cost == res["cost"], algo
        costs[algo] = cost
    assert costs["astar"] == costs["ucs"] <= costs["greedy"]


def test_validate_ok():
    assert call(bridge.validate, DOORSTOP) == {"ok": True}


@pytest.mark.parametrize("lv, reason", [
    (DOORSTOP_ALONE, "This level has no solution"),
    (level(goals=[("g1", (6, 1)), ("g2", (6, 2))], boxes=[("a", (3, 1), 1, None), ("b", (3, 1), 1, None)]),
     "Two boxes start on (3, 1)"),
    (level(goals=[("g1", (6, 1))], boxes=[("a", (3, 1), 1, "g2")]),
     "Box a needs goal g2, which does not exist"),
    (level(goals=[("g1", (6, 1))], boxes=[("a", (3, 1), 3, None)], fragile=[((3, 1), 2)]),
     "Box a (weight 3) is too heavy for the fragile tile at (3, 1)"),
])
def test_validate_reasons(lv, reason):
    assert call(bridge.validate, lv) == {"ok": False, "reason": reason}


def test_solve_unsolvable_is_none():
    assert call(bridge.solve, DOORSTOP_ALONE, "astar") is None


def test_solve_rejects_malformed_level():
    lv = level(goals=[("g1", (6, 1)), ("g2", (6, 2))], boxes=[("a", (3, 1), 1, None), ("b", (3, 1), 1, None)])
    with pytest.raises(ValueError, match="Two boxes start"):
        call(bridge.solve, lv, "astar")


def test_solve_rejects_unknown_algo():
    with pytest.raises(ValueError):
        bridge.solve(BASE, "bfs")
