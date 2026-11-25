"""Dev viewer: draws the test levels with basic shapes and plays solver plans. Not the real UI.

Run: uv run warehouse-viewer
Keys: left/right = level, A/G/U = solve with A*/greedy/UCS and play, R = reset, Esc = quit."""
import pygame

from warehouse_robot import bridge

CELL, PANEL = 56, 260
MOVES = {"u": (0, -1), "d": (0, 1), "l": (-1, 0), "r": (1, 0)}
STEPS_PER_SEC = 6

FLOOR, WALL, GRID = (235, 235, 230), (70, 70, 75), (205, 205, 200)
GOAL, FRAGILE, BELT = (60, 170, 90), (245, 190, 120), (120, 160, 230)
PLATE, DOOR, BOX = (240, 210, 60), (150, 95, 50), (200, 160, 100)
ROBOT, TEXT = (40, 90, 200), (20, 20, 20)


def room(mx, my):
    """Walls on every inner cell outside 1..mx x 1..my."""
    return [[x, y] for x in range(1, 9) for y in range(1, 9) if x > mx or y > my]


def level(walls=(), goals=(), boxes=(), robot=(1, 1), fragile=(), belts=(), plates=(), doors=()):
    return {"walls": [list(w) for w in walls],
            "goals": [{"id": g, "pos": list(p)} for g, p in goals],
            # goal "FREE" marks a free box (needs no goal); upper case, so never a real goal id
            "boxes": [{"id": b, "pos": list(p), "weight": w, "goal": None if t == "FREE" else t, "free": t == "FREE"}
                      for b, p, w, t in boxes],
            "robot": list(robot),
            "fragile": [{"pos": list(p), "max_weight": m} for p, m in fragile],
            "belts": [{"entry": list(e), "exit": list(x)} for e, x in belts],
            "plates": [list(p) for p in plates], "doors": [list(d) for d in doors]}


SHELVES = [[x, y] for y in (3, 6) for x in (2, 3, 4, 6, 7)]
DOOR_ROOM = room(7, 4) + [[4, y] for y in (1, 3, 4)]
KEY_ROOM = room(8, 5) + [[5, y] for y in (1, 2, 4, 5)]  # door gap at 5-3
LEVELS = [
    ("one box", level(goals=[("g1", (6, 1))], boxes=[("a", (3, 1), 1, None)])),
    ("heavy box", level(goals=[("g1", (6, 1))], boxes=[("a", (3, 1), 3, None)])),
    ("belt", level(room(6, 3), [("g1", (6, 2))], [("a", (1, 3), 2, None)], belts=[((2, 2), (5, 2))])),
    ("fragile", level(room(5, 3), [("g1", (5, 2))], [("a", (1, 2), 2, None)],
                      fragile=[((3, 1), 1), ((3, 2), 1)])),
    ("blocker", level(room(5, 5) + [[x, 2] for x in range(2, 6)], [("g1", (5, 1)), ("g2", (4, 4))],
                      [("a", (3, 4), 1, "g1"), ("b", (1, 1), 1, "g2")], robot=(1, 3))),
    ("doorstop", level(DOOR_ROOM, [("g1", (7, 4)), ("g2", (1, 1))],
                       [("a", (2, 3), 1, "g1"), ("b", (1, 3), 1, "g2")], robot=(3, 4),
                       plates=[(2, 1)], doors=[(4, 2)])),
    ("key crate", level(DOOR_ROOM, [("g1", (7, 4))], [("a", (2, 3), 1, "g1"), ("k", (1, 3), 1, "FREE")],
                        robot=(3, 4), plates=[(2, 1)], doors=[(4, 2)])),
    ("heavy key", level(KEY_ROOM, [("g1", (8, 1))], [("a", (2, 3), 1, None), ("k", (3, 5), 3, "FREE")],
                        robot=(4, 1), plates=[(1, 1)], doors=[(5, 3)])),
    ("key, 3 boxes", level(KEY_ROOM, [("g1", (8, 1)), ("g2", (8, 5))],
                           [("a", (2, 3), 1, None), ("b", (3, 1), 2, None), ("k", (1, 1), 2, "FREE")],
                           robot=(1, 5), plates=[(4, 5)], doors=[(5, 3)])),
    ("two pinned", level(room(5, 5), [("g1", (1, 5)), ("g2", (5, 5))],
                         [("a", (5, 2), 1, "g1"), ("b", (1, 2), 2, "g2")], robot=(3, 1))),
    ("shelves, 3 boxes", level(SHELVES, [("g1", (8, 1)), ("g2", (8, 4)), ("g3", (8, 7))],
                               [("a", (3, 2), 1, None), ("b", (5, 5), 2, None), ("c", (2, 7), 3, None)])),
    ("shelves, 4 boxes", level(SHELVES, [("g1", (8, 1)), ("g2", (8, 4)), ("g3", (8, 7)), ("g4", (8, 8))],
                               [("a", (3, 2), 1, None), ("b", (5, 5), 2, "g1"), ("c", (2, 7), 3, None),
                                ("d", (6, 8), 1, "g4")])),
]


def start(lv):
    """Mutable play state for a level: robot cell, held box id, box id -> cell."""
    return {"robot": tuple(lv["robot"]), "held": None, "boxes": {b["id"]: tuple(b["pos"]) for b in lv["boxes"]}}


def apply(st, step):
    """Advance the play state by one plan step (the engine already checked it is legal)."""
    if isinstance(step, str):
        dx, dy = MOVES[step]
        st["robot"] = (st["robot"][0] + dx, st["robot"][1] + dy)
    elif step[0] == "ride":
        st["robot"] = tuple(step[1])
    elif step[0] == "pick":
        st["held"] = step[1]
        del st["boxes"][step[1]]
    else:
        st["boxes"][step[1]] = tuple(step[2])
        st["held"] = None


def belt_cells(lv):
    """(cell, arrow) for every belt cell: entry up to the cell before the exit."""
    out = []
    for b in lv["belts"]:
        (ex, ey), (xx, xy) = b["entry"], b["exit"]
        dx, dy = (xx > ex) - (xx < ex), (xy > ey) - (xy < ey)
        arrow = {(1, 0): ">", (-1, 0): "<", (0, 1): "v", (0, -1): "^"}[(dx, dy)]
        out += [((ex + k * dx, ey + k * dy), arrow) for k in range(abs(xx - ex) + abs(xy - ey))]
    return out


def draw(screen, font, big, name, lv, st, status):
    screen.fill(FLOOR)
    rect = lambda c, pad=0: pygame.Rect(c[0] * CELL + pad, c[1] * CELL + pad, CELL - 2 * pad, CELL - 2 * pad)

    def label(text, c, color=TEXT, dy=0, f=font):
        img = f.render(text, True, color)
        screen.blit(img, img.get_rect(center=(c[0] * CELL + CELL // 2, c[1] * CELL + CELL // 2 + dy)))

    walls = {tuple(w) for w in lv["walls"]} | {(x, y) for x in range(10) for y in range(10) if x in (0, 9) or y in (0, 9)}
    door_open = any(list(p) in lv["plates"] for p in st["boxes"].values())
    for x in range(10):
        for y in range(10):
            pygame.draw.rect(screen, WALL if (x, y) in walls else FLOOR, rect((x, y)))
            pygame.draw.rect(screen, GRID, rect((x, y)), 1)
    for f in lv["fragile"]:
        pygame.draw.rect(screen, FRAGILE, rect(f["pos"], 2))
        label(f"max {f['max_weight']}", f["pos"], dy=18)
    for c, arrow in belt_cells(lv):
        pygame.draw.rect(screen, BELT, rect(c, 2))
        label(arrow * 3, c, f=big)
    for b in lv["belts"]:
        pygame.draw.rect(screen, BELT, rect(b["exit"], 4), 2)
        label("exit", b["exit"], dy=18)
    for p in lv["plates"]:
        pygame.draw.circle(screen, PLATE, rect(p).center, CELL // 3)
        label("plate", p, dy=18)
    for d in lv["doors"]:
        pygame.draw.rect(screen, DOOR, rect(d, 3), 3 if door_open else 0)
        label("door", d, color=TEXT if door_open else FLOOR, dy=-7)
        label("open" if door_open else "shut", d, color=TEXT if door_open else FLOOR, dy=7)
    for g in lv["goals"]:
        pygame.draw.rect(screen, GOAL, rect(g["pos"], 5), 3)
        label(g["id"], g["pos"], GOAL, dy=-18)
    weight = {b["id"]: b["weight"] for b in lv["boxes"]}
    target = {b["id"]: "free" if b.get("free") else b["goal"] for b in lv["boxes"]}
    for b, c in st["boxes"].items():
        pygame.draw.rect(screen, BOX, rect(c, 10))
        label(f"{b} w{weight[b]}", c)
        label(target[b] if target[b] == "free" else f"->{target[b]}" if target[b] else "any", c, dy=12)
    pygame.draw.circle(screen, ROBOT, rect(st["robot"]).center, CELL // 3)
    label("R", st["robot"], FLOOR, f=big)
    if st["held"]:
        label(f"holds {st['held']}", st["robot"], dy=20)

    x0 = 10 * CELL + 12
    lines = [name, "", *status, "", "left/right: level", "A / G / U: solve + play", "R: reset", "Esc: quit"]
    for i, text in enumerate(lines):
        screen.blit((big if i == 0 else font).render(text, True, TEXT), (x0, 14 + i * 22))


def main():
    pygame.init()
    screen = pygame.display.set_mode((10 * CELL + PANEL, 10 * CELL))
    pygame.display.set_caption("Warehouse robot - dev viewer")
    font, big = pygame.font.SysFont(None, 18), pygame.font.SysFont(None, 26)
    clock = pygame.time.Clock()
    idx, st, status, plan, pending, timer = 0, start(LEVELS[0][1]), ["press A to solve"], [], None, 0.0

    def reset(i):
        return i, start(LEVELS[i][1]), ["press A to solve"], [], None

    running = True
    while running:
        dt = clock.tick(60) / 1000
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT or (ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE):
                running = False
            elif ev.type == pygame.KEYDOWN and ev.key in (pygame.K_LEFT, pygame.K_RIGHT):
                idx, st, status, plan, pending = reset((idx + (1 if ev.key == pygame.K_RIGHT else -1)) % len(LEVELS))
            elif ev.type == pygame.KEYDOWN and ev.key == pygame.K_r:
                idx, st, status, plan, pending = reset(idx)
            elif ev.type == pygame.KEYDOWN and ev.key in (pygame.K_a, pygame.K_g, pygame.K_u) and pending is None:
                algo = {pygame.K_a: "astar", pygame.K_g: "greedy", pygame.K_u: "ucs"}[ev.key]
                st, plan = start(LEVELS[idx][1]), []
                pending, status = (algo, bridge.submit(bridge.solve, LEVELS[idx][1], algo)), [f"{algo}: solving..."]

        if pending and pending[1].done():
            algo, fut = pending
            pending = None
            try:
                res = fut.result()
            except ValueError as ex:  # malformed level, or too big to check in time
                status = [f"{algo}: {ex}"]
            else:
                if res is None:
                    status = [f"{algo}: no solution"]
                else:
                    plan, timer = list(res["plan"]), 0.0
                    status = [f"{algo}: cost {res['cost']}", f"nodes expanded {res['nodes']}",
                              f"{len(plan)} steps"]

        if plan:
            timer += dt
            while plan and timer >= 1 / STEPS_PER_SEC:
                timer -= 1 / STEPS_PER_SEC
                apply(st, plan.pop(0))
            if not plan:
                status = status + ["done"]

        draw(screen, font, big, LEVELS[idx][0], LEVELS[idx][1], st, status)
        pygame.display.flip()

    bridge.shutdown()
    pygame.quit()


if __name__ == "__main__":
    main()
