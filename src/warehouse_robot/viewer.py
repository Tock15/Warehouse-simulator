"""Dev viewer: draws the test levels with basic shapes and plays solver plans. Not the real UI.

Run: uv run warehouse-viewer
Keys: left/right = level, A/G/U = solve with A*/greedy/UCS and play, R = reset, Esc = quit."""
import math
from pathlib import Path

import pygame

from warehouse_robot import bridge

ASSETS = Path(__file__).resolve().parents[2] / "assets"

CELL, PANEL = 56, 260
MOVES = {"u": (0, -1), "d": (0, 1), "l": (-1, 0), "r": (1, 0)}
STEP_TIME, LIFT_TIME = 0.2, 0.22  # seconds per walked (or belt) cell, and to lift or place a crate
SINK, HOP, ARC = 6, 3, 10  # px: robot sinks under a load, bobs while walking; crate arcs when lifted/placed
DOOR_FRAMES, DOOR_FPS = 5, 15  # door sheet: frame 0 shut ... frame 4 open
BELT_PERIOD, BELT_FPS = 8, 20  # belt art repeats every 8 source px; scroll one source px per frame

FLOOR, FLOOR_ALT, WALL = (0x8F, 0xD2, 0xFF), (0x7F, 0xC4, 0xF4), (70, 70, 75)  # floor is a checkerboard
GOAL_EMPTY, GOAL_DONE, GOAL_WRONG = (205, 115, 0), (25, 125, 60), (200, 40, 40)
TEXT, BOX_TEXT, BOX_EDGE = (20, 20, 20), (255, 245, 215), (60, 35, 15)


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


SHELVES = [[x, y] for y in (3, 6) for x in (2, 3, 4, 6, 7)]
DOOR_ROOM = room(7, 4) + [[4, y] for y in (1, 3, 4)]
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
    ("two pinned", level(room(5, 5), [("g1", (1, 5)), ("g2", (5, 5))],
                         [("a", (5, 2), 1, "g1"), ("b", (1, 2), 2, "g2")], robot=(3, 1))),
    ("shelves, 3 boxes", level(SHELVES, [("g1", (8, 1)), ("g2", (8, 4)), ("g3", (8, 7))],
                               [("a", (3, 2), 1, None), ("b", (5, 5), 2, None), ("c", (2, 7), 3, None)])),
    ("shelves, 4 boxes", level(SHELVES, [("g1", (8, 1)), ("g2", (8, 4)), ("g3", (8, 7)), ("g4", (8, 8))],
                               [("a", (3, 2), 1, None), ("b", (5, 5), 2, "g1"), ("c", (2, 7), 3, None),
                                ("d", (6, 8), 1, "g4")])),
]


def scaled(img, wh):
    """Scale to wh. Small pixel art is blown up crisply first, so the final smoothscale only shrinks (no blur)."""
    if max(img.get_size()) < max(wh):
        img = pygame.transform.scale_by(img, -(-2 * max(wh) // max(img.get_size())))
    return pygame.transform.smoothscale(img, wh)


def sprite(name, size, square=False):
    """Image from assets/, scaled to fit a size x size square (aspect kept), or stretched to fill it."""
    img = pygame.image.load(ASSETS / name).convert_alpha()
    k = size / max(img.get_size())
    return scaled(img, (size, size) if square else (round(img.get_width() * k), round(img.get_height() * k)))


def sheet(name, n, size):
    """The n square frames of a horizontal sprite sheet, each scaled to size x size."""
    img = pygame.image.load(ASSETS / name).convert_alpha()
    w = img.get_width() // n
    return [scaled(img.subsurface((i * w, 0, w, img.get_height())), (size, size)) for i in range(n)]


def belt_frames(size):
    """Scrolling belt animation for each direction: {(dx, dy): frames}. The art runs left to right."""
    img = pygame.image.load(ASSETS / "conveyor_belt_32.png").convert_alpha()
    frames = []
    for k in range(BELT_PERIOD):
        f = pygame.Surface(img.get_size(), pygame.SRCALPHA)
        f.blit(img, (k, 0))
        f.blit(img, (k - img.get_width(), 0))
        frames.append(scaled(f, (size, size)))
    turn = {(1, 0): 0, (0, -1): 90, (-1, 0): 180, (0, 1): -90}
    return {d: [pygame.transform.rotate(f, a) for f in frames] for d, a in turn.items()}


def door_frames(size):
    """Door animation frames from the sheet: 5 frames of 320 x 320, art inside a 10 px margin."""
    sheet = pygame.image.load(ASSETS / "preview_x10.png").convert()
    return [pygame.transform.smoothscale(sheet.subsurface((i * 320 + 10, 10, 300, 300)), (size, size))
            for i in range(DOOR_FRAMES)]


def door_open(lv, st):
    """Doors are open while any box sits on any plate (the held box is on no plate)."""
    return any(list(p) in lv["plates"] for p in st["boxes"].values())


def outlined(screen, f, text, center):
    """Crate label: cream text with a dark brown outline, readable on the crate art."""
    edge, fill = f.render(text, True, BOX_EDGE), f.render(text, True, BOX_TEXT)
    r = fill.get_rect(center=center)
    for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (1, 1), (-1, 1), (1, -1)):
        screen.blit(edge, r.move(dx, dy))
    screen.blit(fill, r)


def goal_mark(screen, r, color):
    """Crate drop zone: tinted footprint, corner brackets, and an arrow pointing down onto a base line."""
    tint = pygame.Surface(r.size, pygame.SRCALPHA)
    tint.fill((*color, 55))
    screen.blit(tint, r)
    k, (x0, y0), (x1, y1) = r.w // 3, r.topleft, (r.right - 1, r.bottom - 1)
    for (x, y), (dx, dy) in (((x0, y0), (1, 1)), ((x1, y0), (-1, 1)), ((x0, y1), (1, -1)), ((x1, y1), (-1, -1))):
        pygame.draw.lines(screen, color, False, [(x + dx * k, y), (x, y), (x, y + dy * k)], 3)
    cx, cy = r.center
    pygame.draw.polygon(screen, color, [(cx - 7, cy - 6), (cx + 7, cy - 6), (cx, cy + 3)])
    pygame.draw.rect(screen, color, (cx - 3, cy - 14, 6, 8))
    pygame.draw.line(screen, color, (cx - 9, cy + 6), (cx + 9, cy + 6), 3)


def ease(t):
    """Smoothstep: slow start and end."""
    return t * t * (3 - 2 * t)


def lerp(a, b, t):
    return a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t


def cell_px(c):
    """Pixel centre of a cell."""
    return c[0] * CELL + CELL / 2, c[1] * CELL + CELL / 2


def ride_path(lv, robot, exit_):
    """Cells a belt ride passes through: where the robot stands, the belt entry next to it, the exit."""
    belt = next(b for b in lv["belts"] if tuple(b["exit"]) == tuple(exit_)
                and abs(b["entry"][0] - robot[0]) + abs(b["entry"][1] - robot[1]) == 1)
    return [robot, tuple(belt["entry"]), tuple(exit_)]


def along(cells, t):
    """Pixel point a fraction t of the way along a path of cells, at constant speed."""
    pts = [cell_px(c) for c in cells]
    legs = [abs(b[0] - a[0]) + abs(b[1] - a[1]) for a, b in zip(pts, pts[1:])]
    d = t * sum(legs)
    for a, b, n in zip(pts, pts[1:], legs):
        if d <= n:
            return lerp(a, b, d / n if n else 1)
        d -= n
    return pts[-1]


def step_time(lv, st, step):
    """How long a plan step takes to animate: a belt ride takes one STEP_TIME per cell it covers."""
    if isinstance(step, str):
        return STEP_TIME
    if step[0] == "ride":
        _, (ex, ey), (xx, xy) = ride_path(lv, st["robot"], step[1])
        return STEP_TIME * (abs(xx - ex) + abs(xy - ey) + 1)
    return LIFT_TIME


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
    """(cell, direction) for every belt cell: entry up to the cell before the exit."""
    out = []
    for b in lv["belts"]:
        (ex, ey), (xx, xy) = b["entry"], b["exit"]
        dx, dy = (xx > ex) - (xx < ex), (xy > ey) - (xy < ey)
        out += [((ex + k * dx, ey + k * dy), (dx, dy)) for k in range(abs(xx - ex) + abs(xy - ey))]
    return out


def draw(screen, font, big, sprites, name, lv, st, status, door_frame, anim=None):
    """Draw the board in state st; anim = (step, t) shows step t of the way done (st is the state before it)."""
    screen.fill(FLOOR)
    rect = lambda c, pad=0: pygame.Rect(c[0] * CELL + pad, c[1] * CELL + pad, CELL - 2 * pad, CELL - 2 * pad)

    def label(text, c, color=TEXT, dy=0, f=font):
        img = f.render(text, True, color)
        screen.blit(img, img.get_rect(center=(c[0] * CELL + CELL // 2, c[1] * CELL + CELL // 2 + dy)))

    border = {(x, y) for x in range(10) for y in range(10) if x in (0, 9) or y in (0, 9)}
    walls = {tuple(w) for w in lv["walls"]} | border
    # shelves: free-standing wall groups with floor all around; walls joined to the border stay plain
    plain, todo = set(border), list(border)
    while todo:
        x, y = todo.pop()
        for dx, dy in MOVES.values():
            n = (x + dx, y + dy)
            if n in walls and n not in plain:
                plain.add(n)
                todo.append(n)
    shelves = walls - plain
    for x in range(10):
        for y in range(10):
            if (x, y) in shelves:  # variant fixed by position, so the warehouse looks mixed but never flickers
                screen.blit(sprites["shelf"][(x * 3 + y * 5) % len(sprites["shelf"])], rect((x, y)))
            else:
                pygame.draw.rect(screen, WALL if (x, y) in walls else (FLOOR, FLOOR_ALT)[(x + y) % 2], rect((x, y)))
    for f in lv["fragile"]:
        screen.blit(sprites["fragile"], rect(f["pos"]))
        label(f"max {f['max_weight']}", f["pos"], dy=19)
    belt_frame = pygame.time.get_ticks() * BELT_FPS // 1000 % BELT_PERIOD
    for c, d in belt_cells(lv):
        screen.blit(sprites["belt"][d][belt_frame], rect(c))
    for p in lv["plates"]:  # pressed (frame 1) while a crate sits on it
        screen.blit(sprites["plate"][tuple(p) in st["boxes"].values()], rect(p))
    for d in lv["doors"]:
        screen.blit(sprites["door"][door_frame], rect(d))
    weight = {b["id"]: b["weight"] for b in lv["boxes"]}
    target = {b["id"]: b["goal"] for b in lv["boxes"]}
    on = {c: b for b, c in st["boxes"].items()}
    for g in lv["goals"]:
        b = on.get(tuple(g["pos"]))
        color = GOAL_EMPTY if b is None else GOAL_DONE if target[b] in (None, g["id"]) else GOAL_WRONG
        goal_mark(screen, rect(g["pos"], 1), color)
        label(g["id"], g["pos"], color, dy=16)
    step, t = anim or (None, 0.0)
    kind = None if step is None else "walk" if isinstance(step, str) else step[0]
    for b, c in st["boxes"].items():
        if kind == "pick" and b == step[1]:
            continue  # in the air, drawn with the robot
        screen.blit(sprites["crate"], sprites["crate"].get_rect(center=rect(c).center))
        cx, cy = rect(c).center
        outlined(screen, font, f"{b} w{weight[b]}", (cx, cy - 6))
        outlined(screen, font, f"->{target[b]}" if target[b] else "any", (cx, cy + 6))
    # robot: slides between cells (bobbing when walking, steady on a belt) and sinks under a load
    pos, sink, crate = cell_px(st["robot"]), SINK if st["held"] else 0, st["held"]
    if kind == "walk":
        dx, dy = MOVES[step]
        pos = lerp(pos, cell_px((st["robot"][0] + dx, st["robot"][1] + dy)), ease(t))
        sink -= HOP * math.sin(math.pi * t)
    elif kind == "ride":
        pos = along(ride_path(lv, st["robot"], step[1]), t)
    elif kind == "pick":
        sink, crate = SINK * ease(t), step[1]
    elif kind == "drop":
        sink = SINK * (1 - ease(t))
    bot = sprites["robot"].get_rect(center=(round(pos[0]), round(pos[1] + sink)))
    screen.blit(sprites["robot"], bot)
    if crate:  # crate on the robot's head, or travelling between the floor and its head
        head = (bot.centerx, bot.top + 12 - sprites["crate"].get_height() / 2)
        if kind == "pick":
            head = lerp(cell_px(st["boxes"][crate]), head, ease(t))
        elif kind == "drop":
            head = lerp(head, cell_px(step[2]), ease(t))
        if kind in ("pick", "drop"):
            head = (head[0], head[1] - ARC * math.sin(math.pi * t))
        top = sprites["crate"].get_rect(center=(round(head[0]), round(head[1])))
        screen.blit(sprites["crate"], top)
        outlined(screen, font, crate, top.center)

    x0 = 10 * CELL + 12
    lines = [name, "", *status, "", "left/right: level", "A / G / U: solve + play", "R: reset", "Esc: quit"]
    for i, text in enumerate(lines):
        screen.blit((big if i == 0 else font).render(text, True, TEXT), (x0, 14 + i * 22))


def main():
    pygame.init()
    screen = pygame.display.set_mode((10 * CELL + PANEL, 10 * CELL))
    pygame.display.set_caption("Warehouse robot - dev viewer")
    font, big = pygame.font.SysFont(None, 18), pygame.font.SysFont(None, 26)
    sprites = {"robot": sprite("AI-sprite.png", CELL - 8), "crate": sprite("crate_crosstape_32.png", CELL - 4, square=True),
               "door": door_frames(CELL), "belt": belt_frames(CELL), "plate": sheet("box_plate_2f_32.png", 2, CELL),
               "fragile": sprite("fragile_floor_32.png", CELL, square=True),
               "shelf": sheet("shelf_4variants_32.png", 4, CELL)}
    clock = pygame.time.Clock()
    idx, st, status, plan, pending, timer = 0, start(LEVELS[0][1]), ["press A to solve"], [], None, 0.0
    door_t = None  # animated door frame (float); None snaps to the current state on the next frame

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
                door_t = None
            elif ev.type == pygame.KEYDOWN and ev.key == pygame.K_r:
                idx, st, status, plan, pending = reset(idx)
                door_t = None
            elif ev.type == pygame.KEYDOWN and ev.key in (pygame.K_a, pygame.K_g, pygame.K_u) and pending is None:
                algo = {pygame.K_a: "astar", pygame.K_g: "greedy", pygame.K_u: "ucs"}[ev.key]
                st, plan, door_t = start(LEVELS[idx][1]), [], None
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

        lv = LEVELS[idx][1]
        if plan:  # timer = seconds into the current step; a step is applied once its animation ends
            timer += dt
            while plan and timer >= step_time(lv, st, plan[0]):
                timer -= step_time(lv, st, plan[0])
                apply(st, plan.pop(0))
            if not plan:
                status = status + ["done"]
        anim = (plan[0], timer / step_time(lv, st, plan[0])) if plan else None

        goal = DOOR_FRAMES - 1 if door_open(lv, st) else 0
        door_t = goal if door_t is None else (min(goal, door_t + DOOR_FPS * dt) if goal > door_t
                                              else max(goal, door_t - DOOR_FPS * dt))
        draw(screen, font, big, sprites, LEVELS[idx][0], lv, st, status, round(door_t), anim)
        pygame.display.flip()

    bridge.shutdown()
    pygame.quit()


if __name__ == "__main__":
    main()
