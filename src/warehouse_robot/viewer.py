"""Dev viewer: draws the test levels with basic shapes and plays solver plans. Not the real UI.

Run: uv run warehouse-viewer
Screens: home (LEVELS / CREATE) -> level selector -> play (EXIT / SOLVE / RESET), and the level creator
(see creator.py). SOLVE asks which algorithm to use: A*, UCS or greedy.
Keyboard shortcuts while playing: A/G/U = solve with A*/greedy/UCS, R = reset, Esc = exit. Esc on home quits."""
import math
from pathlib import Path

import pygame

from warehouse_robot import bridge, pixelfont

ASSETS = Path(__file__).resolve().parents[2] / "assets"

CELL, BAR = 56, 112  # board cell size; height of the button bar under the board
BOARD = 10 * CELL
PX = 3  # one "pixel" of the pixel-art UI, in screen pixels
MOVES = {"u": (0, -1), "d": (0, 1), "l": (-1, 0), "r": (1, 0)}
STEP_TIME, LIFT_TIME = 0.2, 0.22  # seconds per walked (or belt) cell, and to lift or place a crate
LOAD_SLOWDOWN = 0.35  # carrying a weight-w crate makes each walked cell take (1 + 0.35 w) x longer (animation only)
SINK, ARC = 6, 10  # px: robot sinks under a load; crate arcs when lifted/placed
DOOR_FRAMES, DOOR_FPS = 5, 15  # door sheet: frame 0 shut ... frame 4 open
BELT_PERIOD, BELT_FPS = 8, 20  # belt art repeats every 8 source px; scroll one source px per frame

FLOOR, FLOOR_ALT, WALL = (0x8F, 0xD2, 0xFF), (0x7F, 0xC4, 0xF4), (70, 70, 75)  # floor is a checkerboard
GOAL_EMPTY, GOAL_DONE, GOAL_WRONG = (205, 115, 0), (25, 125, 60), (200, 40, 40)
TEXT, BOX_TEXT, BOX_EDGE = (20, 20, 20), (255, 245, 215), (60, 35, 15)
FRAGILE_TEXT, FRAGILE_EDGE = (255, 255, 255), (40, 40, 45)
UI_BG, UI_EDGE, UI_TEXT, UI_DIM = (40, 38, 60), (20, 18, 35), (255, 255, 255), (170, 170, 200)
# button palettes: (face, highlight, shade)
BTN_SOLVE = ((70, 170, 80), (130, 215, 110), (35, 105, 50))
BTN_RESET = ((225, 145, 40), (255, 200, 95), (155, 85, 20))
BTN_NAV = ((85, 115, 200), (140, 170, 240), (45, 65, 135))
BTN_OFF = ((105, 105, 120), (140, 140, 155), (70, 70, 85))
ALGO_NAME = {"astar": "A*", "greedy": "GREEDY", "ucs": "UCS"}
ALGO_CHOICES = [("astar", "a*", "optimal, guided", BTN_SOLVE), ("ucs", "ucs", "optimal, slow", BTN_NAV),
                ("greedy", "greedy", "quick, may cost more", BTN_RESET)]  # the "solve with" picker, top to bottom
TITLE, TITLE_EDGE = (255, 205, 70), (20, 18, 35)
WIN_W, WIN_H = BOARD, BOARD + BAR


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


def outlined(screen, f, text, center, fill=BOX_TEXT, edge=BOX_EDGE):
    """Text with a 1 px outline, readable on busy sprite art (default: cream on dark brown, for crates)."""
    edge, fill = f.render(text, True, edge), f.render(text, True, fill)
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
    """How long a plan step takes to animate. Walking slows with the carried crate's weight (display only:
    the solver's costs are unchanged); a belt ride takes one STEP_TIME per cell it covers."""
    if isinstance(step, str):
        w = next(b["weight"] for b in lv["boxes"] if b["id"] == st["held"]) if st["held"] else 0
        return STEP_TIME * (1 + LOAD_SLOWDOWN * w)
    if step[0] == "ride":
        _, (ex, ey), (xx, xy) = ride_path(lv, st["robot"], step[1])
        return STEP_TIME * (abs(xx - ex) + abs(xy - ey) + 1)
    return LIFT_TIME


def start(lv):
    """Mutable play state for a level: robot cell, held box id, box id -> cell."""
    return {"robot": tuple(lv["robot"]) if lv["robot"] else None, "held": None, "boxes": {b["id"]: tuple(b["pos"]) for b in lv["boxes"]}}


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


def draw(screen, font, sprites, lv, st, door_frame, anim=None):
    """Draw the board in state st; anim = (step, t) shows step t of the way done (st is the state before it)."""
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
        corner = rect(f["pos"]).topright
        outlined(screen, font, str(f["max_weight"]), (corner[0] - 8, corner[1] + 9), FRAGILE_TEXT, FRAGILE_EDGE)
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
    if st["robot"] is None:  # a creator layout before the robot is placed
        return
    # robot: slides straight between cells and sinks under a load
    pos, sink, crate = cell_px(st["robot"]), SINK if st["held"] else 0, st["held"]
    if kind == "walk":
        dx, dy = MOVES[step]
        pos = lerp(pos, cell_px((st["robot"][0] + dx, st["robot"][1] + dy)), ease(t))
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


def notched(screen, r, color):
    """Filled rectangle with its corners cut by one UI pixel: the classic pixel-art button outline."""
    pygame.draw.rect(screen, color, (r.x + PX, r.y, r.w - 2 * PX, r.h))
    pygame.draw.rect(screen, color, (r.x, r.y + PX, r.w, r.h - 2 * PX))


def draw_button(screen, r, text, colors, hover=False, down=False, scale=2):
    """Pixel-art button in r: dark outline, bevel (light top-left, shade bottom-right) and a drop shadow.
    Pressed, it sinks onto its shadow; hovered, its face lightens."""
    face, light, shade = colors
    body = pygame.Rect(r.x, r.y + (PX if down else 0), r.w, r.h - PX)
    if not down:
        notched(screen, body.move(0, PX), UI_EDGE)
    notched(screen, body, UI_EDGE)
    inner = body.inflate(-2 * PX, -2 * PX)
    pygame.draw.rect(screen, light if hover and not down else face, inner)
    if not down:
        pygame.draw.rect(screen, light, (inner.x, inner.y, inner.w, PX))
        pygame.draw.rect(screen, light, (inner.x, inner.y, PX, inner.h))
    pygame.draw.rect(screen, shade, (inner.x, inner.bottom - PX, inner.w, PX))
    pygame.draw.rect(screen, shade, (inner.right - PX, inner.y, PX, inner.h))
    label = pixelfont.render(text, scale, UI_TEXT)
    at = label.get_rect(center=inner.center)
    screen.blit(pixelfont.render(text, scale, UI_EDGE), at.move(0, scale))
    screen.blit(label, at)


def shadow_text(screen, text, scale, color, center):
    """Centred pixel text with a hard drop shadow one font pixel down and right."""
    img = pixelfont.render(text, scale, color)
    at = img.get_rect(center=center)
    screen.blit(pixelfont.render(text, scale, TITLE_EDGE), at.move(scale, scale))
    screen.blit(img, at)


def backdrop():
    """Menu background: the warehouse floor checkerboard over the whole window, dimmed."""
    bg = pygame.Surface((WIN_W, WIN_H))
    for x in range(0, WIN_W, CELL):
        for y in range(0, WIN_H, CELL):
            bg.fill((FLOOR, FLOOR_ALT)[(x + y) // CELL % 2], (x, y, CELL, CELL))
    shade = pygame.Surface((WIN_W, WIN_H), pygame.SRCALPHA)
    shade.fill((*UI_BG, 205))
    bg.blit(shade, (0, 0))
    return bg


def centred(w, h, y):
    return pygame.Rect((WIN_W - w) // 2, y, w, h)


class Playback:
    """One level being solved and played: its play state, the solver call in flight and the plan being animated."""

    def __init__(self, name, lv):
        self.name, self.lv = name, lv
        self.restart()

    def restart(self):
        self.st, self.plan, self.pending, self.timer = start(self.lv), [], None, 0.0
        self.status = "press SOLVE"
        self.door_t = None  # animated door frame (float); None snaps to the current state

    def solve(self, algo="astar"):
        if self.pending is None:
            self.restart()
            self.pending = (algo, bridge.submit(bridge.solve, self.lv, algo))
            self.status = f"{ALGO_NAME[algo]}: solving..."

    def update(self, dt):
        """Collect a finished solve, advance the animation, ease the doors. Returns anim for draw()."""
        if self.pending and self.pending[1].done():
            algo, fut = self.pending
            self.pending = None
            try:
                res = fut.result()
            except ValueError as ex:  # malformed level, or too big to check in time
                self.status = f"{ALGO_NAME[algo]}: {ex}"
            else:
                if res is None:
                    self.status = f"{ALGO_NAME[algo]}: no solution"
                else:
                    self.plan, self.timer = list(res["plan"]), 0.0
                    self.status = (f"{ALGO_NAME[algo]}: cost {res['cost']} - {res['nodes']} nodes"
                                   f" - {len(self.plan)} steps")
        if self.plan:  # timer = seconds into the current step; a step is applied once its animation ends
            self.timer += dt
            while self.plan and self.timer >= step_time(self.lv, self.st, self.plan[0]):
                self.timer -= step_time(self.lv, self.st, self.plan[0])
                apply(self.st, self.plan.pop(0))
            if not self.plan:
                self.status += " - done"
        goal = DOOR_FRAMES - 1 if door_open(self.lv, self.st) else 0
        t = self.door_t
        self.door_t = goal if t is None else min(goal, t + DOOR_FPS * dt) if goal > t else max(goal, t - DOOR_FPS * dt)
        return (self.plan[0], self.timer / step_time(self.lv, self.st, self.plan[0])) if self.plan else None


class Viewer:
    """Which screen is showing (home / select / create / play), the level being played and the creator's layout."""

    def __init__(self, creator):
        self.screen, self.idx, self.creator = "home", 0, creator
        self.pb = Playback(*LEVELS[0])
        self.picker = None  # while the "solve with" picker is open: the function to call with the chosen algo

    def ask(self, solve):
        """Open the algorithm picker; solve(algo) runs once one is chosen."""
        self.picker = solve
        return True

    def pick(self, algo):
        solve, self.picker = self.picker, None
        if algo:
            solve(algo)

    def go(self, screen):
        self.screen = screen

    def play(self, i):
        self.idx, self.pb, self.screen = i, Playback(*LEVELS[i]), "play"


def buttons(vw):
    """(rect, label, palette, action, text scale) for every button on the current screen."""
    if vw.screen == "home":
        return [(centred(280, 64, 400), "levels", BTN_SOLVE, lambda: vw.go("select"), 3),
                (centred(280, 64, 482), "create", BTN_RESET, lambda: vw.go("create"), 3)]
    back = (centred(220, 52, WIN_H - 76), "< back", BTN_RESET, lambda: vw.go("home"), 2)
    if vw.screen == "select":
        return [(centred(420, 44, 92 + i * 52), f"{i + 1}. {name}", BTN_NAV, lambda i=i: vw.play(i), 2)
                for i, (name, _) in enumerate(LEVELS)] + [back]
    gap, margin, h = 10, 12, 48  # play: the bar under the board
    w = (BOARD - 2 * margin - 2 * gap) // 3
    row = [pygame.Rect(margin + i * (w + gap), BOARD + BAR - h - 12, w, h) for i in range(3)]
    return [(row[0], "exit", BTN_NAV, lambda: vw.go("select"), 2),
            (row[1], "solving" if vw.pb.pending else "solve", BTN_OFF if vw.pb.pending else BTN_SOLVE,
             lambda: vw.pb.pending or vw.ask(vw.pb.solve), 2),
            (row[2], "reset", BTN_RESET, vw.pb.restart, 2)]


def picker_layout():
    """The "solve with" panel, centred on the board: [(rect, label, description, palette, algo or None)]."""
    w, h, bw = 440, 18 + 40 + 3 * 58 + 10 + 44 + 20, 120
    panel = pygame.Rect((BOARD - w) // 2, (BOARD - h) // 2, w, h)
    x, y = panel.x + 24, panel.y + 58
    items = [(pygame.Rect(x, y + i * 58, bw, 48), label, about, colors, algo)
             for i, (algo, label, about, colors) in enumerate(ALGO_CHOICES)]
    items.append((pygame.Rect(x, panel.bottom - 64, w - 48, 44), "cancel", "", BTN_OFF, None))
    return panel, items


def picker_event(vw, ev):
    """The picker takes all mouse and key input while open (1/2/3 or A/U/G pick, Esc cancels).
    Returns True if ev was consumed."""
    if ev.type == pygame.KEYDOWN:
        keys = {pygame.K_1: "astar", pygame.K_a: "astar", pygame.K_2: "ucs", pygame.K_u: "ucs",
                pygame.K_3: "greedy", pygame.K_g: "greedy"}
        if ev.key in keys or ev.key == pygame.K_ESCAPE:
            vw.pick(keys.get(ev.key))
        return True
    if ev.type == pygame.MOUSEBUTTONUP and ev.button == 1:
        for r, _, _, _, algo in picker_layout()[1]:
            if r.collidepoint(ev.pos):
                vw.pick(algo)
                break
        return True
    return ev.type in (pygame.MOUSEBUTTONDOWN, pygame.MOUSEMOTION)


def draw_picker(screen, mouse):
    shade = pygame.Surface(screen.get_size(), pygame.SRCALPHA)
    shade.fill((10, 10, 20, 150))
    screen.blit(shade, (0, 0))
    panel, items = picker_layout()
    notched(screen, panel.move(0, PX), UI_EDGE)
    notched(screen, panel, UI_EDGE)
    pygame.draw.rect(screen, UI_BG, panel.inflate(-2 * PX, -2 * PX))
    screen.blit(pixelfont.render("solve with", 3, UI_EDGE), (panel.x + 27, panel.y + 21))
    screen.blit(pixelfont.render("solve with", 3, TITLE), (panel.x + 24, panel.y + 18))
    for r, label, about, colors, _ in items:
        draw_button(screen, r, label, colors, hover=r.collidepoint(mouse))
        screen.blit(pixelfont.render(about, 2, UI_DIM), (r.right + 16, r.y + 15))


def draw_menu(screen, art, vw):
    """Home and level selector: dimmed floor, a title, and (home) the robot lifting a crate."""
    screen.blit(art["backdrop"], (0, 0))
    if vw.screen == "home":
        shadow_text(screen, "warehouse", 7, TITLE, (WIN_W // 2, 64))
        shadow_text(screen, "robot", 7, TITLE, (WIN_W // 2, 128))
        shadow_text(screen, "pick - carry - solve", 2, UI_DIM, (WIN_W // 2, 182))
        bot = art["robot"].get_rect(midbottom=(WIN_W // 2, 380))
        screen.blit(art["robot"], bot)
        screen.blit(art["crate"], art["crate"].get_rect(midbottom=(bot.centerx, bot.top + 20)))
    else:
        shadow_text(screen, "select level", 4, TITLE, (WIN_W // 2, 46))


def draw_bar(screen, vw):
    """Bottom bar while playing: level name and solver status in pixel text (buttons are drawn separately)."""
    pygame.draw.rect(screen, UI_BG, (0, BOARD, BOARD, BAR))
    pygame.draw.rect(screen, UI_EDGE, (0, BOARD, BOARD, PX))
    screen.blit(pixelfont.render(f"level {vw.idx + 1}/{len(LEVELS)}: {vw.pb.name}", 2, UI_TEXT), (12, BOARD + 12))
    screen.blit(pixelfont.render(vw.pb.status, 2, UI_DIM), (12, BOARD + 32))


def main():
    from warehouse_robot import creator  # imported here: creator builds on this module's drawing helpers

    pygame.init()
    screen = pygame.display.set_mode((WIN_W, WIN_H))
    pygame.display.set_caption("Warehouse robot")
    font = pygame.font.SysFont(None, 18)
    sprites = {"robot": sprite("AI-sprite.png", CELL - 8), "crate": sprite("crate_crosstape_32.png", CELL - 4, square=True),
               "door": door_frames(CELL), "belt": belt_frames(CELL), "plate": sheet("box_plate_2f_32.png", 2, CELL),
               "fragile": sprite("fragile_floor_32.png", CELL, square=True),
               "shelf": sheet("shelf_4variants_32.png", 4, CELL)}
    art = {"backdrop": backdrop(), "robot": sprite("AI-sprite.png", 96),
           "crate": sprite("crate_crosstape_32.png", 84, square=True)}
    parts = creator.make_art(sprites)
    clock, vw = pygame.time.Clock(), Viewer(creator.Creator())
    cr = vw.creator
    held = None  # (screen, index) of the button the mouse went down on; it fires if released over it
    keys = {pygame.K_r: lambda: vw.pb.restart(), pygame.K_a: lambda: vw.pb.solve("astar"),
            pygame.K_g: lambda: vw.pb.solve("greedy"), pygame.K_u: lambda: vw.pb.solve("ucs")}

    def current_buttons():
        if vw.screen == "create":
            return creator.creator_buttons(cr, lambda: vw.go("home"), vw.ask)
        return buttons(vw)

    def on_board(pos):
        return pos[0] < BOARD and pos[1] < BOARD

    running = True
    while running:
        dt = clock.tick(60) / 1000
        size = (creator.WIDTH if vw.screen == "create" else WIN_W, WIN_H)  # the creator is wider: its sidebar
        if screen.get_size() != size:
            screen = pygame.display.set_mode(size)
        btns = current_buttons()
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False
            elif vw.picker and picker_event(vw, ev):
                pass  # the "solve with" picker takes all input while open
            elif vw.screen == "create" and cr.dialog and cr.dialog_event(ev):
                pass  # the creator's details dialog takes all input while open
            elif ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE:  # one screen back; quit from home
                if vw.screen == "home":
                    running = False
                elif not (vw.screen == "create" and cr.cancel()):
                    vw.go({"play": "select"}.get(vw.screen, "home"))
            elif ev.type == pygame.KEYDOWN and ev.key in keys and vw.screen == "play":
                keys[ev.key]()
            elif ev.type == pygame.MOUSEBUTTONDOWN and ev.button in (1, 3):
                hit = next((i for i, b in enumerate(btns) if b[0].collidepoint(ev.pos)), None)
                if hit is not None and ev.button == 1:
                    held = (vw.screen, hit)
                elif vw.screen == "create" and on_board(ev.pos):
                    cr.press((ev.pos[0] // CELL, ev.pos[1] // CELL), ev.button)
                elif vw.screen == "create" and ev.button == 1:
                    cr.click_sidebar(ev.pos)
            elif ev.type == pygame.MOUSEMOTION and vw.screen == "create" and on_board(ev.pos) and held is None:
                if ev.buttons[0] or ev.buttons[2]:  # drag to paint walls or erase
                    cr.press((ev.pos[0] // CELL, ev.pos[1] // CELL), 1 if ev.buttons[0] else 3, drag=True)
            elif ev.type == pygame.MOUSEBUTTONUP and ev.button == 1:
                if held and held[0] == vw.screen and btns[held[1]][0].collidepoint(ev.pos):
                    btns[held[1]][3]()
                held = None
            btns = current_buttons()  # an action may have switched screens

        mouse = pygame.mouse.get_pos()
        if vw.screen == "play":
            anim = vw.pb.update(dt)
            draw(screen, font, sprites, vw.pb.lv, vw.pb.st, round(vw.pb.door_t), anim)
            draw_bar(screen, vw)
        elif vw.screen == "create":
            cr.draw(screen, font, sprites, parts, dt, mouse)
        else:
            draw_menu(screen, art, vw)
        for i, (r, text, colors, _, scale) in enumerate(btns):
            over = r.collidepoint(mouse)
            draw_button(screen, r, text, colors, hover=over, down=over and held == (vw.screen, i), scale=scale)
        if vw.screen == "create" and cr.dialog:
            cr.draw_dialog(screen, mouse)
        if vw.picker:
            draw_picker(screen, mouse)
        pygame.display.flip()

    bridge.shutdown()
    pygame.quit()


if __name__ == "__main__":
    main()
