"""Level creator screen: an empty 10x10 room, a sidebar of parts to place, and SOLVE / RESET / EXIT.

Click a part, then click (or drag, for walls and erase) on the map. Right click erases.
- Crates and fragile floor open a details dialog once placed (weight, and the crate's goal); click one again
  with the same part to edit it.
- There is one robot: its part is disabled while it is on the map.
- Two-step parts: a belt is placed as start then end (same row or column), a door is followed by its plate.
Esc cancels a half-placed part or a dialog. SOLVE asks for an algorithm, solves the layout and plays the plan;
click the map to keep editing. Nothing can be edited while a solve is running.
SAVE / LOAD keep layouts as JSON files in levels/ (the bridge's level dict, plus a "name")."""
import json
import re

import pygame

from warehouse_robot import pixelfont
from warehouse_robot.viewer import (ASSETS, BAR, BOARD, BTN_NAV, BTN_OFF, BTN_RESET, BTN_SOLVE, CELL, DOOR_FRAMES, FLOOR,
                                    GOAL_EMPTY, PX, TITLE, UI_BG, UI_DIM, UI_EDGE, UI_TEXT, Playback, door_open,
                                    draw, draw_button, goal_mark, notched, start)

SIDE = 230  # sidebar width
WIDTH = BOARD + SIDE
MAX_CRATES = 4
TOOLS = [("wall", "wall"), ("erase", "erase"), ("robot", "robot"), ("crate", "crate"), ("goal", "goal"),
         ("fragile", "fragile"), ("belt", "belt"), ("door", "door")]
NEXT_STAGE = {"belt": ("belt_end", "belt end"), "door": ("plate", "plate")}  # second part a tool hands over
ROW_H, ROW_GAP, ROW_TOP = 38, 6, 48
TOOL_FACE, TOOL_PICKED, TOOL_LOCKED = (62, 60, 90), (255, 205, 70), (50, 48, 70)
HOVER, BELT_MARK, BAD = (255, 255, 255), (255, 205, 70), (230, 70, 70)
GHOST_ALPHA = 150
DIALOG_W, OPT_W, OPT_H, OPT_GAP = 400, 64, 40, 8
LEVEL_DIR = ASSETS.parent / "levels"  # saved custom levels
NAME_CHARS, NAME_MAX, LOAD_PAGE = "abcdefghijklmnopqrstuvwxyz0123456789 -_", 18, 6
FIELD, WARN = (20, 18, 35), (240, 110, 90)


def slug(name):
    """File name for a level name: lowercase letters and digits joined by underscores."""
    return re.sub(r"[^a-z0-9]+", "_", name.lower()).strip("_") or "level"


def saved_levels():
    """[(name, path)] of the saved levels, by name. A file that can't be read is listed under its file name."""
    out = []
    for path in sorted(LEVEL_DIR.glob("*.json")):
        try:
            name = json.loads(path.read_text()).get("name") or path.stem
        except (OSError, ValueError, AttributeError):
            name = path.stem
        out.append((str(name), path))
    return sorted(out)


def inner(c):
    """Cells that can be edited: everything inside the border wall."""
    return 1 <= c[0] <= 8 and 1 <= c[1] <= 8


def belt_path(entry, exit_):
    """Belt cells from the entry up to the cell before the exit (the exit is ordinary floor)."""
    dx, dy = (exit_[0] > entry[0]) - (exit_[0] < entry[0]), (exit_[1] > entry[1]) - (exit_[1] < entry[1])
    n = abs(exit_[0] - entry[0]) + abs(exit_[1] - entry[1])
    return [(entry[0] + k * dx, entry[1] + k * dy) for k in range(n)]


def cell_rect(c):
    return pygame.Rect(c[0] * CELL, c[1] * CELL, CELL, CELL)


def make_art(sprites):
    """For every part: a cell-sized ghost (shown under the mouse, same size as when placed) and a 32 px icon."""
    def tile(*layers, floor=False):
        s = pygame.Surface((CELL, CELL), pygame.SRCALPHA)
        if floor:
            s.fill(FLOOR)
        for layer in layers:
            layer(s) if callable(layer) else s.blit(layer, layer.get_rect(center=(CELL // 2, CELL // 2)))
        return s

    cross = lambda s: (pygame.draw.line(s, BAD, (12, 12), (CELL - 13, CELL - 13), 7),
                       pygame.draw.line(s, BAD, (CELL - 13, 12), (12, CELL - 13), 7))
    goal = lambda s: goal_mark(s, pygame.Rect(1, 1, CELL - 2, CELL - 2), GOAL_EMPTY)
    belt = sprites["belt"][(1, 0)][0]
    ghost = {"wall": tile(sprites["shelf"][0]), "erase": tile(cross), "robot": tile(sprites["robot"]),
             "crate": tile(sprites["crate"]), "goal": tile(goal), "fragile": tile(sprites["fragile"]),
             "belt": tile(belt), "belt_end": tile(belt), "door": tile(sprites["door"][0]),
             "plate": tile(sprites["plate"][0])}
    icon = {k: pygame.transform.smoothscale(g, (32, 32)) for k, g in ghost.items()}
    icon["erase"] = pygame.transform.smoothscale(tile(cross, floor=True), (32, 32))
    icon["goal"] = pygame.transform.smoothscale(tile(goal, floor=True), (32, 32))
    for g in ghost.values():
        g.set_alpha(GHOST_ALPHA)
    return {"ghost": ghost, "icon": icon}


class Creator:
    """The layout being built, the part in hand, an open details dialog, and (after SOLVE) the plan playback."""

    def __init__(self):
        self.tool = "wall"
        self.reset()

    def reset(self):
        self.walls, self.plates, self.doors = set(), set(), set()
        self.goals, self.belts = [], []          # goal cells in placement order; belts as (entry, exit)
        self.fragile = {}                        # cell -> max weight it holds
        self.crates = {}                         # cell -> (weight, goal cell it must finish on, or None = any)
        self.robot = None
        self.name = "my level"                   # used when saving
        self.stage = None                        # second half of a two-step part: (tool, first cell)
        self.dialog = None                       # details being edited: {"kind", "cell", "new", "weight", "goal"}
        self.files = None                        # save / load dialog: {"mode": "save", "name"} or {"mode": "load", "page"}
        self.pb = None                           # Playback while a solve runs or its plan plays
        self.last = None                         # cell the last click / drag acted on
        self.status = "pick a part, then click the map"
        if self.tool == "robot" or self.tool is None:
            self.tool = "wall"

    # ---- layout ----

    def belt_cells(self):
        return {c for e, x in self.belts for c in belt_path(e, x)}

    def blocked(self, c):
        """Robot and crates can't start on walls, doors or belts."""
        return c in self.walls or c in self.doors or c in self.belt_cells()

    def clear_tile(self, c):
        """Remove whatever floor part is on c (a belt goes entirely)."""
        self.walls.discard(c)
        self.plates.discard(c)
        self.doors.discard(c)
        self.fragile.pop(c, None)
        if c in self.goals:
            self.goals.remove(c)
        self.belts = [b for b in self.belts if c not in belt_path(*b)]

    def clear_things(self, c):
        self.crates.pop(c, None)
        if self.robot == c:
            self.robot = None

    def goal_ids(self):
        return {c: f"g{i + 1}" for i, c in enumerate(self.goals)}

    def level(self):
        """The layout as a bridge level dict. Crates are named a, b, c, d; a crate pinned to a goal that has
        since been erased may finish on any goal."""
        ids = self.goal_ids()
        return {"walls": [list(c) for c in sorted(self.walls)],
                "goals": [{"id": ids[c], "pos": list(c)} for c in self.goals],
                "boxes": [{"id": "abcd"[i], "pos": list(c), "weight": w, "goal": ids.get(g)}
                          for i, (c, (w, g)) in enumerate(self.crates.items())],
                "robot": list(self.robot) if self.robot else None,
                "fragile": [{"pos": list(c), "max_weight": m} for c, m in self.fragile.items()],
                "belts": [{"entry": list(e), "exit": list(x)} for e, x in self.belts],
                "plates": [list(c) for c in sorted(self.plates)], "doors": [list(c) for c in sorted(self.doors)]}

    def from_level(self, lv):
        """Replace the layout with a level dict (as saved by save_level, or any bridge level)."""
        self.reset()
        ids = {g["id"]: tuple(g["pos"]) for g in lv["goals"]}
        self.walls = {tuple(c) for c in lv["walls"] if inner(tuple(c))}
        self.goals = [tuple(g["pos"]) for g in lv["goals"]]
        self.crates = {tuple(b["pos"]): (int(b["weight"]), ids.get(b.get("goal"))) for b in lv["boxes"][:MAX_CRATES]}
        self.robot = tuple(lv["robot"]) if lv.get("robot") else None
        self.fragile = {tuple(f["pos"]): int(f["max_weight"]) for f in lv.get("fragile", [])}
        self.belts = [(tuple(b["entry"]), tuple(b["exit"])) for b in lv.get("belts", [])]
        self.plates = {tuple(c) for c in lv.get("plates", [])}
        self.doors = {tuple(c) for c in lv.get("doors", [])}
        if self.robot and self.tool == "robot":
            self.tool = None

    def save_level(self, name):
        name = name.strip()
        if not name:
            self.status = "type a name first"
            return
        LEVEL_DIR.mkdir(exist_ok=True)
        path = LEVEL_DIR / f"{slug(name)}.json"
        replaced = path.exists()
        path.write_text(json.dumps({"name": name, **self.level()}, indent=1))
        self.name, self.files = name, None
        self.status = f"saved as levels/{path.name}" + (" (replaced)" if replaced else "")

    def load_level(self, name, path):
        try:
            self.from_level(json.loads(path.read_text()))
        except (OSError, ValueError, KeyError, TypeError, IndexError, AttributeError):
            self.status = f"couldn't read levels/{path.name}"
        else:
            self.name, self.status = name, f"loaded {name}"
        self.files = None

    # ---- editing ----

    def busy(self):
        """A solve is running: SOLVE, RESET, SAVE, LOAD and all editing wait for it."""
        return bool(self.pb and self.pb.pending)

    def holding(self):
        return self.stage[0] if self.stage else self.tool

    def available(self, tool):
        """Whether a sidebar part can be picked: one robot only, and nothing else mid-way through a two-step part."""
        if self.stage:
            return tool == self.tool
        return not (tool == "robot" and self.robot is not None)

    def pick(self, tool):
        if self.busy():
            return
        if self.stage:
            self.status = f"place the {NEXT_STAGE[self.tool][1]} first (esc cancels)"
            return
        if not self.available(tool):
            self.status = "there is already a robot - erase it to place it again"
            return
        self.tool, self.pb = tool, None
        self.status = {"belt": "click where the belt starts", "door": "click where the door goes",
                       "crate": "click to place a crate (click one to edit it)",
                       "fragile": "click to place fragile floor (click one to edit it)"}.get(
            tool, "click the map to place it" + (" (drag to paint)" if tool in ("wall", "erase") else ""))

    def cancel(self):
        """Esc: close a dialog or drop a half-placed part. Returns False if there was nothing to cancel."""
        if self.dialog:
            self.close_dialog(keep=False)
            return True
        if not self.stage:
            return False
        kind, first = self.stage
        if kind == "plate":
            self.doors.discard(first)
        self.stage, self.status = None, "cancelled"
        return True

    def place(self, c, tool):
        """Apply a part to cell c. Returns a message when the part can't go there."""
        if not inner(c):
            return "the outer wall can't be changed"
        if tool == "wall":
            self.clear_tile(c)
            self.clear_things(c)
            self.walls.add(c)
        elif tool == "erase":
            if c in self.crates or self.robot == c:
                self.clear_things(c)
            else:
                self.clear_tile(c)
        elif tool == "robot":
            if self.robot is not None:
                return "there is already a robot - erase it to place it again"
            if self.blocked(c):
                return "the robot must start on floor"
            self.crates.pop(c, None)
            self.robot, self.tool, self.status = c, None, "robot placed - pick the next part"
        elif tool == "crate":
            if c in self.crates:  # edit an existing crate
                self.open_dialog("crate", c, new=False)
                return None
            if self.blocked(c):
                return "crates must start on floor"
            if len(self.crates) >= MAX_CRATES:
                return f"at most {MAX_CRATES} crates"
            if self.robot == c:
                self.robot = None
            self.crates[c] = (1, None)
            self.open_dialog("crate", c, new=True)
        elif tool == "goal":  # floor parts: one per cell, replacing what was there
            self.clear_tile(c)
            self.goals.append(c)
        elif tool == "fragile":
            new = c not in self.fragile
            if new:
                self.clear_tile(c)
                self.fragile[c] = 1
            self.open_dialog("fragile", c, new=new)
        elif tool == "plate":
            if c == self.stage[1]:
                return "the plate can't go on its own door"
            self.clear_tile(c)
            self.plates.add(c)
            self.stage, self.status = None, "door and plate placed"
        elif tool == "door":
            self.clear_tile(c)
            self.clear_things(c)
            self.doors.add(c)
            self.stage, self.status = ("plate", c), "now place its pressure plate (esc cancels)"
        elif tool == "belt":
            self.stage, self.status = ("belt_end", c), "click where the belt ends (same row or column)"
        elif tool == "belt_end":
            first = self.stage[1]
            if c == first or (c[0] != first[0] and c[1] != first[1]):
                return "the belt must run straight along a row or column"
            if c in self.walls or c in self.doors:
                return "a belt can't end on a wall or door"
            if c in self.belt_cells():  # the exit can't be another belt's cell
                self.clear_tile(c)
            for p in belt_path(first, c):  # the belt replaces whatever it runs over
                self.clear_tile(p)
                self.clear_things(p)
            self.belts.append((first, c))
            self.stage, self.status = None, "belt placed"
        return None

    def press(self, c, button, drag=False):
        """Mouse on the map: left places the part in hand, right erases. Drags only paint walls / erase."""
        if self.busy():
            return
        if self.pb:  # leave the solution playback and go back to editing
            self.pb = None
            self.status = "editing"
            return
        tool = "erase" if button == 3 and not self.stage else self.holding()
        if drag and (tool not in ("wall", "erase") or c == self.last):
            return  # a drag acts once per new cell (erasing twice would also take the floor under a crate)
        self.last = c
        if tool is None:
            self.status = "pick a part from the sidebar"
            return
        msg = self.place(c, tool)
        if msg:
            self.status = msg

    def ready(self):
        """Whether SOLVE may go ahead (then the algorithm is asked for); says why not in the status line."""
        if self.busy():
            return False
        self.cancel()
        if self.robot is None:
            self.status = "place the robot first"
            return False
        return True

    def solve(self, algo="astar"):
        self.pb = Playback("custom level", self.level())
        self.pb.solve(algo)

    # ---- details dialog (crate weight and goal, fragile floor limit) ----

    def open_dialog(self, kind, c, new):
        w, g = self.crates[c] if kind == "crate" else (self.fragile[c], None)
        self.dialog = {"kind": kind, "cell": c, "new": new, "weight": w, "goal": g, "was": (w, g)}
        self.status = "choose its details"

    def choose(self, key, value):
        """A dialog choice, shown on the map at once (cancel puts the old value back)."""
        d = self.dialog
        d[key] = value
        if d["kind"] == "crate":
            self.crates[d["cell"]] = (d["weight"], d["goal"])
        else:
            self.fragile[d["cell"]] = d["weight"]

    def close_dialog(self, keep):
        """OK keeps the choices; cancel drops a just-placed part, or puts an existing one back as it was."""
        d, self.dialog = self.dialog, None
        c, store = d["cell"], self.crates if d["kind"] == "crate" else self.fragile
        if not keep and d["new"]:
            store.pop(c, None)
        elif not keep:
            store[c] = d["was"] if d["kind"] == "crate" else d["was"][0]
        self.status = (f"{d['kind']} {'placed' if d['new'] else 'updated'}" if keep
                       else "cancelled" if d["new"] else "no change")

    def dialog_layout(self):
        """Panel rect, its text [(text, scale, color, (x, y))] and controls [(rect, label, chosen?, action)].
        The panel sits in the half of the board away from the cell being set up, so that cell stays visible."""
        d = self.dialog
        crate = d["kind"] == "crate"
        goals = [(None, "any")] + [(c, i.upper()) for c, i in self.goal_ids().items()] if crate else []
        per_row = 5
        goal_h = -(-len(goals) // per_row) * (OPT_H + OPT_GAP)
        h = 18 + 36 + 24 + OPT_H + 16 + (24 + goal_h if crate else 0) + 44 + 14
        top = 8 if d["cell"][1] >= 5 else BOARD - 8 - h
        panel = pygame.Rect((BOARD - DIALOG_W) // 2, top, DIALOG_W, h)
        x0, y = panel.x + 24, panel.y + 18
        text = [("crate" if crate else "fragile floor", 3, TITLE, (x0, y))]
        items = []

        def choose(key, value):
            return lambda: self.choose(key, value)

        y += 36
        text.append(("weight" if crate else "holds crates up to weight", 2, UI_DIM, (x0, y)))
        y += 24
        for i, w in enumerate((1, 2, 3) if crate else (1, 2)):
            items.append((pygame.Rect(x0 + i * (OPT_W + OPT_GAP), y, OPT_W, OPT_H), str(w), d["weight"] == w,
                          choose("weight", w)))
        y += OPT_H + 16
        if crate:
            text.append(("must finish on", 2, UI_DIM, (x0, y)))
            y += 24
            for i, (g, name) in enumerate(goals):
                r = pygame.Rect(x0 + i % per_row * (OPT_W + OPT_GAP), y + i // per_row * (OPT_H + OPT_GAP),
                                OPT_W, OPT_H)
                items.append((r, name, d["goal"] == g, choose("goal", g)))
            y += goal_h
        bw = (DIALOG_W - 48 - 12) // 2
        items.append((pygame.Rect(x0, y, bw, 44), "ok", None, lambda: self.close_dialog(True)))
        items.append((pygame.Rect(x0 + bw + 12, y, bw, 44), "cancel", None, lambda: self.close_dialog(False)))
        return panel, text, items

    def dialog_event(self, ev):
        """While the dialog is open it takes all mouse and key input. Returns True if ev was consumed."""
        if ev.type == pygame.KEYDOWN:
            if ev.key in (pygame.K_RETURN, pygame.K_KP_ENTER):
                self.close_dialog(True)
            elif ev.key == pygame.K_ESCAPE:
                self.close_dialog(False)
            return True
        if ev.type == pygame.MOUSEBUTTONUP and ev.button == 1:
            for r, _, _, action in self.dialog_layout()[2]:
                if r.collidepoint(ev.pos):
                    action()
                    break
            return True
        return ev.type in (pygame.MOUSEBUTTONDOWN, pygame.MOUSEMOTION)

    # ---- modal dialogs: part details, or save / load ----

    def modal(self):
        return self.dialog is not None or self.files is not None

    def modal_event(self, ev):
        """While a dialog is open it takes all mouse and key input. Returns True if ev was consumed."""
        return self.files_event(ev) if self.files else self.dialog_event(ev)

    def draw_modal(self, screen, mouse):
        self.draw_files(screen, mouse) if self.files else self.draw_dialog(screen, mouse)

    def open_files(self, mode):
        if not self.busy():
            self.cancel()
            self.pb = None
            self.files = {"mode": "save", "name": self.name} if mode == "save" else {"mode": "load", "page": 0}

    def files_layout(self):
        """Panel, text [(words, scale, color, (x, y))], buttons [(rect, label, palette, action)], and the name
        field rect (save only)."""
        f, w = self.files, DIALOG_W
        x0, field = (BOARD - w) // 2 + 24, None
        bw = (w - 48 - 12) // 2
        if f["mode"] == "save":
            h = 18 + 36 + 24 + 44 + 20 + 44 + 20
            panel = pygame.Rect((BOARD - w) // 2, (BOARD - h) // 2, w, h)
            text = [("save level", 3, TITLE, (x0, panel.y + 18)), ("name", 2, UI_DIM, (x0, panel.y + 54))]
            field = pygame.Rect(x0, panel.y + 78, w - 48, 44)
            y = field.bottom + 20
            items = [(pygame.Rect(x0, y, bw, 44), "save", BTN_SOLVE, lambda: self.save_level(f["name"])),
                     (pygame.Rect(x0 + bw + 12, y, bw, 44), "cancel", BTN_RESET, self.close_files)]
            return panel, text, items, field
        levels = saved_levels()
        pages = max(1, -(-len(levels) // LOAD_PAGE))
        f["page"] = min(f["page"], pages - 1)
        shown = levels[f["page"] * LOAD_PAGE:(f["page"] + 1) * LOAD_PAGE]
        h = 18 + 40 + max(1, len(shown)) * 48 + (48 if pages > 1 else 0) + 8 + 44 + 20
        panel = pygame.Rect((BOARD - w) // 2, (BOARD - h) // 2, w, h)
        text = [("load level", 3, TITLE, (x0, panel.y + 18))]
        y = panel.y + 58
        if not shown:
            text.append(("no saved levels yet", 2, UI_DIM, (x0, y + 14)))
        items = [(pygame.Rect(x0, y + i * 48, w - 48, 40), name, BTN_NAV, lambda n=name, p=path: self.load_level(n, p))
                 for i, (name, path) in enumerate(shown)]
        y += max(1, len(shown)) * 48
        if pages > 1:
            turn = lambda d: lambda: f.__setitem__("page", (f["page"] + d) % pages)
            items += [(pygame.Rect(x0, y, bw, 40), "< page", BTN_OFF, turn(-1)),
                      (pygame.Rect(x0 + bw + 12, y, bw, 40), "page >", BTN_OFF, turn(1))]
            text.append((f"{f['page'] + 1}/{pages}", 2, UI_DIM, (panel.right - 70, panel.y + 24)))
            y += 48
        items.append((pygame.Rect(x0, y + 8, w - 48, 44), "cancel", BTN_RESET, self.close_files))
        return panel, text, items, field

    def close_files(self):
        self.files, self.status = None, "editing"

    def files_event(self, ev):
        f = self.files
        if ev.type == pygame.KEYDOWN:
            if ev.key == pygame.K_ESCAPE:
                self.close_files()
            elif f["mode"] == "save" and ev.key in (pygame.K_RETURN, pygame.K_KP_ENTER):
                self.save_level(f["name"])
            elif f["mode"] == "save" and ev.key == pygame.K_BACKSPACE:
                f["name"] = f["name"][:-1]
            elif f["mode"] == "save" and ev.unicode and ev.unicode.lower() in NAME_CHARS and len(f["name"]) < NAME_MAX:
                f["name"] += ev.unicode.lower()
            return True
        if ev.type == pygame.MOUSEBUTTONUP and ev.button == 1:
            for r, _, _, action in self.files_layout()[2]:
                if r.collidepoint(ev.pos):
                    action()
                    break
            return True
        return ev.type in (pygame.MOUSEBUTTONDOWN, pygame.MOUSEMOTION, pygame.TEXTINPUT)

    def draw_files(self, screen, mouse):
        panel, text, items, field = self.files_layout()
        shade_and_panel(screen, panel)
        for words, scale, color, (x, y) in text:
            screen.blit(pixelfont.render(words, scale, UI_EDGE), (x + scale, y + scale))
            screen.blit(pixelfont.render(words, scale, color), (x, y))
        if field:
            notched(screen, field, UI_EDGE)
            pygame.draw.rect(screen, FIELD, field.inflate(-2 * PX, -2 * PX))
            caret = "_" if pygame.time.get_ticks() // 500 % 2 else ""
            screen.blit(pixelfont.render(self.files["name"] + caret, 2, UI_TEXT), (field.x + 12, field.y + 15))
        for r, label, colors, _ in items:
            draw_button(screen, r, label, colors, hover=r.collidepoint(mouse))

    def draw_dialog(self, screen, mouse):
        d = self.dialog
        shade = pygame.Surface((WIDTH, BOARD + BAR), pygame.SRCALPHA)
        shade.fill((10, 10, 20, 150))
        screen.blit(shade, (0, 0))
        pygame.draw.rect(screen, TITLE, cell_rect(d["cell"]), PX)  # the part being set up
        if d["goal"] is not None:
            pygame.draw.rect(screen, (90, 230, 120), cell_rect(d["goal"]), PX)  # its chosen goal
        panel, text, items = self.dialog_layout()
        notched(screen, panel.move(0, PX), UI_EDGE)
        notched(screen, panel, UI_EDGE)
        pygame.draw.rect(screen, UI_BG, panel.inflate(-2 * PX, -2 * PX))
        for words, scale, color, (x, y) in text:
            screen.blit(pixelfont.render(words, scale, UI_EDGE), (x + scale, y + scale))
            screen.blit(pixelfont.render(words, scale, color), (x, y))
        for r, text, chosen, _ in items:
            colors = BTN_SOLVE if chosen or text == "ok" else BTN_RESET if text == "cancel" else BTN_NAV
            draw_button(screen, r, text, colors, hover=r.collidepoint(mouse) and not chosen, down=bool(chosen))

    # ---- drawing ----

    def counter_lines(self):
        """The live counter under the parts: crates placed, goals placed, and a warning when goals run short."""
        crates, goals = len(self.crates), len(self.goals)
        lines = [(f"crates {crates}/{MAX_CRATES}", UI_TEXT), (f"goals  {goals}", UI_TEXT)]
        if goals < crates:
            short = crates - goals
            lines.append((f"need {short} more goal{'s' if short > 1 else ''}", WARN))
        return lines

    def rows(self):
        return [pygame.Rect(BOARD + 12, ROW_TOP + i * (ROW_H + ROW_GAP), SIDE - 24, ROW_H) for i in range(len(TOOLS))]

    def click_sidebar(self, pos):
        for r, (tool, _) in zip(self.rows(), TOOLS):
            if r.collidepoint(pos):
                self.pick(tool)
                return True
        return False

    def draw(self, screen, font, sprites, art, dt, mouse):
        """Board (layout or solution playback), sidebar of parts and status bar. The caller draws the bottom
        buttons, then draw_dialog() on top of everything when a dialog is open."""
        title = f"level creator - {self.name}"
        if self.pb:
            anim = self.pb.update(dt)
            draw(screen, font, sprites, self.pb.lv, self.pb.st, round(self.pb.door_t), anim)
            status = self.pb.status
            title = "level creator - click the map to edit" if not self.pb.pending and not self.pb.plan else title
            if self.pb.pending:
                title = "level creator - solving, please wait"
        else:
            lv = self.level()
            st = start(lv)
            draw(screen, font, sprites, lv, st, DOOR_FRAMES - 1 if door_open(lv, st) else 0)
            if not self.modal():
                self.draw_marks(screen, art, mouse)
            status = self.status
        self.draw_sidebar(screen, art, mouse)
        pygame.draw.rect(screen, UI_BG, (0, BOARD, WIDTH, BAR))
        pygame.draw.rect(screen, UI_EDGE, (0, BOARD, WIDTH, PX))
        screen.blit(pixelfont.render(title, 2, UI_TEXT), (12, BOARD + 12))
        screen.blit(pixelfont.render(status, 2, UI_DIM), (12, BOARD + 32))

    def draw_marks(self, screen, art, mouse):
        """Editing overlays: the belt start and its preview, and a see-through copy of the part under the mouse."""
        hover = (mouse[0] // CELL, mouse[1] // CELL) if mouse[0] < BOARD and mouse[1] < BOARD else None
        if self.stage and self.stage[0] == "belt_end":
            first = self.stage[1]
            if hover and hover != first and (hover[0] == first[0] or hover[1] == first[1]):
                tint = pygame.Surface((CELL, CELL), pygame.SRCALPHA)
                tint.fill((*BELT_MARK, 90))
                for p in belt_path(first, hover):
                    screen.blit(tint, cell_rect(p))
            pygame.draw.rect(screen, BELT_MARK, cell_rect(first), PX)
        if hover and inner(hover):
            if self.holding():
                screen.blit(art["ghost"][self.holding()], cell_rect(hover))
            pygame.draw.rect(screen, HOVER, cell_rect(hover), 2)

    def draw_sidebar(self, screen, art, mouse):
        pygame.draw.rect(screen, UI_BG, (BOARD, 0, SIDE, BOARD))
        pygame.draw.rect(screen, UI_EDGE, (BOARD, 0, PX, BOARD))
        screen.blit(pixelfont.render("parts", 3, UI_EDGE), (BOARD + 15, 17))
        screen.blit(pixelfont.render("parts", 3, UI_TEXT), (BOARD + 12, 14))
        for r, (tool, name) in zip(self.rows(), TOOLS):
            picked = tool == self.tool
            locked = not self.available(tool) or (self.busy() and not picked)
            if picked and self.stage:
                tool, name = NEXT_STAGE[tool]
            face = TOOL_PICKED if picked else TOOL_LOCKED if locked else TOOL_FACE
            if r.collidepoint(mouse) and not picked and not locked and not self.modal():
                face = tuple(min(255, v + 25) for v in face)
            notched(screen, r, UI_EDGE)
            pygame.draw.rect(screen, face, r.inflate(-2 * PX, -2 * PX))
            icon = art["icon"][tool].copy()
            if locked:
                icon.set_alpha(90)
            screen.blit(icon, (r.x + 5, r.y + 3))
            color = UI_EDGE if picked else UI_DIM if locked else UI_TEXT
            screen.blit(pixelfont.render(name, 2, color), (r.x + 46, r.y + 12))
            if tool == "robot" and locked and not self.stage:
                screen.blit(pixelfont.render("placed", 1, UI_DIM), (r.right - 44, r.y + 16))
        y = ROW_TOP + len(TOOLS) * (ROW_H + ROW_GAP) + 6
        pygame.draw.rect(screen, UI_EDGE, (BOARD + 12, y, SIDE - 24, PX))
        for i, (words, color) in enumerate(self.counter_lines()):
            screen.blit(pixelfont.render(words, 2, color), (BOARD + 16, y + 16 + i * 24))


def shade_and_panel(screen, panel):
    """Dim the whole window, then draw an empty pixel panel."""
    shade = pygame.Surface(screen.get_size(), pygame.SRCALPHA)
    shade.fill((10, 10, 20, 150))
    screen.blit(shade, (0, 0))
    notched(screen, panel.move(0, PX), UI_EDGE)
    notched(screen, panel, UI_EDGE)
    pygame.draw.rect(screen, UI_BG, panel.inflate(-2 * PX, -2 * PX))


def creator_buttons(cr, exit_, ask):
    """(rect, label, palette, action, text scale) for the creator's bottom bar. ask(fn) asks for the algorithm
    and calls fn(algo). While a solve runs, everything but EXIT is greyed out and does nothing."""
    gap, margin, h = 10, 12, 48
    w = (WIDTH - 2 * margin - 4 * gap) // 5
    row = [pygame.Rect(margin + i * (w + gap), BOARD + BAR - h - 12, w, h) for i in range(5)]
    busy = cr.busy()
    idle = lambda palette: BTN_OFF if busy else palette
    return [(row[0], "solving" if busy else "solve", idle(BTN_SOLVE), lambda: cr.ready() and ask(cr.solve), 2),
            (row[1], "reset", idle(BTN_RESET), lambda: busy or cr.reset(), 2),
            (row[2], "save", idle(BTN_NAV), lambda: cr.open_files("save"), 2),
            (row[3], "load", idle(BTN_NAV), lambda: cr.open_files("load"), 2),
            (row[4], "exit", BTN_NAV, exit_, 2)]
