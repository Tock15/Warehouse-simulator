"""PySwip bridge to the Prolog engine: validate(level) and solve(level, algo) (NewPlan.md section 5).

The GUI calls everything through submit(), which runs it on one background "prolog" thread: the
window keeps redrawing during long searches, and queries never run at the same time (level facts
asserted by one query can't clash with another). Call shutdown() before exiting."""
import queue
import re
import threading
from concurrent.futures import Future
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]

_prolog = None
_jobs = queue.Queue()


def _engine():
    """Start SWI-Prolog and load the engine once. Call it first from the main thread (submit() does):
    the background thread then gets its own secondary engine, which shutdown() can release cleanly."""
    global _prolog
    if _prolog is None:
        from pyswip import Prolog  # imported lazily: needs SWI-Prolog installed

        _prolog = Prolog()
        _prolog.consult((ROOT / "prolog" / "solve.pl").as_posix())
    return _prolog


def _worker():
    while True:
        fn, args, future = _jobs.get()
        try:
            future.set_result(fn(*args))
        except Exception as ex:  # handed to whoever reads the future
            future.set_exception(ex)


# daemon: closing the window never waits for a search that is still running
threading.Thread(target=_worker, name="prolog", daemon=True).start()


def submit(fn, *args):
    """Run fn(*args) on the Prolog thread, in order. Returns a concurrent.futures.Future."""
    future = Future()
    try:
        _engine()
    except Exception as ex:  # e.g. SWI-Prolog not installed: report it through the future
        future.set_exception(ex)
        return future
    _jobs.put((fn, args, future))
    return future


def shutdown(timeout=1.0):
    """Release the Prolog thread's engine so SWI-Prolog exits quietly. Skipped if a search is still running."""
    if _prolog is None:
        return

    def release():
        from pyswip.core import _lib

        _lib.PL_thread_destroy_engine()

    try:
        submit(release).result(timeout)
    except TimeoutError:
        pass


_ID = re.compile(r"[a-z][a-z0-9_]*")
_KEYS = {"walls", "goals", "boxes", "robot", "fragile", "belts", "plates", "doors"}


def _fields(d, keys):
    """d itself, after checking it is a dict with exactly these keys."""
    if not isinstance(d, dict) or set(d) != set(keys):
        raise ValueError(f"expected keys {sorted(keys)}, got {d!r}")
    return d


def _list(v):
    if not isinstance(v, (list, tuple)):
        raise ValueError(f"expected a list, got {v!r}")
    return v


def _id(s):
    if not isinstance(s, str) or not _ID.fullmatch(s):
        raise ValueError(f"bad id {s!r}")
    return s


def _int(v, lo, hi):
    if type(v) is not int or not lo <= v <= hi:  # type(): rejects True/False too
        raise ValueError(f"expected an integer {lo}-{hi}, got {v!r}")
    return v


def _cell(p):
    if not isinstance(p, (list, tuple)) or len(p) != 2:
        raise ValueError(f"bad coordinate {p!r}")
    return f"{_int(p[0], 0, 9)}-{_int(p[1], 0, 9)}"


def _belt(b):
    b = _fields(b, ("entry", "exit"))
    e, x = _cell(b["entry"]), _cell(b["exit"])
    (ex, ey), (xx, xy) = b["entry"], b["exit"]
    if e == x or (ex != xx and ey != xy):
        raise ValueError(f"belt must be a straight line of 2+ cells: {b!r}")
    return f"belt({e},{x})"


def _box(b):
    b = _fields(b, ("id", "pos", "weight", "goal"))
    goal = "any" if b["goal"] is None else f"only({_id(b['goal'])})"
    return f"box({_id(b['id'])},{_cell(b['pos'])},{_int(b['weight'], 1, 3)},{goal})"


def _term(level):
    """The level/8 term text for load_level/1. Trust boundary: every value is checked here,
    so nothing from the UI reaches a query unchecked. Raises ValueError on bad input."""
    lv = _fields(level, _KEYS)

    def terms(key, fn):
        return "[" + ",".join(fn(v) for v in _list(lv[key])) + "]"

    def goal(g):
        g = _fields(g, ("id", "pos"))
        return f"goal({_id(g['id'])},{_cell(g['pos'])})"

    def fragile(f):
        f = _fields(f, ("pos", "max_weight"))
        return f"fragile({_cell(f['pos'])},{_int(f['max_weight'], 1, 2)})"

    return (f"level({terms('walls', _cell)},{terms('goals', goal)},{terms('boxes', _box)},"
            f"{_cell(lv['robot'])},{terms('fragile', fragile)},{terms('belts', _belt)},"
            f"{terms('plates', _cell)},{terms('doors', _cell)})")


def _load(level):
    """Check level and assert it as the current level facts. Run it on the Prolog thread."""
    list(_engine().query(f"level:load_level({_term(level)})"))


_ALGOS = {"astar", "greedy", "ucs"}


def _step(s):
    """Prolog step -> UI step: "u"/"d"/"l"/"r", ("ride", [x, y]), ("pick", id), ("drop", id, [x, y])."""
    if isinstance(s, str):
        return s
    if s[0] == "ride":
        return ("ride", [s[1], s[2]])
    if s[0] == "pick":
        return ("pick", s[1])
    return ("drop", s[1], [s[2], s[3]])


def validate(level):
    """{"ok": True} or {"ok": False, "reason": str}. Runs A*, so call it through submit().
    Raises ValueError on malformed input (bad ids, coordinates, ...)."""
    out = list(_engine().query(f"solve:bridge_validate({_term(level)}, Out)"))[0]["Out"]
    return {"ok": True} if out == ["ok"] else {"ok": False, "reason": out[1]}


def solve(level, algo="astar"):
    """{"plan": [...], "cost": int, "nodes": int}, or None if unsolvable. Call it through submit().
    Raises ValueError on malformed input or a level validate() would reject for a cheap reason."""
    if algo not in _ALGOS:
        raise ValueError(f"algo must be one of {sorted(_ALGOS)}, got {algo!r}")
    out = list(_engine().query(f"solve:bridge_solve({_term(level)}, {algo}, Out)"))[0]["Out"]
    if out[0] == "error":
        raise ValueError(out[1])
    if out[0] == "no_solution":
        return None
    _, steps, cost, nodes = out
    return {"plan": [_step(s) for s in steps], "cost": cost, "nodes": nodes}
