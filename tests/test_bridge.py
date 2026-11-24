import copy

import pytest

from warehouse_robot import bridge

SAMPLE = {
    "walls": [[4, 4]],
    "goals": [{"id": "g1", "pos": [8, 8]}, {"id": "g2", "pos": [8, 1]}],
    "boxes": [{"id": "a", "pos": [3, 3], "weight": 2, "goal": None},
              {"id": "b", "pos": [6, 6], "weight": 3, "goal": "g1"}],
    "robot": [1, 1],
    "fragile": [{"pos": [2, 2], "max_weight": 1}],
    "belts": [{"entry": [2, 5], "exit": [5, 5]}],
    "plates": [[1, 8]],
    "doors": [[5, 1]],
}


def broken(path, value):
    """SAMPLE with the value at path (keys/indexes) replaced; value=KeyError deletes it."""
    lv = copy.deepcopy(SAMPLE)
    *head, last = path
    d = lv
    for k in head:
        d = d[k]
    if value is KeyError:
        del d[last]
    else:
        d[last] = value
    return lv


@pytest.mark.parametrize("path, value", [
    (("boxes", 0, "id"), "A"),
    (("boxes", 0, "id"), "1a"),
    (("boxes", 0, "id"), "a). halt. %"),
    (("boxes", 1, "goal"), "G1"),
    (("goals", 0, "id"), ""),
    (("robot",), [10, 1]),
    (("robot",), [-1, 1]),
    (("robot",), [1.5, 1]),
    (("robot",), [True, 1]),
    (("robot",), [1, 1, 1]),
    (("walls", 0), "4-4"),
    (("boxes", 0, "weight"), 0),
    (("boxes", 0, "weight"), 4),
    (("fragile", 0, "max_weight"), 3),
    (("belts", 0, "exit"), [5, 6]),   # diagonal
    (("belts", 0, "exit"), [2, 5]),   # entry == exit
    (("plates",), "1-8"),
    (("doors",), KeyError),
    (("boxes", 0, "weight"), KeyError),
    (("extra",), []),
])
def test_bad_input_raises(path, value):
    with pytest.raises(ValueError):
        bridge._term(broken(path, value))


def test_round_trip():
    def run():
        bridge._load(SAMPLE)
        q = bridge._engine().query
        return (
            list(q("level:box_start(a, X-Y), level:weight(a, W), level:target(a, T)")),
            list(q("level:target(b, only(G))")),
            list(q("level:belt(Ex-Ey, Xx-Xy, L)")),
            list(q("aggregate_all(count, level:wall(_), N)")),
            list(q("level:robot_start(X-Y)")),
        )

    a, b, belt, walls, robot = bridge.submit(run).result(10)
    assert a == [{"X": 3, "Y": 3, "W": 2, "T": "any"}]
    assert b == [{"G": "g1"}]
    assert belt == [{"Ex": 2, "Ey": 5, "Xx": 5, "Xy": 5, "L": 4}]
    assert walls == [{"N": 37}]  # 36 border + 1 inner
    assert robot == [{"X": 1, "Y": 1}]
