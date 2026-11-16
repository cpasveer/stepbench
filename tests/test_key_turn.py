"""KEY_TURN: each bolt can be turned with its standard L-shaped hex key.

The geometry here is built round one M5 bolt whose head top is at the
origin, its axis +Z: a plate under the head and whatever crowds it.
"""
import math

import build123d as bd
import pytest

from jointbench.catalogue import BOLTS, KEYS
from jointbench.checks.key_turn import key_legs, turn
from jointbench.checks.base import volume

SIZE = "M5"
B = BOLTS[SIZE]
K = KEYS[SIZE]


def _plate():
    """A plate the bolt goes through, its top at the underside of the head."""
    return (bd.Pos(0, 0, -B.head_h - 5) * bd.Box(160, 160, 10)
            - bd.Pos(0, 0, -B.head_h) * bd.Cylinder(B.clear_medium / 2, 30))


def _straight_tool_free(bodies) -> bool:
    """TOOL_ACCESS's test: a straight cylinder of the head diameter, 30 mm
    long, from the head top along the axis, touches nothing."""
    tool = bd.Pos(0, 0, 15) * bd.Cylinder(B.head_d / 2, 30)
    return all(volume(tool & s) <= 0.01 for s in bodies.values())


def test_key_dimensions_come_from_the_catalogue():
    short, long_ = key_legs(SIZE)
    sb, lb = short.bounding_box(), long_.bounding_box()
    top = K.short_leg - K.socket_depth
    assert sb.max.Z == pytest.approx(top) and sb.min.Z == pytest.approx(0)
    assert lb.max.Z == pytest.approx(top) and lb.min.Z == pytest.approx(top - K.s)  # a flat on top
    assert lb.max.X - lb.min.X == pytest.approx(K.long_leg)
    assert (K.s, K.long_leg, K.short_leg) == (4.0, 70.0, 25.0)  # ISO 4762 s = 4, ISO 2936 l1 x l2


def test_free_access_passes_with_a_full_turn():
    r = turn(SIZE, (0, 0, 0), (0, 0, 1), {"part": _plate()})
    assert r["ok"] and r["swing"] == 60.0 and r["open"] == 360.0 and r["tilt"] == 0.0


def test_bolt_tucked_into_a_corner_fails():
    # the bolt sits in the closed end of a deep 10 mm slot: walls 5 mm away
    # on three sides, so the key's long leg has nowhere to swing
    walls = (bd.Pos(0, 0, 25) * bd.Box(100, 100, 50)
             - bd.Pos(-25, 0, 25) * bd.Box(60, 10, 52))  # open to -X, ending 5 mm past the bolt
    bodies = {"part": _plate() + walls}
    r = turn(SIZE, (0, 0, 0), (0, 0, 1), bodies)
    assert not r["ok"] and r["swing"] < 60 and r["blocked_by"] == ["part"]


def test_reachable_straight_on_but_too_cramped_to_swing_fails():
    # eight tall posts on a 15 mm circle round the bolt, 45 degrees apart:
    # a straight tool reaches the head, the key's long leg cannot pass
    posts = {f"post {i}": bd.Pos(15 * math.cos(i * math.pi / 4), 15 * math.sin(i * math.pi / 4), 40)
             * bd.Cylinder(3, 80) for i in range(8)}
    bodies = {"part": _plate(), **posts}
    assert _straight_tool_free(bodies)
    r = turn(SIZE, (0, 0, 0), (0, 0, 1), bodies)
    assert not r["ok"] and r["swing"] < 45
    assert r["blocked_by"] and all(n.startswith("post") for n in r["blocked_by"])


def _low_ring(top: float):
    return bd.Pos(0, 0, top / 2) * (bd.Cylinder(20, top) - bd.Cylinder(16, top))


def test_passes_only_thanks_to_the_ball_end_lean():
    # a low ring wall 16-20 mm from the bolt, its top 2 mm above the bottom of
    # the key's long leg: held straight the key hits it all round, leaning
    # away from the long leg lifts the leg over it
    top = K.short_leg - K.socket_depth - K.s + 2.0
    bodies = {"part": _plate(), "ring": _low_ring(top)}
    assert _straight_tool_free(bodies)
    r = turn(SIZE, (0, 0, 0), (0, 0, 1), bodies)
    assert r["ok"] and r["tilt"] > 0 and r["direction"] == 180.0  # leaning away from the long leg
    # the same ring with the lean taken away fails
    import jointbench.checks.key_turn as kt

    old = kt.KEY_TILT_MAX
    try:
        kt.KEY_TILT_MAX = 0.0
        r0 = turn(SIZE, (0, 0, 0), (0, 0, 1), bodies)
    finally:
        kt.KEY_TILT_MAX = old
    assert not r0["ok"] and r0["blocked_by"] == ["ring"]


def test_deterministic_and_axis_independent():
    top = K.short_leg - K.socket_depth - K.s + 2.0
    bodies = {"part": _plate(), "ring": _low_ring(top)}
    a = turn(SIZE, (0, 0, 0), (0, 0, 1), bodies)
    assert turn(SIZE, (0, 0, 0), (0, 0, 1), bodies) == a
    # the same scene turned and moved: the same verdict and lean
    loc = bd.Location((10, -20, 30), (1, 1, 0), 40)
    moved = {n: s.moved(loc) for n, s in bodies.items()}
    pivot = loc * bd.Location((0, 0, 0))
    axis = bd.Vector(0, 0, 1).rotate(bd.Axis((0, 0, 0), (1, 1, 0)), 40)
    b = turn(SIZE, tuple(pivot.position), tuple(axis), moved)
    assert (b["ok"], b["tilt"], b["direction"]) == (a["ok"], a["tilt"], a["direction"])


# --------------------------------------------------------------------------
# the check inside the grader, and the generators that require it


class _Ctx:
    """The little of a grading context KEY_TURN reads: one pose, bolts with
    their heads at the given points, axis +Z, the given bodies."""

    def __init__(self, bolts, bodies, verdict_only=False):
        from types import SimpleNamespace

        self.task = SimpleNamespace(tolerances={"overlap_volume_max": 0.01},
                                    bolts=lambda pose=0: bolts)
        self._bodies, self.grips, self.verdict_only = bodies, {}, verdict_only

    def poses(self):
        return range(1)

    def stored(self, k):
        return k

    def solid_at(self, k):
        return self._bodies["part"]

    def fixed(self, k):
        return {n: s for n, s in self._bodies.items() if n != "part"}

    def obstacles(self, k):
        return {}


def _bolt(i, size, x):
    from types import SimpleNamespace

    b = BOLTS[size]
    # the head top at z = 0: the bolt's point on the face sits grip + head below
    return SimpleNamespace(id=f"A{i}", size=size, position=(x, 0.0, -b.grip_min - b.head_h), axis=(0.0, 0.0, 1.0))


def _slot_end(x):
    return (bd.Pos(x, 0, 25) * bd.Box(100, 100, 50) - bd.Pos(x - 25, 0, 25) * bd.Box(60, 10, 52))


def test_nut_end_is_not_judged():
    from jointbench.catalogue import nut_side
    from jointbench.checks import key_turn

    # a nut tucked where no key could turn a head: not judged, the check passes
    r = key_turn.run(_Ctx([_bolt(1, nut_side(SIZE), 0.0)], {"part": _plate() + _slot_end(0.0)}))
    assert r.ok and r.details["bolts"] == []


def test_verdict_only_grading_stops_at_the_first_bolt_that_fails():
    from jointbench.checks import key_turn

    bodies = {"part": _plate() + _slot_end(-200.0) + _slot_end(200.0)}
    bolts = [_bolt(1, SIZE, -200.0), _bolt(2, SIZE, 200.0)]
    full = key_turn.run(_Ctx(bolts, bodies))
    quick = key_turn.run(_Ctx(bolts, bodies, verdict_only=True))
    assert not full.ok and len(full.details["problems"]) == 2
    assert not quick.ok and len(quick.details["bolts"]) == 1 and quick.reason == full.details["problems"][0]


def test_every_generator_requires_key_turn():
    from jointbench.assemble import generate as assemble
    from jointbench.moves import generate as moves

    assert "KEY_TURN" in assemble.TOLERANCES["required_checks"]
    # adapts: pending its new reference (see the TODO in families/adapt/angled.py)
    assert "KEY_TURN" in moves.REQUIRED
    for mod in ("level1", "level2", "level3"):  # the connect levels add it as they make a task
        import inspect
        import importlib

        assert "apply_key_turn(task)" in inspect.getsource(importlib.import_module(f"jointbench.levels.{mod}").generate)
