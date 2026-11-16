"""A part's way in through an opening: straight moves and turns (``assemble.path``)."""
from __future__ import annotations

import build123d as bd
import pytest

from jointbench.assemble import joint as J
from jointbench.assemble.path import outside, sweep_path


def _box():
    """A closed box 100 mm inside, walls 5 mm, one 30 x 30 mm opening in the +x wall."""
    walls = {
        "floor": bd.Pos(0, 0, -52.5) * bd.Box(110, 110, 5), "roof": bd.Pos(0, 0, 52.5) * bd.Box(110, 110, 5),
        "back": bd.Pos(-52.5, 0, 0) * bd.Box(5, 110, 110), "left": bd.Pos(0, -52.5, 0) * bd.Box(110, 5, 110),
        "right": bd.Pos(0, 52.5, 0) * bd.Box(110, 5, 110),
        "front_low": bd.Pos(52.5, 0, -32.5) * bd.Box(5, 110, 45), "front_high": bd.Pos(52.5, 0, 32.5) * bd.Box(5, 110, 45),
        "front_a": bd.Pos(52.5, -32.5, 0) * bd.Box(5, 45, 20), "front_b": bd.Pos(52.5, 32.5, 0) * bd.Box(5, 45, 20),
    }
    return walls


def test_a_path_from_outside_with_a_turn():
    walls = _box()
    # a plate 40 long, 8 wide, 8 thick: through the 30 mm opening only end on, then turned flat inside
    plate = bd.Pos(0, 0, 0) * bd.Box(40, 8, 8)
    path = [{"move": [-80.0, 0.0, 0.0]}, {"turn": {"point": [0, 0, 0], "axis": [0, 0, 1], "deg": 90}}]
    # the plate at its place lies along y; its path starts with it along x, 80 mm out beyond the opening
    place = bd.Rot(0, 0, 90) * plate
    hits, rows, why = sweep_path(place, path, walls, box=list(walls.values()))
    assert why is None and max(hits.values()) < 1e-6, (why, hits)
    # the same, but starting inside the box: refused
    hits, rows, why = sweep_path(place, [{"turn": {"point": [0, 0, 0], "axis": [0, 0, 1], "deg": 90}}], walls,
                                 box=list(walls.values()))
    assert why and "inside the box" in why
    # through the opening flat (not end on) it runs into the wall
    hits, rows, why = sweep_path(bd.Rot(0, 0, 90) * plate, [{"move": [-80.0, 0.0, 0.0]}], walls,
                                 box=list(walls.values()))
    assert why is None and max(hits.values()) > 1.0


def test_outside_and_the_file_format():
    walls = list(_box().values())
    assert outside(bd.Pos(200, 0, 0) * bd.Box(10, 10, 10), walls)
    assert not outside(bd.Pos(0, 0, 0) * bd.Box(10, 10, 10), walls)
    segs = J._path([{"move": [1, 2, 3]}, {"turn": {"point": [0, 0, 0], "axis": [0, 0, 1], "deg": 30}}], "p")
    assert segs[1]["turn"]["degrees"] == 30.0
    with pytest.raises(J.AssemblyError):
        J._path([{"turn": {"point": [0, 0, 0], "axis": [0, 0, 1]}}], "p")
