"""BEARING_SEAT on parts built round one bearing, placed at an angle in space."""
import math

import build123d as bd
import pytest

from jointbench.catalogue import BEARINGS
from jointbench.checks.bearing_seat import inner_race_radius, measure
from jointbench.geometry import Placement, rot_axis_angle

D_, D, B = BEARINGS["608"]
PL = Placement((12.0, -30.0, 41.0), rot_axis_angle((0.3, 0.8, 0.5), 37.0))


def boss(bore=D, shoulder=True, outer=D + 8, through=True, block_entry=False, length=B + 4):
    """A round boss holding the bearing: bore of ``bore`` over the width, a
    shoulder past it, a hole for the shaft."""
    body = bd.Pos(0, 0, length / 2 - 1) * bd.Cylinder(outer / 2, length)
    body = body - bd.Pos(0, 0, B / 2 - 0.5) * bd.Cylinder(bore / 2, B + 1)  # the seat, open at z < 0
    if not shoulder:
        body = body - bd.Pos(0, 0, length / 2) * bd.Cylinder(bore / 2, length + 2)
    if through:
        body = body - bd.Pos(0, 0, length / 2) * bd.Cylinder(inner_race_radius(D_, D) + 0.5, length + 4)
    if block_entry:
        body = body + bd.Pos(0, 0, -6) * bd.Box(4, 4, 2)
    return body.moved(PL.location())


def m(part):
    return measure(part, PL, D_, D, B)


def test_good_seat_passes():
    r = m(boss())
    assert all(v["ok"] for v in r.values()), r


@pytest.mark.parametrize("what, part, fails", [
    ("bore 0.2 mm too large", lambda: boss(bore=D + 0.2), "bore"),
    ("no shoulder", lambda: boss(shoulder=False), "shoulder"),
    ("shaft hole closed", lambda: boss(through=False), "inner_race_free"),
    ("something in the way in", lambda: boss(block_entry=True), "way_in"),
    ("wall 1 mm", lambda: boss(outer=D + 2), "wall"),
])
def test_broken_seat_fails_on_its_measure(what, part, fails):
    r = m(part())
    assert not r[fails]["ok"], (what, r[fails])


def test_bore_slightly_large_inside_band_passes():
    assert m(boss(bore=D + 0.08))["bore"]["ok"]


def test_bearing_is_a_loaded_mount_with_its_own_seat():
    """The stiffness analysis finds the bearing's seat (bore and shoulder) and loads it by forces only."""
    import numpy as np

    from jointbench.stiffness.mounts import Mount

    n = np.array(PL.vec((0, 0, 1.0)))
    m = Mount("C", np.array(PL.point((0, 0, B / 2))), np.array(PL.vec((1.0, 0, 0))), np.array(PL.vec((0, 1.0, 0))), n,
              (0.0, 0.0), kind="bearing", D=D, B=B, d=D_)
    pred = m.seat_pred(0.06, 0.15)
    on_bore = np.array([PL.point((D / 2, 0.0, B / 2))])
    on_shoulder = np.array([PL.point((0.0, (inner_race_radius(D_, D) + D / 2) / 2, B))])
    off = np.array([PL.point((D / 2 + 3, 0.0, B / 2)), PL.point((0.0, 0.0, -5.0))])
    assert pred(on_bore).all() and pred(on_shoulder).all() and not pred(off).any()
