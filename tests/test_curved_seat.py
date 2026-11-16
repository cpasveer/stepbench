"""A fixture with a curved face (a pipe, bolts square to its surface): seating measured on the curve."""
import build123d as bd
import pytest

from jointbench.catalogue import BOLTS, make_interface
from jointbench.geometry import Placement, rot_axis_angle, write_step
from jointbench.grade import grade
from jointbench.task import DEFAULT_TOLERANCES, Task

ITF = {"bolt": "M5", "od": 48.0, "holes": [[-20, -30], [20, -30], [-20, 30], [20, 30]]}
PL = Placement((10.0, -5.0, 30.0), rot_axis_angle((0.2, 0.9, 0.4), 33.0))


def _task():
    itf = make_interface("pipe", ITF)
    return Task("pipe-1", 1, 0, {"P": itf}, [{"P": PL}], dict(DEFAULT_TOLERANCES), {})


def saddle(task, gap=0.0, flat=False, t=6.0):
    """A saddle hugging the pipe over the bolts (``flat``: a flat plate instead), holes on every bolt."""
    R = ITF["od"] / 2
    if flat:
        body = bd.Pos(0, 0, R + 2 + t / 2) * bd.Box(64, 50, t)
    else:
        body = bd.Rot(0, 90, 0) * (bd.Cylinder(R + gap + t, 64) - bd.Cylinder(R + gap, 66))
        body = body & (bd.Pos(0, 0, R) * bd.Box(70, 60, 2 * R))  # the upper half, over the bolts
    for (p, a) in task.parts["P"].hole_frames:
        body = body - bd.Plane(origin=p, z_dir=a) * bd.Cylinder(BOLTS["M5"].clear_medium / 2, 80)
    return body.moved(PL.location())


def _seating(tmp_path, part):
    task = _task()
    p = str(tmp_path / "s.step")
    write_step(part, p)
    rep = grade(task, p, optional=False, score=False)
    return next(c for c in rep.checks if c.name == "SEATING")


def test_saddle_on_the_curve_seats(tmp_path):
    c = _seating(tmp_path, saddle(_task()))
    assert c.ok, c.reason


def test_flat_plate_on_a_pipe_does_not_seat(tmp_path):
    c = _seating(tmp_path, saddle(_task(), flat=True))
    assert c.ok is False


def test_saddle_standing_off_does_not_seat(tmp_path):
    c = _seating(tmp_path, saddle(_task(), gap=0.5))
    assert c.ok is False
