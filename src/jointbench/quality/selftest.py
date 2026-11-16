"""Proof the modelling-quality checker can be trusted.

For every rule, a generated part that must pass and one that must fail,
built either side of the limit and checked through a real STEP file. Some
faults cannot occur alone -- a 0.12 mm step is also a 0.12 mm wide sliver
face -- so a failing row names the rule that must fail and the only other
rules allowed to fail with it. Everything else must pass.
"""
from __future__ import annotations

import math
import os
import tempfile
from dataclasses import dataclass, field

import build123d as bd
from OCP.BRepAlgoAPI import BRepAlgoAPI_Fuse

from ..geometry import write_step
from . import check

MIN = (bd.Align.MIN,) * 3


def block():
    return bd.Box(30, 20, 5, align=MIN)


def misaligned(offset=0.1, tilt=1.0):
    """The maintainer's example: a second plate joined to the first 0.1 mm high
    and tilted 1 degree across the joint."""
    b = bd.Pos(28, 0, offset) * bd.Rot(-tilt, 0, 0) * bd.Box(30, 20, 5, align=MIN)
    return block() + b


def flush():
    """The same two plates joined properly: one clean body."""
    return block() + bd.Pos(28, 0, 0) * bd.Box(30, 20, 5, align=MIN)


def chamfered(c):
    body = block()
    e = body.edges().filter_by(bd.Axis.X).sort_by(bd.Axis.Z)[-1]
    return bd.chamfer(e, c)


def corner_cut(a):
    """A block with one corner cut off by a small triangular facet."""
    body = block()
    corner = bd.Vector(30, 20, 5)
    pl = bd.Plane(origin=corner - bd.Vector(a, 0, 0), z_dir=(1, 1, 1))
    return bd.split(body, pl, keep=bd.Keep.BOTTOM)


def prism(points, h=5.0):
    return bd.extrude(bd.Polygon(*points, align=None), amount=h)


def short_side(s):
    """A plate whose corner is cut by a side s mm long."""
    c = s / math.sqrt(2)
    return prism([(0, 0), (20, 0), (20, 10 - c), (20 - c, 10), (0, 10)])


def stepped(h):
    return block() + bd.Pos(30, 0, 0) * bd.Box(20, 20, 5 + h, align=MIN)


def wedge(deg):
    return prism([(0, 0), (30, 0), (0, 30 * math.tan(math.radians(deg)))])


def kinked(deg):
    return prism([(0, 0), (20, 0), (40, 20 * math.tan(math.radians(deg))), (40, 10), (0, 10)])


def unmerged():
    """Two flush boxes fused without the clean-up step: the top, bottom
    and sides stay split along the joint."""
    a = block().wrapped
    b = (bd.Pos(30, 0, 0) * bd.Box(20, 20, 5, align=MIN)).wrapped
    f = BRepAlgoAPI_Fuse(a, b)
    f.Build()
    return bd.Solid(bd.Shape.cast(f.Shape()).solids()[0].wrapped)


def notched(w):
    """A plate with a narrow V notch: its two lips are w mm apart."""
    return prism([(0, 0), (20, 0), (20, 10), (10 + w / 2, 10), (10, 4), (10 - w / 2, 10), (0, 10)])


def bar_joint(angle=50.0, through=True, run_on=4.0):
    """Two parallel plates joined by an 8 x 4 mm bar at ``angle`` to them.
    through=True: the bar is simply run into both plates and on past their
    outer faces (Sonnet's level-2 answer s702 does this). False: the bar is
    trimmed flush with the plates' outer faces."""
    a = math.radians(angle)
    pa = bd.Vector(0, 0, 2)
    pb = bd.Vector(30 / math.tan(a), 0, 32)
    d = (pb - pa).normalized()
    start = pa - d * run_on
    length = (pb - pa).length + 2 * run_on
    bar = bd.Plane(origin=start, z_dir=d) * bd.Box(8, 4, length, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
    if not through:
        bar = bar & bd.Pos(pb.X / 2, 0, 17) * bd.Box(400, 100, 34)
    plate_a = bd.Pos(pa.X, 0, 2) * bd.Box(40, 20, 4)
    plate_b = bd.Pos(pb.X, 0, 32) * bd.Box(40, 20, 4)
    return plate_a + plate_b + bar


def rim():
    """A plate with a 3 mm rim along one edge, its outer face flush with the
    plate's side: a deliberate feature, not a stub."""
    return block() + bd.Pos(0, 0, 5) * bd.Box(30, 3, 3, align=MIN)


CASES = [
    # (label, build, rules that must fail, other rules allowed to fail)
    ("clean block", block, [], []),
    ("two plates joined flush", flush, [], []),
    ("two plates 0.1 mm off, 1 deg tilt", misaligned, ["Q_SMALL_STEP", "Q_THIN_WEDGE"], ["*"]),
    ("chamfer 0.5 mm", lambda: chamfered(0.5), [], []),
    ("chamfer 0.15 mm (sliver)", lambda: chamfered(0.15), ["Q_SLIVER_FACE"], []),
    ("corner facet 0.87 mm2", lambda: corner_cut(1.0), [], []),
    ("corner facet 0.08 mm2", lambda: corner_cut(0.3), ["Q_TINY_FACE"], []),
    ("corner side 0.5 mm", lambda: short_side(0.5), [], []),
    ("corner side 0.05 mm", lambda: short_side(0.05), ["Q_SHORT_EDGE"], ["Q_SLIVER_FACE"]),
    ("step 1 mm", lambda: stepped(1.0), [], []),
    ("step 0.12 mm", lambda: stepped(0.12), ["Q_SMALL_STEP"], ["Q_SLIVER_FACE"]),
    ("wedge 8 deg", lambda: wedge(8), [], []),
    ("wedge 4 deg", lambda: wedge(4), ["Q_THIN_WEDGE"], []),
    ("bend 5 deg", lambda: kinked(5), [], []),
    ("bend 1 deg", lambda: kinked(1), ["Q_CREASE"], []),
    ("fused and not cleaned", unmerged, ["Q_UNMERGED"], []),
    ("bar trimmed to the plates", lambda: bar_joint(50, through=False), [], []),
    ("bar run into the plates (stubs)", lambda: bar_joint(50, through=True), ["Q_STUB"], []),
    ("rim flush with the plate edge", rim, [], []),
    ("bar at 60 deg, trimmed", lambda: bar_joint(60, through=False), [], []),
    ("bar at 30 deg, trimmed", lambda: bar_joint(30, through=False), ["Q_SHALLOW_JOINT"], []),
    ("notch lips 8 mm apart", lambda: notched(8.0), [], []),
    ("notch lips 0.02 mm apart", lambda: notched(0.02), ["Q_NEAR_VERTICES"], ["Q_THIN_WEDGE", "Q_SHALLOW_JOINT"]),
]


@dataclass
class Row:
    label: str
    must: list
    allow: list
    ok: bool = False
    failed: list = field(default_factory=list)
    value: str = ""


def run(cases=CASES) -> list[Row]:
    rows = []
    with tempfile.TemporaryDirectory() as tmp:
        for label, build, must, allow in cases:
            path = os.path.join(tmp, "part.step")
            write_step(build(), path)
            rep = check(path)
            failed = rep.failed()
            if not must:
                ok = not failed
            else:
                ok = set(must) <= set(failed) and ("*" in allow or set(failed) <= set(must) | set(allow))
            row = Row(label, must, allow, ok, failed)
            if must:
                x = next(x for x in rep.rules if x.id == must[0])
                if x.value is not None:
                    row.value = f"{x.value:g} {x.unit}"
            rows.append(row)
    return rows


def table(rows: list[Row]) -> str:
    w = max(len(r.label) for r in rows)
    out = [f"{'part':<{w}}  {'must':<34} {'measured':>12}  {'ok':<3} failed",
           f"{'-' * w}  {'-' * 34} {'-' * 12}  {'-' * 3} {'-' * 6}"]
    for r in rows:
        must = "pass" if not r.must else "fail " + " + ".join(r.must)
        if r.allow == ["*"]:
            must += " (+any)"
        out.append(f"{r.label:<{w}}  {must:<34} {r.value:>12}  {'yes' if r.ok else 'NO '} "
                   f"{', '.join(r.failed) or '-'}")
    return "\n".join(out)


def pictures(out_dir: str, cases=CASES) -> list[str]:
    """One picture per rule, from the first self-test part that must fail it."""
    from ..dfm.viz import render

    done, paths = set(), []
    with tempfile.TemporaryDirectory() as tmp:
        for label, build, must, _ in cases:
            todo = [m for m in must if m not in done]
            if not todo:
                continue
            path = os.path.join(tmp, "part.step")
            write_step(build(), path)
            rep = check(path)
            rep.file = label
            for rid in todo:
                x = next(x for x in rep.rules if x.id == rid)
                paths.append(render(rep, rep.part, x, os.path.join(out_dir, f"{rid}.png")))
                done.add(rid)
    return paths
