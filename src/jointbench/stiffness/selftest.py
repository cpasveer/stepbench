"""Proof the stiffness figure can be trusted.

* a cantilever beam against the textbook formulas (bending with shear,
  stretching, twisting, end moment), within a stated percentage;
* a bracket with and without a rib: the rib must raise both stiffness and
  stiffness per unit material;
* a thin rod, a block and a tube of the same span: what each figure says
  about sticks, blocks and closed sections;
* the flexibility matrix is symmetric (load and reading are mirror images);
* the same file twice gives the same figure.

Every part is written to a real STEP file and analysed through the same
path as an answer.
"""
from __future__ import annotations

import json
import math
import os
import tempfile
import time
from dataclasses import dataclass

import build123d as bd
import numpy as np

from ..geometry import write_step
from . import analyse, compare, settings
from .mounts import Mount

L, A = 100.0, 10.0  # cantilever length and square side, mm


def _mount(name, origin, n, x, size):
    n, x = np.array(n, float), np.array(x, float)
    return Mount(name, np.array(origin, float), x, np.cross(n, x), n, size)


def cantilever():
    part = bd.Box(L, A, A, align=(bd.Align.MIN, bd.Align.CENTER, bd.Align.CENTER))
    mounts = [_mount("A", (0, 0, 0), (1, 0, 0), (0, 1, 0), (A, A)),
              _mount("B", (L, 0, 0), (-1, 0, 0), (0, 1, 0), (A, A))]
    return part, mounts


def textbook() -> dict:
    """Cantilever, clamped at A, loaded at B (B's axes: x = world y,
    y = world z, z = along the beam, towards A)."""
    m = settings()["material"]
    E, nu = m["E"], m["nu"]
    G = E / (2 * (1 + nu))
    I = A**4 / 12
    area = A * A
    kappa = 10 * (1 + nu) / (12 + 11 * nu)  # Timoshenko shear factor, rectangle (Cowper)
    J = 0.1406 * A**4  # torsion constant of a square (Roark)
    bend = 1 / (L**3 / (3 * E * I) + L / (kappa * G * area))
    return {"Fx at B": bend, "Fy at B": bend, "Fz at B": E * area / L,
            "Mx at B": E * I / L, "My at B": E * I / L, "Mz at B": G * J / L}


TOL = {"F": 3.0, "M": 3.0}  # percent allowed off the textbook value


def bracket(rib: bool):
    """An L bracket: a 40 x 30 x 4 base on mount A, a 40 x 30 x 4 upright
    carrying mount B on its outer face; optionally a 4 mm triangular rib."""
    base = bd.Box(40, 30, 4, align=(bd.Align.CENTER, bd.Align.MIN, bd.Align.MIN))
    up = bd.Box(40, 4, 30, align=(bd.Align.CENTER, bd.Align.MIN, bd.Align.MIN))
    part = base + up
    if rib:
        tri = bd.Polygon((4, 4), (30, 4), (4, 30), align=None)
        web = bd.extrude(bd.Plane.YZ * tri, amount=2, both=True)
        part = part + web
    mounts = [_mount("A", (0, 15, 0), (0, 0, 1), (1, 0, 0), (40, 30)),
              _mount("B", (0, 0, 17), (0, 1, 0), (1, 0, 0), (40, 26))]
    return part, mounts


SPAN = 60.0


def bar(kind: str):
    """Same span, end mounts on the end faces: a 6 mm rod, a 20 mm square
    block, and a tube with the block's material (20 mm square equivalent)."""
    if kind == "rod":
        part = bd.Cylinder(3, SPAN, rotation=(0, 90, 0), align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
        size = (6, 6)
    elif kind == "block":
        part = bd.Box(SPAN, 20, 20, align=(bd.Align.MIN, bd.Align.CENTER, bd.Align.CENTER))
        size = (20, 20)
    else:
        ro = 20.0
        ri = math.sqrt(ro**2 - 400 / math.pi)  # same cross-section area as the block
        part = bd.Cylinder(ro, SPAN, rotation=(0, 90, 0), align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN)) \
            - bd.Cylinder(ri, SPAN, rotation=(0, 90, 0), align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
        size = (2 * ro, 2 * ro)
    part = bd.Pos(0, 0, 0) * part
    if kind != "block":
        part = bd.Rot(0, 0, 0) * part
    mounts = [_mount("A", (0, 0, 0), (1, 0, 0), (0, 1, 0), size),
              _mount("B", (SPAN, 0, 0), (-1, 0, 0), (0, 1, 0), size)]
    return part, mounts


def _run(part, mounts, tmp, name, **kw):
    path = os.path.join(tmp, f"{name}.step")
    write_step(part, path)
    return analyse(path, mounts, keep_fields=kw.pop("keep_fields", False), **kw)


@dataclass
class Row:
    label: str
    expected: str
    measured: str
    ok: bool


def run(progress=None) -> tuple[list[Row], dict]:
    rows, reps = [], {}
    with tempfile.TemporaryDirectory() as tmp:
        part, mounts = cantilever()
        rep = _run(part, mounts, tmp, "cantilever")
        reps["cantilever"] = rep
        ref = textbook()
        for c in rep.cases:
            want = ref[c.label]
            off = 100 * (c.stiffness / want - 1)
            tol = TOL[c.label[0]]
            rows.append(Row(f"cantilever {c.label} ({c.unit})", f"{want:.5g} +/- {tol:g} %",
                            f"{c.stiffness:.5g} ({off:+.2f} %)", abs(off) <= tol))
        rows.append(Row("cantilever flexibility symmetric", "asymmetry < 1e-6",
                        f"{rep.symmetry:.1e}", rep.symmetry < 1e-6))
        if progress:
            progress("cantilever")

        t = time.perf_counter()
        again = _run(part, mounts, tmp, "cantilever2")
        same = json.dumps([c.stiffness for c in again.cases]) == json.dumps([c.stiffness for c in rep.cases])
        rows.append(Row("same file twice", "identical figures", "identical" if same else "DIFFERENT", same))
        if progress:
            progress(f"repeat {time.perf_counter() - t:.1f} s")

        plain = _run(*bracket(False), tmp, "bracket")
        ribbed = _run(*bracket(True), tmp, "bracket_rib")
        reps["bracket"], reps["bracket_rib"] = plain, ribbed
        par = compare(ribbed, plain)
        rows.append(Row("bracket with rib vs without: stiffness", "> x2 stiffer",
                        f"x{par.stiffness:.2f}", par.stiffness > 2))
        rows.append(Row("bracket with rib vs without: per unit material", "> x1.5",
                        f"x{par.specific:.2f} (material x{par.material:.2f})", par.specific > 1.5))
        if progress:
            progress("bracket")

        bars = {k: _run(*bar(k), tmp, k) for k in ("rod", "block", "tube")}
        reps.update(bars)
        rod, tube = compare(bars["rod"], bars["block"]), compare(bars["tube"], bars["block"])
        rows.append(Row("rod vs block, same span: stiffness", "rod far weaker",
                        f"x{rod.stiffness:.4f}", rod.stiffness < 0.05))
        rows.append(Row("rod vs block: stiffness per unit material", "rod lower (k/V favours bulk)",
                        f"x{rod.specific:.3f}", rod.specific < 0.5))
        rows.append(Row("rod vs block: shape efficiency", "about equal (both solid)",
                        f"x{rod.efficiency:.2f}", 0.7 < rod.efficiency < 1.3))
        rows.append(Row("tube vs block, same material: stiffness", "tube stiffer",
                        f"x{tube.stiffness:.2f} (material x{tube.material:.2f})", tube.stiffness > 2))
        rows.append(Row("tube vs block: shape efficiency", "tube higher",
                        f"x{tube.efficiency:.2f}", tube.efficiency > 2))
        if progress:
            progress("bars")
    return rows, reps


def table(rows: list[Row]) -> str:
    w = max(len(r.label) for r in rows)
    e = max(len(r.expected) for r in rows)
    out = [f"{'case':<{w}}  {'expected':<{e}}  {'measured':<28} ok",
           f"{'-' * w}  {'-' * e}  {'-' * 28} --"]
    for r in rows:
        out.append(f"{r.label:<{w}}  {r.expected:<{e}}  {r.measured:<28} {'yes' if r.ok else 'NO'}")
    return "\n".join(out)


def convergence(cases, sizes=(4.0, 3.0, 2.0, 1.5, 1.0), progress=None) -> list[dict]:
    """How the figure moves as the elements get finer. ``cases`` is a list
    of (label, step_path, mounts)."""
    out = []
    for label, path, mounts in cases:
        rows = []
        for h in sizes:
            rep = analyse(path, mounts, h=h, keep_fields=False, quality_on_failure=False)
            rows.append({"h": h, "elements": rep.elements, "dofs": rep.dofs, "seconds": round(rep.seconds, 1),
                         "cases": {c.label: c.stiffness for c in rep.cases}})
            if progress:
                progress(f"{label} h={h} {rep.seconds:.1f}s")
        fine = rows[-1]["cases"]
        for r in rows:
            r["change"] = {k: 100 * (v / fine[k] - 1) for k, v in r["cases"].items()}
            r["worst_change"] = max(abs(v) for v in r["change"].values())
            wk = min(r["cases"], key=r["cases"].get)
            r["weakest"], r["weakest_value"] = wk, r["cases"][wk]
        out.append({"label": label, "rows": rows})
    return out


def convergence_table(res) -> str:
    out = []
    for c in res:
        out.append(f"{c['label']}")
        out.append(f"  {'size mm':>7} {'elements':>9} {'unknowns':>9} {'s':>6}  {'weakest case':<12} "
                   f"{'value':>10}  {'largest change vs finest, any case':>34}")
        for r in c["rows"]:
            out.append(f"  {r['h']:>7g} {r['elements']:>9} {r['dofs']:>9} {r['seconds']:>6}  {r['weakest']:<12} "
                       f"{r['weakest_value']:>10.4g}  {r['worst_change']:>33.2f} %")
    return "\n".join(out)


def pictures(out_dir: str) -> list[str]:
    from .viz import dominant_case, render

    paths = []
    with tempfile.TemporaryDirectory() as tmp:
        for name, (part, mounts), case in (("cantilever", cantilever(), "Fx at B"),
                                           ("bracket", bracket(False), None),
                                           ("bracket_rib", bracket(True), None)):
            rep = _run(part, mounts, tmp, name, keep_fields=True)
            c = next(x for x in rep.cases if x.label == case) if case else dominant_case(rep)
            paths.append(render(rep, c, os.path.join(out_dir, f"selftest_{name}.png")))
    return paths


# ------------------------------------------------------------- strength

def _bolted(name, origin, n, x, size, bolt="M5", count=4):
    m = _mount(name, origin, n, x, size)
    m.bolt_size, m.n_bolts = bolt, count
    return m


def notched_bar(r: float, W: float = 20.0, t: float = 6.0, L: float = 120.0):
    """A flat bar W x t, L long, with two opposite semicircular edge notches
    of radius r at mid-length. Mounts on the end faces."""
    bar = bd.Box(L, W, t, align=(bd.Align.MIN, bd.Align.CENTER, bd.Align.CENTER))
    for y in (W / 2, -W / 2):
        bar = bar - bd.Pos(L / 2, y, 0) * bd.Cylinder(r, t + 2)
    mounts = [_bolted("A", (0, 0, 0), (1, 0, 0), (0, 1, 0), (W, t)),
              _bolted("B", (L, 0, 0), (-1, 0, 0), (0, 1, 0), (W, t))]
    return bar, mounts


def kt_semicircular(r, W):
    """Peterson, two opposite semicircular notches in a flat bar in tension,
    on the net section (Pilkey & Pilkey, Peterson's Stress Concentration
    Factors, 3rd ed., eq. for Chart 2.3)."""
    x = 2 * r / W
    return 3.065 - 3.370 * x + 0.647 * x ** 2 + 0.658 * x ** 3


def holed_plate(d: float, H: float = 30.0, t: float = 4.0, L: float = 120.0):
    plate = bd.Box(L, H, t, align=(bd.Align.MIN, bd.Align.CENTER, bd.Align.CENTER)) - bd.Pos(L / 2, 0, 0) * bd.Cylinder(d / 2, t + 2)
    mounts = [_bolted("A", (0, 0, 0), (1, 0, 0), (0, 1, 0), (H, t)),
              _bolted("B", (L, 0, 0), (-1, 0, 0), (0, 1, 0), (H, t))]
    return plate, mounts


def kt_hole(d, H):
    """Peterson, central circular hole in a finite-width plate in tension,
    on the net section (Pilkey & Pilkey, eq. 4.1)."""
    x = 1 - d / H
    return 2 + 0.284 * x - 0.600 * x ** 2 + 1.32 * x ** 3


def filleted_bracket(r: float):
    """The L bracket (40 x 30 x 4 base, 40 x 30 x 4 upright) with an inside
    fillet of radius r where they meet (r = 0: sharp)."""
    part, mounts = bracket(False)
    if r > 0:
        e = [x for x in part.edges() if abs(x.center().Z - 4) < 1e-6 and abs(x.center().Y - 4) < 1e-6]
        part = bd.fillet(e, r)
    mounts = [_bolted(m.name, m.origin, m.n, m.x, m.size, "M4", 4) for m in mounts]
    return part, mounts


def necked_rod(d: float, L: float = 100.0, end: float = 24.0):
    """Two 24 x 24 x 6 end blocks joined by a round rod of diameter d, with
    2 mm fillets at both ends; mounts on the blocks' outer faces."""
    a = bd.Box(6, end, end, align=(bd.Align.MIN, bd.Align.CENTER, bd.Align.CENTER))
    b = bd.Pos(L - 6, 0, 0) * bd.Box(6, end, end, align=(bd.Align.MIN, bd.Align.CENTER, bd.Align.CENTER))
    rod = bd.Pos(6, 0, 0) * bd.Cylinder(d / 2, L - 12, rotation=(0, 90, 0), align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
    part = a + b + rod
    ring = [x for x in part.edges() if x.geom_type == bd.GeomType.CIRCLE and abs(x.radius - d / 2) < 1e-6
            and (abs(x.center().X - 6) < 1e-6 or abs(x.center().X - (L - 6)) < 1e-6)]
    part = bd.fillet(ring, 2.0)
    mounts = [_bolted("A", (0, 0, 0), (1, 0, 0), (0, 1, 0), (end, end)),
              _bolted("B", (L, 0, 0), (-1, 0, 0), (0, 1, 0), (end, end))]
    return part, mounts


def _plate(cx, cz, w=40.0, t=4.0, hole=0.0):
    p = bd.Pos(cx, 0, cz) * bd.Box(w, w, t)
    if hole:
        p = p - bd.Pos(cx, 0, cz) * bd.Cylinder(hole / 2, t + 2)
    return p


def _rod(a, b, d):
    a, b = bd.Vector(*a), bd.Vector(*b)
    v = b - a
    return bd.Plane(origin=tuple(a), z_dir=tuple(v.normalized())) * bd.Cylinder(d / 2, v.length,
                                                                               align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))


def _two_plate_mounts(top=64.0):
    return [_bolted("A", (0, 0, 0), (0, 0, 1), (1, 0, 0), (40, 40), "M4", 4),
            _bolted("B", (0, 0, top), (0, 0, -1), (1, 0, 0), (40, 40), "M4", 4)]


def flimsy_bar():
    """Two 40 x 40 x 4 plates, 60 mm apart, joined by one 4 mm round bar."""
    return _plate(0, 2) + _plate(0, 62) + _rod((0, 0, 3.9), (0, 0, 60.1), 4.0), _two_plate_mounts()


def flimsy_column():
    """A base plate and a ring plate (40 mm square, 24 mm hole) on one 6 mm column off-centre."""
    return _plate(0, 2) + _plate(0, 62, hole=24) + _rod((15, 15, 3.9), (15, 15, 60.1), 6.0), _two_plate_mounts()


def flimsy_spokes():
    """An 8 mm rod from the base plate to a hub, then three 3 mm spokes to the top plate."""
    hub = bd.Pos(0, 0, 40) * bd.Sphere(6)
    body = _plate(0, 2) + _plate(0, 62) + _rod((0, 0, 3.9), (0, 0, 40), 8.0) + hub
    for k in range(3):
        a = 2 * math.pi * k / 3
        body = body + _rod((0, 0, 40), (14 * math.cos(a), 14 * math.sin(a), 60.1), 3.0)
    return body, _two_plate_mounts()
