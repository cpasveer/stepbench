"""Proof the manufacturing checker can be trusted.

For every rule there is at least one generated part that must pass it and
one that must fail it, built so that the fault sits just over the limit and
its twin just under. Each part goes through a real STEP file, so the whole
path a submitted part takes is exercised. A row is "as expected" when
everything the row names happens: the part passes all rules, or it fails
exactly the named rule and no other.
"""
from __future__ import annotations

import math
import os
import tempfile
from dataclasses import dataclass, field

import build123d as bd

from ..geometry import write_step
from . import check

Z = (0.0, 0.0, 1.0)


# ------------------------------------------------------------ FDM parts


def plate(x=30.0, y=20.0, t=5.0):
    return bd.Pos(0, 0, t / 2) * bd.Box(x, y, t)


def fin(th):
    """A base plate with a thin vertical fin standing on it."""
    return plate() + bd.Pos(0, 0, 5 + 5) * bd.Box(th, 16, 10)


def overhang(alpha_deg):
    """A block whose side leans out over the bed at alpha degrees from vertical."""
    a = 8 * math.tan(math.radians(alpha_deg))
    pts = [(0, 0), (20, 0), (20, 5), (20 + a, 13), (20 + a, 17), (0, 17)]
    prof = bd.Plane.XZ * bd.Polygon(*pts, align=None)
    return bd.extrude(prof, amount=-10).solids()[0]  # 10 mm deep in +Y


def bridge(span):
    """Two pillars joined by a beam: the beam's underside bridges ``span``."""
    w = 5.0
    left = bd.Pos(-(span / 2 + w / 2), 0, 5) * bd.Box(w, 12, 10)
    right = bd.Pos(span / 2 + w / 2, 0, 5) * bd.Box(w, 12, 10)
    beam = bd.Pos(0, 0, 12) * bd.Box(span + 2 * w, 12, 4)
    return left + right + beam


def shelf():
    """A block with a flat shelf sticking out sideways: no support below."""
    return bd.Pos(0, 0, 6) * bd.Box(10, 10, 12) + bd.Pos(8, 0, 10) * bd.Box(6, 10, 4)


def holed(dia):
    return plate() - bd.Cylinder(dia / 2, 20)


def pinned(dia):
    return plate() + bd.Pos(0, 0, 8) * bd.Cylinder(dia / 2, 6)


def slotted(gap):
    """Two fins on a plate with a narrow gap between them."""
    f1 = bd.Pos(-(gap / 2 + 1), 0, 9) * bd.Box(2, 16, 8)
    f2 = bd.Pos(gap / 2 + 1, 0, 9) * bd.Box(2, 16, 8)
    return plate() + f1 + f2


def foot(r_bottom):
    """A cone standing on its small end, sides leaning out 40 degrees, with
    an upright rim on top so its edge is not a knife edge."""
    h = 8.0
    r_top = r_bottom + h * math.tan(math.radians(40))
    return bd.Pos(0, 0, h / 2) * bd.Cone(r_bottom, r_top, h) + bd.Pos(0, 0, h + 1.5) * bd.Cylinder(r_top, 3)


def pillar(h):
    """A 10 x 10 mm pillar h tall standing on its end: the stance is 10 mm."""
    return bd.Pos(0, 0, h / 2) * bd.Box(10, 10, h)


def plus_bar():
    """A bar of plus-shaped section (30 mm across, arms 6 mm thick, 60 mm long),
    tilted off every axis. Lying on any of its long faces, the cross arm hangs
    in the air; it prints only standing on its end, square to two long faces
    at once: a direction the checker finds only by crossing two faces."""
    arm = bd.Box(30, 6, 60) + bd.Box(6, 30, 60)
    return (bd.Rot(23, 37, 11) * arm).solids()[0]


def hollow(sealed):
    body = bd.Pos(0, 0, 5) * bd.Box(20, 20, 10)
    if sealed:
        return body - bd.Pos(0, 0, 5) * bd.Box(6, 6, 4)
    return body - bd.Pos(0, 0, 7.5) * bd.Box(6, 6, 5.1)


# ------------------------------------------------------- moulding parts


def tray(t=2.5, taper=2.0, x=40.0, y=30.0):
    """A plate with drafted sides (top smaller than bottom)."""
    return bd.extrude(bd.Rectangle(x, y), amount=t, taper=taper)


def cap(h, a=10.0, t=2.0):
    """A drafted plate with a shallow spherical cap on top: a thick spot that
    rises gently, so it has no corner and needs no extra draft."""
    base = tray(t)
    R = (a * a + h * h) / (2 * h)
    sph = bd.Pos(0, 0, t + h - R) * bd.Sphere(R)
    keep = bd.Pos(0, 0, t + h) * bd.Box(2 * a + 2, 2 * a + 2, 2 * h)
    return base + (sph & keep)


def hourglass(waist_in: bool, t=3.0, taper=6.0):
    """Two drafted halves meeting at the middle. Waist narrower than the
    faces = an undercut for a straight pull; wider = a fine parting line."""
    h = t / 2
    if waist_in:
        lower = bd.extrude(bd.Rectangle(40, 30), amount=h, taper=taper)
        upper = bd.Pos(0, 0, t) * bd.mirror(bd.extrude(bd.Rectangle(40, 30), amount=h, taper=taper),
                                            bd.Plane.XY)
    else:
        lower = bd.Pos(0, 0, h) * bd.mirror(bd.extrude(bd.Rectangle(40, 30), amount=h, taper=taper),
                                            bd.Plane.XY)
        upper = bd.Pos(0, 0, h) * bd.extrude(bd.Rectangle(40, 30), amount=h, taper=taper)
    return lower + upper


def ribbed(rib_t, fillet=1.5, t=2.5, h=4.0, taper=1.5):
    """A drafted plate with a drafted rib on top, optionally filleted at the
    base. Rib draft 1.5 deg so its thin top stays over the 1 mm wall."""
    base = tray(t)
    rib = bd.Pos(0, 0, t) * bd.extrude(bd.Rectangle(rib_t, 20), amount=h, taper=taper)
    body = base + rib
    if fillet:
        sel = [e for e in body.edges()
               if abs(e.center().Z - t) < 1e-6 and abs(e.center().X) < rib_t and abs(e.center().Y) <= 10.01]
        body = bd.fillet(sel, fillet)
    return body


def lying_overhang():
    """The 50 degree overhang block with no direction stated: lying on its
    big side face it prints with no overhang at all."""
    return overhang(50)


def cavity_ceiling():
    """A block with a sealed 14 x 14 x 4 mm cavity: its ceiling spans 14 mm,
    over the 10 mm bridge limit, so it needs support -- inside a closed
    cavity, where it can never be removed."""
    return bd.Pos(0, 0, 6) * bd.Box(24, 24, 12) - bd.Pos(0, 0, 6) * bd.Box(14, 14, 4)


def tight_lip(gap=2.0):
    """A base plate, a back wall, and a top plate reaching out over the base
    ``gap`` mm above it: the top plate's underside needs support, squeezed
    into a gap ``gap`` mm high."""
    base = bd.Pos(15, 0, 2) * bd.Box(30, 20, 4)
    wall = bd.Pos(2, 0, 4 + gap / 2) * bd.Box(4, 20, gap)
    top = bd.Pos(15, 0, 4 + gap + 2) * bd.Box(30, 20, 4)
    return base + wall + top


def round_bar():
    """A round bar lying along Y: declared with the build direction +Z it
    touches the bed along a line only, nothing flat to stand on."""
    return bd.Pos(0, 0, 5) * bd.Cylinder(5, 30, rotation=(90, 0, 0))


# --------------------------------------------- powder bed fusion parts


def thin_fin(th):
    """A 2 mm plate with a fin standing on it (thin base, so the change of
    section at the fin's root stays gentle)."""
    return plate(t=2.0) + bd.Pos(0, 0, 2 + 4) * bd.Box(th, 16, 8)


def thin_pin(dia):
    return plate(t=2.0) + bd.Pos(0, 0, 2 + 3) * bd.Cylinder(dia / 2, 6)


def boss(w, h=8.0):
    """A 2 mm plate with a square boss: the section jumps from 2 mm to w."""
    return plate(40, 30, 2.0) + bd.Pos(0, 0, 2 + h / 2) * bd.Box(w, w, h)


def box_hollow(holes):
    """A 30 x 30 x 20 box with a 20 x 20 x 10 hollow inside, opened by round
    holes from the top (diameters in ``holes``)."""
    body = bd.Pos(0, 0, 10) * bd.Box(30, 30, 20) - bd.Pos(0, 0, 10) * bd.Box(20, 20, 10)
    xs = [-5.0, 5.0, 0.0]
    for x, d in zip(xs, holes):
        body = body - bd.Pos(x, 0, 17.5) * bd.Cylinder(d / 2, 5.2)
    return body


# ------------------------------------------------------------- the rows


FDM, IM, PBF = "fdm", "moulding", "pbf"

CASES = [
    # (label, build, process, direction, rule that must fail or None)
    ("FDM base plate", lambda: plate(), FDM, Z, None),
    ("fin 0.9 mm (over 0.8)", lambda: fin(0.9), FDM, Z, None),
    ("fin 0.7 mm (under 0.8)", lambda: fin(0.7), FDM, Z, "FDM_WALL_MIN"),
    ("overhang 40 deg", lambda: overhang(40), FDM, Z, None),
    ("overhang 50 deg", lambda: overhang(50), FDM, Z, "FDM_OVERHANG"),
    ("shelf with nothing below", shelf, FDM, Z, "FDM_OVERHANG"),
    ("bridge 8 mm (under 10)", lambda: bridge(8), FDM, Z, None),
    ("bridge 12 mm (over 10)", lambda: bridge(12), FDM, Z, "FDM_BRIDGE"),
    ("hole 2.4 mm (over 2)", lambda: holed(2.4), FDM, Z, None),
    ("hole 1.6 mm (under 2)", lambda: holed(1.6), FDM, Z, "FDM_HOLE_MIN"),
    ("pin 3.5 mm (over 3)", lambda: pinned(3.5), FDM, Z, None),
    ("pin 2.5 mm (under 3)", lambda: pinned(2.5), FDM, Z, "FDM_PIN_MIN"),
    ("gap 0.7 mm (over 0.5)", lambda: slotted(0.7), FDM, Z, None),
    ("gap 0.4 mm (under 0.5)", lambda: slotted(0.4), FDM, Z, "FDM_GAP_MIN"),
    ("foot 64 mm2 (over 50)", lambda: foot(4.5), FDM, Z, None),
    ("foot 38 mm2 (under 50)", lambda: foot(3.5), FDM, Z, "FDM_BED_CONTACT"),
    ("pillar 25 mm on a 10 mm stance (2.5 x)", lambda: pillar(25), FDM, Z, None),
    ("pillar 35 mm on a 10 mm stance (3.5 x)", lambda: pillar(35), FDM, Z, "FDM_STANCE"),
    ("open pocket", lambda: hollow(False), FDM, Z, None),
    ("sealed hollow", lambda: hollow(True), FDM, Z, "FDM_SEALED_HOLLOW"),
    ("overhang 50 deg, direction chosen", lying_overhang, FDM, None, None),
    ("plus-section bar, tilted, direction chosen", plus_bar, FDM, None, None),
    ("overhang 50 deg, declared lying on its side", lambda: overhang(50), FDM, (0, 1, 0), None),
    ("round bar declared on its curved side", round_bar, FDM, Z, "FDM_BED_CONTACT|FDM_OVERHANG|FDM_STANCE"),
    ("shelf, support allowed: reachable", shelf, FDM, Z, None, {"support": "allowed"}),
    ("cavity ceiling 14 mm, support allowed", cavity_ceiling, FDM, Z,
     "FDM_SUPPORT_REMOVABLE|FDM_SEALED_HOLLOW", {"support": "allowed"}),
    ("cavity ceiling 14 mm, support forbidden", cavity_ceiling, FDM, Z,
     "FDM_SUPPORT_REMOVABLE|FDM_SEALED_HOLLOW|FDM_BRIDGE|FDM_OVERHANG"),
    ("lip over a 2 mm gap, support allowed", lambda: tight_lip(2.0), FDM, Z, "FDM_SUPPORT_REMOVABLE",
     {"support": "allowed"}),
    ("lip over a 5 mm gap, support allowed", lambda: tight_lip(5.0), FDM, Z, None, {"support": "allowed"}),
    ("drafted plate 2.5 mm", lambda: tray(), IM, Z, None),
    ("draft 1.5 deg (over 1)", lambda: tray(taper=1.5), IM, Z, None),
    ("draft 0 deg (under 1)", lambda: tray(taper=0.0), IM, Z, "IM_DRAFT"),
    ("draft 0.5 deg (under 1)", lambda: tray(taper=0.5), IM, Z, "IM_DRAFT"),
    ("waist out: parting line", lambda: hourglass(False), IM, Z, None),
    ("waist in: undercut", lambda: hourglass(True), IM, Z, "IM_UNDERCUT"),
    ("wall 1.2 mm (over 1)", lambda: tray(1.2), IM, Z, None),
    ("wall 0.8 mm (under 1)", lambda: tray(0.8), IM, Z, "IM_WALL_MIN"),
    ("wall 3.6 mm (under 4)", lambda: tray(3.6), IM, Z, None),
    ("wall 4.5 mm (over 4)", lambda: tray(4.5), IM, Z, "IM_WALL_MAX"),
    ("thick spot 1.3 x wall", lambda: cap(0.6), IM, Z, None),
    ("thick spot 1.8 x wall", lambda: cap(1.6), IM, Z, "IM_WALL_UNIFORM"),
    ("rib 1.3 mm on 2.5 wall (0.52)", lambda: ribbed(1.3), IM, Z, None),
    ("rib 2.0 mm on 2.5 wall (0.80)", lambda: ribbed(2.0), IM, Z, "IM_RIB_RATIO"),
    ("rib base sharp", lambda: ribbed(1.3, fillet=0), IM, Z, "IM_SHARP_CORNER"),
    ("rib fillet 1.0 mm (0.4 x wall)", lambda: ribbed(1.3, fillet=1.0), IM, Z, "IM_SHARP_CORNER"),
    ("drafted plate, direction chosen", lambda: tray(), IM, None, None),
    ("drafted plate, pull declared sideways", lambda: tray(), IM, (1, 0, 0), "IM_DRAFT|IM_UNDERCUT"),
    ("PBF base plate", lambda: plate(), PBF, None, None),
    ("fin 0.9 mm (over 0.8)", lambda: thin_fin(0.9), PBF, None, None),
    ("fin 0.6 mm (under 0.8)", lambda: thin_fin(0.6), PBF, None, "PBF_WALL_MIN|PBF_TRANSITION"),
    ("open pocket", lambda: hollow(False), PBF, None, None),
    ("sealed hollow", lambda: hollow(True), PBF, None, "PBF_SEALED_HOLLOW"),
    ("hollow, two 6 mm escape holes", lambda: box_hollow((6, 6)), PBF, None, None),
    ("hollow, one 6 and one 3 mm hole", lambda: box_hollow((6, 3)), PBF, None, "PBF_ESCAPE_HOLES"),
    ("hole 1.8 mm (over 1.5)", lambda: holed(1.8), PBF, None, None),
    ("hole 1.2 mm (under 1.5)", lambda: holed(1.2), PBF, None, "PBF_HOLE_MIN"),
    ("pin 1.0 mm (over 0.8)", lambda: thin_pin(1.0), PBF, None, None),
    ("pin 0.6 mm (under 0.8)", lambda: thin_pin(0.6), PBF, None, "PBF_PIN_MIN|PBF_TRANSITION|PBF_WALL_MIN"),
    ("gap 0.7 mm (over 0.5)", lambda: slotted(0.7), PBF, None, None),
    ("gap 0.3 mm (under 0.5)", lambda: slotted(0.3), PBF, None, "PBF_GAP_MIN"),
    ("block 14 mm (under 15)", lambda: plate(30, 20, 14), PBF, None, None),
    ("block 17 mm (over 15)", lambda: plate(30, 20, 17), PBF, None, "PBF_SECTION_MAX"),
    ("boss 5 mm on 2 mm plate (x2.5)", lambda: boss(5), PBF, None, None),
    ("boss 8 mm on 2 mm plate (x4)", lambda: boss(8), PBF, None, "PBF_TRANSITION"),
    ("plate 60 x 2 mm (x30)", lambda: plate(60, 20, 2), PBF, None, None),
    ("plate 100 x 2 mm (x50)", lambda: plate(100, 20, 2), PBF, None, "PBF_WARP_FLAT"),
]


@dataclass
class Row:
    label: str
    process: str
    expect: str | None
    ok: bool = False
    failed: list = field(default_factory=list)
    value: str = ""
    direction: str = ""


def run(cases=CASES, progress=None) -> list[Row]:
    rows = []
    with tempfile.TemporaryDirectory() as tmp:
        for case in cases:
            label, build, process, direction, expect = case[:5]
            kw = case[5] if len(case) > 5 else {}
            path = os.path.join(tmp, "part.step")
            write_step(build(), path)
            rep = check(path, process, direction, **kw)
            failed = [x.id for x in rep.rules if not x.ok and x.required]  # reported-only rules do not count
            if expect is None:
                ok = rep.passed
            else:
                allowed = set(expect.split("|"))
                ok = bool(failed) and set(failed) <= allowed and failed[0] in allowed
            row = Row(label, process, expect, ok, failed)
            key = (expect or "").split("|")[0]
            hit = next((x for x in rep.rules if x.id == key), None)
            if hit is not None and hit.value is not None:
                u = "" if hit.unit == "ratio" else f" {hit.unit}"
                row.value = f"{hit.value:g}{u}"
            row.direction = ",".join(f"{c:g}" for c in rep.direction)
            rows.append(row)
            if progress:
                progress(label)
    return rows


def table(rows: list[Row]) -> str:
    w = max(len(r.label) for r in rows)
    out = [f"{'part':<{w}}  {'process':<8}  {'must':<28} {'measured':>12}  {'ok':<3} failed",
           f"{'-' * w}  {'-' * 8}  {'-' * 28} {'-' * 12}  {'-' * 3} {'-' * 6}"]
    for r in rows:
        must = "pass" if r.expect is None else "fail " + r.expect.replace("|", " or ")
        mark = "yes" if r.ok else "NO "
        out.append(f"{r.label:<{w}}  {r.process:<8}  {must:<28} {r.value:>12}  {mark} "
                   f"{', '.join(r.failed) or '-'}")
    return "\n".join(out)


def pictures(out_dir: str, cases=CASES) -> list[str]:
    """One picture per rule, from the first self-test part that must fail it."""
    from .viz import render

    done, paths = set(), []
    with tempfile.TemporaryDirectory() as tmp:
        for case in cases:
            label, build, process, direction, expect = case[:5]
            kw = case[5] if len(case) > 5 else {}
            key = expect.split("|")[0] if expect else None
            if key is None or key in done:
                continue
            path = os.path.join(tmp, f"{key.lower()}.step")
            write_step(build(), path)
            rep = check(path, process, direction, **kw)
            rep.file = label
            x = next(x for x in rep.rules if x.id == key)
            paths.append(render(rep, rep.part, x, os.path.join(out_dir, f"{key}.png")))
            done.add(key)
    return paths


# --------------------------------------------- level-1 reference figure


def level1_refs(task_dirs, progress=None) -> list[dict]:
    """Check the reference answer (``_solution/reference.step``) of each
    generated level-1 task folder against each process, direction chosen by
    the checker. Tells how far today's answers are from being makeable."""
    res = []
    for d in task_dirs:
        path = os.path.join(d, "_solution", "reference.step")
        row = {"task": os.path.basename(os.path.normpath(d))}
        for proc in (FDM, IM):
            rep = check(path, proc)
            row[proc] = {"passed": rep.passed, "failed": rep.failed(),
                         "direction": rep.direction, "chosen": rep.direction_chosen}
        res.append(row)
        if progress:
            progress(row["task"])
    return res


def level1_table(res: list[dict]) -> str:
    out = [f"{'task':<8}  {'fdm':<6} {'fdm fails':<28}  {'moulding':<8} moulding fails",
           f"{'-' * 8}  {'-' * 6} {'-' * 28}  {'-' * 8} {'-' * 14}"]
    for r in res:
        f, m = r[FDM], r[IM]
        out.append(f"{r['task']:<8}  {'pass' if f['passed'] else 'fail':<6} "
                   f"{', '.join(x.replace('FDM_', '') for x in f['failed']) or '-':<28}  "
                   f"{'pass' if m['passed'] else 'fail':<8} "
                   f"{', '.join(x.replace('IM_', '') for x in m['failed']) or '-'}")
    n = len(res)
    out.append(f"\nas they are: {sum(r[FDM]['passed'] for r in res)}/{n} printable (FDM), "
               f"{sum(r[IM]['passed'] for r in res)}/{n} mouldable")
    return "\n".join(out)
