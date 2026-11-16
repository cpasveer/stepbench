"""SECTION_MIN: the material joining any two mounts never gets too thin.

Loads are deliberately not modelled, so this is a geometric stand-in for
strength: the smallest *flat* cut that separates one mount's bolts from
another's must keep at least a required area.

Required area, per pair of mounts: ``section_factor`` times the combined
shank cross-section (pi d^2 / 4 per bolt) of whichever of the two mounts has
the smaller total. Bigger bolts demand a sturdier connection.

How the smallest cut is found. For every pair of mounts we try cutting
planes in many directions: 26 spread evenly over the sphere, the axis of
every cylindrical face of the part (so a rod is cut square to its length),
the normals of its 12 largest flat faces, the directions of all its straight
edges of 5 mm or more (a member's long edges run along it), and the line between
the two mounts' bolt centres. A plane counts only if it separates the pair: every
bolt of one mount on one side, every bolt of the other on the other side.
Along each direction the plane is stepped 2 mm at a time across the gap;
around the three smallest cuts found the search is repeated at 0.25 mm
steps. The area of the part in that plane is the section. (For speed the
coarse planes are first ranked on a fine triangle mesh of the part, and
only those near the smallest are measured exactly; see ``_meshcut``.)

How exact it is. Each section area is exact (a boolean of the solid with a
plane). The search is not: it is the smallest of the planes tried, so the
true thinnest flat cut can lie up to about 0.25 mm from the one reported,
and a neck that is thinnest only along a direction not tried (no sphere
direction, cylinder axis or face normal near it) is measured too thick. Single members are also cut on their own: planes square to the part's
long edges, cylinder axes and face normals are stepped across the whole
part; each section region smaller than the best whole-plane cut is tested
on the triangle mesh (cutting the surface along that region's outline, do
the two mounts' seats end up on different pieces?); the smallest that
passes is refined and confirmed exactly by cutting a 0.02 mm wafer out of
the solid there; if the two mounts'
seats then sit in different pieces, that region is a cut, and its exact
area counts. This finds a rod or arm whose square cut does not put the
bolts neatly on two sides of one plane, and the arm to one of three mounts.
What it cannot see: a weak spot that no flat cut isolates (a twisted or
curved neck), and a connection that is only weak when two members are cut
together where no single plane separates the mounts.
"""
from __future__ import annotations

import math

import build123d as bd
import numpy as np
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder, GeomAbs_Plane

from ..catalogue import BOLTS
from .base import CheckResult, Context, area

NAME = "SECTION_MIN"
DEFAULT_FACTOR = 2.0


def _sphere(n: int):
    out = []
    for i in range(n):
        z = 1 - (i + 0.5) / n
        r = math.sqrt(max(0.0, 1 - z * z))
        a = i * math.pi * (3 - math.sqrt(5))
        out.append(bd.Vector(r * math.cos(a), r * math.sin(a), z))
    return out


def _directions(solid, ca: bd.Vector, cb: bd.Vector, sphere: bool = True):
    dirs = _sphere(26) if sphere else []
    planes = []
    for f in solid.faces():
        ad = BRepAdaptor_Surface(f.wrapped)
        t = ad.GetType()
        if t == GeomAbs_Cylinder:
            a = ad.Cylinder().Axis().Direction()
            dirs.append(bd.Vector(a.X(), a.Y(), a.Z()))
        elif t == GeomAbs_Plane:
            planes.append((-round(f.area, 6), len(planes), bd.Vector(*f.normal_at())))
    dirs += [n for _, _, n in sorted(planes)[:12]]
    # the longest straight edges: a member's long edges (and a rod's seam)
    # run along it, so a cut square to them is a square cut of the member
    from OCP.BRepAdaptor import BRepAdaptor_Curve
    from OCP.GeomAbs import GeomAbs_Line

    lines, seen_l = [], set()
    for e in solid.edges():
        if BRepAdaptor_Curve(e.wrapped).GetType() == GeomAbs_Line:
            p0, p1 = e.position_at(0), e.position_at(1)
            if (p1 - p0).length >= 5.0:
                v = (p1 - p0).normalized()
                if v.Z < 0 or (v.Z == 0 and (v.Y < 0 or (v.Y == 0 and v.X < 0))):
                    v = -v
                key = (round(v.X, 2), round(v.Y, 2), round(v.Z, 2))  # within about half a degree
                if key not in seen_l:
                    seen_l.add(key)
                    lines.append((-round((p1 - p0).length, 6), len(lines), v))
    dirs += [v for _, _, v in sorted(lines)]
    if (cb - ca).length > 1e-9:
        dirs.append((cb - ca).normalized())
    # one of each direction / its opposite, in a fixed order
    out, seen = [], set()
    for u in dirs:
        u = u.normalized()
        if u.Z < 0 or (u.Z == 0 and (u.Y < 0 or (u.Y == 0 and u.X < 0))):
            u = -u
        key = (round(u.X, 4), round(u.Y, 4), round(u.Z, 4))
        if key not in seen:
            seen.add(key)
            out.append(u)
    return out


def _section(solid, u: bd.Vector, k: float, size: float):
    pl = bd.Plane(origin=tuple(u * k), z_dir=tuple(u))
    sec = solid & (pl * bd.Rectangle(size, size))
    return area(sec), sec


def outline(solid, at, normal, points: int = 96) -> list[list[list[float]]]:
    """The edges of the cut through ``solid`` in the plane through ``at``
    with ``normal``, as polylines of world points: the region of the cut
    nearest ``at`` -- its outer edge and any holes in it. For drawing; the
    area is measured elsewhere, exactly."""
    u = bd.Vector(*normal).normalized()
    c = bd.Vector(*at)
    size = 4 * max(solid.bounding_box().size) + 10
    pl = bd.Plane(origin=tuple(c), z_dir=tuple(u))
    sec = solid & (pl * bd.Rectangle(size, size))
    faces = sec.faces() if sec is not None else []
    if not faces:
        return []
    # nearest region; equally near ones (a symmetric part) by position, so the drawing is the same every time
    f = min(faces, key=lambda f: (round((f.center() - c).length, 6), *(round(v, 6) for v in f.center())))
    out = []
    for w in f.wires():
        n = max(8, min(points, int(w.length / 0.5)))
        pts = [w.position_at(i / n) for i in range(n + 1)]
        out.append([[round(p.X, 3), round(p.Y, 3), round(p.Z, 3)] for p in pts])
    return out


def _seat_points(bolts):
    """Points just inside the part on each bolt's seat ring (on the mounting
    face side), used to tell which piece of a cut part holds which mount."""
    out = []
    for b in bolts:
        ax = bd.Vector(*b.axis)
        a = bd.Vector(1, 0, 0) if abs(ax.X) < 0.9 else bd.Vector(0, 1, 0)
        v = ax.cross(a).normalized()
        r = BOLTS[b.size].clear_coarse / 2 + 1.0
        for sgn in (1, -1):
            out.append(bd.Vector(*b.position) + v * (sgn * r) + ax * 0.05)
    return out


def _region_separates(solid, u, k, centre, seats_a, seats_b, size):
    """Cut a 0.02 mm wafer out of the part exactly where one section region
    lies; True (and that region's exact area) if the two mounts' seats then
    sit in different pieces."""
    pl = bd.Plane(origin=tuple(u * k), z_dir=tuple(u))
    sec = solid & (pl * bd.Rectangle(size, size))
    faces = sec.faces() if sec is not None else []
    if not faces:
        return False, 0.0
    c = bd.Vector(*centre)
    # nearest region; equally near ones (a symmetric part) by position, so the drawing is the same every time
    f = min(faces, key=lambda f: (round((f.center() - c).length, 6), *(round(v, 6) for v in f.center())))
    wafer = bd.extrude(f, amount=0.01, dir=tuple(u), both=True)
    rest = solid - wafer
    pieces = rest.solids()
    if len(pieces) < 2:
        return False, float(f.area)

    def owner(pt):
        d = [(pc.distance_to(pt), i) for i, pc in enumerate(pieces)]
        dist, i = min(d)
        return i if dist < 0.2 else None

    oa = {owner(p) for p in seats_a} - {None}
    ob = {owner(p) for p in seats_b} - {None}
    return bool(oa and ob and not (oa & ob)), float(f.area)


def smallest_cut(solid, pa, pb, step: float = 2.0, fine: float = 0.25, seats_a=None, seats_b=None):
    """Smallest separating flat section between point sets pa and pb.
    Returns (area, centre, normal) or None if no tried plane separates them.

    Two kinds of cut are considered: a whole plane that puts every bolt of
    one mount on one side and every bolt of the other on the other side;
    and, when seat points are given, a single region of a plane's section
    (one member cut through) whose removal disconnects the two mounts."""
    ca = sum(pa, bd.Vector()) * (1 / len(pa))
    cb = sum(pb, bd.Vector()) * (1 / len(pb))
    size = 4 * max(solid.bounding_box().size) + 10
    # Coarse planes are ranked on a fine triangle mesh of the part (fast,
    # within ~0.1% of exact); only planes whose mesh area is close to the
    # three smallest are then measured exactly. The result is the same as
    # measuring every coarse plane exactly whenever the mesh is within 2%.
    from ._meshcut import areas as mesh_areas, mesh

    T = mesh(solid)
    coarse = []  # (mesh area, order, direction, offset, lo, hi)
    for i, u in enumerate(_directions(solid, ca, cb)):
        a_, b_ = [p.dot(u) for p in pa], [p.dot(u) for p in pb]
        if max(a_) < min(b_):
            lo, hi = max(a_), min(b_)
        elif max(b_) < min(a_):
            lo, hi = max(b_), min(a_)
        else:
            continue
        ks, k = [], lo + min(step, hi - lo) / 2
        while k < hi:
            ks.append(k)
            k += step
        for k, am in zip(ks, mesh_areas(T, tuple(u), ks)):
            coarse.append((float(am), i, u, k, lo, hi))
    live = sorted(c for c in coarse if c[0] > 1e-6)
    if not live:
        return None
    cut = live[min(2, len(live) - 1)][0] * 1.02 + 0.5
    found = []  # (exact area, order, direction, offset, lo, hi)
    for am, i, u, k, lo, hi in coarse:
        if am <= cut:
            ar, _ = _section(solid, u, k, size)
            if ar > 1e-9:
                found.append((ar, i, u, k, lo, hi))
    if not found:
        return None
    found.sort(key=lambda f: (round(f[0], 6), f[1], f[3]))  # areas equal to 1e-6 mm2 are a tie: kernel noise must not decide
    best = None
    for ar, i, u, k0, lo, hi in found[:3]:
        k = max(lo, k0 - step)
        while k <= min(hi, k0 + step):
            a2, _ = _section(solid, u, k, size)
            if a2 > 1e-9 and (best is None or round(a2, 6) < round(best[0], 6)):
                best = (a2, u, k)
            k += fine
        if best is None or round(ar, 6) < round(best[0], 6):
            best = (ar, u, k0)
    ar, u, k0 = best
    _, sec = _section(solid, u, k0, size)
    result = (ar, tuple(sec.center()), tuple(u))
    if seats_a and seats_b:
        reg = _smallest_region_cut(solid, ca, cb, seats_a, seats_b, step, size, below=result[0])
        if reg is not None and reg[0] < result[0]:
            result = reg
    return result


def _smallest_region_cut(solid, ca, cb, seats_a, seats_b, step, size, confirm: int = 4,
                         below: float = float("inf")):
    """Single-member cuts: every section region of planes across the part,
    tested first on the mesh (does cutting the surface along this region's
    outline put the two mounts' seats in different pieces?), smallest
    passing region first, refined, then confirmed exactly by cutting a thin
    wafer out of the solid there."""
    from ._meshcut import Surface, mesh_indexed, regions

    M = mesh_indexed(solid, 0.05)  # ranking only; the area reported is exact
    V = M[0]
    surf = Surface(M)
    ta, tb = surf.nearest([tuple(p) for p in seats_a]), surf.nearest([tuple(p) for p in seats_b])
    cands = []
    # members are found along their own directions: long edges, cylinder
    # axes, face normals and the line between the mounts (no sphere points)
    for i, u in enumerate(_directions(solid, ca, cb, sphere=False)):
        uu = tuple(u)
        proj = V @ np.array(uu)
        lo, hi = float(proj.min()), float(proj.max())  # the whole part along u
        k = lo + step / 2
        while k < hi:
            for ar, c, tris in regions(M, uu, k, with_tris=True):
                # only a region smaller than the best whole-plane cut can
                # change the answer, so only those are tested
                if ar < below * 1.02 and surf.separates(tris, ta, tb):
                    cands.append((float(ar), i, k, u, tuple(c)))
            k += step
    cands.sort(key=lambda x: (x[0], x[1], x[2]))
    tried = []
    for ar, i, k, u, c in cands:
        if len(tried) >= confirm:
            break
        cv = bd.Vector(*c)
        if any((cv - t).length < 3.0 for t in tried):
            continue  # the same member, already tried
        tried.append(cv)
        # refine along this direction on the mesh, following the same region
        best_k, best_a, best_c = k, ar, c
        kk = k - step
        while kk <= k + step:
            for a2, c2, tris in regions(M, tuple(u), kk, with_tris=True):
                if (bd.Vector(*c2) - cv).length < 5.0 and a2 < best_a and surf.separates(tris, ta, tb):
                    best_k, best_a, best_c = kk, a2, c2
            kk += 0.25
        ok, exact = _region_separates(solid, u, best_k, best_c, seats_a, seats_b, size)
        if ok:
            return exact, tuple(best_c), tuple(u)
    return None


def required_area(task, a: str, b: str) -> float:
    f = task.tolerances.get("section_factor", DEFAULT_FACTOR)
    tot = {n: sum(math.pi * BOLTS[x.size].d ** 2 / 4 for x in task.bolts(0) if x.part == n) for n in (a, b)}
    return f * min(tot.values())


def run(ctx: Context) -> CheckResult:
    t = ctx.task
    names = [n for n, itf in t.parts.items() if itf.holes]  # parts held by bolts (a bearing is not)
    rows, problems, worst = [], [], None
    for k in ctx.poses():
        pts = {n: [bd.Vector(*x.position) for x in t.bolts(ctx.stored(k)) if x.part == n] for n in names}
        for i, a in enumerate(names):
            for b in names[i + 1:]:
                need = required_area(t, a, b)
                sa = _seat_points([x for x in t.bolts(ctx.stored(k)) if x.part == a])
                sb = _seat_points([x for x in t.bolts(ctx.stored(k)) if x.part == b])
                cut = smallest_cut(ctx.solid_at(k), pts[a], pts[b], seats_a=sa, seats_b=sb)
                if cut is None:
                    problems.append(f"{a}-{b}: no flat cut separates the two mounts; cannot measure")
                    continue
                ar, c, u = cut
                ratio = ar / need
                row = {"pair": f"{a}-{b}", "area": round(ar, 3), "required": round(need, 3),
                       "at": [round(v, 2) for v in c], "normal": [round(v, 4) for v in u]}
                try:  # the cut's real edges, for drawing it
                    row["outline"] = outline(ctx.solid_at(k), c, u)
                except Exception:  # noqa: BLE001 - a drawing aid only: the verdict stands without it
                    pass
                rows.append(row)
                worst = ratio if worst is None else min(worst, ratio)
                if ar < need:
                    problems.append(f"{a}-{b}: thinnest section {ar:.1f} mm2 at ({c[0]:.1f}, {c[1]:.1f}, {c[2]:.1f}), "
                                    f"need {need:.1f} mm2")
    value = None if worst is None else round(worst, 4)
    if problems:
        more = f" (+{len(problems) - 1} more)" if len(problems) > 1 else ""
        return CheckResult(NAME, False, problems[0] + more, value, "ratio", {"sections": rows, "problems": problems})
    w = min(rows, key=lambda r: r["area"] / r["required"])
    return CheckResult(NAME, True, f"thinnest section {w['area']:.1f} mm2 between {w['pair']} "
                       f"(need {w['required']:.1f})", value, "ratio", {"sections": rows})
