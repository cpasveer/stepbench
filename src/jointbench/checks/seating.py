"""SEATING: the part sits flat on each mounting face, around every bolt.

A bolted joint carries load through the ring of material the bolt clamps.
So for each bolt we take the bearing ring on the fixed part's face -- from
the largest allowed clearance hole out to the bolt-head diameter, and only
where the fixed part actually has face -- and measure how much of it is
touched by a flat face of the submitted part. "Touched" means a planar face,
facing the fixed part, lying on the mounting plane or standing off it by no
more than ``seat_gap_max``.
"""
from __future__ import annotations

import math

import build123d as bd
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Plane

from ..catalogue import BOLTS
from .base import CheckResult, Context, area

NAME = "SEATING"


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


# a seat may sink this far into the fixed part and still count: rounding in
# a STEP written by another program, at an arbitrary orientation, is ~1e-5 mm.
# Real penetration is NO_INTERFERENCE's job, which measures it as volume.
PENETRATION_NOISE = 1e-3


def _planar_faces(solid, normal, origin, lo, hi, parallel=1e-6):
    """Planar faces whose outward normal is ``normal`` (within ``parallel``)
    and whose every vertex lies at signed distance lo..hi from ``origin``
    along ``normal``. Returns (face, centre distance) pairs, plus the
    nearest centre distance of any parallel planar face."""
    out, nearest = [], None
    o = _dot(origin, normal)
    for f in solid.faces():
        ad = BRepAdaptor_Surface(f.wrapped)
        if ad.GetType() != GeomAbs_Plane:
            continue
        nf = tuple(f.normal_at())
        if _dot(nf, normal) < 1 - parallel:
            continue
        dist = _dot(tuple(f.center()), normal) - o
        ds = [_dot(tuple(v), normal) - o for v in f.vertices()] or [dist]
        if nearest is None or abs(dist) < abs(nearest):
            nearest = dist
        if lo <= min(ds) and max(ds) <= hi:
            out.append((f, dist))
    return out, nearest


def _onto_plane(face, normal, dist):
    """Rotate a nearly parallel seat face so it faces exactly against
    ``normal`` (the mounting face's normal) and move it ``dist`` back onto
    the plane, so the 2D overlap is measured, not 3D noise."""
    nf = bd.Vector(*face.normal_at())
    target = -bd.Vector(*normal)
    axis = nf.cross(target)
    c = face.center()
    if axis.length > 1e-15:
        ang = math.degrees(math.atan2(axis.length, nf.dot(target)))
        face = face.rotate(bd.Axis(c, axis.normalized()), ang)
    return face.moved(bd.Location(tuple(-v * dist for v in normal)))


def contact_ratios(ctx: Context, pose: int):
    """Per bolt: (bolt id, contact ratio, nearest seat-face stand-off)."""
    t = ctx.task
    gap = t.tolerances["seat_gap_max"]
    fixed = ctx.fixed(pose)
    out = []
    for face in t.faces(ctx.stored(pose)):
        n = face.normal
        neg = tuple(-v for v in n)
        # seat candidates on the submission: facing the fixed part (normal
        # -n), every point standing off the face by 0..gap along +n.
        # Distances are measured along -n, so a stand-off s reads as -s.
        seats, nearest = _planar_faces(ctx.solid_at(pose), neg, face.origin, -gap, PENETRATION_NOISE)
        fixed_faces, _ = _planar_faces(fixed[face.part], n, face.origin, -1e-4, 1e-4)
        plane = bd.Plane(origin=face.origin, z_dir=n)
        flat = [_onto_plane(sf, n, -dist) for sf, dist in seats]
        for bolt in [b for b in t.bolts(ctx.stored(pose)) if b.part == face.part]:
            b = BOLTS[bolt.size]
            disc = bd.Face(bd.Wire.make_circle(b.head_d / 2)) - bd.Face(bd.Wire.make_circle(b.clear_coarse / 2))
            region = []
            for ff, _ in fixed_faces:
                # measured in that face's own plane, with its own normal: where the face and the bolt
                # come from different sources (a joint stated by the answer in rounded numbers, level 4)
                # they may be a hair apart in place or direction, and two faces a hair apart share no area
                nf = tuple(ff.normal_at())
                o = tuple(ff.center())
                off = sum((bolt.position[i] - o[i]) * nf[i] for i in range(3))
                p = tuple(bolt.position[i] - off * nf[i] for i in range(3))
                r = bd.Plane(origin=p, x_dir=plane.x_dir, z_dir=nf) * disc & ff
                if area(r) > 0:
                    negf = tuple(-x for x in nf)
                    on_ff = [_onto_plane(sf, nf, -(sum(tuple(sf.center())[i] * negf[i] for i in range(3))
                                                   - sum(o[i] * negf[i] for i in range(3)))) for sf, _ in seats]
                    region.append((r, on_ff))
            region_area = sum(area(r) for r, _ in region)
            touched = 0.0
            for r, on_ff in region:
                for sf in on_ff:
                    touched += area(sf & r)
            ratio = touched / region_area if region_area > 0 else 0.0
            out.append((bolt.id, min(ratio, 1.0), None if nearest is None else -nearest))
    out += _curved(ctx, pose, gap)
    return out


def _coaxial_cylinders(solid, c0, axis, r_lo, r_hi, axis_tol=1e-3, line_tol=0.05):
    """Cylindrical faces of ``solid`` on the line through ``c0`` along ``axis``,
    radius r_lo..r_hi."""
    from OCP.GeomAbs import GeomAbs_Cylinder

    out = []
    for f in solid.faces():
        ad = BRepAdaptor_Surface(f.wrapped)
        if ad.GetType() != GeomAbs_Cylinder:
            continue
        cy = ad.Cylinder()
        if not (r_lo <= cy.Radius() <= r_hi):
            continue
        d = cy.Axis().Direction()
        u = bd.Vector(d.X(), d.Y(), d.Z())
        if abs(abs(u.dot(axis)) - 1) > axis_tol:
            continue
        lp = cy.Axis().Location()
        q = bd.Vector(lp.X(), lp.Y(), lp.Z()) - c0
        if (q - axis * q.dot(axis)).length > line_tol:
            continue
        out.append(f)
    return out


def _curved(ctx: Context, pose: int, gap: float):
    """Per bolt of a pipe fixture: the share of its bearing ring on the pipe's
    surface (from the largest clearance hole out to the head diameter) that a
    coaxial cylindrical face of the part touches, standing off at most ``gap``."""
    t = ctx.task
    out = []
    fixed = ctx.fixed(pose)
    for name, itf in t.parts.items():
        if itf.kind != "pipe":
            continue
        pl = t.poses[ctx.stored(pose)][name]
        c0 = bd.Vector(*pl.point((0.0, 0.0, 0.0)))
        axis = bd.Vector(*pl.vec((1.0, 0.0, 0.0))).normalized()
        R = itf.face_h / 2
        seats = _coaxial_cylinders(ctx.solid_at(pose), c0, axis, R - PENETRATION_NOISE, R + gap)
        surface = _coaxial_cylinders(fixed[name], c0, axis, R - 1e-4, R + 1e-4)
        long = 4 * itf.face_w + 100
        shell = (bd.Plane(origin=tuple(c0), z_dir=tuple(axis)) * (bd.Cylinder(R + gap + 0.05, long) - bd.Cylinder(R - 0.05, long + 1)))
        for bolt in [b for b in t.bolts(ctx.stored(pose)) if b.part == name]:
            b = BOLTS[bolt.size]
            bp = bd.Plane(origin=bolt.position, z_dir=bolt.axis)
            ring = shell & (bp * (bd.Cylinder(b.head_d / 2, 20) - bd.Cylinder(b.clear_coarse / 2, 21)))
            region_area = sum(area(f & ring) for f in surface)
            touched = sum(area(f & ring) for f in seats)
            ratio = touched / region_area if region_area > 0 else 0.0
            out.append((bolt.id, min(ratio, 1.0), None if seats else None))
    return out


def run(ctx: Context) -> CheckResult:
    need = ctx.task.tolerances["seat_contact_min"]
    gap = ctx.task.tolerances["seat_gap_max"]
    rows, problems = [], []
    for k in ctx.poses():
        for bid, ratio, nearest in contact_ratios(ctx, k):
            rows.append({"bolt": bid, "contact": round(ratio, 4)})
            if ratio < need:
                why = ""
                if ratio == 0 and nearest is not None and nearest > gap:
                    why = f"; nearest flat face stands {nearest:.3f} mm off the mounting face"
                elif ratio == 0 and nearest is not None and nearest < 0:
                    why = f"; nearest flat face is sunk {-nearest:.3f} mm into the fixed part"
                elif ratio == 0 and nearest is None:
                    why = "; no flat face facing this mounting face"
                problems.append(f"{bid}: {ratio:.0%} of the bearing ring touches the face "
                                f"(need {need:.0%}){why}")
    worst = round(min(r["contact"] for r in rows), 4) if rows else 0.0
    if problems:
        extra = f" (+{len(problems) - 1} more)" if len(problems) > 1 else ""
        return CheckResult(NAME, False, problems[0] + extra, worst, "ratio",
                           {"bolts": rows, "problems": problems})
    return CheckResult(NAME, True, f"seated on every mounting face; worst bolt ring {worst:.0%} in contact "
                       f"(need {need:.0%})", worst, "ratio", {"bolts": rows})
