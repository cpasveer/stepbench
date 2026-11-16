"""Finding a submitted part's joints from its own geometry.

The solid is first brought into mechanism-plane coordinates (z along the
plane normal). Then two kinds of feature are read off the exact boundary:

* **round**: cylindrical faces whose axis is parallel to the normal. The
  face's orientation says which side the material is on, so a bore (hole)
  and a solid pin are told apart without guessing.
* **straight**: pairs of flat faces whose normals lie in the plane and
  point in opposite directions. Material between them is a *tongue*;
  empty space between them is a *slot*. The mid-plane gives the line the
  joint slides on, the gap gives the width.

How the part was modelled does not matter; any CAD program that writes true
cylinders and planes gives the same features.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepGProp import BRepGProp_Face
from OCP.GeomAbs import GeomAbs_Cylinder, GeomAbs_Plane
from OCP.gp import gp_Pnt, gp_Vec

from .kin import Feature

AX_TOL = 1e-6


@dataclass
class Found:
    kind: str  # pin | hole | tongue | slot
    xy: tuple[float, float]
    size: float
    z: tuple[float, float]
    dir: float | None = None
    along: tuple[float, float] | None = None
    # the real bearing faces along a straight feature, as separate blocks
    # (from xy, like along): a channel interrupted by a gap is two blocks
    segments: tuple | None = None

    def feature(self) -> Feature:
        return Feature(self.kind, self.xy, self.size, self.z, self.dir, self.along, self.segments)

    def text(self) -> str:
        if self.dir is None:
            return f"{self.kind} dia {self.size:.2f} at ({self.xy[0]:.2f}, {self.xy[1]:.2f}), z {self.z[0]:.1f}-{self.z[1]:.1f}"
        return (f"{self.kind} {self.size:.2f} wide at {math.degrees(self.dir) % 180:.1f} deg, "
                f"z {self.z[0]:.1f}-{self.z[1]:.1f}")


def _normal(face, u: float, v: float):
    g = BRepGProp_Face(face.wrapped)
    p, n = gp_Pnt(), gp_Vec()
    g.Normal(u, v, p, n)
    m = n.Magnitude()
    return (p.X(), p.Y(), p.Z()), (n.X() / m, n.Y() / m, n.Z() / m)


def _mid_uv(face):
    ad = BRepAdaptor_Surface(face.wrapped)
    return (ad.FirstUParameter() + ad.LastUParameter()) / 2, (ad.FirstVParameter() + ad.LastVParameter()) / 2


def _samples(face, n: int = 6):
    pts = []
    for e in face.edges():
        for i in range(n + 1):
            pts.append(tuple(e.position_at(i / n)))
    return pts


def round_features(solid) -> list[Found]:
    """Pins and bores with their axis along the normal. The cylinders, their
    side (pin or bore) and their height range come from the joints package's
    finder (``jointbench.joints.common.cylinders``, itself on the core's
    ``checks.holes``), so the project has one way of finding a pin or a bore."""
    from ..joints.common import cylinders

    groups: dict = {}
    for c in cylinders(solid):
        if abs(abs(c.axis[2]) - 1) > AX_TOL:
            continue
        t = -c.loc[2] / c.axis[2]  # the axis where it crosses height 0
        cx, cy = c.loc[0] + t * c.axis[0], c.loc[1] + t * c.axis[1]
        kind = "pin" if c.convex else "hole"
        key = (kind, round(cx, 4), round(cy, 4), round(c.r, 4))
        if key in groups:  # a cylinder split into several faces
            g = groups[key]
            g[0], g[1] = min(g[0], c.z0), max(g[1], c.z1)
        else:
            groups[key] = [c.z0, c.z1, cx, cy, c.r]
    return [Found(k[0], (g[2], g[3]), 2 * g[4], (g[0], g[1])) for k, g in groups.items()]


def straight_features(solid, widths: tuple[float, float] = (0.5, 200.0)) -> list[Found]:
    planes = []
    for f in solid.faces():
        ad = BRepAdaptor_Surface(f.wrapped)
        if ad.GetType() != GeomAbs_Plane:
            continue
        u, v = _mid_uv(f)
        p, n = _normal(f, u, v)
        if abs(n[2]) > AX_TOL:
            continue
        pts = _samples(f, 2)
        t = (-n[1], n[0])
        along = [q[0] * t[0] + q[1] * t[1] for q in pts]
        zs = [q[2] for q in pts]
        planes.append({"n": (n[0], n[1]), "off": p[0] * n[0] + p[1] * n[1], "t": t,
                       "along": (min(along), max(along)), "z": (min(zs), max(zs))})
    out: dict = {}
    for i, a in enumerate(planes):
        for b in planes[i + 1:]:
            if a["n"][0] * b["n"][0] + a["n"][1] * b["n"][1] > -1 + 1e-9:
                continue
            # a's outward normal n; b sits at signed distance s along n
            s = b["off"] * -1 - a["off"]  # b.off is measured along -n
            w = abs(s)
            if not (widths[0] <= w <= widths[1]):
                continue
            kind = "slot" if s > 0 else "tongue"
            tb = b["along"]  # b's t is the reverse of a's
            lo = max(a["along"][0], -tb[1])
            hi = min(a["along"][1], -tb[0])
            zlo, zhi = max(a["z"][0], b["z"][0]), min(a["z"][1], b["z"][1])
            if hi - lo < 1e-6 or zhi - zlo < 1e-6:
                continue
            n, t = a["n"], a["t"]
            mid = a["off"] + s / 2
            ang = math.atan2(t[1], t[0]) % math.pi
            # canonical direction mod pi: flip t if needed, then midline offset
            if abs(ang - math.atan2(t[1], t[0])) > 1e-9:
                t, lo, hi = (-t[0], -t[1]), -hi, -lo
                n = (-n[0], -n[1])
                mid = -mid
            key = (kind, round(ang, 6), round(mid, 4), round(w, 4))
            # faces of one plane pair merge only where their heights meet:
            # two tongues in one plane but at different heights stay two
            for g in out.setdefault(key, []):
                if zlo <= g["z"][1] + 1e-6 and zhi >= g["z"][0] - 1e-6:
                    g["lo"], g["hi"] = min(g["lo"], lo), max(g["hi"], hi)
                    g["z"] = (min(g["z"][0], zlo), max(g["z"][1], zhi))
                    g["segs"].append((lo, hi))
                    break
            else:
                out[key].append({"kind": kind, "ang": ang, "n": n, "t": t, "mid": mid, "w": w,
                                 "lo": lo, "hi": hi, "z": (zlo, zhi), "segs": [(lo, hi)]})
    res = []
    for g in (g for gs in out.values() for g in gs):
        c = (g["lo"] + g["hi"]) / 2
        xy = (g["n"][0] * g["mid"] + g["t"][0] * c, g["n"][1] * g["mid"] + g["t"][1] * c)
        half = (g["hi"] - g["lo"]) / 2
        segs = tuple((a - c, b - c) for a, b in merge_segments(g["segs"]))
        res.append(Found(g["kind"], xy, g["w"], g["z"], g["ang"], (-half, half), segs))
    return res


def merge_segments(segs, gap: float = 1e-3) -> list:
    """Sorted intervals with touching or overlapping ones joined."""
    out = []
    for a, b in sorted(segs):
        if out and a <= out[-1][1] + gap:
            out[-1] = (out[-1][0], max(out[-1][1], b))
        else:
            out.append((a, b))
    return out


def all_features(solid) -> list[Found]:
    return round_features(solid) + straight_features(solid)


# what a removed part must offer at a joint, given what the neighbour has there
MATE = {"pin": "hole", "hole": "pin", "slot": "tongue", "tongue": "slot"}


def fit_range(mate: Feature, joint_kind: str, fits: dict) -> tuple[str, float, float]:
    """(kind needed on the removed part, smallest size, largest size)."""
    lo, hi = fits["clear_min"], fits["clear_max"]
    if joint_kind == "pin_slot":
        need = "slot" if mate.kind == "pin" else "pin"
    else:
        need = MATE[mate.kind]
    if need in ("hole", "slot"):
        return need, mate.size + lo, mate.size + hi
    return need, mate.size - hi, mate.size - lo


def z_overlap(a: tuple, b: tuple) -> float:
    return min(a[1], b[1]) - max(a[0], b[0])


def bearing_blocks(solid, f: Feature) -> list:
    """The real bearing faces of straight feature ``f`` on ``solid`` (the
    part in its own frame), as blocks (lo, hi) along f.dir from f.xy. A guide
    built as two separate blocks gives two; one continuous channel gives one.
    Falls back to f.along when the solid shows no matching face pair."""
    t = (math.cos(f.dir), math.sin(f.dir))
    best = None
    for g in straight_features(solid):
        if g.kind != f.kind or abs(g.size - f.size) > 1e-3 or abs(math.sin(g.dir - f.dir)) > 1e-6:
            continue
        d = (g.xy[0] - f.xy[0], g.xy[1] - f.xy[1])
        if abs(d[0] * -t[1] + d[1] * t[0]) > 1e-3:  # not on f's line
            continue
        sgn = 1.0 if math.cos(g.dir - f.dir) > 0 else -1.0
        c = d[0] * t[0] + d[1] * t[1]
        segs = [tuple(sorted((c + sgn * a, c + sgn * b))) for a, b in (g.segments or (g.along,))]
        if best is None or sum(b - a for a, b in segs) > sum(b - a for a, b in best):
            best = segs
    if best is None:
        return [tuple(f.along)]
    return merge_segments(best)
