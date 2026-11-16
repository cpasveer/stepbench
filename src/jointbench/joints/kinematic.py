"""Kinematic coupling: three balls on one part seated in three V-grooves on the other.

The given part is a base with three V-grooves (a Maxwell coupling). The agent
designs the top part with three balls of the stated diameter, placed so that
each ball touches both flanks of its groove and nothing else touches.

Checks:

* KC_OVERLAP     no overlap with the base (0.01 mm3)
* KC_CONTACTS    the contacts between the two parts are counted: a ball face
                 and a base face touch when the gap is within +-0.005 mm and
                 the touching point lies on both faces; only point contacts
                 (a sphere on a plane, an edge or another surface) are
                 allowed. Exactly six pass; five or fewer fail as loose, seven
                 or more (or any surface contact) as over-constrained.
* KC_CONSTRAINT  the six contact normals and their moments (scaled by the
                 contacts' own size) form a 6 x 6 matrix of full rank whose
                 condition number is at most the stated limit: together they
                 hold the part in every direction, none nearly parallel to
                 another.
* KC_PRELOAD     under the stated clamping force the six contact forces that
                 balance it are all compressive (each at least 1% of the
                 mean): the part stays seated rather than tipping.
"""
from __future__ import annotations

import math

import build123d as bd
import numpy as np
from OCP.BRepClass import BRepClass_FaceClassifier
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.gp import gp_Pnt

from . import common as C

NAME = "kinematic"
TITLE = "Kinematic coupling (three balls in three V-grooves)"
CONTACT_TOL = 0.005  # mm: a gap or a penetration up to this counts as touching
COND_MAX = 10.0
BALLS = [6.0, 8.0, 10.0, 12.0, 16.0]  # ball diameters
GROOVES = [60.0, 90.0, 120.0]  # included angle


def generate(seed: int, rules_told: bool = True, aligned: bool = False, groove_skew: float = 0.0) -> C.Instance:
    rng = C.rng_for(NAME, seed)
    db = rng.choice(BALLS)
    ang = rng.choice(GROOVES)
    R = round(rng.uniform(3.0, 5.0) * db + 10, 1)  # ball circle radius
    rot = round(rng.uniform(0, 120), 3)
    rb = db / 2
    beta = math.radians(ang / 2)
    lo = rb / math.sin(beta) - rb * math.sin(beta) + 0.3  # contact at least 0.3 below the top face
    hi = rb / math.sin(beta) - 1.0  # top part's underside (ball centre height) 1 mm above the base
    hg = round((lo + hi) / 2, 3)
    p = dict(ball_d=db, groove_angle=ang, R=R, rot=rot, groove_depth=hg,
             groove_w=2 * hg * math.tan(beta), groove_len=round(2.5 * db + 6, 1),
             base_half=R + 2 * db + 10, base_t=hg + 8.0, skew=groove_skew,
             zc=-hg + rb / math.sin(beta), preload=[0.0, 0.0, -100.0], preload_at=None)
    p["preload_at"] = [0.0, 0.0, p["zc"] + 10.0]
    return C.Instance(NAME, seed, C.random_frame(rng, aligned), p, rules_told, {})


def groove_dirs(p):
    """(centre point on the top face, groove direction, lateral direction) per groove."""
    out = []
    for i in range(3):
        a = math.radians(p["rot"] + 90 + 120 * i)
        c = (p["R"] * math.cos(a), p["R"] * math.sin(a), 0.0)
        g = a + math.radians(p["skew"])
        u = (math.cos(g), math.sin(g), 0.0)
        v = (-u[1], u[0], 0.0)
        out.append((c, u, v))
    return out


def fixed_local(inst) -> dict:
    p = inst.p
    h, t = p["base_half"], p["base_t"]
    base = bd.Pos(0, 0, -t / 2) * bd.Box(2 * h, 2 * h, t)
    for c, u, _ in groove_dirs(p):
        w, d = p["groove_w"], p["groove_depth"]
        tri = bd.Polygon((-w / 2 - 0.5 * w, 0.5 * d), (w / 2 + 0.5 * w, 0.5 * d), (w / 2, 0.0), (0.0, -d),
                         (-w / 2, 0.0), align=None)
        # the V (with a wider mouth above the top face, so the top face edges stay sharp)
        prism = bd.extrude(bd.Plane((0, 0, 0), x_dir=(0, 1, 0), z_dir=(1, 0, 0)) * tri, p["groove_len"] / 2, both=True)
        ang = math.degrees(math.atan2(u[1], u[0]))
        prism = bd.Pos(*c) * bd.Rot(0, 0, ang) * prism
        base = base - prism
    return {"base": base}


def fixed_parts(inst) -> dict:
    return {k: C.to_world(v, inst.frame) for k, v in fixed_local(inst).items()}


def ball_centres(p):
    return [(c[0], c[1], p["zc"]) for c, _, _ in groove_dirs(p)]


def reference(inst, slide=None, lift=None, extra_ball=False, drop_ball=False):
    """Top plate with three balls. ``slide``: (ball, mm) moves one ball up one
    flank (it keeps touching that flank and leaves the other); ``lift``:
    (ball, gap) lifts one ball so it stands ``gap`` off both flanks (negative:
    sinks in); ``extra_ball``: a fourth ball resting on the base's top face."""
    p = inst.p
    rb = p["ball_d"] / 2
    beta = math.radians(p["groove_angle"] / 2)
    zc = p["zc"]
    plate_t = 6.0
    plate = bd.Pos(0, 0, zc + plate_t / 2) * bd.Cylinder(p["R"] + p["ball_d"] + 4, plate_t)
    part = plate
    for i, (c, u, v) in enumerate(groove_dirs(p)):
        if drop_ball and i == 2:
            continue
        cc = [c[0], c[1], zc]
        if slide and slide[0] == i:
            s = slide[1] / math.sin(2 * beta)
            # up flank A: lateral +v by s*sin(beta), up by s*cos(beta)
            cc = [cc[0] + v[0] * s * math.sin(beta), cc[1] + v[1] * s * math.sin(beta), cc[2] + s * math.cos(beta)]
        if lift and lift[0] == i:
            cc[2] += lift[1] / math.sin(beta)
        part = part + bd.Pos(*cc) * bd.Sphere(rb)
    if extra_ball:
        r4 = zc / 2 + 1.0  # rests on the top face z = 0, reaching up into the plate
        part = part + bd.Pos(0, 0, r4) * bd.Sphere(r4)
    return part


def brief(inst) -> str:
    p = inst.p
    L = [C.frame_text(inst.frame), ""]
    gs = groove_dirs(p)
    L.append(f"The given part is a base whose top face is z = 0, with three V-grooves of {p['groove_angle']:g} deg "
             f"included angle, {p['groove_depth']:.3f} mm deep, {p['groove_len']:g} mm long, bottom line at "
             f"z = {-p['groove_depth']:.3f}. Groove centres on the top face and directions:")
    for i, (c, u, _) in enumerate(gs):
        L.append(f"  groove {i + 1}: centre ({c[0]:.4f}, {c[1]:.4f}, 0), along ({u[0]:.6f}, {u[1]:.6f}, 0)")
    L.append(f"Design one top part that sits on the base through balls of diameter {p['ball_d']:g} mm (spherical "
             "faces, modelled as part of the solid), seated in the grooves.")
    if inst.rules_told:
        L.append("- Each ball touches both flanks of its groove, so the part touches the base at exactly six points "
                 f"(gap or penetration at most {CONTACT_TOL} mm counts as touching), and nowhere else.")
    else:
        L.append("- It must be exactly constrained: touching the base at the minimum number of points that holds it "
                 f"(gap or penetration at most {CONTACT_TOL} mm counts as touching), and nowhere else.")
    L.append(f"- The contacts hold it in every direction: their constraint matrix has full rank and condition number "
             f"at most {COND_MAX:g} (normals, and moments divided by the contacts' RMS distance from their centroid).")
    f, a = p["preload"], p["preload_at"]
    L.append(f"- Under a clamping force ({f[0]:g}, {f[1]:g}, {f[2]:g}) N through ({a[0]:g}, {a[1]:g}, {a[2]:.3f}), "
             "every contact pushes (at least 1% of the mean force); no overlap with the base.")
    return "\n".join(L)


# ----------------------------------------------------------------- check


def _in_face(face, pt, tol=1e-4):
    cl = BRepClass_FaceClassifier(face.wrapped, gp_Pnt(*pt), tol)
    from OCP.TopAbs import TopAbs_IN, TopAbs_ON
    return cl.State() in (TopAbs_IN, TopAbs_ON)


def contacts(part, base, tol=CONTACT_TOL):
    """Every touching point between the two solids: (point, normal into the
    part, kind, gap). Spherical faces of the part are measured exactly against
    planes; any other pair by the kernel's exact distance."""
    out, probs = [], []
    # faces with their exact surface, read once (a face's Python wrapper is
    # new on every call, so faces are paired here, not looked up)
    pfaces = [(f, (c, r)) for c, r, f in C.spheres(part)]
    sph_set = [f for f, _ in pfaces]
    pfaces += [(f, None) for f in part.faces() if not any(f.is_same(g) for g in sph_set)]
    bfaces = [(f, (pt, n), f.bounding_box()) for pt, n, f in C.planes(base)]
    pl_set = [f for f, _, _ in bfaces]
    bfaces += [(f, None, f.bounding_box()) for f in base.faces() if not any(f.is_same(g) for g in pl_set)]
    for pf, s in pfaces:
        pb = pf.bounding_box()
        for bf, pl, bb in bfaces:
            if (pb.min.X > bb.max.X + tol or pb.max.X < bb.min.X - tol or pb.min.Y > bb.max.Y + tol
                    or pb.max.Y < bb.min.Y - tol or pb.min.Z > bb.max.Z + tol or pb.max.Z < bb.min.Z - tol):
                continue
            if s is not None and pl is not None:
                c, r = s
                q, n = pl  # n: outward normal of the base face, pointing to the part
                dist = sum((c[k] - q[k]) * n[k] for k in range(3))
                gap = dist - r
                if gap > tol:
                    continue  # the ball stays clear of the whole plane
                foot = tuple(c[k] - n[k] * dist for k in range(3))
                if _in_face(bf, foot) and _in_face(pf, tuple(c[k] - n[k] * r for k in range(3))):
                    if gap >= -tol:
                        out.append((foot, n, "point", gap))
                    else:
                        probs.append((foot, gap))
                    continue
                # the nearest point is off the face: an edge contact, if any, below
            # general pair: the kernel's minimum distance and its points
            ext = BRepExtrema_DistShapeShape(pf.wrapped, bf.wrapped)
            if not ext.IsDone() or ext.Value() > tol:
                continue
            for k in range(1, ext.NbSolution() + 1):
                a, b = ext.PointOnShape1(k), ext.PointOnShape2(k)
                pa = (a.X(), a.Y(), a.Z())
                if s is not None:
                    c = s[0]
                    nv = np.subtract(c, pa)
                    nv = tuple(nv / np.linalg.norm(nv))
                    out.append((pa, nv, "point", ext.Value()))
                else:
                    out.append((pa, None, "surface", ext.Value()))
    # merge duplicates: the same touch reached through two faces sharing an edge
    merged = []
    for c in out:
        if any(math.dist(c[0], m[0]) < 0.01 for m in merged):
            continue
        merged.append(c)
    return merged, probs


def constraint(cs):
    """(rank, condition number, matrix, centroid, length) of point contacts."""
    P = np.array([c[0] for c in cs], float)
    N = np.array([c[1] for c in cs], float)
    p0 = P.mean(axis=0)
    Lc = math.sqrt(((P - p0) ** 2).sum(axis=1).mean()) or 1.0
    A = np.hstack([N, np.cross(P - p0, N) / Lc])
    s = np.linalg.svd(A, compute_uv=False)
    rank = int((s > 1e-9 * s[0]).sum())
    cond = float(s[0] / s[-1]) if rank == 6 and len(s) == 6 else math.inf
    return rank, cond, A, p0, Lc


def check(solid, inst, cond_max: float = COND_MAX, preload_at=None) -> list:
    p = inst.p
    part = C.to_local(solid, inst.frame)
    base = fixed_local(inst)["base"]
    out = []
    v = C.overlap(part, base)
    out.append(C.res("KC_OVERLAP", v <= C.OVERLAP_MAX, f"overlap with the base {v:.4f} mm3 (limit {C.OVERLAP_MAX})",
                     v, "mm3"))
    cs, sunk = contacts(part, base)
    pts = [c for c in cs if c[2] == "point"]
    surf = [c for c in cs if c[2] == "surface"]
    n = len(cs)
    where = [tuple(round(x, 3) for x in c[0]) for c in cs]
    if sunk:
        ok, why = False, (f"a ball sinks {-min(g for _, g in sunk):.4f} mm into a base face "
                          f"(more than {CONTACT_TOL}) at {tuple(round(x, 3) for x in sunk[0][0])}")
    elif surf:
        ok, why = False, f"over-constrained: {len(surf)} non-point contact(s) (a flat or curved face touching)"
    elif n < 6:  # level 3 tests knowing the number: the reason does not state it
        ok, why = False, (f"loose: {n} contacts, a rigid part needs 6 to be held" if inst.rules_told
                          else f"loose: {n} contacts, too few to hold it")
    elif n > 6:
        ok, why = False, (f"over-constrained: {n} contacts, 6 hold a rigid part exactly" if inst.rules_told
                          else f"over-constrained: {n} contacts, more than hold it exactly")
    else:
        ok, why = True, "6 point contacts"
    gaps = [c[3] for c in cs]
    if cs:
        why += f" (gaps {min(gaps):+.4f} to {max(gaps):+.4f} mm, tol {CONTACT_TOL})"
    out.append(C.res("KC_CONTACTS", ok, why, n, "contacts", points=where))
    if len(pts) >= 1:
        rank, cond, A, p0, Lc = constraint(pts)
        if rank < 6:
            out.append(C.res("KC_CONSTRAINT", False, f"constraint matrix rank {rank}: {6 - rank} way(s) to move remain",
                             rank, "rank"))
        else:
            ok = cond <= cond_max
            out.append(C.res("KC_CONSTRAINT", ok, f"full rank, condition number {cond:.3f} (max {cond_max:g})",
                             cond, "", rank=rank))
    else:
        out.append(C.res("KC_CONSTRAINT", False, "no point contacts", 0, "rank"))
    if len(pts) == 6 and not surf and out[-1].ok:
        rank, cond, A, p0, Lc = constraint(pts)
        F = np.array(p["preload"], float)
        at = np.array(preload_at if preload_at is not None else p["preload_at"], float)
        w = np.concatenate([F, np.cross(at - p0, F) / Lc])
        lam = np.linalg.solve(A.T, -w)
        mean = float(np.abs(lam).mean())
        worst = float(lam.min())
        ok = worst >= 0.01 * mean
        out.append(C.res("KC_PRELOAD", ok,
                         f"contact forces {lam.min():.2f} to {lam.max():.2f} N under the clamping force"
                         + ("" if ok else ": a contact would have to pull, the part tips off it"),
                         worst, "N", forces=[round(float(x), 4) for x in lam]))
    else:
        out.append(C.res("KC_PRELOAD", None, "skipped: needs six point contacts of full rank"))
    return out


SELFTEST = [
    ("reference", {}, None),
    ("one ball lifted 0.0025 mm (still touching)", {"part": {"lift": (1, 0.0025)}}, None),
    ("one ball sunk 0.0025 mm (still touching)", {"part": {"lift": (1, -0.0025)}}, None),
    ("one ball lifted 0.0075 mm (loose)", {"part": {"lift": (1, 0.0075)}}, "KC_CONTACTS"),
    ("one ball sunk 0.0075 mm", {"part": {"lift": (1, -0.0075)}}, "KC_CONTACTS"),
    ("one ball slid up a flank: 5 contacts", {"part": {"slide": (0, 0.05)}}, "KC_CONTACTS"),
    ("a fourth ball on the top face: 7 contacts", {"part": {"extra_ball": True}}, "KC_CONTACTS"),
    ("two balls only: 4 contacts", {"part": {"drop_ball": True}}, "KC_CONSTRAINT"),
    ("condition limit just above the measured", {"check": {"cond_max": "measured*1.01"}}, None),
    ("condition limit just below the measured", {"check": {"cond_max": "measured*0.99"}}, "KC_CONSTRAINT"),
    ("grooves all parallel: rank 5", {"knobs": {"groove_skew": "parallel"}}, "KC_CONSTRAINT"),
    ("clamping force outside the balls: tips", {"check": {"preload_at": "outside"}}, "KC_PRELOAD"),
]


_check = check


def check(solid, inst, cond_max=COND_MAX, preload_at=None):  # noqa: F811 - symbolic self-test knobs
    if isinstance(cond_max, str):
        rep = _check(solid, inst)
        measured = next(r.value for r in rep if r.name == "KC_CONSTRAINT")
        cond_max = measured * float(cond_max.split("*")[1])
    if preload_at == "outside":
        p = inst.p
        c = groove_dirs(p)[0][0]
        preload_at = [3 * c[0], 3 * c[1], p["zc"] + 10.0]
    return _check(solid, inst, cond_max=cond_max, preload_at=preload_at)


_generate = generate


def generate(seed, groove_skew=0.0, **kw):  # noqa: F811
    if groove_skew == "parallel":
        inst = _generate(seed, **kw)
        # every groove along the same direction: the part slides along it
        inst.p["skew"] = "parallel"
        return inst
    return _generate(seed, groove_skew=groove_skew, **kw)


_groove_dirs = groove_dirs


def groove_dirs(p):  # noqa: F811
    if p.get("skew") == "parallel":
        out = []
        for c, _, _ in _groove_dirs(dict(p, skew=0.0)):
            u = (1.0, 0.0, 0.0)
            out.append((c, u, (0.0, 1.0, 0.0)))
        return out
    return _groove_dirs(p)


TASK = """\
**Given:** a base with three V-grooves at stated positions and directions. **Design:** the top part
with three balls. **What makes it hard:** the ball centres must sit at the exact height where a
ball of the stated diameter touches both flanks of a groove of the stated angle (a few lines of
trigonometry, to 5 micrometres), and nothing else may touch. With the rules not told the brief
says only "exactly constrained": the agent must know that a rigid body needs six contacts and that
a ball in a V-groove gives two."""

KNOBS = {
    "rules_told": "false: 'exactly constrained' instead of 'six contacts, two per groove'",
    "groove angle": "60, 90 or 120 deg (drawn from the seed)",
    "aligned": "true: base top face = world xy (easier)",
    "later": "a 'Kelvin' variant (cone, V and flat) and a 'rules not told' grooves-only brief",
}
