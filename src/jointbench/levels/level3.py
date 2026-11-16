"""Level 3: what is in the way.

Test agents passed level 2 easily and all named the same difficulty: keeping
the connecting body clear of bolt heads, the motor's boss and shaft. So level
3 turns that up:

* two or three fixed parts to join with one solid;
* obstacles -- fixed keep-out boxes placed exactly where a straight
  connection would run;
* tool access -- a straight hex-key / driver envelope above every bolt head
  must stay free.

Generation is answer-first. The reference is a plate on each face, a tab
facing a common hub point, and one *routed* arm per plate: from the tab it
steps sideways by ``detour``, runs parallel to the straight line, and steps
back in at the hub. Obstacles are then placed on the straight line between
each tab and the hub ("blockers"), and optionally on the far side of the
detour ("fences"), each leaving exactly ``margin`` of free space to the
reference arm. A draw is kept only if

* the reference and an alternative body both pass every check,
* no obstacle touches a fixed part or the space any bolt and its tool could
  ever need (so a blocked tool is always the answer's own doing), and
* with obstacles present, both straight baselines -- a convex hull of the
  tabs, and straight rods from tabs to hub -- collide with an obstacle or a
  fixed part.

Difficulty knobs (recorded in every task's ``meta.knobs``):

    parts       2 or 3 fixed parts
    obstacles   0..2*parts: blockers first (one per arm), then fences
    margin      free space between the reference arm and each obstacle (mm)
    detour      how far the reference arm steps aside (mm), measured for an
                12 mm arm; a wider arm (where the section rule needs it) steps
                aside further by half its extra width, so the free space is
                the same
"""
from __future__ import annotations

import math
import os
import random
import tempfile

import build123d as bd

from ..catalogue import GENERATOR_VERSION, BOLTS, make_interface, sample_interface
from ..geometry import Placement, matmul, rot_axis_angle, rot_from_quaternion, write_step
from ..task import DEFAULT_TOLERANCES, Obstacle, Task
from . import construct as C
from . import rods as R

LEVEL = 3
ARM_W = 12.0  # narrowest arm; wider where the section requirement asks for it
RISE = 8.0    # rods leave a plate square to it for this far before turning
HUB_OUT = 25.0
OBSTACLE_DEPTH = 24.0  # across the arm's path; wide enough to stop the straight hull
# Default two parts: with makeable rod references, three-part draws are
# rarely admissible yet (three bundles of tool paths crowd the hub); parts=3
# stays available as a knob. See README, "Known limits".
KNOBS_DEFAULT = {"parts": 2, "obstacles": 2, "margin": 4.0, "detour": 25.0, "process": "pbf", "stiffness": 0.0}
GEOMETRY_KNOBS = ("parts", "obstacles", "margin", "detour")
TOLERANCES = dict(DEFAULT_TOLERANCES, tool_length=30.0, tool_diameter_extra=0.0,
                  section_factor=2.0, required_checks=["SECTION_MIN"])

SITUATION = (
    "Fixed parts sit in space at angles to each other, in an arbitrary overall\n"
    "position and orientation. Obstacles -- fixed boxes your part must not\n"
    "touch -- stand between them. Every bolt must stay reachable by a straight\n"
    "tool along its axis."
)

FLIP = ((1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, -1.0))
NAMES = ("A", "B", "C")


def knobs_with_defaults(knobs: dict | None) -> dict:
    k = dict(KNOBS_DEFAULT)
    k.update({a: b for a, b in (knobs or {}).items() if b is not None})
    k["parts"] = int(k["parts"])
    k["obstacles"] = int(k["obstacles"])
    k["margin"] = float(k["margin"])
    k["detour"] = float(k["detour"])
    k["process"] = str(k["process"])
    k["stiffness"] = float(k["stiffness"])
    if k["process"] not in ("fdm", "pbf"):
        raise ValueError("knob process must be pbf (powder bed fusion) or fdm (filament)")
    if k["stiffness"] not in (0.0, 1.0):
        raise ValueError("knob stiffness is 1 (required, from the task's bolts and span) or 0 (off)")
    if k["parts"] not in (2, 3):
        raise ValueError("knob parts must be 2 or 3")
    if not 0 <= k["obstacles"] <= 2 * k["parts"]:
        raise ValueError(f"knob obstacles must be 0..{2 * k['parts']}")
    return k


def task_id(seed: int, knobs: dict) -> str:
    if knobs == KNOBS_DEFAULT:
        return f"L3-s{seed}"
    tid = f"L3-s{seed}"
    if any(knobs[g] != KNOBS_DEFAULT[g] for g in GEOMETRY_KNOBS):
        tid += f"-p{knobs['parts']}o{knobs['obstacles']}m{knobs['margin']:g}d{knobs['detour']:g}"
    if knobs["process"] != "pbf":
        tid += f"-{knobs['process']}"
    if knobs["stiffness"]:
        tid += "-stiff"
    return tid


def _draw(rng: random.Random, knobs: dict) -> dict:
    names = NAMES[: knobs["parts"]]
    d = {"parts": list(names), "knobs": dict(knobs)}
    t_choices = [4.0, 5.0, 6.0, 8.0]
    for n in names:
        itf = sample_interface(rng)
        b = BOLTS[itf.bolt]
        d[n] = {"kind": itf.kind, "params": itf.params, "yaw": 5.0 * rng.randrange(72),
                "t": rng.choice([t for t in t_choices if t >= b.grip_min])}
    first_dir = 5.0 * rng.randrange(72)
    for i, n in enumerate(names[1:]):
        d[n].update({
            "height": float(rng.randrange(80, 161)) / 2,  # 40 .. 80 mm
            "lateral": float(rng.randrange(80, 161)) / 2,  # 40 .. 80 mm
            "lateral_dir": first_dir + (0.0 if i == 0 else 5.0 * rng.randrange(24, 49)),  # 2nd: 120..240 deg on
            "tilt": 5.0 * rng.randrange(4, 15),
            "tilt_jitter": 5.0 * rng.randrange(-6, 7),  # tilt axis +-30 deg off square
        })
    d["scene_quat"] = [rng.gauss(0.0, 1.0) for _ in range(4)]
    d["scene_origin"] = [float(rng.randrange(-300, 301)) / 2 for _ in range(3)]
    d["detour_pick"] = [rng.random() for _ in names]
    d["hub_pick"] = rng.random()
    return d


def task_from_draw(seed: int, d: dict) -> Task:
    names = d["parts"]
    parts = {n: make_interface(d[n]["kind"], d[n]["params"]) for n in names}
    g = Placement(tuple(d["scene_origin"]), rot_from_quaternion(*d["scene_quat"]))
    pose = {"A": Placement((0.0, 0.0, 0.0), rot_axis_angle((0, 0, 1), d["A"]["yaw"])).then(g)}
    for n in names[1:]:
        e = d[n]
        # tilt about an axis roughly square to the sideways offset, in the sense
        # that turns the face towards the middle of the arrangement
        psi = math.radians(e["lateral_dir"])
        phi = math.radians(e["lateral_dir"] + 90.0 + e["tilt_jitter"])
        axis = (math.cos(phi), math.sin(phi), 0.0)
        tilt = rot_axis_angle(axis, e["tilt"])
        nb = tuple(sum(tilt[i][k] * (0.0, 0.0, -1.0)[k] for k in range(3)) for i in range(3))
        if nb[0] * math.cos(psi) + nb[1] * math.sin(psi) > 0:
            tilt = rot_axis_angle(axis, -e["tilt"])
        local = Placement((e["lateral"] * math.cos(psi), e["lateral"] * math.sin(psi), e["height"]),
                          matmul(tilt, matmul(FLIP, rot_axis_angle((0, 0, 1), e["yaw"]))))
        pose[n] = local.then(g)
    return Task(task_id(seed, d["knobs"]), LEVEL, seed, parts, [pose], dict(TOLERANCES),
                {"generator": "level3", "knobs": d["knobs"]}, generator_version=GENERATOR_VERSION)


# --------------------------------------------------------------------------
# planning the routed reference


def _boxes_apart(a, b, gap: float = 0.0) -> bool:
    """True when the axis-aligned bounding boxes are more than ``gap`` apart:
    then the solids certainly do not touch, and no exact test is needed."""
    p, q = a.bounding_box(), b.bounding_box()
    return (p.min.X - q.max.X > gap or q.min.X - p.max.X > gap or p.min.Y - q.max.Y > gap
            or q.min.Y - p.max.Y > gap or p.min.Z - q.max.Z > gap or q.min.Z - p.max.Z > gap)


def _clear(solid, others) -> bool:
    from ..checks.base import volume  # an empty intersection may come back as None

    def overlap_free(o):
        try:
            return volume(solid & o) <= 1e-6
        except Exception:  # noqa: BLE001 - the kernel could not intersect: count it as a clash
            return False

    return all((getattr(o, "corridor", None) is not None and C.far_from_corridor(solid, o.corridor))
               or _boxes_apart(solid, o) or overlap_free(o) for o in others)


def _distance_at_least(a, b, gap: float) -> bool:
    try:
        return _boxes_apart(a, b, gap) or a.distance_to(b) >= gap
    except Exception:  # noqa: BLE001 - an empty or broken shape: not a safe distance
        return False


def _frame_along(v: bd.Vector, hint: bd.Vector):
    e1 = v.normalized()
    e2 = (hint - e1 * hint.dot(e1))
    e2 = e2.normalized() if e2.length > 1e-9 else e1.cross(bd.Vector(0, 0, 1) if abs(e1.Z) < 0.9 else bd.Vector(1, 0, 0)).normalized()
    return (e1, e2, e1.cross(e2).normalized())


def _arm(plate: dict, a: dict, H: bd.Vector, w: float, aligned: bool = False) -> bd.Part:
    """The routed arm: tab -> W1 -> W2 -> hub, hulls of cubes of side w.
    With ``aligned``, every segment's cubes are turned along that segment,
    so the arm's section is w x w everywhere (a hull between offset squares
    is otherwise thinner across its length by the cosine of its slant);
    used where a member is built to a stated section (par, self-test)."""
    W1, W2 = bd.Vector(*a["W1"]), bd.Vector(*a["W2"])
    s, q = bd.Vector(*a["s"]), bd.Vector(*a["p"])
    ax = (s, q, s.cross(q).normalized())  # the arm's own frame
    if not aligned:
        return (C.hull(C.tab_quad(plate, w) + C.cube_points(W1, w, ax))
                + C.hull(C.cube_points(W1, w, ax) + C.cube_points(W2, w, ax))
                + C.hull(C.cube_points(W2, w, ax) + C.cube_points(H, 1.4 * w, ax)))
    T = C.centre(C.tab_quad(plate, w))
    a1, a3 = _frame_along(W1 - T, q), _frame_along(H - W2, q)
    root = C.hull(C.tab_quad(plate, w) + C.cube_points(W1, w, a1))
    lead = C.hull(C.cube_points(T + (W1 - T) * 0.35, w, a1) + C.cube_points(W1, w, a1))
    return (root + lead + C.hull(C.cube_points(W1, w, ax) + C.cube_points(W2, w, ax))
            + C.hull(C.cube_points(W2, w, a3) + C.cube_points(H, w, a3)) + bd.Pos(*H) * bd.Box(*(3 * [1.4 * w])))


def _plan(task: Task, d: dict, keep_out: list) -> str | None:
    """Choose the hub and each arm's detour; store them in the draw. Returns
    None on success, else the reason the draw cannot be used.

    ``keep_out`` holds the fixed parts and every bolt's tool corridor. The
    hub distance and each arm's sideways direction are searched (in an order
    fixed by the draw) for the first choice that keeps clear of them."""
    D = d["knobs"]["detour"] + (d["arm_w"] - ARM_W) / 2
    why = "a face looks away from the hub"
    # Every bolt's tool path points into the middle of the arrangement, so the
    # hub is searched beside the middle: offsets of 0..45 mm in 14 directions
    # (6 axis, 8 diagonal), starting from a direction fixed by the draw.
    mid = C.hub_point(task, out=HUB_OUT)
    # 26 directions (faces, edges and corners of a cube) at 15-75 mm: cheap
    # now that clearances are measured as distances, not by solids
    dirs = [bd.Vector(x, y, z).normalized() for x in (-1, 0, 1) for y in (-1, 0, 1) for z in (-1, 0, 1)
            if (x, y, z) != (0, 0, 0)]
    k0 = int(d["hub_pick"] * len(dirs))
    dirs = dirs[k0:] + dirs[:k0]
    cands = [mid] + [mid + u * r for r in (15.0, 30.0, 45.0, 60.0, 75.0) for u in dirs]
    for H in cands:
        if any((H - C._v(f.origin)).dot(C._v(f.normal)) < 0.2 * (H - C._v(f.origin)).length
               for f in task.faces(0)):
            continue
        n_r, r_r = _rod_sizing(task, d, alt=True)
        hub_size = max(2 * r_r, n_r * (2 * r_r + R.GAP)) + 2.0  # the rod layers' spheres at the hub
        if not Capsule([(H, H)], hub_size * 0.87).clear_of(keep_out):  # a ball round the hub's rod spheres
            why = "no hub clear of tool paths and fixed parts"
            continue
        arms, failed = {}, None
        for i, n in enumerate(d["parts"]):
            p = C.plate(task, n, H, max(R.PLATE_T, d[n]["t"]), tab=d["tab"])
            T = C.centre(C.tab_quad(p, d["arm_w"]))
            L = (H - T).length
            if L < 25:
                failed = "a tab is too close to the hub"
                break
            s = (H - T).normalized()
            x, nrm = p["frame"].x_dir, p["frame"].z_dir
            # sideways directions that neither double back over the plate's
            # bolts nor dive towards the fixed part's face
            ok = [q for q in C.perpendiculars(s) if q.dot(x) > -0.2 and q.dot(nrm) > -0.2]
            k0 = int(d["detour_pick"][i] * max(len(ok), 1))
            chosen = None
            for q in ok[k0:] + ok[:k0]:
                a = {"T": list(T), "s": list(s), "p": list(q), "L": L,
                     "W1": list(T + s * (0.2 * L) + q * D), "W2": list(T + s * (0.8 * L) + q * D)}
                if _rod_capsule(p, a, H, task, d).clear_of(keep_out):
                    chosen = a
                    break
            if chosen is None:
                failed = "no clear route for an arm"
                break
            arms[n] = chosen
        if failed:
            why = failed
            continue
        d["hub"], d["arms"] = list(H), arms
        return None
    return why


def _obstacles(task: Task, d: dict, keep_out: list, ref_arms: list) -> list[Obstacle] | None:
    """Place the obstacles the knobs ask for, or None if one cannot be placed.

    Blockers sit across the straight line from a tab to the hub; fences on
    the far side of the detour. Each is tried at a few positions along its
    arm (in a fixed order) and kept at the first that touches no fixed part
    and no tool corridor and leaves at least half the margin to the
    reference (measured, and recorded in the draw)."""
    k = d["knobs"]
    names = d["parts"]
    D, m, w = k["detour"] + (d["arm_w"] - ARM_W) / 2, k["margin"], d["arm_w"]
    out, gaps = [], []
    for j in range(k["obstacles"]):
        kind = "blocker" if j < len(names) else "fence"
        # its own arm first, then the other arms' lines (every task must
        # still block both straight answers, which generate() checks)
        order = names[j % len(names):] + names[: j % len(names)]
        placed = None
        for name in order:
            arm = d["arms"][name]
            T, s, q, L = bd.Vector(*arm["T"]), bd.Vector(*arm["s"]), bd.Vector(*arm["p"]), arm["L"]
            r = s.cross(q).normalized()
            ls = min(12.0, 0.25 * L)
            lo, hi = (-8.0, D - w / 2 - m) if kind == "blocker" else (D + w / 2 + m, D + w / 2 + m + 10.0)
            rot = tuple(tuple(tuple(v)[i] for v in (s, q, r)) for i in range(3))  # columns: s, q, r
            tries = [(f, depth, sh) for f in (0.4, 0.5, 0.33, 0.6, 0.7)
                     for depth in (OBSTACLE_DEPTH, 2 * OBSTACLE_DEPTH / 3) for sh in (0.0, 6.0, -6.0)]
            for f, depth, sh in tries:
                c = T + s * (f * L) + q * ((lo + hi) / 2) + r * sh
                ob = Obstacle(f"O{j + 1}", (round(ls, 6), round(hi - lo, 6), depth), Placement(tuple(c), rot))
                # cheap tests first: the box against every tool corridor and
                # every arm, as distances to line segments (0.25 mm slack for
                # the sampling); solids only for what is left
                if any(_seg_obb(cp, cp + ca * cL, ob) < cr + 0.25 for cp, ca, cr, cL in
                       (o.corridor for o in keep_out if getattr(o, "corridor", None) is not None)):
                    continue
                arm_gap = min((min(_seg_obb(P, Q, ob) for P, Q in a.segs) - a.r - 0.25)
                              for a in ref_arms if isinstance(a, Capsule))
                if arm_gap < 0.5 * m:
                    continue
                sol = ob.solid()
                others = [x for x in keep_out if getattr(x, "corridor", None) is None] + [o.solid() for o in out]
                if not _clear(sol, others) or not all(
                        _distance_at_least(sol, fx, 1.0) for fx in keep_out[: len(names)]):
                    continue
                if not all(_distance_at_least(sol, a, 0.5 * m) for a in ref_arms if not isinstance(a, Capsule)):
                    continue
                placed = (ob, arm_gap)
                break
            if placed:
                break
        if placed is None:
            return None
        out.append(placed[0])
        gaps.append(round(placed[1], 3))
    d["obstacle_gaps"] = gaps
    return out


# --------------------------------------------------------------------------
# the reference connector and its broken copies


def build_connector(task: Task, d: dict, *, style: str = "reference", hole_shift: float = 0.0,
                    hole_d: float | None = None, push: float = 0.0, lift: float = 0.0,
                    bump: bool = False, tool_block: bool = False, member_area: float | None = None,
                    thin_plates: bool = False, fin: bool = False) -> bd.Part:
    """Routed reference. Same switches as level 1, plus:
    bump        a spur from the first arm into obstacle O1 (must fail CLEAR_PATH)
    tool_block  a bar over bolt A1's head, beyond its head space (must fail TOOL_ACCESS)
    member_area arms of this square cross-section instead of the reference width
    thin_plates plates only as thick as the bolts allow
    fin         a 0.4 mm fin on plate A: fits, but no printer can make it
    """
    alt = style == "alt"
    H = bd.Vector(*d["hub"])
    n_rods, r = _rod_sizing(task, d, member_area, alt)

    def thick(n):
        if thin_plates:  # as thin as the bolts allow, but never under a third of the rod
            return max(BOLTS[task.parts[n].bolt].grip_min, 2 * r / 3 + 0.1)
        return max(R.PLATE_T, d[n]["t"]) + (1.0 if alt else 0.0)

    plates = {n: C.plate(task, n, H, thick(n), z0=lift if n == "A" else 0.0, tab=d["tab"]) for n in d["parts"]}
    body = None
    for n in d["parts"]:
        piece = plates[n]["solid"]
        for line in _rod_lines(plates[n], d["arms"][n], d, r, n_rods):
            piece = piece + R.path(line, r)
        body = piece if body is None else body + piece
    if bump and task.obstacles_at(0):
        ob = task.obstacles_at(0)[0]
        a = d["arms"][d["parts"][0]]
        T, s, q = bd.Vector(*a["T"]), bd.Vector(*a["s"]), bd.Vector(*a["p"])
        start = T + s * (a["L"] / 2) + q * (d["knobs"]["detour"] + (d["arm_w"] - ARM_W) / 2)
        end = bd.Vector(*ob.placement.origin)
        v = end - start
        body = body + bd.Plane(origin=tuple(start), z_dir=tuple(v.normalized())) * bd.Pos(0, 0, v.length / 2) \
            * bd.Cylinder(2.5, v.length)
    if fin:
        p = plates["A"]
        body = body + p["frame"] * bd.Pos(p["x0"] - 3.0, (p["y0"] + p["y1"]) / 2, p["z0"] + p["t"] / 2) \
            * bd.Box(8.0, 8.0, 0.4)
    if tool_block:
        p = plates["A"]
        bt = BOLTS[task.parts["A"].bolt]
        b0 = p["bolts_local"][0]
        top = p["z0"] + p["t"]
        zb = top + bt.head_h + task.tolerances["head_clearance_axial"] + 5.0
        xa, xb = b0.X - 3.0, p["x1"] + p["tab"] / 2
        bar = p["frame"] * bd.Pos((xa + xb) / 2, b0.Y, zb) * bd.Box(xb - xa, 4, 4)
        post = p["frame"] * bd.Pos(xb, b0.Y, (top - 0.5 + zb + 2) / 2) * bd.Box(4, 4, zb + 2 - top + 0.5)
        body = body + bar + post
    body = C.cut_holes(body, task, plates, hole_shift=hole_shift, hole_d=hole_d)
    body = C.cut_protrusions(body, task, plates=plates)
    if push:
        na = plates["A"]["frame"].z_dir
        body = body.moved(bd.Location(tuple(-push * v for v in na)))
    solids = body.solids()
    return solids[0] if len(solids) == 1 else body


# --------------------------------------------------------------------------


class Capsule:
    """An arm for planning: its rod path as line segments with a radius --
    the alternative body's whole bundle fits inside. Clearances to it are
    measured exactly as distances, without building any solid."""

    def __init__(self, segs, r):
        self.segs, self.r = segs, r

    def edges(self):
        if not hasattr(self, "_edges"):
            self._edges = [bd.Edge.make_line(tuple(p), tuple(q)) if (q - p).length > 1e-9 else bd.Vertex(*tuple(p))
                           for p, q in self.segs]
        return self._edges

    def distance_to_solid(self, solid) -> float:
        """Surface-to-surface distance (negative = overlap) to a solid."""
        try:
            return min(e.distance_to(solid) for e in self.edges()) - self.r
        except Exception:  # noqa: BLE001
            return -1.0

    def clear_of(self, others, gap: float = 0.0) -> bool:
        for o in others:
            cor = getattr(o, "corridor", None)
            if cor is not None:  # a tool corridor: segment-to-segment, exact
                p, a, rc, L = cor
                if all(_seg_seg(P, Q, p, p + a * L) >= self.r + rc + gap for P, Q in self.segs):
                    continue
                return False
            if _boxes_apart_capsule(self, o, gap):
                continue
            if self.distance_to_solid(o) < gap:
                return False
        return True


def _seg_obb(p, q, ob, step: float = 0.5) -> float:
    """Distance from segment pq to obstacle box ``ob`` (an Obstacle):
    points sampled every ``step`` mm, each measured exactly to the box; the
    true distance is at most step / 2 smaller."""
    import numpy as np

    c = np.array(ob.placement.origin, float)
    R_ = np.array(ob.placement.rot, float)  # columns: the box axes
    half = np.array(ob.size, float) / 2
    P, Q = np.array(tuple(p), float), np.array(tuple(q), float)
    n = max(2, int(np.linalg.norm(Q - P) / step) + 2)
    pts = P[None, :] + (Q - P)[None, :] * np.linspace(0, 1, n)[:, None]
    local = (pts - c) @ R_
    out = np.maximum(np.abs(local) - half, 0.0)
    return float(np.sqrt((out ** 2).sum(axis=1)).min())


from .rods import seg_seg as _seg_seg  # noqa: E402 (exact segment-to-segment distance)


def _boxes_apart_capsule(cap, solid, gap) -> bool:
    import numpy as np

    pts = np.array([tuple(v) for seg in cap.segs for v in seg], float)
    lo, hi = pts.min(axis=0) - cap.r, pts.max(axis=0) + cap.r
    bb = solid.bounding_box()
    return bool(np.any(lo - np.array(tuple(bb.max)) > gap) or np.any(np.array(tuple(bb.min)) - hi > gap))


def _rod_capsule(plate: dict, a: dict, H: bd.Vector, task, d) -> Capsule:
    n, r = _rod_sizing(task, d, alt=True)
    rb = max(r, n * (2 * r + R.GAP) / 2)
    fr = plate["frame"]
    foot = fr.from_local_coords(bd.Vector(plate["x1"] + plate["tab"] / 2, (plate["y0"] + plate["y1"]) / 2,
                                          plate["z0"] + plate["t"] / 2))
    up = foot + fr.z_dir * (plate["t"] / 2 + RISE)
    pts = [foot, up, bd.Vector(*a["W1"]), bd.Vector(*a["W2"]), H]
    # the stretch inside its own plate is not a clash: start at the plate top
    start = foot + fr.z_dir * (plate["t"] / 2 + 0.01)
    return Capsule([(start, up)] + list(zip(pts[1:-1], pts[2:])), rb)


def _rod_proxy(plate: dict, a: dict, H: bd.Vector, task, d) -> bd.Part:
    """What the planner keeps clear for one arm: the arm's rod path as one
    rod as thick as the alternative body's whole bundle (so every layer of
    every body fits inside it)."""
    n, r = _rod_sizing(task, d, alt=True)
    rb = max(r, n * (2 * r + R.GAP) / 2)
    fr = plate["frame"]
    foot = fr.from_local_coords(bd.Vector(plate["x1"] + plate["tab"] / 2, (plate["y0"] + plate["y1"]) / 2,
                                          plate["z0"] + plate["t"] / 2))
    up = foot + fr.z_dir * (plate["t"] / 2 + RISE)
    return R.path([foot, up, bd.Vector(*a["W1"]), bd.Vector(*a["W2"]), H], rb)


def _rod_sizing(task, d, member_area=None, alt=False):
    """Rods per arm and their radius: 1.25 x the largest required section
    (1.21 x more for the alternative body), or exactly ``member_area``."""
    from ..checks.section_min import required_area

    names = list(task.parts)
    need = 1.25 * max(required_area(task, a, b) for i, a in enumerate(names) for b in names[i + 1:])
    area = member_area if member_area is not None else need * (1.21 if alt else 1.0)
    return R.sizing(area)


def _layer_direction(d) -> bd.Vector:
    """Side-by-side rods stack along one direction shared by every arm, so
    each layer meets in its own sphere at the hub."""
    H = bd.Vector(*d["hub"])
    legs = [(bd.Vector(*d["arms"][n]["W2"]) - H).normalized() for n in d["parts"]]
    e = legs[0].cross(legs[1])
    if e.length < 1e-6:
        e = legs[0].cross(bd.Vector(0, 0, 1) if abs(legs[0].Z) < 0.9 else bd.Vector(1, 0, 0))
    return e.normalized()


def _rod_lines(plate: dict, a: dict, d: dict, r: float, n_rods: int) -> list:
    """The polylines of one arm's rods: from inside the plate, square up out
    of it, to the two waypoints, to the hub; one line per layer."""
    fr = plate["frame"]
    e = _layer_direction(d)
    H = bd.Vector(*d["hub"])
    W1, W2 = bd.Vector(*a["W1"]), bd.Vector(*a["W2"])
    xc = plate["x1"] + plate["tab"] / 2
    yc = (plate["y0"] + plate["y1"]) / 2
    lines = []
    for o in R.offsets(n_rods, r):
        foot = fr.from_local_coords(bd.Vector(xc, yc + o, plate["z0"] + plate["t"] / 2))
        up = foot + fr.z_dir * (plate["t"] / 2 + RISE)
        lines.append([foot, up, W1 + e * o, W2 + e * o, H + e * o])
    return lines


def _rods_buildable(task, d) -> str | None:
    """Reasons a rod reference would have a crevice (a bend sharper than
    60 degrees, or two arms meeting at the hub closer than that)."""
    n_rods, r = _rod_sizing(task, d)
    H = bd.Vector(*d["hub"])
    e = _layer_direction(d)
    legs = []
    for n in d["parts"]:
        a = d["arms"][n]
        if n_rods > 1 and abs(e.dot(bd.Vector(*a["s"]))) > 0.9:
            return "rod layers would run along an arm"
        P = C.plate(task, n, H, max(R.PLATE_T, d[n]["t"]), tab=d["tab"])
        for line in _rod_lines(P, a, d, r, n_rods):
            if not R.min_bend_ok(line):
                return "a rod would bend too sharply"
        legs.append((bd.Vector(*a["W2"]) - H).normalized())
    for i in range(len(legs)):
        for j in range(i + 1, len(legs)):
            if math.degrees(math.acos(max(-1.0, min(1.0, legs[i].dot(legs[j]))))) < 60.0:
                return "two arms would meet at the hub too steeply"
    return None


def _passes(task, part, tmp, skip=frozenset()) -> tuple[bool, list]:
    from ..grade import grade

    p = os.path.join(tmp, "candidate.step")
    write_step(part, p)
    r = grade(task, p, optional=False, score=False, skip=skip)
    return r.passed, r.failed()


def generate(seed: int, knobs: dict | None = None, max_tries: int = 800):
    """Return (task, draw, reference solid). Deterministic in seed and knobs.
    ``draw['rejections']`` counts the draws thrown away, by reason."""
    from ..baseline import straight_hull, straight_rod

    knobs = knobs_with_defaults(knobs)
    # the random stream depends on the geometry knobs only, so switching on a
    # requirement or changing the process keeps the same geometry
    geo = {g: knobs[g] for g in GEOMETRY_KNOBS}
    geo_default = {g: KNOBS_DEFAULT[g] for g in GEOMETRY_KNOBS}
    rng = random.Random(f"L3-{seed}-{sorted(geo.items())}" if geo != geo_default else f"L3-{seed}")
    rej: dict = {}
    timing: dict = {}
    import time as _time

    clock = [_time.perf_counter()]

    def lap(stage):
        now = _time.perf_counter()
        timing[stage] = round(timing.get(stage, 0.0) + now - clock[0], 3)
        clock[0] = now

    def reject(why):
        rej[why] = rej.get(why, 0) + 1

    for _ in range(max_tries):
        d = _draw(rng, knobs)
        task = task_from_draw(seed, d)
        from ..requirements import apply_process

        apply_process(task, knobs["process"])  # a part that cannot be made fails
        from ..requirements import apply_key_turn

        apply_key_turn(task)  # every bolt turnable with its hex key, or the draw is rejected
        if knobs["stiffness"]:
            # from the task alone, so it is known before anything is graded;
            # the reference must meet it like any answer, or the draw is rejected
            from ..requirements import apply_stiffness

            apply_stiffness(task)
        # arms wide enough that even the 2 mm narrower alternative keeps the
        # required section (square arm: side = sqrt(area), 25% to spare)
        from ..checks.section_min import required_area

        need = max(required_area(task, a, b) for i, a in enumerate(task.parts) for b in list(task.parts)[i + 1:])
        n_alt, r_alt = R.sizing(1.25 * 1.21 * need)  # the thicker alternative body's rods
        bundle = n_alt * (2 * r_alt + R.GAP)
        # the planner keeps clear a square arm this wide: it holds the rod bundle
        d["arm_w"] = float(max(ARM_W, math.ceil(math.sqrt(1.25 * need)) + 2.0, math.ceil(bundle) + 2.0))
        # tabs long enough that a rod rising from their middle clears the
        # plate's own bolt heads and tool paths
        d["tab"] = float(max(14.0, math.ceil(bundle) + 8.0, 2 * r_alt + 10.0))
        fx = task.fixed_solids(0)
        names = list(fx)
        if not all(_distance_at_least(fx[a], fx[b], 1.0) for i, a in enumerate(names) for b in names[i + 1:]):
            reject("fixed parts too close")
            continue
        # the alternative body's plates are 1 mm thicker than the reference's
        corridors = [C.bolt_corridor(task, b, task.tolerances, max(R.PLATE_T, d[b.part]["t"]) + 1.0)
                     for b in task.bolts(0)]
        keep_out = list(fx.values()) + corridors
        lap("draw and scene")
        why = _plan(task, d, keep_out)
        lap("plan hub and arms")
        if why:
            reject(why)
            continue
        why = _rods_buildable(task, d)
        if why:
            reject(why)
            continue
        H = bd.Vector(*d["hub"])
        ref_arms = []
        for n in d["parts"]:
            pl_ = C.plate(task, n, H, max(R.PLATE_T, d[n]["t"]) + 1.0, tab=d["tab"])  # the thicker plate
            try:
                ref_arms += [_rod_capsule(pl_, d["arms"][n], H, task, d), pl_["solid"]]
            except Exception:  # noqa: BLE001
                ref_arms = None
                break
        if ref_arms is None:
            reject("reference rods could not be built")
            continue
        obs = _obstacles(task, d, keep_out, ref_arms)
        lap("place obstacles")
        if obs is None:
            reject("an obstacle has no clear place")
            continue
        task.obstacles = [obs] if obs else []
        with tempfile.TemporaryDirectory() as tmp:
            # cheapest test first, before the reference is even built: with
            # obstacles, both straight answers must collide (verdict-only
            # grading stops before the slow checks)
            if obs:
                blocked = True
                for base in (straight_hull, straight_rod):
                    try:
                        ok_b, why_b = _passes(task, base(task), tmp)
                    except Exception:  # noqa: BLE001
                        ok_b, why_b = False, ["FILE_VALID"]
                    if ok_b or not ({"CLEAR_PATH", "NO_INTERFERENCE"} & set(why_b)):
                        blocked = False
                        break
                lap("straight answers")
                if not blocked:
                    reject("a straight answer is not blocked")
                    continue
            try:
                ref = build_connector(task, d)
                alt = build_connector(task, d, style="alt")
            except Exception:  # noqa: BLE001 - degenerate geometry: draw again
                reject("reference could not be built")
                continue
            lap("build reference")
            if len(ref.solids()) != 1 or len(alt.solids()) != 1:
                reject("reference not one solid")
                continue
            from ..geometry import BUILD_ROOM, fits_build_volume

            if not fits_build_volume(ref, BUILD_ROOM):
                reject("reference does not fit a Prusa MINI with room to spare")
                continue
            ok_ref, why_ref = _passes(task, ref, tmp)
            if not ok_ref:
                reject("reference fails: " + ",".join(why_ref))
                continue
            # the alternative body must be makeable too: it is the self-test's
            # proof that a differently shaped valid answer passes
            ok_alt, why_alt = _passes(task, alt, tmp, skip=frozenset({"STIFFNESS"}))
            lap("grade reference")
            if not ok_alt:
                reject("alternative body fails: " + ",".join(why_alt))
                continue
        from ..requirements import par as par_for

        task.par = par_for(task, d, build_connector)
        lap("par")
        if task.par is None:
            reject("no lean answer qualifies for par")
            continue

        d["rejections"] = rej
        d["rejected"] = sum(rej.values())
        d["timing_s"] = timing  # where generation spent its time, by stage
        d["summary"] = (f"{knobs['parts']} parts, {len(obs)} obstacles, margin {knobs['margin']:g} mm, "
                        f"rejected {d['rejected']} draws")
        return task, d, ref
    raise RuntimeError(f"seed {seed}: no valid level-3 draw in {max_tries} tries ({rej})")


def _undersized(task):
    return {"hole_d": BOLTS[task.parts["A"].bolt].clear_fine - 0.2}


SELFTEST_VARIANTS = [
    ("reference answer", lambda t: {}, None),
    ("different body, same interfaces", lambda t: {"style": "alt"}, None),
    ("holes shifted 0.05 mm (inside tol)", lambda t: {"hole_shift": 0.05}, None),
    ("holes shifted 0.25 mm", lambda t: {"hole_shift": 0.25}, "HOLES"),
    ("holes shifted 0.5 mm", lambda t: {"hole_shift": 0.5}, "HOLES"),
    ("holes shifted 1 mm", lambda t: {"hole_shift": 1.0}, "HOLES"),
    ("holes shifted 2 mm", lambda t: {"hole_shift": 2.0}, "HOLES"),
    ("holes 0.2 mm under clearance", _undersized, "HOLES"),
    ("body pushed 1 mm into part A", lambda t: {"push": 1.0}, "NO_INTERFERENCE"),
    ("seat on A lifted 1 mm", lambda t: {"lift": 1.0}, "SEATING"),
    ("spur into obstacle O1", lambda t: {"bump": True}, "CLEAR_PATH"),
    ("bar over bolt A1's head", lambda t: {"tool_block": True}, "TOOL_ACCESS"),
]


def _need(task):
    from ..checks.section_min import required_area

    names = list(task.parts)
    return max(required_area(task, a, b) for i, a in enumerate(names) for b in names[i + 1:])


SELFTEST_VARIANTS += [
    # a sound member, well clear of the limit, must pass (at 1.1 x the variant's
    # plates or slanted segments can themselves be the thinnest section)
    ("member at 1.5 x the required section", lambda t: {"member_area": 1.5 * _need(t)}, ("ok", "SECTION_MIN")),
    ("member at 0.9 x the required section", lambda t: {"member_area": 0.9 * _need(t)}, "SECTION_MIN"),
    ("0.4 mm fin: fits, cannot be made", lambda t: {"fin": True}, "*_WALL_MIN"),  # the task's process
]
