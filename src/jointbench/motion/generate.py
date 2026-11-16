"""Answer-first generation of a mechanism task.

1. Draw a mechanism (``mechs``): pins first, lengths read off.
2. Sweep it through its input range in closed form; reject it if it jams
   or if its worst transmission angle is under ``GATE_DEG`` (near a dead
   point the motion is fragile and the two solvers start to disagree).
3. Pose it from its joints alone by Newton iteration and reject it unless
   that agrees with the closed form to 1e-6 mm (``kin.agreement``).
4. Fit a frame plate and a housing round the swept envelope, remove one
   part, optionally place obstacles where a straight version of that part
   would sweep (so the shape must route round them), and write the table
   of input against output.
5. Grade the removed part with the real grader. If anything fails, the
   draw is thrown away and the next one from the same seeded stream used.

Every task that comes out is therefore solvable, and its reference has
passed the grader before anyone sees it.
"""
from __future__ import annotations

import math
import random

from ..geometry import Placement, r6, rot_from_quaternion
from . import kin
from .grade import DEFAULT_FITS, DEFAULT_STEP, DEFAULT_TOL, grade_solid
from .interfere import Scene, _move, plan_hull
from .kin import Feature, Mechanism, Part
from .mechs import BASE_T, BASE_TOP, DEFAULT_REMOVE, DRAW, REMOVABLE, W, band, complete_yoke, link_recipe
from .parts3d import build, build_cached

GENERATOR_VERSION = 3  # 2: obstacles placed on plan outlines, before the housing
# 3: the crank-slider's slider block 1.5 x its width long plus 4 mm (GUIDED);
#    a task records its version and is always rebuilt with it
GATE_DEG = 35.0
TABLE_STEP = 10.0
KNOBS_DEFAULT = {"remove": None, "output_on_removed": None, "pivots": "given", "obstacles": 0,
                 "placed": "anywhere"}


class GenerationFailed(RuntimeError):
    pass


def _random_frame(rng) -> Placement:
    q = [rng.gauss(0, 1) for _ in range(4)]
    R = rot_from_quaternion(*q)
    o = tuple(rng.uniform(-150, 150) for _ in range(3))
    return Placement(o, R)


def _sweep_ok(m: Mechanism, step: float = 2.0):
    sols, jam = kin.sweep(m, kin.input_values(m, step))
    if jam is not None:
        return None, f"jams: {jam}"
    worst = min((mu for s in sols for mu in s.transmission.values()), default=90.0)
    if worst < GATE_DEG:
        return None, f"transmission {worst:.1f} < {GATE_DEG}"
    return sols, ""


def _envelope(m: Mechanism, solids: dict, sols) -> tuple:
    xs, ys = [], []
    for pid, s in solids.items():
        if m.parts[pid].kind != "link":
            continue
        h = plan_hull(s)
        for so in sols:
            P = _move(h, so.poses[pid])
            xs += [P[:, 0].min(), P[:, 0].max()]
            ys += [P[:, 1].min(), P[:, 1].max()]
    return min(xs), min(ys), max(xs), max(ys)


def _add_frame(m: Mechanism, solids: dict, sols, rng) -> None:
    x0, y0, x1, y1 = _envelope(m, solids, sols)
    for p in m.parts.values():
        if p.kind == "ground":
            for f in p.features.values():
                x0, y0, x1, y1 = min(x0, f.xy[0] - 8), min(y0, f.xy[1] - 8), max(x1, f.xy[0] + 8), max(y1, f.xy[1] + 8)
            if p.recipe.get("add"):
                for a in p.recipe["add"]:
                    if "box3" in a:
                        cx, cy, _, lx, ly, _, _ = a["box3"]
                        x0, y0 = min(x0, cx - lx / 2), min(y0, cy - ly / 2)
                        x1, y1 = max(x1, cx + lx / 2), max(y1, cy + ly / 2)
    c = rng.uniform(3.0, 6.0)  # housing clearance round the swept envelope
    tw = 6.0
    x0, y0, x1, y1 = x0 - c, y0 - c, x1 + c, y1 + c
    top = max(s.bounding_box().max.Z for pid, s in solids.items() if m.parts[pid].kind == "link") + 1.0
    g = m.parts["ground"]
    g.recipe["plan"] = [{"box": [(x0 + x1) / 2, (y0 + y1) / 2, x1 - x0 + 2 * tw, y1 - y0 + 2 * tw, 0.0]}]
    g.recipe["z"] = [BASE_TOP - BASE_T, BASE_TOP]
    zc, zh = (BASE_TOP + top) / 2, top - BASE_TOP
    walls = [
        {"box3": [(x0 + x1) / 2, y0 - tw / 2, zc, x1 - x0 + 2 * tw, tw, zh, 0.0]},
        {"box3": [(x0 + x1) / 2, y1 + tw / 2, zc, x1 - x0 + 2 * tw, tw, zh, 0.0]},
        {"box3": [x0 - tw / 2, (y0 + y1) / 2, zc, tw, y1 - y0, zh, 0.0]},
        {"box3": [x1 + tw / 2, (y0 + y1) / 2, zc, tw, y1 - y0, zh, 0.0]},
    ]
    m.parts["housing"] = Part("housing", "obstacle", {}, label="the housing",
                              recipe={"z": [BASE_TOP, top], "add": walls})
    m.meta["housing_clearance"] = c


def _place_obstacles(m: Mechanism, removed: str, n: int, solids: dict, sols, rng) -> int:
    """Keep-out boxes in the removed part's layer, centred where a straight
    version of it passes at some pose; the reference is then bent round them.
    Tested on plan outlines (both are flat plates); the finished task is then
    graded in full, which has the last word. Returns how many were placed."""
    import numpy as np

    from .outline import Outline, outline_gap

    chain = m.meta.get("chain", {}).get(removed)
    if not chain or n <= 0:
        return 0
    part = m.parts[removed]
    z0, z1 = part.recipe["z"]
    zm = (z0 + z1) / 2
    width = m.meta.get("width", {}).get(removed, W)
    # the paths of the neighbours' pins inside this part's layer: an obstacle must keep clear of them
    paths = []
    for j in m.joints.values():
        if removed in (j.a[0], j.b[0]):
            mp, mf = j.other(removed)
            if m.parts[mp].kind == "link":
                r = m.parts[mp].features[mf].size / 2
                P = np.array([kin.world_pt(m, so.poses, mp, mf) for so in sols])
                paths.append((P, r))
    via: dict = {}
    placed = 0
    obstacles = []
    for i in range(n):
        for _ in range(40):
            segs = [(chain[k], chain[k + 1]) for k in range(len(chain) - 1) if f"{chain[k]}-{chain[k + 1]}" not in via]
            if not segs:
                return placed
            a, b = segs[int(rng.random() * len(segs))]
            fa, fb = part.features[a].xy, part.features[b].xy
            if math.dist(fa, fb) < 20:
                continue
            k = int(rng.random() * len(sols))
            f = rng.uniform(0.4, 0.6)
            loc = (fa[0] + f * (fb[0] - fa[0]), fa[1] + f * (fb[1] - fa[1]))
            P = kin.apply(sols[k].poses[removed], loc)
            s = rng.uniform(6.0, 9.0)
            ang = rng.uniform(0, math.pi)
            box = {"box3": [P[0], P[1], zm, s, s, z1 - z0, math.degrees(ang)]}
            ob_solid = build({"z": [z0, z1], "add": [box]})
            corners = np.array([kin.apply((P[0], P[1], ang), (sx * s / 2, sy * s / 2))
                                for sx, sy in ((1, 1), (-1, 1), (-1, -1), (1, -1))])
            # clear of the neighbours' pins (with 1 mm to spare) at every pose
            near = False
            for Pp, r in paths:
                d = np.min(np.hypot(*(Pp[:, None, :] - corners[None]).transpose(2, 0, 1)))
                inside = np.all(np.abs((Pp - np.array(P)) @ np.array([[math.cos(ang), -math.sin(ang)],
                                                                        [math.sin(ang), math.cos(ang)]])) < s / 2, axis=1)
                if d < r + 1.0 + s * 0.71 and (inside.any() or d < r + 1.0 + s):
                    near = True
                    break
            if near or any(math.dist(P, q) < s + 6 for q in obstacles):
                continue
            o_out = Outline(ob_solid, zm, 1e-3)
            d = (fb[0] - fa[0], fb[1] - fa[1])
            ln = math.hypot(*d)
            nrm = (-d[1] / ln, d[0] / ln)
            mid = ((fa[0] + fb[0]) / 2, (fa[1] + fb[1]) / 2)
            ok = False
            for h in (s / 2 + width / 2 + 3, s / 2 + width / 2 + 6, s / 2 + width / 2 + 10):
                for side in ((1, -1) if rng.random() < 0.5 else (-1, 1)):
                    w = (mid[0] + side * h * nrm[0], mid[1] + side * h * nrm[1])
                    v2 = {**via, f"{a}-{b}": [w[0], w[1]]}
                    rec = link_recipe(part.features, chain, (z0, z1), width=width, via=v2)
                    sol = build(rec)
                    l_out = Outline(sol, zm, 1e-3)
                    if any(outline_gap(l_out, so.poses[removed], o_out, (0.0, 0.0, 0.0), reach=2.0)[0] < 1.0
                           for so in sols):
                        continue
                    oid = f"obstacle{placed + 1}"
                    via, part.recipe, solids[removed] = v2, rec, sol
                    m.parts[oid] = Part(oid, "obstacle", {}, label=f"obstacle {placed + 1}",
                                        recipe={"z": [z0, z1], "add": [box]})
                    solids[oid] = ob_solid
                    obstacles.append(P)
                    ok = True
                    break
                if ok:
                    break
            if ok:
                placed += 1
                break
    m.meta["via"] = via
    return placed


def _downstream(m: Mechanism, removed: str) -> list[str]:
    for i, st in enumerate(m.steps):
        placed = [st.get(k) for k in ("part", "a", "b", "c") if st.get(k)]
        if removed in placed:
            later = [p for s in m.steps[i:] for p in (s.get(k) for k in ("part", "a", "b", "c")) if p]
            return [p for p in dict.fromkeys(later) if p != removed]
    return []


def _park(m: Mechanism, pid: str, rng) -> None:
    """Move a part to a 'parked' pose for the brief: same joints, wrong place."""
    p = m.parts[pid]
    for j in m.joints.values():
        if pid in (j.a[0], j.b[0]) and j.other(pid)[0] == "ground":
            f = p.features[j.feature_of(pid)]
            if j.kind == "revolute":
                c = kin.apply(p.shown, f.xy)
                da = math.radians(rng.uniform(70, 110))
                pose = (p.shown[0], p.shown[1], p.shown[2] + da)
                q = kin.apply(pose, f.xy)
                p.shown = (pose[0] + c[0] - q[0], pose[1] + c[1] - q[1], pose[2])
            else:
                u = kin.unit(p.shown[2] + f.dir)
                ds = rng.uniform(15, 25) * (1 if rng.random() < 0.5 else -1)
                p.shown = (p.shown[0] + ds * u[0], p.shown[1] + ds * u[1], p.shown[2])
            return


def world_of(m: Mechanism, xy, z: float) -> tuple:
    return m.frame.point((xy[0], xy[1], z))



def _slide_travel(m: Mechanism, j, removed: str) -> tuple[float, float]:
    """How far the removed part moves along a slide joint over the input range
    (every 1 deg), from where it sits at the first input value."""
    sols, _ = kin.sweep(m, kin.input_values(m, 1.0))
    mine = j.a if j.a[0] == removed else j.b
    other = j.b if mine is j.a else j.a
    c0, d0 = kin._line_world(m, sols[0].poses, *other)
    u = kin.unit(d0)
    ts = []
    for s in sols:
        ca, _ = kin._line_world(m, s.poses, *mine)
        cb, d = kin._line_world(m, s.poses, *other)
        ts.append(kin.dot(kin.sub(ca, cb), kin.unit(d)))
    return min(ts) - ts[0], max(ts) - ts[0]

def make(mtype: str, seed: int, knobs: dict | None = None, max_tries: int = 60, replay: int | None = None,
         version: int | None = None):
    """Returns (task dict, full mechanism, reference solid in world coordinates).

    ``replay``: the attempt that was accepted when the task was made. The
    draws are run again exactly (same random stream, same deterministic
    rejections), and every draw that reached the final grading before that
    attempt is known to have failed it, so nothing is graded: the grader
    rebuilds what it needs from seed, knobs and attempt alone."""
    knobs = {**KNOBS_DEFAULT, **(knobs or {})}
    if knobs["remove"] is None:
        knobs["remove"] = DEFAULT_REMOVE[mtype]
    if knobs["remove"] not in REMOVABLE[mtype]:
        raise ValueError(f"{mtype}: can remove {REMOVABLE[mtype]}, not {knobs['remove']!r}")
    rng = random.Random(f"motion-{mtype}-{seed}-" + ",".join(f"{k}={knobs[k]}" for k in sorted(knobs)))
    why = []
    for attempt in range(max_tries):
        m = DRAW[mtype](rng, dict(knobs, _version=version or GENERATOR_VERSION))
        m.frame = _random_frame(rng) if knobs["placed"] == "anywhere" else Placement((0.0, 0.0, 0.0), ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)))
        sols, msg = _sweep_ok(m)
        if sols is None:
            why.append(msg)
            continue
        for pid, pose in sols[0].poses.items():
            if m.parts[pid].kind == "link":
                m.parts[pid].shown = pose
        sols, msg = _sweep_ok(m)  # shown poses now set: same motion
        if m.meta.get("complete") == "yoke":
            complete_yoke(m, sols)
        agree = kin.agreement(m, 5.0)
        if not agree.get("ok"):
            why.append(f"solvers disagree: {agree}")
            continue
        removed = knobs["remove"]
        solids = {pid: build(p.recipe) for pid, p in m.parts.items() if p.recipe and pid != "ground"}
        n_obs = _place_obstacles(m, removed, int(knobs["obstacles"]), solids, sols, rng)
        if n_obs < int(knobs["obstacles"]):
            why.append(f"placed {n_obs} of {knobs['obstacles']} obstacles")
            continue
        if not m.meta.get("no_housing"):
            _add_frame(m, solids, sols, rng)  # after the obstacles: the housing fits the bent link
            solids["housing"] = build(m.parts["housing"].recipe)
        solids["ground"] = build(m.parts["ground"].recipe)
        task = _task_dict(m, mtype, seed, knobs, removed, rng)
        task["generator_version"] = version or GENERATOR_VERSION
        ref_world = solids[removed].moved(kin.pose_to_placement(m.frame, m.parts[removed].shown).location())
        hidden = Mechanism.from_json(task["mechanism"])
        for pid, p in hidden.parts.items():
            if pid != removed:
                p.solid = solids[pid]
        if replay is not None:
            if attempt < replay:
                why.append("reference failed when the task was made")
                continue
            task["generation"] = {"attempt": attempt, "replayed": True}
            m.meta["solids"] = solids
            return task, m, ref_world
        rep = grade_solid(task, hidden, ref_world)
        if not rep.ok:
            why.append("reference fails: " + "; ".join(c.reason for c in rep.checks if c.ok is False))
            continue
        task["generation"] = {"attempt": attempt, "rejected": [w[:160] for w in why], "agreement_mm": agree["mm"], "agreement_deg": agree["deg"],
                              "reference_seconds": rep.info.get("seconds"), "obstacles": n_obs}
        m.meta["solids"] = solids
        return task, m, ref_world
    raise GenerationFailed(f"{mtype} seed {seed}: no valid draw in {max_tries} tries; last: {why[-3:]}")


def _task_dict(m: Mechanism, mtype: str, seed: int, knobs: dict, removed: str, rng) -> dict:
    vals = kin.input_values(m, TABLE_STEP)
    sols, jam = kin.sweep(m, vals)
    assert jam is None
    plane = [list(kin.output_value(m, s.poses)) for s in sols]
    o = m.output
    if o["kind"] == "point":
        f = m.parts[o["part"]].features[o["feature"]]
        zc = (f.z[0] + f.z[1]) / 2
        world = [list(world_of(m, p, zc)) for p in plane]
    else:
        world = plane
    shown = {pid: m.parts[pid].shown for pid in m.parts}
    given = {}
    for j in m.joints.values():
        if removed in (j.a[0], j.b[0]):
            mp, mf = j.other(removed)
            f = m.parts[mp].features[mf]
            pose = sols[0].poses.get(mp, m.parts[mp].shown)
            xy = kin.apply(pose, f.xy)
            g = {"joint": j.id, "with": mp, "feature": f.kind, "size": f.size, "plane_xy": list(xy),
                 "world": list(world_of(m, xy, (f.z[0] + f.z[1]) / 2)), "z_band": list(f.z)}
            if f.dir is not None:
                g["dir_deg"] = math.degrees(pose[2] + f.dir) % 180
            if j.kind == "slide" and f.along is not None:
                # what the engagement rule turns on: the mate's straight faces
                # end to end (gaps included) and the part's travel along them
                from .measure import bearing_blocks
                t = _slide_travel(m, j, removed)
                p = m.parts[mp]
                blocks = bearing_blocks(p.solid if p.solid is not None else build_cached(p.recipe), f)
                if math.degrees(pose[2] + f.dir) % 360 >= 180:  # state both along dir_deg
                    blocks, t = [(-b, -a) for a, b in reversed(blocks)], (-t[1], -t[0])
                g["mate_blocks"] = [[round(a, 4), round(b, 4)] for a, b in blocks]
                g["travel"] = list(t)
            given[j.id] = g
    full = m.to_json()
    parked = []
    if knobs["pivots"] == "withheld":
        for pid in _downstream(m, removed):
            if pid in m.parts and m.parts[pid].kind == "link":
                _park(m, pid, rng)
                parked.append(pid)
    hidden = m.to_json(hide=removed)
    for p in m.parts.values():  # restore the true shown poses on the full model
        p.shown = shown[p.id]
    task = {
        "format": 1,
        "kind": "motion",
        "type": mtype,
        "id": f"motion-{mtype}-s{seed}",
        "seed": seed,
        "generator_version": GENERATOR_VERSION,
        "knobs": knobs,
        "removed": removed,
        "mechanism": hidden,
        "table": {"inputs": vals, "kind": o["kind"], "plane": plane, "world": world},
        "fits": DEFAULT_FITS,
        "tolerances": DEFAULT_TOL,
        "step_deg": DEFAULT_STEP,
        # withheld: positions are hidden only where the mate is a parked part
        "joints_of_removed": given if knobs["pivots"] == "given" else {
            k: (v if v["with"] not in parked else
                {kk: vv for kk, vv in v.items() if kk not in ("plane_xy", "world", "dir_deg")})
            for k, v in given.items()},
        "parked": parked,
        "_full": full,
    }
    return task
