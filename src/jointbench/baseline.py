"""Scripted baselines that read the task and never think. They mark floors:
if one passes a task, that task does not measure what it is meant to.

``naive``     Knowledge-free. Fills the space between the two faces with one
              rectangular block covering every bolt and drills a clearance
              hole straight through on each bolt axis. Knows nothing about
              bolt heads, grip, protrusions or where the fixed bodies are.

``template``  The level-1 trick, applied blindly. It knows everything level 1
              teaches -- a plate on each face, a straight wall beside the
              bolts, holes only through the plates, room for the motor boss
              and shaft -- but assumes, as at level 1, that face B is
              parallel to face A. It answers the question: does a level
              defeat a memorised answer to the level below?

``hull``      The level-2 answer applied blindly, as test agents wrote it: a
              plate on every face with a tab towards a common hub point, the
              tabs joined by one convex hull, holes and the motor's
              clearances cut. Straight, no routing.

``rod``       The same plates and tabs joined by straight round rods from
              each tab to the hub.

All four are built from task.json alone, in each face's own frame, so an
arbitrary overall placement of the scene does not by itself defeat them;
only the situation can. Level 3 admits a task only if hull and rod both
collide with something.
"""
from __future__ import annotations

import os

import build123d as bd

from .catalogue import BOLTS, edge_margin
from .geometry import write_step
from .grade import grade
from .task import Task

BASELINES = ("naive", "template", "hull", "rod")


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def _frame_a(task: Task):
    fa, fb = task.faces(0)[:2]
    plane = bd.Plane(origin=fa.origin, z_dir=fa.normal)
    height = _dot([fb.origin[i] - fa.origin[i] for i in range(3)], fa.normal)
    return plane, height


def naive_block(task: Task) -> bd.Part:
    plane, depth = _frame_a(task)
    bolts = task.bolts(0)
    local = [plane.to_local_coords(bd.Vector(*b.position)) for b in bolts]
    m = max(BOLTS[b.size].d * 2 for b in bolts)
    xs, ys = [p.X for p in local], [p.Y for p in local]
    x0, x1, y0, y1 = min(xs) - m, max(xs) + m, min(ys) - m, max(ys) + m
    block = plane * bd.Pos((x0 + x1) / 2, (y0 + y1) / 2, depth / 2) * bd.Box(x1 - x0, y1 - y0, depth)
    for b in bolts:
        bt = BOLTS[b.size]
        bp = bd.Plane(origin=b.position, z_dir=b.axis)
        block = block - bp * bd.Pos(0, 0, depth / 2) * bd.Cylinder(bt.clear_medium / 2, depth + 2)
    return block


def level1_template(task: Task) -> bd.Part:
    plane, height = _frame_a(task)
    names = list(task.parts)[:2]
    rects, ts = {}, {}
    for name in names:
        bt = BOLTS[task.parts[name].bolt]
        loc = [plane.to_local_coords(bd.Vector(*b.position)) for b in task.bolts(0) if b.part == name]
        e = edge_margin(bt)
        rects[name] = [min(p.X for p in loc) - e, min(p.Y for p in loc) - e,
                       max(p.X for p in loc) + e, max(p.Y for p in loc) + e]
        ts[name] = max(5.0, bt.grip_min)
    a, b = names
    x_wall = max(rects[a][2], rects[b][2])
    wall = [x_wall, min(rects[a][1], rects[b][1]), x_wall + 8.0, max(rects[a][3], rects[b][3])]

    def slab(r, z0, t):
        w, h = r[2] - r[0], r[3] - r[1]
        return plane * bd.Pos((r[0] + r[2]) / 2, (r[1] + r[3]) / 2, z0) * bd.extrude(bd.Rectangle(w, h), t)

    def grow(r, s):
        return [min(r[0], s[0]), min(r[1], s[1]), max(r[2], s[2]), max(r[3], s[3])]

    body = slab(grow(rects[a], wall), 0.0, ts[a]) + slab(wall, 0.0, height) \
        + slab(grow(rects[b], wall), height - ts[b], ts[b])
    for bolt in task.bolts(0):
        bt = BOLTS[bolt.size]
        t = ts[bolt.part]
        bp = bd.Plane(origin=bolt.position, z_dir=bolt.axis)
        body = body - bp * bd.Pos(0, 0, t / 2) * bd.Cylinder(bt.clear_medium / 2, t + 2)
    for name in names:
        pp = task.poses[0][name]
        n = pp.vec((0, 0, 1))
        for pr in task.parts[name].protrusions:
            c = pp.point((pr.x, pr.y, 0.0))
            h = pr.height + 1.0
            body = body - bd.Plane(origin=c, z_dir=n) * bd.Pos(0, 0, h / 2 - 0.01) * bd.Cylinder(
                pr.radius + 0.5, h + 0.02)
    return body


def _plates_to_hub(task: Task):
    from .levels import construct as C

    H = C.hub_point(task)
    plates = {n: C.plate(task, n, H, max(5.0, BOLTS[task.parts[n].bolt].grip_min)) for n in task.parts}
    return C, H, plates


def straight_hull(task: Task) -> bd.Part:
    C, H, plates = _plates_to_hub(task)
    quads = [q for p in plates.values() for q in C.tab_quad(p)]
    body = C.hull(quads)
    for p in plates.values():
        body = body + p["solid"]
    body = C.cut_holes(body, task, plates)
    return C.cut_protrusions(body, task, plates=plates)


def straight_rod(task: Task) -> bd.Part:
    C, H, plates = _plates_to_hub(task)
    body = bd.Pos(*H) * bd.Sphere(7.0)
    for p in plates.values():
        T = C.centre(C.tab_quad(p, 10.0))
        v = H - T
        rod = bd.Plane(origin=tuple(T), z_dir=tuple(v.normalized())) * bd.Pos(0, 0, v.length / 2) \
            * bd.Cylinder(5.0, v.length)
        body = body + p["solid"] + rod
    body = C.cut_holes(body, task, plates)
    return C.cut_protrusions(body, task, plates=plates)


def solve(task_dir: str, which: str = "naive"):
    from . import families

    task = Task.load(os.path.join(task_dir, "task.json"))
    fam = families.of(task)
    if which not in fam.baselines:
        raise KeyError(f"family {fam.name!r} has baselines {sorted(fam.baselines)}, not {which!r}")
    build = fam.baselines[which]
    out = os.path.join(task_dir, f"baseline_{which}.step")  # never clobbers an answer
    write_step(build(task), out)
    return grade(task, out), out


# --------------------------------------------------------------------------
# the baselines table


def _table_one(args):
    """Reference and every baseline on one generated task."""
    import tempfile

    from . import families
    from .workspace import write_workspace

    family, lv, s = args
    task, d, ref = families.level(family, lv).generate(s)
    out = {}
    with tempfile.TemporaryDirectory() as tmp:
        ws = write_workspace(task, ref, tmp)
        write_step(ref, os.path.join(ws, "ref.step"))
        for who in ("reference",) + tuple(families.get(family).baselines):
            try:
                if who == "reference":
                    rep = grade(task, os.path.join(ws, "ref.step"), manufacturing=False)
                else:
                    rep, _ = solve(ws, who)
            except Exception:  # noqa: BLE001 - a baseline that cannot even be built fails
                out[who] = (False, None, False)
                continue
            m = rep.info.get("material")
            out[who] = (rep.passed, m["ratio"] if (rep.passed and m) else None, bool(m and m["counts"]))
    return lv, s, out


def table(levels=(1, 2, 3), n: int = 10, seed: int = 0, progress=None, jobs: int = 1,
          family: str = "connect") -> dict:
    """Generate ``n`` tasks per level, run every baseline and the reference,
    and count passes; for parts that pass, the material ratio (par / volume)
    and whether it counts. Returns {level: {who: {...}}}."""
    from . import families
    from .jobs import pmap

    names = ("reference",) + tuple(families.get(family).baselines)
    out = {lv: {w: {"passed": 0, "n": 0, "ratios": [], "counting": 0} for w in names} for lv in levels}
    items = [(family, lv, s) for lv in levels for s in range(seed, seed + n)]
    for lv, s, res in pmap(_table_one, items, jobs):
        for who, (ok, ratio, counts) in res.items():
            r = out[lv][who]
            r["n"] += 1
            r["passed"] += int(ok)
            if ratio is not None:
                r["ratios"].append(ratio)
                r["counting"] += int(counts)
        if progress:
            progress(lv, s)
    return out


def table_text(res: dict) -> str:
    lv = sorted(res)
    who = list(next(iter(res.values())))
    head = "| who | " + " | ".join(f"level {x}" for x in lv) + " |"
    lines = [head, "|---|" + "---|" * len(lv)]
    for w in who:
        cells = []
        for x in lv:
            r = res[x][w]
            cell = f"{r['passed']}/{r['n']}"
            if r["ratios"]:
                rs = sorted(r["ratios"])
                cell += f", material {rs[len(rs) // 2]:.2f} (median of {len(rs)}; {r['counting']} count)"
            cells.append(cell)
        lines.append(f"| {w} | " + " | ".join(cells) + " |")
    return "\n".join(lines)
