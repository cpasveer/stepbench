"""Self-test: proof that the motion grader can be trusted.

For each mechanism type and a few seeds and settings, the removed part is
rebuilt in deliberately different ways and graded:

``reference``   the removed part itself                      must pass
``reshaped``    a different body, same joints               must pass
``shift_d``     one joint moved d = 0.05, 0.25, 0.5, 1 mm    verdict must match an independent
                (Oldham: one tongue turned d degrees)       computation of the path (numeric
                                                            solve of the joints as built), and
                                                            the grader's error must equal it
``fouls``       a bump reaching a fixed part at one angle   must fail NO_INTERFERENCE there
``jams``        a length that cannot close the loop         must fail ASSEMBLES
``straight``    with obstacles: the unbent, straight link   must fail NO_INTERFERENCE

The jam and foul rows also report *where* they fail, to show the reason
names the input value. A second test, ``agree``, compares the closed-form
solver with the independent numeric solve on every generated mechanism.
"""
from __future__ import annotations

import copy
import json
import math
import time

from OCP.BRepExtrema import BRepExtrema_DistShapeShape

from . import generate, kin
from .grade import grade_solid
from .interfere import location
from .kin import Feature, Mechanism
from .mechs import BOSS, HOLE, W, link_recipe, yoke_recipe
from .parts3d import build

CONFIGS = {
    "fourbar": [{"remove": "coupler"}, {"remove": "coupler", "output_on_removed": True, "obstacles": 1},
                {"remove": "rocker", "pivots": "withheld"}, {"remove": "crank"}],
    "slider": [{"remove": "rod"}, {"remove": "slider"}, {"remove": "rod", "output_on_removed": True, "obstacles": 1}],
    "yoke": [{"remove": "yoke"}, {"remove": "crank"}],
    "oldham": [{"remove": "disc"}],
    "cam": [{}], "geneva": [{}], "ratchet": [{}],
}
PATH_FAIL = ("OUTPUT", "ASSEMBLES")
SHIFTS = (0.05, 0.25, 0.5, 1.0)


def _world(m: Mechanism, rem: str, solid):
    return solid.moved(kin.pose_to_placement(m.frame, m.parts[rem].shown).location())


def _feats(m: Mechanism, rem: str) -> dict:
    return {k: copy.copy(v) for k, v in m.parts[rem].features.items()}


def _link(m: Mechanism, rem: str, feats: dict, width=None, boss=BOSS, via=None):
    chain = m.meta["chain"][rem]
    z = m.parts[rem].recipe["z"]
    w = width if width is not None else m.meta.get("width", {}).get(rem, W)
    v = m.meta.get("via", {}) if via is None else via
    return build(link_recipe(feats, chain, tuple(z), width=w, via=v, boss=boss))


def _moved_feature(f: Feature, d) -> Feature:
    g = copy.copy(f)
    g.xy = (f.xy[0] + d[0], f.xy[1] + d[1])
    return g


# ---------------------------------------------------------------- per type


def reshaped(m: Mechanism, rem: str):
    rec = copy.deepcopy(m.parts[rem].recipe)
    if rem in m.meta.get("chain", {}):
        w = m.meta.get("width", {}).get(rem, W)
        s = _link(m, rem, _feats(m, rem), width=w - 3, boss=BOSS - 2)
        # lightening holes along the first segment
        ch = m.meta["chain"][rem]
        a, b = m.parts[rem].features[ch[0]].xy, m.parts[rem].features[ch[-1]].xy
        if math.dist(a, b) > 45 and not m.meta.get("via"):
            import build123d as bd
            z0, z1 = m.parts[rem].recipe["z"]
            for t in (0.35, 0.65):
                c = (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
                s = s - bd.Pos(c[0], c[1], z0 - 1) * bd.Cylinder(1.5, z1 - z0 + 2, align=(bd.Align.CENTER,) * 2 + (bd.Align.MIN,))
            s = s.solids()[0]
        return s
    if rem == "yoke":
        y = m.meta["yoke"]
        return build(yoke_recipe(y["sd"], y["c"], y["Ls"] + 2.0, y["Lf"] - 4.0, y["Wf"] - 4.0, y["XB"] + 2.0, y["xo"]))
    if rem == "slider":
        L = m.parts[rem].features["tongue"].along[1]  # a chamfered block, as long as the reference
        rec["plan"] = [{"poly": [[-L, -7], [L - 2, -7], [L, -5], [L, 5], [L - 2, 7], [-L, 7]]}]
        return build(rec)
    if rem == "disc":
        r = rec["plan"][0]["disc"][1]
        rec["plan"] = [{"box": [0.0, 0.0, 1.5 * r, 1.5 * r, 45.0]}]
        return build(rec)
    raise ValueError(rem)


def shifted(m: Mechanism, rem: str, delta: float):
    """One joint moved by ``delta`` mm (or turned ``delta`` degrees, for the
    Oldham disc's upper tongue). Returns (solid, the moved features)."""
    feats = _feats(m, rem)
    if rem in m.meta.get("chain", {}):
        ch = m.meta["chain"][rem]
        a, b = ch[0], ch[-1]
        fa, fb = feats[a].xy, feats[b].xy
        L = math.dist(fa, fb)
        u = ((fb[0] - fa[0]) / L, (fb[1] - fa[1]) / L)
        feats[b] = _moved_feature(feats[b], (delta * u[0], delta * u[1]))
        return _link(m, rem, feats), feats
    rec = copy.deepcopy(m.parts[rem].recipe)
    if rem == "yoke":
        y = m.meta["yoke"]
        c = (y["c"][0] + delta, y["c"][1])
        feats["slot"] = _moved_feature(feats["slot"], (delta, 0.0))
        return build(yoke_recipe(y["sd"], c, y["Ls"], y["Lf"], y["Wf"], y["XB"], y["xo"])), feats
    if rem == "slider":
        p = rec["pins"][0]
        rec["pins"][0] = [p[0], p[1] + delta, *p[2:]]  # across the guide: along it would change nothing
        feats["pin"] = _moved_feature(feats["pin"], (0.0, delta))
        return build(rec), feats
    if rem == "disc":
        rec["add"][1]["box3"][6] += delta  # turn the upper tongue
        g = copy.copy(feats["t2"])
        g.dir = feats["t2"].dir + math.radians(delta)
        feats["t2"] = g
        return build(rec), feats
    raise ValueError(rem)


def path_error(task: dict, m: Mechanism, rem: str, feats: dict):
    """The output's largest departure from the table for a removed part with
    these joints, computed independently: Newton iteration on the joint
    equations (kin.numeric_solve), not the grader's closed-form steps, and
    from the features as built, not as measured. None if it cannot move."""
    m2 = m.copy()
    m2.parts[rem] = copy.copy(m.parts[rem])
    m2.parts[rem].features = feats
    vals = task["table"]["inputs"]
    sols = kin.numeric_solve(m2, vals, start={p: m.parts[p].shown for p in m.moving()})
    if any(s is None for s in sols):
        return None
    fixed = {p.id: p.shown for p in m2.parts.values() if p.kind != "link"}
    worst = 0.0
    for s, tv in zip(sols, task["table"]["plane"]):
        v = kin.output_value(m2, {**fixed, **s})
        if m2.output["kind"] == "point":
            e = math.dist(v, tv[:2])
        else:
            e = abs(math.degrees(kin.wrap(math.radians(v[0] - tv[0]))))
        worst = max(worst, e)
    return worst


def jams(m: Mechanism, rem: str, task: dict):
    """A part with a length that cannot keep the loop closed at some input value."""
    feats = _feats(m, rem)
    sols, _ = kin.sweep(m, kin.input_values(m, 1.0))
    st = next((s for s in m.steps if rem in (s.get("a"), s.get("b"), s.get("part"), s.get("c"))), None)
    if m.type == "fourbar":
        rrr = m.steps[1]
        P = [kin.world_pt(m, s.poses, *m.joints[rrr["ja"]].other(rrr["a"])) for s in sols]
        Q = [kin.world_pt(m, s.poses, *m.joints[rrr["jb"]].other(rrr["b"])) for s in sols]
        dmax = max(math.dist(p, q) for p, q in zip(P, Q))
        la = math.dist(m.feat("coupler", "j_c1").xy, m.feat("coupler", "j_r1").xy)
        lb = math.dist(m.feat("rocker", "j_r0").xy, m.feat("rocker", "j_r1").xy)
        short = la + lb - dmax + 1.0
        ch = m.meta["chain"][rem]
        a, b = ch[0], ch[-1]
        sign = 1.0 if rem == "crank" else -1.0  # a longer crank, a shorter coupler or rocker
        fa, fb = feats[a].xy, feats[b].xy
        L = math.dist(fa, fb)
        u = ((fb[0] - fa[0]) / L, (fb[1] - fa[1]) / L)
        feats[b] = _moved_feature(feats[b], (sign * short * u[0], sign * short * u[1]))
        if "mark" in feats and rem == "coupler":
            pass
        return _link(m, rem, feats)
    if m.type == "slider":
        if rem == "rod":
            ch = m.meta["chain"][rem]
            g = m.parts["ground"].features["guide"]
            extra = g.along[1] - g.along[0]
            a, b = ch[0], ch[-1]
            fa, fb = feats[a].xy, feats[b].xy
            L = math.dist(fa, fb)
            u = ((fb[0] - fa[0]) / L, (fb[1] - fa[1]) / L)
            feats[b] = _moved_feature(feats[b], (extra * u[0], extra * u[1]))
            if "mark" in feats:
                feats["mark"] = _moved_feature(feats["mark"], (extra * u[0], extra * u[1]))
            return _link(m, rem, feats)
        if rem == "crank":
            rod = m.feat("rod", "j_c1").xy, m.feat("rod", "j_s1").xy
            L = math.dist(*rod)
            feats["pin"] = _moved_feature(feats["pin"], (L, 0.0))
            return _link(m, rem, feats)
        return None
    if m.type == "yoke":
        if rem == "yoke":
            y = m.meta["yoke"]
            return build(yoke_recipe(y["sd"], y["c"], y["Ls"] / 2, y["Lf"], y["Wf"], y["XB"], y["xo"]))
        feats["pin"] = _moved_feature(feats["pin"], (25.0, 0.0))
        return _link(m, rem, feats)
    if m.type == "oldham":
        rec = copy.deepcopy(m.parts[rem].recipe)
        rec["add"][1]["box3"][6] = 0.0  # both tongues parallel
        return build(rec)
    return None



def guided(m: Mechanism, rem: str, d: float):
    """GUIDED at its limit. Scotch yoke (a guide of two blocks): a centred bar
    whose worst block cover is half the block + 1 mm (d > 0, must pass), or
    whose end stops 1 mm short of the far block (d < 0, must fail). Oldham
    disc (one continuous groove): the first tongue cut so its worst bearing is
    1.5 x its width + d mm. The variant is part of the reference body, so
    nothing new can collide. None where it does not apply."""
    from .grade import DEFAULT_FITS
    from .grade import guided as check

    sols, _ = kin.sweep(m, kin.input_values(m, 1.0))
    values = kin.input_values(m, 1.0)

    def margin(sol):
        mm = m.copy()
        mm.meta["_guided_check"] = rem
        return check(mm, rem, sol, values, sols, DEFAULT_FITS).value

    if m.type == "yoke" and rem == "yoke":
        y = m.meta["yoke"]

        def solid(h):
            rec = yoke_recipe(y["sd"], y["c"], y["Ls"], y["Lf"], y["Wf"], y["XB"], y["xo"])
            rec["plan"][1] = {"box": [0.0, 0.0, 2 * h, W, 0.0]}
            return build(rec)
        # from a bar that cannot reach either block at the ends of its stroke
        # (it still clears the slot plate: D >= plate + stroke + 3) to the reference bar
        lo, hi = y["D"] - 1.0 - max(abs(s.poses["yoke"][0]) for s in sols), y["XB"]
        from .grade import _blocks
        shortest = min(b - a for a, b in _blocks(m, "ground", "guide"))
        # d < 0: first the bar that just reaches 0.5 mm into the far block
        target = 1.0 if d > 0 else 0.5 - DEFAULT_FITS["block_cover"] * shortest
    elif m.type == "oldham" and rem == "disc":
        rec0 = m.parts[rem].recipe

        def solid(L):
            rec = copy.deepcopy(rec0)
            rec["add"][0]["box3"][3] = L
            return build(rec)
        lo, hi = 1.0, rec0["add"][0]["box3"][3]
        target = d
    else:
        return None
    if margin(solid(hi)) < target or margin(solid(lo)) > target:
        return None
    for _ in range(30):
        mid = (lo + hi) / 2
        if margin(solid(mid)) > target:
            hi = mid
        else:
            lo = mid
    if m.type == "yoke" and d < 0:
        return solid(hi - 1.5)  # 0.5 mm in, made 1.5 mm shorter: it ends 1 mm short of the block
    return solid(hi)


def fouls(m: Mechanism, rem: str, solids: dict, depth: float = 2.0):
    """The part with a bump that just reaches the nearest fixed part at the
    input value where the part comes closest to it. Returns (solid, value)."""
    import build123d as bd
    target = "housing" if "housing" in solids else "ground"
    tsolid = solids[target]
    if target == "ground":  # no housing (Oldham): aim at the frame's corner posts
        from .parts3d import _solid3
        posts = [a for a in m.parts["ground"].recipe.get("add", []) if "cyl" in a]
        if posts:
            tsolid = _solid3(posts[0])
            for a in posts[1:]:
                tsolid = tsolid + _solid3(a)
    vals = kin.input_values(m, 5.0)
    sols, _ = kin.sweep(m, vals)
    best = None
    part = solids[rem]
    for v, s in zip(vals, sols):
        mv = part.moved(location(s.poses[rem]))
        d = BRepExtrema_DistShapeShape(mv.wrapped, tsolid.wrapped)
        if d.Value() > 1e-6 and (best is None or d.Value() < best[0]):
            pa, pb = d.PointOnShape1(1), d.PointOnShape2(1)
            best = (d.Value(), v, s.poses[rem], (pa.X(), pa.Y(), pa.Z()), (pb.X(), pb.Y(), pb.Z()))
    dist, v, pose, pa, pb = best
    inv = kin.inverse(pose)
    la = kin.apply(inv, pa[:2])
    lb = kin.apply(inv, pb[:2])
    dx, dy = lb[0] - la[0], lb[1] - la[1]
    n = math.hypot(dx, dy)
    if n < 1e-9:
        return None, v
    L = dist + 4.0 + depth  # from 4 mm inside the part to ``depth`` mm into the fixed part
    bb = part.bounding_box()
    zlo, zhi = m.parts[rem].recipe["z"] if m.parts[rem].recipe else (bb.min.Z, bb.max.Z)
    zc = min(max(pa[2], zlo + 1.5), zhi - 1.5)  # in the part's own plate, clear of its neighbours
    c = (la[0] + dx / n * (L / 2 - 4.0), la[1] + dy / n * (L / 2 - 4.0))
    bump = bd.Pos(c[0], c[1], zc) * bd.Rot(0, 0, math.degrees(math.atan2(dy, dx))) * bd.Box(L, 4.0, 3.0)
    out = (part + bump).solids()
    return (out[0] if len(out) == 1 else None), v


def mirrored(m: Mechanism, rem: str):
    """The bent link with each bend on the other side of its pin line: same
    holes, wrong shape. Against its obstacle it must foul."""
    via = m.meta.get("via") or {}
    if not via:
        return None
    feats = _feats(m, rem)
    flipped = {}
    for key, w in via.items():
        a, b = key.split("-")
        pa, pb = feats[a].xy, feats[b].xy
        dx, dy = pb[0] - pa[0], pb[1] - pa[1]
        L2 = dx * dx + dy * dy
        t = ((w[0] - pa[0]) * dx + (w[1] - pa[1]) * dy) / L2
        foot = (pa[0] + t * dx, pa[1] + t * dy)
        flipped[key] = [2 * foot[0] - w[0], 2 * foot[1] - w[1]]
    return _link(m, rem, feats, via=flipped)


def straight(m: Mechanism, rem: str):
    if not m.meta.get("via"):
        return None
    return _link(m, rem, _feats(m, rem), via={})


# ---------------------------------------------------------------- running


def _grade(task, m, solids, rem, solid):
    hidden = Mechanism.from_json(task["mechanism"])
    for pid, p in hidden.parts.items():
        if pid != rem and pid in solids:
            p.solid = solids[pid]
    return grade_solid(task, hidden, _world(m, rem, solid))


def rows_for(mtype: str, seed: int, knobs: dict) -> list[dict]:
    t0 = time.perf_counter()
    task, m, ref = generate.make(mtype, seed, knobs)
    gen_s = time.perf_counter() - t0
    rem = task["removed"]
    solids = m.meta["solids"]
    unit = "deg" if rem == "disc" else "mm"
    cases = [("reference", lambda: solids[rem], None),
             ("reshaped", lambda: reshaped(m, rem), None)]
    cases += [(f"shift_{d:g}", (lambda d=d: shifted(m, rem, d)), "path") for d in SHIFTS]
    cases += [
             ("fouls", lambda: fouls(m, rem, solids), ("NO_INTERFERENCE",)),
             ("jams", lambda: jams(m, rem, task), ("ASSEMBLES",)),
             ("straight", lambda: straight(m, rem), ("NO_INTERFERENCE",)),
             ("mirrored", lambda: mirrored(m, rem), ("NO_INTERFERENCE",)),
             ("guided_limit_inside", lambda: guided(m, rem, 1.0), None),
             ("guided_limit_out", lambda: guided(m, rem, -1.0), ("GUIDED",))]
    out = []
    for name, fn, expect in cases:
        try:
            got = fn()
        except Exception as e:  # noqa: BLE001
            out.append({"type": mtype, "seed": seed, "knobs": knobs, "variant": name, "ok": False,
                        "error": f"could not build: {e}"})
            continue
        where = feats = None
        if isinstance(got, tuple):
            if expect == "path":
                got, feats = got
            else:
                got, where = got
        if got is None:
            continue  # not applicable to this part
        t1 = time.perf_counter()
        rep = _grade(task, m, solids, rem, got)
        dt = time.perf_counter() - t1
        failed = rep.failed()
        extra = {}
        if expect is None:
            ok = rep.ok
        elif expect == "path":
            # the verdict must match an independent computation of the path
            dev = path_error(task, m, rem, feats)
            out_chk = next((c for c in rep.checks if c.name == "OUTPUT"), None)
            tol = task["tolerances"]["point_mm" if m.output["kind"] == "point" else "angle_deg"]
            extra = {"independent_error": None if dev is None else round(dev, 6),
                     "grader_error": out_chk.value if out_chk is not None else None, "tol": tol}
            if dev is None:
                ok = (not rep.ok) and failed[0] == "ASSEMBLES"
            elif abs(dev - tol) < 0.02 * tol:
                ok = True  # on the limit: either verdict is right
            elif dev > tol:
                ok = (not rep.ok) and failed[0] in PATH_FAIL
            else:
                ok = "OUTPUT" not in failed
            if out_chk is not None and out_chk.value is not None and dev is not None:
                ok = ok and abs(out_chk.value - dev) < 1e-4
        else:
            ok = (not rep.ok) and failed[0] in expect
        row = {"type": mtype, "seed": seed, "knobs": knobs, "removed": rem, "variant": name,
               "verdict": "pass" if rep.ok else "fail", "failed": failed, "ok": ok, "seconds": round(dt, 3),
               "reason": next((c.reason for c in rep.checks if c.ok is False), rep.checks[-1].reason),
               "unit": unit, **extra}
        if where is not None:
            row["closest_at"] = round(where % 360, 1)
            hit = next((c for c in rep.checks if c.name == "NO_INTERFERENCE"), None)
            if hit is not None and hit.details.get("clashes"):
                angs = [c[0] % 360 for c in hit.details["clashes"]]
                row["fouls_from_to"] = [round(min(angs), 1), round(max(angs), 1)]
        if name == "reference":
            row["generate_s"] = round(gen_s, 3)
            row["agreement_mm"] = task["generation"]["agreement_mm"]
        out.append(row)
    return out


def _job(args):
    try:
        if args[0] in ("cam", "geneva", "ratchet"):
            return contact_rows(args[0], args[1])
        return rows_for(*args)
    except Exception as e:  # noqa: BLE001 - reported as an unexpected row
        import traceback
        return [{"type": args[0], "seed": args[1], "knobs": args[2], "variant": "generate", "ok": False,
                 "error": f"{type(e).__name__}: {e} " + traceback.format_exc().splitlines()[-3]}]


def run(types, n, seed0, jobs, json_path) -> int:
    types = types or list(CONFIGS)
    work = [(t, seed0 + i, k) for t in types for k in CONFIGS[t] for i in range(n)]
    rows = []
    if jobs == 1:
        for w in work:
            rows += _job(w)
    else:
        import multiprocessing as mp
        with mp.get_context("spawn").Pool(None if jobs == 0 else jobs) as pool:
            for r in pool.imap(_job, work):
                rows += r
    bad = [r for r in rows if not r["ok"]]
    for r in rows:
        tag = "as expected" if r["ok"] else "UNEXPECTED"
        k = ",".join(f"{a}={b}" for a, b in r["knobs"].items())
        print(f"{tag:<11} {r['type']:<8} s{r['seed']:<3} {k:<45} {r['variant']:<11} "
              f"{r.get('verdict', '-'):<5} {r.get('seconds', 0):>5.2f}s  {r.get('reason', r.get('error', ''))[:110]}")
    print(f"\n{len(rows) - len(bad)}/{len(rows)} rows as expected")
    if json_path:
        with open(json_path, "w") as f:
            json.dump(rows, f, indent=1)
    return 0 if not bad else 1


def agree(types, n) -> int:
    types = types or ["fourbar", "slider", "yoke", "oldham"]
    worst = 0.0
    total = 0
    for t in types:
        for s in range(n):
            task, m, _ = generate.make(t, s)
            a = kin.agreement(m, 1.0)
            total += 1
            worst = max(worst, a.get("mm", math.inf))
            print(f"{t:<8} s{s:<3} closed form vs numeric: {a.get('mm', float('nan')):.2e} mm, "
                  f"{a.get('deg', float('nan')):.2e} deg over {a.get('n')} poses {'ok' if a.get('ok') else 'DISAGREE'}")
    print(f"\n{total} mechanisms; worst disagreement {worst:.2e} mm")
    return 0 if worst < 1e-6 else 1


# ---------------------------------------------------------------- contact mechanisms


def _offset_closed(pts, d):
    """Points of a closed outline moved ``d`` mm outward along their normals."""
    import numpy as np
    P = np.array(pts, dtype=float)
    T = np.roll(P, -1, axis=0) - np.roll(P, 1, axis=0)
    T /= np.hypot(T[:, 0], T[:, 1])[:, None]
    N = np.stack([T[:, 1], -T[:, 0]], axis=1)
    N *= np.sign(np.sum(N * (P - P.mean(axis=0)), axis=1))[:, None]
    return (P + d * N).tolist()


def contact_variants(m: Mechanism, task: dict):
    """(name, recipe or None, expected first failing check or None)."""
    rem = task["removed"]
    rec = m.parts[rem].recipe
    out = [("reference", copy.deepcopy(rec), None)]
    if m.type == "cam":
        r = copy.deepcopy(rec)
        c = m.meta["cam"]
        rr = c["Rb"] - 9.0
        r["holes"] += [[rr * math.cos(a), rr * math.sin(a), 5.0] for a in (2.0, 3.0, 4.0)]
        out.append(("reshaped", r, None))
        for d in (0.05, 0.3, 1.0):
            r = copy.deepcopy(rec)
            r["plan"][0]["spline"] = _offset_closed(rec["plan"][0]["spline"], d)
            out.append((f"profile_{d:g}", r, None if d < 0.2 else "OUTPUT"))
        r = copy.deepcopy(rec)
        r["holes"][1][0] += 1.0  # drive hole 1 mm out: the cam cannot sit on both pins
        out.append(("drive_hole_off", r, "MOTION"))
    elif m.type == "geneva":
        g = m.meta["geneva"]
        r = copy.deepcopy(rec)
        r["holes"] += [[0.45 * g["Rw"] * math.cos(g["gam"] + math.pi + 2 * k * math.pi / g["n"]),
                        0.45 * g["Rw"] * math.sin(g["gam"] + math.pi + 2 * k * math.pi / g["n"]), 4.0]
                       for k in range(g["n"])]
        out.append(("reshaped", r, None))
        r = copy.deepcopy(rec)
        for c in r["cut"]:
            if "box3" in c:
                c["box3"][4] = 5.8  # slots narrower than the pin
        out.append(("narrow_slots", r, "MOTION"))
        r = copy.deepcopy(rec)
        r["cut"] = [c for c in r["cut"] if "box3" in c]
        r["plan"] = [{"disc": [[0.0, 0.0], g["C"] - g["Rl"] - 1.0]}]
        out.append(("no_locking_arcs", r, "LOCKED"))
    elif m.type == "ratchet":
        r = copy.deepcopy(rec)
        Rr, d = m.meta["ratchet"]["Rr"], m.meta["ratchet"]["d"]
        rr = Rr - d - 7.0
        r["holes"] += [[rr * math.cos(a), rr * math.sin(a), 4.0] for a in (1.5, 3.0, 4.5)]
        out.append(("reshaped", r, None))
        r = copy.deepcopy(rec)
        r["plan"][0]["poly"] = [[x, -y] for x, y in rec["plan"][0]["poly"]][::-1]
        out.append(("teeth_reversed", r, "LOCKS_BACKWARD"))  # the pawl now rides over both ways
        r = copy.deepcopy(rec)
        pts = r["plan"][0]["poly"]
        pitch = 2 * math.pi / m.meta["ratchet"]["N"]
        for i in range(1, len(pts), 2):  # move each tip back half a pitch: faces become slopes
            a = math.atan2(pts[i][1], pts[i][0]) + 0.5 * pitch
            pts[i] = [Rr * math.cos(a), Rr * math.sin(a)]
        out.append(("sloped_faces", r, "LOCKS_BACKWARD"))
    return out


def contact_rows(mtype: str, seed: int) -> list[dict]:
    from . import contact
    task, m, ref = contact.make(mtype, seed)
    rem = task["removed"]
    solids = m.meta["solids"]
    hidden = Mechanism.from_json(task["mechanism"])
    rows = []
    for name, rec, expect in contact_variants(m, task):
        try:
            solid = contact.build_part(rem, rec)
        except Exception as e:  # noqa: BLE001
            rows.append({"type": mtype, "seed": seed, "variant": name, "ok": False, "error": f"could not build: {e}"})
            continue
        t1 = time.perf_counter()
        rep = contact.grade_solid(task, hidden, solids, _world(m, rem, solid))
        failed = rep.failed()
        ok = rep.ok if expect is None else ((not rep.ok) and expect in failed)
        rows.append({"type": mtype, "seed": seed, "knobs": {}, "removed": rem, "variant": name,
                     "verdict": "pass" if rep.ok else "fail", "failed": failed, "ok": ok,
                     "seconds": round(time.perf_counter() - t1, 3),
                     "reason": next((c.reason for c in rep.checks if c.ok is False), rep.checks[-1].reason)})
    return rows
