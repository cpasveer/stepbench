"""Grading a moving mechanism with one part replaced by a submitted solid.

The submitted STEP is the missing part modelled where it sits at the first
input value (input value ``range[0]``), in world coordinates. The grader:

1. checks the file (the core's FILE_VALID);
2. finds the part's joints from its own geometry (``measure``) and matches
   each to the neighbour it must mate with -- ``JOINTS``;
3. rebuilds the mechanism with the measured features and poses it at every
   input step in closed form -- ``ASSEMBLES`` (with where it jams);
4. reads the worst transmission angle over the sweep -- ``TRANSMISSION``;
5. compares the output with the required table -- ``OUTPUT``;
6. places every part at every step and measures exact overlap between
   solids -- ``NO_INTERFERENCE`` (with the input value where it fouls).

The submitted part's placement in the file decides nothing but the
assembly's starting orientation: where it actually sits at each step comes
from its measured joints, so an answer is graded on what it does.
"""
from __future__ import annotations

import itertools
import json
import math
import os
import time
from dataclasses import dataclass, field

from ..checks.base import CheckResult
from . import kin
from .interfere import Scene
from .kin import Mechanism, Part
from .measure import all_features, bearing_blocks, fit_range, z_overlap
from .parts3d import build_cached

DEFAULT_FITS = {"clear_min": 0.1, "clear_max": 0.6, "engage_min": 2.0, "mark_d": 3.0, "mark_tol": 0.1,
                # GUIDED: a sliding part bears on real guide faces at least this
                # many times its own width (the plain-bearing length rule), and
                # covers at least this share of every separate guide block
                "bearing_per_width": 1.5, "block_cover": 0.5}
DEFAULT_TOL = {"point_mm": 0.2, "angle_deg": 0.2, "transmission_min_deg": 20.0, "overlap_volume_max": 0.01}
DEFAULT_STEP = 1.0  # degrees of input between two grading poses (see docs/motion.md)


@dataclass
class Report:
    checks: list
    info: dict = field(default_factory=dict)
    keep: list | None = None  # solutions at the table rows, when asked for

    @property
    def ok(self) -> bool:
        return all(c.ok for c in self.checks if c.required)

    def failed(self) -> list[str]:
        return [c.name for c in self.checks if c.required and c.ok is False]

    def to_json(self) -> dict:
        return {"verdict": "pass" if self.ok else "fail", "checks": [c.to_json() for c in self.checks],
                "info": self.info}

    def text(self, title: str = "") -> str:
        n_ok = sum(1 for c in self.checks if c.ok)
        lines = [f"{'PASS' if self.ok else 'FAIL'}  ({n_ok}/{len(self.checks)} checks)  {title}".rstrip()]
        for c in self.checks:
            tag = "ok  " if c.ok else ("skip" if c.ok is None else "FAIL")
            lines.append(f"  {tag} {c.name:<16} {c.reason}")
        i = self.info
        if "step_deg" in i:
            lines.append(f"  info {i.get('poses', 0)} poses at {i['step_deg']:g} deg steps; largest move of any "
                         f"point between steps {i.get('max_move_mm', 0):.2f} mm; {i.get('exact_tests', 0)} exact "
                         f"overlap tests; {i.get('seconds', 0):.2f} s")
        return "\n".join(lines)


# ------------------------------------------------------------------ helpers


def needs(mech: Mechanism, removed: str, fits: dict) -> list[dict]:
    """What the removed part must offer: one entry per joint (plus the output
    mark if the output point is on it), with the mating feature."""
    out = []
    for j in mech.joints.values():
        if removed not in (j.a[0], j.b[0]):
            continue
        mp, mf = j.other(removed)
        mate = mech.parts[mp].features[mf]
        kind, lo, hi = fit_range(mate, j.kind, fits)
        out.append({"name": j.feature_of(removed), "joint": j.id, "kind": kind, "lo": lo, "hi": hi,
                    "mate": mate, "mate_part": mp})
    o = mech.output
    if o["kind"] == "point" and o["part"] == removed and o["feature"] not in [n["name"] for n in out]:
        d, t = fits["mark_d"], fits["mark_tol"]
        out.append({"name": o["feature"], "joint": None, "kind": "hole", "lo": d - t, "hi": d + t,
                    "mate": None, "mate_part": None, "mark": True})
    return out


def _describe_need(mech: Mechanism, n: dict) -> str:
    if n.get("mark"):
        return f"output mark (hole dia {n['lo']:.2f}-{n['hi']:.2f})"
    lbl = mech.parts[n["mate_part"]].label or n["mate_part"]
    word = {"hole": "hole dia", "pin": "pin dia", "slot": "slot width", "tongue": "tongue width"}[n["kind"]]
    return f"{word} {n['lo']:.2f}-{n['hi']:.2f} for {lbl}'s {n['mate'].kind}"


def candidates(found: list, n: dict, fits: dict) -> list:
    out = []
    for f in found:
        if f.kind != n["kind"] or not (n["lo"] - 1e-6 <= f.size <= n["hi"] + 1e-6):
            continue
        if n["mate"] is not None:
            need = min(fits["engage_min"], 0.8 * (n["mate"].z[1] - n["mate"].z[0]))
            if z_overlap(f.z, n["mate"].z) < need - 1e-6:
                continue
        out.append(f)
    return out


def with_part(mech: Mechanism, removed: str, feats: dict, solid=None) -> Mechanism:
    m = mech.copy()
    old = mech.parts[removed]
    m.parts[removed] = Part(removed, "link", feats, (0.0, 0.0, 0.0), None, old.label, solid)
    return m


def output_error(mech: Mechanism, sol, target) -> float:
    v = kin.output_value(mech, sol.poses)
    if mech.output["kind"] == "point":
        return math.dist(v, target[:2])
    return abs(math.degrees(kin.wrap(math.radians(v[0] - target[0]))))


def _fmt_in(mech: Mechanism, v: float) -> str:
    return f"{v % 360:.1f} deg"



_SLIDES: dict = {}  # whether the part being graded has a sliding joint (for the skipped list)
_BLOCKS: dict = {}  # fixed part recipe + feature -> its bearing blocks


def _blocks(m: Mechanism, pid: str, fid: str, solid=None) -> list:
    f = m.parts[pid].features[fid]
    if f.segments is not None:
        return [tuple(s) for s in f.segments]
    p = m.parts[pid]
    if solid is not None:
        return bearing_blocks(solid, f)
    key = json.dumps([p.recipe, f.to_json()], sort_keys=True)
    if key not in _BLOCKS:
        _BLOCKS[key] = bearing_blocks(p.solid if p.solid is not None else build_cached(p.recipe), f)
    return _BLOCKS[key]


def _cover(a: list, lo: float, hi: float) -> float:
    return sum(max(0.0, min(b, hi) - max(x, lo)) for x, b in a)


def guided(m: Mechanism, removed: str, solid, values, sols, fits) -> CheckResult:
    """GUIDED: every sliding joint of the removed part is carried on real
    guide faces at every pose. The guide is the slot side, the sliding part
    the tongue side; both are measured as their actual flat faces, so the gap
    between two guide blocks carries nothing. At every pose the tongue bears
    on at least bearing_per_width x its width of guide face in all, and,
    where the guide is several separate blocks, covers at least block_cover
    of each block's length (in both: a part held by one short block can tip
    and bind)."""
    k, share = fits["bearing_per_width"], fits["block_cover"]
    worst = None  # (margin, value, text)
    joints = [j for j in m.joints.values() if j.kind == "slide" and removed in (j.a[0], j.b[0])]
    if not joints:
        return None
    notes = []
    for j in joints:
        fa = m.parts[j.a[0]].features[j.a[1]]
        tg, gd = (j.a, j.b) if fa.kind == "tongue" else (j.b, j.a)
        w = m.parts[tg[0]].features[tg[1]].size
        need = k * w
        ts = _blocks(m, *tg, solid if tg[0] == removed else None)
        gs = _blocks(m, *gd, solid if gd[0] == removed else None)
        lt, lg = m.parts[tg[0]].label or tg[0], m.parts[gd[0]].label or gd[0]
        notes.append(f"{lt} in {lg}: {len(gs)} guide block{'s' if len(gs) > 1 else ''}, need {need:.1f} mm bearing")
        for v, s in zip(values, sols):
            _, ang = kin._line_world(m, s.poses, *gd)
            T = kin.line_segments(m, s.poses, *tg, ang, ts)
            G = kin.line_segments(m, s.poses, *gd, ang, gs)
            covers = [_cover(T, lo, hi) for lo, hi in G]
            total = sum(covers)
            cands = [(total - need, f"{lt} bears on only {total:.1f} mm of {lg}'s guide faces "
                                    f"(need {need:.1f}: {k:g} x its {w:.2f} mm width)")]
            if len(G) > 1:
                for (lo, hi), c in zip(G, covers):
                    cands.append((c - share * (hi - lo),
                                  f"{lt} covers only {c:.1f} of {hi - lo:.1f} mm of one of {lg}'s {len(G)} guide "
                                  f"blocks (need {share * (hi - lo):.1f}): carried on the other block alone, it can tip"))
            mg, txt = min(cands)
            if worst is None or mg < worst[0]:
                worst = (mg, v, txt)
    mg, v, txt = worst
    if mg < -1e-6:
        return CheckResult("GUIDED", False, f"at {_fmt_in(m, v)} {txt}", round(mg, 3), "mm")
    return CheckResult("GUIDED", True, "carried on real guide faces at every pose; " + "; ".join(notes)
                       + f"; least margin {mg:.1f} mm at {_fmt_in(m, v)}", round(mg, 3), "mm")

# ------------------------------------------------------------------ grading


def grade_solid(task: dict, mech: Mechanism, solid_world, step_deg: float | None = None,
                solids: dict | None = None, keep_solutions: bool = False) -> Report:
    """Grade a submitted solid (world coordinates, at the first input value)
    for task ``task`` (the dict in motion.json) and mechanism ``mech`` (its
    fixed parts complete, the removed part without features)."""
    t0 = time.perf_counter()
    removed = task["removed"]
    _SLIDES["on"] = any(j.kind == "slide" and removed in (j.a[0], j.b[0]) for j in mech.joints.values())
    fits = {**DEFAULT_FITS, **task.get("fits", {})}
    tol = {**DEFAULT_TOL, **task.get("tolerances", {})}
    step = step_deg or task.get("step_deg", DEFAULT_STEP)
    table_in = task["table"]["inputs"]
    table_out = [tuple(v) for v in task["table"]["plane"]]
    checks: list[CheckResult] = []

    inv = kin.frame_inverse(mech.frame)
    solid = solid_world.moved(inv.location())

    # -- JOINTS
    found = all_features(solid)
    need = needs(mech, removed, fits)
    cand = [candidates(found, n, fits) for n in need]
    missing = [n for n, c in zip(need, cand) if not c]
    if missing:
        n = missing[0]
        near = [f for f in found if f.kind == n["kind"]]
        hint = ""
        if near:
            best = min(near, key=lambda f: abs(f.size - (n["lo"] + n["hi"]) / 2))
            hint = f"; nearest: {best.text()}"
        z = f" reaching z {n['mate'].z[0]:.1f}-{n['mate'].z[1]:.1f}" if n["mate"] is not None else ""
        checks.append(CheckResult("JOINTS", False, f"no {_describe_need(mech, n)}{z}{hint}"
                                  + (f" (+{len(missing) - 1} more missing)" if len(missing) > 1 else ""),
                                  len(missing), "missing"))
        return _skip_rest(checks, ["ASSEMBLES", "TRANSMISSION", "GUIDED", "OUTPUT", "NO_INTERFERENCE"], t0)

    # choose the assignment of found features to joints that assembles best
    best = None
    combos = 0
    for combo in itertools.product(*cand):
        if len({id(c) for c in combo}) < len(combo):
            continue
        combos += 1
        if combos > 400:
            break
        feats = {n["name"]: c.feature() for n, c in zip(need, combo)}
        m = with_part(mech, removed, feats)
        sols, jam = kin.sweep(m, table_in)
        err = max((output_error(m, s, tv) for s, tv in zip(sols, table_out)), default=math.inf)
        # The part is submitted where it sits at the first input value, which
        # fixes which hole is which. Among matchings that move equally well
        # (two equal holes: the rod either way round), keep the one that
        # leaves the part where it was modelled, not turned end for end.
        moved = math.inf
        if sols:
            x, y, a = sols[0].poses[removed]
            moved = math.hypot(x, y) + 50.0 * abs(kin.wrap(a))
        lim = tol["point_mm"] if mech.output.get("kind") == "point" else tol["angle_deg"]
        key = (-len(sols), err > lim, round(moved, 3), err)
        if best is None or key < best[0]:
            best = (key, feats, combo)
    if best is None:
        checks.append(CheckResult("JOINTS", False, "one feature would have to serve two joints: "
                                  + ", ".join(_describe_need(mech, n) for n in need), len(need), "joints"))
        return _skip_rest(checks, ["ASSEMBLES", "TRANSMISSION", "GUIDED", "OUTPUT", "NO_INTERFERENCE"], t0)
    _, feats, combo = best
    checks.append(CheckResult("JOINTS", True, f"{len(need)} found: " + "; ".join(
        f"{c.kind} {c.size:.2f}" + (" (output mark)" if n.get("mark") else f" for {mech.parts[n['mate_part']].label or n['mate_part']}")
        for n, c in zip(need, combo)), len(need), "joints",
        {"features": {n["name"]: c.feature().to_json() for n, c in zip(need, combo)}, "assignments_tried": combos}))

    # -- ASSEMBLES over the full input range at the grading step
    m = with_part(mech, removed, feats, solid)
    m.meta["_guided_check"] = removed  # its sliding joints are judged by GUIDED, on the real faces
    values = sorted(set(round(v, 9) for v in kin.input_values(m, step)) | set(round(v, 9) for v in table_in))
    sols, jam = kin.sweep(m, values)
    if jam is not None:
        checks.append(CheckResult("ASSEMBLES", False, f"jams at {_fmt_in(m, jam.value)}: {jam.reason}",
                                  jam.value, "deg", {"short_by_mm": jam.short_by}))
        return _skip_rest(checks, ["TRANSMISSION", "GUIDED", "OUTPUT", "NO_INTERFERENCE"], t0)
    checks.append(CheckResult("ASSEMBLES", True, f"poses at all {len(values)} input values over "
                              f"{m.range[0]:g}-{m.range[1]:g} deg", len(values), "poses"))

    # -- TRANSMISSION
    worst, at = 180.0, None
    for v, s in zip(values, sols):
        for mu in s.transmission.values():
            if mu < worst:
                worst, at = mu, v
    lim = tol["transmission_min_deg"]
    if worst < lim:
        checks.append(CheckResult("TRANSMISSION", False, f"transmission angle falls to {worst:.1f} deg at "
                                  f"{_fmt_in(m, at)} (need {lim:g}): it would bind there", round(worst, 3), "deg"))
    else:
        checks.append(CheckResult("TRANSMISSION", True, f"worst transmission angle {worst:.1f} deg at "
                                  f"{_fmt_in(m, at)} (need {lim:g})", round(worst, 3), "deg"))

    # -- GUIDED
    g = guided(m, removed, solid, values, sols, fits)
    if g is not None:
        checks.append(g)

    # -- OUTPUT
    by_v = dict(zip(values, sols))
    errs = [(output_error(m, by_v[round(v, 9)], tv), v) for v, tv in zip(table_in, table_out)]
    emax, eat = max(errs)
    lim, unit = (tol["point_mm"], "mm") if m.output["kind"] == "point" else (tol["angle_deg"], "deg")
    what = "output point" if unit == "mm" else "output angle"
    if emax > lim:
        n_bad = sum(1 for e, _ in errs if e > lim)
        checks.append(CheckResult("OUTPUT", False, f"{what} off by {emax:.3f} {unit} at {_fmt_in(m, eat)} "
                                  f"(tol {lim:g}; {n_bad} of {len(errs)} table rows out)", round(emax, 5), unit,
                                  {"errors": [round(e, 5) for e, _ in errs]}))
    else:
        checks.append(CheckResult("OUTPUT", True, f"{what} follows the table within {emax:.4f} {unit} "
                                  f"(tol {lim:g}) over {len(errs)} rows", round(emax, 5), unit))

    # -- NO_INTERFERENCE
    sol_solids = dict(solids) if solids else {}
    for pid, p in m.parts.items():
        if pid == removed:
            sol_solids[pid] = solid
        elif pid not in sol_solids:
            sol_solids[pid] = p.solid if p.solid is not None else build_cached(p.recipe)
    cuts = sorted({z for p in m.parts.values() for f in p.features.values() for z in f.z}
                  | {z for p in m.parts.values() if p.recipe for z in p.recipe["z"]})
    scene = Scene(m.parts, sol_solids, tol["overlap_volume_max"], joints=m.joints, cuts=cuts)
    hits = []
    for v, s in zip(values, sols):
        for a, b, vol in scene.clashes(s.poses):
            hits.append((v, a, b, vol))
    if hits:
        v, a, b, vol = hits[0]  # the first input value where it fouls
        A, B = (b, a) if a != removed and b == removed else (a, b)
        span = sorted({h[0] for h in hits if {h[1], h[2]} == {a, b}})
        rng = f" (from {_fmt_in(m, span[0])} to {_fmt_in(m, span[-1])})" if len(span) > 1 else ""
        checks.append(CheckResult("NO_INTERFERENCE", False,
                                  f"{scene.label[A]} fouls {scene.label[B]} at {_fmt_in(m, v)}: "
                                  f"{vol:.3f} mm3 overlap{rng}", round(vol, 4), "mm3",
                                  {"clashes": [[round(h[0], 4), h[1], h[2], round(h[3], 4)] for h in hits[:50]],
                                   "n": len(hits)}))
    else:
        checks.append(CheckResult("NO_INTERFERENCE", True, f"no overlap between any two parts at any of "
                                  f"{len(values)} poses (limit {tol['overlap_volume_max']:g} mm3)", 0.0, "mm3"))

    rep = Report(checks)
    if keep_solutions:  # every part's plane pose at each table row, for a family's pose_answer
        rep.keep = [by_v[round(v, 9)] for v in table_in]
    rep.info = {"step_deg": step, "poses": len(values), "exact_tests": scene.n_exact,
                "max_move_mm": round(max_move(scene, sols), 4), "seconds": round(time.perf_counter() - t0, 3)}
    return rep


def max_move(scene: Scene, sols) -> float:
    """Largest distance any point of any moving part travels between two
    consecutive poses (measured at its outline's corners, in a straight line)."""
    import numpy as np

    from .interfere import _move
    worst = 0.0
    for pid, h in scene.hull.items():
        if scene.kind[pid] != "link":
            continue
        prev = None
        for s in sols:
            cur = _move(h, s.poses[pid])
            if prev is not None:
                worst = max(worst, float(np.max(np.hypot(*(cur - prev).T))))
            prev = cur
    return worst


def _skip_rest(checks, names, t0) -> Report:
    for n in names:
        if n == "GUIDED" and not _SLIDES.get("on"):
            continue
        checks.append(CheckResult(n, None, "skipped: an earlier check failed"))
    return Report(checks, {"seconds": round(time.perf_counter() - t0, 3)})


# ------------------------------------------------------------------ task folders


def load_task(task_dir: str) -> tuple[dict, Mechanism]:
    with open(os.path.join(task_dir, "motion.json")) as f:
        task = json.load(f)
    mech = Mechanism.from_json(task["mechanism"])
    return task, mech


def grade_task(task_dir: str, step_path: str | None = None, step_deg: float | None = None) -> Report:
    from ..checks import file_valid
    from ..checks.base import Context

    task, mech = load_task(task_dir)
    path = step_path or os.path.join(task_dir, "output.step")
    ctx = Context(task=None, step_path=path)
    fv = file_valid.run(ctx)
    if not fv.ok:
        return _skip_rest([fv], ["JOINTS", "ASSEMBLES", "TRANSMISSION", "OUTPUT", "NO_INTERFERENCE"],
                          time.perf_counter())
    rep = grade_solid(task, mech, ctx.solid, step_deg)
    rep.checks.insert(0, fv)
    return rep
