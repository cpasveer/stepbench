"""Powder bed fusion (SLS / MJF): the rules.

The part is built inside a bed of loose powder that holds up every
overhang and bridge, so no rule depends on the build direction: there is
no overhang rule, no bridge rule and no bed-contact rule. What matters
instead is the powder itself (it must be able to leave every hollow) and
heat (thick sections and abrupt changes of thickness distort as they cool,
and so do large thin flat areas).
"""
from __future__ import annotations

import math

import build123d as bd
import numpy as np
from OCP.BRepClass3d import BRepClass3d
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.TopoDS import TopoDS

from .fdm import _on_faces, _surf
from .geom import Part, pt, r
from .rules import Process, RuleResult, result


def _where(part: Part, i: int) -> dict:
    return {"point": pt(part.P[i]), "face": int(part.F[i])}


def _face_median(part: Part, vals: np.ndarray, fi: int) -> float:
    sel = (part.F == fi) & np.isfinite(vals)
    if not sel.any():
        return math.nan
    v, w = vals[sel], part.W[sel]
    o = np.argsort(v, kind="stable")
    c = np.cumsum(w[o])
    return float(v[o][np.searchsorted(c, c[-1] / 2)])


def _face_high(part: Part, vals: np.ndarray, fi: int) -> float:
    """The section a face bounds: the largest ball diameter over its samples
    (the ball shrinks near the face's own edges, so a median or mean would
    understate the section well inside it)."""
    sel = (part.F == fi) & np.isfinite(vals)
    if not sel.any():
        return math.nan
    return float(vals[sel].max())


def _inner_shells(solid) -> list:
    outer = BRepClass3d.OuterShell_s(TopoDS.Solid(solid.wrapped))
    return [sh for sh in solid.shells() if not sh.wrapped.IsSame(outer)]


def hollows_behind_holes(part: Part, min_d: float) -> list[dict]:
    """Hollows that open to the outside only through round holes. Every
    round hole is plugged with a cylinder of its own radius and length; a
    hollow that is sealed in the plugged part but not in the real one is
    reached only through those holes. For each, the holes that open into it
    and how many are at least ``min_d`` across. Exact (solid booleans)."""
    holes = [x for x in part.rounds() if x.full and x.concave]
    if not holes:
        return []
    plugs = []
    for x in holes:
        s = []
        for f in x.faces:
            for v in bd.Face(part.faces[f]).vertices():
                s.append(float((np.array([v.X, v.Y, v.Z]) - x.axis_point) @ x.axis_dir))
        s0, s1 = min(s), max(s)
        if s1 - s0 < 1e-6:
            continue
        base = x.axis_point + s0 * x.axis_dir
        pl = bd.Plane(origin=tuple(base), z_dir=tuple(x.axis_dir))
        plugs.append((x, pl * bd.Cylinder(x.radius, s1 - s0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))))
    if not plugs:
        return []
    filled = part.solid
    skipped = 0
    try:
        filled = part.solid.fuse(*[c for _, c in plugs])
    except Exception:  # noqa: BLE001 -- the kernel refused the joint fuse: plug one by one
        for _, c in plugs:
            try:
                filled = filled + c
            except Exception:  # noqa: BLE001
                skipped += 1
    hollows_behind_holes.skipped = skipped
    sols = filled.solids() if hasattr(filled, "solids") else [filled]
    before = len(_inner_shells(part.solid))
    out = []
    for sol in sols:
        inner = _inner_shells(sol)
        if len(inner) <= before and len(sols) == 1:
            break
        for sh in inner:
            touching = []
            for x, c in plugs:
                d = BRepExtrema_DistShapeShape(sh.wrapped, c.wrapped)
                if d.IsDone() and d.Value() < 1e-3:
                    touching.append(x)
            if not touching:
                continue  # a hollow that was sealed already
            bb = sh.bounding_box()
            centre = np.array([(bb.min.X + bb.max.X) / 2, (bb.min.Y + bb.max.Y) / 2, (bb.min.Z + bb.max.Z) / 2])
            out.append({"centre": centre, "holes": touching,
                        "escape": sum(2 * x.radius >= min_d - 1e-9 for x in touching)})
    return out


def independent(part: Part, proc: Process) -> list[RuleResult]:
    R, S = proc.rules, proc.settings
    out = []
    sp = part.spacing

    rule = R["PBF_WALL_MIN"]
    t = part.thickness()
    i = int(np.argmin(t))
    v = float(t[i])
    bad = int((t < rule.value - 1e-9).sum())
    out.append(result(rule, r(v), f"thinnest wall {v:.2f} mm ({rule.limit_text()})",
                      f"wall {v:.2f} mm thin at {pt(part.P[i])} ({rule.limit_text()}; {bad} of {len(t)} samples under)",
                      [_where(part, i)], "sampled", {"spacing_mm": sp, "samples": len(t)}))
    _surf(out[-1], "wall_thickness", t, t < rule.value - 1e-9, "straight-through wall thickness at each triangle")

    rule = R["PBF_SEALED_HOLLOW"]
    vf = part.void_faces()
    vs = [p for p, _ in vf]
    out.append(result(rule, len(vs), "no sealed hollows",
                      f"{len(vs)} sealed hollow(s): the powder inside cannot get out",
                      [{"point": pt(p), "face": None} for p in vs]))
    faces = [f for _, fs in vf for f in fs]
    _surf(out[-1], "sealed_hollow", _on_faces(part, faces), np.isin(part.F, faces),
          "the walls of sealed hollows; the picture cuts the part open", "flag", cut=pt(vs[0]) if vs else None)

    if proc.wants("PBF_ESCAPE_HOLES"):
        rule = R["PBF_ESCAPE_HOLES"]
        min_d = float(S["escape_hole_diameter"])
        hollows_behind_holes.skipped = 0
        hh = hollows_behind_holes(part, min_d)
        skipped = hollows_behind_holes.skipped
        vals = np.full(len(part.P), np.nan)
        failf = np.zeros(len(part.P), bool)
        marks = []
        for h in hh:
            for x in h["holes"]:
                sel = np.isin(part.F, x.faces)
                vals[sel] = h["escape"]
                failf |= sel & (h["escape"] < rule.value)
                marks.append({"type": "point", "point": pt(x.where), "label": f"{2 * x.radius:.1f} mm",
                              "fail": bool(h["escape"] < rule.value)})
        if not hh:
            note = f" ({skipped} hole(s) could not be plugged and were not looked through)" if skipped else ""
            out.append(result(rule, None, "no hollow is reached only through holes" + note, "",
                              details={"holes_not_plugged": skipped}))
        else:
            worst = min(hh, key=lambda h: (h["escape"], tuple(np.round(h["centre"], 6))))
            n = worst["escape"]
            out.append(result(rule, n,
                              f"{len(hh)} hollow(s) behind holes, each with at least {n} escape hole(s) of {min_d:g} mm or more",
                              f"a hollow at {pt(worst['centre'])} has {n} escape hole(s) of {min_d:g} mm or more "
                              f"(of {len(worst['holes'])} holes into it; {rule.limit_text()})",
                              [{"point": pt(worst["centre"]), "face": None}],
                              details={"hollows": len(hh), "escape_hole_diameter_mm": min_d}))
        _surf(out[-1], "escape_hole", vals, failf,
              f"on each hole into a hollow: how many holes of {min_d:g} mm or more that hollow has; labels give each hole's diameter",
              marks=marks)

    for rid, concave, word in (("PBF_HOLE_MIN", True, "hole"), ("PBF_PIN_MIN", False, "pin")):
        rule = R[rid]
        rs = [x for x in part.rounds() if x.full and x.concave == concave]
        vals = np.full(len(part.P), np.nan)
        for x in rs:
            vals[np.isin(part.F, x.faces)] = 2 * x.radius
        if not rs:
            out.append(result(rule, None, f"no round {word}s", ""))
        else:
            m = min(rs, key=lambda x: (x.radius, tuple(x.where)))
            v = 2 * m.radius
            out.append(result(rule, r(v), f"{len(rs)} {word}(s), smallest {v:.2f} mm ({rule.limit_text()})",
                              f"{word} {v:.2f} mm across ({rule.limit_text()})",
                              [{"point": pt(m.where), "face": m.faces[0]}]))
        _surf(out[-1], f"{word}_diameter", vals, np.isfinite(vals) & (vals < rule.value - 1e-9),
              f"diameter of each round {word}; blank elsewhere")

    rule = R["PBF_GAP_MIN"]
    g = part.gap()
    # a narrow gap that only lies in an open inside corner (faces meeting at gap_open_corner or more)
    # is not judged: every V narrows to nothing at its bottom (maintainer, 13 November 2026)
    from .locate import open_corner_gaps

    corner = open_corner_gaps(part, g, rule.value, S["gap_open_corner"])
    g = np.where(corner, np.inf, g)
    fin = np.isfinite(g)
    if fin.any():
        j = int(np.flatnonzero(fin)[np.argmin(g[fin])])
        v = float(g[j])
        out.append(result(rule, r(v), f"narrowest gap {v:.2f} mm ({rule.limit_text()})",
                          f"gap {v:.2f} mm narrow at {pt(part.P[j])} ({rule.limit_text()})",
                          [_where(part, j)], "sampled", {"spacing_mm": sp}))
    else:
        out.append(result(rule, None, "no face looks at another face", "", [], "sampled"))
    _surf(out[-1], "gap", g, fin & (g < rule.value - 1e-9),
          "clear distance straight out of the surface to the part; blank where nothing is in front")

    if proc.wants("PBF_SECTION_MAX", "PBF_TRANSITION"):
        rule = R["PBF_SECTION_MAX"]
        v, k = part.ball_max()  # exact, measuring only where a larger ball is still possible
        out.append(result(rule, r(v), f"thickest section {v:.2f} mm ({rule.limit_text()})",
                          f"section {v:.2f} mm thick at {pt(part.P[k])} ({rule.limit_text()})",
                          [_where(part, k)], "sampled", {"spacing_mm": sp}))
        lim = rule.value
        _surf(out[-1], "section_thickness", part.ball, lambda: part.ball() > lim + 1e-9,
              "diameter of the largest ball that fits in the material at each triangle")

        # abrupt transitions: across each inside corner, the sections either side
        rule = R["PBF_TRANSITION"]
        med = {}
        worst = (0.0, None)
        ratio_f = np.full(len(part.P), np.nan)
        for e in part.edges():
            if not e.concave or e.turn < S["corner_turn"]:
                continue
            a, c = e.faces
            for f in (a, c):
                if f not in med:
                    med[f] = part.ball_face_max(f)
            ta, tc = med[a], med[c]
            if not (math.isfinite(ta) and math.isfinite(tc)) or min(ta, tc) <= 0:
                continue
            q = max(ta, tc) / min(ta, tc)
            thin = a if ta < tc else c
            sel = part.F == thin
            ratio_f[sel] = np.fmax(ratio_f[sel], q)
            if q > worst[0] + 1e-12:
                worst = (q, e)
        if worst[1] is None:
            out.append(result(rule, None, "no inside corners between walls", "", [], "sampled"))
        else:
            q, e = worst
            out.append(result(rule, r(q), f"largest change of section at an inside corner x{q:.2f} ({rule.limit_text()})",
                              f"section changes x{q:.2f} at the inside corner at {pt(e.mid)} ({rule.limit_text()})",
                              [{"point": pt(e.mid), "face": int(e.faces[0])}], "sampled", {"spacing_mm": sp}))
        _surf(out[-1], "transition_ratio", ratio_f, np.isfinite(ratio_f) & (ratio_f > rule.value + 1e-9),
              "on the thinner side of each inside corner: how many times thicker the section across the corner is "
              "(section of a face: the largest ball diameter over it)")

    # warping: each flat face's length over the section under it
    rule = R["PBF_WARP_FLAT"]
    wv = np.full(len(part.P), np.nan)
    worst = (0.0, None, 0.0, 0.0)
    for fi in range(len(part.faces)):
        if part.plane(fi) is None:
            continue
        sel = part.F == fi
        if sel.sum() < 3:
            continue
        P = part.P[sel]
        c = P - P.mean(axis=0)
        _, _, vt = np.linalg.svd(c, full_matrices=False)
        length = float(np.ptp(c @ vt[0]))
        th = _face_median(part, t, fi)
        if not math.isfinite(th) or th <= 0:
            continue
        q = length / th
        wv[sel] = q
        if q > worst[0] + 1e-12:
            worst = (q, fi, length, th)
    if worst[1] is None:
        out.append(result(rule, None, "no flat faces", "", [], "sampled"))
    else:
        q, fi, length, th = worst
        sel = np.flatnonzero(part.F == fi)
        out.append(result(rule, r(q), f"flattest-thinnest face {length:.0f} mm long on a {th:.1f} mm section, x{q:.1f} "
                                      f"({rule.limit_text()})",
                          f"flat face {length:.0f} mm long on a {th:.1f} mm section, x{q:.1f} ({rule.limit_text()})",
                          [_where(part, int(sel[0]))], "sampled", {"spacing_mm": sp}))
    _surf(out[-1], "warp_ratio", wv, np.isfinite(wv) & (wv > rule.value + 1e-9),
          "on each flat face: its length over the median straight-through thickness under it")
    return out


def dependent(part: Part, proc: Process, d) -> list[RuleResult]:
    return []  # nothing depends on the build direction: the powder supports the part
