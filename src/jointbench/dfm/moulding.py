"""Injection moulding in a simple two-part mould: the rules.

``d`` is the pull direction, a line: one mould half pulls along +d, the
other along -d. Wall, rib and corner rules do not depend on it; draft and
undercut do.
"""
from __future__ import annotations

import math

import numpy as np

from .geom import Part, pt, r, unit
from .fdm import _on_faces, _surf
from .rules import Process, RuleResult, result

SIDE = 1e-9  # |n.d| at or below this: the face is parallel to the pull


def _where(part: Part, i: int) -> dict:
    return {"point": pt(part.P[i]), "face": int(part.F[i])}


def _face_median_thickness(part: Part, fi: int) -> float:
    t, w = part.thickness(), part.W
    sel = (part.F == fi) & np.isfinite(t)
    if not sel.any():
        return math.inf
    tt, ww = t[sel], w[sel]
    o = np.argsort(tt, kind="stable")
    c = np.cumsum(ww[o])
    return float(tt[o][np.searchsorted(c, c[-1] / 2)])


def ribs(part: Part, turn_min: float) -> list[dict]:
    """Ribs found from the solid's own faces: two flat faces, nearly opposite
    (within 10 deg, so drafted ribs count), facing away from each other, both
    joined by an inner corner -- sharp, or through a fillet -- to the same
    flat base face standing across them. Thickness is measured at the base,
    exactly, between the lines where the two side planes meet the base plane."""
    nominal = part.nominal_wall()
    planes = {i: part.plane(i) for i in range(len(part.faces))}
    planes = {i: p for i, p in planes.items() if p is not None}
    rounds_of = {}
    for x in part.rounds():
        if x.concave and not x.full:
            for f in x.faces:
                rounds_of[f] = x

    def bases(i):
        out = set()
        for e in part.neighbours(i):
            j = e.faces[1] if e.faces[0] == i else e.faces[0]
            if j in planes and e.concave and e.turn >= turn_min:
                out.add(j)
            elif j in rounds_of and e.turn < 1.0:
                for e2 in part.neighbours(j):
                    k = e2.faces[1] if e2.faces[0] == j else e2.faces[0]
                    if k != i and k in planes and e2.turn < 1.0:
                        out.add(k)
        return {b for b in out if abs(float(planes[b][0] @ planes[i][0])) < 0.3}

    found = []
    keys = sorted(planes)
    base_sets = {i: bases(i) for i in keys}
    cos10 = math.cos(math.radians(10))
    for a_i, i in enumerate(keys):
        ni, pi = planes[i]
        for j in keys[a_i + 1:]:
            nj, pj = planes[j]
            if float(ni @ nj) > -cos10:
                continue
            if float((pj - pi) @ ni) >= 0:  # must face away from each other
                continue
            if abs(float((pj - pi) @ ni)) > 2.0 * nominal:
                continue
            common = sorted(base_sets[i] & base_sets[j])
            for b in common:
                nb, pb = planes[b]

                def on_base(n, p):
                    # a point on the line where plane (n, p) meets the base plane
                    A = np.array([n, nb])
                    rhs = np.array([n @ p, nb @ pb])
                    x, *_ = np.linalg.lstsq(A, rhs, rcond=None)
                    return x

                u = unit(ni - (ni @ nb) * nb)
                w = abs(float((on_base(nj, pj) - on_base(ni, pi)) @ u))
                t = _face_median_thickness(part, b)
                found.append({"sides": [i, j], "base": b, "thickness": w, "wall": t,
                              "ratio": w / t if t > 0 else math.inf,
                              "where": (on_base(ni, pi) + on_base(nj, pj)) / 2})
    return found


def independent(part: Part, proc: Process) -> list[RuleResult]:
    R, S = proc.rules, proc.settings
    out = []
    sp = part.spacing
    t = part.thickness()
    nominal = part.nominal_wall()

    rule = R["IM_WALL_MIN"]
    i = int(np.argmin(t))
    v = float(t[i])
    out.append(result(rule, r(v), f"thinnest wall {v:.2f} mm ({rule.limit_text()})",
                      f"wall {v:.2f} mm thin at {pt(part.P[i])} ({rule.limit_text()})",
                      [_where(part, i)], "sampled", {"spacing_mm": sp}))
    _surf(out[-1], "wall_thickness", t, t < rule.value - 1e-9,
          "straight-through wall thickness at each triangle")

    if proc.wants("IM_WALL_MAX", "IM_WALL_UNIFORM"):
        v, k = part.ball_max()  # exact, measuring only where a larger ball is still possible
        rule = R["IM_WALL_MAX"]
        out.append(result(rule, r(v), f"thickest section {v:.2f} mm ({rule.limit_text()})",
                          f"section {v:.2f} mm thick at {pt(part.P[k])} ({rule.limit_text()})",
                          [_where(part, k)], "sampled", {"spacing_mm": sp}))
        lim = rule.value
        _surf(out[-1], "section_thickness", part.ball, lambda: part.ball() > lim + 1e-9,
              "diameter of the largest ball that fits in the material, touching here")

        rule = R["IM_WALL_UNIFORM"]
        ratio = v / nominal if nominal > 0 else math.inf
        out.append(result(rule, r(ratio), f"thickest {v:.2f} mm is {ratio:.2f} x the {nominal:.2f} mm "
                          f"nominal wall ({rule.limit_text()})",
                          f"section {v:.2f} mm is {ratio:.2f} x the {nominal:.2f} mm nominal wall "
                          f"({rule.limit_text()})",
                          [_where(part, k)], "sampled", {"nominal_wall_mm": r(nominal)}))
        lim_u = rule.value
        _surf(out[-1], "section_ratio", lambda: part.ball() / nominal, lambda: part.ball() / nominal > lim_u + 1e-9,
              f"section thickness over the {nominal:.2f} mm nominal wall")

    rule = R["IM_RIB_RATIO"]
    rb = ribs(part, S["corner_turn"])
    if rb:
        worst = max(rb, key=lambda x: (round(x["ratio"], 9), -x["sides"][0]))
        v = worst["ratio"]
        out.append(result(rule, r(v), f"{len(rb)} rib(s), thickest {worst['thickness']:.2f} mm = "
                          f"{v:.2f} x its {worst['wall']:.2f} mm wall ({rule.limit_text()})",
                          f"rib {worst['thickness']:.2f} mm thick on a {worst['wall']:.2f} mm wall "
                          f"= {v:.2f} x ({rule.limit_text()})",
                          [{"point": pt(worst["where"]), "face": worst["sides"][0]}], "exact",
                          {"ribs": [{"sides": x["sides"], "base": x["base"],
                                     "thickness_mm": r(x["thickness"]), "wall_mm": r(x["wall"])}
                                    for x in rb]}))
    else:
        out.append(result(rule, None, "no ribs", ""))
    rv = np.full(len(part.P), np.nan)
    for x in rb:
        sel = np.isin(part.F, x["sides"])
        rv[sel] = np.fmax(np.nan_to_num(rv[sel], nan=-1.0), x["ratio"])
    _surf(out[-1], "rib_ratio", rv, np.isfinite(rv) & (rv > rule.value + 1e-9),
          "rib thickness at its base over the wall it stands on, shown on the rib's sides")

    rule = R["IM_SHARP_CORNER"]
    sharp = [e for e in part.edges() if e.concave and e.turn >= S["corner_turn"]]
    fillets = [x for x in part.rounds() if x.concave and not x.full]
    if sharp:
        e = sharp[0]
        out.append(result(rule, 0.0, "", f"{len(sharp)} sharp inner corner(s), first at {pt(e.mid)} "
                          f"(turn {e.turn:.0f} deg, radius 0)",
                          [{"point": pt(x.mid), "face": x.faces[0]} for x in sharp[:20]], "exact",
                          {"sharp_edges": len(sharp)}))
    elif fillets:
        m = min(fillets, key=lambda x: (x.radius, tuple(x.where)))
        v = m.radius / nominal
        out.append(result(rule, r(v), f"smallest inner radius {m.radius:.2f} mm = {v:.2f} x wall "
                          f"({rule.limit_text()})",
                          f"inner radius {m.radius:.2f} mm = {v:.2f} x the {nominal:.2f} mm wall "
                          f"({rule.limit_text()})",
                          [{"point": pt(m.where), "face": m.faces[0]}]))
    else:
        out.append(result(rule, None, "no inner corners", ""))
    fv = np.full(len(part.P), np.nan)
    for x in fillets:
        fv[np.isin(part.F, x.faces)] = x.radius / nominal
    marks = [{"type": "polyline", "points": [pt(q) for q in part.edge_polyline(e.index)],
              "label": "sharp", "fail": True} for e in sharp]
    _surf(out[-1], "inner_radius_ratio", fv, np.isfinite(fv) & (fv < rule.value - 1e-9),
          "inner fillet radius over the nominal wall; sharp inner corners are drawn as marked edges",
          marks=marks)
    return out


def dependent(part: Part, proc: Process, d) -> list[RuleResult]:
    R, S = proc.rules, proc.settings
    d = unit(d)
    out = []
    c = part.N @ d

    rule = R["IM_DRAFT"]
    band = math.sin(math.radians(S["draft_face_band"]))
    along = np.abs(c) < band
    if along.any():
        dr = np.degrees(np.arcsin(np.clip(np.abs(c), 0, 1)))
        k = int(np.flatnonzero(along)[np.argmin(dr[along])])
        v = float(dr[k])
        bad = along & (dr < rule.value - 1e-9)
        out.append(result(rule, r(v, 3), f"least draft {v:.2f} deg ({rule.limit_text()})",
                          f"draft {v:.2f} deg on face {int(part.F[k])} ({rule.limit_text()}), "
                          f"{float(part.W[bad].sum()):.0f} mm2 short of draft",
                          [_where(part, k)], "sampled",
                          {"area_short_mm2": r(float(part.W[bad].sum()), 2)}))
    else:
        out.append(result(rule, None, "no faces run along the pull", "", [], "sampled"))
    drv = np.where(along, np.degrees(np.arcsin(np.clip(np.abs(c), 0, 1))), np.nan)
    _surf(out[-1], "draft_angle", drv, along & (drv < rule.value - 1e-9),
          "angle to the pull line of faces running along it; blank: faces across the pull")

    if proc.wants("IM_UNDERCUT"):
        rule = R["IM_UNDERCUT"]
        hidden = []
        for i, (p, n, ci) in enumerate(zip(part.P, part.N, c)):
            o = p + 1e-3 * n
            if ci > SIDE:
                free = not math.isfinite(part.ray(o, d))
            elif ci < -SIDE:
                free = not math.isfinite(part.ray(o, -d))
            else:
                free = (not math.isfinite(part.ray(o, d))) or (not math.isfinite(part.ray(o, -d)))
            if not free:
                hidden.append(i)
        area = float(part.W[hidden].sum()) if hidden else 0.0
        out.append(result(rule, r(area, 2), "every face can be seen by one mould half along the pull",
                          f"{area:.1f} mm2 hidden from both mould halves (undercut), first at "
                          f"{pt(part.P[hidden[0]]) if hidden else ''}",
                          [_where(part, i) for i in hidden[:20]], "sampled",
                          {"hidden_samples": len(hidden)}, ok=not hidden))
        hv = np.zeros(len(part.P))
        hv[hidden] = 1.0
        _surf(out[-1], "undercut", hv, hv > 0, "1 = hidden from both mould halves along the pull", "flag")
    return out
