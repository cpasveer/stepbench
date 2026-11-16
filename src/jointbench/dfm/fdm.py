"""Filament printing (FDM): the rules.

``d`` is the build direction: a unit vector pointing from the bed up into
the part. The bed is the plane through the part's lowest point,
perpendicular to ``d``.

Rules that do not depend on the build direction (wall, gap, holes, pins,
hollows) are measured once; the rest (overhang, bridge, bed contact, stance)
once per direction tried.
"""
from __future__ import annotations

import math

import numpy as np
from OCP.BRepAdaptor import BRepAdaptor_Curve
from OCP.GCPnts import GCPnts_QuasiUniformDeflection
from OCP.TopAbs import TopAbs_EDGE
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS

from .geom import Part, pt, r, unit, vertical_extent
from .rules import Field, Process, RuleResult, result

BED_TOL = 0.01  # mm: a flat face this close to the lowest point lies on the bed


def _where(part: Part, i: int) -> dict:
    return {"point": pt(part.P[i]), "face": int(part.F[i])}


def _on_faces(part: Part, faces, value=1.0) -> np.ndarray:
    v = np.full(len(part.P), np.nan)
    v[np.isin(part.F, list(faces))] = value
    return v


def _surf(res: RuleResult, fid: str, values, fail, note="", type="scalar", marks=(), cut=None):
    """Attach a field to a rule result. ``values`` and ``fail`` may be
    functions: then they are computed only when a picture or field file
    needs them (Field.ready), not for the verdict."""
    if not callable(values):
        values = np.where(np.isfinite(values), values, np.nan)
        fail = np.asarray(fail, dtype=bool)
    res.surface = Field(fid, res.id, res.title, res.unit, res.kind, res.limit, values,
                        fail, type, note, list(marks), cut)
    return res


def _corner_walls(part: Part, t: np.ndarray, limit: float, open_corner: float) -> np.ndarray:
    """Samples under the wall limit whose ray does not measure a wall.

    A wall is material between two roughly opposite surfaces. The ray goes
    from the sample straight into the material and stops where it leaves.
    If the surface it leaves through is turned less than 180 - open_corner
    deg from the sample's own (the two make a wedge of material of at least
    ``open_corner`` deg), the ray has only cut across a corner: at a convex
    edge of inside angle a, a sample x mm from the edge reads x * tan(a),
    which goes to nothing at the edge for every corner under 90 deg, however
    sturdy. Like the gap rule's open corners (gap_open_corner): the region
    of such a wedge narrower than the limit lies within limit / (2 tan(a/2))
    of the edge, about 1 mm at 45 deg. Sharper wedges still count as walls,
    and knife edges are Q_THIN_WEDGE's. Exact: one ray
    and the true surface normal where it leaves, per thin sample."""
    from .locate import hit_face

    out = np.zeros(len(t), dtype=bool)
    cos_max = math.cos(math.radians(180.0 - open_corner))
    for k in np.flatnonzero(np.isfinite(t) & (t < limit - 1e-9)):
        d = -part.N[k]
        fj = hit_face(part, part.P[k], d)
        if fj is None:
            continue  # the kernel cannot say: the reading stands
        n_exit = part.normal_at(fj, part.P[k] + t[k] * d)
        if float(part.N[k] @ n_exit) > cos_max + 1e-12:
            out[k] = True
    return out


# ------------------------------------------------- direction-free rules


def independent(part: Part, proc: Process) -> list[RuleResult]:
    R, S = proc.rules, proc.settings
    out = []
    sp = part.spacing

    lim = R["FDM_WALL_MIN"].value
    t = part.thickness()
    # a thin reading whose ray leaves through a surface that is not roughly
    # opposite its own is the distance to a corner, not a wall (see _corner_walls)
    corner = _corner_walls(part, t, lim, S["wall_open_corner"])
    tw = np.where(corner, np.inf, t)
    i = int(np.argmin(tw))
    v = float(tw[i])
    bad = int((tw < lim - 1e-9).sum())
    out.append(result(R["FDM_WALL_MIN"], r(v),
                      f"thinnest wall {v:.2f} mm ({R['FDM_WALL_MIN'].limit_text()})",
                      f"wall {v:.2f} mm thin at {pt(part.P[i])} ({R['FDM_WALL_MIN'].limit_text()}; "
                      f"{bad} of {len(t)} samples under)",
                      [_where(part, i)], "sampled", {"spacing_mm": sp, "samples": len(t),
                                                     "corner_samples_not_walls": int(corner.sum())}))
    _surf(out[-1], "wall_thickness", np.where(corner, np.nan, t), tw < lim - 1e-9,
          "straight-through wall thickness at each triangle; blank where the ray only cuts a corner")

    rule = R["FDM_GAP_MIN"]
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

    for rid, concave, word in (("FDM_HOLE_MIN", True, "hole"), ("FDM_PIN_MIN", False, "pin")):
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

    rule = R["FDM_SEALED_HOLLOW"]
    vf = part.void_faces()
    vs = [p for p, _ in vf]
    out.append(result(rule, len(vs), "no sealed hollows",
                      f"{len(vs)} sealed hollow(s) inside the part",
                      [{"point": pt(p), "face": None} for p in vs]))
    faces = [f for _, fs in vf for f in fs]
    _surf(out[-1], "sealed_hollow", _on_faces(part, faces), np.isin(part.F, faces),
          "the walls of sealed hollows; the picture cuts the part open", "flag",
          cut=pt(vs[0]) if vs else None)
    return out


# --------------------------------------------- direction-dependent rules


def _hull_min_width(p2: np.ndarray) -> float:
    """Narrowest width of the convex outline around 2D points (the smallest
    distance between two parallel lines that hold them all)."""
    pts = sorted(set(map(tuple, np.round(p2, 6))))
    if len(pts) < 3:
        return 0.0

    def half(ps):
        h = []
        for q in ps:
            while len(h) >= 2 and ((h[-1][0] - h[-2][0]) * (q[1] - h[-2][1])
                                   - (h[-1][1] - h[-2][1]) * (q[0] - h[-2][0])) <= 0:
                h.pop()
            h.append(q)
        return h

    hull = np.array(half(pts)[:-1] + half(pts[::-1])[:-1], dtype=float)
    if len(hull) < 3:
        return 0.0
    best = math.inf
    for i in range(len(hull)):
        e = hull[(i + 1) % len(hull)] - hull[i]
        n = np.array([-e[1], e[0]]) / np.linalg.norm(e)
        w = (hull - hull[i]) @ n
        best = min(best, float(w.max() - w.min()))
    return best


def _stance(part: Part, rule, d, bed_faces, zmin) -> RuleResult:
    """Height on the bed over the stance width: the narrowest width of the
    outline around every face that lies on the bed. A tall part on a narrow
    foot is knocked loose by the nozzle dragging at its top."""
    _, zmax = vertical_extent(part, d)
    height = zmax - zmin
    e1 = unit(np.cross(d, [1.0, 0, 0] if abs(d[0]) < 0.9 else [0, 1.0, 0]))
    e2 = np.cross(d, e1)
    o = d * zmin
    segs = [_boundary_segments(part.faces[i], o, e1, e2).reshape(-1, 2) for i in bed_faces]
    width = _hull_min_width(np.vstack(segs)) if segs else 0.0
    on = np.isin(part.F, bed_faces)
    if width <= 1e-6:
        res = result(rule, None, "", f"nothing flat on the bed to stand on ({rule.limit_text()})",
                     [], "exact", {"height_mm": r(height, 2), "stance_width_mm": 0.0}, ok=False)
    else:
        v = height / width
        big = max(bed_faces, key=lambda i: (part.area(i), -i))
        sel = np.flatnonzero(part.F == big)
        where = [{"point": pt(part.P[sel[len(sel) // 2]]) if sel.size else None, "face": big}]
        res = result(rule, r(v, 2),
                     f"{height:.1f} mm tall on a {width:.1f} mm stance, {v:.2f} x ({rule.limit_text()})",
                     f"{height:.1f} mm tall on a {width:.1f} mm stance, {v:.2f} x ({rule.limit_text()})",
                     where, "exact", {"height_mm": r(height, 2), "stance_width_mm": r(width, 2)})
    return _surf(res, "stance", _on_faces(part, bed_faces), on & (not res.ok),
                 "the faces the part stands on; the stance is the narrowest width of the outline "
                 "around them", "flag")


def _boundary_segments(face, origin, e1, e2):
    """The face boundary as 2D line segments in the plane (arcs split finely,
    0.005 mm chord error)."""
    segs = []
    ex = TopExp_Explorer(face, TopAbs_EDGE)
    while ex.More():
        c = BRepAdaptor_Curve(TopoDS.Edge(ex.Current()))
        q = GCPnts_QuasiUniformDeflection(c, 0.005)
        pts = []
        if q.IsDone() and q.NbPoints() >= 2:
            for k in range(1, q.NbPoints() + 1):
                p = q.Value(k)
                w = np.array([p.X(), p.Y(), p.Z()]) - origin
                pts.append((float(w @ e1), float(w @ e2)))
        for a, b in zip(pts[:-1], pts[1:]):
            segs.append((a, b))
        ex.Next()
    return np.array(segs, dtype=float).reshape(-1, 2, 2)


def _exits(segs, p2, u2):
    """Distances from p2 along +u2 and -u2 to the face boundary."""
    a, b = segs[:, 0, :], segs[:, 1, :]
    s = b - a
    den = u2[0] * s[:, 1] - u2[1] * s[:, 0]
    ok = np.abs(den) > 1e-12
    w = a - p2
    t = np.where(ok, (w[:, 0] * s[:, 1] - w[:, 1] * s[:, 0]) / np.where(ok, den, 1), np.nan)
    lam = np.where(ok, (w[:, 0] * u2[1] - w[:, 1] * u2[0]) / np.where(ok, den, 1), np.nan)
    hit = ok & (lam >= -1e-9) & (lam <= 1 + 1e-9)
    fw = t[hit & (t > 1e-9)]
    bw = -t[hit & (t < -1e-9)]
    return (float(fw.min()) if fw.size else math.inf), (float(bw.min()) if bw.size else math.inf)


def _exits_all(segs, p2, U2):
    """``_exits`` for several directions at once: the same arithmetic,
    element by element, with one array per quantity instead of one call per
    direction."""
    a, b = segs[:, 0, :], segs[:, 1, :]
    s = b - a
    w = a - p2
    ux = np.array([u[0] for u in U2])[:, None]
    uy = np.array([u[1] for u in U2])[:, None]
    den = ux * s[:, 1] - uy * s[:, 0]
    ok = np.abs(den) > 1e-12
    dd = np.where(ok, den, 1)
    t = np.where(ok, (w[:, 0] * s[:, 1] - w[:, 1] * s[:, 0]) / dd, np.nan)
    lam = np.where(ok, (w[:, 0] * uy - w[:, 1] * ux) / dd, np.nan)
    hit = ok & (lam >= -1e-9) & (lam <= 1 + 1e-9)
    fw = np.where(hit & (t > 1e-9), t, np.inf).min(axis=1) if s.size else np.full(len(U2), np.inf)
    bw = np.where(hit & (t < -1e-9), -t, np.inf).min(axis=1) if s.size else np.full(len(U2), np.inf)
    return [(float(x), float(y)) for x, y in zip(fw, bw)]


def _ceiling_spans(part: Part, proc: Process, d, faces) -> dict[int, float | None]:
    """For each sample on a flat ceiling: (length, end, end) of the shortest
    straight span through it whose two ends land on material below (a
    bridge), or None when no direction tried has two supported ends (a
    cantilever)."""
    nd = int(proc.settings["bridge_directions"])
    out = {}
    a = np.array([1.0, 0, 0]) if abs(d[0]) < 0.9 else np.array([0, 1.0, 0])
    e1 = unit(np.cross(d, a))
    e2 = np.cross(d, e1)
    dirs = [math.pi * k / nd for k in range(nd)]
    probe = 0.05
    # first every candidate span and its two probe points, then all probes
    # classified in one go, then the shortest supported span per sample (in
    # the same order as one-by-one, so ties resolve the same way)
    cand = []  # (sample, span, end, end)
    probes = []
    for fi in faces:
        idx = np.flatnonzero(part.F == fi)
        if not idx.size:
            continue
        origin = part.P[idx[0]]
        segs = _boundary_segments(part.faces[fi], origin, e1, e2)
        U2 = [np.array([math.cos(ang), math.sin(ang)]) for ang in dirs]
        for i in idx:
            out[int(i)] = None
            w = part.P[i] - origin
            p2 = np.array([w @ e1, w @ e2])
            FB = _exits_all(segs, p2, U2)
            for k, u2 in enumerate(U2):
                f, b = FB[k]
                if not (math.isfinite(f) and math.isfinite(b)):
                    continue
                u3 = u2[0] * e1 + u2[1] * e2
                probes.append(part.P[i] + (f + probe) * u3 - probe * d)
                probes.append(part.P[i] - (b + probe) * u3 - probe * d)
                cand.append((int(i), f + b, part.P[i] + f * u3, part.P[i] - b * u3))
    ins = part.inside_many(np.array(probes)) if probes else np.zeros(0, bool)
    for k, (i, span, pa, pb) in enumerate(cand):
        if ins[2 * k] and ins[2 * k + 1]:
            best = out[i]
            if best is None or span < best[0]:
                out[i] = (span, pa, pb)
    return out


KEEL_MIN_TURN = 10.0  # deg: an edge where the surface turns less than this is a smooth crease, judged as surface


def _keels(part: Part, d, zmin: float, skip_faces: set, limit: float) -> list:
    """Downward-pointing edges ("keels"): a convex edge whose two faces both rise away from it, so
    the edge is the lowest line of the material there and each layer extends it in mid-air. Each face
    may lean within the limit while the edge between them runs much flatter; the edge line itself
    must then stay within the overhang limit. Exact, along each edge (straight pieces between points
    at most 1 mm apart). Edges on the bed, on flat ceilings (the bridge rule's), on small holes (exempt)
    are skipped. Returns (angle from vertical, edge index, worst point, length of the edge's pieces
    over the limit, or its whole length if none, (start, end) of its steepest piece, runs) per keel;
    runs: [(start, end, length)] of each unbroken stretch of pieces over the limit."""
    out = []
    for e in part.edges():
        if e.concave or e.turn < KEEL_MIN_TURN or set(e.faces) & skip_faces:
            continue
        pts = part.edge_polyline(e.index)
        best = None
        over_len = total = 0.0
        runs, last = [], None
        for j, (a, b) in enumerate(zip(pts[:-1], pts[1:])):
            seg = b - a
            L = float(np.linalg.norm(seg))
            if L < 1e-6:
                continue
            m = (a + b) / 2
            if float(m @ d) - zmin <= BED_TOL:
                continue
            t = seg / L
            n1, n2 = part.normal_at(e.faces[0], m), part.normal_at(e.faces[1], m)
            # the direction into each face, square to the edge, pointing away from the other face
            w1, w2 = unit(np.cross(n1, t)), unit(np.cross(n2, t))
            if float(w1 @ n2) > 0:
                w1 = -w1
            if float(w2 @ n1) > 0:
                w2 = -w2
            if float(w1 @ d) <= 1e-6 or float(w2 @ d) <= 1e-6 or float(n1 @ d) >= -1e-6 or float(n2 @ d) >= -1e-6:
                continue  # not the lowest line here: a face goes down from it, or one faces up
            angle = math.degrees(math.acos(min(1.0, abs(float(t @ d)))))
            total += L
            if angle > limit + 1e-9:
                over_len += L
                if last == j - 1:
                    runs[-1] = (runs[-1][0], b, runs[-1][2] + L)
                else:
                    runs.append((a, b, L))
                last = j
            if best is None or angle > best[0]:
                best = (angle, m, (a, b))
        if best:
            out.append((best[0], e.index, best[1], over_len or total, best[2], runs))
    return out


def _printed_below(part: Part, q, d, zmin: float, line: float, layer: float) -> bool:
    """Is there material one layer below q, within one line width of it (the line printed at q then
    rests on a line of the layer before)? The bed counts as material."""
    if float(q @ d) - zmin <= layer + BED_TOL:
        return True
    a = np.array([1.0, 0, 0]) if abs(d[0]) < 0.9 else np.array([0, 1.0, 0])
    e1 = unit(np.cross(d, a))
    e2 = np.cross(d, e1)
    base = q - layer * d
    probes = [base] + [base + rr * (math.cos(t) * e1 + math.sin(t) * e2)
                       for rr in (line / 2, line) for t in (math.pi * k / 4 for k in range(8))]
    return any(part.inside(p) for p in probes)


def _keel_run_verdict(part: Part, run, d, zmin: float, line: float, layer: float, bridge_max: float) -> str:
    """How a stretch of keel flatter than the edge limit prints (16 November 2026):
    'mid-air'   its lowest point has nothing printed under it: it starts in mid-air (fails);
    'cantilever' it rises less than one layer from end to end, so it is printed in one layer, and
                one end has nothing under it (fails);
    'bridge'    both ends rest on material printed below: judged as a bridge, by FDM_BRIDGE's length;
                returned as 'bridge' (within) or 'long bridge' (fails);
    'grows'     its lowest point rests on printed material and it rises from there, each layer
                building on the one before (passes)."""
    a, b, length = run
    lo, hi = (a, b) if float(a @ d) <= float(b @ d) else (b, a)
    s_lo = _printed_below(part, lo, d, zmin, line, layer)
    s_hi = _printed_below(part, hi, d, zmin, line, layer)
    level = float((hi - lo) @ d) < layer
    if s_lo and s_hi:
        return "bridge" if length <= bridge_max + 1e-9 else "long bridge"
    if level and (s_lo or s_hi):
        return "cantilever"
    if not s_lo:
        return "mid-air"
    return "grows"


KEEL_FAILS = ("mid-air", "cantilever", "long bridge")


def dependent(part: Part, proc: Process, d) -> list[RuleResult]:
    R = proc.rules
    S = proc.settings
    d = unit(d)
    flat = math.cos(math.radians(S["flat_tolerance"]))
    zmin, _ = vertical_extent(part, d)
    out = []

    # bed contact: flat faces facing straight down at the lowest level (exact)
    bed_faces = []
    for i in range(len(part.faces)):
        pl = part.plane(i)
        if pl is None:
            continue
        n, o = pl
        if float(n @ d) <= -flat and abs(float(o @ d) - zmin) <= BED_TOL:
            bed_faces.append(i)
    area = sum(part.area(i) for i in bed_faces)
    rule = R["FDM_BED_CONTACT"]
    if bed_faces:
        big = max(bed_faces, key=lambda i: (part.area(i), -i))
        sel = np.flatnonzero(part.F == big)
        where = [{"point": pt(part.P[sel[len(sel) // 2]]) if sel.size else None, "face": big}]
    else:
        lo = int(np.argmin(part.P @ d))
        where = [_where(part, lo)]
    out.append(result(rule, r(area, 2),
                      f"{area:.0f} mm2 flat on the bed over {len(bed_faces)} face(s) ({rule.limit_text()})",
                      (f"only {area:.1f} mm2 flat on the bed ({rule.limit_text()})" if bed_faces
                       else "no flat face at the lowest point to stand on"),
                      where))
    on = np.isin(part.F, bed_faces)
    _surf(out[-1], "bed_contact", _on_faces(part, bed_faces), on & (not out[-1].ok),
          "the faces that lie flat on the bed", "flag",
          marks=[] if bed_faces else [{"type": "point", "point": where[0]["point"],
                                       "label": "lowest point", "fail": True}])

    # stance: height on the bed against the narrowest width of the foot (exact)
    if proc.wants("FDM_STANCE") and "FDM_STANCE" in R:
        out.append(_stance(part, R["FDM_STANCE"], d, bed_faces, zmin))

    # overhang and bridges (sampled normals; a flat face has one exact normal)
    down = -(part.N @ d)  # > 0: the surface faces down
    ang = np.degrees(np.arcsin(np.clip(down, -1, 1)))
    on_bed = np.isin(part.F, bed_faces)
    hole_faces = set()
    for x in part.rounds():
        if x.full and x.concave and 2 * x.radius <= S["overhang_hole_exempt_diameter"] + 1e-9:
            hole_faces.update(x.faces)
    exempt = on_bed | np.isin(part.F, sorted(hole_faces))
    ceilings = [i for i in range(len(part.faces)) if i not in bed_faces
                and (pl := part.plane(i)) is not None and float(pl[0] @ d) <= -flat]
    spans = _ceiling_spans(part, proc, d, ceilings)
    bridged = np.zeros(len(part.P), dtype=bool)
    for i, sp_ in spans.items():
        if sp_ is not None:
            bridged[i] = True
    judged = ~exempt & ~bridged & (down > 1e-9)
    rule = R["FDM_OVERHANG"]
    edge_limit = math.degrees(math.atan(S['edge_line_width'] / S['edge_layer_height']))  # 63.4 deg at 0.4 / 0.2
    keels = _keels(part, d, zmin, set(ceilings) | hole_faces | set(bed_faces), edge_limit)
    worst_keel = max(keels, key=lambda x: (round(x[0], 6), -x[1])) if keels else None
    # a keel flatter than the limit fails only where it starts in mid-air (or is a cantilever, or a
    # bridge longer than FDM_BRIDGE allows); one that rests on printed material at its start grows
    # from it layer by layer (16 November 2026)
    runs = {x[1]: [(run, _keel_run_verdict(part, run, d, zmin, S["edge_line_width"], S["edge_layer_height"],
                                           R["FDM_BRIDGE"].value)) for run in x[5]] for x in keels}
    keel_over = [x for x in keels if any(v in KEEL_FAILS for _, v in runs[x[1]])]
    keel_len = sum(run[2] for x in keel_over for run, v in runs[x[1]] if v in KEEL_FAILS)
    kinds = [v for x in keels for _, v in runs[x[1]]]
    k = v = None
    if judged.any():
        k = int(np.flatnonzero(judged)[np.argmax(ang[judged])])
        v = float(ang[k])
    if v is not None or worst_keel:
        over = judged & (ang > rule.value + 1e-9)
        bad_area = float(part.W[over].sum())
        cant = sum(1 for i, sp_ in spans.items() if sp_ is None)
        details = {"area_over_limit_mm2": r(bad_area, 2), "cantilever_samples": cant,
                   "exempt_hole_faces": sorted(hole_faces),
                   "keel_edges": len(keels), "keel_length_over_limit_mm": r(keel_len, 2),
                   "steepest_keel_deg": r(worst_keel[0], 2) if worst_keel else None,
                   "flat_keel_runs": {k: kinds.count(k) for k in sorted(set(kinds))}}
        keel_bad = bool(keel_over)
        face_bad = v is not None and v > rule.value + 1e-9
        if (keel_bad and not face_bad) or v is None:  # the edge decides, or only edges face down
            shown = max(keel_over, key=lambda x: (round(x[0], 6), -x[1])) if keel_bad else worst_keel
            a, ei, p, length, _, _ = shown
            why = sorted({v for _, v in runs[ei] if v in KEEL_FAILS})
            said = {"mid-air": "it starts in mid-air", "cantilever": "it sticks out level from one support",
                    "long bridge": f"it spans more than the {R['FDM_BRIDGE'].value:g} mm bridge limit between supports"}
            grows = kinds.count("grows") + kinds.count("bridge")
            ok_text = (f"steepest downward edge {a:.1f} deg (edges: max {edge_limit:.1f} deg where an edge starts "
                       f"in mid-air)" + (f"; {grows} flatter stretch(es) rest on printed material" if grows else ""))
            out.append(result(rule, r(a, 2), ok_text,
                              f"downward edge {a:.1f} deg from vertical (edges: max {edge_limit:.1f} deg, one line width "
                              f"per layer), {length:.0f} mm long: "
                              f"two downward faces meet in a keel that each layer extends in mid-air, and "
                              f"{' and '.join(said[w] for w in why)}; "
                              f"{keel_len:.0f} mm of such edges and {bad_area:.0f} mm2 of surface need support",
                              [{"point": pt(p), "edge": ei}], "exact", details, ok=not keel_bad))
        else:
            extra = " (flat ceiling with no support at one end)" if (k in spans and spans[k] is None) else ""
            more = f", and {keel_len:.0f} mm of downward edges" if keel_len else ""
            out.append(result(rule, r(v, 2), f"steepest overhang {v:.1f} deg ({rule.limit_text()})",
                              f"overhang {v:.1f} deg{extra} ({rule.limit_text()}), "
                              f"{bad_area:.0f} mm2{more} needs support",
                              [_where(part, k)], "sampled", details))
    else:
        out.append(result(rule, 0.0, "no face overhangs", "", [], "sampled"))
    ov = np.where(judged, ang, np.nan)
    marks = [{"type": "segment", "points": [pt(a), pt(b)], "value": r(x[0], 2), "label": f"{x[0]:.0f} deg edge",
              "fail": True} for x in keel_over for a, b in [x[4]]]
    _surf(out[-1], "overhang_angle", ov, judged & (ang > rule.value + 1e-9),
          "angle from vertical of each downward-facing surface; 90 = a flat ceiling with no "
          "support at one end; blank: faces up, lies on the bed, a small round hole, or a bridge. "
          "Lines: downward edges (keels) steeper than the limit", marks=marks)

    rule = R["FDM_BRIDGE"]
    real = {i: sp_[0] for i, sp_ in spans.items() if sp_ is not None}
    if real:
        k = max(real, key=lambda i: (round(real[i], 6), -i))
        v = real[k]
        out.append(result(rule, r(v, 2), f"longest bridge {v:.1f} mm ({rule.limit_text()})",
                          f"bridge {v:.1f} mm long ({rule.limit_text()})",
                          [_where(part, k)], "sampled",
                          {"directions": int(S["bridge_directions"])}))
    else:
        out.append(result(rule, None, "no bridges", "", [], "sampled"))
    bv = np.full(len(part.P), np.nan)
    for i, x in real.items():
        bv[i] = x
    marks = []
    for fi in sorted({int(part.F[i]) for i in real}):  # the longest span on each ceiling
        k = max((i for i in real if part.F[i] == fi), key=lambda i: (round(real[i], 6), -i))
        _, a, b = spans[k]
        marks.append({"type": "segment", "points": [pt(a), pt(b)], "value": r(real[k], 2),
                      "label": f"{real[k]:.1f} mm", "fail": not rule.ok(real[k])})
    _surf(out[-1], "bridge_span", bv, np.isfinite(bv) & (bv > rule.value + 1e-9),
          "shortest supported span through each point of a flat ceiling; blank elsewhere",
          marks=marks)
    return out


# -------------------------------------------------- removable support

_OUT_DIRS = [np.array(v, float) / np.linalg.norm(v) for v in
             [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1),
              (1, 1, 1), (1, 1, -1), (1, -1, 1), (1, -1, -1), (-1, 1, 1), (-1, 1, -1), (-1, -1, 1), (-1, -1, -1)]]


def support_samples(part: Part, dep: list[RuleResult]) -> np.ndarray:
    """Samples that would need support in this direction: the overhang
    rule's failing samples and those of bridges that are too long."""
    need = np.zeros(len(part.P), bool)
    for x in dep:
        if x.id in ("FDM_OVERHANG", "FDM_BRIDGE") and x.surface is not None:
            need |= np.asarray(x.surface.ready().fail, bool)
    return need


def support_columns(part: Part, d, need, on_part: bool = False) -> dict:
    """For each sample needing support: (height of the support column down to
    the part or the bed, the column's middle point[, whether it ends on the
    part rather than the bed])."""
    d = unit(d)
    zmin, _ = vertical_extent(part, d)
    out = {}
    for i in np.flatnonzero(need):
        h = part.ray(part.P[i], -d)
        lands = math.isfinite(h)
        if not lands:
            h = float(part.P[i] @ d) - zmin
        col = (h, part.P[i] - d * (h / 2))
        out[int(i)] = col + (lands,) if on_part else col
    return out


def support_removable(part: Part, proc: Process, d, dep: list[RuleResult]) -> RuleResult:
    """Support that cannot be removed fails the part, whether support is
    allowed or not. For each bit of surface needing support, the middle of
    its support column is judged:

    * closed in: no straight line from it in 14 directions (axes and
      diagonals) leaves the part -- the support is in a closed pocket or
      cavity and cannot be reached;
    * too narrow: the column ends on the part and is shorter than the reach
      limit (support in a thin gap between a ceiling and a floor of the
      part; a short column down to the bed is open all round), or the slot
      it stands in is
      narrower than the reach limit in some horizontal direction (walls on
      both sides closer than that).

    The value is the area of overhang held up by such support. Sampled
    (the overhang samples), with exact rays."""
    rule = proc.rules["FDM_SUPPORT_REMOVABLE"]
    reach = float(proc.settings["support_reach"])
    d = unit(d)
    if not any(x.id == "FDM_OVERHANG" for x in dep):
        from dataclasses import replace

        dep = dependent(part, replace(proc, wanted=None), d)
    need = support_samples(part, dep)
    cols = support_columns(part, d, need, on_part=True)
    a = np.array([1.0, 0, 0]) if abs(d[0]) < 0.9 else np.array([0, 1.0, 0])
    e1 = unit(np.cross(d, a))
    e2 = np.cross(d, e1)
    horiz = [unit(math.cos(t) * e1 + math.sin(t) * e2) for t in np.arange(4) * math.pi / 4]
    bad = np.zeros(len(part.P), bool)
    why = {}
    for i, (h, m, lands) in cols.items():
        if all(math.isfinite(part.ray(m, u, start=0.0)) for u in _OUT_DIRS):
            bad[i], why[i] = True, "closed in"
            continue
        if lands and h < reach:  # a thin gap between two parts of the part (the bed is open all round)
            bad[i], why[i] = True, f"in a {h:.1f} mm gap"
            continue
        for u in horiz:
            w = part.ray(m, u, start=0.0) + part.ray(m, -u, start=0.0)
            if w < reach:
                bad[i], why[i] = True, f"in a {w:.1f} mm slot"
                break
    area = float((part.W[bad] * np.abs(part.N[bad] @ d)).sum())
    total = float((part.W[need] * np.abs(part.N[need] @ d)).sum())
    if not need.any():
        res = result(rule, 0.0, "no support needed in this direction", "", [], "sampled", {"support_area_mm2": 0.0})
    else:
        k = int(np.flatnonzero(bad)[0]) if bad.any() else int(np.flatnonzero(need)[0])
        res = result(rule, r(area, 2),
                     f"all support can be reached ({total:.0f} mm2 of overhang held up)",
                     f"{area:.0f} mm2 of support cannot be removed ({why.get(k, '')} at {pt(part.P[k])}; "
                     f"reach limit {reach:g} mm)",
                     [_where(part, k)], "sampled",
                     {"support_area_mm2": r(total, 2), "unremovable_area_mm2": r(area, 2), "reach_mm": reach})
    vals = np.where(need, np.where(bad, 1.0, 0.0), np.nan)
    return _surf(res, "support_removable", vals, bad,
                 "1 = support here could not be removed; 0 = support here can be reached; blank: no support",
                 "flag")
