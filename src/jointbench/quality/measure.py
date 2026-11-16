"""The modelling-quality rules. Every one is measured exactly on the solid's
own faces, edges and corners -- areas, lengths, angles between faces at an
edge, angles between edges at a corner. Nothing is sampled, so nothing small
can slip between sample points; the display mesh is used only to show the
results."""
from __future__ import annotations

import math

import numpy as np
from OCP.BRep import BRep_Tool
from OCP.BRepAdaptor import BRepAdaptor_Curve, BRepAdaptor_Surface
from OCP.BRepTools import BRepTools, BRepTools_WireExplorer
from OCP.GCPnts import GCPnts_AbscissaPoint
from OCP.GeomAbs import GeomAbs_Cylinder, GeomAbs_Plane
from OCP.TopAbs import TopAbs_EDGE, TopAbs_FACE, TopAbs_REVERSED, TopAbs_VERTEX, TopAbs_WIRE
from OCP.TopExp import TopExp, TopExp_Explorer
from OCP.TopoDS import TopoDS

from ..dfm.fdm import _surf
from ..dfm.geom import Part, pt, r, unit
from ..dfm.rules import Process, RuleResult, result

MAX_MARKS = 40


def _edge_length(e) -> float:
    return float(GCPnts_AbscissaPoint.Length_s(BRepAdaptor_Curve(e)))


def _vec(p):
    return np.array([p.X(), p.Y(), p.Z()])


def _face_field(part: Part, per_face: dict[int, float]) -> np.ndarray:
    v = np.full(len(part.P), np.nan)
    for f, val in per_face.items():
        v[part.F == f] = val
    return v


def _ring(p, label, fail=True):
    return {"type": "point", "point": pt(p), "label": label, "fail": fail}


# ------------------------------------------------------------ face shape


def face_shapes(part: Part) -> list[dict]:
    """Area, outer perimeter, mean width (2 x area / outer perimeter) and
    length (half the outer perimeter) of every face. Exact."""
    out = []
    for i, f in enumerate(part.faces):
        area = part.area(i)
        ow = BRepTools.OuterWire_s(f)
        per = 0.0
        ex = TopExp_Explorer(ow, TopAbs_EDGE)
        while ex.More():
            e = TopoDS.Edge(ex.Current())
            if not BRep_Tool.Degenerated_s(e):
                per += _edge_length(e)
            ex.Next()
        width = 2 * area / per if per > 0 else 0.0
        length = per / 2
        sel = np.flatnonzero(part.F == i)
        where = part.P[sel[len(sel) // 2]] if sel.size else _face_point(f)
        out.append({"face": i, "area": area, "perimeter": per, "width": width,
                    "length": length, "where": where})
    return out


def _face_point(f):
    ad = BRepAdaptor_Surface(f)
    u0, u1, v0, v1 = BRepTools.UVBounds_s(f)
    return _vec(ad.Value((u0 + u1) / 2, (v0 + v1) / 2))


# --------------------------------------------------------------- corners


def face_corners(part: Part, tangent: float = 0.05) -> list[dict]:
    """The angle inside each face at each of its corners, between the two
    boundary edges that meet there. Exact (edge tangents at the corner).

    ``blend`` marks a cusp that is no wedge of material: the two edges meet
    with equal tangents (angle under ``tangent`` deg) and the two faces
    beyond them run on as one smooth surface there (their normals at the
    corner within ``tangent`` deg). That is where a fillet blends tangent
    into a face and the blend line ends on this face -- e.g. a rib's side
    face at the end of a root fillet. The material there turns through the
    angles between this face and its neighbours, not through the cusp. A
    cusp whose neighbours meet at an angle (a knife edge of material between
    a round and a tangent flat) is no blend."""
    from OCP.collections import IndexedDataMap_TopoDS_Shape_List_TopoDS_Shape_TopTools_ShapeMapHasher as M

    emap = M()
    TopExp.MapShapesAndAncestors_s(part.shape, TopAbs_EDGE, TopAbs_FACE, emap)

    def beyond(e, i):
        """The face on the other side of edge e from face i, or None."""
        k = emap.FindIndex(e)
        if k == 0:
            return None
        js = {part.face_index(TopoDS.Face(g)) for g in emap.FindFromIndex(k)} - {i}
        return js.pop() if len(js) == 1 else None

    def blend(i, e_in, e_out, q):
        a, b = beyond(e_in, i), beyond(e_out, i)
        if a is None or b is None:
            return False
        if a == b:
            return True
        c = float(part.normal_at(a, q) @ part.normal_at(b, q))
        return math.degrees(math.acos(max(-1.0, min(1.0, c)))) < tangent

    out = []
    for i, f in enumerate(part.faces):
        wex = TopExp_Explorer(f, TopAbs_WIRE)
        while wex.More():
            w = TopoDS.Wire(wex.Current())
            seq = []
            it = BRepTools_WireExplorer(w, f)
            while it.More():
                e = it.Current()
                if not BRep_Tool.Degenerated_s(e):
                    c = BRepAdaptor_Curve(e)
                    t0, t1 = c.FirstParameter(), c.LastParameter()
                    rev = e.Orientation() == TopAbs_REVERSED
                    a, b = (t1, t0) if rev else (t0, t1)  # traversal start, end
                    p_s, d_s = _d1(c, a)
                    p_e, d_e = _d1(c, b)
                    if rev:
                        d_s, d_e = -d_s, -d_e
                    seq.append((p_s, unit(d_s), p_e, unit(d_e), e))
                else:
                    seq.append(None)
                it.Next()
            n = len(seq)
            for k in range(n):
                cur, nxt = seq[k], seq[(k + 1) % n]
                if cur is None or nxt is None or n < 2:
                    continue
                corner = cur[2]
                incoming, outgoing = cur[3], nxt[1]
                if np.linalg.norm(incoming) == 0 or np.linalg.norm(outgoing) == 0:
                    continue
                ang = math.degrees(math.acos(max(-1.0, min(1.0, float(-incoming @ outgoing)))))
                out.append({"face": i, "angle": ang, "where": corner,
                            "blend": ang < tangent and blend(i, cur[4], nxt[4], corner)})
            wex.Next()
    return out


def _d1(c, t):
    from OCP.gp import gp_Pnt, gp_Vec

    p, v = gp_Pnt(), gp_Vec()
    c.D1(t, p, v)
    return _vec(p), _vec(v)


# ----------------------------------------------------------------- edges


def edge_turns(part: Part, n: int = 5) -> list[dict]:
    """For every edge between two faces: the smallest and largest angle the
    surface turns across it, read at n points along the edge. Exact at those
    points (true surface normals)."""
    from OCP.collections import IndexedDataMap_TopoDS_Shape_List_TopoDS_Shape_TopTools_ShapeMapHasher as M

    emap = M()
    TopExp.MapShapesAndAncestors_s(part.shape, TopAbs_EDGE, TopAbs_FACE, emap)
    by_index = {e.index: e for e in part.edges()}
    out = []
    for k in range(1, emap.Extent() + 1):
        info = by_index.get(k - 1)
        if info is None:
            continue
        e = TopoDS.Edge(emap.FindKey(k))
        c = BRepAdaptor_Curve(e)
        t0, t1 = c.FirstParameter(), c.LastParameter()
        turns = []
        for j in range(n):
            t = t0 + (t1 - t0) * (j + 0.5) / n
            q = _vec(c.Value(t))
            n1, n2 = part.normal_at(info.faces[0], q), part.normal_at(info.faces[1], q)
            turns.append(math.degrees(math.acos(max(-1.0, min(1.0, float(n1 @ n2))))))
        out.append({"edge": info.index, "faces": info.faces, "min": min(turns), "max": max(turns),
                    "concave": info.concave, "where": info.mid, "length": _edge_length(e)})
    return out


def _same_surface(part: Part, i: int, j: int) -> bool:
    a, b = BRepAdaptor_Surface(part.faces[i]), BRepAdaptor_Surface(part.faces[j])
    if a.GetType() == GeomAbs_Plane and b.GetType() == GeomAbs_Plane:
        (n1, p1), (n2, p2) = part.plane(i), part.plane(j)
        return float(n1 @ n2) > 1 - 1e-9 and abs(float(n1 @ (p2 - p1))) < 1e-6
    if a.GetType() == GeomAbs_Cylinder and b.GetType() == GeomAbs_Cylinder:
        ca, cb = a.Cylinder(), b.Cylinder()
        if abs(ca.Radius() - cb.Radius()) > 1e-6:
            return False
        da, db = ca.Axis().Direction(), cb.Axis().Direction()
        if abs(abs(da.Dot(db)) - 1) > 1e-9:
            return False
        la, lb = ca.Axis().Location(), cb.Axis().Location()
        w = _vec(lb) - _vec(la)
        d = _vec(da)
        return float(np.linalg.norm(w - (w @ d) * d)) < 1e-6
    return False


# ----------------------------------------------------------------- rules


def run(part: Part, proc: Process) -> tuple[list[RuleResult], dict]:
    R, S = proc.rules, proc.settings
    out = []
    shapes = face_shapes(part)

    # sliver faces
    rule = R["Q_SLIVER_FACE"]
    aspect = S["sliver_aspect"]
    slivers = [x for x in shapes if x["width"] > 0 and x["length"] / x["width"] >= aspect]
    bad = [x for x in slivers if x["width"] < rule.value]
    v = min((x["width"] for x in slivers), default=None)
    worst = min(bad, key=lambda x: (x["width"], x["face"])) if bad else None
    res = result(rule, r(v, 4) if v is not None else None,
                 f"no sliver faces (narrowest long face {v:.2f} mm wide)" if v is not None else "no long narrow faces",
                 f"{len(bad)} sliver face(s), narrowest {worst['width']:.3f} mm wide and "
                 f"{worst['length']:.1f} mm long" if worst else "",
                 [{"point": pt(x["where"]), "face": x["face"]} for x in sorted(bad, key=lambda x: x["width"])[:MAX_MARKS]],
                 details={"faces": [{"face": x["face"], "width_mm": r(x["width"], 4),
                                     "length_mm": r(x["length"], 3)} for x in bad]})
    out.append(_surf(res, "sliver_width", _face_field(part, {x["face"]: x["width"] for x in slivers}),
                     np.isin(part.F, [x["face"] for x in bad]),
                     f"mean width of faces at least {aspect:g} x longer than wide; blank: other faces",
                     marks=[_ring(x["where"], f"{x['width']:.2f} mm") for x in bad[:MAX_MARKS]]))

    # tiny faces
    rule = R["Q_TINY_FACE"]
    tiny = [x for x in shapes if x["area"] < rule.value]
    small = min(shapes, key=lambda x: (x["area"], x["face"]))
    res = result(rule, r(small["area"], 4),
                 f"smallest face {small['area']:.2f} mm2 ({rule.limit_text()})",
                 f"{len(tiny)} tiny face(s), smallest {small['area']:.3f} mm2 ({rule.limit_text()})",
                 [{"point": pt(x["where"]), "face": x["face"]} for x in tiny[:MAX_MARKS]])
    out.append(_surf(res, "face_area", _face_field(part, {x["face"]: x["area"] for x in shapes}),
                     np.isin(part.F, [x["face"] for x in tiny]), "area of each face",
                     marks=[_ring(x["where"], f"{x['area']:.3f} mm2") for x in tiny[:MAX_MARKS]]))

    # short edges
    rule = R["Q_SHORT_EDGE"]
    turns = edge_turns(part)
    elist = sorted(turns, key=lambda x: (x["length"], x["edge"]))
    short = [x for x in elist if x["length"] < rule.value]
    v = elist[0]["length"] if elist else None
    res = result(rule, r(v, 4) if v is not None else None,
                 f"shortest edge {v:.2f} mm ({rule.limit_text()})" if v is not None else "no edges",
                 f"{len(short)} very short edge(s), shortest {v:.3f} mm ({rule.limit_text()})" if short else "",
                 [{"point": pt(x["where"]), "face": x["faces"][0]} for x in short[:MAX_MARKS]])
    out.append(_surf(res, "short_edges", np.full(len(part.P), np.nan), np.zeros(len(part.P), bool),
                     "edges are drawn, not coloured: each very short edge is ringed",
                     marks=[_ring(x["where"], f"{x['length']:.3f} mm") for x in short[:MAX_MARKS]]))

    # small steps: a face between two faces facing the same way, low enough
    # that the two were surely meant to be flush
    rule = R["Q_SMALL_STEP"]
    cos_par = math.cos(math.radians(S["step_parallel"]))
    planes = {i: part.plane(i) for i in range(len(part.faces))}
    nbrs: dict[int, set] = {}
    for e in part.edges():
        a, b = e.faces
        nbrs.setdefault(a, set()).add(b)
        nbrs.setdefault(b, set()).add(a)
    steps = []
    for s_i, x in enumerate(shapes):
        ns = sorted(j for j in nbrs.get(s_i, ()) if planes[j] is not None)
        n_s = part.normal_at(s_i, x["where"])
        verts = [_vec(BRep_Tool.Pnt_s(TopoDS.Vertex(v.wrapped))) for v in _face_vertices(part, s_i)]
        best = None
        for a_k, f1 in enumerate(ns):
            for f2 in ns[a_k + 1:]:
                n1, p1 = planes[f1]
                n2, p2 = planes[f2]
                if float(n1 @ n2) < cos_par or abs(float(n1 @ n_s)) > 0.5:
                    continue
                h = max(abs(float(n1 @ (q - p1))) for q in verts)
                h = max(h, max(abs(float(n2 @ (q - p2))) for q in verts))
                if h > 1e-4 and (best is None or h < best[0]):
                    best = (h, f1, f2)
        if best:
            steps.append({"face": s_i, "height": best[0], "between": best[1:], "where": x["where"]})
    bad = [x for x in steps if x["height"] < rule.value]
    v = min((x["height"] for x in steps), default=None)
    res = result(rule, r(v, 4) if v is not None else None,
                 f"lowest step {v:.2f} mm ({rule.limit_text()})" if v is not None else "no steps",
                 f"{len(bad)} hairline step(s), lowest {v:.3f} mm high: faces meant to be flush"
                 if bad else "",
                 [{"point": pt(x["where"]), "face": x["face"]} for x in sorted(bad, key=lambda x: x["height"])[:MAX_MARKS]],
                 details={"steps": [{"face": x["face"], "height_mm": r(x["height"], 4),
                                     "between_faces": list(x["between"])} for x in bad]})
    out.append(_surf(res, "step_height", _face_field(part, {x["face"]: x["height"] for x in steps}),
                     np.isin(part.F, [x["face"] for x in bad]),
                     "height of faces that step between two faces facing the same way",
                     marks=[_ring(x["where"], f"step {x['height']:.2f} mm") for x in bad[:MAX_MARKS]]))

    # thin wedges and knife edges: small angles inside a face's corners and
    # across edges where the surface folds back on itself
    rule = R["Q_THIN_WEDGE"]
    # a cusp where a fillet blends tangent into a face is no wedge (see face_corners)
    corners = face_corners(part, S["tangent"])
    wedges = [{"angle": c["angle"], "where": c["where"], "face": c["face"], "kind": "face corner"}
              for c in corners if not c["blend"]]
    wedges += [{"angle": 180.0 - t["max"], "where": t["where"], "face": t["faces"][0],
                "kind": "crack" if t["concave"] else "knife edge"} for t in turns]
    bad = sorted([w for w in wedges if w["angle"] < rule.value], key=lambda w: (w["angle"], w["face"]))
    v = min((w["angle"] for w in wedges), default=None)
    res = result(rule, r(v, 3) if v is not None else None,
                 f"sharpest corner {v:.1f} deg ({rule.limit_text()})" if v is not None else "no corners",
                 f"{len(bad)} thin wedge(s), sharpest {v:.2f} deg ({bad[0]['kind']})" if bad else "",
                 [{"point": pt(w["where"]), "face": w["face"]} for w in bad[:MAX_MARKS]],
                 details={"wedges": [{"face": w["face"], "angle_deg": r(w["angle"], 3), "kind": w["kind"],
                                      "point": pt(w["where"])} for w in bad[:MAX_MARKS]]})
    wf = sorted({w["face"] for w in bad})
    out.append(_surf(res, "thin_wedge", _face_field(part, {f: 1.0 for f in wf}), np.isin(part.F, wf),
                     "faces with a corner under the limit; each wedge tip is ringed", "flag",
                     marks=[_ring(w["where"], f"{w['angle']:.1f} deg") for w in bad[:MAX_MARKS]]))

    # near-flat creases
    rule = R["Q_CREASE"]
    tang = S["tangent"]
    creases = [t for t in turns if tang <= t["max"] < rule.value]
    v = min((t["max"] for t in turns if t["max"] >= tang), default=None)
    res = result(rule, r(v, 3) if v is not None else None,
                 f"gentlest bend between faces {v:.1f} deg ({rule.limit_text()})" if v is not None
                 else "all faces meet smoothly or not at all",
                 f"{len(creases)} near-flat crease(s), gentlest {v:.2f} deg: faces meant to be one"
                 if creases else "",
                 [{"point": pt(t["where"]), "face": t["faces"][0]} for t in creases[:MAX_MARKS]])
    marks = [{"type": "polyline", "points": [pt(q) for q in part.edge_polyline(t["edge"])],
              "label": "crease", "fail": True} for t in creases[:MAX_MARKS]]
    marks += [_ring(t["where"], f"{t['max']:.1f} deg") for t in creases[:MAX_MARKS]]
    cf = sorted({f for t in creases for f in t["faces"]})
    out.append(_surf(res, "crease", _face_field(part, {f: 1.0 for f in cf}), np.isin(part.F, cf),
                     "faces meeting at a near-flat crease; the crease edges are drawn", "flag",
                     marks=marks))

    # unmerged faces
    rule = R["Q_UNMERGED"]
    pairs = [t for t in turns if t["max"] < tang and _same_surface(part, *t["faces"])]
    res = result(rule, len(pairs), "no face is split on its own surface",
                 f"{len(pairs)} pair(s) of faces on the same surface not merged",
                 [{"point": pt(t["where"]), "face": t["faces"][0]} for t in pairs[:MAX_MARKS]])
    uf = sorted({f for t in pairs for f in t["faces"]})
    marks = [{"type": "polyline", "points": [pt(q) for q in part.edge_polyline(t["edge"])],
              "label": "split", "fail": True} for t in pairs[:MAX_MARKS]]
    out.append(_surf(res, "unmerged", _face_field(part, {f: 1.0 for f in uf}), np.isin(part.F, uf),
                     "faces that share a surface with a neighbour; the needless edges are drawn",
                     "flag", marks=marks))

    # near-coincident corners
    rule = R["Q_NEAR_VERTICES"]
    near, dmin = _near_vertices(part, rule.value)
    res = result(rule, r(dmin, 5) if dmin is not None else None,
                 f"closest unjoined corners {dmin:.2f} mm apart ({rule.limit_text()})" if dmin is not None
                 else "fewer than two corners",
                 f"{len(near)} pair(s) of corners nearly on top of each other, closest {dmin:.4f} mm"
                 if near else "",
                 [{"point": pt(a), "face": None} for a, _, _ in near[:MAX_MARKS]])
    out.append(_surf(res, "near_corners", np.full(len(part.P), np.nan), np.zeros(len(part.P), bool),
                     "corners are ringed, not coloured", "flag",
                     marks=[_ring(a, f"{d:.3f} mm") for a, _, d in near[:MAX_MARKS]]))

    # stubs: a member that runs on past the face it joins, by a short way
    rule = R["Q_STUB"]
    stubs = find_stubs(part, S["corner_turn"])
    bad = [x for x in stubs if x["height"] < rule.value]
    v = min((x["height"] for x in stubs), default=None)
    res = result(rule, r(v, 3) if v is not None else None,
                 f"no short stubs (shortest run-on {v:.1f} mm, {rule.limit_text()})" if v is not None
                 else "no member runs on past a face it joins",
                 f"{len(bad)} stub(s): a member runs {v:.2f} mm past the face it joins" if bad else "",
                 [{"point": pt(x["where"]), "face": x["faces"][0]} for x in bad[:MAX_MARKS]],
                 details={"stubs": [{"faces": x["faces"], "past_face": x["plate"],
                                     "height_mm": r(x["height"], 3)} for x in bad]})
    sf = sorted({f for x in bad for f in x["faces"]})
    out.append(_surf(res, "stub", _face_field(part, {f: 1.0 for f in sf}), np.isin(part.F, sf),
                     "faces of material that runs on past the face it joins", "flag",
                     marks=[_ring(x["where"], f"stub {x['height']:.1f} mm") for x in bad[:MAX_MARKS]]))

    # shallow joints: a member meets a face at a narrow angle, leaving a crevice
    rule = R["Q_SHALLOW_JOINT"]
    inside = [t for t in turns if t["concave"] and t["max"] >= S["corner_turn"]]
    joints = [dict(t, angle=180.0 - t["max"]) for t in inside]
    bad = sorted([j for j in joints if j["angle"] < rule.value], key=lambda j: (j["angle"], j["edge"]))
    v = min((j["angle"] for j in joints), default=None)
    res = result(rule, r(v, 2) if v is not None else None,
                 f"narrowest inside corner {v:.0f} deg ({rule.limit_text()})" if v is not None
                 else "no inside corners",
                 f"{len(bad)} shallow joint(s): faces meet at {v:.1f} deg, leaving a narrow crevice"
                 if bad else "",
                 [{"point": pt(j["where"]), "face": j["faces"][0]} for j in bad[:MAX_MARKS]])
    marks = [{"type": "polyline", "points": [pt(q) for q in part.edge_polyline(j["edge"])],
              "label": "", "fail": True} for j in bad[:MAX_MARKS]]
    marks += [_ring(j["where"], f"{j['angle']:.0f} deg") for j in bad[:MAX_MARKS]]
    jf = sorted({f for j in bad for f in j["faces"]})
    out.append(_surf(res, "shallow_joint", _face_field(part, {f: 1.0 for f in jf}), np.isin(part.F, jf),
                     "faces meeting at a narrow inside angle; the crevice edges are drawn", "flag",
                     marks=marks))

    info = _info(part, shapes)
    fillets = [x for x in part.rounds() if x.concave and not x.full]
    info["inside_corners_sharp"] = len(inside)
    info["inside_corners_filleted"] = len(fillets)
    return out, info


def _member_side(part: Part, g: int, n, tilt_deg: float = 30.0) -> bool:
    """Could face ``g`` be the side of a member that runs along ``n``? A
    plane whose normal is nearly square to ``n``, or a cylinder whose axis
    is nearly along ``n``; anything else (a cylinder lying across, a sphere,
    a free-form surface) is not."""
    from OCP.BRepAdaptor import BRepAdaptor_Surface

    pl = part.plane(g)
    if pl is not None:
        return abs(float(pl[0] @ n)) <= math.sin(math.radians(tilt_deg))
    ad = BRepAdaptor_Surface(part.faces[g])
    if ad.GetType() == GeomAbs_Cylinder:
        d = ad.Cylinder().Axis().Direction()
        return abs(float(np.array([d.X(), d.Y(), d.Z()]) @ n)) >= math.cos(math.radians(tilt_deg))
    return False


def find_stubs(part: Part, turn_min: float, eps: float = 1e-4) -> list[dict]:
    """Material that runs on past a flat face P it is joined to.

    From every inside corner on P, collect the connected faces lying wholly
    on P's outer side. They are the end of a member pushed through the
    plate -- a stub -- when the member visibly continues on the plate's
    inner side, in one of two ways:
      (a) one of these faces lies on the same plane or cylinder as a face
          wholly behind P (the member's side, split by the plate), or
      (b) a neighbouring face runs through P's plane and meets, at an inside
          corner, a flat face facing the opposite way to P (the member's
          side runs on through and joins the plate's other face).
    The stub's length is how far it reaches past P. A boss or rib standing
    on P, or a rim flush with the plate's edge, meets neither condition, and
    neither does a plate seen from the side of a bar crossing it (the
    plate's faces run across the bar unbroken). Exact: vertex heights above
    P's plane, and surface identity."""
    from OCP.BRep import BRep_Tool as BT

    verts = {}
    for i in range(len(part.faces)):
        verts[i] = np.array([_vec(BT.Pnt_s(TopoDS.Vertex(v.wrapped))) for v in _face_vertices(part, i)])
    faces = [i for i in verts if len(verts[i])]
    nb: dict[int, set] = {}
    for e in part.edges():
        a, b = e.faces
        nb.setdefault(a, set()).add(b)
        nb.setdefault(b, set()).add(a)
    cos_opp = math.cos(math.radians(30))
    found, seen = [], set()
    for e in part.edges():
        if not e.concave or e.turn < turn_min:
            continue
        for P, Q in (e.faces, e.faces[::-1]):
            pl = part.plane(P)
            if pl is None or Q not in verts or not len(verts[Q]):
                continue
            n, o = pl
            h = {i: (verts[i] - o) @ n for i in faces}
            if h[Q].min() < -eps or h[Q].max() <= eps:
                continue
            comp, todo = {Q}, [Q]
            while todo:
                f = todo.pop()
                for g in nb.get(f, ()):
                    if g != P and g not in comp and g in h and h[g].min() >= -eps:
                        comp.add(g)
                        todo.append(g)
            key = (P, frozenset(comp))
            if key in seen:
                continue
            seen.add(key)
            behind = [g for g in faces if h[g].max() <= eps and g != P]
            twin = any(_same_surface(part, f, g) for f in comp for g in behind)
            # (b) needs the side of a member running through P: a flat side
            # square to P (normal within 30 deg of P's plane) or a round side
            # whose axis runs along P's normal. A curved surface lying across
            # P -- the top of a saddle round a counterbore cut into it -- is no
            # member's side, however far it dips below P's plane
            crossing = {g for f in comp for g in nb.get(f, ())
                        if g != P and g not in comp and h[g].min() < -eps and _member_side(part, g, n)}
            runs_on = False
            for g in crossing:
                for e2 in part.neighbours(g):
                    k = e2.faces[1] if e2.faces[0] == g else e2.faces[0]
                    pk = part.plane(k)
                    if (k != P and e2.concave and e2.turn >= turn_min and pk is not None
                            and float(pk[0] @ n) < -cos_opp):
                        runs_on = True
            if not (twin or runs_on):
                continue
            height = max(float(h[f].max()) for f in comp | crossing)
            top = max(comp | crossing, key=lambda f: (float(h[f].max()), -f))
            where = verts[top][int(np.argmax(h[top]))]
            found.append({"plate": P, "faces": sorted(comp), "height": height, "where": where})
    found.sort(key=lambda x: (x["height"], x["plate"]))
    return found

def _face_vertices(part: Part, i: int):  # noqa: D103
    import build123d as bd

    return bd.Face(part.faces[i]).vertices()


def _near_vertices(part: Part, limit: float):
    from scipy.spatial import cKDTree

    from OCP.collections import IndexedMap_TopoDS_Shape_TopTools_ShapeMapHasher as Map

    vmap = Map()
    TopExp.MapShapes_s(part.shape, TopAbs_VERTEX, vmap)
    pts = np.array([_vec(BRep_Tool.Pnt_s(TopoDS.Vertex(vmap.FindKey(i)))) for i in range(1, vmap.Extent() + 1)])
    if len(pts) < 2:
        return [], None
    joined = set()
    emap = Map()
    TopExp.MapShapes_s(part.shape, TopAbs_EDGE, emap)
    for k in range(1, emap.Extent() + 1):
        e = TopoDS.Edge(emap.FindKey(k))
        a = vmap.FindIndex(TopExp.FirstVertex_s(e)) - 1
        b = vmap.FindIndex(TopExp.LastVertex_s(e)) - 1
        joined.add((min(a, b), max(a, b)))
    tree = cKDTree(pts)
    dist, idx = tree.query(pts, k=min(len(pts), 8))
    dmin, near = None, []
    for i in range(len(pts)):
        for d, j in zip(dist[i][1:], idx[i][1:]):
            if j <= i or (i, j) in joined:
                continue
            if dmin is None or d < dmin:
                dmin = float(d)
            if d < limit:
                near.append((pts[i], pts[j], float(d)))
    near.sort(key=lambda t: t[2])
    return near, dmin


def _info(part: Part, shapes) -> dict:
    kinds: dict[str, list] = {}
    for i, f in enumerate(part.faces):
        t = BRepAdaptor_Surface(f).GetType()
        name = {GeomAbs_Plane: "plane", GeomAbs_Cylinder: "cylinder"}.get(t, "other")
        kinds.setdefault(name, []).append(shapes[i]["area"])
    n = len(part.faces)
    total = sum(x["area"] for x in shapes) or 1.0
    return {
        "faces": n,
        "edges": len(part.edges()),
        "share_of_faces": {k: r(len(v) / n, 3) for k, v in sorted(kinds.items())},
        "share_of_area": {k: r(sum(v) / total, 3) for k, v in sorted(kinds.items())},
    }
