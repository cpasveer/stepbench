"""Spline joint: straight-sided splines (ISO 14) that slide together along the axis.

Either side may be given; the agent designs the other. The profile is
checked by cutting the agent's part square to the axis at three stations
along the engagement and reading the section's exact edges: straight flank
lines and circular arcs. From them:

* SPL_PROFILE    the number of splines, flank pairs parallel (straight-sided),
                 each spline symmetric about a radius, splines evenly spaced,
                 the same at every station
* SPL_WIDTH      spline / slot width (distance between the two parallel
                 flanks) in the ISO 14 zone of the agent's side
* SPL_MINOR      minor diameter (the centring diameter) in its zone
* SPL_MAJOR      major diameter in its zone
* SPL_ENGAGEMENT the profile spans at least the stated length of the
                 counterpart's
* SPL_INSERTION  slid out along +z in 1 mm steps, from home until clear,
                 nothing overlaps (which also covers the home position)

Involute splines (ISO 4156 / DIN 5480) are not built; see docs/joints.md.
"""
from __future__ import annotations

import math

import build123d as bd
from OCP.BRepAdaptor import BRepAdaptor_Curve
from OCP.BRepAlgoAPI import BRepAlgoAPI_Section
from OCP.GeomAbs import GeomAbs_Circle, GeomAbs_Line
from OCP.gp import gp_Dir, gp_Pln, gp_Pnt

from . import common as C
from .standards import SPLINE_SLIDING, SPLINES, mid, zone

NAME = "spline"
TITLE = "Spline joint (ISO 14 straight-sided)"
PARALLEL_MAX = 0.01  # deg: the two flanks of one spline are parallel within this
SYMMETRY_MAX = 0.01  # mm: a spline's centre line passes this close to the axis
SPACING_MAX = 0.02  # deg: splines evenly spaced within this


def generate(seed: int, design: str | None = None, rules_told: bool = True, aligned: bool = False) -> C.Instance:
    rng = C.rng_for(NAME, seed)
    sp = rng.choice(SPLINES)
    d_draw = rng.choice(["shaft", "hub"])
    design = design or d_draw
    Lh = round(max(20.0, sp.D * rng.uniform(0.7, 1.2)), 1)
    phase = round(rng.uniform(0, 360.0 / sp.n), 3)
    zones = {side: {k: list(zone(getattr(sp, k), code)) for k, code in SPLINE_SLIDING[side].items()}
             for side in ("hub", "shaft")}
    p = dict(series=sp.series, n=sp.n, d=sp.d, D=sp.D, B=sp.B, code=sp.code, design=design,
             hub_len=Lh, hub_od=round(sp.D * 1.5 + 10, 1), phase=phase, engage_min=round(0.9 * Lh, 1),
             zones=zones, given_side="hub" if design == "shaft" else "shaft")
    return C.Instance(NAME, seed, C.random_frame(rng, aligned), p, rules_told, {"design": design})


# ------------------------------------------------------------- geometry


def _teeth(n, width, r_out, phase, z0, z1):
    """n bars of the given width from the axis out to r_out, as one solid."""
    h = z1 - z0
    out = None
    for i in range(n):
        bar = bd.Pos(r_out / 2, 0, (z0 + z1) / 2) * bd.Box(r_out, width, h)
        bar = C.rotated_z(bar, phase + i * 360.0 / n)
        out = bar if out is None else out + bar
    return out


def shaft_solid(n, d, D, B, phase, z0, z1):
    h = z1 - z0
    core = bd.Pos(0, 0, (z0 + z1) / 2) * bd.Cylinder(d / 2, h)
    teeth = _teeth(n, B, D / 2 + 1, phase, z0, z1) & (bd.Pos(0, 0, (z0 + z1) / 2) * bd.Cylinder(D / 2, h))
    return core + teeth


def hub_solid(n, d, D, B, phase, z0, z1, od):
    h = z1 - z0
    ring = bd.Pos(0, 0, (z0 + z1) / 2) * bd.Cylinder(od / 2, h)
    hole = bd.Pos(0, 0, (z0 + z1) / 2) * bd.Cylinder(d / 2, h + 2)
    slots = _teeth(n, B, D / 2 + 1, phase, z0 - 1, z1 + 1) & \
        (bd.Pos(0, 0, (z0 + z1) / 2) * bd.Cylinder(D / 2, h + 2))
    return ring - hole - slots


def _given_dims(p):
    z = p["zones"][p["given_side"]]
    return mid(z["d"]), mid(z["D"]), mid(z["B"])


def fixed_local(inst) -> dict:
    p = inst.p
    d, D, B = _given_dims(p)
    if p["design"] == "shaft":
        return {"hub": hub_solid(p["n"], d, D, B, p["phase"], 0.0, p["hub_len"], p["hub_od"])}
    # given shaft: splined from z = -10 to hub_len + 10, plain journal below
    s = shaft_solid(p["n"], d, D, B, p["phase"], -10.0, p["hub_len"] + 10.0)
    s = s + bd.Pos(0, 0, -20) * bd.Cylinder(D / 2 + 3, 20)
    return {"shaft": s}


def fixed_parts(inst) -> dict:
    return {k: C.to_world(v, inst.frame) for k, v in fixed_local(inst).items()}


def counterpart_z(p):
    return (0.0, p["hub_len"]) if p["design"] == "shaft" else (-10.0, p["hub_len"] + 10.0)


def reference(inst, B=None, d=None, D=None, n=None, phase=0.0, engage=None, collar=False, taper=False):
    p = inst.p
    z = p["zones"][p["design"]]

    def pick(v, zz):
        if v is None:
            return mid(zz)
        if isinstance(v, str):
            return zz[0 if v.startswith("lo") else 1] + float(v[2:])
        return v
    dd, DD, BB = pick(d, z["d"]), pick(D, z["D"]), pick(B, z["B"])
    nn = p["n"] if n is None else n
    ph = p["phase"] + phase
    Lh = p["hub_len"]
    if p["design"] == "shaft":
        z0 = 0.0 if engage is None else Lh - (p["engage_min"] + float(engage))
        z1 = Lh + 12.0
        part = shaft_solid(nn, dd, DD, BB, ph, z0, z1)
        part = part + bd.Pos(0, 0, z1 + 5) * bd.Cylinder(p["D"] / 2 + 3, 10)  # a collar above the hub
        if collar:  # a collar at the leading end: it cannot pass through the hub
            part = part + bd.Pos(0, 0, z0 - 1) * bd.Cylinder(p["D"] / 2 + 1, 2)
        if taper:  # the leading 4 mm of the splines 0.1 mm wider: no longer prismatic
            part = part + shaft_solid(nn, dd, DD, BB + 0.1, ph, z0, z0 + 4)
    else:
        z0 = 0.0 if engage is None else -10.0 + (p["hub_len"] + 20 - (p["engage_min"] + float(engage)))
        part = hub_solid(nn, dd, DD, BB, ph, z0, z0 + Lh, p["hub_od"])
        if collar:  # a lip inside the bore at the bottom
            part = part + (bd.Pos(0, 0, z0 - 1) * bd.Cylinder(p["hub_od"] / 2, 2)
                           - bd.Pos(0, 0, z0 - 1) * bd.Cylinder(p["d"] / 2 - 1, 2.2))
        if taper:  # the leading 4 mm of the slots 0.1 mm narrower
            part = part + hub_solid(nn, dd, DD, BB - 0.1, ph, z0, z0 + 4, p["hub_od"])
    return part


# ----------------------------------------------------------------- brief


def brief(inst) -> str:
    p = inst.p
    d, D, B = _given_dims(p)
    L = [C.frame_text(inst.frame), ""]
    side = "shaft" if p["design"] == "shaft" else "hub"
    other = "hub" if side == "shaft" else "shaft"
    zz = p["zones"][side]
    if side == "shaft":
        L.append(f"The given part is a hub (z = 0 to {p['hub_len']:g}) with an internal straight-sided spline "
                 f"ISO 14 {p['series']} series {p['code']}: {p['n']} slots, as modelled minor diameter {d:.4f}, "
                 f"major {D:.4f}, slot width {B:.4f} mm, one slot centred on the direction {p['phase']:g} deg from +x.")
        L.append(f"Design one splined shaft that slides into it along z; model it slid home, spanning the hub.")
    else:
        L.append(f"The given part is a splined shaft (splines from z = -10 to {p['hub_len'] + 10:g}) ISO 14 "
                 f"{p['series']} series {p['code']}: {p['n']} splines, as modelled minor diameter {d:.4f}, major "
                 f"{D:.4f}, spline width {B:.4f} mm, one spline centred on the direction {p['phase']:g} deg from +x.")
        L.append(f"Design one hub, {p['hub_len']:g} mm long, that slides onto it along z, between z = -10 and "
                 f"{p['hub_len'] + 10:g}.")
    if inst.rules_told:
        L.append(f"- Your {side}: minor diameter {zz['d'][0]:.4f} to {zz['d'][1]:.4f}, major {zz['D'][0]:.4f} to "
                 f"{zz['D'][1]:.4f}, {'spline' if side == 'shaft' else 'slot'} width {zz['B'][0]:.4f} to "
                 f"{zz['B'][1]:.4f} mm (ISO 14 sliding fit: {SPLINE_SLIDING[side]['d']}, "
                 f"{SPLINE_SLIDING[side]['D']}, {SPLINE_SLIDING[side]['B']}).")
    else:
        L.append(f"- ISO 14 {p['code']} straight-sided spline, sliding fit, the {other} as given.")
    L.append(f"- Straight-sided: the two flanks of each spline parallel within {PARALLEL_MAX:g} deg, symmetric about a "
             f"radius within {SYMMETRY_MAX:g} mm, splines evenly spaced within {SPACING_MAX:g} deg; on the z axis.")
    L.append(f"- The profile spans at least {p['engage_min']:g} mm of the given part's splined length.")
    L.append("- Slid out along +z from where you model it until clear, it touches the given part nowhere.")
    return "\n".join(L)


# ----------------------------------------------------------------- check


def section_edges(solid, z):
    """Exact edges of the section by the plane at height z: lines and circles."""
    sec = BRepAlgoAPI_Section(solid.wrapped, gp_Pln(gp_Pnt(0, 0, z), gp_Dir(0, 0, 1)))
    sec.Build()
    comp = bd.Compound(sec.Shape())
    lines, arcs, other = [], [], 0
    for e in comp.edges():
        ad = BRepAdaptor_Curve(e.wrapped)
        t = ad.GetType()
        p0, p1 = tuple(e.position_at(0)), tuple(e.position_at(1))
        if t == GeomAbs_Line:
            lines.append((p0, p1))
        elif t == GeomAbs_Circle:
            c = ad.Circle()
            loc = c.Location()
            pm = tuple(e.position_at(0.5))
            arcs.append(((loc.X(), loc.Y()), c.Radius(), math.atan2(pm[1], pm[0])))
        else:
            other += 1
    return lines, arcs, other


def measure_profile(solid, z, p):
    """Splines (or slots) of the section at z: list of (centre angle, width, symmetry offset, parallel error)
    plus the minor and major radii. Splines are the outer arcs' neighbouring flank pairs."""
    lines, arcs, other = section_edges(solid, z)
    lo_r, hi_r = p["d"] / 2 - 0.6, p["D"] / 2 + 0.6
    arcs = [a for a in arcs if lo_r <= a[1] <= hi_r and math.hypot(*a[0]) < 0.05]
    flank_min = 0.4 * (p["D"] - p["d"]) / 2
    fl = []
    for a, b in lines:
        ra, rb = math.hypot(a[0], a[1]), math.hypot(b[0], b[1])
        if not (lo_r <= min(ra, rb) and max(ra, rb) <= hi_r):
            continue
        if math.dist(a[:2], b[:2]) < flank_min:
            continue
        if ra > rb:
            a, b = b, a  # inner end first
        dx, dy = b[0] - a[0], b[1] - a[1]
        ln = math.hypot(dx, dy)
        m = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
        fl.append({"p": a, "dir": (dx / ln, dy / ln), "ang": math.atan2(m[1], m[0])})
    if not arcs or not fl:
        return None
    r_outer = max(a[1] for a in arcs)
    r_inner = min(a[1] for a in arcs)
    outer = [a for a in arcs if abs(a[1] - r_outer) < 1e-4]
    inner = [a for a in arcs if abs(a[1] - r_inner) < 1e-4]
    splines = []
    for _, _, am in outer:
        # nearest flank on each side of the outer arc's middle
        def rel(f):
            return (f["ang"] - am + math.pi) % (2 * math.pi) - math.pi
        left = [f for f in fl if rel(f) > 0]
        right = [f for f in fl if rel(f) < 0]
        if not left or not right:
            continue
        f1 = min(left, key=rel)
        f2 = max(right, key=rel)
        cross = f1["dir"][0] * f2["dir"][1] - f1["dir"][1] * f2["dir"][0]
        par = math.degrees(math.asin(min(1.0, abs(cross))))
        n1 = (-f1["dir"][1], f1["dir"][0])
        width = abs((f2["p"][0] - f1["p"][0]) * n1[0] + (f2["p"][1] - f1["p"][1]) * n1[1])
        # centre line: through the midpoint of the two flank lines, along their direction
        mx, my = (f1["p"][0] + f2["p"][0]) / 2, (f1["p"][1] + f2["p"][1]) / 2
        sym = abs(mx * n1[0] + my * n1[1])
        cang = math.degrees(math.atan2(f1["dir"][1] + f2["dir"][1], f1["dir"][0] + f2["dir"][0]))
        splines.append({"angle": cang, "width": width, "sym": sym, "par": par})
    return {"splines": splines, "minor": 2 * r_inner, "major": 2 * r_outer, "n_inner": len(inner),
            "n_outer": len(outer), "other_edges": other}


def check(solid, inst) -> list:
    p = inst.p
    part = C.to_local(solid, inst.frame)
    fixed = next(iter(fixed_local(inst).values()))
    side = p["design"]
    zz = p["zones"][side]
    out = []
    cz = counterpart_z(p)
    # stations: spread over the part's span within the counterpart's splined length
    bb = part.bounding_box()
    s0, s1 = max(cz[0], bb.min.Z), min(cz[1], bb.max.Z)
    span = s1 - s0
    stations = [s0 + span * f for f in (0.1, 0.5, 0.9)] if span > 0 else []
    profs = [(z, measure_profile(part, z, p)) for z in stations]
    probs, rows = [], []
    widths, minors, majors = [], [], []
    for z, pr in profs:
        if pr is None or not pr["splines"]:
            probs.append(f"z = {z:.2f}: no straight-sided spline profile in the section")
            continue
        sp = pr["splines"]
        if len(sp) != p["n"] or pr["n_outer"] != p["n"]:
            probs.append(f"z = {z:.2f}: {len(sp)} {'splines' if side == 'shaft' else 'slots'} (need {p['n']})")
        worst_par = max(s["par"] for s in sp)
        worst_sym = max(s["sym"] for s in sp)
        if worst_par > PARALLEL_MAX:
            probs.append(f"z = {z:.2f}: flanks {worst_par:.3f} deg from parallel (max {PARALLEL_MAX:g})")
        if worst_sym > SYMMETRY_MAX:
            probs.append(f"z = {z:.2f}: a spline's centre line {worst_sym:.4f} mm off the axis (max {SYMMETRY_MAX:g})")
        angs = sorted(s["angle"] % 360 for s in sp)
        pitch = 360.0 / p["n"]
        sp_err = max(abs(((a - angs[0]) / pitch - round((a - angs[0]) / pitch)) * pitch) for a in angs)
        if sp_err > SPACING_MAX:
            probs.append(f"z = {z:.2f}: splines unevenly spaced by {sp_err:.3f} deg (max {SPACING_MAX:g})")
        widths += [s["width"] for s in sp]
        minors.append(pr["minor"])
        majors.append(pr["major"])
        rows.append({"z": round(z, 3), "n": len(sp), "parallel": round(worst_par, 5), "symmetry": round(worst_sym, 5),
                     "spacing": round(sp_err, 5)})
    out.append(C.res("SPL_PROFILE", not probs and bool(rows), "; ".join(probs[:2]) or
                     f"{p['n']} straight-sided {'splines' if side == 'shaft' else 'slots'} at {len(rows)} stations, "
                     "flanks parallel, symmetric, evenly spaced", len(probs), "problems", stations=rows))

    def zone_result(name, label, vals, zn, code, words=("too small", "too large")):
        if not vals:
            return C.res(name, None, f"skipped: no profile measured")
        lo, hi = min(vals), max(vals)
        ok = zn[0] - 1e-6 <= lo and hi <= zn[1] + 1e-6
        return C.res(name, ok, C.zone_reason(inst, label, lo, hi, zn, code, words),
                     hi if hi > zn[1] else lo, "mm")
    codes = SPLINE_SLIDING[side]
    out.append(zone_result("SPL_WIDTH", "spline width" if side == "shaft" else "slot width", widths, zz["B"],
                           f"{p['B']:g} {codes['B']}", ("too narrow", "too wide")))
    out.append(zone_result("SPL_MINOR", "minor diameter", minors, zz["d"], f"{p['d']:g} {codes['d']}"))
    out.append(zone_result("SPL_MAJOR", "major diameter", majors, zz["D"], f"{p['D']:g} {codes['D']}"))
    # engagement: the z span over which the part's flank planes run, inside the counterpart's length
    fl = [(c, n, f) for c, n, f in C.planes(part) if abs(n[2]) < 1e-6]
    spans = []
    for c, n, f in fl:
        r = math.hypot(c[0], c[1])
        if p["d"] / 2 - 0.6 <= r <= p["D"] / 2 + 0.6:
            z0, z1 = C.face_z_range(f)
            spans.append((max(z0, cz[0]), min(z1, cz[1])))
    eng = min((b - a for a, b in spans), default=0.0) if spans else 0.0
    eng = max(0.0, eng)
    out.append(C.res("SPL_ENGAGEMENT", eng >= p["engage_min"] - 1e-6,
                     f"every flank spans {eng:.3f} mm of the given splines (need {p['engage_min']:g})", eng, "mm"))
    # insertion
    # until the part's lowest point is above the given splines; the given
    # splined length is at least 20 mm, so a 1 mm step cannot jump over it
    stroke = max(0.0, cz[1] - bb.min.Z) + 1.0
    n = int(math.ceil(stroke / 1.0))
    hit = None
    for i in range(n + 1):
        s = stroke * i / n
        v = C.overlap(C.moved_by(part, (0, 0, s)) if s else part, fixed)
        if v > C.OVERLAP_MAX:
            hit = (s, v)
            break
    out.append(C.res("SPL_INSERTION", hit is None,
                     f"slid out along +z over {stroke:.1f} mm in {n + 1} steps: nothing overlaps" if hit is None else
                     f"overlaps the given part by {hit[1]:.3f} mm3 when slid {hit[0]:.1f} mm out along +z",
                     0.0 if hit is None else hit[1], "mm3", steps=n + 1))
    return out


SELFTEST = [
    ("reference, shaft designed", {"knobs": {"design": "shaft"}}, None),
    ("reference, hub designed", {"knobs": {"design": "hub"}}, None),
    ("width just inside, narrow", {"part": {"B": "lo+0.001"}}, None),
    ("width just inside, wide", {"part": {"B": "hi-0.001"}}, None),
    ("width just too narrow", {"part": {"B": "lo-0.001"}}, "SPL_WIDTH"),
    ("width just too wide", {"part": {"B": "hi+0.001"}}, "SPL_WIDTH"),
    ("shaft: width just inside", {"knobs": {"design": "shaft"}, "part": {"B": "hi-0.001"}}, None),
    ("shaft: width just too wide", {"knobs": {"design": "shaft"}, "part": {"B": "hi+0.001"}}, "SPL_WIDTH"),
    ("shaft: minor just too small", {"knobs": {"design": "shaft"}, "part": {"d": "lo-0.001"}}, "SPL_MINOR"),
    ("shaft: collar at the leading end", {"knobs": {"design": "shaft"}, "part": {"collar": True}},
     "SPL_INSERTION"),
    ("minor diameter just inside", {"part": {"d": "lo+0.001"}}, None),
    ("minor diameter just outside", {"part": {"d": "hi+0.001"}}, "SPL_MINOR"),
    ("major diameter just outside", {"part": {"D": "lo-0.001"}}, "SPL_MAJOR"),
    ("one spline fewer", {"part": {"n": "n-1"}}, "SPL_PROFILE"),
    ("turned a quarter pitch", {"part": {"phase": "quarter"}}, "SPL_INSERTION"),
    ("engagement just enough", {"part": {"engage": 0.05}}, None),
    ("engagement just short", {"part": {"engage": -0.05}}, "SPL_ENGAGEMENT"),
    ("collar that cannot pass through", {"part": {"collar": True}}, "SPL_INSERTION"),
    ("splines 0.1 mm tighter at the leading end", {"part": {"taper": True}}, "SPL_INSERTION"),
]

_reference = reference


def reference(inst, **kw):  # noqa: F811
    p = inst.p
    if kw.get("n") == "n-1":
        kw["n"] = p["n"] - 1
    if kw.get("phase") == "quarter":
        kw["phase"] = 90.0 / p["n"]
    return _reference(inst, **kw)


TASK = """\
**Given:** a hub with an internal ISO 14 spline, or a splined shaft. **Design:** the mating part,
slid home. **What makes it hard:** the profile has three independent tolerance zones (minor, major,
width), set by the ISO 14 sliding-fit classes, and in the "rules not told" variant only the
designation (e.g. "6 x 23 x 26") is given, so the agent must know the spline width and the classes;
the splines must line up with the given ones at an arbitrary angle; and the part must be one that
can actually slide on: nothing on the leading side wider than the bore."""

KNOBS = {
    "design": "shaft | hub: which part the agent designs",
    "rules_told": "false: only the ISO 14 designation and 'sliding fit'",
    "aligned": "true: joint axis = world z (easier)",
}
