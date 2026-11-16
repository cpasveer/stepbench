"""Involute spline (30 deg, flat root, ISO 4156 proportions): the agent designs the shaft.

An involute flank is not a primitive surface: every CAD program stores it as
a B-spline. It can still be measured exactly, because one quantity is
constant along a true involute of base radius rb. A point at radius r and
polar angle phi lies on the involute whose "start angle" is

    c = phi + inv(a_r)   (one flank direction)   or   phi - inv(a_r)   (the other),
    inv(a) = tan a - a,  cos a_r = rb / r.

So sampling a flank in a section and computing c at every sample gives:
the flank's side (the sign whose c stays constant), its profile deviation
(spread of c times rb, which is the normal distance between the extreme
involutes), and the tooth thickness at the pitch circle from the two
flanks' c values. A B-spline fitted through 4 points of a true involute
already stays within 0.24 um of it after a STEP round trip (measured; see
docs/joints.md), so the stored approximation never decides a verdict.

Checks:

* INV_PROFILE    tooth count, every flank an involute of the stated base
                 circle within 0.005 mm, teeth evenly spaced, same at three stations
* INV_THICKNESS  tooth thickness (arc) at the pitch circle in the stated range
* INV_MAJOR      tip diameter in range
* INV_MINOR      root diameter in range
* INV_INSERTION  slides out along +z without touching the hub (as SPL_INSERTION)
"""
from __future__ import annotations

import math

import build123d as bd
from OCP.BRepAdaptor import BRepAdaptor_Curve
from OCP.BRepAlgoAPI import BRepAlgoAPI_Section
from OCP.GeomAbs import GeomAbs_Circle, GeomAbs_Line
from OCP.gp import gp_Dir, gp_Pln, gp_Pnt

from . import common as C

NAME = "involute"
TITLE = "Involute spline (30 deg flat root)"
ALPHA = 30.0
PROFILE_MAX = 0.005  # mm
SPACING_MAX = 0.02  # deg
MODULES = [1.0, 1.25, 1.5, 2.0]
TEETH = [14, 16, 18, 20, 22, 24, 26]


def inv(a):
    return math.tan(a) - a


def generate(seed: int, rules_told: bool = True, aligned: bool = False) -> C.Instance:
    rng = C.rng_for(NAME, seed)
    m, z = rng.choice(MODULES), rng.choice(TEETH)
    rp = m * z / 2
    s_basic = math.pi * m / 2
    Lh = round(max(20.0, 2 * rp * rng.uniform(0.8, 1.2)), 1)
    p = dict(m=m, z=z, alpha=ALPHA, rp=rp, rb=rp * math.cos(math.radians(ALPHA)),
             s=[round(s_basic - 0.06, 4), round(s_basic - 0.02, 4)],  # shaft tooth thickness at the pitch circle
             e_hub=round(s_basic + 0.02, 4),  # hub space width at the pitch circle, as modelled
             da=[round(m * (z + 1) - 0.1, 4), round(m * (z + 1), 4)],  # shaft major (tip) diameter
             df=[round(m * (z - 1.5) - 0.1, 4), round(m * (z - 1.5) + 0.05, 4)],  # shaft minor (root)
             hub_minor=m * (z - 1), hub_major=m * (z + 1.5), hub_len=Lh, hub_od=round(2 * rp * 1.5 + 10, 1),
             phase=round(rng.uniform(0, 360 / z), 3), engage_min=round(0.9 * Lh, 1))
    return C.Instance(NAME, seed, C.random_frame(rng, aligned), p, rules_told, {})


def _tooth(p, half_pitch_angle, r1, r2, centre, n=16):
    """A face bounded by two involute flanks (symmetric about ``centre`` deg),
    whose angular half-thickness at the pitch circle is ``half_pitch_angle``,
    from radius r1 to r2."""
    rb, rp = p["rb"], p["rp"]
    ap = math.acos(rb / rp)
    c0 = half_pitch_angle + inv(ap)  # + side: phi = c0 - inv(a_r)
    cen = math.radians(centre)

    def pt(r, sgn):
        a = math.acos(min(1.0, rb / r))
        ph = sgn * (c0 - inv(a)) + cen
        return bd.Vector(r * math.cos(ph), r * math.sin(ph), 0)
    rs = [r1 + (r2 - r1) * i / (n - 1) for i in range(n)]
    plus = bd.Edge.make_spline([pt(r, 1) for r in rs])
    minus = bd.Edge.make_spline([pt(r, -1) for r in reversed(rs)])
    top = bd.Edge.make_three_point_arc(pt(r2, 1), bd.Vector(r2 * math.cos(cen), r2 * math.sin(cen), 0), pt(r2, -1))
    bot = bd.Edge.make_three_point_arc(pt(r1, -1), bd.Vector(r1 * math.cos(cen), r1 * math.sin(cen), 0), pt(r1, 1))
    return bd.Face(bd.Wire([plus, top, minus, bot]))


def _prism(face, z0, z1):
    return bd.Pos(0, 0, z0) * bd.extrude(face, z1 - z0, dir=(0, 0, 1))


def shaft_solid(p, s, da, df, z0, z1, phase=0.0, n_teeth=None):
    teeth = [_prism(_tooth(p, s / (2 * p["rp"]), df / 2 - 0.2, da / 2, p["phase"] + phase + i * 360 / p["z"]), z0, z1)
             for i in range(n_teeth or p["z"])]
    core = bd.Pos(0, 0, (z0 + z1) / 2) * bd.Cylinder(df / 2, z1 - z0)
    return core.fuse(*teeth).clean()


def hub_solid(p, e, z0, z1):
    h = z1 - z0
    ring = bd.Pos(0, 0, (z0 + z1) / 2) * (bd.Cylinder(p["hub_od"] / 2, h) - bd.Cylinder(p["hub_minor"] / 2, h + 2))
    spaces = [_prism(_tooth(p, e / (2 * p["rp"]), p["hub_minor"] / 2 - 0.2, p["hub_major"] / 2,
                            p["phase"] + i * 360 / p["z"]), z0 - 1, z1 + 1) for i in range(p["z"])]
    return ring.cut(*spaces).clean()


def fixed_local(inst) -> dict:
    p = inst.p
    return {"hub": hub_solid(p, p["e_hub"], 0.0, p["hub_len"])}


def fixed_parts(inst) -> dict:
    return {k: C.to_world(v, inst.frame) for k, v in fixed_local(inst).items()}


def reference(inst, s=None, da=None, df=None, phase=0.0, n_teeth=None, collar=False):
    p = inst.p

    def pick(v, zz):
        if v is None:
            return 0.5 * (zz[0] + zz[1])
        if isinstance(v, str):
            return zz[0 if v.startswith("lo") else 1] + float(v[2:])
        return v
    if phase == "half":
        phase = 180.0 / p["z"]
    part = shaft_solid(p, pick(s, p["s"]), pick(da, p["da"]), pick(df, p["df"]), 0.0, p["hub_len"] + 10,
                       phase, n_teeth if n_teeth != "z-1" else p["z"] - 1)
    if collar:
        part = part + bd.Pos(0, 0, -1) * bd.Cylinder(p["hub_major"] / 2 + 1, 2)
    return part


def brief(inst) -> str:
    p = inst.p
    L = [C.frame_text(inst.frame), ""]
    L.append(f"The given part is a hub (z = 0 to {p['hub_len']:g}) with an internal involute spline: module {p['m']:g}, "
             f"{p['z']} teeth, pressure angle {ALPHA:g} deg, flat root; pitch diameter {2 * p['rp']:g}, base diameter "
             f"{2 * p['rb']:.6f}; space width at the pitch circle {p['e_hub']:.4f} mm; minor {p['hub_minor']:g}, major "
             f"{p['hub_major']:g} mm; one space centred on the direction {p['phase']:g} deg from +x.")
    L.append("Design one splined shaft that slides into it along z; model it slid home, spanning the hub.")
    L.append(f"- Every flank an involute of the base circle above within {PROFILE_MAX} mm; {p['z']} teeth evenly spaced "
             f"within {SPACING_MAX} deg.")
    L.append(f"- Tooth thickness at the pitch circle (arc) {p['s'][0]:.4f} to {p['s'][1]:.4f} mm; tip diameter "
             f"{p['da'][0]:.4f} to {p['da'][1]:.4f}; root diameter {p['df'][0]:.4f} to {p['df'][1]:.4f} mm.")
    L.append(f"- The profile spans at least {p['engage_min']:g} mm of the hub; slid out along +z it touches nothing.")
    return "\n".join(L)


def _section(solid, z):
    sec = BRepAlgoAPI_Section(solid.wrapped, gp_Pln(gp_Pnt(0, 0, z), gp_Dir(0, 0, 1)))
    sec.Build()
    return bd.Compound(sec.Shape()).edges()


def measure(solid, z, p, samples=13):
    rb = p["rb"]
    ap = math.acos(rb / p["rp"])
    flanks, tips, roots = [], [], []
    for e in _section(solid, z):
        t = BRepAdaptor_Curve(e.wrapped).GetType()
        if t == GeomAbs_Circle:
            r = math.hypot(*tuple(e.position_at(0.5))[:2])
            if r < p["rp"] * 1.3:
                (tips if r > p["rp"] else roots).append(r)
            continue
        if t == GeomAbs_Line:
            continue
        ad = BRepAdaptor_Curve(e.wrapped)
        u0, u1 = ad.FirstParameter(), ad.LastParameter()
        pts = []
        for k in range(samples):
            q = ad.Value(u0 + (u1 - u0) * k / (samples - 1))
            pts.append((q.X(), q.Y(), q.Z()))
        rs = [math.hypot(x, y) for x, y, _ in pts]
        if min(rs) < rb or max(rs) > p["rp"] * 1.3:
            continue
        phis = [math.atan2(y, x) for x, y, _ in pts]
        phis = [phis[0] + ((ph - phis[0] + math.pi) % (2 * math.pi) - math.pi) for ph in phis]
        cp = [ph + inv(math.acos(rb / r)) for ph, r in zip(phis, rs)]
        cm = [ph - inv(math.acos(rb / r)) for ph, r in zip(phis, rs)]
        sp, sm = max(cp) - min(cp), max(cm) - min(cm)
        side, c, spread = (1, sum(cp) / len(cp), sp) if sp < sm else (-1, sum(cm) / len(cm), sm)
        flanks.append({"side": side, "c": c, "dev": spread * rb})
    # pair each "+" flank (phi = c+ - inv) with the "-" flank on its other side: a tooth
    plus = sorted((f for f in flanks if f["side"] == 1), key=lambda f: f["c"] % (2 * math.pi))
    minus = [f for f in flanks if f["side"] == -1]
    teeth = []
    for fp in plus:
        # the tooth's "-" flank has c- = centre - (half + inv ap); nearest below fp's c+ in angle
        best = None
        for fm in minus:
            d = (fp["c"] - fm["c"]) % (2 * math.pi)
            if best is None or d < best[0]:
                best = (d, fm)
        if best is None:
            continue
        d, fm = best
        thick = p["rp"] * (d - 2 * inv(ap))
        centre = math.degrees(fp["c"] - d / 2)
        teeth.append({"thickness": thick, "centre": centre, "dev": max(fp["dev"], fm["dev"])})
    return {"teeth": teeth, "n_flanks": len(flanks), "tip": tips, "root": roots}


def check(solid, inst) -> list:
    p = inst.p
    part = C.to_local(solid, inst.frame)
    hub = fixed_local(inst)["hub"]
    out = []
    bb = part.bounding_box()
    s0, s1 = max(0.0, bb.min.Z), min(p["hub_len"], bb.max.Z)
    stations = [s0 + (s1 - s0) * f for f in (0.1, 0.5, 0.9)] if s1 > s0 else []
    probs, th, tips, roots, devs = [], [], [], [], []
    for z in stations:
        mz = measure(part, z, p)
        t = mz["teeth"]
        if len(t) != p["z"] or mz["n_flanks"] != 2 * p["z"]:
            probs.append(f"z = {z:.2f}: {len(t)} involute teeth, {mz['n_flanks']} flanks (need {p['z']}, {2 * p['z']})")
        if t:
            dev = max(x["dev"] for x in t)
            devs.append(dev)
            if dev > PROFILE_MAX:
                probs.append(f"z = {z:.2f}: a flank strays {dev:.4f} mm from the involute (max {PROFILE_MAX})")
            angs = sorted(x["centre"] % 360 for x in t)
            pitch = 360 / p["z"]
            err = max(abs(((a - angs[0]) / pitch - round((a - angs[0]) / pitch)) * pitch) for a in angs)
            if err > SPACING_MAX:
                probs.append(f"z = {z:.2f}: teeth unevenly spaced by {err:.3f} deg")
            th += [x["thickness"] for x in t]
        tips += mz["tip"]
        roots += mz["root"]
    out.append(C.res("INV_PROFILE", not probs and bool(devs), "; ".join(probs[:2]) or
                     f"{p['z']} involute teeth at {len(stations)} stations, worst profile deviation "
                     f"{max(devs) * 1000:.2f} um (max {PROFILE_MAX * 1000:g})", max(devs) if devs else None, "mm"))

    def rng(name, label, vals, zz):
        if not vals:
            return C.res(name, False, f"no {label} measured")
        lo, hi = min(vals), max(vals)
        return C.res(name, zz[0] - 1e-6 <= lo and hi <= zz[1] + 1e-6,
                     f"{label} {lo:.4f} to {hi:.4f} mm (need {zz[0]:.4f} to {zz[1]:.4f})", lo, "mm")
    out.append(rng("INV_THICKNESS", "tooth thickness at the pitch circle", th, p["s"]))
    out.append(rng("INV_MAJOR", "tip diameter", [2 * r for r in tips], p["da"]))
    out.append(rng("INV_MINOR", "root diameter", [2 * r for r in roots], p["df"]))
    # 2 mm steps: anything that must pass through the hub is inside it for at
    # least the hub's length (20 mm or more), so no step can jump over it
    stroke = max(0.0, p["hub_len"] - bb.min.Z) + 1.0
    n = int(math.ceil(stroke / 2.0))
    hit = None
    for i in range(n + 1):
        sz = stroke * i / n
        v = C.overlap(C.moved_by(part, (0, 0, sz)) if sz else part, hub)
        if v > C.OVERLAP_MAX:
            hit = (sz, v)
            break
    out.append(C.res("INV_INSERTION", hit is None, "slides out along +z without touching" if hit is None else
                     f"overlaps the hub by {hit[1]:.3f} mm3 when slid {hit[0]:.1f} mm out", 0 if hit is None else hit[1],
                     "mm3"))
    return out


SELFTEST = [
    ("reference", {}, None),
    ("thickness just inside, thin", {"part": {"s": "lo+0.001"}}, None),
    ("thickness just inside, thick", {"part": {"s": "hi-0.001"}}, None),
    ("thickness just too thin", {"part": {"s": "lo-0.001"}}, "INV_THICKNESS"),
    ("thickness just too thick", {"part": {"s": "hi+0.001"}}, "INV_THICKNESS"),
    ("tip diameter just too large", {"part": {"da": "hi+0.002"}}, "INV_MAJOR"),
    ("root diameter just too small", {"part": {"df": "lo-0.002"}}, "INV_MINOR"),
    ("one tooth missing", {"part": {"n_teeth": "z-1"}}, "INV_PROFILE"),
    ("turned half a pitch", {"part": {"phase": "half"}}, "INV_INSERTION"),
    ("collar at the leading end", {"part": {"collar": True}}, "INV_INSERTION"),
]

TASK = """\
**Given:** a hub with an internal 30 deg involute spline. **Design:** the shaft. **What makes it
hard:** an involute flank has to be built from its definition (base circle, involute function), not
drawn; the tooth thickness is set at the pitch circle, where no edge of the model lies; and the
teeth must line up with the hub's spaces. Not built yet: ISO 4156's own tolerance formulas
(effective and actual space width, fit classes), which are stated here as plain numbers."""

KNOBS = {"aligned": "true: axis = world z", "later": "hub designed; ISO 4156 tolerance classes from the formulas"}
