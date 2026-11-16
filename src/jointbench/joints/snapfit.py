"""Snap fit: a cantilever hook entering a window and catching behind its edge.

The given part is a plate (top face z = 0, bottom z = -T) with a rectangular
window. The agent designs a part that sits on the plate and holds itself with
one cantilever hook: the hook goes down through the window, bends inward
(towards -x) as its ramp passes the window edge at x = a, and springs back
so its lip catches under the plate.

Checks (overlap volumes and their extents along the bending direction; all
on exact booleans of the solids):

* SNAP_SEATED     seated, nothing overlaps the plate
* SNAP_INSERTION  moved back up along +z in 0.25 mm steps until clear, the
                  part never overlaps the plate by more than the stated
                  deflection, measured across the overlap along x
* SNAP_RETENTION  pulled up by the stated test distance, the overlap is at
                  least the stated engagement across x (the lip is held by
                  the plate), and the face it bears on is square to the pull
                  within 5 degrees (a sloped face would let it slide out)
* SNAP_STRAIN     information only: the bending strain from the standard
                  cantilever formula, eps = 1.5 t y / l^2, with t and l read
                  from the beam's two flat faces and y the measured
                  deflection; reported with how far the formula can be trusted
"""
from __future__ import annotations

import math

import build123d as bd

from . import common as C

NAME = "snapfit"
TITLE = "Snap fit (cantilever hook)"
STEP = 0.25  # mm, insertion steps
RETAIN_ANGLE_MAX = 5.0  # degrees from square


def generate(seed: int, rules_told: bool = True, aligned: bool = False) -> C.Instance:
    rng = C.rng_for(NAME, seed)
    T = rng.choice([2.0, 2.5, 3.0, 4.0])
    a = round(rng.uniform(6, 12), 1)  # window half-width in x
    wy = round(rng.uniform(5, 10), 1)  # window half-width in y
    e = round(rng.uniform(0.8, 1.5), 2)  # reference engagement
    t = round(rng.uniform(1.2, 2.0), 2)  # beam thickness
    l_free = round(rng.uniform(10, 18), 1)  # beam length above the plate top
    p = dict(T=T, a=a, wy=wy, plate_half=a + 20.0, e_ref=e, t_ref=t, l_ref=l_free,
             w=round(2 * wy - 3, 1), c=0.3, g=0.2, ramp=30.0,
             engage_min=round(e - 0.3, 2), deflect_max=round(e + 0.3, 2), pull=0.5)
    return C.Instance(NAME, seed, C.random_frame(rng, aligned), p, rules_told, {})


def fixed_local(inst) -> dict:
    p = inst.p
    h = p["plate_half"]
    plate = bd.Pos(0, 0, -p["T"] / 2) * bd.Box(2 * h, 2 * h, p["T"])
    plate = plate - bd.Pos(0, 0, -p["T"] / 2) * bd.Box(2 * p["a"], 2 * p["wy"], p["T"] + 2)
    return {"plate": plate}


def fixed_parts(inst) -> dict:
    return {k: C.to_world(v, inst.frame) for k, v in fixed_local(inst).items()}


def reference(inst, e=None, t=None, back_angle=0.0, sink=0.0, lip_low=0.0, ramp=None):
    """Body on the plate, a pocket in it, the beam hanging from the pocket's
    ceiling, the lip with its ramp at the bottom. ``e`` engagement, ``t`` beam
    thickness, ``back_angle`` slope of the retaining face (degrees), ``sink``
    body sunk into the plate, ``lip_low`` lip lowered by that much."""
    p = inst.p
    a, T, c, g = p["a"], p["T"], p["c"], p["g"]
    e = p["e_ref"] if e is None else e
    t = p["t_ref"] if t is None else t
    ramp = p["ramp"] if ramp is None else ramp
    hr = p["l_ref"]  # pocket ceiling height = beam root
    w = p["w"]
    x_out = a - c  # beam outer face
    x_in = x_out - t
    body_h = hr + 5.0
    body = bd.Pos(0, 0, body_h / 2 - sink) * bd.Box(2 * a + 20, 2 * p["wy"] + 16, body_h)
    pocket_x0, pocket_x1 = x_in - (p["deflect_max"] + 1.0), a + e + 2.0
    pocket = bd.Pos((pocket_x0 + pocket_x1) / 2, 0, hr / 2 - 1 - sink) * bd.Box(pocket_x1 - pocket_x0, w + 2, hr + 2)
    body = body - pocket
    z_top = -T - g - lip_low  # retaining face
    fh = 0.3  # flat on the lip's outer face
    z_ramp = z_top - fh
    z_tip = z_ramp - (c + e) / math.tan(math.radians(ramp))
    # beam + lip as one profile in the xz plane, extruded along y
    dz_back = (c + e) * math.tan(math.radians(back_angle))
    pts = [(x_in, hr - sink), (x_out, hr - sink), (x_out, z_top), (a + e, z_top - dz_back),
           (a + e, z_ramp - dz_back), (x_out, z_tip - dz_back), (x_in, z_tip - dz_back)]
    prof = bd.Polygon(*pts, align=None)
    hook = bd.extrude(bd.Plane.XZ * prof, w / 2, both=True)
    return body + hook


def brief(inst) -> str:
    p = inst.p
    L = [C.frame_text(inst.frame), ""]
    L.append(f"The given part is a plate {p['T']:g} mm thick (top face z = 0, bottom z = {-p['T']:g}) with a "
             f"rectangular window x = {-p['a']:g} to {p['a']:g}, y = {-p['wy']:g} to {p['wy']:g}.")
    L.append("Design one part that sits on the plate's top face and is held by a single cantilever hook through the "
             f"window that catches under the window's edge at x = {p['a']:g}. It is pushed on along -z; the hook bends "
             "towards -x as it passes the edge. Model it seated.")
    L.append(f"- Pushed on in {STEP:g} mm steps, the part may overlap the plate by at most {p['deflect_max']:g} mm "
             "across x (how far the hook must bend); seated, it overlaps nothing.")
    L.append(f"- Pulled back {p['pull']:g} mm along +z, the hook overlaps the plate by at least "
             f"{p['engage_min']:g} mm across x (the lip is caught), and the face it bears on is square to z within "
             f"{RETAIN_ANGLE_MAX:g} deg.")
    if inst.rules_told:
        L.append("- The bending strain of the hook (1.5 t y / l^2) is reported, not graded.")
    return "\n".join(L)


def _overlap_x(part, plate, dz):
    ov = C.overlap_shape(C.moved_by(part, (0, 0, dz)) if dz else part, plate)
    v = C.volume(ov) if ov is not None else 0.0
    if v <= C.OVERLAP_MAX:
        return v, 0.0, None
    ext = C.extent_along(ov, (1, 0, 0))
    return v, ext[1] - ext[0], ov


def check(solid, inst) -> list:
    p = inst.p
    part = C.to_local(solid, inst.frame)
    plate = fixed_local(inst)["plate"]
    out = []
    v, _, ov = _overlap_x(part, plate, 0.0)
    out.append(C.res("SNAP_SEATED", v <= C.OVERLAP_MAX, f"seated overlap {v:.4f} mm3 (limit {C.OVERLAP_MAX})", v, "mm3",
                     where=None if ov is None else tuple(ov.center())))
    # insertion: from seated, up until the part's lowest point clears the plate top
    bb = part.bounding_box()
    stroke = max(0.0, -bb.min.Z) + STEP
    n = int(math.ceil(stroke / STEP))
    worst, worst_z, worst_v = 0.0, 0.0, 0.0
    for i in range(1, n + 1):
        dz = stroke * i / n
        vv, dx, _ = _overlap_x(part, plate, dz)
        if dx > worst:
            worst, worst_z, worst_v = dx, dz, vv
    ok = worst <= p["deflect_max"] + 1e-6
    out.append(C.res("SNAP_INSERTION", ok, f"pushing on, the largest overlap across x is {worst:.3f} mm "
                     f"(max {p['deflect_max']:g}), {worst_z:.2f} mm before seated", worst, "mm",
                     where=(p["a"], 0, -worst_z), steps=n))
    # retention
    vr, dxr, ovr = _overlap_x(part, plate, p["pull"])
    probs = []
    if dxr < p["engage_min"] - 1e-6:
        probs.append(f"pulled back {p['pull']:g} mm the hook is caught over {dxr:.3f} mm across x "
                     f"(need {p['engage_min']:g})")
    ang = None
    if ovr is not None:
        # the overlap's upward faces come from the hook's retaining face (it
        # faces up, against the plate's underside)
        downs = [(f, n_) for _, n_, f in C.planes(ovr) if n_[2] > 0.05]
        curved = [f for f in ovr.faces() if f.geom_type != bd.GeomType.PLANE]
        if downs:
            f, n_ = max(downs, key=lambda fn: fn[0].area)
            ang = math.degrees(math.acos(min(1.0, n_[2])))
            if ang > RETAIN_ANGLE_MAX + 1e-6:
                probs.append(f"retaining face {ang:.2f} deg from square to the pull (max {RETAIN_ANGLE_MAX:g}): "
                             "it would slide out")
        elif curved:
            probs.append("retaining face is not flat")
    out.append(C.res("SNAP_RETENTION", not probs, "; ".join(probs) or
                     f"pulled back {p['pull']:g} mm, caught over {dxr:.3f} mm across x (need {p['engage_min']:g}), "
                     f"retaining face {ang:.2f} deg from square", dxr, "mm", angle=ang))
    out.append(_strain(part, worst))
    return out


def _strain(part, y):
    """Find the beam: two flat faces facing +x and -x, a thickness t apart,
    running side by side along z; l = their common length. Report
    eps = 1.5 t y / l^2 (rectangular cantilever, fixed at the root, load at the tip)."""
    pl = [(c, n, f) for c, n, f in C.planes(part) if abs(abs(n[0]) - 1) < 1e-6]
    best = None
    for c1, n1, f1 in pl:
        if n1[0] > 0:
            continue  # f1 faces -x: the beam's inner face
        for c2, n2, f2 in pl:
            if n2[0] < 0:
                continue  # f2 faces +x: the outer face
            t = c2[0] - c1[0]
            if t <= 0:
                continue
            z1, z2 = C.face_z_range(f1), C.face_z_range(f2)
            y1, y2 = _yr(f1), _yr(f2)
            lz = min(z1[1], z2[1]) - max(z1[0], z2[0])
            if lz <= 0 or min(y1[1], y2[1]) <= max(y1[0], y2[0]):
                continue
            if t > 0.5 * lz:
                continue  # not slender: not a beam
            root_match = abs(z1[1] - z2[1]) < 1e-3
            score = lz / t
            if best is None or score > best[0]:
                best = (score, t, lz, root_match)
    if best is None:
        return C.res("SNAP_STRAIN", True, "no slender flat-faced beam found: strain not estimated", None,
                     required=False, reliable=False)
    _, t, l, root = best
    eps = 1.5 * t * y / l ** 2
    note = ("straight beam of constant section with a square root: the formula holds to its own assumptions "
            "(rigid root, small deflection); a sharp root corner raises the real peak strain, a flexing root "
            "lowers it, neither is measured"
            if root else "the two beam faces end at different heights: root unclear, figure is rough")
    return C.res("SNAP_STRAIN", True, f"bending strain {100 * eps:.2f}% (t {t:.2f} mm, l {l:.2f} mm, y {y:.3f} mm); "
                 + note, eps, "", required=False, t=t, l=l, y=y, reliable=root)


def _yr(f):
    ys = [v.Y for v in f.vertices()]
    return min(ys), max(ys)


SELFTEST = [
    ("reference", {}, None),
    ("engagement just inside the most bending", {"part": {"e": "dmax-0.02"}}, None),
    ("engagement bends the hook too far", {"part": {"e": "dmax+0.02"}}, "SNAP_INSERTION"),
    ("engagement just enough", {"part": {"e": "emin+0.02"}}, None),
    ("engagement too small", {"part": {"e": "emin-0.02"}}, "SNAP_RETENTION"),
    ("retaining face sloped 4 deg", {"part": {"back_angle": 4.0}}, None),
    ("retaining face sloped 8 deg", {"part": {"back_angle": 8.0}}, "SNAP_RETENTION"),
    ("body sunk 0.05 mm into the plate", {"part": {"sink": 0.05}}, "SNAP_SEATED"),
    ("lip hangs 0.6 mm low: not caught when pulled", {"part": {"lip_low": 0.6}}, "SNAP_RETENTION"),
]

_reference = reference


def reference(inst, **kw):  # noqa: F811
    p = inst.p
    e = kw.get("e")
    if isinstance(e, str):
        base = p["deflect_max"] if e.startswith("dmax") else p["engage_min"]
        kw["e"] = base + float(e[4:])
    return _reference(inst, **kw)


TASK = """\
**Given:** a plate with a window. **Design:** a part that sits on the plate and holds itself with a
cantilever hook. **What makes it hard:** the lip's overhang must lie in a band a few tenths of a
millimetre wide (enough to catch, not so much the hook bends too far); the lip must sit just under
the plate with a small axial gap; the beam needs a pocket to bend into; and the retaining face must
be square, not sloped. A later knob can grade the strain against a stated material limit, which
asks the agent to size the beam (length and thickness) from the cantilever formula."""

KNOBS = {
    "strain_max": "(later) grade SNAP_STRAIN against a stated permissible strain, e.g. 2% for ABS",
    "two_hooks": "(later) two opposed hooks; the body is then located by both",
    "aligned": "true: plate top face = world xy (easier)",
}
