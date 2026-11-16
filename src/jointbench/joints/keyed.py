"""Keyed joint: shaft and hub, each with a keyway, holding one parallel key.

The key is the DIN 6885-1 form A parallel key (rounded ends) of the standard
section for the shaft diameter; its section b x h is the one of ISO/R 773.
Either side may be given (with its keyway); the agent designs the other.
Fits: shaft keyway width N9, hub keyway width JS9 (the "normal" fit), bore
H7 on a shaft h6.

Keyway walls and floor are exact planes, so the check reads them directly:

* KEY_SEAT     the bore (or shaft) on the joint axis, diameter in its ISO zone
* KEY_WIDTH    keyway width (distance between its two side walls) in N9 / JS9
* KEY_DEPTH    keyway depth: shaft floor at d/2 - t1, hub roof at d/2 + t2
               (DIN 6885 depth, tolerance +0.1 up to h = 6, +0.2 above)
* KEY_ALIGN    the keyway's centre plane is the given keyway's: parallel within
               0.1 deg, offset at most 0.02 mm
* KEY_LENGTH   a form A key of the stated length has room (shaft keyway's
               straight part at least L - b) and the hub covers it
* KEY_FITS     a form A key of the standard section, at the small end of its
               own tolerance (width h9, height h11), laid on the shaft keyway
               floor and centred in it, overlaps neither part
* KEY_OVERLAP  shaft and hub do not overlap
"""
from __future__ import annotations

import math

import build123d as bd

from . import common as C
from .standards import KEY_LENGTHS, it, key_for, mid, zone

NAME = "keyed"
TITLE = "Keyed joint (DIN 6885 parallel key)"
DIAMETERS = [12, 15, 16, 20, 22, 25, 28, 30, 32, 35, 40, 45, 50, 55, 60, 70, 80]
# the keyway's centre plane may stand at most this far from the given one,
# anywhere across the keyway (offset plus tilt times the roof radius); it is
# set below the smallest side clearance of the normal fit (IT9 / 2 = 0.015 mm
# for a 4 mm key), so an aligned keyway never pinches the key
ALIGN_MAX = 0.01  # mm


def generate(seed: int, design: str | None = None, rules_told: bool = True, aligned: bool = False) -> C.Instance:
    rng = C.rng_for(NAME, seed)
    d = float(rng.choice(DIAMETERS))
    k = key_for(d)
    d_draw = rng.choice(["hub", "shaft"])
    design = design or d_draw
    L = min((x for x in KEY_LENGTHS if x >= 1.2 * d), default=KEY_LENGTHS[-1])
    hub_len = float(L + rng.choice([4, 6, 8, 10]))
    phi = round(rng.uniform(0, 360), 3)  # keyway direction in the joint frame
    zc = hub_len / 2  # key centre along z
    p = dict(d=d, b=k.b, h=k.h, t1=k.t1, t2=k.t2, depth_tol=k.depth_tol, key_len=float(L),
             design=design, hub_len=hub_len, hub_od=round(1.8 * d + 12, 1), phi=phi, zc=zc,
             shaft_w=list(zone(k.b, "N9")), hub_w=list(zone(k.b, "JS9")),
             bore=list(zone(d, "H7")), shaft_d=list(zone(d, "h6")))
    return C.Instance(NAME, seed, C.random_frame(rng, aligned), p, rules_told, {"design": design})


# ------------------------------------------------------------- geometry


def _shaft(p, dia, width, t1, length, phi, zc, offset=0.0):
    z0, z1 = -15.0, p["hub_len"] + 15.0
    # the cylinder's seam turned away from the keyway: a seam running into a
    # slightly shifted keyway leaves a tiny edge the quality rules reject
    s = bd.Pos(0, 0, (z0 + z1) / 2) * bd.Rot(0, 0, 180) * bd.Cylinder(dia / 2, z1 - z0)
    # form A keyway: straight part length - width, round ends
    floor = d_floor = dia / 2 - t1  # noqa: F841
    depth = dia / 2 - d_floor + 2.0
    xc = d_floor + depth / 2
    slot = bd.Pos(xc, offset, zc) * bd.Box(depth, width, length - width)
    for e in (-1, 1):
        slot = slot + bd.Pos(xc, offset, zc + e * (length - width) / 2) * \
            bd.Rot(0, 90, 0) * bd.Cylinder(width / 2, depth)
    return C.rotated_z(s - slot, phi)


def _hub(p, bore, width, t2, phi, offset=0.0, z0=0.0, length=None):
    Lh = p["hub_len"] if length is None else length
    hub = bd.Pos(0, 0, z0 + Lh / 2) * bd.Cylinder(p["hub_od"] / 2, Lh)
    hub = hub - bd.Pos(0, 0, z0 + Lh / 2) * bd.Rot(0, 0, 180) * bd.Cylinder(bore / 2, Lh + 2)
    roof = p["d"] / 2 + t2
    xc = roof / 2
    slot = bd.Pos(xc, offset, z0 + Lh / 2) * bd.Box(roof, width, Lh + 2)
    return C.rotated_z(hub - slot, phi)


def fixed_local(inst) -> dict:
    p = inst.p
    if p["design"] == "hub":  # the shaft is given
        return {"shaft": _shaft(p, mid(p["shaft_d"]), mid(p["shaft_w"]), p["t1"] + p["depth_tol"] / 2,
                                p["key_len"], p["phi"], p["zc"])}
    return {"hub": _hub(p, mid(p["bore"]), mid(p["hub_w"]), p["t2"] + p["depth_tol"] / 2, p["phi"])}


def fixed_parts(inst) -> dict:
    return {k: C.to_world(v, inst.frame) for k, v in fixed_local(inst).items()}


def reference(inst, width=None, depth=None, phi=None, offset=0.0, length=None, dia=None, hub_len=None):
    p = inst.p
    # phi: the keyway turned so its centre plane moves this many mm at the roof radius
    if phi == "quarter":
        ph = p["phi"] + 90.0
    else:
        ph = p["phi"] if phi is None else p["phi"] + math.degrees(math.asin(phi / (p["d"] / 2 + p["t2"])))
    if p["design"] == "hub":
        w = mid(p["hub_w"]) if width is None else _sym(width, p["hub_w"])
        t2 = p["t2"] + p["depth_tol"] / 2 if depth is None else _sym(depth, (p["t2"], p["t2"] + p["depth_tol"]))
        bore = mid(p["bore"]) if dia is None else _sym(dia, p["bore"])
        if hub_len is not None:  # a short hub, centred on the key
            Lh = p["key_len"] + float(hub_len)
            return _hub(p, bore, w, t2, ph, offset, z0=p["zc"] - Lh / 2, length=Lh)
        return _hub(p, bore, w, t2, ph, offset)
    w = mid(p["shaft_w"]) if width is None else _sym(width, p["shaft_w"])
    t1 = p["t1"] + p["depth_tol"] / 2 if depth is None else _sym(depth, (p["t1"], p["t1"] + p["depth_tol"]))
    sd = mid(p["shaft_d"]) if dia is None else _sym(dia, p["shaft_d"])
    L = p["key_len"] if length is None else p["key_len"] + float(length)
    return _shaft(p, sd, w, t1, L, ph, p["zc"], offset)


def _sym(v, z):
    """'lo+0.001' / 'hi-0.001' relative to a zone, or a plain number."""
    if isinstance(v, str):
        return z[0 if v.startswith("lo") else 1] + float(v[2:])
    return v


# ---------------------------------------------------------------- brief


def brief(inst) -> str:
    p = inst.p
    L = [C.frame_text(inst.frame), ""]
    if p["design"] == "hub":
        L.append(f"The given part is a shaft of diameter {mid(p['shaft_d']):.4f} mm on the z axis "
                 f"(z = -15 to {p['hub_len'] + 15:g}) with a form A keyway for a {p['b']:g} x {p['h']:g} key, "
                 f"{p['key_len']:g} mm long, centred at z = {p['zc']:g}; its centre plane contains the z axis and "
                 f"points from the axis in direction {p['phi']:g} deg from +x (towards +y).")
        L.append(f"Design one hub, {p['hub_len']:g} mm long or longer, on this shaft with a keyway for the same key.")
    else:
        L.append(f"The given part is a hub (z = 0 to {p['hub_len']:g}) with a bore of diameter "
                 f"{mid(p['bore']):.4f} mm on the z axis and a through keyway for a {p['b']:g} x {p['h']:g} key; "
                 f"its centre plane contains the z axis and points in direction {p['phi']:g} deg from +x.")
        L.append(f"Design one shaft through this hub with a keyway for a form A key {p['key_len']:g} mm long, "
                 f"lying inside the hub.")
    if inst.rules_told:
        if p["design"] == "hub":
            L.append(f"- Bore {p['d']:g} H7: {p['bore'][0]:.4f} to {p['bore'][1]:.4f} mm, on the z axis.")
            L.append(f"- Keyway width JS9: {p['hub_w'][0]:.4f} to {p['hub_w'][1]:.4f} mm; roof at "
                     f"{p['d'] / 2 + p['t2']:.3f} to {p['d'] / 2 + p['t2'] + p['depth_tol']:.3f} mm from the axis "
                     f"(t2 = {p['t2']:g} +{p['depth_tol']:g}, DIN 6885-1).")
        else:
            L.append(f"- Shaft {p['d']:g} h6: {p['shaft_d'][0]:.4f} to {p['shaft_d'][1]:.4f} mm, on the z axis.")
            L.append(f"- Keyway width N9: {p['shaft_w'][0]:.4f} to {p['shaft_w'][1]:.4f} mm; floor at "
                     f"{p['d'] / 2 - p['t1'] - p['depth_tol']:.3f} to {p['d'] / 2 - p['t1']:.3f} mm from the axis "
                     f"(t1 = {p['t1']:g} +{p['depth_tol']:g}, DIN 6885-1); straight part at least "
                     f"{p['key_len'] - p['b']:g} mm.")
    else:
        L.append(f"- Shaft {p['d']:g} mm, bore H7 on shaft h6, one DIN 6885-1 form A parallel key of the standard "
                 "section for this diameter, normal fit (shaft keyway N9, hub keyway JS9).")
    L.append(f"- The keyway's centre plane is the given one's: nowhere across the keyway (out to "
             f"{p['d'] / 2 + p['t2']:.3f} mm from the axis) more than {ALIGN_MAX:g} mm from it.")
    L.append("- A key of the standard section at the small end of its tolerance (width h9, height h11), "
             "laid on the shaft keyway floor, must fit both parts; shaft and hub must not overlap.")
    return "\n".join(L)


# ---------------------------------------------------------------- check


def find_keyway(solid, d, b, internal):
    """The keyway of a shaft (internal=False) or hub (True) in the joint frame.

    Side walls: two planes parallel to z, facing each other across a gap of
    about b. Floor (shaft) or roof (hub): a plane parallel to z between them,
    facing into the slot. Returns dict(width, u, v, mid, floor, z0, z1) with u
    the wall normal, v the direction from the axis into the slot, ``mid`` the
    signed offset of the centre plane from the axis, ``floor`` the distance of
    floor / roof from the axis, and the straight walls' z range."""
    pl = [(c, n, f) for c, n, f in C.planes(solid) if abs(n[2]) < 1e-6]
    best = None
    for i, (c1, n1, f1) in enumerate(pl):
        for c2, n2, f2 in pl[i + 1:]:
            if sum(a * b_ for a, b_ in zip(n1, n2)) > -1 + 1e-9:
                continue  # not antiparallel
            gap = sum((c2[k] - c1[k]) * n1[k] for k in range(3))
            if gap <= 0 or abs(gap - b) > 0.25 * b:
                continue  # walls of a slot face each other: f1's normal points to f2
            mid_off = 0.5 * (sum(c1[k] * n1[k] for k in range(3)) + sum(c2[k] * n1[k] for k in range(3)))
            if abs(mid_off) > 0.5:
                continue
            u = n1
            # v: perpendicular to u and z, pointing to where the walls are
            v = (-u[1], u[0], 0.0)
            cm = [(c1[k] + c2[k]) / 2 for k in range(3)]
            if cm[0] * v[0] + cm[1] * v[1] < 0:
                v = (-v[0], -v[1], 0.0)
            za = C.face_z_range(f1)
            zb = C.face_z_range(f2)
            score = abs(gap - b) + abs(mid_off)
            if best is None or score < best[0]:
                best = (score, dict(width=gap, u=u, v=v, mid=mid_off, z0=max(za[0], zb[0]),
                                    z1=min(za[1], zb[1]), walls=(f1, f2)))
    if best is None:
        return None
    kw = best[1]
    v = kw["v"]
    want = tuple(-x for x in v) if internal else v  # floor faces out of the shaft; roof faces the axis
    floors = []
    for c, n, f in pl:
        if sum(n[k] * want[k] for k in range(3)) < 1 - 1e-6:
            continue
        lat = sum(c[k] * kw["u"][k] for k in range(3)) - kw["mid"]
        if abs(lat) > kw["width"]:
            continue
        floors.append(sum(c[k] * v[k] for k in range(3)))
    if not floors:
        kw["floor"] = None
    else:  # nearest to the bore surface
        kw["floor"] = min(floors, key=lambda r: abs(r - d / 2))
    return kw


def key_solid(p, kw_shaft, width, height):
    """A form A key in the shaft keyway: bottom on the floor, centred on its centre plane and straight part."""
    u, v = kw_shaft["u"], kw_shaft["v"]
    L = p["key_len"]
    zc = 0.5 * (kw_shaft["z0"] + kw_shaft["z1"])
    r0 = kw_shaft["floor"]
    body = bd.Pos(r0 + height / 2, 0, zc) * bd.Box(height, width, L - width)
    for e in (-1, 1):
        body = body + bd.Pos(r0 + height / 2, 0, zc + e * (L - width) / 2) * bd.Rot(0, 90, 0) * \
            bd.Cylinder(width / 2, height)
    ang = math.degrees(math.atan2(v[1], v[0]))
    # in the local key frame +x is v and +y is ... shift sideways onto the centre plane
    body = C.rotated_z(body, ang)
    side = kw_shaft["mid"]
    return C.moved_by(body, (u[0] * side, u[1] * side, 0.0))


def check(solid, inst) -> list:
    p = inst.p
    part = C.to_local(solid, inst.frame)
    fname, fixed = next(iter(fixed_local(inst).items()))
    shaft, hub = (fixed, part) if p["design"] == "hub" else (part, fixed)
    designing_hub = p["design"] == "hub"
    out = []
    # seat
    g, _ = C.coaxial_group(C.cylinders(part), p["d"] / 2, convex=not designing_hub)
    zn = p["bore"] if designing_hub else p["shaft_d"]
    code = "H7" if designing_hub else "h6"
    what = "bore" if designing_hub else "shaft"
    if g is None:
        out.append(C.res("KEY_SEAT", False, f"no {what} of about {p['d']:g} mm on the joint axis"))
    else:
        dia = 2 * g["r"]
        ok = (g["offset"] <= C.AXIS_OFFSET_MAX and g["angle"] <= C.AXIS_ANGLE_MAX
              and zn[0] - 1e-6 <= dia <= zn[1] + 1e-6)
        out.append(C.res("KEY_SEAT", ok, C.zone_reason(inst, what, dia, dia, zn, f"{p['d']:g} {code}") + ", "
                         f"axis {g['offset']:.4f} mm off", dia, "mm"))
    # keyways
    kw = find_keyway(part, p["d"], p["b"], internal=designing_hub)
    kg = find_keyway(fixed, p["d"], p["b"], internal=not designing_hub)
    if kw is None or kw["floor"] is None:
        msg = f"no keyway of about {p['b']:g} mm width found in the {'hub' if designing_hub else 'shaft'}"
        out.append(C.res("KEY_WIDTH", False, msg))
        for n in ("KEY_DEPTH", "KEY_ALIGN", "KEY_LENGTH", "KEY_FITS"):
            out.append(C.res(n, None, "skipped: " + msg))
    else:
        wz = p["hub_w"] if designing_hub else p["shaft_w"]
        wc = "JS9" if designing_hub else "N9"
        ok = wz[0] - 1e-6 <= kw["width"] <= wz[1] + 1e-6
        out.append(C.res("KEY_WIDTH", ok, C.zone_reason(inst, "keyway width", kw["width"], kw["width"], wz,
                                                        f"{p['b']:g} {wc}", ("too narrow", "too wide")),
                         kw["width"], "mm"))
        if designing_hub:
            t, nom, lab = kw["floor"] - p["d"] / 2, p["t2"], "t2 (hub, roof above the bore)"
        else:
            t, nom, lab = p["d"] / 2 - kw["floor"], p["t1"], "t1 (shaft, floor below the surface)"
        ok = nom - 1e-6 <= t <= nom + p["depth_tol"] + 1e-6
        out.append(C.res("KEY_DEPTH", ok, C.zone_reason(inst, f"keyway depth {lab}", t, t,
                                                        (nom, nom + p["depth_tol"]), "DIN 6885-1",
                                                        ("too shallow", "too deep")), t, "mm"))
        ang = math.degrees(math.acos(max(-1.0, min(1.0, sum(kw["v"][k] * kg["v"][k] for k in range(3))))))
        # offset of the centre planes, measured along the given keyway's wall normal
        u = kg["u"]
        sign = 1.0 if sum(kw["u"][k] * u[k] for k in range(3)) > 0 else -1.0
        off = abs(sign * kw["mid"] - kg["mid"])
        r_out = p["d"] / 2 + p["t2"]
        dev = off + r_out * math.sin(math.radians(ang))
        ok = dev <= ALIGN_MAX + 1e-9
        out.append(C.res("KEY_ALIGN", ok, f"keyway centre plane up to {dev:.4f} mm from the given one (max {ALIGN_MAX:g}): "
                         f"offset {off:.4f} mm, turned {ang:.4f} deg", dev, "mm", offset=off, angle=ang))
        ks = kg if designing_hub else kw  # the shaft's keyway carries the key
        straight = ks["z1"] - ks["z0"]
        need = p["key_len"] - p["b"]
        zc = 0.5 * (ks["z0"] + ks["z1"])
        k0, k1 = zc - p["key_len"] / 2, zc + p["key_len"] / 2
        hz = C.face_z_range(kw["walls"][0]) if designing_hub else C.face_z_range(kg["walls"][0])
        probs = []
        if straight < need - 1e-6:
            probs.append(f"shaft keyway straight part {straight:.3f} mm, a {p['key_len']:g} mm key needs {need:g}")
        if hz[0] > k0 + 1e-6 or hz[1] < k1 - 1e-6:
            probs.append(f"hub keyway z {hz[0]:.2f} to {hz[1]:.2f} does not cover the key, z {k0:.2f} to {k1:.2f}")
        out.append(C.res("KEY_LENGTH", not probs, "; ".join(probs) or
                         f"room for a {p['key_len']:g} mm form A key (straight part {straight:.3f} mm), "
                         "hub covers it", straight, "mm"))
        bmin = p["b"] - it(p["b"], 9)
        hmin = p["h"] - it(p["h"], 11)
        key = key_solid(p, ks, bmin, hmin)
        v1, v2 = C.overlap(key, shaft), C.overlap(key, hub)
        worst = max(v1, v2)
        ok = worst <= C.OVERLAP_MAX
        out.append(C.res("KEY_FITS", ok,
                         (f"a {bmin:.3f} x {hmin:.3f} x {p['key_len']:g} key" if inst.rules_told
                          else f"the standard {p['key_len']:g} mm key")  # level 3: its section is not told
                         + f" overlaps the shaft by {v1:.4f} mm3 and "
                         f"the hub by {v2:.4f} mm3 (limit {C.OVERLAP_MAX})", worst, "mm3",
                         where=tuple(key.center())))
    v = C.overlap(shaft, hub)
    out.append(C.res("KEY_OVERLAP", v <= C.OVERLAP_MAX, f"shaft and hub overlap {v:.4f} mm3 (limit {C.OVERLAP_MAX})",
                     v, "mm3"))
    return out


SELFTEST = [
    ("reference, hub designed", {"knobs": {"design": "hub"}}, None),
    ("reference, shaft designed", {"knobs": {"design": "shaft"}}, None),
    ("width just inside, narrow", {"part": {"width": "lo+0.001"}}, None),
    ("width just inside, wide", {"part": {"width": "hi-0.001"}}, None),
    ("width just too narrow", {"part": {"width": "lo-0.001"}}, "KEY_WIDTH"),
    ("width just too wide", {"part": {"width": "hi+0.001"}}, "KEY_WIDTH"),
    ("depth just inside, shallow", {"part": {"depth": "lo+0.005"}}, None),
    ("depth just too shallow", {"part": {"depth": "lo-0.005"}}, "KEY_DEPTH"),
    ("depth just too deep", {"part": {"depth": "hi+0.005"}}, "KEY_DEPTH"),
    ("keyway turned 0.008 mm at the roof", {"part": {"phi": 0.008}}, None),
    ("keyway turned 0.012 mm at the roof", {"part": {"phi": 0.012}}, "KEY_ALIGN"),
    ("keyway a quarter turn out", {"part": {"phi": "quarter"}}, "KEY_FITS"),
    ("keyway 0.008 mm to the side", {"part": {"offset": 0.008}}, None),
    ("keyway 0.012 mm to the side", {"part": {"offset": 0.012}}, "KEY_ALIGN"),
    ("shaft keyway 0.1 mm too short", {"knobs": {"design": "shaft"}, "part": {"length": -0.1}}, "KEY_LENGTH"),
    ("hub 0.2 mm shorter than the key", {"knobs": {"design": "hub"}, "part": {"hub_len": -0.2}}, "KEY_LENGTH"),
    ("bore just too small", {"knobs": {"design": "hub"}, "part": {"dia": "lo-0.001"}}, "KEY_SEAT"),
]

TASK = """\
**Given:** a shaft with a form A keyway, or a hub with a through keyway. **Design:** the other
part, with its own keyway for the same key. **What makes it hard:** with the rules not told, the
brief names only the shaft diameter and "DIN 6885 form A, normal fit": the agent must know the key
section (b x h), both keyway depths t1 and t2, and the N9 / JS9 zones, then place the keyway
exactly on the given one's centre plane at an arbitrary angle. A keyway 90 degrees out is the
typical modelling slip; it fails as "the key does not fit"."""

KNOBS = {
    "design": "hub | shaft: which part the agent designs",
    "rules_told": "false: only the diameter and the standard are named",
    "aligned": "true: joint axis = world z (easier)",
}
