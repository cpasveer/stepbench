"""Interference fit: a shaft pressed into a hub.

Either side may be given. Given a hub, the agent designs the shaft; given a
shaft, the agent designs the hub. The fit is an ISO 286 press fit (H7/p6,
H7/r6 or H7/s6); the brief states it as the fit and as numbers, or, with
``rules_told=False``, as the fit alone, so the agent must know the zone.

Checks, all on exact cylinders, cones and boolean volumes:

* FIT_COAXIAL    the fitted cylinder lies on the joint axis
* FIT_SIZE       its diameter lies inside the ISO zone of the agent's side
                 (shaft p6/r6/s6, or hub bore H7); the interference with the
                 given part as modelled is reported
* FIT_ENGAGEMENT the fitted cylinder overlaps the counterpart's by at least
                 the stated length
* FIT_LEAD_IN    a coaxial cone at the leading end of the fitted surface,
                 half-angle in range, long enough, starting clear of the
                 counterpart's diameter
* FIT_OVERLAP    the usual no-overlap rule, with one declared exception: the
                 thin ring between the counterpart's surface and the far
                 edge of the agent's tolerance zone, along the counterpart's
                 fitted length. Any overlap outside that ring fails.
"""
from __future__ import annotations

import math

import build123d as bd

from . import common as C
from .standards import mid, zone

NAME = "interference"
TITLE = "Interference fit (shaft pressed into a hub)"
FITS = ["p6", "r6", "s6"]
DIAMETERS = [10, 12, 16, 20, 25, 30, 35, 40, 50, 60]
LEAD_ANGLE = (5.0, 30.0)  # cone half-angle from the axis, degrees
LEAD_LEN_MIN = 1.0  # mm, along the axis
LEAD_START = 0.05  # mm: the cone's small end is at least this far inside the counterpart's diameter


def generate(seed: int, given: str | None = None, rules_told: bool = True, aligned: bool = False) -> C.Instance:
    rng = C.rng_for(NAME, seed)
    D = float(rng.choice(DIAMETERS))
    s = rng.choice(FITS)
    g_draw = rng.choice(["hub", "shaft"])
    given = given or g_draw
    hz, sz = zone(D, "H7"), zone(D, s)
    Lh = round(max(12.0, D * rng.uniform(0.8, 1.4)), 1)
    p = dict(D=D, fit=f"H7/{s}", given=given, hole_zone=list(hz), shaft_zone=list(sz),
             hub_len=Lh, hub_od=round(D * 1.8 + 10, 1), engage_min=round(0.75 * Lh, 1))
    if given == "hub":
        p["given_d"] = mid(hz)
        p["agent_zone"] = list(sz)
        p["interference"] = [sz[0] - p["given_d"], sz[1] - p["given_d"]]
    else:
        p["given_d"] = mid(sz)
        p["agent_zone"] = list(hz)
        p["interference"] = [p["given_d"] - hz[1], p["given_d"] - hz[0]]
    return C.Instance(NAME, seed, C.random_frame(rng, aligned), p, rules_told, {"given": given})


def fixed_local(inst) -> dict:
    p = inst.p
    if p["given"] == "hub":
        hub = bd.Pos(0, 0, p["hub_len"] / 2) * bd.Cylinder(p["hub_od"] / 2, p["hub_len"])
        hub = hub - bd.Pos(0, 0, p["hub_len"] / 2) * bd.Cylinder(p["given_d"] / 2, p["hub_len"] + 2)
        return {"hub": hub}
    # given shaft: a stub from z = -20 up to the hub length + 5, on a flange
    top = p["hub_len"] + 5.0
    shaft = bd.Pos(0, 0, (top - 20) / 2) * bd.Cylinder(p["given_d"] / 2, top + 20)
    flange = bd.Pos(0, 0, -23) * bd.Cylinder(p["D"] + 10, 6)
    return {"shaft": shaft + flange}


def fixed_parts(inst) -> dict:
    return {k: C.to_world(v, inst.frame) for k, v in fixed_local(inst).items()}


def counterpart_z(p):
    """Axial extent of the given part's fitted surface."""
    return (0.0, p["hub_len"]) if p["given"] == "hub" else (-20.0, p["hub_len"] + 5.0)


def reference(inst, dia=None, offset=0.0, engage=None, lead=True, lead_angle=15.0,
              lead_len=None, shoulder_sink=0.0):
    p = inst.p
    d = mid(p["agent_zone"]) if dia is None else dia
    if isinstance(d, str):
        d = p["agent_zone"][0 if d.startswith("lo") else 1] + float(d[2:])
    Lh = p["hub_len"]
    eng = Lh if engage is None else (p["engage_min"] + float(engage[3:]) if isinstance(engage, str) else engage)
    ll = 2.0 if lead_len is None else lead_len
    if p["given"] == "hub":
        # shaft: fitted from z = Lh - eng up (the lead-in below that); a shoulder
        # sits on the hub's top face. Pressed home, the lead-in pokes out below.
        z0 = Lh - eng - (ll if lead else 0.0)
        # runs 6 mm past the shoulder: a shorter stub fails the quality rule Q_STUB
        fit = bd.Pos(0, 0, (z0 + Lh + 12) / 2) * bd.Cylinder(d / 2, Lh + 12 - z0)
        if lead:
            dr = ll * math.tan(math.radians(lead_angle))
            cone = bd.Solid.make_cone(d / 2 - dr, d / 2, ll, bd.Plane((0, 0, z0)))
            fit = (fit - bd.Pos(0, 0, z0 + ll / 2) * bd.Cylinder(d / 2 + 1, ll)) + cone
        shoulder = bd.Pos(0, 0, Lh + 3 - shoulder_sink) * bd.Cylinder(d / 2 + 4, 6)
        part = fit + shoulder
    else:
        # hub: bore fitted over [z0, z0 + Lh]; pressed down onto the shaft from above
        z0 = p["hub_len"] + 5.0 - eng - (ll if lead else 0.0) if engage is not None else 0.0
        hub = bd.Pos(0, 0, z0 + Lh / 2) * bd.Cylinder(p["hub_od"] / 2, Lh)
        hub = hub - bd.Pos(0, 0, z0 + Lh / 2) * bd.Cylinder(d / 2, Lh + 2)
        if lead:
            dr = ll * math.tan(math.radians(lead_angle))
            cone = bd.Solid.make_cone(d / 2 + dr, d / 2, ll, bd.Plane((0, 0, z0)))
            hub = hub - cone
        part = hub
    if offset:
        part = C.moved_by(part, (offset, 0, 0))
    return part


def brief(inst) -> str:
    p = inst.p
    L = [C.frame_text(inst.frame), ""]
    if p["given"] == "hub":
        L.append(f"The given part is a hub (outside diameter {p['hub_od']:g} mm, z = 0 to {p['hub_len']:g}) "
                 f"with a bore of diameter {p['given_d']:.4f} mm on the z axis.")
        L.append("Design one shaft pressed into it; it is pressed in moving along -z, so its leading end is its "
                 "lowest end. Model it pressed home.")
        side = "shaft"
    else:
        L.append(f"The given part is a shaft of diameter {p['given_d']:.4f} mm on the z axis, "
                 f"its free end at z = {p['hub_len'] + 5:g} (it stands on a flange below z = -20).")
        L.append("Design one hub pressed onto it; it is pressed on moving along -z, so the leading end of its bore "
                 "is its lowest end. Model it pressed home.")
        side = "bore"
    if inst.rules_told:
        a, b = p["agent_zone"]
        L.append(f"- Press fit {p['D']:g} {p['fit']} (ISO 286): your {side} diameter {a:.4f} to {b:.4f} mm, "
                 f"an interference of {p['interference'][0]:.4f} to {p['interference'][1]:.4f} mm with the "
                 "given part as modelled.")
    else:
        L.append(f"- Press fit {p['D']:g} {p['fit']} (ISO 286).")
    L.append(f"- The {side} is on the z axis within {C.AXIS_OFFSET_MAX} mm and parallel within {C.AXIS_ANGLE_MAX} deg, "
             f"engaged over at least {p['engage_min']:g} mm along z.")
    L.append(f"- A lead-in chamfer at the leading end of the fitted {side}: a cone on the axis, half-angle "
             f"{LEAD_ANGLE[0]:g} to {LEAD_ANGLE[1]:g} deg from the axis, at least {LEAD_LEN_MIN:g} mm long along z, "
             f"whose entering edge clears the given part's diameter by at least {LEAD_START:g} mm.")
    L.append("- Overlap with the given part is allowed only in the fitted ring itself: between the given surface and "
             "the far edge of your tolerance zone, along the given part's fitted length. Anywhere else, nothing "
             "may overlap (0.01 mm3).")
    return "\n".join(L)


def exception_zone(p):
    """The one region where the shaft and hub may share space."""
    z0, z1 = counterpart_z(p)
    a, b = p["agent_zone"]
    if p["given"] == "hub":  # shaft larger than the bore: ring from bore out to the largest shaft
        r_in, r_out = p["given_d"] / 2, b / 2 + 1e-4
    else:  # bore smaller than the shaft: ring from the smallest bore out to the shaft
        r_in, r_out = a / 2 - 1e-4, p["given_d"] / 2
    if r_out <= r_in:
        return None
    ring = bd.Pos(0, 0, (z0 + z1) / 2) * bd.Cylinder(r_out, z1 - z0)
    return ring - bd.Pos(0, 0, (z0 + z1) / 2) * bd.Cylinder(r_in, z1 - z0 + 2)


def check(solid, inst) -> list:
    p = inst.p
    part = C.to_local(solid, inst.frame)
    fixed = next(iter(fixed_local(inst).values()))
    convex = p["given"] == "hub"  # the agent's fitted surface is a shaft
    side = "shaft" if convex else "bore"
    cyls = C.cylinders(part)
    g, _ = C.coaxial_group(cyls, p["D"] / 2, convex=convex)
    out = []
    if g is None:
        msg = f"no {side} of about {p['D']:g} mm on the joint axis"
        out += [C.res("FIT_COAXIAL", False, msg)] + \
               [C.res(n, None, "skipped: " + msg) for n in ("FIT_SIZE", "FIT_ENGAGEMENT", "FIT_LEAD_IN")]
    else:
        ok = g["offset"] <= C.AXIS_OFFSET_MAX and g["angle"] <= C.AXIS_ANGLE_MAX
        out.append(C.res("FIT_COAXIAL", ok, f"{side} axis {g['offset']:.4f} mm off the joint axis "
                         f"(max {C.AXIS_OFFSET_MAX}), tilted {g['angle']:.4f} deg", g["offset"], "mm"))
        d = 2 * g["r"]
        a, b = p["agent_zone"]
        inter = (d - p["given_d"]) if convex else (p["given_d"] - d)
        code = p["fit"].split("/")[1 if convex else 0]
        ok = a - 1e-6 <= d <= b + 1e-6
        out.append(C.res("FIT_SIZE", ok, C.zone_reason(inst, side, d, d, (a, b), f"{p['D']:g} {code}") + "; "
                         f"interference with the given part {inter * 1000:.1f} um", d, "mm", interference=inter))
        cz = counterpart_z(p)
        faces = [c for c in cyls if c.convex == convex and abs(c.r - g["r"]) < 1e-6
                 and abs(c.offset - g["offset"]) < 1e-5]
        eng = C.union_length((max(c.z0, cz[0]), min(c.z1, cz[1])) for c in faces)
        out.append(C.res("FIT_ENGAGEMENT", eng >= p["engage_min"] - 1e-6,
                         f"engaged {eng:.3f} mm along z (need {p['engage_min']:g})", eng, "mm"))
        out.append(_lead_in(part, p, g, convex, side))
    # overlap, with the declared exception
    ov = C.overlap_shape(part, fixed)
    v_all = C.volume(ov) if ov is not None else 0.0
    zone_solid = exception_zone(p)
    excess = v_all
    if ov is not None and v_all > 0 and zone_solid is not None:
        excess = C.volume(ov - zone_solid)
    ok = excess <= C.OVERLAP_MAX
    where = None
    if not ok:
        rest = ov - zone_solid if zone_solid is not None else ov
        where = tuple(rest.center())
    out.append(C.res("FIT_OVERLAP", ok,
                     f"overlap {v_all:.3f} mm3 in all, {excess:.4f} mm3 outside the declared fit ring "
                     f"(limit {C.OVERLAP_MAX})", excess, "mm3", where=where, total=v_all))
    return out


def _lead_in(part, p, g, convex, side):
    """A coaxial cone that continues the fitted cylinder at its leading (lowest) end."""
    r, z_lead = g["r"], g["z0"]
    best = None
    for apex, ax, semi, f in C.cones(part):
        # coaxial: axis parallel to z through the joint axis
        if abs(abs(ax[2]) - 1) > 1e-6 or C.axis_offset(apex, ax) > C.AXIS_OFFSET_MAX + 1e-6:
            continue
        z0, z1 = C.face_z_range(f)
        if abs(z1 - z_lead) > 1e-4:  # its top edge must meet the cylinder's lowest edge
            continue
        # radii at both ends from the cone geometry
        r_top = abs(apex[2] - z1) * math.tan(math.radians(semi))
        r_bot = abs(apex[2] - z0) * math.tan(math.radians(semi))
        if abs(r_top - r) > 1e-3:
            continue
        best = (semi, z1 - z0, r_bot)
    if best is None:
        return C.res("FIT_LEAD_IN", False, f"no lead-in cone at the leading end of the {side} (z = {z_lead:.3f})",
                     where=(r, 0, z_lead))
    semi, length, r_bot = best
    clear = (p["given_d"] / 2 - r_bot) if convex else (r_bot - p["given_d"] / 2)
    probs = []
    if not (LEAD_ANGLE[0] - 1e-6 <= semi <= LEAD_ANGLE[1] + 1e-6):
        probs.append(f"half-angle {semi:.2f} deg outside {LEAD_ANGLE[0]:g}-{LEAD_ANGLE[1]:g}")
    if length < LEAD_LEN_MIN - 1e-6:
        probs.append(f"length {length:.3f} mm under {LEAD_LEN_MIN:g}")
    if clear < LEAD_START - 1e-6:
        probs.append(f"entering edge clears the given diameter by {clear:.3f} mm (need {LEAD_START:g})")
    reason = (f"lead-in cone {semi:.2f} deg, {length:.3f} mm long, entering edge {clear:.3f} mm clear"
              if not probs else "lead-in: " + "; ".join(probs))
    return C.res("FIT_LEAD_IN", not probs, reason, semi, "deg", where=(r, 0, z_lead), length=length)


SELFTEST = [
    ("reference, hub given", {"knobs": {"given": "hub"}}, None),
    ("reference, shaft given", {"knobs": {"given": "shaft"}}, None),
    ("diameter just inside, low end", {"part": {"dia": "lo+0.0005"}}, None),
    ("diameter just inside, high end", {"part": {"dia": "hi-0.0005"}}, None),
    ("diameter just below the zone", {"part": {"dia": "lo-0.0005"}}, "FIT_SIZE"),
    ("diameter just above the zone", {"part": {"dia": "hi+0.0005"}}, "FIT_SIZE"),
    ("axis 0.012 mm off", {"part": {"offset": 0.012}}, "FIT_COAXIAL"),
    ("engagement just enough", {"part": {"engage": "min+0.05"}}, None),
    ("engagement just short", {"part": {"engage": "min-0.05"}}, "FIT_ENGAGEMENT"),
    ("no lead-in", {"part": {"lead": False}}, "FIT_LEAD_IN"),
    ("lead-in 29 deg", {"part": {"lead_angle": 29.0}}, None),
    ("lead-in 31 deg", {"part": {"lead_angle": 31.0}}, "FIT_LEAD_IN"),
    ("lead-in 1.05 mm long", {"part": {"lead_len": 1.05}}, None),
    ("lead-in 0.95 mm long", {"part": {"lead_len": 0.95}}, "FIT_LEAD_IN"),
    ("shoulder sunk 0.05 mm into the hub", {"knobs": {"given": "hub"}, "part": {"shoulder_sink": 0.05}},
     "FIT_OVERLAP"),
    ("oversize beyond the zone overlaps", {"knobs": {"given": "hub"}, "part": {"dia": "hi+0.01"}}, "FIT_OVERLAP"),
]

TASK = """\
**Given:** a hub with a bore, or a shaft standing on a flange. **Design:** the mating shaft or hub,
pressed home. **What makes it hard:** the size is set in micrometres from an ISO fit, and in the
"rules not told" variant only the fit name is given, so the agent must know that 25 p6 means
+22 to +35 um; the lead-in has to sit at the leading end in the direction of pressing; and the part
must overlap the counterpart in the fitted ring and nowhere else (a shoulder that sits on the hub
must sit on it, not in it)."""

KNOBS = {
    "given": "hub | shaft: which side is given",
    "rules_told": "false: only the fit name (H7/p6, H7/r6, H7/s6), not the numbers",
    "aligned": "true: joint axis = world z (easier)",
}
