"""Revolute joint: a lever that turns on a pin, or a pin that turns in a bore.

The given part is a plate with either a bore (the agent's part carries the
pin) or a standing pin (the agent's part carries the bore). The agent's part
is a lever: the running surface on the joint axis, and an arm with a drive
hole at a stated radius. A lug on the plate limits the turning range: the arm
must pass it nowhere inside the stated range.

Checks, all on exact cylinders and exact boolean volumes:

* REV_COAXIAL   the running cylinder lies on the joint axis
* REV_CLEARANCE diametral clearance bore - pin within the stated range
* REV_BEARING   the running surface overlaps the counterpart's for at least
                the stated bearing length
* REV_RETAINED  retained: the part cannot move more than the stated axial play
                either way; free end: it lifts off along +z without touching
* REV_DRIVE     the drive hole: diameter, parallel to the axis, at the radius
* REV_ROTATION  turned through the whole range in steps, nothing overlaps
"""
from __future__ import annotations

import math

import build123d as bd

from . import common as C
from .standards import hole_zone, mid, shaft_zone

NAME = "revolute"
TITLE = "Revolute joint (pin in a bore)"
FITS = [("H7", "f7"), ("H7", "g6"), ("H8", "f7"), ("H8", "e8")]
DIAMETERS = [6, 8, 10, 12, 16, 20, 25]
STEP_ARC = 1.5  # mm: the rotation step is chosen so the farthest point moves at most this


def _fit_zones(d, fit):
    h, s = fit
    return hole_zone(d, h[0], int(h[1:])), shaft_zone(d, s[0], int(s[1:]))


def generate(seed: int, given: str | None = None, retained: bool | None = None,
             rules_told: bool = True, aligned: bool = False) -> C.Instance:
    rng = C.rng_for(NAME, seed)
    d = rng.choice(DIAMETERS)
    fit = rng.choice(FITS)
    # every draw is made whatever the knobs say, so a knob changes only itself
    g_draw, r_draw = rng.choice(["hole", "pin"]), rng.random() < 0.6
    given = given or g_draw
    retained = r_draw if retained is None else retained
    hz, sz = _fit_zones(d, fit)
    # the given side is modelled at the middle of its own zone
    given_d = mid(hz) if given == "hole" else mid(sz)
    if given == "hole":  # agent's pin must be in the shaft zone
        agent_zone = sz
        clr = (given_d - sz[1], given_d - sz[0])
    else:
        agent_zone = hz
        clr = (hz[0] - given_d, hz[1] - given_d)
    L = round(d * rng.uniform(1.0, 1.6), 1)  # plate thickness / pin length
    bearing_min = round(0.8 * L, 1)
    play = 0.2
    play_max = 0.3
    R = round(rng.uniform(2.2, 3.5) * d + 10, 1)  # drive hole radius
    dd = rng.choice([4.0, 5.0, 6.0])
    hi = float(rng.randint(40, 120))
    lo = -float(rng.randint(30, 120))
    p = dict(d=float(d), fit=f"{fit[0]}/{fit[1]}", given=given, retained=retained,
             given_d=given_d, agent_zone=list(agent_zone), clearance=list(clr),
             L=L, bearing_min=bearing_min, play=play, play_max=play_max,
             R=R, drive_d=dd, rot_lo=lo, rot_hi=hi,
             arm_t=round(max(4.0, 0.4 * d), 1), arm_r=dd / 2 + 3.0, hub_r=d / 2 + 4.0)
    # the lug sits in the middle of the forbidden arc
    p["lug_angle"] = (hi + 360.0 + lo) / 2.0
    p["plate_half"] = R + p["arm_r"] + 8.0
    return C.Instance(NAME, seed, C.random_frame(rng, aligned), p, rules_told,
                      {"given": given, "retained": retained})


# --------------------------------------------------------------- geometry


def _lug(p, z0):
    r1 = p["R"] - p["arm_r"] - 2.0
    r2 = p["R"] + p["arm_r"] + 2.0
    h = p["play"] + p["arm_t"] + 2.0
    lug = bd.Pos((r1 + r2) / 2, 0, z0 + h / 2) * bd.Box(r2 - r1, 6.0, h)
    return C.rotated_z(lug, p["lug_angle"])


def fixed_local(inst) -> dict:
    p = inst.p
    hp = p["plate_half"]
    if p["given"] == "hole":
        plate = bd.Pos(0, 0, p["L"] / 2) * bd.Box(2 * hp, 2 * hp, p["L"])
        plate = plate - bd.Pos(0, 0, p["L"] / 2) * bd.Cylinder(p["given_d"] / 2, p["L"] + 2)
        part = plate + _lug(p, p["L"])
    else:
        tb = 6.0
        plate = bd.Pos(0, 0, -tb / 2) * bd.Box(2 * hp, 2 * hp, tb)
        pin = bd.Pos(0, 0, p["L"] / 2 - 0.5) * bd.Cylinder(p["given_d"] / 2, p["L"] + 1.0)
        part = plate + pin
        if p["retained"]:
            part = part + bd.Pos(0, 0, p["L"] + 1.5) * bd.Cylinder(p["d"] / 2 + 3.0, 3.0)
        part = part + _lug(p, 0.0)
    return {"base": part}


def fixed_parts(inst) -> dict:
    return {k: C.to_world(v, inst.frame) for k, v in fixed_local(inst).items()}


def _arm(p, z0, t, angle=0.0):
    R, ra = p["R"], p["arm_r"]
    bar = bd.Pos(R / 2, 0, z0 + t / 2) * bd.Box(R, 2 * ra, t)
    end = bd.Pos(R, 0, z0 + t / 2) * bd.Cylinder(ra, t)
    hub = bd.Pos(0, 0, z0 + t / 2) * bd.Cylinder(p["hub_r"], t)
    arm = bar + end + hub
    arm = arm - bd.Pos(R, 0, z0 + t / 2) * bd.Cylinder(p["drive_d"] / 2, t + 2)
    return C.rotated_z(arm, angle) if angle else arm


def reference(inst, pin_d=None, offset=0.0, bearing=None, play=None, collar=None,
              arm_angle=0.0, drive_r=None):
    """The answer, and the knobs the self-test breaks it with."""
    p = dict(inst.p)
    if drive_r is not None:
        p["R"] = drive_r
    gap = p["play"] if play is None else play
    run_d = pin_d if pin_d is not None else mid(p["agent_zone"])
    L, t = p["L"], p["arm_t"]
    collar = p["retained"] if collar is None else collar
    if p["given"] == "hole":
        top = L + gap
        bot = -(gap + 3.0) if collar else -1.0
        pin = bd.Pos(0, 0, (top + bot) / 2) * bd.Cylinder(run_d / 2, top - bot)
        if bearing is not None:  # a neck from the pin's lower end: full diameter only over `bearing` mm
            z_top = L - bearing
            h = z_top - (bot - 1)
            zc = (z_top + bot - 1) / 2
            pin = pin - (bd.Pos(0, 0, zc) * bd.Cylinder(run_d / 2 + 1, h)
                         - bd.Pos(0, 0, zc) * bd.Cylinder(run_d / 2 - 0.5, h))
        part = pin + _arm(p, top, t)
        if collar:
            part = part + bd.Pos(0, 0, -gap - 1.5) * bd.Cylinder(p["d"] / 2 + 3.0, 3.0)
    else:
        z0 = gap
        z1 = L - gap if p["retained"] else L - 1.0
        hub = bd.Pos(0, 0, (z0 + z1) / 2) * bd.Cylinder(p["hub_r"], z1 - z0)
        part = hub + _arm(p, z0, t)
        part = part - bd.Pos(0, 0, (z0 + z1) / 2) * bd.Cylinder(run_d / 2, z1 - z0 + 2)
        if bearing is not None:  # counterbore the bore so only `bearing` mm runs
            cb = (z1 - z0) - bearing
            part = part - bd.Pos(0, 0, z1 - cb / 2) * bd.Cylinder(run_d / 2 + 0.5, cb + 0.001)
        if collar and not p["retained"]:  # a lip that catches nothing, but blocks lifting off
            pass
    if arm_angle:
        part = C.rotated_z(part, arm_angle)
    if offset:
        part = C.moved_by(part, (offset, 0, 0))
    return part


# ----------------------------------------------------------------- brief


def brief(inst) -> str:
    p = inst.p
    lines = [C.frame_text(inst.frame), ""]
    if p["given"] == "hole":
        lines.append(f"The given part is a plate {p['L']:g} mm thick (z = 0 to {p['L']:g}) with a bore "
                     f"of diameter {p['given_d']:.4f} mm on the z axis, and a lug standing on its top face.")
        lines.append("Design one lever that turns in this bore: a pin on the z axis running in the bore, "
                     f"and an arm above the plate.")
    else:
        lines.append(f"The given part is a base plate (top face z = 0) with a pin of diameter "
                     f"{p['given_d']:.4f} mm on the z axis, from z = 0 to {p['L']:g}"
                     + (", capped by a head above z = " + f"{p['L']:g}" if p["retained"] else "")
                     + ", and a lug standing on the plate.")
        lines.append("Design one lever that turns on this pin: a bore on the z axis running on the pin, "
                     "and an arm.")
    lo, hi = p["clearance"]
    if inst.rules_told:
        lines.append(f"- Running fit {p['fit']}: diametral clearance between bore and pin "
                     f"{lo:.4f} to {hi:.4f} mm, so your {'pin' if p['given'] == 'hole' else 'bore'} "
                     f"diameter is {p['agent_zone'][0]:.4f} to {p['agent_zone'][1]:.4f} mm.")
    else:
        lines.append(f"- Running fit {p['fit']} (ISO 286) between bore and pin.")
    lines.append(f"- Running surface on the z axis within {C.AXIS_OFFSET_MAX} mm and parallel within "
                 f"{C.AXIS_ANGLE_MAX} degrees; it overlaps the counterpart's for at least "
                 f"{p['bearing_min']:g} mm along z.")
    if p["retained"]:
        lines.append(f"- Retained: the lever may move along z by at most {p['play_max']:g} mm either way "
                     "before it touches the given part (model it as assembled).")
    else:
        lines.append("- Free end: the lever lifts off along +z without touching anything.")
    lines.append(f"- Drive hole: diameter {p['drive_d']:g} to {p['drive_d'] + 0.1:g} mm, parallel to z, "
                 f"its axis at radius {p['R']:g} mm (within 0.05) on the +x axis in the position you model.")
    lines.append(f"- It turns about the z axis from {p['rot_lo']:g} to +{p['rot_hi']:g} degrees "
                 "(positive is counter-clockwise seen from +z) without touching the given part; "
                 "nothing may overlap it in the modelled position either.")
    return "\n".join(lines)


# ----------------------------------------------------------------- check


def _translate_play(part, fixed, direction, limit, lim_vol=C.OVERLAP_MAX):
    """Largest move along ``direction`` (up to ``limit``) without overlap,
    by bisection to 0.001 mm; ``limit`` if it never touches."""
    if C.overlap(C.moved_by(part, [c * limit for c in direction]), fixed) <= lim_vol:
        return limit
    a, b = 0.0, limit
    while b - a > 0.001:
        m = 0.5 * (a + b)
        if C.overlap(C.moved_by(part, [c * m for c in direction]), fixed) <= lim_vol:
            a = m
        else:
            b = m
    return a


def check(solid, inst, step_arc: float = STEP_ARC) -> list:
    p = inst.p
    part = C.to_local(solid, inst.frame)
    fixed = fixed_local(inst)["base"]
    out = []
    want_convex = p["given"] == "hole"
    cyls = C.cylinders(part)
    run, _ = C.coaxial_group(cyls, p["d"] / 2, convex=want_convex)
    what = "pin" if want_convex else "bore"
    if run is None:
        msg = f"no {what} of about {p['d']:g} mm diameter on the joint axis"
        out.append(C.res("REV_COAXIAL", False, msg))
        out.append(C.res("REV_CLEARANCE", None, "skipped: " + msg))
        out.append(C.res("REV_BEARING", None, "skipped: " + msg))
    else:
        ok = run["offset"] <= C.AXIS_OFFSET_MAX and run["angle"] <= C.AXIS_ANGLE_MAX
        out.append(C.res("REV_COAXIAL", ok,
                         f"{what} axis {run['offset']:.4f} mm off the joint axis (max {C.AXIS_OFFSET_MAX}), "
                         f"tilted {run['angle']:.4f} deg (max {C.AXIS_ANGLE_MAX})",
                         run["offset"], "mm", where=(0, 0, (run["z0"] + run["z1"]) / 2)))
        dia = 2 * run["r"]
        clr = (p["given_d"] - dia) if want_convex else (dia - p["given_d"])
        lo, hi = p["clearance"]
        ok = lo - 1e-6 <= clr <= hi + 1e-6
        out.append(C.res("REV_CLEARANCE", ok,
                         f"{what} {dia:.4f} mm, "
                         + C.zone_reason(inst, "clearance", clr, clr, (lo, hi), p["fit"], ("too tight", "too loose")),
                         clr, "mm", diameter=dia))
        # counterpart's running surface along z
        cz = (0.0, p["L"])
        faces = [c for c in cyls if c.convex == want_convex and abs(c.r - run["r"]) < 1e-6
                 and abs(c.offset - run["offset"]) < 1e-5]
        brg = C.union_length((max(c.z0, cz[0]), min(c.z1, cz[1])) for c in faces)
        ok = brg >= p["bearing_min"] - 1e-6
        out.append(C.res("REV_BEARING", ok,
                         f"bearing length {brg:.3f} mm where the {what} meets the counterpart "
                         f"(need {p['bearing_min']:g})", brg, "mm"))
    # retention
    if p["retained"]:
        up = _translate_play(part, fixed, (0, 0, 1), 5.0)
        dn = _translate_play(part, fixed, (0, 0, -1), 5.0)
        worst = max(up, dn)
        ok = worst <= p["play_max"] + 1e-6
        out.append(C.res("REV_RETAINED", ok,
                         f"axial play {up:.3f} mm up, {dn:.3f} mm down (max {p['play_max']:g} each way)",
                         worst, "mm", up=up, down=dn))
    else:
        lift = p["L"] + 5.0
        steps = max(4, int(math.ceil(lift / 1.0)))
        hit = None
        for i in range(1, steps + 1):
            s = lift * i / steps
            v = C.overlap(C.moved_by(part, (0, 0, s)), fixed)
            if v > C.OVERLAP_MAX:
                hit = (s, v)
                break
        out.append(C.res("REV_RETAINED", hit is None,
                         "lifts off along +z without touching" if hit is None else
                         f"lifting off along +z it hits the given part after {hit[0]:.2f} mm "
                         f"({hit[1]:.3f} mm3)", 0.0 if hit is None else hit[0], "mm"))
    # drive hole
    best = None
    for c in cyls:
        if c.convex or c.angle > C.AXIS_ANGLE_MAX * 10:
            continue
        if abs(2 * c.r - p["drive_d"]) > 1.0:
            continue
        rr = c.offset  # distance from joint axis
        t = -c.loc[2] / c.axis[2]
        x, y = c.loc[0] + t * c.axis[0], c.loc[1] + t * c.axis[1]
        err = math.hypot(x - p["R"], y)
        if best is None or err < best[0]:
            best = (err, 2 * c.r, rr, (x, y))
    if best is None:
        out.append(C.res("REV_DRIVE", False, f"no hole of about {p['drive_d']:g} mm parallel to the axis"))
    else:
        err, dia, rr, xy = best
        ok = err <= 0.05 + 1e-9 and p["drive_d"] - 1e-6 <= dia <= p["drive_d"] + 0.1 + 1e-6
        out.append(C.res("REV_DRIVE", ok,
                         f"drive hole {dia:.3f} mm at ({xy[0]:.3f}, {xy[1]:.3f}), "
                         f"{err:.4f} mm from ({p['R']:g}, 0) (max 0.05)", err, "mm",
                         where=(xy[0], xy[1], 0)))
    # rotation through the range
    bb = part.bounding_box()
    rmax = max(math.hypot(x, y) for x in (bb.min.X, bb.max.X) for y in (bb.min.Y, bb.max.Y))
    stepdeg = min(2.0, math.degrees(step_arc / max(rmax, 1e-6)))
    lo, hi = p["rot_lo"], p["rot_hi"]
    n = int(math.ceil((hi - lo) / stepdeg))
    angles = sorted({0.0} | {lo + (hi - lo) * i / n for i in range(n + 1)})
    worst, worst_a = 0.0, 0.0
    for a in angles:
        v = C.overlap(C.rotated_z(part, a) if a else part, fixed)
        if v > worst:
            worst, worst_a = v, a
        if v > C.OVERLAP_MAX and a != 0.0:
            break
    ok = worst <= C.OVERLAP_MAX
    out.append(C.res("REV_ROTATION", ok,
                     f"turned {lo:g} to +{hi:g} deg in {len(angles)} steps of {stepdeg:.2f} deg: "
                     + ("nothing overlaps" if ok else f"overlaps the given part by {worst:.3f} mm3 at {worst_a:.1f} deg"),
                     worst, "mm3", angle=worst_a, steps=len(angles)))
    return out


# ------------------------------------------------------------- self-test

def _z(inst, i):
    return inst.p["agent_zone"][i]


SELFTEST = [
    ("reference, pin given", {"knobs": {"given": "pin"}}, None),
    ("reference, bore given", {"knobs": {"given": "hole"}}, None),
    ("reference, free end", {"knobs": {"retained": False}}, None),
    ("running dia just inside (bore given)", {"knobs": {"given": "hole"}, "part": {"pin_d": "lo+"}}, None),
    ("running dia just too small", {"knobs": {"given": "hole"}, "part": {"pin_d": "lo-"}}, "REV_CLEARANCE"),
    ("running dia just too large", {"knobs": {"given": "pin"}, "part": {"pin_d": "hi+"}}, "REV_CLEARANCE"),
    ("axis 0.005 mm off", {"part": {"offset": 0.005}}, None),
    ("axis 0.012 mm off", {"part": {"offset": 0.012}}, "REV_COAXIAL"),
    ("bearing just long enough", {"part": {"bearing": "min+"}}, None),
    ("bearing just too short", {"part": {"bearing": "min-"}}, "REV_BEARING"),
    ("axial play just inside", {"knobs": {"retained": True}, "part": {"play": 0.29}}, None),
    ("axial play just too much", {"knobs": {"retained": True}, "part": {"play": 0.31}}, "REV_RETAINED"),
    ("free end blocked by a collar", {"knobs": {"retained": False, "given": "hole"}, "part": {"collar": True}},
     "REV_RETAINED"),
    ("drive hole 0.04 mm off", {"part": {"drive_r": "R+0.04"}}, None),
    ("drive hole 0.06 mm off", {"part": {"drive_r": "R+0.06"}}, "REV_DRIVE"),
    ("arm turned to hit the lug", {"part": {"arm_angle": "lug"}}, "REV_ROTATION"),
]


def _resolve(inst, kw):
    """Turn the self-test's symbolic knobs into numbers for this instance."""
    p, out = inst.p, dict(kw)
    z = p["agent_zone"]
    if out.get("pin_d") == "lo+":
        out["pin_d"] = z[0] + 0.0005
    elif out.get("pin_d") == "lo-":
        out["pin_d"] = z[0] - 0.0005 if p["given"] == "hole" else z[0] - 0.0005
    elif out.get("pin_d") == "hi+":
        out["pin_d"] = z[1] + 0.0005
    if out.get("bearing") == "min+":
        out["bearing"] = p["bearing_min"] + 0.05
    elif out.get("bearing") == "min-":
        out["bearing"] = p["bearing_min"] - 0.05
    if isinstance(out.get("drive_r"), str):
        out["drive_r"] = p["R"] + float(out["drive_r"][1:])
    if out.get("arm_angle") == "lug":
        out["arm_angle"] = p["lug_angle"] - p["rot_hi"]
    return out


_reference = reference


def reference(inst, **kw):  # noqa: F811 - the self-test's symbolic knobs resolved first
    return _reference(inst, **_resolve(inst, kw))


TASK = """\
**Given:** a plate with a bore, or a base with a standing pin (with or without a head), and a
lug that limits the swing. **Design:** the lever that turns on it, with a drive hole at a stated
radius. **What makes it hard:** choosing the running diameter inside a tolerance zone of a few
hundredths (from an ISO fit, or from the fit name alone); keeping the running surface long enough
while the arm clears the plate; retention modelled as assembled with the play inside 0.3 mm; and
an arm that clears the lug over the whole range, which at an arbitrary orientation means working in
the joint frame."""

KNOBS = {
    "given": "hole | pin: which side of the joint is given",
    "retained": "true: axial play limited both ways; false: free end, lifts off along +z",
    "rules_told": "false: only the fit name (e.g. H7/f7) is given, not the clearance numbers",
    "aligned": "true: joint axis = world z at the origin (easier)",
}
