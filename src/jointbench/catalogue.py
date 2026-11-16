"""The catalogue of real interfaces the fixed parts are made from.

Everything here is a published standard or a common commercial pattern, so a
task reads like a real design brief rather than a puzzle:

* bolts: ISO 4762 socket head cap screws, M3-M8;
* clearance holes: ISO 273 fine / medium / coarse series -- any diameter in
  the fine..coarse range is accepted;
* interfaces: generic mounts described by shape -- square, rectangular and
  round (bolt circle) faces with parametric bolt patterns, some with a plain
  centre spigot. (Generator version 1 also used a NEMA 17 motor face and
  2- and 4-hole flanges; those are kept only so older task folders load.)

Each interface is described in its own LOCAL frame: the mounting face is the
XY plane, its outward normal is +Z (pointing at the connector), and the body
of the part lies at Z < 0. Bolts thread into tapped holes in the fixed part
and are inserted from the connector side, so level 1 has no nuts.

Adding an interface = adding one builder function and one entry in
``INTERFACES``. Nothing else in the benchmark needs to change.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import os
import random

import build123d as bd


@dataclass(frozen=True)
class Bolt:
    """One metric socket head cap screw size."""

    name: str
    d: float  # nominal shank diameter
    clear_fine: float  # ISO 273 fine series clearance hole
    clear_medium: float
    clear_coarse: float
    head_d: float  # ISO 4762 head diameter dk
    head_h: float  # ISO 4762 head height k
    tap_drill: float  # tapped hole in the fixed part

    @property
    def grip_min(self) -> float:
        """Thinnest clamped section accepted under the head."""
        return round(max(3.0, 0.75 * self.d), 2)

    grip_max: float = 25.0  # longest grip a stock screw + engagement covers


BOLTS: dict[str, Bolt] = {
    "M3": Bolt("M3", 3.0, 3.2, 3.4, 3.6, 5.5, 3.0, 2.5),
    "M4": Bolt("M4", 4.0, 4.3, 4.5, 4.8, 7.0, 4.0, 3.3),
    "M5": Bolt("M5", 5.0, 5.3, 5.5, 5.8, 8.5, 5.0, 4.2),
    "M6": Bolt("M6", 6.0, 6.4, 6.6, 7.0, 10.0, 6.0, 5.0),
    "M8": Bolt("M8", 8.0, 8.4, 9.0, 10.0, 13.0, 8.0, 6.8),
}


@dataclass(frozen=True)
class HexKey:
    """The L-shaped hex key that turns one bolt size (KEY_TURN).

    * ``s`` -- hexagon socket size, nominal width across flats, and
      ``socket_depth`` -- socket depth t (minimum): ISO 4762 (socket head cap
      screws), the standard of ``BOLTS``.
    * ``long_leg`` (l1) and ``short_leg`` (l2) -- overall lengths of the two
      legs of a hexagon socket screw key of that size, normal series:
      ISO 2936 (hexagon socket screw keys; the same lengths as DIN 911).
      Each is measured over the outside of the bend, so the short leg's
      length includes the long leg's thickness and vice versa.
    """

    bolt: str
    s: float
    socket_depth: float
    long_leg: float
    short_leg: float


#                     ISO 4762      ISO 2936 normal series
#                      s    t min    l1     l2
KEYS: dict[str, HexKey] = {
    "M3": HexKey("M3", 2.5, 1.3, 56.0, 18.0),
    "M4": HexKey("M4", 3.0, 2.0, 63.0, 20.0),
    "M5": HexKey("M5", 4.0, 2.5, 70.0, 25.0),
    "M6": HexKey("M6", 5.0, 3.0, 80.0, 28.0),
    "M8": HexKey("M8", 6.0, 4.0, 90.0, 32.0),
}

# How far a key may lean off the bolt axis and still turn the screw: a
# ball-end key. ISO 2936 covers plain keys and gives no angle; 25 degrees is
# the working angle of commercial ball-end keys, set by the benchmark's
# (14 November 2026). A plain key is the case of zero lean.
KEY_TILT_MAX = 25.0
# One hex step: a hex key is re-seated every 60 degrees, so a bolt can be
# tightened if the key can swing this far without touching anything.
KEY_SWING = 60.0


# --------------------------------------------------------------------------
# Level 4: two parts of the answer bolted to each other, through both, with a
# hex nut on the far side.


@dataclass(frozen=True)
class Nut:
    """ISO 4032 hex nut, style 1 (regular), coarse thread."""

    size: str
    s: float  # width across flats
    e: float  # width across corners (min)
    m: float  # height (max)
    dw: float  # bearing face diameter (min)
    pitch: float  # thread pitch of the bolt


NUTS: dict[str, Nut] = {
    "M3": Nut("M3", 5.5, 6.01, 2.4, 4.6, 0.5),
    "M4": Nut("M4", 7.0, 7.66, 3.2, 5.9, 0.7),
    "M5": Nut("M5", 8.0, 8.79, 4.7, 6.9, 0.8),
    "M6": Nut("M6", 10.0, 11.05, 5.2, 8.9, 1.0),
    "M8": Nut("M8", 13.0, 14.38, 6.8, 11.6, 1.25),
}
NUT_SUFFIX = "-nut"


# ISO 4762 nominal lengths (mm) and the range each size is made in (product standard, preferred lengths)
LENGTHS = (6, 8, 10, 12, 16, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 80)
LENGTH_RANGE = {"M3": (6, 30), "M4": (6, 40), "M5": (8, 50), "M6": (10, 60), "M8": (12, 80)}


def lengths(size: str) -> list:
    lo, hi = LENGTH_RANGE[size]
    return [L for L in LENGTHS if lo <= L <= hi]


def stick_out(size: str, length: float, clamped: float) -> float:
    """How far a through bolt of ``length`` stands out of its nut when it clamps
    ``clamped`` mm of parts: at least one thread pitch (the nut fully engaged),
    at most two (the nut's room above allows that much)."""
    return float(length) - float(clamped) - NUTS[size].m


def nut_side(size: str) -> str:
    """The catalogue entry for the nut end of a through bolt of ``size``:
    the same shank and clearance holes, the nut in place of the head (its
    width across corners as the head diameter, its height plus two thread
    pitches of bolt end standing out of it as the head height). The checks
    that place a bolt head (HOLES, SEATING, NO_INTERFERENCE) then place the
    nut, unchanged."""
    return size + NUT_SUFFIX


def is_nut(size: str) -> bool:
    return size.endswith(NUT_SUFFIX)


for _n in NUTS.values():
    _b = BOLTS[_n.size]
    BOLTS[nut_side(_n.size)] = Bolt(nut_side(_n.size), _b.d, _b.clear_fine, _b.clear_medium, _b.clear_coarse,
                                    _n.e, round(_n.m + 2 * _n.pitch, 2), 0.0)

# What decides a bolted joint between two printed parts is the plastic round
# each bolt, never the steel (ISO 898-1 class 8.8 carries kilonewtons even at
# M3; 14 November 2026). The plastic's allowable stresses come from the
# one strength the project already cites for printed PLA: 40 MPa across the
# layers ([materials.fdm] in the stiffness settings: Prusament and Ultimaker
# PLA data sheets give 45-60 MPa in the layer plane, about 30-40 MPa across
# layers; the weaker value is used, as the stiffness analysis does), over the
# same safety factor of 2:
#   bearing  (plastic pressed: under a head or nut, on a hole wall)
#            = 40 / 2 = 20 MPa. Plastics are stronger in compression than in
#            tension; no credit is taken for it.
#   shear    (a plug punched through, plastic torn out to an edge)
#            = 40 / sqrt(3) / 2 = 11.5 MPa: the von Mises relation between
#            shear and tensile strength. No published shear strength of printed
#            PLA across layers was found to cite instead.
PLASTIC_SOURCE = ("printed PLA, 40 MPa across layers (stiffness settings, [materials.fdm]: Prusament and Ultimaker "
                  "PLA data sheets), safety factor 2; bearing = 40 / 2 = 20 MPa (no credit for compression); shear = "
                  "40 / sqrt(3) / 2 = 11.5 MPa (von Mises relation)")


def plastic_allowables(process: str = "fdm") -> dict:
    """The allowable bearing and shear stresses (MPa) of the printed plastic round a joint bolt."""
    import math as _m

    from .stiffness import settings as _st

    mat = _st()["materials"][process]
    f, sf = float(mat["strength"]), float(mat["safety"])
    return {"bearing_MPa": round(f / sf, 3), "shear_MPa": round(f / _m.sqrt(3) / sf, 3), "strength_MPa": f,
            "safety": sf, "material": mat["name"], "source": PLASTIC_SOURCE}


@dataclass
class Protrusion:
    """Something that sticks out of the mounting face into connector space
    (a motor pilot boss, a shaft). Local frame, along +Z from the face."""

    name: str
    x: float
    y: float
    radius: float
    height: float


@dataclass
class Interface:
    """A fixed part's mounting interface, in its local frame."""

    kind: str
    params: dict
    bolt: str
    holes: list[tuple[float, float]]  # bolt positions on the face (local XY)
    face_w: float  # extent of the flat mounting face, local X
    face_h: float  # local Y
    protrusions: list[Protrusion] = field(default_factory=list)
    description: str = ""
    # a fixture whose bolts are not all square to one flat face (a pipe): per
    # hole, its point on the fixture's surface and its axis, both local; then
    # ``holes`` only counts them
    hole_frames: list = field(default_factory=list)
    # a "solid" fixed part, held in memory while a task is being generated;
    # in a task folder it is read from params["step"] (never serialised here)
    solid: object = field(default=None, repr=False, compare=False)

    @property
    def has_face(self) -> bool:
        """Catalogue mounts have a flat mounting face; a solid part has one
        only if its family declares it (params["face"])."""
        return self.kind not in ("solid", "bearing", "pipe") or "face" in self.params

    def to_json(self) -> dict:
        return {
            "kind": self.kind,
            "params": self.params,
            "bolt": self.bolt,
            "holes_local": [[x, y] for x, y in self.holes],
            **({"hole_frames_local": [{"point": list(p), "axis": list(a)} for p, a in self.hole_frames]}
               if self.hole_frames else {}),
            "face_size": [self.face_w, self.face_h],
            "protrusions": [p.__dict__ for p in self.protrusions],
            "description": self.description,
        }


def edge_margin(bolt: Bolt) -> float:
    """Material needed from a hole centre to the edge of a part face."""
    return bolt.head_d / 2 + 3.0


# --------------------------------------------------------------------------
# interface definitions


def nema17(params: dict | None = None) -> Interface:
    # NEMA ICS 16: 42.3 mm square face, M3 on a 31.0 mm square,
    # 22 mm pilot boss 2 mm proud, 5 mm shaft 24 mm long.
    h = 31.0 / 2
    return Interface(
        kind="nema17",
        params={},
        bolt="M3",
        holes=[(-h, -h), (h, -h), (h, h), (-h, h)],
        face_w=42.3,
        face_h=42.3,
        protrusions=[
            Protrusion("pilot_boss", 0.0, 0.0, 11.0, 2.0),
            Protrusion("shaft", 0.0, 0.0, 2.5, 24.0),
        ],
        description="NEMA 17 stepper motor face: 4x M3 on a 31 mm square, "
        "22 mm pilot boss 2 mm proud, 5 mm shaft 24 mm long",
    )


def flange2(params: dict) -> Interface:
    b = BOLTS[params["bolt"]]
    p = params["pitch"]
    e = edge_margin(b)
    return Interface(
        kind="flange2",
        params=dict(params),
        bolt=b.name,
        holes=[(-p / 2, 0.0), (p / 2, 0.0)],
        face_w=p + 2 * e,
        face_h=2 * e,
        description=f"2-hole flange: 2x {b.name} at {p:g} mm pitch",
    )


def flange4(params: dict) -> Interface:
    b = BOLTS[params["bolt"]]
    px, py = params["pitch_x"], params["pitch_y"]
    e = edge_margin(b)
    return Interface(
        kind="flange4",
        params=dict(params),
        bolt=b.name,
        holes=[(-px / 2, -py / 2), (px / 2, -py / 2), (px / 2, py / 2), (-px / 2, py / 2)],
        face_w=px + 2 * e,
        face_h=py + 2 * e,
        description=f"4-hole rectangular flange: 4x {b.name} on {px:g} x {py:g} mm",
    )


def _spigot(params: dict) -> list[Protrusion]:
    sp = params.get("spigot")
    return [Protrusion("spigot", 0.0, 0.0, sp["d"] / 2, sp["h"])] if sp else []


def _spigot_text(params: dict) -> str:
    sp = params.get("spigot")
    return f", plain centre spigot {sp['d']:g} mm dia standing {sp['h']:g} mm proud" if sp else ""


def square_mount(params: dict) -> Interface:
    """Flat square face, one bolt near each corner on a square pattern."""
    b = BOLTS[params["bolt"]]
    p = params["pitch"]
    e = edge_margin(b)
    h = p / 2
    return Interface(
        kind="square", params=dict(params), bolt=b.name,
        holes=[(-h, -h), (h, -h), (h, h), (-h, h)],
        face_w=p + 2 * e, face_h=p + 2 * e, protrusions=_spigot(params),
        description=f"flat square mount {p + 2 * e:g} mm across: 4x {b.name} on a {p:g} mm square"
        + _spigot_text(params),
    )


def rect_mount(params: dict) -> Interface:
    """Flat rectangular face with a grid of nx by ny bolts."""
    b = BOLTS[params["bolt"]]
    nx, ny, px, py = params["nx"], params["ny"], params["pitch_x"], params["pitch_y"]
    e = edge_margin(b)
    xs = [(i - (nx - 1) / 2) * px for i in range(nx)]
    ys = [(j - (ny - 1) / 2) * py for j in range(ny)] if ny > 1 else [0.0]
    w, hgt = (nx - 1) * px + 2 * e, ((ny - 1) * py if ny > 1 else 0.0) + 2 * e
    grid = f"{nx} x {ny} grid at {px:g} x {py:g} mm" if ny > 1 else f"a row of {nx} at {px:g} mm"
    return Interface(
        kind="rect", params=dict(params), bolt=b.name,
        holes=[(x, y) for y in ys for x in xs],
        face_w=w, face_h=hgt,
        description=f"flat rectangular mount {w:g} x {hgt:g} mm: {nx * ny}x {b.name} in {grid}",
    )


def round_mount(params: dict) -> Interface:
    """Flat round face with n bolts equally spaced on a bolt circle."""
    import math as _m

    b = BOLTS[params["bolt"]]
    n, pcd = params["n"], params["pcd"]
    e = edge_margin(b)
    r = pcd / 2
    holes = [(round(r * _m.cos(2 * _m.pi * k / n), 9), round(r * _m.sin(2 * _m.pi * k / n), 9)) for k in range(n)]
    od = pcd + 2 * e
    return Interface(
        kind="round", params=dict(params), bolt=b.name, holes=holes,
        face_w=od, face_h=od, protrusions=_spigot(params),
        description=f"flat round mount {od:g} mm dia: {n}x {b.name} equally spaced on a {pcd:g} mm bolt circle"
        + _spigot_text(params),
    )


# Generator version 1 drew nema17 / flange2 / flange4. They stay here so task
# folders generated then still load and grade; new tasks draw only from the
# generic mounts below (generator version 2).
def solid_part(params: dict) -> Interface:
    """A fixed part given as an arbitrary solid -- a grooved base, a hub with
    a keyway -- for families other than the connecting part.

    params: {"step": file name of the solid in its own local frame, next to
    task.json; "sha256": that file's hash; optional "face": {"size": [w, h]}
    when the part has a flat mounting face on its local XY plane (normal +Z);
    optional "description"}. No bolts: such families bring their own checks.
    """
    face = params.get("face") or {}
    w, h = (face.get("size") or [0.0, 0.0])
    return Interface(kind="solid", params=dict(params), bolt="", holes=[], face_w=float(w), face_h=float(h),
                     description=params.get("description", "a fixed part given as a solid"))


# ball bearings, ISO 15 deep groove series: bore d, outside D, width B (mm)
BEARINGS = {"608": (8.0, 22.0, 7.0), "6000": (10.0, 26.0, 8.0), "6001": (12.0, 28.0, 8.0),
            "6200": (10.0, 30.0, 9.0), "6201": (12.0, 32.0, 10.0)}


def bearing(params: dict) -> Interface:
    """A ball bearing to be held: no bolts and no flat mounting face. Its own
    frame: the axis is local +Z, the face the bearing enters by lies at z = 0,
    and it is pressed in along +Z until its far face, at z = B, meets the
    shoulder."""
    d, D, B = BEARINGS[params["size"]]
    return Interface(kind="bearing", params=dict(params), bolt="", holes=[], face_w=0.0, face_h=0.0,
                     description=f"ball bearing {params['size']}: bore {d:g} mm, outside {D:g} mm, width {B:g} mm")


def pipe(params: dict) -> Interface:
    """A round pipe (or bar) whose bolts point straight out of it. Its own
    frame: the pipe's axis is local X through the origin; a bolt at angle a
    and position x along it sits on the surface at (x, R sin a, R cos a) and
    points outward, along (0, sin a, cos a); a = 0 is local +Z, as for a
    flat face. params: bolt, od (outside diameter), holes: [[x, a_deg], ...],
    and optionally length (mm, centred on the bolts; default: just past them)."""
    import math as _m

    b = BOLTS[params["bolt"]]
    R = params["od"] / 2
    frames = []
    for x, a in params["holes"]:
        s, c = _m.sin(_m.radians(a)), _m.cos(_m.radians(a))
        frames.append(((float(x), round(R * s, 9), round(R * c, 9)), (0.0, round(s, 9), round(c, 9))))
    xs = [x for x, _ in params["holes"]]
    length = params.get("length") or max(xs) - min(xs) + 2 * (b.head_d + 8)
    return Interface(kind="pipe", params=dict(params), bolt=b.name, holes=[(p[0], p[1]) for p, _ in frames],
                     face_w=float(length), face_h=float(params["od"]), hole_frames=frames,
                     description=f"round pipe {params['od']:g} mm outside diameter: {len(frames)}x {b.name} "
                                 "screwed straight into it, square to its surface")


def bearing_dims(itf: Interface) -> tuple[float, float, float]:
    return BEARINGS[itf.params["size"]]


INTERFACES = {"pipe": pipe, "bearing": bearing, "nema17": nema17, "flange2": flange2, "flange4": flange4, "solid": solid_part,
              "square": square_mount, "rect": rect_mount, "round": round_mount}
GENERIC = ("rect", "round", "square")
GENERATOR_VERSION = 5  # 5: truss references, stiffness required (4: makeable rod references)


def make_interface(kind: str, params: dict) -> Interface:
    return INTERFACES[kind](params)


def sample_interface(rng: random.Random) -> Interface:
    """Draw one generic mount: outline, bolt pattern and size. Deterministic
    given the rng state. (Generator version 2: no named products.)"""
    import math as _m

    kind = rng.choice(GENERIC)
    bolt = rng.choice(["M3", "M4", "M5", "M6", "M8"])
    b = BOLTS[bolt]
    pitches = [p for p in (20, 25, 30, 40, 50, 60, 70, 80) if p >= b.head_d + 8]

    def spigot(room: float):
        # a plain centre spigot on about 40% of round and square mounts
        if rng.random() >= 0.4 or room < 8:
            return None
        return {"d": float(min(room, rng.choice([8, 10, 12, 16, 20, 25, 30]))), "h": float(rng.choice([2, 3, 5, 8]))}

    if kind == "square":
        p = rng.choice(pitches)
        params = {"bolt": bolt, "pitch": p}
        sp = spigot(2 * (p / 2 - b.head_d / 2 - 3))
        if sp:
            params["spigot"] = sp
        return square_mount(params)
    if kind == "rect":
        nx, ny = rng.choice([(2, 1), (3, 1), (2, 2), (3, 2)])
        return rect_mount({"bolt": bolt, "nx": nx, "ny": ny,
                           "pitch_x": rng.choice(pitches), "pitch_y": rng.choice(pitches)})
    n = rng.choice([3, 4, 5, 6])
    # bolt circle big enough that neighbouring heads keep 3 mm apart
    pcd_min = (b.head_d + 3) / _m.sin(_m.pi / n)
    pcd = rng.choice([v for v in (30, 40, 50, 60, 70, 80, 90) if v >= pcd_min] or [90])
    params = {"bolt": bolt, "n": n, "pcd": pcd}
    sp = spigot(2 * (pcd / 2 - b.head_d / 2 - 3))
    if sp:
        params["spigot"] = sp
    return round_mount(params)


def sample_interface_v1(rng: random.Random) -> Interface:
    """Generator version 1 (NEMA 17, 2- and 4-hole flanges). Kept only so the
    history is documented; no level calls it any more."""
    kind = rng.choice(["flange2", "flange4", "nema17"])
    if kind == "nema17":
        return nema17({})
    bolt = rng.choice(["M3", "M4", "M5", "M6", "M8"])
    b = BOLTS[bolt]
    pitches = [p for p in (20, 25, 30, 40, 50, 60, 70, 80) if p >= b.head_d + 8]
    if kind == "flange2":
        return flange2({"bolt": bolt, "pitch": rng.choice(pitches)})
    return flange4({"bolt": bolt, "pitch_x": rng.choice(pitches), "pitch_y": rng.choice(pitches)})


# --------------------------------------------------------------------------
# fixed-part solids (local frame)


def fixed_part_solid(itf: Interface, root: str | None = None) -> bd.Part:
    """The fixed part as a solid: body behind the face, tapped holes, and any
    protrusions in front of it. A "solid" part is the given solid itself:
    held in memory while generating, read from ``root``/params["step"] when
    the task was loaded from a folder."""
    if itf.kind == "solid":
        if itf.solid is not None:
            return itf.solid
        if root is None:
            raise ValueError("a solid fixed part needs the task folder to read its STEP from")
        from .geometry import read_step

        s = read_step(os.path.join(root, itf.params["step"]))
        itf.solid = s.solids()[0] if len(s.solids()) == 1 else s
        return itf.solid
    import json as _json

    key = (itf.kind, _json.dumps(itf.params, sort_keys=True))
    if key not in _SOLID_CACHE:  # the same mount is built once and then only moved
        _SOLID_CACHE[key] = _build_fixed_part(itf)
    return _SOLID_CACHE[key]


_SOLID_CACHE: dict = {}


def _build_fixed_part(itf: Interface) -> bd.Part:
    if itf.kind == "pipe":  # a solid pipe along local X, tapped radially
        b = BOLTS[itf.bolt]
        xs = [p[0] for p, _ in itf.hole_frames]
        body = bd.Pos((max(xs) + min(xs)) / 2, 0, 0) * bd.Rot(0, 90, 0) * bd.Cylinder(itf.face_h / 2, itf.face_w)
        tap = 1.5 * b.d + 2
        for p, a in itf.hole_frames:
            body = body - bd.Plane(origin=p, z_dir=a) * bd.Pos(0, 0, -tap / 2) * bd.Cylinder(b.tap_drill / 2, tap)
        return body
    if itf.kind == "bearing":  # the bearing itself: a ring, in its seated place
        d, D, B = bearing_dims(itf)
        return bd.Pos(0, 0, B / 2) * (bd.Cylinder(D / 2, B) - bd.Cylinder(d / 2, B + 1))
    b = BOLTS[itf.bolt]
    if itf.kind == "nema17":
        body = bd.Pos(0, 0, -20) * bd.Box(itf.face_w, itf.face_h, 40)
        tap_depth = 4.5
    elif itf.kind in GENERIC:
        plate_t = 10.0
        if itf.kind == "round":
            body = bd.Pos(0, 0, -plate_t / 2) * bd.Cylinder(itf.face_w / 2, plate_t)
            hub = bd.Pos(0, 0, -plate_t - 10) * bd.Cylinder(itf.face_w * 0.3, 20)
        else:
            body = bd.Pos(0, 0, -plate_t / 2) * bd.Box(itf.face_w, itf.face_h, plate_t)
            hub = bd.Pos(0, 0, -plate_t - 10) * bd.Box(itf.face_w * 0.6, itf.face_h * 0.6, 20)
        body = body + hub
        tap_depth = 1.5 * b.d + 2
    else:
        plate_t = 10.0
        if itf.kind == "flange2":
            body =bd.Pos(0, 0, -plate_t) * bd.extrude(bd.SlotOverall(itf.face_w, itf.face_h), plate_t)
            hub = bd.Pos(0, 0, -plate_t - 15) * bd.Cylinder(itf.face_h / 2, 30)
        else:
            body = bd.Pos(0, 0, -plate_t / 2) * bd.Box(itf.face_w, itf.face_h, plate_t)
            hub = bd.Pos(0, 0, -plate_t - 10) * bd.Box(itf.face_w * 0.6, itf.face_h * 0.6, 20)
        body = body + hub
        tap_depth = 1.5 * b.d + 2
    for x, y in itf.holes:
        body = body - bd.Pos(x, y, -tap_depth / 2) * bd.Cylinder(b.tap_drill / 2, tap_depth)
    for p in itf.protrusions:
        body = body + bd.Pos(p.x, p.y, p.height / 2) * bd.Cylinder(p.radius, p.height)
    return body
