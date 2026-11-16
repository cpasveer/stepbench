"""Level 6, "moves", on a version 1 setup: a linkage carries B home and latches it.

Shared by the three setups; each setup (``pipe``, later ``facing`` and
``flange``) supplies its fixed parts with B's home pose (its pose from levels
1-3), the stated load, the mechanism's plane and the frame's base on the fixed
fixture. Same seed, same fixed parts as the setup's levels 1-3.

The task: fixture A stays fixed. Fixture B starts at a stated start pose and
must be carried, by one continuous motion of a linkage the model designs,
through two stated intermediate poses into its end pose -- its home. There it
latches: pulled straight down (the stated direction), B with what is bolted to
it is stopped by the fixed side within the stated play, and it holds the
stated load there. Everything happens inside a closed box with one opening in
its top wall; every part goes in through it along a straight line.

Made backwards from a hidden four-bar, designed at B's home:

  frame    the setup's base on A, with a clevis post at each ground pivot and
           a post whose flat top is the latch's ledge, each rising straight up
  carrier  a plate on B with a clevis at each coupler pivot and an arm whose
           end is the latch's lug, resting just above the ledge at home
  crank, rocker  flat links, each in its own layer; the latch has its own layer
  4 catalogue M4 axle screws with nuts, clamped in the clevises

The drive runs the crank from the start to home (50-110 degrees, either way);
the start pose and the intermediate poses (at 25-40 % and 55-75 %) are where
the hidden four-bar puts B. Draws are screened cheaply in 2D, layer by layer
(the links, the carrier, B and the latch's lug against the frame, the pipe and
each other over the whole motion; the screw lines; the four-bar's transmission
angle), then the reference is built and graded with the real grader, printing
rules included; a draw is kept only if it passes everything.
"""
from __future__ import annotations

import json
import math
import os
import random
import shutil

import build123d as bd
import numpy as np

from ...catalogue import BOLTS, GENERATOR_VERSION
from ...checks.base import volume
from ...geometry import BUILD_ROOM, Placement, build_box, fits_build_volume, write_step
from ...task import DEFAULT_TOLERANCES, Obstacle, Task
from .. import assembly as AS
from .. import linkage as LK
from .. import motion as MO
from . import geom2d as G2

LEVEL = 6
NAME = "moves"
AXLE = "M4"
LE = 3.5  # mm, each clevis ear along the axis (at least the M4 screw's least grip, 3 mm)
LINK_T = 5.0  # mm, a link's thickness along the axis
REF_GAP = 0.4  # mm, the reference's running gap
LUG_T = 5.0  # mm, the latch layer's thickness
LUG_S, LUG_H = 8.0, 6.0  # mm, the lug's size in the plane (across, high)
WALL = 6.0  # mm, the box's walls
BOX_ROOM = 8.0  # mm between the box's walls and anything inside
OPEN_ROOM = 3.0  # mm round the parts' outlines in the opening
HINGE_MISS_MIN = 10.0
DRIVE_MIN = 35  # deg: the least drive from the start to home
LINK_MAX = 105.0  # mm, a link's length (pivot to pivot): it prints on a Prusa MINI with room
POST_MAX = 70.0  # mm, a ground pivot's height over the frame's base
FRAME_MAX = 180.0 - 2 * BUILD_ROOM - 5.0  # mm: the frame's height from its print bed (the build-volume rule, less 5)
POSE_MOVE_MIN = 15.0
POSE_TURN_MIN = 8.0
TRANSMISSION = (35.0, 145.0)
SCREEN_GAP = 1.0  # mm the 2D screens keep between things (the grader then measures exactly)
G_ACC = 9.81
L6_TOLERANCES = {
    "gap_min": 0.3, "play_max": 0.6, "tilt_max_deg": 5.0, "loose_shift": 5.0, "loose_tilt": 15.0,
    "step_max_deg": 2.0, "step_move_max": 1.5, "pose_position": 0.5, "pose_angle_deg": 0.5,
    "pivot_offset_max": 0.05, "pivot_angle_max_deg": 0.1,
    "latch_play_max": 0.6,  # mm: pulled down, B's side meets the fixed side within this
    "screw_room": 5.0,  # mm: inside the box, room beyond a screw's or nut's full length to bring it to its line
}
REQUIRED = ["SEAT_INSIDE", "CLEAN", "MANUFACTURING", "KEY_TURN"]
GRIP = LINK_T + 2 * REF_GAP + 2 * LE  # mm, an axle screw's grip: ear, gap, link, gap, ear
AXLE_LEN = next(Ln for Ln in AS.LENGTHS if Ln >= GRIP + AS.NUTS[AXLE][1])
# mm past a stack's inner face that each nut needs clear: its place, the shank beyond it, and the room to hold it
NUT_ROOM = AXLE_LEN - GRIP + AS.NUTS[AXLE][1] + L6_TOLERANCES["screw_room"]


def setup_for(name: str, seed: int):
    if name == "pipe":
        from .pipe import Setup
    elif name == "flange":
        from .flange import Setup
    else:
        raise ValueError(f"no level 6 for setup {name!r} yet")
    setup = Setup(seed)
    global POST_LEAN
    POST_LEAN = float(getattr(setup, "post_lean", POST_LEAN_DEFAULT))  # a setup on a flat base may stand its posts upright
    return setup


def rk() -> float:
    ab = BOLTS[AXLE]
    return max(ab.head_d, AS.nut_corner_d(AXLE)) / 2 + 2.5


class Plane:
    """The mechanism's plane in the world: origin, s, t, axis (= s x t)."""

    def __init__(self, fr: dict):
        self.O, self.s, self.t, self.a = (np.asarray(fr[k], float) for k in ("origin", "s", "t", "axis"))

    def world(self, st, w: float = 0.0) -> np.ndarray:
        return self.O + st[0] * self.s + st[1] * self.t + w * self.a

    def st(self, p) -> np.ndarray:
        v = np.asarray(p, float) - self.O
        return np.array([float(v @ self.s), float(v @ self.t)])

    def motion(self, ang: float, shift) -> Placement:
        """The 3D rigid motion that turns the plane by ``ang`` (rad) about its origin, then shifts it."""
        return MO.about(tuple(self.O), tuple(self.a), math.degrees(ang)).then(
            MO.shift(shift[0] * self.s + shift[1] * self.t))


def section(solid, P: Plane, w: float) -> list:
    """The solid's section in the layer at ``w``, as polygons in (s, t)."""
    pl = bd.Plane(origin=tuple(P.world((0, 0), w)), x_dir=tuple(P.s), z_dir=tuple(P.a))
    try:
        sec = solid & (pl * bd.Box(3000.0, 3000.0, 0.01))
    except Exception:  # noqa: BLE001
        return []
    out = []
    for f in sec.faces():
        if abs(float(np.array(tuple(f.normal_at(f.center()))) @ P.a)) < 0.9:
            continue
        pts = [np.array(tuple(v)) for v in f.outer_wire().positions(list(np.linspace(0, 1, 160)))[:-1]]
        out.append(np.array([P.st(p) for p in pts]))
    return out


def relative(start: Placement, target: Placement) -> Placement:
    R = np.array(target.rot) @ np.array(start.rot).T
    t = np.asarray(target.origin) - R @ np.asarray(start.origin)
    return Placement(tuple(float(v) for v in t), tuple(tuple(float(v) for v in r) for r in R))


# --------------------------------------------------------------------------
# the hidden four-bar, in the plane


def coupler(d: dict, deg: float):
    """(C2, C4) with the crank turned ``deg`` from B's home, or None."""
    m = d["mech"]
    return LK.fourbar(m["O2"], m["O4"], m["r2"], m["r3"], m["r4"], m["theta_home"] + math.radians(deg), m["mode"])


def coupler_move(d: dict, C2, C4) -> tuple[float, np.ndarray]:
    """(angle, shift) of the coupler from home: a point x at home goes to R(angle) x + shift."""
    m = d["mech"]
    h2, h4 = np.asarray(m["C2"]), np.asarray(m["C4"])
    ang = math.atan2(*(np.asarray(C4) - np.asarray(C2))[::-1]) - math.atan2(*(h4 - h2)[::-1])
    c, s = math.cos(ang), math.sin(ang)
    return ang, np.asarray(C2) - np.array([[c, -s], [s, c]]) @ h2


def transmission(d: dict, C2, C4) -> float:
    m = d["mech"]
    v1, v2 = np.asarray(C4) - np.asarray(C2), np.asarray(C4) - np.asarray(m["O4"])
    c = float(v1 @ v2 / (np.linalg.norm(v1) * np.linalg.norm(v2)))
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def path(d: dict, step: float = 1.0):
    """[(deg from the start, C2, C4)] from the start to home, or None if it cannot close or nears a dead point."""
    R = d["drive"]["range_deg"]
    n = max(2, math.ceil(abs(R) / step))
    out = []
    for i in range(n + 1):
        deg_home = -R + R * i / n  # the crank's turn from home: -R at the start, 0 at home
        got = coupler(d, deg_home)
        if got is None:
            return None
        if not TRANSMISSION[0] <= transmission(d, *got) <= TRANSMISSION[1]:
            return None
        out.append((R * i / n, got[0], got[1]))
    return out


def b_placement(d: dict, P: Plane, home_b: Placement, C2, C4) -> Placement:
    ang, sh = coupler_move(d, C2, C4)
    return home_b.then(P.motion(ang, sh))


def hinge_miss(poses: list, P: Plane) -> float:
    """The least, over every fixed hinge square to the plane, of the largest distance by which B's centre
    misses a pose (see 4c: each pose's own turn fixes its pole; the best centre lies among the poles)."""
    b = [P.st(x.origin) for x in poses]
    r0 = np.array(poses[0].rot)
    poles, w = [], []
    for k in range(1, len(poses)):
        rel = np.array(poses[k].rot) @ r0.T
        dth = math.atan2(float(P.t @ rel @ P.s), float(P.s @ rel @ P.s))
        if abs(dth) < 1e-9:
            return math.inf
        chord = b[k] - b[0]
        half = float(np.linalg.norm(chord)) / 2
        perp = np.array([-chord[1], chord[0]]) / (2 * half)
        poles.append((b[0] + b[k]) / 2 + perp * half / math.tan(dth / 2))
        w.append(2 * abs(math.sin(dth / 2)))
    Pp = np.array(poles)
    f = lambda x: max(wk * float(np.linalg.norm(x - pk)) for wk, pk in zip(w, Pp))  # noqa: E731
    c = Pp.mean(axis=0)
    half = float(np.max(np.abs(Pp - c))) + 1.0
    g = np.linspace(-1.0, 1.0, 41)
    for _ in range(20):
        c = min((c + half * np.array([gx, gy]) for gx in g for gy in g), key=f)
        half /= 4.0
    return f(c)


# --------------------------------------------------------------------------
# the draw


def layers(w0: float, crank_first: bool) -> dict:
    W = LINK_T + 2 * REF_GAP + 2 * LE
    lug = max(LUG_T + 2 * REF_GAP, NUT_ROOM + 0.5)  # the stacks far enough apart that a nut's room misses the other
    lo = w0 - (2 * W + lug) / 2
    a, b, c = lo + W / 2, lo + W + lug / 2, lo + W + lug + W / 2
    wc, wr = (a, c) if crank_first else (c, a)
    return {"crank": wc, "latch": b, "rocker": wr, "W": W, "lo": lo, "hi": lo + 2 * W + lug}


def slabs(L: dict) -> dict:
    """Every layer's slabs: {name: (w0, w1)}."""
    inner = LINK_T / 2 + REF_GAP
    out = {}
    for k in ("crank", "rocker"):
        w = L[k]
        out[k + "_link"] = (w - LINK_T / 2, w + LINK_T / 2)
        out[k + "_ear_lo"] = (w - inner - LE, w - inner)
        out[k + "_ear_hi"] = (w + inner, w + inner + LE)
    out["latch"] = (L["latch"] - LUG_T / 2, L["latch"] + LUG_T / 2)
    return out


class Scene2D:
    """What the screens look at, cut in the layers once: the frame's base and A (fixed), B at home."""

    def __init__(self, setup, P: Plane, z_bed: float, z_top: float):
        self.P = P
        self.z_bed = z_bed
        fx = setup.fixed_solids()
        self.base = setup.base(z_bed, z_top)
        self.fixed_solid = fx[setup.fixed]
        self.b_solid = fx[setup.carried]
        # a setup may ask for the frame's posts to be drawn clear of its keep-out bodies (bolts' key ways)
        self.keep = bd.Compound(setup.keep_out()) if getattr(setup, "posts_clear_keep_out", False) else None
        self._cache = {}

    def columns(self, w: float, s_: float) -> list:
        """The base's material along the vertical line at s in layer w: [(t_lo, t_hi)] (s to 0.5 mm)."""
        s_ = round(s_ * 2) / 2
        key = ("col", round(w, 3), s_)
        if key not in self._cache:
            ts = []
            for poly in self.at(w)["base"]:
                A, B = poly, np.roll(poly, -1, axis=0)
                sel = ((A[:, 0] <= s_) & (s_ < B[:, 0])) | ((B[:, 0] <= s_) & (s_ < A[:, 0]))
                a_, b_ = A[sel], B[sel]
                ts += list(a_[:, 1] + (s_ - a_[:, 0]) * (b_[:, 1] - a_[:, 1]) / (b_[:, 0] - a_[:, 0]))
            ts.sort()
            self._cache[key] = [(ts[i], ts[i + 1]) for i in range(0, len(ts) - 1, 2)]
        return self._cache[key]

    def at(self, w: float) -> dict:
        key = round(w, 3)
        if key not in self._cache:
            self._cache[key] = {"base": section(self.base, self.P, w), "fixed": section(self.fixed_solid, self.P, w),
                                "B": section(self.b_solid, self.P, w)}
        return self._cache[key]


def _hits_keep(scene: "Scene2D", shapes: list, ws: list) -> bool:
    """Whether the frame's post ``shapes`` (in the plane) come within SCREEN_GAP of the setup's keep-out
    bodies in any layer of ``ws``."""
    for w in ws:
        key = ("keep", round(w, 3))
        if key not in scene._cache:
            scene._cache[key] = section(scene.keep, scene.P, w)
        if scene._cache[key] and G2.shape_dist(shapes, scene._cache[key]) < SCREEN_GAP:
            return True
    return False


def _probe_ws(sl: tuple) -> list:
    return [sl[0] + 0.05, (sl[0] + sl[1]) / 2, sl[1] - 0.05]


def base_high(scene: Scene2D, s0: float, s1: float, ws: list) -> float:
    """The highest t the frame's base reaches under the band s0..s1 in any layer of ``ws`` (-inf if none)."""
    best = -math.inf
    for w in ws:
        for s_ in np.linspace(s0, s1, 7):
            for _, hi_ in scene.columns(w, s_):
                best = max(best, hi_)
    return best


def saddle_top(scene: Scene2D, s0: float, s1: float, ws: list, t_max: float) -> float | None:
    """The highest t below ``t_max`` at which the frame's base holds material under the band s0..s1, in every
    layer of ``ws`` (the least over the band and the layers); None if some column has nothing below."""
    tops = []
    for w in ws:
        for s_ in np.linspace(s0, s1, 7):
            best = None
            for lo_, hi_ in scene.columns(w, s_):
                if lo_ < t_max:
                    top = min(hi_, t_max)
                    best = top if best is None else max(best, top)
            if best is None:
                return None
            tops.append(best)
    return min(tops)


WHY: dict = {}  # why draws were dropped before the screens (for tuning)


def _no(why: str):
    WHY[why] = WHY.get(why, 0) + 1
    return None


def draw(rng: random.Random, setup, P: Plane, scene: Scene2D) -> dict | None:
    R_ = rk()
    itf_b = setup.parts()[setup.carried]
    bb = BOLTS[itf_b.bolt]
    t_b = max(5.0, bb.grip_min)
    half_b = min(itf_b.face_w, itf_b.face_h) / 2 - 1.5
    # the layers sit towards the side where the fixed fixture's base is widest (setup.layer_bias); few layer sets,
    # so their sections are cut once
    L = layers(getattr(setup, "layer_bias", 0.0) + rng.choice([-1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5]), rng.random() < 0.5)
    lay_half = getattr(setup, "layers_half", None) or half_b  # a setup may let the layers reach past B's plate
    if L["hi"] > lay_half or L["lo"] < -lay_half:
        return _no("layers wider than B")
    SL = slabs(L)
    # the carrier's pivots: in front of B, at B's middle height (the clevises' outer ears share layers with B's
    # bolt rows), one nearer B and one farther out, so each screw's line misses the other knuckle
    s_c = t_b + bb.head_h + R_ + 2.0
    near, far = s_c + rng.uniform(0.0, 6.0), s_c + 2 * R_ + 6.0 + rng.uniform(0.0, 10.0)
    if rng.random() < 0.5:
        near, far = far, near
    C2 = np.array([near, rng.uniform(-2.5, 2.5)])
    C4 = np.array([far, rng.uniform(-2.5, 2.5)])
    # ground pivots on short posts that rise straight up from the frame's base, to the right of the carrier's
    # pivots, within a printable link's reach of them
    posts, Os = {}, {}
    s_lo = max(30.0, max(C2[0], C4[0]) + 2 * R_ + 4.0)
    s_hi = max(s_lo + 1.0, getattr(setup, "pivot_s_max", 95.0))
    s_first = rng.uniform(s_lo, s_hi)
    for name, C, stack in (("O2", C2, "crank"), ("O4", C4, "rocker")):
        ws = _probe_ws(SL[stack + "_ear_lo"]) + _probe_ws(SL[stack + "_ear_hi"])
        if name == "O2":
            s_o = s_first
        else:  # the other ground pivot well apart from the first, along the base
            opts = [x for x in (s_first - rng.uniform(22.0, 45.0), s_first + rng.uniform(22.0, 45.0)) if s_lo <= x <= s_hi]
            if not opts:
                return _no("no room for the second ground pivot")
            s_o = rng.choice(opts)
        top = saddle_top(scene, s_o - R_, s_o + R_, ws, 60.0)
        if top is None:
            return _no("no base under a ground pivot")
        high = base_high(scene, s_o - R_, s_o + R_, ws + _probe_ws(SL[stack + "_link"]))
        # the frame prints standing on its bed: from the bed up to the post's top it fits the printer with room
        t_cap = (scene.z_bed - P.O[2]) + FRAME_MAX - R_
        if high + R_ + 8.0 > t_cap:
            return _no("a ground pivot would make the frame too tall")
        O = np.array([s_o, rng.uniform(high + R_ + 8.0, min(high + R_ + 48.0, t_cap))])
        if O[1] - top > POST_MAX or np.linalg.norm(O - C) > LINK_MAX:
            return _no("a ground pivot out of a printable reach")
        if scene.keep and _hits_keep(scene, [G2.circle(O, R_), G2.rect(O[0] - R_, O[0] + R_, top - 6.0, O[1])], ws):
            return _no("a ground pivot's post in a key way")
        Os[name], posts[name] = O, top
    O2, O4 = Os["O2"], Os["O4"]
    if np.linalg.norm(O2 - O4) < 2 * R_ + 8:
        return _no("ground pivots too close")
    r2, r3, r4 = (float(np.linalg.norm(x)) for x in (C2 - O2, C4 - C2, C4 - O4))
    th = math.atan2(*(C2 - O2)[::-1])
    got = LK.fourbar(O2, O4, r2, r3, r4, th, 1)
    mode = 1 if got is not None and np.linalg.norm(got[1] - C4) < 1e-6 else -1
    d = {"layers": L, "Rk": R_, "t_b": t_b, "mech": {"O2": O2.tolist(), "O4": O4.tolist(), "C2": C2.tolist(),
                                                     "C4": C4.tolist(), "r2": r2, "r3": r3, "r4": r4,
                                                     "theta_home": th, "mode": mode},
         "post_tops": posts}
    # the drive: backwards from home as far as the linkage goes without nearing a dead point and with its links off
    # the base; the direction whose start turns B's face upwards (B then never hangs over its own carrier)
    link_ws = {st: _probe_ws(SL[st + "_link"]) for st in ("crank", "rocker")}

    def reach_for(sign):
        reach = 0
        for k in range(1, 121):
            got_ = coupler(d, -sign * k)
            if got_ is None or not TRANSMISSION[0] <= transmission(d, *got_) <= TRANSMISSION[1]:
                break
            if k % 5 == 0:
                for st_, O_, C_ in (("crank", O2, got_[0]), ("rocker", O4, got_[1])):
                    for w in link_ws[st_]:
                        sc = scene.at(w)
                        if G2.shape_dist([G2.stadium(O_, C_, R_, 12)], sc["base"] + sc["fixed"]) < SCREEN_GAP:
                            return reach
            reach = k
        return reach

    signs = [-1, 1]
    rng.shuffle(signs)
    R = None
    for sg in signs:
        reach = reach_for(sg)
        if reach >= DRIVE_MIN:
            R = sg * float(rng.randint(DRIVE_MIN, reach))
            break
    if R is None:
        return _no(f"the drive reaches less than {DRIVE_MIN} deg")
    d["drive"] = {"range_deg": R, "targets_at": [round(rng.uniform(0.25, 0.4) * R, 2), round(rng.uniform(0.55, 0.75) * R, 2), R]}
    # the latch: a lug on the carrier just above a ledge on the frame, in its own layer. Tried at a few places:
    # kept where the lug's arm stays clear of the ledge's post over the whole drive and ends just above it
    ws = _probe_ws(SL["latch"])
    pts = []
    for k in range(0, int(abs(R)) + 1, 2):
        got = coupler(d, -R + math.copysign(k, R))
        pts.append(coupler_move(d, *got))
    pts.append(coupler_move(d, *coupler(d, 0.0)))
    for _ in range(15):
        X = np.array([rng.uniform(t_b + 6.0, 40.0), rng.uniform(-half_b + LUG_H, 6.0)])
        top = saddle_top(scene, X[0] - LUG_S / 2, X[0] + LUG_S / 2, ws, X[1] - LUG_H / 2 - REF_GAP - 3.0)
        if top is None or top > X[1] - LUG_H / 2 - REF_GAP - 3.0 - 1e-6:
            continue
        ledge = float(X[1] - LUG_H / 2 - REF_GAP)
        post = [_latch_post_2d(X, ledge, top)]
        if scene.keep and _hits_keep(scene, [G2.rect(X[0] - LUG_S / 2, X[0] + LUG_S / 2, top - 6.0, ledge)], ws):
            continue
        arm = G2.rect(t_b - 0.5, X[0] + LUG_S / 2, X[1] - LUG_H / 2, X[1] + LUG_H / 2)
        if all(G2.shape_dist([G2.move(arm, a_, sh_)], post) >= SCREEN_GAP for a_, sh_ in pts[:-3]):
            d["latch"] = {"X": X.tolist(), "ledge": ledge, "post_foot": top}
            return d
    return _no("no place for the latch")


# --------------------------------------------------------------------------
# the 2D screens


def frame_shapes(d: dict) -> dict:
    """The frame's posts in the plane, per layer kind: {"crank"/"rocker" ears, "latch"}: polygons."""
    R_ = d["Rk"]
    m = d["mech"]
    out = {}
    for name, stack in (("O2", "crank"), ("O4", "rocker")):
        O = np.asarray(m[name])
        foot = d["post_tops"][name] - 25.0  # as built: down into the base, its sides leaning out (POST_LEAN)
        k = math.tan(math.radians(POST_LEAN)) * (O[1] - foot)
        out[stack + "_ear"] = [G2.circle(O, R_), np.array([(O[0] - R_ - k, foot), (O[0] + R_ + k, foot),
                                                           (O[0] + R_, O[1]), (O[0] - R_, O[1])], float)]
    out["latch"] = [_latch_post_2d(np.asarray(d["latch"]["X"]), d["latch"]["ledge"], d["latch"]["post_foot"])]
    return out


def _latch_post_2d(X, ledge: float, post_foot: float) -> np.ndarray:
    """The ledge's post in the plane, as built: from the ledge down into the base, its sides leaning out."""
    foot = post_foot - 25.0
    k = math.tan(math.radians(POST_LEAN)) * (ledge - foot)
    return np.array([(X[0] - LUG_S / 2 - k, foot), (X[0] + LUG_S / 2 + k, foot), (X[0] + LUG_S / 2, ledge),
                     (X[0] - LUG_S / 2, ledge)], float)


def carrier_shapes(d: dict) -> dict:
    """The carrier in the plane at home, per layer kind: plate (every layer), ears, the lug arm."""
    R_, t_b = d["Rk"], d["t_b"]
    m = d["mech"]
    L = d["layers"]
    plate = [G2.rect(0.0, t_b, -20.0, 20.0)]
    out = {"plate": plate}
    for name, stack in (("C2", "crank"), ("C4", "rocker")):
        C = np.asarray(m[name])
        out[stack + "_ear"] = [G2.circle(C, R_), G2.rect(t_b - 0.5, C[0], C[1] - R_, C[1] + R_)]
    X = np.asarray(d["latch"]["X"])
    out["latch"] = [G2.rect(t_b - 0.5, X[0] + LUG_S / 2, X[1] - LUG_H / 2, X[1] + LUG_H / 2)]
    return out


def screen(d: dict, setup, P: Plane, scene: Scene2D) -> str | None:
    pth = path(d)
    if pth is None:
        return "the hidden linkage nears a dead point or cannot close"
    m, L = d["mech"], d["layers"]
    SL = slabs(L)
    R_ = d["Rk"]
    O2, O4, C2h, C4h = (np.asarray(m[k]) for k in ("O2", "O4", "C2", "C4"))
    fr = frame_shapes(d)
    car = carrier_shapes(d)
    sample = pth[::3] + [pth[-1]]
    gap = SCREEN_GAP
    for deg, C2, C4 in sample:
        ang, sh = coupler_move(d, C2, C4)
        end = deg == pth[-1][0]
        links = {"crank": G2.stadium(O2, C2, R_), "rocker": G2.stadium(O4, C4, R_)}
        for stack in ("crank", "rocker"):
            ws = _probe_ws(SL[stack + "_link"])
            for w in ws:
                sc = scene.at(w)
                if G2.shape_dist([links[stack]], sc["base"] + sc["fixed"]) < gap:
                    return f"the {stack} meets the frame's base or A at {deg:.0f} deg"
                Bm = [G2.move(p, ang, sh) for p in sc["B"]]
                if G2.shape_dist([links[stack]], Bm) < gap:
                    return f"the {stack} meets B at {deg:.0f} deg"
                if G2.shape_dist([links[stack]], [G2.move(p, ang, sh) for p in car["plate"]]) < gap:
                    return f"the {stack} meets the carrier's plate at {deg:.0f} deg"
        # the carrier (moving) against the frame's posts, base and A
        for kind, wsl in (("crank_ear", SL["crank_ear_lo"]), ("crank_ear", SL["crank_ear_hi"]),
                          ("rocker_ear", SL["rocker_ear_lo"]), ("rocker_ear", SL["rocker_ear_hi"]), ("latch", SL["latch"])):
            mov = [G2.move(p, ang, sh) for p in car[kind] + car["plate"]]
            for w in _probe_ws(wsl):
                sc = scene.at(w)
                fixed = sc["base"] + sc["fixed"] + fr[kind]
                need = gap if not (end and kind == "latch") else REF_GAP - 0.05
                if G2.shape_dist(mov, fixed) < need:
                    what = "the frame's base" if G2.shape_dist(mov, sc["base"]) < need else (
                        "A" if G2.shape_dist(mov, sc["fixed"]) < need else "a post")
                    part = "plate" if G2.shape_dist([G2.move(p, ang, sh) for p in car["plate"]], fixed) < need else kind
                    return f"the carrier ({part.replace('_', ' ')}) meets {what} at {deg:.0f} deg"
                if G2.shape_dist([G2.move(p, ang, sh) for p in sc["B"]], fixed) < gap:
                    return f"B meets the frame or A at {deg:.0f} deg"
    # B's bolt heads, their room and a straight key's way along their axis: clear of the carrier's ears and arm
    home_b = setup.home[setup.carried]
    itf_b = setup.parts()[setup.carried]
    bb_ = BOLTS[itf_b.bolt]
    r_h = bb_.head_d / 2 + 0.5
    for x, y in itf_b.holes:
        bw = np.asarray(home_b.point((x, y, 0.0)))
        b_st, b_w = P.st(bw), float((bw - P.O) @ P.a)
        zone = [G2.rect(d["t_b"] - 0.5, d["t_b"] + bb_.head_h + 1.0 + 30.0, b_st[1] - r_h, b_st[1] + r_h)]
        for kind, sl in (("crank_ear", SL["crank_ear_lo"]), ("crank_ear", SL["crank_ear_hi"]),
                         ("rocker_ear", SL["rocker_ear_lo"]), ("rocker_ear", SL["rocker_ear_hi"]), ("latch", SL["latch"])):
            if sl[1] > b_w - r_h and sl[0] < b_w + r_h and G2.shape_dist(car[kind], zone) < 0.3:
                return "the carrier covers one of B's bolt heads or its key's way"
    # the way in: the answer is assembled at home, every part coming straight down; at home B stands beside the
    # mechanism, and the parts in their own layers do not hang over one another (checked by the grader)
    # each screw and nut comes along its pivot's line, from inside the box: the line clear of the base and A
    nut_r = AS.nut_corner_d(AXLE) / 2
    wmin, wmax = L["lo"] - 30.0, L["hi"] + 30.0
    for X_ in (O2, O4, C2h, C4h):
        disk = [G2.circle(X_, nut_r + 0.5, 16)]
        for w in np.arange(wmin, wmax, 3.0):
            sc = scene.at(round(w * 2) / 2)
            if G2.shape_dist(disk, sc["base"] + sc["fixed"]) < 0.3:
                return "a screw's line runs into the frame's base or A"
    # at home the lug sits over the ledge; the drive's last stretch brings it there from the side
    X = np.asarray(d["latch"]["X"])
    d0, C2a, C4a = pth[-4]
    ang, sh = coupler_move(d, C2a, C4a)
    Xa = G2.move([X], ang, sh)[0]
    if Xa[1] < d["latch"]["ledge"] + LUG_H / 2:
        return "the lug comes to the ledge from below"
    # each nut's room (from its stack's inner face, NUT_ROOM along the axis) misses whatever lies in those layers
    nut_r = AS.nut_corner_d(AXLE) / 2
    W = L["W"]
    for P_, stack in ((O2, "crank"), (C2h, "crank"), (O4, "rocker"), (C4h, "rocker")):
        inward = 1.0 if L[stack] < L["latch"] else -1.0
        face = L[stack] + inward * W / 2
        room = sorted((face, face + inward * NUT_ROOM))
        hits = lambda sl: sl[1] > room[0] and sl[0] < room[1]  # noqa: E731
        disk = [G2.circle(P_, nut_r + 1.0)]
        other = "rocker" if stack == "crank" else "crank"
        if hits(SL[other + "_link"]):
            link = G2.stadium(O4, C4h, R_) if other == "rocker" else G2.stadium(O2, C2h, R_)
            if G2.shape_dist(disk, [link]) < 0.5:
                return "a screw's line crosses the other link"
        if hits(SL["latch"]) and G2.shape_dist(disk, fr["latch"] + car["latch"]) < 0.5:
            return "a screw's line crosses the latch"
        if (hits(SL[other + "_ear_lo"]) or hits(SL[other + "_ear_hi"])) and G2.shape_dist(disk, fr[other + "_ear"] + car[other + "_ear"]) < 0.5:
            return "a screw's line crosses the other layer's clevis"
    return None


# --------------------------------------------------------------------------
# the task, the box


POST_LEAN_DEFAULT = 10.0  # deg: a post's sides lean out towards its foot, so where it meets a sloping base no
# crevice is sharper than the clean rules allow (45 deg) on a base sloping up to 55 deg
POST_LEAN = POST_LEAN_DEFAULT  # the setup in use may set its own (``post_lean``; see ``setup_for``)


def _post(plane: bd.Plane, R: float, depth: float, z0: float, z1: float) -> bd.Part:
    """A pivot's post in ``plane`` (local y up): round top of radius R about the pivot, sides leaning out by
    POST_LEAN towards its foot, ``depth`` below the pivot."""
    k = math.tan(math.radians(POST_LEAN)) * depth
    # counter-clockwise, so the face's normal is +z and it extrudes the same way as the round top
    sk = bd.Polygon((-R - k, -depth), (R + k, -depth), (R, 0.0), (-R, 0.0), align=None) + bd.Circle(R)
    return plane * bd.Pos(0, 0, z0) * bd.extrude(sk, z1 - z0)


class Unbuildable(Exception):
    """The draw cannot be built (found while building the reference): the search goes on."""


def _cut_post(setup, body: bd.Part) -> bd.Part:
    """A post cut to stand on the base's faces, out of the fixed fixture's shadow and clear of its bolts."""
    cuts = ([setup.below_roof()] if hasattr(setup, "below_roof") else []) + \
        ([setup.shadow()] if hasattr(setup, "shadow") else []) + \
        (setup.post_keep_out() if hasattr(setup, "post_keep_out") else [])
    for c in cuts:
        try:
            body = body - c
        except ValueError:  # nothing left to cut
            raise Unbuildable("a post is cut away entirely") from None
        if not body.solids() or sum(x.volume for x in body.solids()) < 1.0:
            raise Unbuildable("a post is cut away entirely")
    return body


def _web_top(d: dict, name: str, stack: str, R: float, depth: float) -> float | None:
    """Height (local to the pivot, up) just under the lowest point the link reaches over the post's span, over the
    whole motion; None if it never passes over it."""
    m = d["mech"]
    O = np.asarray(m[name], float)
    k = math.tan(math.radians(POST_LEAN)) * depth
    span = (O[0] - R - k, O[0] + R + k)
    low = -R  # the link's round end about the pivot
    pth = path(d)
    if pth is None:
        return None
    for _, C2, C4 in pth:
        Q = np.asarray(C2 if stack == "crank" else C4, float)
        poly = G2.stadium(O, Q, R + REF_GAP + 0.6, 48)
        for a_, b_ in zip(poly, np.roll(poly, -1, axis=0)):
            for f in np.linspace(0.0, 1.0, 6):
                q = a_ + f * (b_ - a_)
                if span[0] <= q[0] <= span[1]:
                    low = min(low, float(q[1] - O[1]))
    return low


def _web(plane: bd.Plane, R: float, depth: float, top: float, w0: float, w1: float) -> bd.Part:
    """The slot between a post's ears, from its foot up to ``top`` (local, up), its sides leaning as the ears'."""
    tl = math.tan(math.radians(POST_LEAN))
    hb, ht = R + tl * depth, R + tl * max(0.0, -top)
    sk = bd.Polygon((-hb, -depth), (hb, -depth), (ht, top), (-ht, top), align=None)
    return plane * bd.Pos(0, 0, w0) * bd.extrude(sk, w1 - w0)


def _uprofile(plane: bd.Plane, R: float, y_end: float, z0: float, z1: float) -> bd.Part:
    sk = bd.Pos(0, y_end / 2) * bd.Rectangle(2 * R, abs(y_end)) + bd.Circle(R)
    return plane * bd.Pos(0, 0, z0) * bd.extrude(sk, z1 - z0)


def _bar(P: Plane, A_, B_, w: float, R: float, thick: float) -> bd.Part:
    A_, B_ = np.asarray(A_, float), np.asarray(B_, float)
    Ln = float(np.linalg.norm(B_ - A_))
    x = P.s * (B_ - A_)[0] / Ln + P.t * (B_ - A_)[1] / Ln
    pl = bd.Plane(origin=tuple(P.world((A_ + B_) / 2, w)), x_dir=tuple(x), z_dir=tuple(P.a))
    return pl * bd.extrude(bd.SlotCenterToCenter(Ln, 2 * R), thick / 2, both=True)


def build(setup, P: Plane, d: dict, z_bed: float, z_top: float, *, shift_o4: float = 0.0, no_rocker: bool = False,
          drop_bolt: str | None = None, no_ledge: bool = False, tail: bool = False) -> tuple[dict, dict]:
    """The reference at B's HOME pose, every part in the world: ({name: solid}, assembly dict at home).
    Broken copies: ``shift_o4``, ``no_rocker``, ``drop_bolt``, ``no_ledge`` (the latch's post stops short),
    ``tail`` (the crank gets an arm that swings into the frame)."""
    m, L = d["mech"], d["layers"]
    SL = slabs(L)
    R_, t_b = d["Rk"], d["t_b"]
    ab = BOLTS[AXLE]
    itf_b = setup.parts()[setup.carried]
    home_b = setup.home[setup.carried]
    O2, O4, C2, C4 = (np.asarray(m[k], float) for k in ("O2", "O4", "C2", "C4"))
    O4 = O4 + np.array([shift_o4, 0.0])
    # the frame: base, clevis posts, the ledge's post
    frame = setup.base(z_bed, z_top)
    frm = bd.Plane(origin=tuple(P.O), x_dir=tuple(P.s), z_dir=tuple(P.a))  # local y = t (up)
    pivots = [("O2", ["frame", "crank"], O2, "crank"), ("C2", ["crank", "carrier"], C2, "crank")]
    if not no_rocker:
        pivots += [("O4", ["frame", "rocker"], O4, "rocker"), ("C4", ["rocker", "carrier"], C4, "rocker")]
    for name, _, Pp, stack in pivots:
        if not name.startswith("O"):
            continue
        sec = bd.Plane(origin=tuple(P.world(Pp)), x_dir=tuple(P.s), z_dir=tuple(P.a))
        depth = Pp[1] - (d["post_tops"][name] - 25.0)  # down into the base; cut to its faces below
        pieces = [_post(sec, R_, depth, *SL[stack + k]) for k in ("_ear_lo", "_ear_hi")]
        # the slot between the ears filled from the base up to just under where the link ever passes: no crevice
        # where a steep base meets an ear's inner face
        tw = _web_top(d, name, stack, R_, depth)
        if tw is not None and tw > -depth + 1.0 and getattr(setup, "post_webs", True):
            pieces.append(_web(sec, R_, depth, tw, SL[stack + "_ear_lo"][1], SL[stack + "_ear_hi"][0]))
        for ear in pieces:
            ear = _cut_post(setup, ear)
            frame = frame + ear
        frame = frame - sec * bd.Pos(0, 0, L[stack]) * bd.Cylinder(ab.clear_medium / 2, 4 * LE + 2 * LINK_T)
    X = np.asarray(d["latch"]["X"])
    ledge = d["latch"]["ledge"] - (3.0 if no_ledge else 0.0)
    foot = d["latch"]["post_foot"] - 6.0
    w0, w1 = SL["latch"]
    foot = d["latch"]["post_foot"] - 25.0
    k_ = math.tan(math.radians(POST_LEAN)) * (ledge - foot)
    lsec = bd.Plane(origin=tuple(P.world((X[0], ledge))), x_dir=tuple(P.s), z_dir=tuple(P.a))
    post = lsec * bd.Pos(0, 0, w0) * bd.extrude(bd.Polygon((-LUG_S / 2 - k_, foot - ledge), (LUG_S / 2 + k_, foot - ledge),
                                                           (LUG_S / 2, 0.0), (-LUG_S / 2, 0.0), align=None), w1 - w0)
    post = _cut_post(setup, post)
    frame = setup.finish(frame + post, z_bed)
    # the carrier: plate on B (B's face, set back), clevises, the lug's arm
    pb = home_b
    sk = bd.Rectangle(itf_b.face_w - 3.0, itf_b.face_h - 3.0) if itf_b.kind != "round" else bd.Circle(itf_b.face_w / 2 - 1.5)
    carrier = bd.extrude(sk, t_b)
    if hasattr(setup, "carrier_plate"):  # a setup may shape the plate (in B's own axes, its face z = 0)
        carrier = setup.carrier_plate(t_b, L)
    for x, y in itf_b.holes:
        carrier = carrier - bd.Pos(x, y, t_b / 2) * bd.Cylinder(BOLTS[itf_b.bolt].clear_medium / 2, t_b + 2)
    carrier = carrier.moved(pb.location())
    car_pl = bd.Plane(origin=tuple(P.O), x_dir=tuple(P.t), z_dir=tuple(P.a))  # local y = a x t = -s
    for name, _, Pp, stack in pivots:
        if name.startswith("O"):
            continue
        sec = bd.Plane(origin=tuple(P.world(Pp)), x_dir=tuple(P.t), z_dir=tuple(P.a))
        for k in ("_ear_lo", "_ear_hi"):
            w0_, w1_ = SL[stack + k]
            carrier = carrier + _uprofile(sec, R_, Pp[0] - t_b + 0.5, w0_, w1_)
        carrier = carrier - sec * bd.Pos(0, 0, L[stack]) * bd.Cylinder(ab.clear_medium / 2, 4 * LE + 2 * LINK_T)
    s0_ = t_b - 0.5
    arm = car_pl * bd.Pos((X[1] - LUG_H / 2 + X[1] + LUG_H / 2) / 2, -(s0_ + X[0] + LUG_S / 2) / 2, (w0 + w1) / 2) * bd.Box(
        LUG_H, X[0] + LUG_S / 2 - s0_, LUG_T)
    carrier = carrier + arm
    # the links
    bore = ab.d + 2 * REF_GAP
    tail_pt = None
    if tail:
        Rg = d["drive"]["range_deg"]
        Ln = 30.0
        beta = math.radians(-90.0 + Rg / 2)
        tail_pt = O2 + Ln * np.array([math.cos(beta), math.sin(beta)])
    crank = _bar(P, O2, C2, L["crank"], R_, LINK_T)
    if tail_pt is not None:
        crank = crank + _bar(P, O2, tail_pt, L["crank"], R_, LINK_T)
    parts = {"frame": frame, "carrier": carrier, "crank": crank}
    if not no_rocker:
        parts["rocker"] = _bar(P, O4, C4, L["rocker"], R_, LINK_T)
    for k, (A_, B_, stack) in {"crank": (O2, C2, "crank"), "rocker": (O4, C4, "rocker")}.items():
        if k in parts:
            for X_ in (A_, B_):
                parts[k] = parts[k] - bd.Plane(origin=tuple(P.world(X_, L[stack])), z_dir=tuple(P.a)) * bd.Cylinder(
                    bore / 2, LINK_T + 2)
    # the screws, heads on each stack's outer side (away from the latch layer)
    grip, length = GRIP, AXLE_LEN
    fasteners = []
    for name, _, Pp, stack in pivots:
        if name == drop_bolt:
            continue
        out = 1.0 if L[stack] > L["latch"] else -1.0
        head = P.world(Pp, L[stack] + out * (LINK_T / 2 + REF_GAP + LE))
        fasteners.append({"kind": "bolt", "size": AXLE, "length": float(length), "head_at": [float(v) for v in head],
                          "axis": [float(v) for v in -out * P.a], "grip": round(grip, 6),
                          "held_by": "frame" if name.startswith("O") else "carrier"})
    down = [0.0, 0.0, -1.0]
    way_in = getattr(setup, "insert_path", None)  # a setup may bring a part in along a path (moves and turns)
    asm = {"parts": [dict({"name": n, "file": f"{n}.step", "bolts_to": {"frame": setup.fixed, "carrier": setup.carried}.get(n)},
                          **({"path": pth_} if (pth_ := way_in and way_in(n, parts[n])) else {"insert_direction": down}))
                     for n in parts],
           "pivots": [{"name": n, "joins": j, "point": [float(v) for v in P.world(Pp, L[st])], "axis": [float(v) for v in P.a]}
                      for n, j, Pp, st in pivots],
           "drive": {"pivot": "O2", "range_deg": -d["drive"]["range_deg"]},  # from home out to the start
           "fasteners": fasteners}
    return parts, asm


def at_start(setup, P: Plane, d: dict, parts: dict, asm: dict) -> tuple[dict, dict]:
    """Move the home-built reference to the start pose: the carrier with B, each link with its own motion."""
    m = d["mech"]
    O2, O4 = np.asarray(m["O2"]), np.asarray(m["O4"])
    C2h, C4h = np.asarray(m["C2"]), np.asarray(m["C4"])
    C2s, C4s = coupler(d, -d["drive"]["range_deg"])
    ang, sh = coupler_move(d, C2s, C4s)
    mv = {"frame": None, "carrier": P.motion(ang, sh)}

    def link_motion(O, Ch, Cs):
        a0 = math.atan2(*(Ch - O)[::-1])
        a1 = math.atan2(*(Cs - O)[::-1])
        da = a1 - a0
        c, s = math.cos(da), math.sin(da)
        return P.motion(da, O - np.array([[c, -s], [s, c]]) @ O)

    mv["crank"] = link_motion(O2, C2h, C2s)
    mv["rocker"] = link_motion(O4, C4h, C4s)
    out = {n: (s if mv[n] is None else s.moved(mv[n].location())) for n, s in parts.items()}
    asm = json.loads(json.dumps(asm))
    body_of = {"frame": None, "carrier": mv["carrier"], "crank": mv["crank"], "rocker": mv["rocker"]}
    for pv in asm["pivots"]:  # a pivot moves with the second part it joins, which turns about it
        mvp = body_of[pv["joins"][1]] or body_of[pv["joins"][0]]
        if mvp is not None:
            pv["point"] = [float(v) for v in mvp.point(pv["point"])]
    for f in asm["fasteners"]:
        mvf = body_of[f["held_by"]]
        if mvf is not None:
            f["head_at"] = [float(v) for v in mvf.point(f["head_at"])]
            f["axis"] = [float(v) for v in mvf.vec(f["axis"])]
    return out, asm


def box(setup, P: Plane, d: dict, solids_home: dict, z_bed: float) -> tuple[list, dict]:
    """The closed box round everything, in the plane's axes, with one opening in its top wall (sized to let
    every part through, straight down) and windows where the fixed fixture runs through its walls."""
    pts = []
    pth = path(d)
    home_b = setup.home[setup.carried]
    fb = setup.fixed_solids()[setup.carried]
    for deg, C2, C4 in pth[::5] + [pth[-1]]:
        ang, sh = coupler_move(d, C2, C4)
        mv = P.motion(ang, sh)
        for s_ in (fb, solids_home["carrier"]):
            bb = s_.moved(mv.location()).bounding_box()
            pts += [(x, y, z) for x in (bb.min.X, bb.max.X) for y in (bb.min.Y, bb.max.Y) for z in (bb.min.Z, bb.max.Z)]
    on_way = [s_ for k_, s_ in (solids_home.get("_at_start") or {}).items() if k_.endswith(" (on its way)")]
    for s_ in [solids_home[n] for n in ("frame", "crank", "rocker") if n in solids_home] + on_way:
        bb = s_.bounding_box()
        pts += [(x, y, z) for x in (bb.min.X, bb.max.X) for y in (bb.min.Y, bb.max.Y) for z in (bb.min.Z, bb.max.Z)]
    for _, C2, C4 in pth[::5]:  # the links over the motion: their ends' circles
        for Pp in (C2, C4):
            for w in (d["layers"]["lo"], d["layers"]["hi"]):
                for e in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    pts.append(tuple(P.world(np.asarray(Pp) + d["Rk"] * np.array(e), w)))
    # every bolt into a fixture, at home, with room for its hex key to swing all round it (KEY_TURN's key: the
    # long leg sideways, from the head's top up along the bolt by the short leg and the tool's reach)
    from ...catalogue import KEYS

    for n, itf in setup.parts().items():
        pl = setup.home[n]
        frames = [(pl.point(p_), pl.vec(a_)) for p_, a_ in itf.hole_frames] if itf.hole_frames else \
            [(pl.point((x, y, 0.0)), pl.vec((0.0, 0.0, 1.0))) for x, y in itf.holes]
        k = KEYS[itf.bolt]
        for p_, a_ in frames:
            p_, a_ = np.asarray(p_, float), np.asarray(a_, float) / np.linalg.norm(a_)
            e1 = np.cross(a_, [0.0, 0.0, 1.0]) if abs(a_[2]) < 0.9 else np.cross(a_, [1.0, 0.0, 0.0])
            e1 /= np.linalg.norm(e1)
            e2 = np.cross(a_, e1)
            for h in (0.0, 15.0 + k.short_leg):
                for ang in np.linspace(0, 2 * math.pi, 12, endpoint=False):
                    pts.append(tuple(p_ + a_ * h + k.long_leg * (math.cos(ang) * e1 + math.sin(ang) * e2)))
    Q = np.array([[float((np.array(p) - P.O) @ P.s), float((np.array(p) - P.O) @ P.a), float((np.array(p) - P.O) @ P.t)]
                  for p in pts])
    lo = Q.min(axis=0) - BOX_ROOM
    hi = Q.max(axis=0) + BOX_ROOM
    lo[2] = min(lo[2], z_bed - P.O[2] - 1.0)  # the floor under the frame's print bed
    # a part coming in on a slant sweeps sideways on its way down: the side walls stand clear of its path from
    # where it ends up to where it is clear above the top wall (the box's height stays as it is)
    for n, dv in (solids_home.get("_dirs") or {}).items():
        dv = np.asarray(dv, float) / np.linalg.norm(dv)
        down = -float(dv @ P.t)
        if n not in solids_home or down <= 1e-6 or abs(float(dv @ P.s)) + abs(float(dv @ P.a)) < 1e-6:
            continue
        bb = solids_home[n].bounding_box()
        cs = np.array([[float((np.array(c) - P.O) @ v) for v in (P.s, P.a, P.t)]
                       for c in [(x, y, z) for x in (bb.min.X, bb.max.X) for y in (bb.min.Y, bb.max.Y)
                                 for z in (bb.min.Z, bb.max.Z)]])
        lift = (hi[2] + WALL - cs[:, 2].min()) / down
        back = -np.array([float(dv @ P.s), float(dv @ P.a)])
        for f in np.linspace(0.0, 1.0, 9):
            sh = cs[:, :2] + f * lift * back
            lo[:2] = np.minimum(lo[:2], sh.min(axis=0) - BOX_ROOM)
            hi[:2] = np.maximum(hi[:2], sh.max(axis=0) + BOX_ROOM)
    return box_walls(setup, P, d, lo, hi, solids_home)


def _grid_boxes(u0, u1, v0, v1, holes):
    """Rectangle u0..u1 x v0..v1 less the rectangular ``holes`` [(u0, u1, v0, v1)], as rectangles."""
    us = sorted({u0, u1} | {max(u0, min(u1, h[i])) for h in holes for i in (0, 1)})
    vs = sorted({v0, v1} | {max(v0, min(v1, h[i])) for h in holes for i in (2, 3)})
    out = []
    for i in range(len(us) - 1):
        for j in range(len(vs) - 1):
            cu, cv = (us[i] + us[i + 1]) / 2, (vs[j] + vs[j + 1]) / 2
            if us[i + 1] - us[i] < 1e-6 or vs[j + 1] - vs[j] < 1e-6:
                continue
            if any(h[0] <= cu <= h[1] and h[2] <= cv <= h[3] for h in holes):
                continue
            out.append((us[i], us[i + 1], vs[j], vs[j + 1]))
    return out


def box_walls(setup, P: Plane, d: dict, lo, hi, solids_home: dict) -> tuple[list, dict]:
    """Walls as Obstacle boxes in the plane's axes (s, a, t)."""
    rot = tuple(tuple(float(v) for v in row) for row in np.column_stack([P.s, P.a, P.t]))
    w = WALL
    fixed = setup.fixed_solids()[setup.fixed]

    def frame_pt(q):  # (s, a, t) -> world
        return P.O + q[0] * P.s + q[1] * P.a + q[2] * P.t

    walls, holes_by = [], {}
    # the fixed fixture's windows: where it crosses each wall's plane, its section's rectangle + 1 mm
    for axis_i, side, val in ((0, 0, lo[0] - w / 2), (0, 1, hi[0] + w / 2), (1, 0, lo[1] - w / 2), (1, 1, hi[1] + w / 2),
                              (2, 0, lo[2] - w / 2), (2, 1, hi[2] + w / 2)):
        n = [P.s, P.a, P.t][axis_i]
        slab = bd.Plane(origin=tuple(frame_pt([val if k == axis_i else 0.0 for k in range(3)])), z_dir=tuple(n)) * bd.Box(
            4000.0, 4000.0, w + 2.0)
        try:
            cut = fixed & slab
            bbx = cut.bounding_box() if cut.volume > 1e-6 else None
        except Exception:  # noqa: BLE001
            bbx = None
        holes = []
        if bbx is not None:
            cs = np.array([[x, y, z] for x in (bbx.min.X, bbx.max.X) for y in (bbx.min.Y, bbx.max.Y) for z in (bbx.min.Z, bbx.max.Z)])
            Qc = np.array([[float((c - P.O) @ P.s), float((c - P.O) @ P.a), float((c - P.O) @ P.t)] for c in cs])
            others = [k for k in range(3) if k != axis_i]
            holes.append((Qc[:, others[0]].min() - 1.0, Qc[:, others[0]].max() + 1.0, Qc[:, others[1]].min() - 1.0,
                          Qc[:, others[1]].max() + 1.0))
        holes_by[(axis_i, side)] = holes
    # the opening in the top wall: every part's outline where it crosses the wall on its way in, with room
    tops = []
    dirs = solids_home.get("_dirs", {})
    for n, s_ in solids_home.get("_at_start", {}).items():
        bb = s_.bounding_box()
        cs = np.array([[x, y, z] for x in (bb.min.X, bb.max.X) for y in (bb.min.Y, bb.max.Y) for z in (bb.min.Z, bb.max.Z)])
        dv = np.asarray(dirs.get(n, -P.t), float)
        t_top = hi[2]
        proj = []
        for c in cs:
            ct = float((c - P.O) @ P.t)
            mu = (t_top - ct) / float(-dv @ P.t) if float(-dv @ P.t) > 1e-9 else 0.0
            proj.append(c - mu * dv)  # where this corner crossed the top wall's plane
        Qc = np.array([[float((c - P.O) @ P.s), float((c - P.O) @ P.a)] for c in list(cs) + proj])  # (both sets)
        tops.append((Qc[:, 0].min() - OPEN_ROOM, Qc[:, 0].max() + OPEN_ROOM, Qc[:, 1].min() - OPEN_ROOM, Qc[:, 1].max() + OPEN_ROOM))
    opening = (min(t[0] for t in tops), max(t[1] for t in tops), min(t[2] for t in tops), max(t[3] for t in tops)) if tops else None
    if opening is not None:
        opening = (max(opening[0], lo[0]), min(opening[1], hi[0]), max(opening[2], lo[1]), min(opening[3], hi[1]))
        holes_by[(2, 1)].append(opening)
    S0, S1, A0, A1, T0, T1 = lo[0] - w, hi[0] + w, lo[1] - w, hi[1] + w, lo[2] - w, hi[2] + w
    specs = [  # (axis_i, side, plane value range along it, the two other axes' ranges)
        (0, 0, (S0, lo[0]), (A0, A1), (T0, T1)), (0, 1, (hi[0], S1), (A0, A1), (T0, T1)),
        (1, 0, (A0, lo[1]), (lo[0], hi[0]), (T0, T1)), (1, 1, (hi[1], A1), (lo[0], hi[0]), (T0, T1)),
        (2, 0, (T0, lo[2]), (lo[0], hi[0]), (lo[1], hi[1])), (2, 1, (hi[2], T1), (lo[0], hi[0]), (lo[1], hi[1])),
    ]
    names = {(0, 0): "wall_back", (0, 1): "wall_front", (1, 0): "wall_left", (1, 1): "wall_right",
             (2, 0): "floor", (2, 1): "ceiling"}
    for axis_i, side, (p0, p1), r1, r2 in specs:
        for k, (u0, u1, v0, v1) in enumerate(_grid_boxes(r1[0], r1[1], r2[0], r2[1], holes_by[(axis_i, side)])):
            lo3, hi3 = [0.0] * 3, [0.0] * 3
            others = [i for i in range(3) if i != axis_i]
            lo3[axis_i], hi3[axis_i] = p0, p1
            lo3[others[0]], hi3[others[0]] = u0, u1
            lo3[others[1]], hi3[others[1]] = v0, v1
            c = [(a_ + b_) / 2 for a_, b_ in zip(lo3, hi3)]
            walls.append(Obstacle(f"{names[(axis_i, side)]}_{k}", tuple(round(b_ - a_, 6) for a_, b_ in zip(lo3, hi3)),
                                  Placement(tuple(float(v) for v in frame_pt(c)), rot)))
    info = {"origin": [float(v) for v in P.O], "axes": [[float(v) for v in P.s], [float(v) for v in P.a],
                                                        [float(v) for v in P.t]],
            "inside_s": [float(lo[0]), float(hi[0])], "inside_a": [float(lo[1]), float(hi[1])],
            "inside_t": [float(lo[2]), float(hi[2])], "opening_s": list(opening[:2]) if opening else None,
            "opening_a": list(opening[2:]) if opening else None}
    return walls, info


def task_from(setup, P: Plane, d: dict, walls: list, box_info: dict) -> Task:
    """The task, assembled at B's home. B's path runs from the start through pose 1 and pose 2 home; the drive
    takes it from home outwards, so the targets are listed as the drive meets them: pose 2, pose 1, the start."""
    home_b = setup.home[setup.carried]
    R = d["drive"]["range_deg"]
    tg = []
    for name, deg in (("pose2", d["drive"]["targets_at"][1]), ("pose1", d["drive"]["targets_at"][0]), ("start", 0.0)):
        C2, C4 = coupler(d, -R + deg)
        tg.append((name, b_placement(d, P, home_b, C2, C4)))
    parts = setup.parts()
    pose = {setup.fixed: setup.home[setup.fixed], setup.carried: home_b}
    tol = dict(DEFAULT_TOLERANCES, required_checks=list(REQUIRED), seat_setback=1.0, seat_overhang_max=0.5,
               manufacturing={"process": "fdm", "quantity": 1, "direction": None}, moves=dict(L6_TOLERANCES),
               tool_length=30.0, tool_diameter_extra=0.0, score="material")
    load_kg = setup.load_kg
    meta = {"generator": f"moves-l6-{setup.name}", "level_name": "6", "situation": setup.name, "test": "moves",
            "motion": {"fixed": setup.fixed, "moving": setup.carried,
                       "targets": [{"name": n, "placement": {"origin": [float(v) for v in p.origin],
                                                             "rotation": [[float(v) for v in r] for r in p.rot]}}
                                   for n, p in tg],
                       "latch": {"pull_direction": [0.0, 0.0, -1.0], "load_kg": load_kg,
                                 "load_N": round(load_kg * G_ACC, 3)},
                       "box": box_info}}
    t = Task(f"moves-L6-{setup.name}-s{setup.seed}", LEVEL, setup.seed, parts, [pose], tol, meta, [walls],
             GENERATOR_VERSION, family=NAME)
    return t


def choose_ways(setup, P: Plane, parts: dict, asm: dict) -> str | None:
    """Each part's straight way in, in order: straight down if that is clear of the fixtures and the parts already
    in place, else down and across (the plane's own directions), the first that is clear; a part the setup gives
    its own path (``insert_path``) keeps that (the grader judges it). Writes them into ``asm``; returns the name of
    a part with no clear way, or None."""
    from ..sweep import collisions

    fx = setup.fixed_solids()
    placed = {f"fixture {n}": s_ for n, s_ in fx.items()}
    cands = [-P.t] + [(-P.t + k * P.s) / math.hypot(1.0, k) for k in (-0.5, 0.5, -1.0, 1.0, -1.5, 1.5)]
    way_in = getattr(setup, "insert_path", None)  # a setup's own path for a part (e.g. the flange's frame) stands
    for p in asm["parts"]:
        s_ = parts[p["name"]]
        pth_ = way_in(p["name"], s_) if way_in else None
        if pth_:
            p.pop("insert_direction", None)
            p["path"] = pth_
            placed[p["name"]] = s_
            continue
        for c in cands:
            hits = collisions(s_, c, placed)
            if max(hits.values(), default=0.0) <= 0.01:
                p.pop("path", None)
                p["insert_direction"] = [float(v) for v in c]
                break
        else:
            return p["name"]
        placed[p["name"]] = s_
    return None


def write_answer(folder: str, solids: dict, asm: dict) -> str:
    os.makedirs(folder, exist_ok=True)
    for p in asm["parts"]:
        write_step(solids[p["name"]], os.path.join(folder, p["file"]))
    path_ = os.path.join(folder, AS.ASSEMBLY_FILE)
    with open(path_, "w", encoding="utf-8", newline="\n") as f:
        f.write(json.dumps(asm, indent=2) + "\n")
    return path_


def heights(setup, P: Plane) -> tuple[float, float]:
    """(print bed, top) of the frame's base: from below the fixed fixture's lowest bolt to above B's home
    (a setup with its own rule supplies ``heights``)."""
    if hasattr(setup, "heights"):
        return setup.heights(P)
    fx = setup.parts()[setup.fixed]
    pa = setup.home[setup.fixed]
    zs = [float(pa.point(p)[2]) for p, _ in fx.hole_frames] if fx.hole_frames else [float(pa.origin[2])]
    return min(zs) - BOLTS[fx.bolt].head_d / 2 - 6.0, float(P.O[2]) + 10.0  # 6 mm: round5's EDGE


def fits(s_) -> bool:
    """The part fits the Prusa MINI with room: as it stands (the frame prints upright, world up its print
    direction) or in its best orientation (``geometry.fits_build_volume``)."""
    bb = s_.bounding_box()
    upright = sorted([bb.size.X, bb.size.Y, bb.size.Z])
    from ...geometry import BUILD_VOLUME

    if all(v <= w - 2 * BUILD_ROOM + 1e-6 for v, w in zip(upright, sorted(BUILD_VOLUME))):
        return True
    return fits_build_volume(s_, BUILD_ROOM)


def top_bolts(setup) -> float:
    """Just above the fixed fixture's highest bolt head: the least height the frame's base must reach."""
    fx = setup.parts()[setup.fixed]
    pa = setup.home[setup.fixed]
    zs = [float(pa.point(p)[2]) for p, _ in fx.hole_frames] if fx.hole_frames else [float(pa.origin[2])]
    return max(zs) + BOLTS[fx.bolt].head_d / 2 + 10.0


def frozen_path(setup_name: str, seed: int) -> str:
    return os.path.join(os.path.dirname(__file__), "draws", f"{setup_name}-s{seed}.json")


def finish(setup, P: "Plane", d: dict, z_bed: float, z_top_bolts: float, *, check=True, progress=None):
    """From a draw that passed the screens: (task, reference, assembly), or why it does not work."""
    import tempfile

    from .grade import grade

    say = progress or (lambda s: None)
    # the saddle needs to run only as high as the highest bolt head or post foot it carries, and never higher than
    # the base the screens looked at (the links swing low over it)
    z_screened = heights(setup, P)[1]
    z_top = min(z_screened, max(z_top_bolts, P.O[2] + max(list(d["post_tops"].values()) + [d["latch"]["post_foot"]]) + 8.0))
    try:
        home, asm = build(setup, P, d, z_bed, z_top)
    except Unbuildable as e:
        return str(e)
    start = home  # the answer is modelled and assembled at home
    stuck = choose_ways(setup, P, home, asm)
    if stuck:
        return f"no straight way in for the reference's {stuck}"
    # every part modelled cleanly (the grader's quality rules: exact and cheap); a draw whose parts cannot be is
    # dropped, never patched
    from ...quality import check_solid

    for n, s_ in home.items():
        bad = [r for r in check_solid(s_).rules if r.required and not r.ok]
        if bad:
            return f"the reference's {n} is not clean ({bad[0].id}: {bad[0].reason})"
    # each part's straight way in (a part on a path: its first move's direction, for the box's opening)
    d["ways"] = {p["name"]: p["insert_direction"] if "insert_direction" in p else
                 [float(v) for v in np.asarray(p["path"][0]["move"]) / np.linalg.norm(p["path"][0]["move"])]
                 for p in asm["parts"]}
    too_big = [f"{n} {'x'.join(f'{v:.0f}' for v in build_box(s_))}" for n, s_ in start.items() if not fits(s_)]
    if too_big:
        return f"a reference part does not fit the build volume with room ({'; '.join(too_big)} mm)"
    # a part on a path crosses the top wall in its pose before its later steps (the flange's frame: held open)
    at_top = dict(start)
    for p in asm["parts"]:
        if "path" in p and len(p["path"]) > 1:
            from ..grade import _inverse, _step_motion

            later = Placement((0.0, 0.0, 0.0), ((1.0, 0, 0), (0, 1.0, 0), (0, 0, 1.0)))
            for st in p["path"][1:]:
                kind = "move" if "move" in st else "turn"
                stp = ("move", np.asarray(st["move"], float)) if kind == "move" else (
                    "turn", np.asarray(st["turn"]["point"], float), np.asarray(st["turn"]["axis"], float), st["turn"]["deg"])
                later = later.then(_step_motion(stp))
            at_top[p["name"] + " (on its way)"] = start[p["name"]].moved(_inverse(later).location())
            d["ways"][p["name"] + " (on its way)"] = d["ways"][p["name"]]
    walls, info = box(setup, P, d, dict(home, _at_start=at_top, _dirs=d["ways"]), z_bed)
    for k in [k for k in d["ways"] if k.endswith(" (on its way)")]:
        del d["ways"][k]
    task = task_from(setup, P, d, walls, info)
    d["z_bed"], d["z_top"] = z_bed, float(z_top)
    if check:
        with tempfile.TemporaryDirectory() as tmp:
            rep = grade(task, write_answer(tmp, start, asm), progress=say)
        if not rep.passed:
            badc = next(c for c in rep.checks if c.name in rep.failed())
            return f"reference fails {', '.join(rep.failed())} ({badc.name}: {badc.reason})"
    return task, start, asm


def freeze(setup_name: str, seed: int, d: dict) -> str:
    """Store a draw (after its reference passed the whole grader) so ``generate`` rebuilds it directly."""
    path = frozen_path(setup_name, seed)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    keep = {k: v for k, v in d.items() if k not in ("ways", "z_bed", "z_top")}
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        json.dump({"draw": keep}, f, default=float, indent=1)
    return path


def generate(setup_name: str, seed: int = 0, *, check=True, progress=None, max_draws: int = 200000,
             use_frozen: bool = True):
    """(task, draw, reference at home, assembly) for the setup and seed. A draw stored in ``draws/`` (one the
    search found and the full grader passed: see ``freeze``) is rebuilt instead of searched for again."""
    say = progress or (lambda s: None)
    setup = setup_for(setup_name, seed)
    P = Plane(setup.frame())
    z_bed, z_top = heights(setup, P)
    z_top_bolts = top_bolts(setup)
    # a draw found before (the search is deterministic, but slow): rebuilt and, if asked, graded again
    frozen = frozen_path(setup_name, seed) if use_frozen else None
    if frozen is not None and os.path.isfile(frozen):
        with open(frozen, encoding="utf-8") as f:
            d = json.load(f)["draw"]
        got = finish(setup, P, d, z_bed, z_top_bolts, check=check, progress=say)
        if isinstance(got, str):
            raise RuntimeError(f"the stored draw for {setup_name} seed {seed} no longer works: {got}")
        task, start, asm = got
        return task, d, start, asm
    scene = Scene2D(setup, P, z_bed, z_top)
    rng = random.Random(f"{NAME}-L6-{setup_name}-{seed}")
    tried = 0
    draw_ = getattr(setup, "draw", None) or (lambda rng_, P_, sc_: draw(rng_, setup, P_, sc_))  # a setup may draw
    for attempt in range(max_draws):
        d = draw_(rng, P, scene)
        if d is None:
            continue
        why = screen(d, setup, P, scene)
        tried += 1
        if why:
            say(f"draw {attempt}: {why}")
            continue
        home_b = setup.home[setup.carried]
        tg = [setup.home[setup.carried]]
        C2s, C4s = coupler(d, -d["drive"]["range_deg"])
        poses = [b_placement(d, P, home_b, C2s, C4s)]
        for deg in d["drive"]["targets_at"]:
            C2, C4 = coupler(d, -d["drive"]["range_deg"] + deg)
            poses.append(b_placement(d, P, home_b, C2, C4))
        bad = None
        for k in range(1, len(poses)):
            mv = float(np.linalg.norm(np.asarray(poses[k].origin) - np.asarray(poses[k - 1].origin)))
            rel = np.array(poses[k].rot) @ np.array(poses[k - 1].rot).T
            turn = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(rel) - 1) / 2))))
            if mv < POSE_MOVE_MIN and turn < POSE_TURN_MIN:  # distinct if either moved or turned enough
                bad = f"poses {k - 1} and {k} lie too close ({mv:.1f} mm, {turn:.1f} deg)"
        miss = hinge_miss(poses, P)
        if bad is None and miss < HINGE_MISS_MIN:
            bad = f"a single hinge would come within {miss:.1f} mm of every pose"
        if bad:
            say(f"draw {attempt}: {bad}")
            continue
        d["hinge_miss_mm"] = round(miss, 2)
        say(f"draw {attempt}: screens passed; building the reference")
        d["attempt"] = attempt
        got = finish(setup, P, d, z_bed, z_top_bolts, check=check, progress=say)
        if isinstance(got, str):
            say(f"draw {attempt}: {got}")
            continue
        task, start, asm = got
        return task, d, start, asm
    raise RuntimeError(f"no workable draw for {setup_name} seed {seed}")


def write_folder(task: Task, d: dict, start: dict, asm: dict, out_dir: str, broken: list | None = None) -> str:
    """The task folder: what the model sees, and ``_solution/`` (reference and broken copies) beside it."""
    from .brief import brief

    folder = os.path.join(out_dir, task.id)
    if os.path.isdir(folder):
        shutil.rmtree(folder)
    os.makedirs(folder)
    task.root = folder
    with open(os.path.join(folder, "task.json"), "w", encoding="utf-8", newline="\n") as f:
        f.write(task.dumps())
    with open(os.path.join(folder, "README.md"), "w", encoding="utf-8", newline="\n") as f:
        f.write(brief(task))
    fx = task.fixed_solids(0)
    for n, s_ in fx.items():
        write_step(s_, os.path.join(folder, f"fixed_{n}.step"))
    mo = task.meta["motion"]
    b0 = task.poses[0][mo["moving"]]
    for k, tg in enumerate(mo["targets"]):
        pl = Placement.from_json(tg["placement"])
        write_step(fx[mo["moving"]].moved(relative(b0, pl).location()),
                   os.path.join(folder, f"fixed_{mo['moving']}_{tg['name']}.step"))
    if task.obstacles_at(0):
        write_step(bd.Compound([o.solid() for o in task.obstacles_at(0)]), os.path.join(folder, "box.step"))
    from .starter import submission

    with open(os.path.join(folder, "submission.py"), "w", encoding="utf-8", newline="\n") as f:
        f.write(submission())
    sol = os.path.join(folder, "_solution")
    write_answer(sol, start, asm)
    with open(os.path.join(sol, "reference.json"), "w", encoding="utf-8", newline="\n") as f:
        f.write(json.dumps({"seed": task.seed, "draw": d}, indent=2, sort_keys=True, default=float) + "\n")
    for name, (solids_b, asm_b), must, why in broken or []:
        bf = os.path.join(sol, "broken", name)
        write_answer(bf, solids_b, asm_b)
        for fn, txt in (("MUST_FAIL", must), ("WHY", why)):
            with open(os.path.join(bf, fn), "w", encoding="utf-8", newline="\n") as f:
                f.write(txt + "\n")
    return folder


BROKEN = [
    ("misses-pose", {"shift_o4": 3.0}, "POSES", "the rocker's ground pivot sits 3 mm off: the linkage still moves, "
     "along another path"),
    ("clash-mid-motion", {"tail": True}, "MOTION", "the crank has a second arm that swings into the frame during the drive"),
    ("two-free-motions", {"no_rocker": True}, "MOBILITY", "no rocker: B hangs on the crank alone and can move two ways"),
    ("pivot-without-pin", {"drop_bolt": "C4"}, "PIVOTS", "the rocker-to-carrier pivot is declared but has no screw through it"),
    ("no-latch", {"no_ledge": True}, "LATCH", "the latch's ledge stands 3 mm lower: at home B drops onto it"),
]


def broken_variants(setup_name: str, seed: int, d: dict) -> list:
    """[(name, (solids at the start, assembly), check it must fail, what it shows)]."""
    setup = setup_for(setup_name, seed)
    P = Plane(setup.frame())
    z_bed, z_top = d.get("z_bed"), d.get("z_top")
    if z_bed is None:
        z_bed, z_top = heights(setup, P)
    out = []
    for name, opts, must, why in BROKEN:
        parts, asm = build(setup, P, d, z_bed, z_top, **opts)
        for p in asm["parts"]:  # the reference's ways in
            if p["name"] in d.get("ways", {}) and "path" not in p:
                p["insert_direction"] = d["ways"][p["name"]]
        out.append((name, (parts, asm), must, why))
    return out
