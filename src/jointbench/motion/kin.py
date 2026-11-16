"""Mechanisms in parallel planes: the data model and the closed-form solver.

Every part moves in a plane parallel to one mechanism plane. The plane has a
frame in the world (origin, x, y, normal); inside it a part's placement is
three numbers, ``(x, y, a)``: a shift in the plane and a turn about the
normal. Height along the normal never changes, so parts in different height
bands can never touch, and that is what lets links cross in plan view.

A part carries named *features* in its own frame -- pins and holes (round,
for revolute joints) and tongues and slots (two parallel faces, for sliding
joints). A *joint* pairs a feature on one part with a feature on another.

The solver does not search. It follows a short list of *steps*, each of
which places one or two parts from parts already placed, in closed form:

``crank``   the input part turns about a ground pivot to the input angle
``rrr``     two links closing a loop by three pins: circle meets circle
            (the four-bar's coupler and rocker)
``rrp``     a link and a slider: circle meets line (crank-slider)
``psl``     a body on a slide with a slot that a pin must lie in: one
            linear equation (Scotch yoke)
``ppr``     a body sliding on two others plus a turning hub: line meets
            line (Oldham coupling)

Where two answers exist (circle meets circle twice) the step carries a
``mode`` saying which one is the shown assembly; along a sweep the solver
then keeps the answer nearest the previous position, so the mechanism
cannot jump from one assembly to the other. Where no answer exists the
step raises :class:`Jam` with the input value and the shortfall in mm.

An independent check, :func:`numeric_solve`, poses the same mechanism from
its joints alone (no steps) by Newton iteration on all joint equations at
once. The generator requires the two to agree.
"""
from __future__ import annotations

import copy
import math
from dataclasses import dataclass, field

import numpy as np

from ..geometry import Placement, matmul

Pose2 = tuple[float, float, float]  # x, y, angle (rad)
XY = tuple[float, float]


# ---------------------------------------------------------------- plane math


def rot(a: float, p: XY) -> XY:
    c, s = math.cos(a), math.sin(a)
    return (c * p[0] - s * p[1], s * p[0] + c * p[1])


def apply(pose: Pose2, p: XY) -> XY:
    q = rot(pose[2], p)
    return (q[0] + pose[0], q[1] + pose[1])


def inverse(pose: Pose2) -> Pose2:
    t = rot(-pose[2], (-pose[0], -pose[1]))
    return (t[0], t[1], -pose[2])


def compose(a: Pose2, b: Pose2) -> Pose2:
    """``a`` after ``b``: p -> a(b(p))."""
    t = apply(a, (b[0], b[1]))
    return (t[0], t[1], a[2] + b[2])


def sub(a: XY, b: XY) -> XY:
    return (a[0] - b[0], a[1] - b[1])


def cross(a: XY, b: XY) -> float:
    return a[0] * b[1] - a[1] * b[0]


def dot(a: XY, b: XY) -> float:
    return a[0] * b[0] + a[1] * b[1]


def unit(a: float) -> XY:
    return (math.cos(a), math.sin(a))


def wrap(a: float) -> float:
    """Angle into (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def pose_two_points(l1: XY, l2: XY, w1: XY, w2: XY) -> Pose2:
    """The placement that puts local l1 on world w1 and turns l1->l2 onto w1->w2."""
    a = math.atan2(w2[1] - w1[1], w2[0] - w1[0]) - math.atan2(l2[1] - l1[1], l2[0] - l1[0])
    r = rot(a, l1)
    return (w1[0] - r[0], w1[1] - r[1], a)


def pose_to_placement(frame: Placement, pose: Pose2) -> Placement:
    """World placement of a part whose plane pose is ``pose``."""
    c, s = math.cos(pose[2]), math.sin(pose[2])
    local = Placement((pose[0], pose[1], 0.0), ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0)))
    return local.then(frame)


def frame_inverse(frame: Placement) -> Placement:
    rt = tuple(tuple(frame.rot[j][i] for j in range(3)) for i in range(3))
    o = tuple(-sum(rt[i][k] * frame.origin[k] for k in range(3)) for i in range(3))
    return Placement(o, rt)


# ---------------------------------------------------------------- the model


@dataclass
class Feature:
    """A mating feature in a part's own frame.

    ``kind``: ``pin`` (solid cylinder), ``hole`` (bore), ``tongue`` (two
    outward parallel faces), ``slot`` (two inward parallel faces) or
    ``mark`` (a small hole that marks an output point). ``xy`` is the axis
    (round) or a point on the mid-plane (straight); ``dir`` the mid-plane's
    direction in the plane (straight only); ``size`` the diameter or width;
    ``z`` the height band along the normal it occupies."""

    kind: str
    xy: XY
    size: float
    z: tuple[float, float]
    dir: float | None = None
    along: tuple[float, float] | None = None  # extent along a straight feature, from xy
    segments: tuple | None = None  # the real bearing faces along it, as blocks (None: one, = along)

    def to_json(self) -> dict:
        d = {"kind": self.kind, "xy": list(self.xy), "size": self.size, "z": list(self.z)}
        if self.dir is not None:
            d["dir"] = self.dir
        if self.along is not None:
            d["along"] = list(self.along)
        if self.segments is not None:
            d["segments"] = [list(s) for s in self.segments]
        return d

    @staticmethod
    def from_json(d: dict) -> "Feature":
        return Feature(d["kind"], tuple(d["xy"]), d["size"], tuple(d["z"]), d.get("dir"),
                       tuple(d["along"]) if d.get("along") else None,
                       tuple(tuple(s) for s in d["segments"]) if d.get("segments") else None)


@dataclass
class Part:
    id: str
    kind: str  # ground | link | obstacle
    features: dict[str, Feature]
    shown: Pose2 = (0.0, 0.0, 0.0)  # plane pose as written to its STEP file
    recipe: dict | None = None  # how to rebuild its solid (see parts3d.build)
    label: str = ""  # plain name for reports: "the crank", "the housing"
    solid: object = None  # build123d solid in the part's own frame (not stored)

    def to_json(self, with_features: bool = True) -> dict:
        d = {"id": self.id, "kind": self.kind, "label": self.label, "shown": list(self.shown)}
        if with_features:
            d["features"] = {k: f.to_json() for k, f in self.features.items()}
        if self.recipe is not None and with_features:
            d["recipe"] = self.recipe
        return d

    @staticmethod
    def from_json(d: dict) -> "Part":
        return Part(d["id"], d["kind"], {k: Feature.from_json(f) for k, f in d.get("features", {}).items()},
                    tuple(d.get("shown", (0, 0, 0))), d.get("recipe"), d.get("label", ""))


@dataclass
class Joint:
    """``revolute``: two round features share an axis. ``slide``: two straight
    features share a mid-plane (the parts keep their relative angle).
    ``pin_slot``: a pin's axis lies on a slot's mid-plane (a is the pin)."""

    id: str
    kind: str
    a: tuple[str, str]  # (part, feature)
    b: tuple[str, str]

    def feature_of(self, part: str) -> str:
        if self.a[0] == part:
            return self.a[1]
        if self.b[0] == part:
            return self.b[1]
        raise KeyError(f"joint {self.id} does not touch {part}")

    def other(self, part: str) -> tuple[str, str]:
        return self.b if self.a[0] == part else self.a

    def to_json(self) -> dict:
        return {"id": self.id, "kind": self.kind, "a": list(self.a), "b": list(self.b)}

    @staticmethod
    def from_json(d: dict) -> "Joint":
        return Joint(d["id"], d["kind"], tuple(d["a"]), tuple(d["b"]))


@dataclass
class Mechanism:
    type: str
    frame: Placement  # plane coordinates -> world
    parts: dict[str, Part]
    joints: dict[str, Joint]
    steps: list[dict]
    input: dict  # {"kind": "angle", "part", "from": joint, "to": joint} or {"line": joint}
    output: dict  # {"kind": "angle"|"point", "part", ...}
    range: tuple[float, float]  # input range, degrees
    meta: dict = field(default_factory=dict)

    def joint(self, jid: str) -> Joint:
        return self.joints[jid]

    def feat(self, part: str, jid: str) -> Feature:
        return self.parts[part].features[self.joints[jid].feature_of(part)]

    def moving(self) -> list[str]:
        return [p for p, v in self.parts.items() if v.kind == "link"]

    def copy(self) -> "Mechanism":
        m = copy.copy(self)
        m.parts = {k: copy.copy(v) for k, v in self.parts.items()}
        for p in m.parts.values():
            p.features = dict(p.features)
        m.joints = dict(self.joints)
        m.steps = [dict(s) for s in self.steps]
        m.meta = dict(self.meta)
        return m

    def to_json(self, hide: str | None = None) -> dict:
        return {
            "type": self.type,
            "frame": self.frame.to_json(),
            "parts": [p.to_json(with_features=(p.id != hide)) for p in self.parts.values()],
            "joints": [j.to_json() for j in self.joints.values()],
            "steps": self.steps,
            "input": self.input,
            "output": self.output,
            "range": list(self.range),
            "meta": {k: v for k, v in self.meta.items() if k != "solids" and not k.startswith("_")},
        }

    @staticmethod
    def from_json(d: dict) -> "Mechanism":
        return Mechanism(d["type"], Placement.from_json(d["frame"]),
                         {p["id"]: Part.from_json(p) for p in d["parts"]},
                         {j["id"]: Joint.from_json(j) for j in d["joints"]},
                         d["steps"], d["input"], d["output"], tuple(d["range"]), d.get("meta", {}))


# ---------------------------------------------------------------- solving


class Jam(RuntimeError):
    """No position exists at this input value."""

    def __init__(self, value_deg: float, step: str, reason: str, short_by: float = 0.0):
        self.value = value_deg
        self.step = step
        self.reason = reason
        self.short_by = short_by
        super().__init__(f"jams at {value_deg:.2f}: {reason}")


@dataclass
class Solution:
    poses: dict[str, Pose2]
    transmission: dict[str, float]  # step label -> transmission angle (deg)


def world_pt(m: Mechanism, poses: dict, part: str, feat: str) -> XY:
    return apply(poses[part], m.parts[part].features[feat].xy)


def _known(m: Mechanism, poses: dict, jid: str, new: str) -> tuple[str, str]:
    other = m.joints[jid].other(new)
    if other[0] not in poses:
        raise ValueError(f"step order: {other[0]} not placed before joint {jid}")
    return other


def _line_world(m: Mechanism, poses: dict, part: str, feat: str) -> tuple[XY, float]:
    f = m.parts[part].features[feat]
    return apply(poses[part], f.xy), poses[part][2] + f.dir


def _slide_angle(m: Mechanism, poses: dict, jid: str, new: str, ref_angle: float) -> float:
    """Angle of ``new`` so that its straight feature on ``jid`` lies parallel to
    the placed part's; of the two (half a turn apart), the one nearest
    ``ref_angle``."""
    kp, kf = _known(m, poses, jid, new)
    _, kdir = _line_world(m, poses, kp, kf)
    nf = m.parts[new].features[m.joints[jid].feature_of(new)]
    base = kdir - nf.dir
    k = round((ref_angle - base) / math.pi)
    return base + k * math.pi


def _engagement(m: Mechanism, poses: dict, jid: str, value_deg: float, key: str) -> None:
    """A slide joint stays engaged: the shorter of its two straight features
    keeps at least half its length inside the other. Raises Jam."""
    j = m.joints[jid]
    if m.meta.get("_guided_check") in (j.a[0], j.b[0]):
        return  # grading the part on this joint: GUIDED judges it on the real faces instead
    fa, fb = m.parts[j.a[0]].features[j.a[1]], m.parts[j.b[0]].features[j.b[1]]
    if fa.along is None or fb.along is None:
        return
    ca, da = _line_world(m, poses, *j.a)
    cb, _ = _line_world(m, poses, *j.b)
    u = unit(da)
    ta, tb = dot(ca, u), dot(cb, u)
    sa = 1 if math.cos(poses[j.a[0]][2] + fa.dir - da) > 0 else -1
    ia = sorted((ta + sa * fa.along[0], ta + sa * fa.along[1]))
    sb = 1 if math.cos(poses[j.b[0]][2] + fb.dir - da) > 0 else -1
    ib = sorted((tb + sb * fb.along[0], tb + sb * fb.along[1]))
    ov = min(ia[1], ib[1]) - max(ia[0], ib[0])
    shorter = min(ia[1] - ia[0], ib[1] - ib[0])
    if ov < 0.5 * shorter - 1e-9:
        who = m.parts[j.a[0]].label or j.a[0]
        other = m.parts[j.b[0]].label or j.b[0]
        raise Jam(value_deg, key, f"{who} runs off {other}: the shorter of the two is {shorter:.1f} mm end to "
                  f"end, and only {max(ov, 0):.1f} mm of it still overlaps (need {0.5 * shorter:.1f})",
                  0.5 * shorter - ov)


def line_segments(m: Mechanism, poses: dict, part: str, feat: str, u_ang: float, segs) -> list:
    """Blocks (lo, hi) of a straight feature, placed at ``poses``, as
    coordinates along the direction u_ang (world plane)."""
    f = m.parts[part].features[feat]
    c, d = _line_world(m, poses, part, feat)
    u = unit(u_ang)
    t = dot(c, u)
    sg = 1 if math.cos(poses[part][2] + f.dir - u_ang) > 0 else -1
    return sorted(tuple(sorted((t + sg * a, t + sg * b))) for a, b in segs)


def _pin_in_slot(m: Mechanism, poses: dict, jid: str, value_deg: float, key: str) -> None:
    """The pin stays between the ends of its slot. Raises Jam."""
    j = m.joints[jid]
    pin = m.parts[j.a[0]].features[j.a[1]]
    sl = m.parts[j.b[0]].features[j.b[1]]
    if sl.along is None:
        return
    P = world_pt(m, poses, *j.a)
    S, d = _line_world(m, poses, *j.b)
    t = dot(sub(P, S), unit(d))
    r = pin.size / 2
    lo, hi = sl.along
    if t - r < lo - 1e-9 or t + r > hi + 1e-9:
        over = max(lo - (t - r), (t + r) - hi)
        raise Jam(value_deg, key, f"the pin of {m.parts[j.a[0]].label or j.a[0]} reaches the end of the slot in "
                  f"{m.parts[j.b[0]].label or j.b[0]} (past it by {over:.2f} mm)", over)


def input_angle_local(m: Mechanism, inp: dict) -> float:
    """Direction, in the input part's own frame, that the input angle measures."""
    p = m.parts[inp["part"]]
    if "line" in inp:
        return p.features[m.joints[inp["line"]].feature_of(inp["part"])].dir
    a = p.features[m.joints[inp["from"]].feature_of(inp["part"])].xy
    b = p.features[m.joints[inp["to"]].feature_of(inp["part"])].xy
    return math.atan2(b[1] - a[1], b[0] - a[0])


def output_value(m: Mechanism, poses: dict) -> tuple:
    """The output at these poses: (x, y) in the plane, or (angle in degrees,)."""
    o = m.output
    part = o["part"]
    if o["kind"] == "point":
        return world_pt(m, poses, part, o["feature"])
    if "line" in o:
        f = m.parts[part].features[m.joints[o["line"]].feature_of(part)]
        return (math.degrees(poses[part][2] + f.dir),)
    a = world_pt(m, poses, part, m.joints[o["from"]].feature_of(part))
    b = world_pt(m, poses, part, m.joints[o["to"]].feature_of(part))
    return (math.degrees(math.atan2(b[1] - a[1], b[0] - a[0])),)


def solve(m: Mechanism, value_deg: float, state: dict | None = None) -> Solution:
    """Every part's plane pose at input ``value_deg``. ``state`` carries the
    previous solution along a sweep (assembly continuity); pass the same
    dict for consecutive calls. Raises :class:`Jam`."""
    state = {} if state is None else state
    poses: dict[str, Pose2] = {}
    trans: dict[str, float] = {}
    for p in m.parts.values():
        if p.kind != "link":
            poses[p.id] = tuple(p.shown)
    th = math.radians(value_deg)
    for i, st in enumerate(m.steps):
        t = st["type"]
        key = f"s{i}"
        if t == "crank":
            part = st["part"]
            gp, gf = _known(m, poses, st["pivot"], part)
            O = world_pt(m, poses, gp, gf)
            f = m.feat(part, st["pivot"]).xy
            a = th - input_angle_local(m, m.input)
            r = rot(a, f)
            poses[part] = (O[0] - r[0], O[1] - r[1], a)
        elif t == "rrr":
            A, B = st["a"], st["b"]
            kp = _known(m, poses, st["ja"], A)
            kq = _known(m, poses, st["jb"], B)
            P = world_pt(m, poses, *kp)
            Q = world_pt(m, poses, *kq)
            fa, fm = m.feat(A, st["ja"]).xy, m.feat(A, st["jm"]).xy
            gm, gb = m.feat(B, st["jm"]).xy, m.feat(B, st["jb"]).xy
            la, lb = math.dist(fa, fm), math.dist(gm, gb)
            d = math.dist(P, Q)
            if d > la + lb:
                raise Jam(value_deg, key, f"{_lbl(m, A)} and {_lbl(m, B)} cannot reach: "
                          f"pins {d - la - lb:.2f} mm too far apart", d - la - lb)
            if d < abs(la - lb):
                raise Jam(value_deg, key, f"{_lbl(m, A)} and {_lbl(m, B)} cannot close: "
                          f"pins {abs(la - lb) - d:.2f} mm too close", abs(la - lb) - d)
            x = (la * la - lb * lb + d * d) / (2 * d)
            h = math.sqrt(max(la * la - x * x, 0.0))
            u = ((Q[0] - P[0]) / d, (Q[1] - P[1]) / d)
            base = (P[0] + x * u[0], P[1] + x * u[1])
            cands = [(base[0] - h * u[1], base[1] + h * u[0]), (base[0] + h * u[1], base[1] - h * u[0])]
            if key in state:
                M = min(cands, key=lambda c: math.dist(c, state[key]))
            else:
                M = cands[0] if st.get("mode", 1) > 0 else cands[1]
            state[key] = M
            poses[A] = pose_two_points(fa, fm, P, M)
            poses[B] = pose_two_points(gm, gb, M, Q)
            v1, v2 = sub(P, M), sub(Q, M)
            mu = abs(math.degrees(math.atan2(cross(v1, v2), dot(v1, v2))))
            trans[key] = min(mu, 180 - mu)
        elif t == "rrp":
            A, B = st["a"], st["b"]
            kp = _known(m, poses, st["ja"], A)
            P = world_pt(m, poses, *kp)
            gpart, gfeat = _known(m, poses, st["jb"], B)
            Gw, gdir = _line_world(m, poses, gpart, gfeat)
            ref = state.get(key + "a", B_shown_angle(m, B))
            aB = _slide_angle(m, poses, st["jb"], B, ref)
            state[key + "a"] = aB
            bl = m.feat(B, st["jb"]).xy
            bm = m.feat(B, st["jm"]).xy
            u = unit(gdir)
            rr = rot(aB, sub(bm, bl))
            # B's straight feature point sits on the guide line at Gw + s u (+ 0 n)
            M0 = (Gw[0] + rr[0], Gw[1] + rr[1])
            fa, fm = m.feat(A, st["ja"]).xy, m.feat(A, st["jm"]).xy
            la = math.dist(fa, fm)
            w = sub(M0, P)
            wu = dot(w, u)
            disc = wu * wu - dot(w, w) + la * la
            if disc < 0:
                hgt = abs(cross(w, u))
                raise Jam(value_deg, key, f"{_lbl(m, A)} too short to reach {_lbl(m, B)}'s line "
                          f"by {hgt - la:.2f} mm", hgt - la)
            roots = [-wu + math.sqrt(disc), -wu - math.sqrt(disc)]
            if key in state:
                s = min(roots, key=lambda r: abs(r - state[key]))
            else:
                # mode: the side of the crank pin's foot the slider pin lies on
                s = max(roots) if st.get("mode", 1) > 0 else min(roots)
            state[key] = s
            M = (M0[0] + s * u[0], M0[1] + s * u[1])
            rb = rot(aB, bl)
            poses[B] = (Gw[0] + s * u[0] - rb[0], Gw[1] + s * u[1] - rb[1], aB)
            poses[A] = pose_two_points(fa, fm, P, M)
            _engagement(m, poses, st["jb"], value_deg, key)
            rodd = sub(M, P)
            ang = abs(math.degrees(math.atan2(cross(rodd, u), dot(rodd, u))))
            ang = min(ang, 180 - ang)  # rod vs guide; 0 = pushing straight along it
            trans[key] = 90 - ang
        elif t == "psl":
            B = st["b"]
            gpart, gfeat = _known(m, poses, st["js"], B)
            Gw, gdir = _line_world(m, poses, gpart, gfeat)
            ref = state.get(key + "a", B_shown_angle(m, B))
            aB = _slide_angle(m, poses, st["js"], B, ref)
            state[key + "a"] = aB
            u = unit(gdir)
            bl = m.feat(B, st["js"]).xy
            sl = m.parts[B].features[m.joints[st["jp"]].feature_of(B)]
            kp = _known(m, poses, st["jp"], B)
            P = world_pt(m, poses, *kp)
            rr = rot(aB, sub(sl.xy, bl))
            S0 = (Gw[0] + rr[0], Gw[1] + rr[1])
            e = unit(aB + sl.dir)
            den = cross(e, u)
            if abs(den) < 1e-9:
                raise Jam(value_deg, key, f"the slot of {_lbl(m, B)} runs parallel to its guide", 0.0)
            s = cross(e, sub(P, S0)) / den
            rb = rot(aB, bl)
            poses[B] = (Gw[0] + s * u[0] - rb[0], Gw[1] + s * u[1] - rb[1], aB)
            _engagement(m, poses, st["js"], value_deg, key)
            _pin_in_slot(m, poses, st["jp"], value_deg, key)
            trans[key] = math.degrees(math.asin(min(1.0, abs(den))))
        elif t == "ppr":
            B, C = st["b"], st["c"]
            refb = state.get(key + "b", B_shown_angle(m, B))
            aB = _slide_angle(m, poses, st["j1"], B, refb)
            state[key + "b"] = aB
            # C: its angle from the slide with B, its position from its pivot
            poses[B] = (0.0, 0.0, aB)  # provisional, angle only
            refc = state.get(key + "c", B_shown_angle(m, C))
            aC = _slide_angle(m, poses, st["j2"], C, refc)
            state[key + "c"] = aC
            gp, gf = _known(m, poses, st["jc"], C)
            O = world_pt(m, poses, gp, gf)
            fc = rot(aC, m.feat(C, st["jc"]).xy)
            poses[C] = (O[0] - fc[0], O[1] - fc[1], aC)
            # B's translation t: its two straight features on the two lines
            apart, afeat = m.joints[st["j1"]].other(B)
            A1, d1 = _line_world(m, poses, apart, afeat)
            C2, d2 = _line_world(m, poses, C, m.joints[st["j2"]].feature_of(C))
            n1, n2 = unit(d1 + math.pi / 2), unit(d2 + math.pi / 2)
            b1 = rot(aB, m.feat(B, st["j1"]).xy)
            b2 = rot(aB, m.feat(B, st["j2"]).xy)
            r1 = dot(n1, sub(A1, b1))
            r2 = dot(n2, sub(C2, b2))
            det = n1[0] * n2[1] - n1[1] * n2[0]
            if abs(det) < 1e-9:
                raise Jam(value_deg, key, f"the two slides of {_lbl(m, B)} are parallel", 0.0)
            tx = (r1 * n2[1] - n1[1] * r2) / det
            ty = (n1[0] * r2 - r1 * n2[0]) / det
            poses[B] = (tx, ty, aB)
            _engagement(m, poses, st["j1"], value_deg, key)
            _engagement(m, poses, st["j2"], value_deg, key)
            trans[key] = math.degrees(math.asin(min(1.0, abs(det))))
        else:
            hook = STEP_HOOKS.get(t)
            if hook is None:
                raise ValueError(f"unknown step type {t!r}")
            hook(m, st, key, value_deg, poses, state, trans)
    return Solution(poses, trans)


# contact-driven steps (cam, Geneva, ratchet) register here; see contact.py
STEP_HOOKS: dict = {}


def B_shown_angle(m: Mechanism, part: str) -> float:
    return m.parts[part].shown[2]


def _lbl(m: Mechanism, part: str) -> str:
    return m.parts[part].label or part


def sweep(m: Mechanism, values: list[float]):
    """Solve at each input value in order, keeping assembly continuity.
    Returns (solutions, jam): the solutions reached, and the Jam if one stopped it."""
    state: dict = {}
    out = []
    for v in values:
        try:
            out.append(solve(m, v, state))
        except Jam as j:
            return out, j
    return out, None


def input_values(m: Mechanism, step_deg: float) -> list[float]:
    a, b = m.range
    n = max(1, int(round((b - a) / step_deg)))
    return [a + (b - a) * i / n for i in range(n + 1)]


# ---------------------------------------------------------------- numeric check


def _residuals(m: Mechanism, q: np.ndarray, names: list[str], fixed: dict, value_deg: float) -> np.ndarray:
    poses = dict(fixed)
    for i, n in enumerate(names):
        poses[n] = (q[3 * i], q[3 * i + 1], q[3 * i + 2])
    r = []
    for j in m.joints.values():
        if j.kind == "revolute":
            pa = world_pt(m, poses, *j.a)
            pb = world_pt(m, poses, *j.b)
            r += [pa[0] - pb[0], pa[1] - pb[1]]
        elif j.kind == "slide":
            A, da = _line_world(m, poses, *j.a)
            B, db = _line_world(m, poses, *j.b)
            r += [math.sin(da - db), cross(unit(db), sub(A, B))]
        elif j.kind == "pin_slot":
            P = world_pt(m, poses, *j.a)
            S, ds = _line_world(m, poses, *j.b)
            r += [cross(unit(ds), sub(P, S))]
        elif j.kind == "contact":
            continue  # contact joints are not equations; numeric check skips them
    inp = m.input
    a_local = input_angle_local(m, inp)
    r.append(wrap(poses[inp["part"]][2] + a_local - math.radians(value_deg)))
    return np.array(r)


def numeric_solve(m: Mechanism, values: list[float], start: dict[str, Pose2] | None = None,
                  tol: float = 1e-11, max_iter: int = 30):
    """Pose the mechanism from its joints alone by Newton iteration on every
    joint equation at once, continuing from the previous input value. No
    closed-form step is used. Returns a list of pose dicts (None where it
    failed to converge)."""
    names = m.moving()
    fixed = {p.id: tuple(p.shown) for p in m.parts.values() if p.kind != "link"}
    start = start or {n: m.parts[n].shown for n in names}
    q = np.array([v for n in names for v in start[n]], dtype=float)
    out = []
    for v in values:
        ok = False
        for _ in range(max_iter):
            F = _residuals(m, q, names, fixed, v)
            if np.max(np.abs(F)) < tol:
                ok = True
                break
            J = np.empty((len(F), len(q)))
            h = 1e-7
            for k in range(len(q)):
                dq = np.zeros_like(q)
                dq[k] = h
                J[:, k] = (_residuals(m, q + dq, names, fixed, v) - _residuals(m, q - dq, names, fixed, v)) / (2 * h)
            step = np.linalg.lstsq(J, -F, rcond=None)[0]
            q = q + step
        if not ok:
            F = _residuals(m, q, names, fixed, v)
            ok = np.max(np.abs(F)) < 1e-8
        out.append({n: (q[3 * i], q[3 * i + 1], q[3 * i + 2]) for i, n in enumerate(names)} if ok else None)
    return out


def agreement(m: Mechanism, step_deg: float = 5.0) -> dict:
    """Largest difference between the closed-form and the numeric solve over
    the input range: in mm at every joint feature, and in degrees."""
    vals = input_values(m, step_deg)
    sols, jam = sweep(m, vals)
    if jam is not None:
        return {"ok": False, "reason": str(jam)}
    if any(j.kind == "contact" for j in m.joints.values()):
        return {"ok": None, "reason": "contact-driven: no joint equations to cross-check"}
    num = numeric_solve(m, vals, start=sols[0].poses)
    worst_mm, worst_deg, where = 0.0, 0.0, None
    for v, s, n in zip(vals, sols, num):
        if n is None:
            return {"ok": False, "reason": f"numeric solve did not converge at {v:.1f}"}
        for pid in n:
            dd = abs(math.degrees(wrap(s.poses[pid][2] - n[pid][2])))
            worst_deg = max(worst_deg, dd)
            for f in m.parts[pid].features.values():
                e = math.dist(apply(s.poses[pid], f.xy), apply(n[pid], f.xy))
                if e > worst_mm:
                    worst_mm, where = e, v
    return {"ok": worst_mm < 1e-6 and worst_deg < 1e-6, "mm": worst_mm, "deg": worst_deg,
            "at": where, "n": len(vals)}


__all__ = ["Feature", "Part", "Joint", "Mechanism", "Jam", "Solution", "solve", "sweep",
           "numeric_solve", "agreement", "input_values", "output_value", "pose_to_placement",
           "frame_inverse", "apply", "compose", "inverse", "matmul"]
