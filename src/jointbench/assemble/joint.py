"""The answer's own bolted joint, and the assembly file that states it.

Level 4a's (assembles) answer is two parts. Each part bolts to one fixed part, and the two
bolt to each other: through bolts, the head bearing on one part (the *head
side*), a hex nut on the other (the *nut side*). The assembly file states,
in the task's world coordinates:

    {"parts": [{"name": "part1", "file": "part1.step", "bolts_to": "A",
                "insert_direction": [0, 0, -1]},
               {"name": "part2", "file": "part2.step", "bolts_to": "B",
                "insert_direction": [-1, 0, 0]}],
     "joint": {"bolt": "M5", "head_on": "part2", "axis": [0, 0, 1],
               "positions": [[x, y, z], ...]}}

Parts are put in place in the order listed, each moving along its
``insert_direction`` from outside the scene. Joint positions lie on the face
where the two parts meet; the axis points from the nut side into the head
side, as a fixed part's bolt axis points from the fixed part into the answer.

The joint is judged by the grader's own checks: to each part, the other part
is presented as one more fixed part with a mounting face on the joint plane
and these bolts in it (a nut, on the nut side, in place of the head). So the
holes must line up with the stated positions, each part must lie on the
other around every bolt, the grip must be in range, and head and nut must
have room, exactly as at a fixed part.
"""
from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass

import numpy as np

from ..catalogue import BOLTS, NUTS, Interface, nut_side

JOINT = "J"  # the other part, as a fixed part, in each part's own judgement
MIN_BOLTS = 2
PLANE_TOL = 0.01  # mm: joint positions must lie on one plane square to the axis


class AssemblyError(ValueError):
    pass


@dataclass
class PartEntry:
    name: str
    file: str
    bolts_to: str
    direction: np.ndarray
    path: list = None  # after coming in: [{"move": [..]} | {"turn": {"point", "axis", "degrees"}}], ending in place


@dataclass
class Joint:
    bolt: str
    head_on: str
    axis: np.ndarray  # unit, nut side -> head side
    positions: list  # np arrays on the joint plane
    length: float | None = None  # nominal bolt length, mm (where the task asks for it)

    @property
    def centre(self) -> np.ndarray:
        return np.mean(self.positions, axis=0)


@dataclass
class Assembly:
    parts: list[PartEntry]
    joint: Joint

    def other(self, k: int) -> int:
        return 1 - k


def _vec(v, what) -> np.ndarray:
    try:
        a = np.array([float(x) for x in v])
    except Exception:  # noqa: BLE001
        raise AssemblyError(f"{what} must be three numbers") from None
    if a.shape != (3,):
        raise AssemblyError(f"{what} must be three numbers")
    n = float(np.linalg.norm(a))
    if n < 1e-9:
        raise AssemblyError(f"{what} has length zero")
    return a / n


MAX_SEGMENTS = 12
MAX_TURN = 180.0  # degrees in one turn


def _path(segs, name) -> list:
    if not isinstance(segs, list) or len(segs) > MAX_SEGMENTS:
        raise AssemblyError(f"{name}'s path must be a list of at most {MAX_SEGMENTS} moves and turns")
    out = []
    for i, s in enumerate(segs):
        what = f"{name}'s path step {i + 1}"
        if not isinstance(s, dict) or len(s) != 1 or not ({"move", "turn"} & set(s)):
            raise AssemblyError(f'{what} must be {{"move": [dx, dy, dz]}} or {{"turn": {{"point", "axis", "degrees"}}}}')
        if "move" in s:
            try:
                v = [float(x) for x in s["move"]]
            except Exception:  # noqa: BLE001
                raise AssemblyError(f"{what}: a move is three numbers") from None
            if len(v) != 3:
                raise AssemblyError(f"{what}: a move is three numbers")
            out.append({"move": v})
            continue
        t = s["turn"]
        if isinstance(t, dict) and "deg" in t and "degrees" not in t:
            t = dict(t, degrees=t["deg"])  # "deg" taken for "degrees"
        if not isinstance(t, dict) or not {"point", "axis", "degrees"} <= set(t):
            raise AssemblyError(f"{what}: a turn has a point, an axis and degrees")
        try:
            pt = [float(x) for x in t["point"]]
            deg = float(t["degrees"])
        except Exception:  # noqa: BLE001
            raise AssemblyError(f"{what}: a turn's point is three numbers and its degrees one number") from None
        if len(pt) != 3 or abs(deg) > MAX_TURN:
            raise AssemblyError(f"{what}: point of three numbers, at most {MAX_TURN:g} degrees")
        out.append({"turn": {"point": pt, "axis": [float(x) for x in _vec(t["axis"], what + "'s axis")], "degrees": deg}})
    return out


def load(path: str, fixed_parts, allowed: dict | None = None) -> Assembly:
    """Read and check an assembly file; raises AssemblyError with the reason."""
    if not os.path.exists(path):
        raise AssemblyError(f"no assembly file {os.path.basename(path)}")
    try:
        with open(path, encoding="utf-8") as f:
            d = json.load(f)
    except Exception as e:  # noqa: BLE001
        raise AssemblyError(f"the assembly file is not valid JSON: {e}") from None
    ps = d.get("parts")
    if not isinstance(ps, list) or len(ps) != 2:
        raise AssemblyError("the assembly file must list exactly two parts")
    parts = []
    for i, p in enumerate(ps):
        for key in ("name", "file", "bolts_to"):
            if key not in p:
                raise AssemblyError(f"part {i + 1} has no {key!r}")
        if "insert_direction" not in p and not p.get("path"):
            raise AssemblyError(f"part {i + 1} has neither an 'insert_direction' nor a 'path' (a way in)")
        if p["bolts_to"] not in fixed_parts:
            raise AssemblyError(f"part {p['name']} bolts to {p['bolts_to']!r}, which is not a fixed part "
                                f"({', '.join(fixed_parts)})")
        parts.append(PartEntry(str(p["name"]), str(p["file"]), p["bolts_to"],
                               _vec(p["insert_direction"], f"{p['name']}'s insert_direction")
                               if "insert_direction" in p else None,
                               _path(p.get("path") or [], p["name"])))
    if parts[0].name == parts[1].name:
        raise AssemblyError("the two parts need different names")
    if parts[0].bolts_to == parts[1].bolts_to:
        raise AssemblyError("each part bolts to a different fixed part")
    if set(p.bolts_to for p in parts) != set(fixed_parts):
        raise AssemblyError(f"between them the parts must bolt to every fixed part ({', '.join(fixed_parts)})")
    j = d.get("joint")
    if not isinstance(j, dict):
        raise AssemblyError("the assembly file has no 'joint'")
    for key in ("bolt", "head_on", "axis", "positions"):
        if key not in j:
            raise AssemblyError(f"the joint has no {key!r}")
    sizes = list((allowed or {}).get("sizes") or NUTS)
    if j["bolt"] not in sizes:
        raise AssemblyError(f"joint bolt {j['bolt']!r} is not allowed in this task ({', '.join(sizes)})")
    if j["head_on"] not in (parts[0].name, parts[1].name):
        raise AssemblyError(f"head_on must name one of the parts ({parts[0].name}, {parts[1].name})")
    axis = _vec(j["axis"], "the joint axis")
    try:
        pos = [np.array([float(x) for x in p]) for p in j["positions"]]
    except Exception:  # noqa: BLE001
        raise AssemblyError("joint positions must be points of three numbers") from None
    if any(p.shape != (3,) for p in pos):
        raise AssemblyError("joint positions must be points of three numbers")
    if len(pos) < MIN_BOLTS:
        raise AssemblyError(f"the joint needs at least {MIN_BOLTS} bolts (one bolt lets the parts turn)")
    most = (allowed or {}).get("max_bolts")
    if most and len(pos) > most:
        raise AssemblyError(f"the joint has {len(pos)} bolts; this task allows at most {most}")
    off = max(abs(float((p - pos[0]) @ axis)) for p in pos)
    if off > PLANE_TOL:
        raise AssemblyError(f"joint positions do not lie on one face square to the axis ({off:.3f} mm off; "
                            f"limit {PLANE_TOL:g})")
    b = BOLTS[j["bolt"]]
    for i, p in enumerate(pos):
        for q in pos[i + 1:]:
            gap = float(np.linalg.norm(p - q))
            if gap < b.head_d:
                raise AssemblyError(f"two joint bolts are {gap:.2f} mm apart; heads need {b.head_d:g} mm")
    length = None
    if (allowed or {}).get("length_required") or "length" in j:
        from ..catalogue import lengths

        if "length" not in j:
            raise AssemblyError("the joint has no 'length': the bolts' nominal length (ISO 4762), in mm")
        try:
            length = float(j["length"])
        except Exception:  # noqa: BLE001
            raise AssemblyError("the joint's length must be a number (mm)") from None
        if length not in [float(v) for v in lengths(j["bolt"])]:
            raise AssemblyError(f"no {j['bolt']} bolt is made {length:g} mm long (ISO 4762: "
                                f"{', '.join(str(v) for v in lengths(j['bolt']))})")
    return Assembly(parts, Joint(j["bolt"], j["head_on"], axis, pos, length))


def frame(axis: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Two in-plane unit axes square to ``axis``, chosen the same way every time."""
    ref = np.array([1.0, 0.0, 0.0]) if abs(axis[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    x = ref - axis * float(ref @ axis)
    x /= np.linalg.norm(x)
    return x, np.cross(axis, x)


def joint_interface(asm: Assembly, k: int, other_world, placement_cls):
    """The other part, as part ``k`` sees it: a fixed part whose mounting face
    is the joint plane, facing part k, with the joint bolts in it. Returns
    (interface, placement)."""
    j = asm.joint
    head_side = asm.parts[k].name == j.head_on
    n = j.axis if head_side else -j.axis
    x, _ = frame(j.axis)
    y = np.cross(n, x)
    c = j.centre
    rot = tuple(tuple(float(v) for v in row) for row in np.stack([x, y, n], axis=1))
    pl = placement_cls(tuple(float(v) for v in c), rot)
    holes = [(float((p - c) @ x), float((p - c) @ y)) for p in j.positions]
    us = [h[0] for h in holes]
    vs = [h[1] for h in holes]
    size = j.bolt if head_side else nut_side(j.bolt)
    itf = Interface(kind="solid", params={"step": "", "joint": True, "face": {"size": [
        max(us) - min(us) + 2 * BOLTS[size].head_d, max(vs) - min(vs) + 2 * BOLTS[size].head_d]},
        "description": f"{asm.parts[1 - k].name}, at the joint"},
        bolt=size, holes=holes, face_w=0.0, face_h=0.0,
        description=f"{asm.parts[1 - k].name}, at the joint")
    itf.face_w, itf.face_h = itf.params["face"]["size"]
    itf.solid = other_world.moved(pl.location().inverse())
    return itf, pl


# --------------------------------------------------------------------------
# the force in each joint bolt


def _fibonacci(n: int) -> np.ndarray:
    """``n`` directions spread evenly over the sphere, the same every time."""
    i = np.arange(n) + 0.5
    phi = np.arccos(1 - 2 * i / n)
    th = math.pi * (1 + 5 ** 0.5) * i
    return np.stack([np.cos(th) * np.sin(phi), np.sin(th) * np.sin(phi), np.cos(phi)], 1)


DIRECTIONS = 4000  # load directions tried; the worst one decides


KEY_DEPTH = 8.0  # mm: a key stands at most this far from the joint face
KEY_GAP = 0.5  # mm: a key flank this close to the groove's flank bears on it once the joint takes load
KEY_ARM = 5.0  # mm: a key whose flank reaches this far from the bolts' centre, along the flank, also stops a twist


EDGE_DIRECTIONS = 72  # in-plane directions in which each hole's distance to the part's free edge is measured


def default_sides(asm: Assembly) -> dict:
    """The plastic round each joint bolt when the parts are not measured (tests):
    each part as thin as the grader allows, holes at the coarse clearance, no
    free edge near."""
    b, nut = BOLTS[asm.joint.bolt], NUTS[asm.joint.bolt]
    k = len(asm.joint.positions)
    far = np.full(EDGE_DIRECTIONS, math.inf)
    return {"head": [{"t": b.grip_min, "hole": b.clear_coarse, "outer": b.head_d, "edge": far} for _ in range(k)],
            "nut": [{"t": b.grip_min, "hole": b.clear_coarse, "outer": nut.dw, "edge": far} for _ in range(k)]}


def _edge_at(edge: np.ndarray, d2: np.ndarray) -> np.ndarray:
    """Distance to the free edge in in-plane directions ``d2`` (m, 2): the
    smaller of the two measured directions either side."""
    ang = np.mod(np.arctan2(d2[:, 1], d2[:, 0]), 2 * math.pi) / (2 * math.pi) * len(edge)
    i0 = np.floor(ang).astype(int) % len(edge)
    return np.minimum(edge[i0], edge[(i0 + 1) % len(edge)])


def bolt_forces(asm: Assembly, contact_points: np.ndarray, load_point: np.ndarray, load_N: float,
                load_on_head_side: bool, keys: list | None = None, sides: dict | None = None) -> dict:
    """The force in every joint bolt when the weight ``load_N`` acts at
    ``load_point`` (the loaded fixed part's mount), in each of ``DIRECTIONS``
    directions, and what the printed plastic round each bolt makes of it.

    Statics, not stiffness: the loaded fixed part is bolted to one part only,
    which touches the other only at the joint (JOINT_ROOM makes sure), so the
    joint passes on the whole load and its moment about the joint. The
    joint is split into bolt forces the way a bolt group is designed
    (rigid flanges):

      pulling apart   the force along the axis is shared equally; a moment
                      that would open the joint tips it about the edge of
                      the contact face on the side it presses (the heel),
                      and each bolt takes a share proportional to its
                      distance from that edge;
      pressing        the contact face carries it; the bolts get nothing;
      sliding         the force across the axis is shared equally; a twist
                      about the axis is shared in proportion to each bolt's
                      distance from the bolts' centre.

    Shear keys (``keys``: [{"normal": in-plane unit vector, "arm": mm}], one
    per pair of facing flanks, from ``grade.shear_keys``) take the slide in
    the directions their flanks face -- both directions once their normals
    are not all parallel -- and the twist once one reaches ``KEY_ARM`` from
    the bolts' centre; the bolts then take only what is left.

    The steel never decides: the plastic does (14 November 2026). On
    each side of the joint (head side, nut side), for each bolt, from that
    part's own measured geometry (``sides``: thickness under head or nut,
    hole diameter, bearing diameter of head or nut, distance to the free edge
    per direction):

      pull-through  the pull over the ring under the head (nut) against the
                    allowable bearing stress, and the pull punching a plug
                    of the head's (nut's) diameter through the thickness under
                    it against the allowable shear stress;
      bearing       the slide over shank diameter x that part's thickness,
                    against the allowable bearing stress;
      tear-out      the slide over the two planes that shear out the plastic
                    between the hole and the free edge, in the direction the
                    bolt pushes that part (thickness x (edge distance - hole
                    radius) each), against the allowable shear stress.

    The largest of these ratios is the bolt's utilisation; at most 1 passes.
    ``contact_points``: points on the outline of the contact face. Returns
    per bolt its worst utilisation, which mode and side decide it, and the
    forces at that worst direction."""
    from ..catalogue import plastic_allowables

    j = asm.joint
    n = j.axis
    ex, ey = frame(n)
    c = j.centre
    P = np.array([[(p - c) @ ex, (p - c) @ ey] for p in j.positions])  # bolts, in-plane
    Q = np.array([[(q - c) @ ex, (q - c) @ ey] for q in contact_points]) if len(contact_points) else P
    k = len(P)
    r2 = float((P ** 2).sum())
    allow = plastic_allowables()
    sides = sides or default_sides(asm)
    U = _fibonacci(DIRECTIONS)
    # the load acts on the loaded part's side; what the joint passes on to the
    # other side is that force and its moment about the joint's centre, signed
    # so that "along +axis" pulls the head side away from the nut side
    sign = 1.0 if load_on_head_side else -1.0
    F = sign * load_N * U  # (m, 3) force on the head side, through the joint
    M = np.cross(load_point - c, F)
    N = F @ n
    V = np.stack([F @ ex, F @ ey], 1)
    Mx, My, Mn = M @ ex, M @ ey, M @ n
    keys = keys or []
    kn = np.array([[(np.asarray(q["normal"], float) @ ex), (np.asarray(q["normal"], float) @ ey)] for q in keys]) \
        if keys else np.zeros((0, 2))
    if len(kn):
        kn = kn / np.linalg.norm(kn, axis=1)[:, None]
    span = np.linalg.matrix_rank(kn, tol=0.05) if len(kn) else 0
    key_twist = any(q["arm"] >= KEY_ARM for q in keys)
    if span == 2:
        V = np.zeros_like(V)
    elif span == 1:
        kk = kn[0]
        V = V - np.outer(V @ kk, kk)
    if key_twist:
        Mn = np.zeros_like(Mn)
    m = len(U)
    T = np.tile(np.maximum(N, 0.0)[:, None] / k, (1, k))  # (m, k) pull in each bolt
    mp = np.stack([Mx, My], 1)
    mm = np.linalg.norm(mp, axis=1)
    for i in np.flatnonzero(mm > 1e-12):
        a = mp[i] / mm[i]
        # turning the head side about in-plane axis a moves point r by a x r,
        # whose part along the axis is a_x r_y - a_y r_x: > 0 lifts (opens), < 0 presses
        s_b = a[0] * P[:, 1] - a[1] * P[:, 0]
        heel = float((a[0] * Q[:, 1] - a[1] * Q[:, 0]).min())
        dist = np.maximum(s_b - heel, 0.0)
        den = float((dist ** 2).sum())
        T[i] += mm[i] * dist / den if den > 1e-12 else math.inf
    S = np.tile((V / k)[:, None, :], (1, k, 1))  # (m, k, 2) slide on each bolt, as the head side pushes it
    if r2 > 1e-12:
        S = S + (Mn / r2)[:, None, None] * np.stack([-P[:, 1], P[:, 0]], 1)[None, :, :]
    s_abs = np.linalg.norm(S, axis=2)
    sig, tau = allow["bearing_MPa"], allow["shear_MPa"]
    d = BOLTS[j.bolt].d
    rows = []
    for b in range(k):
        modes = {}
        for side, push in (("head", -1.0), ("nut", 1.0)):  # the bolt pushes the head side back, the nut side on
            g = sides[side][b]
            ring = math.pi / 4 * max(g["outer"] ** 2 - g["hole"] ** 2, 0.0)
            modes[(side, "pull-through (bearing under the " + ("head" if side == "head" else "nut") + ")")] = \
                T[:, b] / (sig * ring) if ring > 0 else np.full(m, math.inf)
            modes[(side, "pull-through (punching through the plastic under the " + ("head" if side == "head" else "nut") + ")")] = \
                T[:, b] / (tau * math.pi * g["outer"] * g["t"])
            modes[(side, "bearing on the hole wall")] = s_abs[:, b] / (sig * d * g["t"])
            dirs = push * S[:, b, :]
            norm = np.maximum(np.linalg.norm(dirs, axis=1), 1e-12)
            e = _edge_at(np.asarray(g["edge"], float), dirs / norm[:, None])
            ligament = np.maximum(e - g["hole"] / 2, 0.0)
            with np.errstate(divide="ignore", invalid="ignore"):
                tear = np.where(s_abs[:, b] > 1e-12, s_abs[:, b] / (2 * tau * g["t"] * ligament), 0.0)
            modes[(side, "tear-out to the free edge")] = tear
        worst_key, worst_i, worst_u = None, 0, -1.0
        for key, arr in modes.items():
            i = int(np.argmax(arr))
            if arr[i] > worst_u:
                worst_key, worst_i, worst_u = key, i, float(arr[i])
        side, mode = worst_key
        part = j.head_on if side == "head" else next(p.name for p in asm.parts if p.name != j.head_on)
        rows.append({"bolt": f"{JOINT}{b + 1}", "utilisation": round(worst_u, 4), "side": side, "part": part,
                     "mode": mode, "tension_N": round(float(T[worst_i, b]), 2),
                     "shear_N": round(float(s_abs[worst_i, b]), 2),
                     "load_direction": [round(float(v), 4) for v in U[worst_i]],
                     "by_mode": {f"{sd} {md}": round(float(arr.max()), 4) for (sd, md), arr in modes.items()}})
    return {"bolts": rows, "allowables": allow, "load_N": load_N, "directions": DIRECTIONS,
            "keys": {"count": len(keys), "slide_taken": ["none", "one direction", "all"][span], "twist_taken": key_twist}}
