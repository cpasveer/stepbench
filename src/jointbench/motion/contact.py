"""Shape that drives the motion: cam and follower, Geneva drive, ratchet and pawl.

Here a part's position is not fixed by pins but by touching a profile. At
each input step it is found geometrically, by *advancing until touching*:

* **cam**: the roller follower is pushed towards the cam along its guide
  until the roller's circle first touches the cam, found by stepping the
  roller in by exactly its current clearance (the distance to the cam,
  measured exactly on the cam's boundary), which can never overshoot.
* **Geneva**: the wheel stays where it is unless the driver's pin would be
  inside it; then it is turned, in its driven direction, by the least angle
  that frees the pin (bisection to 1e-7 rad).
* **ratchet**: the pawl rests on the wheel (a spring is assumed). When the
  wheel turns into it, the pawl is lifted by the least angle that frees it,
  and the contact normal at that touch says whether the push lifts the pawl
  (it rides over) or drives it into the wheel (it locks).

Distances are measured with OpenCASCADE's exact distance between shapes, on
thin horizontal slices of the parts at the contact height. Exactness is
stated per mechanism in docs/motion.md.
"""
from __future__ import annotations

import json
import math
import os
import random
import time

import build123d as bd
import numpy as np
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeVertex
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.gp import gp_Pnt, gp_Trsf, gp_Vec, gp_Ax1, gp_Dir
from OCP.TopLoc import TopLoc_Location

from ..checks.base import CheckResult
from ..geometry import Placement, write_step
from . import kin
from .generate import _random_frame
from .grade import DEFAULT_FITS, Report, _skip_rest
from .grade import max_move as _max_move
from .interfere import Scene
from .kin import Feature, Joint, Mechanism, Part
from .measure import all_features
from .outline import DEFLECTION, Outline, outline_gap
from .mechs import BASE_T, BASE_TOP, BOSS, HOLE, PIN, U, band, pin_top
from .parts3d import build

GEN_VERSION = 1
TABLE_STEP = 10.0
DRIVE_D, DRIVE_HOLE = 4.0, 4.4


# ---------------------------------------------------------------- geometry helpers




def slice_at(solid, z: float, t: float = 0.02):
    bb = solid.bounding_box()
    box = bd.Pos((bb.min.X + bb.max.X) / 2, (bb.min.Y + bb.max.Y) / 2, z) * bd.Box(
        bb.max.X - bb.min.X + 2, bb.max.Y - bb.min.Y + 2, t)
    s = solid & box
    return s


def _trsf(pose) -> TopLoc_Location:
    tr = gp_Trsf()
    tr.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), pose[2])
    t2 = gp_Trsf()
    t2.SetTranslation(gp_Vec(pose[0], pose[1], 0))
    return TopLoc_Location(t2.Multiplied(tr))


def moved(shape, pose):
    return shape.wrapped.Moved(_trsf(pose))


def dist(a, b):
    d = BRepExtrema_DistShapeShape(a, b)
    d.Perform()
    return d.Value(), d


def point_dist(shape_w, p):
    v = BRepBuilderAPI_MakeVertex(gp_Pnt(*p)).Vertex()
    d = BRepExtrema_DistShapeShape(v, shape_w)
    d.Perform()
    return d.Value(), d


def two_pin_pose(local_a, local_b, world_a, world_b, value, what, tol=0.15):
    la, wa = math.dist(local_a, local_b), math.dist(world_a, world_b)
    if abs(la - wa) > tol:
        raise kin.Jam(value, "fit", f"{what}: its two holes are {la:.2f} mm apart, the pins {wa:.2f} mm "
                      f"(off by {abs(la - wa):.2f})", abs(la - wa))
    return kin.pose_two_points(local_a, local_b, world_a, world_b)


def _mech(mtype, rng, parts, joints, rng_range, output, knobs) -> Mechanism:
    m = Mechanism(mtype, None, parts, joints, [{"type": "crank", "part": "driver", "pivot": "j_d0"}],
                  {"kind": "angle", "part": "driver", "from": "j_d0", "to": "j_d1"}, output, rng_range)
    m.frame = _random_frame(rng) if knobs.get("placed", "anywhere") == "anywhere" else \
        Placement((0.0, 0.0, 0.0), ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)))
    return m


def _driver(r_dowel: float, layer: int, dowel_to: float, extra_plan=None, radius=None):
    """A driving disc on a post, with a dowel pin pointing down into the part below."""
    b = band(layer)
    feats = {"bore": Feature("hole", (0.0, 0.0), HOLE, b),
             "dowel": Feature("pin", (r_dowel, 0.0), DRIVE_D, (dowel_to, b[1]))}
    rec = {"z": list(b), "plan": [{"disc": [[0.0, 0.0], radius or (r_dowel + 6)]}] + (extra_plan or []),
           "holes": [[0.0, 0.0, HOLE]], "pins": [[r_dowel, 0.0, DRIVE_D, dowel_to, b[1]]]}
    return feats, rec


def _base(parts_xy_r, posts, extra_add=None, top=13.5):
    xs = [p[0] - p[2] for p in parts_xy_r] + [p[0] + p[2] for p in parts_xy_r]
    ys = [p[1] - p[2] for p in parts_xy_r] + [p[1] + p[2] for p in parts_xy_r]
    x0, x1, y0, y1 = min(xs) - 10, max(xs) + 10, min(ys) - 10, max(ys) + 10
    return {"z": [BASE_TOP - BASE_T, BASE_TOP], "plan": [{"box": [(x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0, 0.0]}],
            "add": extra_add or [], "pins": [[x, y, PIN, BASE_TOP - 4, t] for (x, y, t) in posts]}


# ---------------------------------------------------------------- cam


def _cam_lift(D, h, b1, b2, b3):
    """Cycloidal rise over b1, dwell b2, cycloidal return over b3, then dwell."""
    D = D % (2 * math.pi)
    if D < b1:
        x = D / b1
        return h * (x - math.sin(2 * math.pi * x) / (2 * math.pi))
    if D < b1 + b2:
        return h
    if D < b1 + b2 + b3:
        x = (D - b1 - b2) / b3
        return h * (1 - (x - math.sin(2 * math.pi * x) / (2 * math.pi)))
    return 0.0


def draw_cam(rng, knobs):
    Rb, rho, h = U(rng, 22, 30), U(rng, 5, 7), U(rng, 6, 12)
    b1, b2, b3 = (math.radians(U(rng, 110, 150)), math.radians(U(rng, 20, 60)), math.radians(U(rng, 110, 150)))
    alpha = U(rng, 0, 2 * math.pi)
    th0 = U(rng, 0, 360)
    u = kin.unit(alpha)
    b0, b1z = band(0), band(1)
    # pitch curve and profile in the cam's own frame (cam frame = its pose at the first input value)
    N = 720
    P, prof = [], []
    for i in range(N):
        D = 2 * math.pi * i / N
        r = Rb + rho + _cam_lift(D, h, b1, b2, b3)
        # the cam's own frame sits turned by th0 at the first input value (its drive hole on the dowel)
        a_loc = alpha - math.radians(th0) - D
        P.append((r * math.cos(a_loc), r * math.sin(a_loc)))
    P = np.array(P)
    T = (np.roll(P, -1, axis=0) - np.roll(P, 1, axis=0))
    T /= np.hypot(T[:, 0], T[:, 1])[:, None]
    nrm = np.stack([T[:, 1], -T[:, 0]], axis=1)
    out = np.sign(np.sum(nrm * P, axis=1))[:, None]
    nrm = nrm * out
    prof = P - rho * nrm
    # pressure angle and curvature gates on the pitch curve
    radial = P / np.hypot(P[:, 0], P[:, 1])[:, None]
    # follower moves along u; at cam angle D the line is radial in the cam frame
    press = np.degrees(np.arccos(np.clip(np.abs(np.sum(nrm * radial, axis=1)), 0, 1)))
    meta = {"Rb": Rb, "rho": rho, "h": h, "b": [b1, b2, b3], "alpha": alpha, "max_pressure": float(press.max())}
    r_dowel = 10.0
    cam_feats = {"bore": Feature("hole", (0.0, 0.0), HOLE, b0),
                 "drive": Feature("hole", (r_dowel, 0.0), DRIVE_HOLE, b0)}
    cam_rec = {"z": list(b0), "plan": [{"spline": prof.tolist()}], "holes": [[0.0, 0.0, HOLE], [r_dowel, 0.0, DRIVE_HOLE]]}
    drv_feats, drv_rec = _driver(r_dowel, 1, b0[0])
    # follower: roller centre at its local origin, block behind it along +x
    s_min, s_max = Rb + rho, Rb + rho + h
    xr = Rb + h + 2 * rho + 2.0  # rails start clear of the cam and of the roller at full lift
    Lb = (xr - s_min) + 22.0
    fol_feats = {"tongue": Feature("tongue", (Lb / 2 + 1.0, 0.0), 10.0, b0, 0.0, (-Lb / 2, Lb / 2)),
                 "roller": Feature("pin", (0.0, 0.0), 2 * rho, b0)}
    fol_rec = {"z": list(b0), "plan": [{"disc": [[0.0, 0.0], rho]}, {"box": [Lb / 2 + 1.0, 0.0, Lb, 10.0, 0.0]}]}
    g_lo, g_hi = xr, s_max + 1.0 + Lb + 2.0
    ch = 10.4
    rails = []
    for sy in (1, -1):
        cx, cy = (g_lo + g_hi) / 2, sy * (ch / 2 + 3)
        rails.append({"box3": [cx * u[0] - cy * u[1], cx * u[1] + cy * u[0], (BASE_TOP - 2 + b0[1]) / 2,
                               g_hi - g_lo, 6.0, b0[1] - BASE_TOP + 2, math.degrees(alpha)]})
    gc = ((g_lo + g_hi) / 2) * u[0], ((g_lo + g_hi) / 2) * u[1]
    gnd_feats = {"post": Feature("pin", (0.0, 0.0), PIN, (BASE_TOP, pin_top(1))),
                 "guide": Feature("slot", gc, ch, (BASE_TOP, b0[1]), alpha, (-(g_hi - g_lo) / 2, (g_hi - g_lo) / 2))}
    far = g_hi + 5
    gnd_rec = _base([(0, 0, Rb + h + 12), (far * u[0], far * u[1], 12)], [(0.0, 0.0, pin_top(1))], rails)
    parts = {
        "ground": Part("ground", "ground", gnd_feats, label="the frame", recipe=gnd_rec),
        "driver": Part("driver", "link", drv_feats, label="the driving disc", recipe=drv_rec),
        "cam": Part("cam", "link", cam_feats, label="the cam"),
        "follower": Part("follower", "link", fol_feats, (0.0, 0.0, alpha), label="the follower", recipe=fol_rec),
    }
    parts["cam"].recipe = cam_rec
    joints = {j.id: j for j in [
        Joint("j_d0", "revolute", ("ground", "post"), ("driver", "bore")),
        Joint("j_d1", "revolute", ("driver", "dowel"), ("cam", "drive")),
        Joint("j_c0", "revolute", ("ground", "post"), ("cam", "bore")),
        Joint("j_f0", "slide", ("follower", "tongue"), ("ground", "guide")),
        Joint("j_fc", "contact", ("follower", "roller"), ("cam", "bore")),
    ]}
    m = _mech("cam", rng, parts, joints, (th0, th0 + 360.0), {"kind": "point", "part": "follower", "feature": "roller"}, knobs)
    m.meta["cam"] = meta
    m.meta["removed"] = "cam"
    return m


def solve_cam(m: Mechanism, solids: dict, values: list[float]):
    """Poses of every part at each input value; the follower by contact."""
    rz = m.parts["follower"].features["roller"].z
    from .outline import z_levels
    lv = z_levels(solids["cam"])
    if lv is not None and not any(rz[0] < z < rz[1] for z in lv):
        zs = [(rz[0] + rz[1]) / 2]  # a flat cam: one outline serves the roller's whole height
    else:
        zs = [rz[0] + 0.5, (rz[0] + rz[1]) / 2, rz[1] - 0.5]
    outs = [o for o in (Outline(solids["cam"], z) for z in zs) if len(o.S)]
    if not outs:
        raise kin.Jam(values[0], "contact", "the cam has no material at the roller's height", 0.0)
    rho = m.parts["follower"].features["roller"].size / 2
    g = m.parts["ground"].features["guide"]
    u = kin.unit(g.dir)
    fol = m.parts["follower"]
    reach = _cam_reach(m, solids)
    out = []
    worst_press, at = 0.0, None
    prev = {}
    last_v = None
    for v in values:
        poses = {p: m.parts[p].shown for p in m.parts if m.parts[p].kind != "link"}
        poses.update(kin.solve(_driver_only(m), v, {}).poses)
        O = kin.world_pt(m, poses, "ground", "post")
        # a warm start: the cam's boundary moves at most reach * turn between steps
        move = reach * abs(math.radians(v - last_v)) if last_v is not None else None
        last_v = v
        D = kin.world_pt(m, poses, "driver", "dowel")
        cf = m.parts["cam"].features
        poses["cam"] = two_pin_pose(cf["bore"].xy, cf["drive"].xy, O, D, v, "the cam")
        inv = kin.inverse(poses["cam"])
        best_s, best_n = None, None
        for k, o in enumerate(outs):
            s_i = reach + rho + 2.0
            if move is not None and k in prev:
                s_try = prev[k] + 4 * move + 0.05  # covers pressure angles up to 75 deg
                pc = kin.apply(inv, (O[0] + s_try * u[0], O[1] + s_try * u[1]))
                if not o.inside(pc)[0] and o.closest(pc)[0] - rho > 0:
                    s_i = s_try
            for _ in range(5000):  # step in by the clearance: never passes the first touch
                pc = kin.apply(inv, (O[0] + s_i * u[0], O[1] + s_i * u[1]))
                d, q = o.closest(pc)
                gap = d - rho
                if gap < 1e-7:
                    break
                s_i -= gap
            prev[k] = s_i
            if best_s is None or s_i > best_s:
                nv = kin.rot(poses["cam"][2], (pc[0] - q[0], pc[1] - q[1]))
                nn = math.hypot(*nv) or 1.0
                best_s, best_n = s_i, (nv[0] / nn, nv[1] / nn)
        press = math.degrees(math.acos(min(1.0, abs(best_n[0] * u[0] + best_n[1] * u[1]))))
        if press > worst_press:
            worst_press, at = press, v
        c = (O[0] + best_s * u[0], O[1] + best_s * u[1])
        r0 = kin.rot(fol.shown[2], fol.features["roller"].xy)
        poses["follower"] = (c[0] - r0[0], c[1] - r0[1], fol.shown[2])
        out.append(kin.Solution(poses, {}))
    return out, (worst_press, at)


def _cam_reach(m, solids):
    bb = solids["cam"].bounding_box()
    return max(abs(bb.min.X), abs(bb.max.X), abs(bb.min.Y), abs(bb.max.Y)) * math.sqrt(2)


def _driver_only(m: Mechanism) -> Mechanism:
    d = m.meta.get("_driver_only")
    if d is not None and d.parts is not None and d.parts.get("driver") is m.parts.get("driver"):
        return d
    d = m.copy()
    d.parts = m.parts
    d.steps = [{"type": "crank", "part": "driver", "pivot": "j_d0"}]
    m.meta["_driver_only"] = d
    return d


# ---------------------------------------------------------------- Geneva


def draw_geneva(rng, knobs):
    n = rng.choice([4, 5, 6])
    a = U(rng, 22, 30)
    C = a / math.sin(math.pi / n)
    Rw = math.sqrt(C * C - a * a)
    beta = 0.6 * math.pi / n
    Rl = math.sqrt(C * C + Rw * Rw - 2 * C * Rw * math.cos(beta))
    gam = U(rng, 0, 2 * math.pi)  # direction from driver centre to wheel centre
    O1, O2 = (0.0, 0.0), (C * math.cos(gam), C * math.sin(gam))
    b0, b1 = band(0), band(1)
    th0 = math.degrees(gam + math.pi)  # pin pointing away from the wheel: mid-dwell
    # driver: locking disc (layer 0) with a cut-out for the wheel's horns, arm (layer 1), pin down
    # the classic clearance cut: one arc centred where the wheel centre is when the pin is on the line of centres
    cut = [{"disc": [[C, 0.0], Rw + 1.0]}]
    drv_feats = {"bore": Feature("hole", (0.0, 0.0), HOLE, (b0[0], b1[1])),
                 "pin": Feature("pin", (a, 0.0), PIN, (b0[0], b1[1]))}
    drv_rec = {"z": list(b1), "plan": [{"capsule": [[0.0, 0.0], [a, 0.0], BOSS]}],
               "add": [{"cyl": [0.0, 0.0, 2 * Rl, b0[0], b0[1]]}, {"cyl": [0.0, 0.0, BOSS, b0[0], b1[1]]}], "cut": [],
               "holes": [[0.0, 0.0, HOLE]], "pins": [[a, 0.0, PIN, b0[0], b1[1]]], "lock_cut": cut}
    r_in = C - a - 4.0
    slot_len = Rw + 4 - r_in
    wheel_cut = []
    for k in range(n):
        ph = math.pi + gam + (2 * k + 1) * math.pi / n  # slots at +-pi/n from the line to the driver
        c = (r_in + slot_len / 2)
        wheel_cut.append({"box3": [c * math.cos(ph), c * math.sin(ph), (b0[0] + b0[1]) / 2, slot_len, HOLE,
                                   b0[1] - b0[0] + 2, math.degrees(ph)]})
        pb = math.pi + gam + 2 * k * math.pi / n  # locking arcs face the driver between slots
        wheel_cut.append({"cyl": [C * math.cos(pb), C * math.sin(pb), 2 * (Rl + 0.3), b0[0] - 1, b0[1] + 1]})
    wh_feats = {"bore": Feature("hole", (0.0, 0.0), HOLE, b0)}
    wh_rec = {"z": list(b0), "plan": [{"disc": [[0.0, 0.0], Rw]}], "cut": wheel_cut, "holes": [[0.0, 0.0, HOLE]]}
    gnd_feats = {"p1": Feature("pin", O1, PIN, (BASE_TOP, pin_top(1))), "p2": Feature("pin", O2, PIN, (BASE_TOP, pin_top(0)))}
    gnd_rec = _base([(0, 0, max(a + 8, Rl)), (O2[0], O2[1], Rw)], [(O1[0], O1[1], pin_top(1)), (O2[0], O2[1], pin_top(0))])
    parts = {
        "ground": Part("ground", "ground", gnd_feats, label="the frame", recipe=gnd_rec),
        "driver": Part("driver", "link", drv_feats, label="the driver", recipe=drv_rec),
        "wheel": Part("wheel", "link", wh_feats, (O2[0], O2[1], 0.0), label="the Geneva wheel", recipe=wh_rec),
    }
    joints = {j.id: j for j in [
        Joint("j_d0", "revolute", ("ground", "p1"), ("driver", "bore")),
        Joint("j_w0", "revolute", ("ground", "p2"), ("wheel", "bore")),
        Joint("j_d1", "contact", ("driver", "pin"), ("wheel", "bore")),
    ]}
    m = _mech("geneva", rng, parts, joints, (th0, th0 + 360.0), {"kind": "angle", "part": "wheel", "relative": True}, knobs)
    m.meta["geneva"] = {"n": n, "a": a, "C": C, "Rw": Rw, "Rl": Rl, "gam": gam, "sense": -1}
    m.meta["removed"] = "wheel"
    return m


def build_driver(rec):
    r = dict(rec)
    cut = r.pop("lock_cut", [])
    s = build({k: v for k, v in r.items()})
    if cut:
        b0 = band(0)
        face = None
        from .parts3d import _face
        for sh in cut:
            f = _face(sh)
            face = f if face is None else face + f
        s = (s - bd.Pos(0, 0, b0[0] - 0.5) * bd.extrude(face, b0[1] - b0[0] + 1, dir=(0, 0, 1))).solids()[0]
    return s


def solve_geneva(m: Mechanism, solids: dict, values: list[float]):
    wheel = solids["wheel"]
    pin = m.parts["driver"].features["pin"]
    zc = (band(0)[0] + band(0)[1]) / 2
    ol = Outline(wheel, zc)
    rp = pin.size / 2
    sense = m.meta["geneva"]["sense"]
    wf = m.parts["wheel"].features["bore"]
    out = []
    phi = 0.0
    for v in values:
        poses = {p: m.parts[p].shown for p in m.parts if m.parts[p].kind != "link"}
        sol = kin.solve(_driver_only(m), v, {})
        poses.update(sol.poses)
        O2 = kin.world_pt(m, poses, "ground", "p2")
        P = kin.world_pt(m, poses, "driver", "pin")

        def wheel_pose(ph):
            r = kin.rot(ph, wf.xy)
            return (O2[0] - r[0], O2[1] - r[1], ph)

        def clear(ph):
            pw = kin.apply(kin.inverse(wheel_pose(ph)), P)
            if ol.inside(pw)[0]:
                return False
            return ol.closest(pw)[0] >= rp - 1e-9

        if not clear(phi):
            lo, hi = phi, None
            for k in range(1, 200):
                trial = phi + sense * math.radians(0.5) * k
                if clear(trial):
                    hi = trial
                    break
            if hi is None:
                raise kin.Jam(v, "contact", "the pin cannot be freed by turning the wheel: it jams", 0.0)
            for _ in range(40):
                mid = (lo + hi) / 2
                if clear(mid):
                    hi = mid
                else:
                    lo = mid
            phi = hi
        poses["wheel"] = wheel_pose(phi)
        out.append(kin.Solution(poses, {}))
    return out


# ---------------------------------------------------------------- ratchet


def draw_ratchet(rng, knobs):
    N = rng.choice(range(10, 17))
    Rr, d = U(rng, 26, 34), U(rng, 3.5, 5.0)
    pitch = 2 * math.pi / N
    b0, b1 = band(0), band(1)
    gam = U(rng, 0, 2 * math.pi)
    pts = []
    for k in range(N):
        f0 = -k * pitch
        pts.append([(Rr - d) * math.cos(f0), (Rr - d) * math.sin(f0)])
        f1 = f0 - pitch
        pts.append([Rr * math.cos(f1 + 1e-3), Rr * math.sin(f1 + 1e-3)])
    r_dowel = 10.0
    wh_feats = {"bore": Feature("hole", (0.0, 0.0), HOLE, b0), "drive": Feature("hole", (r_dowel, 0.0), DRIVE_HOLE, b0)}
    wh_rec = {"z": list(b0), "plan": [{"poly": pts}], "holes": [[0.0, 0.0, HOLE], [r_dowel, 0.0, DRIVE_HOLE]]}
    drv_feats, drv_rec = _driver(r_dowel, 1, b0[0])
    # pawl: pivot A, tip T resting in a notch at angle gam (between a face and the next back)
    Lp = U(rng, 28, 38)
    tipr = Rr - d * 0.45
    T = (tipr * math.cos(gam), tipr * math.sin(gam))
    side = -1  # with these teeth: free when the wheel turns forward, locked backward
    tang = (-math.sin(gam) * side, math.cos(gam) * side)
    A = (T[0] + Lp * tang[0] + 6 * math.cos(gam), T[1] + Lp * tang[1] + 6 * math.sin(gam))
    ang = math.atan2(T[1] - A[1], T[0] - A[0])
    L = math.dist(A, T)
    pawl_feats = {"bore": Feature("hole", (0.0, 0.0), HOLE, b0)}
    nrm = (-math.sin(ang), math.cos(ang))
    pawl_rec = {"z": list(b0), "plan": [{"disc": [[0.0, 0.0], BOSS / 2]},
                                        {"poly": [[0.0, -3.0], [L - 6, -3.0], [L, 0.0], [L - 6, 3.0], [0.0, 3.0]]}],
                "holes": [[0.0, 0.0, HOLE]]}
    gnd_feats = {"p1": Feature("pin", (0.0, 0.0), PIN, (BASE_TOP, pin_top(1))), "pA": Feature("pin", A, PIN, (BASE_TOP, pin_top(0)))}
    gnd_rec = _base([(0, 0, Rr + 4), (A[0], A[1], 12)], [(0.0, 0.0, pin_top(1)), (A[0], A[1], pin_top(0))])
    parts = {
        "ground": Part("ground", "ground", gnd_feats, label="the frame", recipe=gnd_rec),
        "driver": Part("driver", "link", drv_feats, label="the driving disc", recipe=drv_rec),
        "wheel": Part("wheel", "link", wh_feats, label="the ratchet wheel", recipe=wh_rec),
        "pawl": Part("pawl", "link", pawl_feats, (A[0], A[1], ang), label="the pawl", recipe=pawl_rec),
    }
    joints = {j.id: j for j in [
        Joint("j_d0", "revolute", ("ground", "p1"), ("driver", "bore")),
        Joint("j_d1", "revolute", ("driver", "dowel"), ("wheel", "drive")),
        Joint("j_w0", "revolute", ("ground", "p1"), ("wheel", "bore")),
        Joint("j_p0", "revolute", ("ground", "pA"), ("pawl", "bore")),
        Joint("j_pw", "contact", ("pawl", "bore"), ("wheel", "bore")),
    ]}
    th0 = 0.0
    m = _mech("ratchet", rng, parts, joints, (th0, th0 + 360.0), {"kind": "none"}, knobs)
    m.meta["ratchet"] = {"N": N, "Rr": Rr, "d": d, "pitch_deg": 360.0 / N, "A": list(A), "nrm": list(nrm)}
    m.meta["removed"] = "wheel"
    return m


def _pawl_pose(m, A, psi):
    p = m.parts["pawl"]
    r = kin.rot(psi, p.features["bore"].xy)
    return (A[0] - r[0], A[1] - r[1], psi)


def solve_ratchet(m: Mechanism, solids: dict, values: list[float], psi0: float | None = None):
    """Turn the wheel through ``values``. Returns (solutions, events): each push
    of the wheel against the pawl is an event, 'lift' or 'lock'."""
    zc = sum(band(0)) / 2
    wol = Outline(solids["wheel"], zc)
    pol = Outline(solids["pawl"], zc)
    A = m.parts["ground"].features["pA"].xy
    psi = m.parts["pawl"].shown[2] if psi0 is None else psi0
    bb = solids["pawl"].bounding_box()
    Rmax = max(math.hypot(x, y) for x in (bb.min.X, bb.max.X) for y in (bb.min.Y, bb.max.Y))
    # lift sense: the turn that moves the tip away from the wheel centre
    tipl = (bb.max.X, 0.0)
    out, events = [], []

    def gap(poses, ps):
        d, on_pawl, on_wheel = outline_gap(pol, _pawl_pose(m, A, ps), wol, poses["wheel"])
        return d, (on_pawl, on_wheel)

    O = m.parts["ground"].features["p1"].xy
    wf = m.parts["wheel"].features

    def scene(v):
        poses = {p: m.parts[p].shown for p in m.parts if m.parts[p].kind != "link"}
        poses.update(kin.solve(_driver_only(m), v, {}).poses)
        D = kin.world_pt(m, poses, "driver", "dowel")
        poses["wheel"] = two_pin_pose(wf["bore"].xy, wf["drive"].xy, O, D, v, "the ratchet wheel")
        return poses

    def lift_free(poses, psi, lift):
        lo, hi = psi, None
        for k in range(1, 400):
            tr = psi + lift * math.radians(0.25) * k
            if gap(poses, tr)[0] > 1e-9:
                hi = tr
                break
        if hi is None:
            return None
        for _ in range(30):  # to 1e-9 of the bracket: well under a micrometre at the tip
            mid = (lo + hi) / 2
            if gap(poses, mid)[0] > 1e-9:
                hi = mid
            else:
                lo = mid
        return hi

    for i, v in enumerate(values):
        poses = scene(v)
        T = kin.apply(_pawl_pose(m, A, psi), tipl)
        perp = (-(T[1] - A[1]), T[0] - A[0])
        lift = 1.0 if perp[0] * (T[0] - O[0]) + perp[1] * (T[1] - O[1]) > 0 else -1.0
        d, ext = gap(poses, psi)
        if d <= 1e-9:
            if i == 0:  # first placement on the wheel, not a push
                psi = lift_free(poses, psi, lift)
                if psi is None:
                    raise kin.Jam(v, "contact", "the pawl cannot be placed on the wheel", 0.0)
                poses["pawl"] = _pawl_pose(m, A, psi)
                out.append(kin.Solution(poses, {}))
                continue
            # pushed. Find the wheel angle of first touch (pawl still where it was)
            # and read the contact normal there, before anything slides
            lo, hi = values[i - 1], v
            for _ in range(24):  # to a millionth of the step
                mid = (lo + hi) / 2
                if gap(scene(mid), psi)[0] > 1e-9:
                    lo = mid
                else:
                    hi = mid
            d0, (p, q) = gap(scene(lo), psi)  # p on the pawl, q on the wheel, a hair apart
            if p is None:
                n = (0.0, 0.0)
            else:
                n = (p[0] - q[0], p[1] - q[1])
                nn = math.hypot(*n)
                n = (n[0] / nn, n[1] / nn) if nn > 1e-15 else (0.0, 0.0)
            # moment about the pawl pivot of the wheel pushing the pawl along n
            M = (q[0] - A[0]) * n[1] - (q[1] - A[1]) * n[0] if p is not None else 0.0
            kind = "lift" if M * lift > 0 else "lock"
            events.append((kind, lo, M * lift))
            if kind == "lock":
                raise kin.Jam(lo, "contact", "the wheel drives the pawl into its teeth: locked", 0.0)
            psi = lift_free(poses, psi, lift)
            if psi is None:
                raise kin.Jam(v, "contact", "the pawl cannot be lifted clear: locked", 0.0)
        else:
            # the spring lowers the pawl until it touches (never through the wheel)
            for _ in range(200):
                d, ext = gap(poses, psi)
                if d < 1e-5:
                    break
                psi -= lift * d / Rmax
            if gap(poses, psi)[0] <= 1e-9:
                psi += lift * 1e-7
        poses["pawl"] = _pawl_pose(m, A, psi)
        out.append(kin.Solution(poses, {}))
    return out, events


# ---------------------------------------------------------------- generation and grading

DRAW = {"cam": draw_cam, "geneva": draw_geneva, "ratchet": draw_ratchet}
TOL = {"point_mm": 0.2, "angle_deg": 0.2, "pressure_max_deg": 35.0, "overlap_volume_max": 0.01,
       "lock_within_extra_deg": 2.0}


def build_part(pid, rec):
    if rec is None:
        return None
    if pid == "driver" and "lock_cut" in rec:
        return build_driver(rec)
    rec = dict(rec)
    if rec.get("plan") and "spline" in rec["plan"][0]:
        pts = [tuple(p) for p in rec["plan"][0]["spline"]]
        wire = bd.Wire([bd.Spline(*[(x, y) for x, y in pts] + [pts[0]], periodic=False)])
        face = bd.Face(wire)
        z0, z1 = rec["z"]
        s = bd.Pos(0, 0, z0) * bd.extrude(face, z1 - z0, dir=(0, 0, 1))
        for h in rec.get("holes", []):
            s = s - bd.Pos(h[0], h[1], z0 - 1) * bd.Cylinder(h[2] / 2, z1 - z0 + 2, align=(bd.Align.CENTER,) * 2 + (bd.Align.MIN,))
        return s.solids()[0]
    return build(rec)


def solve_any(m, solids, values):
    if m.type == "cam":
        sols, press = solve_cam(m, solids, values)
        return sols, {"pressure": press}
    if m.type == "geneva":
        return solve_geneva(m, solids, values), {}
    sols, ev = solve_ratchet(m, solids, values)
    return sols, {"events": ev}


def output_of(m, sol, start=None):
    o = m.output
    if o["kind"] == "point":
        return kin.output_value(m, sol.poses)
    if o["kind"] == "angle":
        a = math.degrees(sol.poses[o["part"]][2])
        return (a - (start or 0.0),)
    return ()


def make(mtype: str, seed: int, knobs: dict | None = None, max_tries: int = 40):
    knobs = dict(knobs or {})
    rng = random.Random(f"motion-{mtype}-{seed}-" + ",".join(f"{k}={knobs[k]}" for k in sorted(knobs)))
    why = []
    for attempt in range(max_tries):
        m = DRAW[mtype](rng, knobs)
        if mtype == "cam" and m.meta["cam"]["max_pressure"] > 30:
            why.append("pressure angle")
            continue
        solids = {pid: build_part(pid, p.recipe) for pid, p in m.parts.items()}
        rem = m.meta["removed"]
        try:
            vals = kin.input_values(m, 1.0)
            sols, extra = solve_any(m, solids, vals)
        except kin.Jam as j:
            why.append(str(j))
            continue
        for pid, pose in sols[0].poses.items():
            m.parts[pid].shown = pose
        task = _task(m, mtype, seed, knobs, rem, solids)
        ref_world = solids[rem].moved(kin.pose_to_placement(m.frame, m.parts[rem].shown).location())
        rep = grade_solid(task, m, solids, ref_world)
        if not rep.ok:
            why.append("reference fails: " + "; ".join(c.reason for c in rep.checks if c.ok is False))
            continue
        task["generation"] = {"attempt": attempt, "rejected": [w[:160] for w in why], "reference_seconds": rep.info.get("seconds")}
        if mtype == "cam":
            task["generation"]["analytic_vs_contact_mm"] = _cam_analytic_check(m, task)
        m.meta["solids"] = solids
        return task, m, ref_world
    raise RuntimeError(f"{mtype} seed {seed}: no valid draw; last: {why[-3:]}")


def _cam_analytic_check(m, task) -> float:
    """Largest difference between the follower table (found by contact on
    the reference solid) and the lift law the cam was drawn from."""
    c = m.meta["cam"]
    O = m.parts["ground"].features["post"].xy
    u = kin.unit(c["alpha"])
    worst = 0.0
    for v, p in zip(task["table"]["inputs"], task["table"]["plane"]):
        D = math.radians(v - m.range[0])
        s = c["Rb"] + c["rho"] + _cam_lift(D, c["h"], *c["b"])
        worst = max(worst, math.dist(p, (O[0] + s * u[0], O[1] + s * u[1])))
    return worst


def _task(m, mtype, seed, knobs, rem, solids) -> dict:
    vals = kin.input_values(m, TABLE_STEP)
    table = {"inputs": vals, "kind": m.output["kind"], "plane": [], "world": []}
    if m.output["kind"] != "none":
        # contact motion depends on the path taken: solve at the grading step, sample the table
        fine = kin.input_values(m, 1.0)
        sols_f, _ = solve_any(m, solids, fine)
        by = {round(v, 9): s for v, s in zip(fine, sols_f)}
        sols = [by[round(v, 9)] for v in vals]
        start = math.degrees(sols[0].poses[m.output["part"]][2]) if m.output["kind"] == "angle" else None
        for s in sols:
            v = output_of(m, s, start)
            table["plane"].append(list(v))
            if m.output["kind"] == "point":
                f = m.parts[m.output["part"]].features[m.output["feature"]]
                table["world"].append(list(m.frame.point((v[0], v[1], sum(f.z) / 2))))
            else:
                table["world"].append(list(v))
    full = m.to_json()
    hidden = m.to_json(hide=rem)
    joints = {}
    for j in m.joints.values():
        if rem in (j.a[0], j.b[0]) and j.kind != "contact":
            mp, mf = j.other(rem)
            f = m.parts[mp].features[mf]
            joints[j.id] = {"joint": j.id, "with": mp, "feature": f.kind, "size": f.size, "z_band": list(f.z)}
    return {"format": 1, "kind": "motion", "type": mtype, "id": f"motion-{mtype}-s{seed}", "seed": seed,
            "generator_version": GEN_VERSION, "knobs": knobs, "removed": rem, "mechanism": hidden,
            "table": table, "fits": DEFAULT_FITS, "tolerances": TOL, "step_deg": 1.0,
            "joints_of_removed": joints, "_full": full}


def _measure_part(m, rem, solid, fits):
    """Bore (and drive hole) of the removed part, from its own geometry."""
    from .grade import candidates, needs
    found = all_features(solid)
    need = [n for n in needs(m, rem, fits) if n["joint"] is None or m.joints[n["joint"]].kind != "contact"]
    feats, missing = {}, []
    for n in need:
        c = candidates(found, n, fits)
        if not c:
            missing.append(n)
            continue
        # nearest to where the neighbour's feature sits (several same-size holes are possible)
        feats[n["name"]] = c[0].feature() if len(c) == 1 else min(
            c, key=lambda f: math.dist(f.xy, (0.0, 0.0)) if n["name"] == "bore" else -math.dist(f.xy, (0.0, 0.0))).feature()
    return feats, missing, need


def grade_solid(task: dict, m_full_or_hidden: Mechanism, fixed_solids: dict, solid_world) -> Report:
    t0 = time.perf_counter()
    m = m_full_or_hidden.copy()
    rem = task["removed"]
    fits = task["fits"]
    tol = {**TOL, **task.get("tolerances", {})}
    checks = []
    solid = solid_world.moved(kin.frame_inverse(m.frame).location())
    feats, missing, need = _measure_part(m, rem, solid, fits)
    if missing:
        from .grade import _describe_need
        checks.append(CheckResult("JOINTS", False, "no " + _describe_need(m, missing[0]), len(missing), "missing"))
        return _skip_rest(checks, ["MOTION", "OUTPUT", "QUALITY", "NO_INTERFERENCE"], t0)
    checks.append(CheckResult("JOINTS", True, f"{len(feats)} found: " + ", ".join(
        f"{k} dia {v.size:.2f}" for k, v in feats.items()), len(feats), "joints"))
    old = m.parts[rem]
    m.parts[rem] = Part(rem, "link", feats, (0.0, 0.0, 0.0), None, old.label)
    solids = {k: v for k, v in fixed_solids.items() if k != rem}
    solids[rem] = solid
    values = kin.input_values(m, task.get("step_deg", 1.0))
    try:
        sols, extra = solve_any(m, solids, values)
    except kin.Jam as j:
        checks.append(CheckResult("MOTION", False, f"stops at {j.value % 360:.1f} deg: {j.reason}", j.value, "deg"))
        return _skip_rest(checks, ["OUTPUT", "QUALITY", "NO_INTERFERENCE"], t0)
    checks.append(CheckResult("MOTION", True, f"moves through all {len(values)} input values", len(values), "poses"))
    # OUTPUT
    if task["table"]["kind"] != "none":
        tin = task["table"]["inputs"]
        by = {round(v, 9): s for v, s in zip(values, sols)}
        start = math.degrees(sols[0].poses[m.output["part"]][2]) if m.output["kind"] == "angle" else None
        errs = []
        for v, tv in zip(tin, task["table"]["plane"]):
            s = by.get(round(v, 9))
            if s is None:
                s = solve_any(m, solids, [v])[0][0]
            o = output_of(m, s, start)
            if m.output["kind"] == "point":
                errs.append((math.dist(o, tv[:2]), v))
            else:
                errs.append((abs(math.degrees(kin.wrap(math.radians(o[0] - tv[0])))), v))
        e, at = max(errs)
        lim, unit = (tol["point_mm"], "mm") if m.output["kind"] == "point" else (tol["angle_deg"], "deg")
        checks.append(CheckResult("OUTPUT", e <= lim, (f"off by {e:.3f} {unit} at {at % 360:.1f} deg (tol {lim:g})" if e > lim
                                  else f"follows the table within {e:.4f} {unit} (tol {lim:g})"), round(e, 5), unit))
    # QUALITY per type
    if m.type == "cam":
        worst, at = extra["pressure"]
        lim = tol["pressure_max_deg"]
        checks.append(CheckResult("PRESSURE_ANGLE", worst <= lim, f"worst pressure angle {worst:.1f} deg at "
                                  f"{at % 360:.1f} deg (limit {lim:g}){': the follower would bind' if worst > lim else ''}",
                                  round(worst, 3), "deg"))
    elif m.type == "geneva":
        checks.append(_geneva_locked(m, solids, values, sols))
    elif m.type == "ratchet":
        checks.append(_ratchet_lock(m, solids, task, tol))
    # NO_INTERFERENCE
    scene = Scene(m.parts, solids, tol["overlap_volume_max"], joints=m.joints,
                  cuts=sorted({z for p in m.parts.values() for f in p.features.values() for z in f.z}))
    hits = []
    for v, s in zip(values, sols):
        for a, b, vol in scene.clashes(s.poses):
            hits.append((v, a, b, vol))
    if hits:
        v, a, b, vol = hits[0]
        checks.append(CheckResult("NO_INTERFERENCE", False, f"{scene.label[a]} fouls {scene.label[b]} at {v % 360:.1f} deg: "
                                  f"{vol:.3f} mm3", round(vol, 4), "mm3", {"clashes": [[h[0], h[1], h[2], h[3]] for h in hits[:50]]}))
    else:
        checks.append(CheckResult("NO_INTERFERENCE", True, f"no overlap at any of {len(values)} poses", 0.0, "mm3"))
    rep = Report(checks)
    rep.info = {"step_deg": task.get("step_deg", 1.0), "poses": len(values), "exact_tests": scene.n_exact,
                "max_move_mm": round(_max_move(scene, sols), 4), "seconds": round(time.perf_counter() - t0, 3)}
    return rep


def _geneva_locked(m, solids, values, sols) -> CheckResult:
    """During the dwell the wheel must be held: turned 1 deg either way it hits the locking disc."""
    from ..checks.base import volume
    from .interfere import location
    g = m.meta["geneva"]
    idx = [0, len(values) // 12, len(values) - 1 - len(values) // 12]
    for i in idx:
        s = sols[i]
        rel_free = []
        for sgn in (1, -1):
            pw = s.poses["wheel"]
            O2 = kin.world_pt(m, s.poses, "ground", "p2")
            turn = math.radians(sgn * 1.0)
            r = kin.rot(pw[2] + turn, m.parts["wheel"].features["bore"].xy)
            pose = (O2[0] - r[0], O2[1] - r[1], pw[2] + turn)
            rel = kin.compose(kin.inverse(s.poses["driver"]), pose)
            v = volume(solids["wheel"].moved(location(rel)) & solids["driver"])
            rel_free.append(v <= 0.01)
        if all(rel_free):
            return CheckResult("LOCKED", False, f"the wheel is free to turn 1 deg either way during the dwell at "
                               f"{values[i] % 360:.1f} deg: nothing holds it", values[i], "deg")
    return CheckResult("LOCKED", True, f"held during the dwell (turned 1 deg it meets the locking disc), "
                       f"checked at {len(idx)} input values", len(idx), "poses")


def _ratchet_lock(m, solids, task, tol) -> CheckResult:
    """Turned backwards from several positions, the wheel must lock within one tooth pitch."""
    pitch = m.meta["ratchet"]["pitch_deg"]
    lim = pitch + tol["lock_within_extra_deg"]
    worst = 0.0
    starts = [m.range[0] + k * pitch / 3 for k in range(3)]
    for st in starts:
        fwd = [m.range[0] + k for k in range(int(st - m.range[0]) + 1)] + ([st] if st > m.range[0] else [])
        try:
            sols, _ = solve_ratchet(m, solids, fwd)
        except kin.Jam:
            return CheckResult("LOCKS_BACKWARD", False, "cannot reach the start of the backward test", None)
        psi = sols[-1].poses["pawl"][2]
        back = [st - 0.5 * k for k in range(int(lim / 0.5) + 2)]
        try:
            solve_ratchet(m, solids, back, psi0=psi)
        except kin.Jam as j:
            worst = max(worst, st - j.value)
            continue
        return CheckResult("LOCKS_BACKWARD", False, f"turned backwards from {st % 360:.1f} deg it runs on past "
                           f"{lim:.1f} deg without locking", lim, "deg")
    return CheckResult("LOCKS_BACKWARD", worst <= lim, f"locks within {worst:.1f} deg when turned backwards "
                       f"(one tooth is {pitch:.1f} deg; limit {lim:.1f})", round(worst, 3), "deg")


def write(task, m, ref, out_dir):
    folder = os.path.join(out_dir, task["id"])
    os.makedirs(os.path.join(folder, "_solution"), exist_ok=True)
    solids = m.meta["solids"]
    for pid, p in m.parts.items():
        if pid == task["removed"]:
            continue
        write_step(solids[pid].moved(kin.pose_to_placement(m.frame, p.shown).location()),
                   os.path.join(folder, f"fixed_{pid}.step"))
    write_step(ref, os.path.join(folder, "_solution", "reference.step"))
    full = task.pop("_full")
    with open(os.path.join(folder, "_solution", "mechanism_full.json"), "w") as f:
        json.dump(full, f, indent=1)
    with open(os.path.join(folder, "motion.json"), "w") as f:
        json.dump(task, f, indent=1)
    task["_full"] = full
    return folder


def grade_task(folder, step_path=None) -> Report:
    from ..checks import file_valid
    from ..checks.base import Context
    with open(os.path.join(folder, "motion.json")) as f:
        task = json.load(f)
    m = Mechanism.from_json(task["mechanism"])
    solids = {pid: build_part(pid, p.recipe) for pid, p in m.parts.items() if p.recipe is not None}
    ctx = Context(task=None, step_path=step_path or os.path.join(folder, "output.step"))
    fv = file_valid.run(ctx)
    if not fv.ok:
        return Report([fv])
    rep = grade_solid(task, m, solids, ctx.solid)
    rep.checks.insert(0, fv)
    return rep
