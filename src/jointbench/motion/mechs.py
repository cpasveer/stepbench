"""The four planar mechanisms, grown answer-first as 3D parts.

Each ``draw_*`` function samples a mechanism the way linkbench does: pins
are placed at concrete positions first and lengths are read off, so the
mechanism closes at the first input value by construction. What is left to
prove -- that it turns through its whole range, never binds, never fouls
itself -- is proved afterwards by the same grader an agent faces
(``generate.make``).

Parts are flat plates in stacked height bands ("layers"), as real planar
linkages are built: layer 0 just above the frame, layer 1 above that.
Joined parts sit in neighbouring layers; a pin rises from the lower part
through the upper one. The frame is a base plate with posts (ground
pivots), rails (slide guides) or bearing plates.
"""
from __future__ import annotations

import math

from .kin import Feature, Joint, Mechanism, Part, apply

T, G = 6.0, 1.0  # plate thickness, gap between layers
PIN, HOLE = 6.0, 6.4  # pin and its running-fit hole
BOSS, W = 16.0, 10.0  # boss diameter round a pin, link width
MARK = 3.0  # the output-point mark: a small through hole
BASE_TOP, BASE_T = -1.0, 8.0


def band(k: int) -> tuple[float, float]:
    z0 = k * (T + G)
    return (z0, z0 + T)


def pin_top(k: int) -> float:
    return band(k)[1] + 0.5


def U(rng, a, b):
    return a + (b - a) * rng.random()


# ------------------------------------------------------------------ recipes


def link_recipe(feats: dict, order: list[str], z: tuple, width: float = W, via: dict | None = None,
                boss: float = BOSS) -> dict:
    """A plate through its features in ``order``: capsules between them (or via
    waypoints), a boss round each, bores for holes, pins added."""
    via = via or {}
    plan = []
    pts = [feats[n].xy for n in order]
    for i in range(len(order) - 1):
        a, b = order[i], order[i + 1]
        w = via.get(f"{a}-{b}")
        if w:
            plan.append({"capsule": [list(pts[i]), list(w), width]})
            plan.append({"capsule": [list(w), list(pts[i + 1]), width]})
        else:
            plan.append({"capsule": [list(pts[i]), list(pts[i + 1]), width]})
    for n in order:
        f = feats[n]
        plan.append({"disc": [list(f.xy), (boss if f.kind != "mark" else MARK + 7) / 2]})
    holes = [[*f.xy, f.size] for f in feats.values() if f.kind in ("hole", "mark")]
    pins = [[*f.xy, f.size, f.z[0], f.z[1]] for f in feats.values() if f.kind == "pin"]
    return {"z": list(z), "plan": plan, "holes": holes, "pins": pins}


def ground_posts(posts: dict) -> list:
    return [[x, y, PIN, BASE_TOP - 4, top] for (x, y, top) in posts.values()]


# ------------------------------------------------------------------ four-bar


def draw_fourbar(rng, knobs: dict) -> Mechanism:
    r = U(rng, 18, 28)
    while True:
        g, la, lb = U(rng, 55, 95), U(rng, 45, 100), U(rng, 40, 90)
        L = [r, g, la, lb]
        if min(L) != r:
            continue
        if r + max(L) > sum(L) - r - max(L) - 8:  # Grashof with margin: the crank turns fully
            continue
        break
    th0 = U(rng, 0, 360)
    mode = 1 if rng.random() < 0.5 else -1
    out_point = _out_choice(knobs, "coupler", "rocker", rng)
    b0, b1 = band(0), band(1)
    crank = {"bore": Feature("hole", (0.0, 0.0), HOLE, b0), "pin": Feature("pin", (r, 0.0), PIN, (b0[0], pin_top(1)))}
    coup = {"h1": Feature("hole", (0.0, 0.0), HOLE, b1), "h2": Feature("hole", (la, 0.0), HOLE, b1)}
    order = ["h1", "h2"]
    if out_point:
        cu, cv = U(rng, 0.25, 0.75), U(rng, 14, 30) * (1 if rng.random() < 0.5 else -1)
        coup["mark"] = Feature("mark", (cu * la, cv), MARK, b1)
        order = ["h1", "mark", "h2"]
    rock = {"bore": Feature("hole", (0.0, 0.0), HOLE, b0), "pin": Feature("pin", (lb, 0.0), PIN, (b0[0], pin_top(1)))}
    gnd = {"p1": Feature("pin", (0.0, 0.0), PIN, (BASE_TOP, pin_top(0))),
           "p2": Feature("pin", (g, 0.0), PIN, (BASE_TOP, pin_top(0)))}
    parts = {
        "ground": Part("ground", "ground", gnd, label="the frame",
                       recipe={"z": [BASE_TOP - BASE_T, BASE_TOP], "pins": ground_posts(
                           {"p1": (0.0, 0.0, pin_top(0)), "p2": (g, 0.0, pin_top(0))})}),
        "crank": Part("crank", "link", crank, label="the crank",
                      recipe=link_recipe(crank, ["bore", "pin"], b0, width=BOSS)),
        "coupler": Part("coupler", "link", coup, label="the coupler", recipe=link_recipe(coup, order, b1)),
        "rocker": Part("rocker", "link", rock, label="the rocker", recipe=link_recipe(rock, ["bore", "pin"], b0)),
    }
    joints = {j.id: j for j in [
        Joint("j_c0", "revolute", ("ground", "p1"), ("crank", "bore")),
        Joint("j_c1", "revolute", ("crank", "pin"), ("coupler", "h1")),
        Joint("j_r1", "revolute", ("rocker", "pin"), ("coupler", "h2")),
        Joint("j_r0", "revolute", ("ground", "p2"), ("rocker", "bore")),
    ]}
    steps = [{"type": "crank", "part": "crank", "pivot": "j_c0"},
             {"type": "rrr", "a": "coupler", "b": "rocker", "ja": "j_c1", "jm": "j_r1", "jb": "j_r0", "mode": mode}]
    output = ({"kind": "point", "part": "coupler", "feature": "mark"} if out_point
              else {"kind": "angle", "part": "rocker", "from": "j_r0", "to": "j_r1"})
    m = Mechanism("fourbar", None, parts, joints, steps,
                  {"kind": "angle", "part": "crank", "from": "j_c0", "to": "j_c1"}, output, (th0, th0 + 360.0))
    m.meta["chain"] = {"coupler": order, "rocker": ["bore", "pin"], "crank": ["bore", "pin"]}
    m.meta["width"] = {"coupler": W, "rocker": W, "crank": BOSS}
    return m


def _out_choice(knobs: dict, carrier: str, other: str, rng) -> bool:
    """Whether the output is a point on ``carrier`` (else on ``other``),
    from the knobs ``remove`` and ``output_on_removed``."""
    rem = knobs.get("remove")
    on = knobs.get("output_on_removed")
    if rem == carrier:
        return bool(on)
    if rem == other:
        return not on
    return rng.random() < 0.5


# ------------------------------------------------------------------ crank-slider


def draw_slider(rng, knobs: dict) -> Mechanism:
    r = U(rng, 18, 28)
    L = U(rng, 3.2 * r, 4.5 * r)
    e = U(rng, -0.3 * r, 0.3 * r)
    th0 = U(rng, 0, 360)
    BW = 14.0
    # version 3: the block bears on at least 1.5 x its width (GUIDED) with 4 mm to spare
    BL = 18.0 if knobs.get("_version", 3) < 3 else 1.5 * BW + 4.0
    ch = BW + 0.4
    rem = knobs.get("remove")
    rod_mark = rem == "rod" and bool(knobs.get("output_on_removed"))
    b0, b1 = band(0), band(1)
    xmin, xmax = math.sqrt((L - r) ** 2 - e * e), math.sqrt((L + r) ** 2 - e * e)
    xa, xb = xmin - BL / 2 - 4, xmax + BL / 2 + 4
    crank = {"bore": Feature("hole", (0.0, 0.0), HOLE, b0), "pin": Feature("pin", (r, 0.0), PIN, (b0[0], pin_top(1)))}
    rod = {"h1": Feature("hole", (0.0, 0.0), HOLE, b1), "h2": Feature("hole", (L, 0.0), HOLE, b1)}
    order = ["h1", "h2"]
    if rod_mark:
        rod["mark"] = Feature("mark", (U(rng, 0.3, 0.7) * L, U(rng, 12, 22) * (1 if rng.random() < 0.5 else -1)), MARK, b1)
        order = ["h1", "mark", "h2"]
    slider = {"tongue": Feature("tongue", (0.0, 0.0), BW, b0, 0.0, (-BL / 2, BL / 2)),
              "pin": Feature("pin", (0.0, 0.0), PIN, (b0[0], pin_top(1)))}
    rail_y = ch / 2 + 3.0
    rails = [{"box3": [(xa + xb) / 2, e + s * rail_y, (BASE_TOP - 2 + b0[1]) / 2, xb - xa, 6.0, b0[1] - BASE_TOP + 2, 0.0]}
             for s in (1, -1)]
    gnd = {"p1": Feature("pin", (0.0, 0.0), PIN, (BASE_TOP, pin_top(0))),
           "guide": Feature("slot", ((xa + xb) / 2, e), ch, (BASE_TOP, b0[1]), 0.0, (-(xb - xa) / 2, (xb - xa) / 2))}
    parts = {
        "ground": Part("ground", "ground", gnd, label="the frame",
                       recipe={"z": [BASE_TOP - BASE_T, BASE_TOP], "add": rails,
                               "pins": ground_posts({"p1": (0.0, 0.0, pin_top(0))})}),
        "crank": Part("crank", "link", crank, label="the crank", recipe=link_recipe(crank, ["bore", "pin"], b0, width=BOSS)),
        "rod": Part("rod", "link", rod, label="the connecting rod", recipe=link_recipe(rod, order, b1)),
        "slider": Part("slider", "link", slider, label="the slider",
                       recipe={"z": list(b0), "plan": [{"box": [0.0, 0.0, BL, BW, 0.0]}],
                               "pins": [[0.0, 0.0, PIN, b0[0], pin_top(1)]]}),
    }
    joints = {j.id: j for j in [
        Joint("j_c0", "revolute", ("ground", "p1"), ("crank", "bore")),
        Joint("j_c1", "revolute", ("crank", "pin"), ("rod", "h1")),
        Joint("j_s1", "revolute", ("slider", "pin"), ("rod", "h2")),
        Joint("j_s0", "slide", ("slider", "tongue"), ("ground", "guide")),
    ]}
    steps = [{"type": "crank", "part": "crank", "pivot": "j_c0"},
             {"type": "rrp", "a": "rod", "b": "slider", "ja": "j_c1", "jm": "j_s1", "jb": "j_s0", "mode": 1}]
    output = ({"kind": "point", "part": "rod", "feature": "mark"} if rod_mark
              else {"kind": "point", "part": "slider", "feature": "pin"})
    m = Mechanism("slider", None, parts, joints, steps,
                  {"kind": "angle", "part": "crank", "from": "j_c0", "to": "j_c1"}, output, (th0, th0 + 360.0))
    m.meta["chain"] = {"rod": order, "crank": ["bore", "pin"]}
    m.meta["width"] = {"rod": W, "crank": BOSS}
    return m


# ------------------------------------------------------------------ Scotch yoke


def draw_yoke(rng, knobs: dict) -> Mechanism:
    r = U(rng, 15, 28)
    tilt = 0.0 if rng.random() < 0.5 else U(rng, 10, 25) * (1 if rng.random() < 0.5 else -1)
    sd = math.radians(90 + tilt)
    th0 = U(rng, 0, 360)
    b0, b1 = band(0), band(1)
    crank = {"bore": Feature("hole", (0.0, 0.0), HOLE, b0), "pin": Feature("pin", (r, 0.0), PIN, (b0[0], pin_top(1)))}
    yoke = {"slot": Feature("slot", (0.0, 0.0), HOLE, b1, sd),
            "tongue": Feature("tongue", (0.0, 0.0), W, b1, 0.0)}
    gnd = {"p1": Feature("pin", (0.0, 0.0), PIN, (BASE_TOP, pin_top(0))),
           "guide": Feature("slot", (0.0, 0.0), W + 0.4, (b0[1] + 0.5, b1[1]), 0.0)}
    parts = {
        "ground": Part("ground", "ground", gnd, label="the frame",
                       recipe={"z": [BASE_TOP - BASE_T, BASE_TOP], "pins": ground_posts({"p1": (0.0, 0.0, pin_top(0))})}),
        "crank": Part("crank", "link", crank, label="the crank", recipe=link_recipe(crank, ["bore", "pin"], b0, width=BOSS)),
        "yoke": Part("yoke", "link", yoke, label="the yoke"),
    }
    joints = {j.id: j for j in [
        Joint("j_c0", "revolute", ("ground", "p1"), ("crank", "bore")),
        Joint("j_ps", "pin_slot", ("crank", "pin"), ("yoke", "slot")),
        Joint("j_ys", "slide", ("yoke", "tongue"), ("ground", "guide")),
    ]}
    steps = [{"type": "crank", "part": "crank", "pivot": "j_c0"},
             {"type": "psl", "b": "yoke", "jp": "j_ps", "js": "j_ys"}]
    m = Mechanism("yoke", None, parts, joints, steps,
                  {"kind": "angle", "part": "crank", "from": "j_c0", "to": "j_c1"}, {}, (th0, th0 + 360.0))
    m.input = {"kind": "angle", "part": "crank", "from": "j_c0", "to": "j_ps"}
    m.output = {"kind": "point", "part": "yoke", "feature": "mark"}
    m.meta["complete"] = "yoke"
    m.meta["yoke"] = {"r": r, "sd": sd, "mark_side": 1 if rng.random() < 0.5 else -1}
    return m


def complete_yoke(m: Mechanism, sols) -> None:
    """The yoke's slot length, guides and bar follow from the swept motion."""
    from .kin import dot, sub, unit
    y = m.meta["yoke"]
    r, sd = y["r"], y["sd"]
    e = unit(sd)
    ts, ss = [], []
    for s in sols:
        P = apply(s.poses["crank"], m.parts["crank"].features["pin"].xy)
        Y = s.poses["yoke"]
        ts.append(dot(sub(P, (Y[0], Y[1])), e))
        ss.append(Y[0])
    tmin, tmax = min(ts), max(ts)
    smax = max(abs(v) for v in ss)
    Ls = (tmax - tmin) + HOLE + 3.0
    c = (e[0] * (tmin + tmax) / 2, e[1] * (tmin + tmax) / 2)
    Lf, Wf = Ls + 12.0, HOLE + 12.0
    fx = abs(c[0]) + abs(Lf / 2 * e[0]) + abs(Wf / 2 * e[1])
    D = max(r + BOSS / 2 + 3.0, fx + smax + 3.0)
    Lg = 16.0
    XB = D + Lg + smax + 3.0
    b0, b1 = band(0), band(1)
    xo = y["mark_side"] * (fx + 5.0)
    m.parts["yoke"].features["mark"] = Feature("mark", (xo, 0.0), MARK, b1)
    m.parts["yoke"].features["slot"] = Feature("slot", (0.0, 0.0), HOLE, b1, sd, (tmin - HOLE / 2 - 1.5, tmax + HOLE / 2 + 1.5))
    m.parts["yoke"].features["tongue"] = Feature("tongue", (0.0, 0.0), W, b1, 0.0, (-XB, XB))
    y.update({"c": list(c), "Ls": Ls, "Lf": Lf, "Wf": Wf, "XB": XB, "xo": xo, "tmin": tmin, "tmax": tmax, "D": D})
    m.parts["yoke"].recipe = yoke_recipe(sd, c, Ls, Lf, Wf, XB, xo)
    ch = W + 0.4
    add = []
    for sx in (1, -1):
        cx = sx * (D + Lg / 2)
        add.append({"box3": [cx, 0.0, (BASE_TOP - 2 + b0[1] + 0.5) / 2, Lg, ch + 12, b0[1] + 0.5 - BASE_TOP + 2, 0.0]})
        for sy in (1, -1):
            add.append({"box3": [cx, sy * (ch / 2 + 3), (BASE_TOP - 2 + b1[1]) / 2, Lg, 6.0, b1[1] - BASE_TOP + 2, 0.0]})
    m.parts["ground"].recipe["add"] = add
    m.parts["ground"].features["guide"] = Feature("slot", (0.0, 0.0), ch, (b0[1] + 0.5, b1[1]), 0.0, (-(D + Lg), D + Lg))
    m.meta["chain"] = {"crank": ["bore", "pin"]}
    m.meta["width"] = {"crank": BOSS}


def yoke_recipe(sd, c, Ls, Lf, Wf, XB, xo, bar_w: float = W) -> dict:
    b1 = band(1)
    zc = (b1[0] + b1[1]) / 2
    return {
        "z": list(b1),
        "plan": [{"box": [c[0], c[1], Lf, Wf, math.degrees(sd)]}, {"box": [0.0, 0.0, 2 * XB, bar_w, 0.0]}],
        "cut": [{"box3": [c[0], c[1], zc, Ls, HOLE, T + 2, math.degrees(sd)]}],
        "holes": [[xo, 0.0, MARK]],
    }


# ------------------------------------------------------------------ Oldham coupling


def draw_oldham(rng, knobs: dict) -> Mechanism:
    Rh = U(rng, 24, 32)
    e = U(rng, 3, 8)
    beta = U(rng, 0, 2 * math.pi)
    O2 = (e * math.cos(beta), e * math.sin(beta))
    wg = U(rng, 8, 12)
    wt = wg - 0.4
    Rd = Rh - U(rng, 0, 3)
    th0 = U(rng, 0, 360)
    SH, SHB = 12.0, 12.4
    hub_in = {"shaft": Feature("pin", (0.0, 0.0), SH, (-22.0, 0.0)),
              "groove": Feature("slot", (0.0, 0.0), wg, (4.0, 8.0), 0.0, (-Rh, Rh))}
    disc = {"t1": Feature("tongue", (0.0, 0.0), wt, (4.5, 8.5), 0.0, (-(Rd - 2), Rd - 2)),
            "t2": Feature("tongue", (0.0, 0.0), wt, (14.5, 18.5), math.pi / 2, (-(Rd - 2), Rd - 2))}
    hub_out = {"shaft": Feature("pin", (0.0, 0.0), SH, (23.0, 45.0)),
               "groove": Feature("slot", (0.0, 0.0), wg, (15.0, 19.0), math.pi / 2, (-Rh, Rh))}
    S = Rh + e + 16
    gnd = {"b1": Feature("hole", (0.0, 0.0), SHB, (-12.0, -2.0)), "b2": Feature("hole", O2, SHB, (25.0, 35.0))}
    posts = [{"cyl": [sx * (S - 7), sy * (S - 7), 10.0, -12.0, 35.0]} for sx in (1, -1) for sy in (1, -1)]
    parts = {
        "ground": Part("ground", "ground", gnd, label="the frame", recipe={
            "z": [-12.0, -2.0], "plan": [{"box": [0.0, 0.0, 2 * S, 2 * S, 0.0]}],
            "add": [{"box3": [0.0, 0.0, 30.0, 2 * S, 2 * S, 10.0, 0.0]}] + posts,
            "holes": [[0.0, 0.0, SHB, -13.0, -1.0], [O2[0], O2[1], SHB, 24.0, 36.0]]}),
        "hub_in": Part("hub_in", "link", hub_in, (0.0, 0.0, math.radians(th0)), label="the input hub", recipe={
            "z": [0.0, 8.0], "plan": [{"disc": [[0.0, 0.0], Rh]}],
            "cut": [{"box3": [0.0, 0.0, 6.5, 2 * Rh + 4, wg, 5.0, 0.0]}],
            "pins": [[0.0, 0.0, SH, -22.0, 0.5]]}),
        "disc": Part("disc", "link", disc, (0.0, 0.0, math.radians(th0)), label="the middle disc", recipe={
            "z": [8.5, 14.5], "plan": [{"disc": [[0.0, 0.0], Rd]}],
            "add": [{"box3": [0.0, 0.0, 6.75, 2 * Rd - 4, wt, 4.5, 0.0]},
                    {"box3": [0.0, 0.0, 16.25, 2 * Rd - 4, wt, 4.5, 90.0]}]}),
        "hub_out": Part("hub_out", "link", hub_out, (O2[0], O2[1], math.radians(th0)), label="the output hub", recipe={
            "z": [15.0, 23.0], "plan": [{"disc": [[0.0, 0.0], Rh]}],
            "cut": [{"box3": [0.0, 0.0, 16.5, 2 * Rh + 4, wg, 5.0, 90.0]}],
            "pins": [[0.0, 0.0, SH, 22.5, 45.0]]}),
    }
    joints = {j.id: j for j in [
        Joint("j_a0", "revolute", ("hub_in", "shaft"), ("ground", "b1")),
        Joint("j_1", "slide", ("disc", "t1"), ("hub_in", "groove")),
        Joint("j_2", "slide", ("disc", "t2"), ("hub_out", "groove")),
        Joint("j_b0", "revolute", ("hub_out", "shaft"), ("ground", "b2")),
    ]}
    steps = [{"type": "crank", "part": "hub_in", "pivot": "j_a0"},
             {"type": "ppr", "b": "disc", "c": "hub_out", "j1": "j_1", "j2": "j_2", "jc": "j_b0"}]
    m = Mechanism("oldham", None, parts, joints, steps, {"kind": "angle", "part": "hub_in", "line": "j_1"},
                  {"kind": "angle", "part": "hub_out", "line": "j_2"}, (th0, th0 + 360.0))
    m.meta["no_housing"] = True
    return m


DRAW = {"fourbar": draw_fourbar, "slider": draw_slider, "yoke": draw_yoke, "oldham": draw_oldham}
REMOVABLE = {"fourbar": ["coupler", "rocker", "crank"], "slider": ["rod", "slider", "crank"],
             "yoke": ["yoke", "crank"], "oldham": ["disc"]}
DEFAULT_REMOVE = {"fourbar": "coupler", "slider": "rod", "yoke": "yoke", "oldham": "disc"}
