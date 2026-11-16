"""3D parts of a mechanism, built from a small recipe.

Coordinates are the part's own frame: x and y lie in the mechanism plane, z
along its normal. A recipe is plain numbers, so ``mechanism.json`` can store
it and the grader rebuilds every fixed part exactly, never relying on how
well a STEP file round-trips (the core does the same with ``task.json``).

Recipe keys (all optional except ``z``):

``z``        [z0, z1]: the plate's height band
``plan``     2D shapes unioned into the plate outline, extruded over ``z``:
             ``{"capsule": [[x, y], [x, y], width]}``, ``{"disc": [[x, y], r]}``,
             ``{"box": [cx, cy, lx, ly, angle_deg]}``, ``{"poly": [[x, y], ...]}``,
             ``{"spline": [[x, y], ...]}`` (closed smooth curve through the points)
``add``      3D boxes added: ``{"box3": [cx, cy, cz, lx, ly, lz, angle_deg]}``,
             or cylinders ``{"cyl": [x, y, d, z0, z1]}``
``cut``      3D boxes or cylinders removed (same forms)
``holes``    ``[x, y, d]`` bores through everything, or ``[x, y, d, z0, z1]``
``pins``     ``[x, y, d, z0, z1]`` solid pins added last
"""
from __future__ import annotations

import math
from functools import lru_cache

import build123d as bd


def _face(sh: dict):
    if "capsule" in sh:
        (x1, y1), (x2, y2), w = sh["capsule"]
        L = math.hypot(x2 - x1, y2 - y1)
        a = math.degrees(math.atan2(y2 - y1, x2 - x1))
        f = bd.Pos(x1, y1) * bd.Circle(w / 2) + bd.Pos(x2, y2) * bd.Circle(w / 2)
        if L > 1e-9:
            f = f + bd.Pos((x1 + x2) / 2, (y1 + y2) / 2) * bd.Rot(0, 0, a) * bd.Rectangle(L, w)
        return f
    if "disc" in sh:
        (x, y), r = sh["disc"]
        return bd.Pos(x, y) * bd.Circle(r)
    if "box" in sh:
        cx, cy, lx, ly, ang = sh["box"]
        return bd.Pos(cx, cy) * bd.Rot(0, 0, ang) * bd.Rectangle(lx, ly)
    if "spline" in sh:  # a closed smooth outline through the points
        pts = [tuple(p) for p in sh["spline"]]
        return bd.Face(bd.Wire([bd.Spline(*pts, pts[0])]))
    if "poly" in sh:
        return bd.Polygon(*[tuple(p) for p in sh["poly"]], align=None)
    raise ValueError(f"unknown plan shape {sh}")


def _solid3(sh: dict):
    if "box3" in sh:
        cx, cy, cz, lx, ly, lz, ang = sh["box3"]
        return bd.Pos(cx, cy, cz) * bd.Rot(0, 0, ang) * bd.Box(lx, ly, lz)
    if "cyl" in sh:
        x, y, d, z0, z1 = sh["cyl"]
        return bd.Pos(x, y, z0) * bd.Cylinder(d / 2, z1 - z0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
    raise ValueError(f"unknown solid {sh}")


def build(recipe: dict) -> bd.Solid:
    z0, z1 = recipe["z"]
    body = None
    if recipe.get("plan"):
        face = None
        for sh in recipe["plan"]:
            f = _face(sh)
            face = f if face is None else face + f
        body = bd.Pos(0, 0, z0) * bd.extrude(face, z1 - z0, dir=(0, 0, 1))
    for sh in recipe.get("add", []):
        s = _solid3(sh)
        body = s if body is None else body + s
    lo, hi = -1e4, 1e4
    for sh in recipe.get("cut", []):
        body = body - _solid3(sh)
    for h in recipe.get("holes", []):
        x, y, d = h[:3]
        za, zb = (h[3], h[4]) if len(h) == 5 else (lo, hi)
        body = body - _solid3({"cyl": [x, y, d, za, zb]})
    for p in recipe.get("pins", []):
        body = body + _solid3({"cyl": p})
    solids = body.solids()
    if len(solids) != 1:
        raise ValueError(f"recipe built {len(solids)} solids, expected 1")
    return solids[0]


@lru_cache(maxsize=256)
def _build_cached(key: str) -> bd.Solid:
    import json
    return build(json.loads(key))


def build_cached(recipe: dict) -> bd.Solid:
    import json
    return _build_cached(json.dumps(recipe, sort_keys=True))


def primitives(recipe: dict) -> list:
    """The pieces the recipe adds, as separate solids. Their union contains
    the built part (cuts and holes only remove material), so testing pieces
    for separation is a sound shortcut."""
    z0, z1 = recipe["z"]
    out = []
    for sh in recipe.get("plan", []):
        out.append(bd.Pos(0, 0, z0) * bd.extrude(_face(sh), z1 - z0, dir=(0, 0, 1)))
    for sh in recipe.get("add", []):
        out.append(_solid3(sh))
    for p in recipe.get("pins", []):
        out.append(_solid3({"cyl": p}))
    return out
