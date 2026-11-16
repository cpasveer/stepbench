"""Plan-view pictures of a mechanism, for briefs and docs -- never for grading.

Seen along the plane normal. Each part is drawn as its top faces (outer
outline filled, holes cut out), lowest layer first. The removed part is
shown in red at the first input value with thin ghosts at a few further
values, and the required output path or angle marks are drawn over it.
"""
from __future__ import annotations

import math

import numpy as np
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Plane
from PIL import Image, ImageDraw, ImageFont

from . import kin

COLOURS = {"ground": (205, 205, 200), "obstacle": (120, 120, 125)}
LINKS = [(70, 110, 170), (90, 150, 110), (190, 140, 60), (130, 100, 170), (60, 150, 160)]
REMOVED = (205, 60, 50)


def _wire_pts(w, n_per_edge: int = 72):
    pts = []
    for e in w.edges():
        n = 1 if e.geom_type.name == "LINE" else n_per_edge
        ps = [e.position_at(i / n) for i in range(n + 1)]
        pts.append([(p.X, p.Y) for p in ps])
    # chain the edges end to start
    if not pts:
        return []
    ring = list(pts.pop(0))
    while pts:
        end = ring[-1]
        best, flip, bi = None, False, 0
        for i, seg in enumerate(pts):
            d0, d1 = math.dist(end, seg[0]), math.dist(end, seg[-1])
            if best is None or min(d0, d1) < best:
                best, flip, bi = min(d0, d1), d1 < d0, i
        seg = pts.pop(bi)
        ring += list(reversed(seg)) if flip else seg
    return ring


def top_faces(solid) -> list[tuple[float, list, list]]:
    out = []
    for f in solid.faces():
        if BRepAdaptor_Surface(f.wrapped).GetType() != GeomAbs_Plane:
            continue
        n = f.normal_at()
        if n.Z < 0.99:
            continue
        z = f.center().Z
        out.append((z, _wire_pts(f.outer_wire()), [_wire_pts(w) for w in f.inner_wires()]))
    out.sort(key=lambda t: t[0])
    return out


def _tf(pts, pose):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return [(c * x - s * y + pose[0], s * x + c * y + pose[1]) for x, y in pts]


def picture(mech: kin.Mechanism, solids: dict, path: str, removed: str | None = None, title: str = "",
            ghosts: int = 5, size: int = 900, table: list | None = None, sols=None) -> None:
    if sols is None:
        try:
            sols, jam = kin.sweep(mech, kin.input_values(mech, 2.0))
            if jam is not None or not sols:
                sols = None
        except (KeyError, TypeError, ValueError):
            sols = None  # a part without features (the brief's view): draw as shown
    pose0 = sols[0].poses if sols else {pid: p.shown for pid, p in mech.parts.items()}
    tops = {pid: top_faces(s) for pid, s in solids.items()}
    # extent
    xs, ys = [], []
    for pid, fl in tops.items():
        p = pose0.get(pid, mech.parts[pid].shown)
        for _, outer, _ in fl:
            for x, y in _tf(outer, p):
                xs.append(x)
                ys.append(y)
    x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
    pad = 30
    sc = (size - 2 * pad) / max(x1 - x0, y1 - y0)
    W = int((x1 - x0) * sc + 2 * pad)
    H = int((y1 - y0) * sc + 2 * pad) + 30
    img = Image.new("RGB", (W, H), (250, 250, 248))
    d = ImageDraw.Draw(img, "RGBA")

    def px(p):
        return (pad + (p[0] - x0) * sc, H - 30 - pad - (p[1] - y0) * sc)

    def draw_part(pid, pose, fill, outline=None, width=1, ghost=False):
        for _, outer, inners in tops[pid]:
            if len(outer) < 3:
                continue
            if ghost:
                d.line([px(q) for q in _tf(outer, pose)] + [px(_tf(outer[:1], pose)[0])], fill=fill, width=1)
                continue
            # a face with holes: paint through a mask so the holes show what lies below
            mask = Image.new("L", img.size, 0)
            md = ImageDraw.Draw(mask)
            md.polygon([px(q) for q in _tf(outer, pose)], fill=fill[3] if len(fill) == 4 else 255)
            for inner in inners:
                if len(inner) >= 3:
                    md.polygon([px(q) for q in _tf(inner, pose)], fill=0)
            img.paste(fill[:3], (0, 0), mask)
            d.line([px(q) for q in _tf(outer, pose)] + [px(_tf(outer[:1], pose)[0])], fill=outline or (40, 40, 40))
            for inner in inners:
                if len(inner) >= 3:
                    d.line([px(q) for q in _tf(inner, pose)] + [px(_tf(inner[:1], pose)[0])], fill=(40, 40, 40))

    order = sorted(solids, key=lambda k: (mech.parts[k].kind == "link", min((t[0] for t in tops[k]), default=0)))
    li = 0
    for pid in order:
        kind = mech.parts[pid].kind
        pose = pose0.get(pid, mech.parts[pid].shown)
        if pid == removed and sols:
            n = len(sols)
            for g in range(1, ghosts + 1):
                draw_part(pid, sols[int(g * n / (ghosts + 1))].poses[pid], (205, 60, 50, 110), ghost=True)
            draw_part(pid, pose, REMOVED + (170,))
            continue
        fill = COLOURS.get(kind) or LINKS[li % len(LINKS)]
        if kind == "link":
            li += 1
        draw_part(pid, pose, fill + ((150,) if kind == "link" else (255,)))
    # output trace
    o = mech.output
    if o.get("kind") == "point":
        if sols:
            path_pts = [px(kin.output_value(mech, s.poses)) for s in sols]
            d.line(path_pts, fill=(20, 20, 20, 200), width=2)
        elif table:
            d.line([px(v) for v in table], fill=(20, 20, 20, 200), width=2)
        if table:
            for v in table:
                q = px(v)
                d.ellipse([q[0] - 3, q[1] - 3, q[0] + 3, q[1] + 3], outline=(0, 0, 0), width=1)
    try:
        font = ImageFont.load_default(size=16)
    except TypeError:
        font = ImageFont.load_default()
    d.text((pad, H - 26), title, fill=(30, 30, 30), font=font)
    img.save(path)


def solids_for(mech: kin.Mechanism, extra: dict | None = None) -> dict:
    from .parts3d import build_cached
    out = {}
    for pid, p in mech.parts.items():
        if extra and pid in extra:
            out[pid] = extra[pid]
        elif p.solid is not None:
            out[pid] = p.solid
        elif p.recipe is not None:
            out[pid] = build_cached(p.recipe)
    return out


__all__ = ["picture", "solids_for", "top_faces", "np"]
