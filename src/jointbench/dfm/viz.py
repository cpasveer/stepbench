"""Showing the measurements: a viewer-neutral field file, and still pictures.

Field file (``--fields <dir>``): one JSON per checked part and process, the
part's display mesh with, for every rule, one value per triangle. The format
is described in ``docs/dfm.md``; nothing in it is specific to one viewer.

Pictures (``--images <dir>``): one PNG per rule, the part in an isometric
view coloured by that rule's field, with a legend that marks the limit.
Colour runs on viridis, whose lightness rises steadily from one end to the
other, so it reads for colour-blind viewers and prints in grey. Failing
regions are hatched black and white on top of their colour, so they stand
out in any print. Triangles a rule does not measure are plain light grey.
The view is chosen from the eight isometric corners as the one that shows
the most failing area (or, if nothing fails, the most measured area).
"""
from __future__ import annotations

import json
import math
import os

import numpy as np

from .geom import r

FORMAT = "jointbench-dfm-fields"
VERSION = 2  # 2: adds sampling, corner_values and probes; every version-1 key is unchanged

# colour scales: angles on a fixed range, so pictures compare; thicknesses
# and gaps capped at a multiple of the limit, because a straight ray along a
# plate reads its length, not its thickness, and would wash out the scale
FIXED = {"overhang_angle": (0.0, 90.0), "draft_angle": (0.0, 10.0), "rib_ratio": (0.0, 1.0),
         "inner_radius_ratio": (0.0, 1.0), "transition_ratio": (1.0, 6.0),
         "warp_ratio": (0.0, 80.0)}
CAP = {"wall_thickness": 5.0, "gap": 10.0, "section_thickness": 2.0, "bridge_span": 3.0,
       "hole_diameter": 5.0, "pin_diameter": 5.0, "sliver_width": 4.0, "step_height": 5.0,
       "face_area": 20.0,
       "escape_hole": 3.0}

ORANGE = np.array([230, 159, 0], float)  # Okabe-Ito orange: a marked region
GREY = np.array([222, 222, 222], float)
BG = (255, 255, 255)


# ------------------------------------------------------------ field file


def fields_json(rep, part) -> dict:
    tris = part.T
    edges = [[[r(c, 4) for c in p] for p in line] for line in part.edge_lines()]
    fields = []
    for x in rep.rules:
        f = x.surface
        if f is None:
            continue
        f.ready()
        vals = [None if not math.isfinite(v) else r(float(v), 4) for v in f.values]
        fin = f.values[np.isfinite(f.values)]
        fields.append({
            "id": f.id, "rule": f.rule, "title": f.title, "type": f.type, "unit": f.unit,
            "kind": f.kind, "limit": f.limit, "status": "pass" if x.ok else "fail",
            "reason": x.reason, "on": "triangle", "values": vals,
            "corner_values": corners(part, f.values) if f.type == "scalar" else None,
            "fail": [int(i) for i in np.flatnonzero(f.fail)],
            "range": [r(float(fin.min()), 4), r(float(fin.max()), 4)] if fin.size else None,
            "probes": probes(part, f, x) if f.type == "scalar" else [],
            "note": f.note, "marks": f.marks, "cut": f.cut, "where": x.where,
        })
    return {
        "format": FORMAT, "version": VERSION, "file": rep.file, "process": rep.process,
        "process_name": rep.process_name, "passed": rep.passed,
        "direction": {"name": rep.direction_name, "vector": rep.direction,
                      "chosen": rep.direction_chosen},
        "units": {"length": "mm", "angle": "deg", "area": "mm2"},
        "sampling": {"verdict_spacing_mm": part.spacing, "display_spacing_mm": part.spacing,
                     "note": "the display mesh is the mesh the rules judged (one sample per triangle, at its "
                             "centre); corner_values interpolate it smoothly within each face"},
        "mesh": {"vertices": [[r(c, 4) for c in v] for v in part.V],
                 "triangles": tris.tolist(),
                 "triangle_face": part.F.tolist(),
                 "edges": edges},
        "fields": fields,
    }


def corners(part, values) -> list:
    """Per triangle, a value at each of its three corners: the area-weighted
    mean of the measured triangles of the same face that share that corner.
    A viewer interpolating these gets a smooth map within every face and a
    sharp change at the solid's real edges (values are not blended across
    two faces)."""
    v = np.asarray(values, float)
    ok = np.isfinite(v)
    key = part.T.astype(np.int64) * (int(part.F.max()) + 1) + part.F[:, None]  # (vertex, face) per corner
    keys, inv = np.unique(key.ravel(), return_inverse=True)
    w = np.repeat(np.where(ok, part.W, 0.0), 3)
    num = np.bincount(inv, weights=w * np.repeat(np.nan_to_num(v), 3), minlength=len(keys))
    den = np.bincount(inv, weights=w, minlength=len(keys))
    cv = np.where(den > 0, num / np.where(den > 0, den, 1), np.nan)[inv].reshape(-1, 3)
    cv = np.where(ok[:, None], cv, np.nan)
    return [[None if not math.isfinite(c) else r(float(c), 4) for c in row] for row in cv]


def probes(part, f, x) -> list:
    """A few labelled points for a viewer: the worst place (as in the
    report), the lowest and highest measured values, and the quartiles."""
    v = np.asarray(f.values, float)
    idx = np.flatnonzero(np.isfinite(v))
    out = []
    if x.where and x.where[0].get("point") is not None and x.value is not None:
        out.append({"label": "worst", "point": x.where[0]["point"], "value": x.value, "face": x.where[0].get("face")})
    if not idx.size:
        return out
    order = idx[np.argsort(v[idx], kind="stable")]
    for label, q in (("lowest", 0.0), ("25 %", 0.25), ("median", 0.5), ("75 %", 0.75), ("highest", 1.0)):
        i = int(order[min(len(order) - 1, int(round(q * (len(order) - 1))))])
        out.append({"label": label, "point": [r(float(c), 4) for c in part.P[i]], "value": r(float(v[i]), 4),
                    "triangle": i, "face": int(part.F[i])})
    return out


def write_fields(rep, part, out_dir: str) -> str:
    os.makedirs(out_dir, exist_ok=True)
    stem = os.path.splitext(rep.file)[0]
    path = os.path.join(out_dir, f"{stem}.{rep.process}.fields.json")
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(fields_json(rep, part), fh, separators=(",", ":"))
    return path


# ------------------------------------------------------------- colours


_VIR = np.array([
    [0.2777273272234177, 0.005407344544966578, 0.3340998053353061],
    [0.1050930431085774, 1.404613529898575, 1.384590162594685],
    [-0.3308618287255563, 0.214847559468213, 0.09509516302823659],
    [-4.634230498983486, -5.799100973351585, -19.33244095627987],
    [6.228269936347081, 14.17993336680509, 56.69055260068105],
    [4.776384997670288, -13.74514537774601, -65.35303263337234],
    [-5.435455855934631, 4.645852612178535, 26.3124352495832],
])


def viridis(t) -> np.ndarray:
    """Viridis colour for t in 0..1 (polynomial fit), as 0..255 RGB."""
    t = np.clip(np.asarray(t, float), 0, 1)[..., None]
    c = _VIR[6]
    for k in range(5, -1, -1):
        c = _VIR[k] + t * c
    return np.clip(c, 0, 1) * 255


def hatch_mask(h, w):
    y, x = np.mgrid[0:h, 0:w]
    k = ((x + y) // 3) % 4
    return k == 0, k == 2  # black stripe, white stripe


# ------------------------------------------------------------ rendering

VIEWS = [(1, -1, 1), (-1, -1, 1), (1, 1, 1), (-1, 1, 1),
         (1, -1, -1), (-1, -1, -1), (1, 1, -1), (-1, 1, -1)]


def _camera(v):
    v = np.array(v, float) / np.linalg.norm(v)
    up = np.array([0, 0, 1.0])
    right = np.cross(up, v)
    right /= np.linalg.norm(right)
    sup = np.cross(v, right)
    return v, right, sup


def _raster(part, view, w, h, keep, cull, pad=0.08, frame=None):
    """Triangle-id and depth buffers for one orthographic view."""
    v, right, sup = _camera(view)
    V = part.V
    X, Y, D = V @ right, V @ sup, V @ v
    if frame is None:
        frame = (X.min(), X.max(), Y.min(), Y.max())
    x0, x1, y0, y1 = frame
    sc = min(w * (1 - 2 * pad) / max(x1 - x0, 1e-9), h * (1 - 2 * pad) / max(y1 - y0, 1e-9))
    ox = (w - sc * (x1 - x0)) / 2 - sc * x0
    oy = (h - sc * (y1 - y0)) / 2 + sc * y1
    sx, sy = X * sc + ox, oy - Y * sc
    ids = np.full((h, w), -1, int)
    depth = np.full((h, w), -np.inf)
    facing = part.N @ v
    for t in np.flatnonzero(keep):
        if cull and facing[t] <= 0:
            continue
        a, b, c = part.T[t]
        xs, ys = sx[[a, b, c]], sy[[a, b, c]]
        bx0, bx1 = max(int(math.floor(xs.min())), 0), min(int(math.ceil(xs.max())), w - 1)
        by0, by1 = max(int(math.floor(ys.min())), 0), min(int(math.ceil(ys.max())), h - 1)
        if bx0 > bx1 or by0 > by1:
            continue
        den = (ys[1] - ys[2]) * (xs[0] - xs[2]) + (xs[2] - xs[1]) * (ys[0] - ys[2])
        if abs(den) < 1e-12:
            continue
        py, px = np.mgrid[by0:by1 + 1, bx0:bx1 + 1] + 0.5
        l0 = ((ys[1] - ys[2]) * (px - xs[2]) + (xs[2] - xs[1]) * (py - ys[2])) / den
        l1 = ((ys[2] - ys[0]) * (px - xs[2]) + (xs[0] - xs[2]) * (py - ys[2])) / den
        l2 = 1 - l0 - l1
        ins = (l0 >= -1e-6) & (l1 >= -1e-6) & (l2 >= -1e-6)
        if not ins.any():
            continue
        z = l0 * D[a] + l1 * D[b] + l2 * D[c]
        sub_d = depth[by0:by1 + 1, bx0:bx1 + 1]
        sub_i = ids[by0:by1 + 1, bx0:bx1 + 1]
        upd = ins & (z > sub_d)
        sub_d[upd] = z[upd]
        sub_i[upd] = t
    proj = (right, sup, v, sc, ox, oy)
    return ids, depth, proj, frame


def _project(p, proj):
    right, sup, v, sc, ox, oy = proj
    p = np.asarray(p, float)
    return p @ right * sc + ox, oy - p @ sup * sc, p @ v


def _keep(part, f, view):
    if f.cut is None:
        return np.ones(len(part.T), bool), True
    v = np.array(view, float) / np.linalg.norm(view)
    cen = part.V[part.T].mean(axis=1)
    return (cen - np.array(f.cut)) @ v <= 0, False


def choose_view(part, f):
    """The isometric corner that shows the most failing (else measured) area."""
    want = f.fail if f.fail.any() else np.isfinite(f.values) & (f.values != 0 if f.type == "flag" else True)
    best, best_n = VIEWS[0], -1
    for view in VIEWS:
        keep, cull = _keep(part, f, view)
        ids, _, _, _ = _raster(part, view, 160, 120, keep, cull)
        vis = ids[ids >= 0]
        n = int(want[vis].sum()) if vis.size else 0
        if n > best_n:
            best, best_n = view, n
    return best


def render(rep, part, x, path: str, size=(1200, 760)):
    """One picture for one rule result ``x`` (with its surface field)."""
    from PIL import Image, ImageDraw, ImageFont

    f = x.surface.ready()
    W, H = size
    vw, vh = 820, H - 110
    view = choose_view(part, f)
    keep, cull = _keep(part, f, view)
    ids, depth, proj, _ = _raster(part, view, vw, vh, keep, cull)
    v = proj[2]

    vals = f.values
    fin = np.isfinite(vals)
    if f.type == "scalar":
        lo = float(vals[fin].min()) if fin.any() else 0.0
        hi = float(vals[fin].max()) if fin.any() else 1.0
        if f.limit is not None:
            lo, hi = min(lo, f.limit), max(hi, f.limit)
        capped = False
        if f.id in CAP and f.limit:
            lo = min(lo, 0.0)
            if hi > CAP[f.id] * f.limit:
                hi, capped = CAP[f.id] * f.limit, True
        if f.id in FIXED:
            lo, hi = FIXED[f.id]
        if hi - lo < 1e-9:
            lo, hi = lo - 1, hi + 1
        tri_col = np.where(fin[:, None], viridis((np.nan_to_num(vals) - lo) / (hi - lo)), GREY)
    else:
        lo = hi = 0.0
        tri_col = np.where((fin & (vals > 0))[:, None], ORANGE, GREY)
    light = np.array(v) * 0.7 + np.array([0, 0, 1.0]) * 0.3
    light /= np.linalg.norm(light)
    lam = np.abs(part.N @ light)
    measured = fin & ((vals > 0) if f.type == "flag" else True)
    shade = np.where(measured, 0.82 + 0.18 * lam, 0.62 + 0.38 * lam)
    tri_col = tri_col * shade[:, None]

    img = np.full((vh, vw, 3), 255.0)
    hit = ids >= 0
    img[hit] = tri_col[ids[hit]]
    fail_px = np.zeros((vh, vw), bool)
    fail_px[hit] = f.fail[ids[hit]]
    blk, wht = hatch_mask(vh, vw)
    img[fail_px & blk] = (20, 20, 20)
    img[fail_px & wht] = (255, 255, 255)

    # true edges, drawn where visible
    for line in part.edge_lines():
        X, Y, D = _project(line, proj)
        for k in range(len(line) - 1):
            n = int(max(abs(X[k + 1] - X[k]), abs(Y[k + 1] - Y[k]))) + 1
            for s in np.linspace(0, 1, n + 1):
                px = int(X[k] + s * (X[k + 1] - X[k]))
                py = int(Y[k] + s * (Y[k + 1] - Y[k]))
                if 0 <= px < vw and 0 <= py < vh:
                    dz = D[k] + s * (D[k + 1] - D[k])
                    if dz >= depth[py, px] - 0.05:
                        img[py, px] = (45, 45, 45)

    canvas = Image.new("RGB", (W, H), BG)
    canvas.paste(Image.fromarray(img.astype(np.uint8)), (0, 80))
    dr = ImageDraw.Draw(canvas)
    font = ImageFont.load_default(size=15)
    big = ImageFont.load_default(size=20)
    small = ImageFont.load_default(size=13)

    def mark_line(pts, label, fail):
        X, Y, _ = _project(np.array(pts), proj)
        xy = [(float(a), float(b) + 80) for a, b in zip(X, Y)]
        dr.line(xy, fill=(255, 255, 255), width=7)
        dr.line(xy, fill=(0, 0, 0), width=3)
        if fail:
            dr.line(xy, fill=(255, 255, 255), width=1)
        if label:
            mx, my = xy[len(xy) // 2]
            dr.text((mx + 6, my - 20), label, fill=(0, 0, 0), font=font,
                    stroke_width=3, stroke_fill=(255, 255, 255))

    said = set()
    for m in f.marks:
        if m["type"] in ("segment", "polyline"):
            label = m.get("label", "")
            if m["type"] == "polyline":  # name a kind of edge once, not on every edge
                label = "" if label in said else label
                said.add(m.get("label", ""))
            mark_line(m["points"], label, m.get("fail", False))
        elif m["type"] == "point":
            X, Y, _ = _project(np.array([m["point"]]), proj)
            cx, cy = float(X[0]), float(Y[0]) + 80
            dr.ellipse((cx - 12, cy - 12, cx + 12, cy + 12), outline=(0, 0, 0), width=3)
            dr.text((cx + 16, cy - 8), m.get("label", ""), fill=(0, 0, 0), font=font,
                    stroke_width=3, stroke_fill=(255, 255, 255))
    if not x.ok and x.where and x.where[0].get("point") is not None and f.type == "scalar":
        X, Y, _ = _project(np.array([x.where[0]["point"]]), proj)
        cx, cy = float(X[0]), float(Y[0]) + 80
        dr.ellipse((cx - 14, cy - 14, cx + 14, cy + 14), outline=(255, 255, 255), width=6)
        dr.ellipse((cx - 14, cy - 14, cx + 14, cy + 14), outline=(0, 0, 0), width=3)
        u = "" if x.unit == "ratio" else f" {x.unit}"
        if not f.marks:
            dr.text((cx + 18, cy - 10), f"worst {x.value:g}{u}", fill=(0, 0, 0), font=font,
                    stroke_width=3, stroke_fill=(255, 255, 255))

    # header
    status = "PASS" if x.ok else "FAIL"
    dr.text((20, 14), f"{x.id}   {x.title}", fill=(0, 0, 0), font=big)
    dv = ", ".join(f"{c:g}" for c in rep.direction)
    dr.text((20, 44), f"{status}: {x.reason}", fill=(0, 0, 0), font=font)
    dtext = f"{rep.direction_name} ({dv}), {rep.direction_chosen}  |  " if rep.direction else ""
    dr.text((20, H - 26), f"{rep.file}  |  {rep.process_name}  |  {dtext}"
            f"view from ({', '.join(str(c) for c in view)})"
            + ("  |  cut open" if f.cut is not None else ""),
            fill=(90, 90, 90), font=small)

    # legend
    lx = vw + 40
    unit = "" if f.unit == "ratio" else f" {f.unit}"
    if f.type == "scalar":
        top, bot, bw = 120, 520, 34
        bar = np.zeros((bot - top, bw, 3))
        tv = np.linspace(hi, lo, bot - top)
        bar[:] = viridis((tv - lo) / (hi - lo))[:, None, :]
        if f.limit is not None:
            bad = tv < f.limit if f.kind == "min" else tv > f.limit
            b1, w1 = hatch_mask(bot - top, bw)
            bar[bad[:, None] & b1] = (20, 20, 20)
            bar[bad[:, None] & w1] = (255, 255, 255)
        canvas.paste(Image.fromarray(bar.astype(np.uint8)), (lx, top))
        dr.rectangle((lx, top, lx + bw, bot), outline=(0, 0, 0))

        def ypos(val):
            return top + (hi - val) / (hi - lo) * (bot - top)

        ylim = ypos(f.limit) if f.limit is not None else -1e9
        for val, pre in ((hi, ">= " if capped else ""), (lo, "")):
            y = ypos(val)
            dr.line((lx + bw, y, lx + bw + 6, y), fill=(0, 0, 0))
            if abs(y - ylim) > 22:
                dr.text((lx + bw + 10, y - 8), f"{pre}{val:.3g}{unit}", fill=(0, 0, 0), font=font)
        if f.limit is not None:
            y = ypos(f.limit)
            dr.line((lx - 8, y, lx + bw + 8, y), fill=(0, 0, 0), width=3)
            dr.text((lx + bw + 10, y - 8), f"limit {f.limit:g}{unit} ({f.kind})", fill=(0, 0, 0),
                    font=font)
        dr.text((lx, 92), f"{f.id.replace('_', ' ')}" + (f" ({f.unit})" if f.unit != "ratio" else ""),
                fill=(0, 0, 0), font=font)
        ky = 545
    else:
        ky = 120
        dr.rectangle((lx, ky, lx + 34, ky + 22), fill=tuple(ORANGE.astype(int)), outline=(0, 0, 0))
        dr.text((lx + 44, ky + 3), f.id.replace("_", " "), fill=(0, 0, 0), font=font)
        ky += 40
    sw = np.full((22, 34, 3), 255.0)
    b1, w1 = hatch_mask(22, 34)
    sw[b1] = (20, 20, 20)
    canvas.paste(Image.fromarray(sw.astype(np.uint8)), (lx, ky))
    dr.rectangle((lx, ky, lx + 34, ky + 22), outline=(0, 0, 0))
    dr.text((lx + 44, ky + 3), "fails the rule", fill=(0, 0, 0), font=font)
    ky += 34
    dr.rectangle((lx, ky, lx + 34, ky + 22), fill=tuple(GREY.astype(int)), outline=(0, 0, 0))
    dr.text((lx + 44, ky + 3), "not measured here", fill=(0, 0, 0), font=font)
    ky += 40
    for line in _wrap(f.note, 40):
        dr.text((lx, ky), line, fill=(70, 70, 70), font=small)
        ky += 17
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    canvas.save(path, optimize=True)
    return path


def _wrap(text, n):
    out, cur = [], ""
    for word in text.split():
        if len(cur) + len(word) + 1 > n:
            out.append(cur)
            cur = word
        else:
            cur = f"{cur} {word}".strip()
    if cur:
        out.append(cur)
    return out


def write_images(rep, part, out_dir: str) -> list[str]:
    stem = os.path.splitext(rep.file)[0]
    paths = []
    for x in rep.rules:
        if x.surface is not None:
            paths.append(render(rep, part, x, os.path.join(out_dir, f"{stem}.{rep.process}.{x.id}.png")))
    return paths
