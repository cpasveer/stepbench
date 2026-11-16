"""Showing the stiffness result: a displacement field file and still pictures.

Field file (``--fields DIR``, format ``jointbench-stiffness-fields`` v1,
described in ``docs/stiffness.md``): the surface of the analysis mesh, which
seat triangles are held and which are loaded, and for every load case the
displacement of every surface vertex per unit load, plus its size per
triangle so a viewer that colours triangles (as for the DFM fields) can use
it unchanged.

Pictures (``--images DIR``): for each load case, the part deformed under
the load, exaggerated by a stated factor, over its undeformed shape in pale
grey. Colour is the size of the displacement on viridis (colour-blind safe,
prints in grey); the held seat is hatched black and white; an arrow shows
the load at the loaded mount.
"""
from __future__ import annotations

import json
import os
from types import SimpleNamespace

import numpy as np

from ..dfm.viz import VIEWS, _project, _raster, hatch_mask, viridis

FORMAT = "jointbench-stiffness-fields"
VERSION = 2  # 2: adds stress (von Mises) per case and the stress envelope; every version-1 key is unchanged


def _g(v, n=4):
    return float(f"{v:.{n}g}")


def _surface(rep):
    P, T = rep.surface
    used = np.unique(T)
    remap = -np.ones(len(P), np.int64)
    remap[used] = np.arange(len(used))
    return used, P[used], remap[T]


def fields_json(rep) -> dict:
    used, V, T = _surface(rep)
    mounts = [{"name": k, "role": "held" if held else "loaded",
               "triangles": [int(i) for i in np.flatnonzero(flag)]}
              for k, (flag, held) in rep.regions.items()]
    cases = []
    span = float(np.linalg.norm(V.max(0) - V.min(0)))
    env = None
    for c in rep.cases:
        d = rep.displacements[c.label][used]
        mag = np.linalg.norm(d, axis=1)[T].mean(axis=1)
        entry = {**c.to_json(),
                 "displacement": [[_g(a) for a in row] for row in d],
                 "magnitude": [_g(a) for a in mag],
                 "range": [_g(float(mag.min())), _g(float(mag.max()))]}
        if c.label in rep.stresses:
            load = design_load(rep, c)
            sv = rep.stresses[c.label][used] * load
            entry["design_load"] = {"value": load, "unit": "N" if c.kind == "force" else "N mm"}
            entry["stress"] = stress_field(V, T, sv)
            dmax = float(np.linalg.norm(d, axis=1).max()) * load
            entry["deformed"] = {"exaggeration": _g(0.08 * span / dmax) if dmax > 0 else None,
                                 "note": "displacement x design_load x exaggeration moves the largest movement to "
                                         "8 % of the part's size"}
            env = sv if env is None else np.maximum(env, sv)
        cases.append(entry)
    j = rep.to_json()
    return {
        "format": FORMAT, "version": VERSION, "file": rep.file, "task": rep.task,
        "material": rep.material, "E": rep.E, "nu": rep.nu,
        "units": {"length": "mm", "displacement": "mm per N (force cases) or per N mm (moment cases)"},
        "summary": {**{k: j[k] for k in ("volume", "mass", "weakest_force", "weakest_moment", "par", "precision")},
                    # the one figure: the weakest way to load the part (forces and moments together)
                    **({"weakest": {"stiffness": rep.figure.stiffness, "unit": "N/mm", "mount": rep.figure.mount,
                                    "description": rep.figure.description,
                                    "per_cm3": rep.figure.stiffness / rep.volume if rep.volume else None}}
                       if getattr(rep, "figure", None) else {})},
        "mesh": {"vertices": [[round(float(a), 4) for a in v] for v in V], "triangles": T.tolist()},
        "mounts": mounts,
        "cases": cases,
        "stress_envelope": stress_field(V, T, env) if env is not None else None,
        "stress_note": STRESS_NOTE,
        "design_loads": rep.design_loads,
    }


STRESS_NOTE = ("Von Mises stress in MPa under the design load of each case (force cases: the design force; "
               "moment cases: that force times the span), for viewing only: it is NOT a requirement and in no "
               "verdict. At a sharp inside corner the peak grows as the elements get finer (a stress singularity), "
               "so the colour scale ends at the 99th percentile of the surface values ('scale'), not at the maximum. "
               "stress_envelope is the highest stress over the six cases at each point: the picture to show first.")


def design_load(rep, case) -> float:
    dl = rep.design_loads.get(case.loaded, {"force_N": 1.0, "moment_Nmm": 1.0})
    return float(dl["force_N"] if case.kind == "force" else dl["moment_Nmm"])


def stress_field(V, T, sv) -> dict:
    """Per-vertex values (smooth: the solver's stresses are averaged at each
    mesh corner), per-triangle means, range, the colour scale at the 99th
    percentile, and probe points (the peak and representative values)."""
    sv = np.asarray(sv, float)
    tri = sv[T].mean(axis=1)
    p99 = float(np.percentile(sv, 99))
    order = np.argsort(sv, kind="stable")
    probes = []
    for label, q in (("peak", 1.0), ("99 %", 0.99), ("90 %", 0.90), ("median", 0.5)):
        i = int(order[min(len(order) - 1, int(round(q * (len(order) - 1))))])
        probes.append({"label": label, "point": [round(float(c), 4) for c in V[i]], "value": _g(float(sv[i])),
                       "vertex": i})
    return {"unit": "MPa", "on": "vertex", "vertex_values": [_g(float(a)) for a in sv],
            "values": [_g(float(a)) for a in tri], "range": [_g(float(sv.min())), _g(float(sv.max()))],
            "scale": [0.0, _g(p99)], "scale_note": "colours end at the 99th percentile; above it, the top colour",
            "probes": probes}


def write_fields(rep, out_dir: str) -> str:
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, f"{os.path.splitext(rep.file)[0]}.stiffness.fields.json")
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(fields_json(rep), fh, separators=(",", ":"))
    return path


def _normals(V, T):
    n = np.cross(V[T[:, 1]] - V[T[:, 0]], V[T[:, 2]] - V[T[:, 0]])
    return n / np.maximum(np.linalg.norm(n, axis=1), 1e-12)[:, None]


def _view(part):
    """The isometric corner that shows the most surface."""
    best, best_n = VIEWS[0], -1
    keep = np.ones(len(part.T), bool)
    for view in VIEWS:
        ids, _, _, _ = _raster(part, view, 160, 120, keep, True)
        n = int((ids >= 0).sum())
        if n > best_n:
            best, best_n = view, n
    return best


def render(rep, case, path: str, size=(1200, 760)) -> str:
    from PIL import Image, ImageDraw, ImageFont

    used, V, T = _surface(rep)
    d = rep.displacements[case.label][used]
    mag_v = np.linalg.norm(d, axis=1)
    dmax = float(mag_v.max()) or 1.0
    span = float(np.linalg.norm(V.max(0) - V.min(0)))
    factor = 0.08 * span / dmax
    Vd = V + d * factor
    base = SimpleNamespace(V=V, T=T, N=_normals(V, T))
    bent = SimpleNamespace(V=Vd, T=T, N=_normals(Vd, T))
    allv = SimpleNamespace(V=np.vstack([V, Vd]), T=T, N=base.N)
    view = _view(base)
    W, H = size
    vw, vh = 820, H - 125
    keep = np.ones(len(T), bool)
    _, _, _, frame = _raster(allv, view, 40, 30, np.zeros(len(T), bool), True)
    ids0, _, proj, _ = _raster(base, view, vw, vh, keep, True, frame=frame)
    ids1, _, _, _ = _raster(bent, view, vw, vh, keep, True, frame=frame)

    v = proj[2]
    light = np.array(v) * 0.7 + np.array([0, 0, 1.0]) * 0.3
    light /= np.linalg.norm(light)
    mag_t = mag_v[T].mean(axis=1)
    col = viridis(mag_t / dmax) * (0.78 + 0.22 * np.abs(bent.N @ light))[:, None]
    held = np.zeros(len(T), bool)
    for flag, is_held in rep.regions.values():
        if is_held:
            held |= flag
    img = np.full((vh, vw, 3), 255.0)
    h0 = ids0 >= 0
    img[h0] = (232, 232, 232)
    h1 = ids1 >= 0
    img[h1] = col[ids1[h1]]
    blk, wht = hatch_mask(vh, vw)
    hp = np.zeros((vh, vw), bool)
    hp[h1] = held[ids1[h1]]
    img[hp & blk] = (20, 20, 20)
    img[hp & wht] = (255, 255, 255)

    canvas = Image.new("RGB", (W, H), (255, 255, 255))
    canvas.paste(Image.fromarray(img.astype(np.uint8)), (0, 90))
    dr = ImageDraw.Draw(canvas)
    font = ImageFont.load_default(size=15)
    big = ImageFont.load_default(size=20)
    small = ImageFont.load_default(size=13)

    # the load: an arrow (force) or a curled arrow label (moment) at the loaded mount
    reg = [k for k, (_, is_held) in rep.regions.items() if not is_held and k == case.loaded]
    if reg:
        flag = rep.regions[case.loaded][0]
        c0 = V[T[flag]].reshape(-1, 3).mean(axis=0) if flag.any() else V.mean(axis=0)
        ax = np.array(case.movement[:3] if case.kind == "force" else case.movement[3:])
        ax = ax / (np.linalg.norm(ax) or 1)
        X, Y, _ = _project(np.array([c0, c0 + ax * 0.25 * span]), proj)
        p0, p1 = (float(X[0]), float(Y[0]) + 90), (float(X[1]), float(Y[1]) + 90)
        for wdt, colr in ((9, (255, 255, 255)), (4, (0, 0, 0))):
            dr.line([p0, p1], fill=colr, width=wdt)
        dv = np.array(p1) - np.array(p0)
        if np.linalg.norm(dv) > 1:
            dv /= np.linalg.norm(dv)
            nv = np.array([-dv[1], dv[0]])
            tip = np.array(p1)
            dr.polygon([tuple(tip), tuple(tip - 16 * dv + 8 * nv), tuple(tip - 16 * dv - 8 * nv)], fill=(0, 0, 0))
        word = "force" if case.kind == "force" else "moment (axis shown)"
        dr.text((p1[0] + 8, p1[1] - 8), f"unit {word}", fill=(0, 0, 0), font=font,
                stroke_width=3, stroke_fill=(255, 255, 255))

    kind = "N/mm" if case.kind == "force" else "N mm/rad"
    dr.text((20, 14), f"Stiffness   held at {case.fixed}, {case.label}", fill=(0, 0, 0), font=big)
    par = ""
    if rep.par:
        par = f"   |   x{rep.par.ratios[case.label]:.2f} of the reference in this case"
    dr.text((20, 44), f"this case: {case.stiffness:.4g} {kind}{par}", fill=(0, 0, 0), font=font)
    if rep.figure:
        w = rep.figure
        dr.text((20, 64), f"weakest way to load the part: {w.description} at {w.mount} -> {w.stiffness:.4g} N/mm",
                fill=(0, 0, 0), font=font)
    dr.text((20, H - 26), f"{rep.file}  |  {rep.material}, linear static  |  deformation shown "
            f"x{factor:.3g} (per unit load)  |  view from ({', '.join(str(c) for c in view)})",
            fill=(90, 90, 90), font=small)

    lx, top, bot, bw = vw + 40, 120, 520, 34
    unit = "um per N" if case.kind == "force" else "um per N mm"
    bar = viridis(np.linspace(1, 0, bot - top))[:, None, :].repeat(bw, axis=1)
    canvas.paste(Image.fromarray(bar.astype(np.uint8)), (lx, top))
    dr.rectangle((lx, top, lx + bw, bot), outline=(0, 0, 0))
    dr.text((lx, 92), f"displacement ({unit})", fill=(0, 0, 0), font=font)
    for val, y in ((dmax, top), (0.0, bot)):
        dr.line((lx + bw, y, lx + bw + 6, y), fill=(0, 0, 0))
        dr.text((lx + bw + 10, y - 8), f"{val * 1000:.3g}", fill=(0, 0, 0), font=font)
    ky = 545
    sw = np.full((22, 34, 3), 255.0)
    b1, _ = hatch_mask(22, 34)
    sw[b1] = (20, 20, 20)
    canvas.paste(Image.fromarray(sw.astype(np.uint8)), (lx, ky))
    dr.rectangle((lx, ky, lx + 34, ky + 22), outline=(0, 0, 0))
    dr.text((lx + 44, ky + 3), f"held seat ({case.fixed})", fill=(0, 0, 0), font=font)
    ky += 34
    dr.rectangle((lx, ky, lx + 34, ky + 22), fill=(232, 232, 232), outline=(0, 0, 0))
    dr.text((lx + 44, ky + 3), "shape before loading", fill=(0, 0, 0), font=font)
    ky += 40
    for line in (f"Deformation exaggerated x{factor:.3g}.",
                 "Linear: twice the load moves", "everything twice as far."):
        dr.text((lx, ky), line, fill=(70, 70, 70), font=small)
        ky += 17
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    canvas.save(path, optimize=True)
    return path


def dominant_case(rep):
    """The single load case closest to the part's weakest way of loading."""
    w = np.array(rep.figure.direction) ** 2
    name = ["Fx", "Fy", "Fz", "Mx", "My", "Mz"][int(np.argmax(w))]
    return next(c for c in rep.cases if c.label == f"{name} at {rep.figure.mount}")


def render_stress(rep, path: str, size=(1200, 760)) -> str:
    """The stress envelope (highest von Mises over the six cases, each under
    its design load) on the undeformed part: viridis, scaled to the 99th
    percentile, the peak ringed. For viewing only, and says so."""
    from PIL import Image, ImageDraw, ImageFont

    used, V, T = _surface(rep)
    env = None
    for c in rep.cases:
        if c.label in rep.stresses:
            sv = rep.stresses[c.label][used] * design_load(rep, c)
            env = sv if env is None else np.maximum(env, sv)
    p99 = float(np.percentile(env, 99)) or 1.0
    base = SimpleNamespace(V=V, T=T, N=_normals(V, T))
    view = _view(base)
    W, H = size
    vw, vh = 820, H - 125
    ids, _, proj, _ = _raster(base, view, vw, vh, np.ones(len(T), bool), True)
    v = proj[2]
    light = np.array(v) * 0.7 + np.array([0, 0, 1.0]) * 0.3
    light /= np.linalg.norm(light)
    col = viridis(np.clip(env[T].mean(axis=1) / p99, 0, 1)) * (0.8 + 0.2 * np.abs(base.N @ light))[:, None]
    img = np.full((vh, vw, 3), 255.0)
    hit = ids >= 0
    img[hit] = col[ids[hit]]
    canvas = Image.new("RGB", (W, H), (255, 255, 255))
    canvas.paste(Image.fromarray(img.astype(np.uint8)), (0, 90))
    dr = ImageDraw.Draw(canvas)
    font, big, small = (ImageFont.load_default(size=k) for k in (15, 20, 13))
    k = int(np.argmax(env))
    X, Y, _ = _project(np.array([V[k]]), proj)
    cx, cy = float(X[0]), float(Y[0]) + 90
    for wdt, colr in ((6, (255, 255, 255)), (3, (0, 0, 0))):
        dr.ellipse((cx - 14, cy - 14, cx + 14, cy + 14), outline=colr, width=wdt)
    dr.text((cx + 18, cy - 10), f"peak {env[k]:.3g} MPa", fill=(0, 0, 0), font=font, stroke_width=3,
            stroke_fill=(255, 255, 255))
    dr.text((20, 14), "Stress (von Mises), highest over the six load cases", fill=(0, 0, 0), font=big)
    dl = "; ".join(f"{m}: {d['force_N']:g} N / {d['moment_Nmm']:g} N mm" for m, d in rep.design_loads.items())
    dr.text((20, 44), f"under the design load ({dl})", fill=(0, 0, 0), font=font)
    dr.text((20, 64), "for viewing only: not a requirement, in no verdict", fill=(0, 0, 0), font=font)
    dr.text((20, H - 26), f"{rep.file}  |  {rep.material}, linear static  |  view from "
            f"({', '.join(str(c) for c in view)})", fill=(90, 90, 90), font=small)
    lx, top, bot, bw = vw + 40, 120, 520, 34
    bar = viridis(np.linspace(1, 0, bot - top))[:, None, :].repeat(bw, axis=1)
    canvas.paste(Image.fromarray(bar.astype(np.uint8)), (lx, top))
    dr.rectangle((lx, top, lx + bw, bot), outline=(0, 0, 0))
    dr.text((lx, 92), "stress (MPa)", fill=(0, 0, 0), font=font)
    for val, y, pre in ((p99, top, ">= "), (0.0, bot, "")):
        dr.line((lx + bw, y, lx + bw + 6, y), fill=(0, 0, 0))
        dr.text((lx + bw + 10, y - 8), f"{pre}{val:.3g}", fill=(0, 0, 0), font=font)
    ky = 545
    for line in ("The scale ends at the 99th", "percentile, so one sharp corner", "does not wash out the picture.",
                 "At a sharp inside corner the", "peak grows with finer elements."):
        dr.text((lx, ky), line, fill=(70, 70, 70), font=small)
        ky += 17
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    canvas.save(path, optimize=True)
    return path


def write_images(rep, out_dir: str, cases=None) -> list[str]:
    stem = os.path.splitext(rep.file)[0]
    out = []
    if rep.stresses:
        out.append(render_stress(rep, os.path.join(out_dir, f"{stem}.stiffness.stress.png")))
    for c in rep.cases:
        if cases and c.label not in cases:
            continue
        name = c.label.replace(" ", "_")
        out.append(render(rep, c, os.path.join(out_dir, f"{stem}.stiffness.{name}.png")))
    return out
