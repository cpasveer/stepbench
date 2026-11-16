"""One still picture per joint, for the docs: flat-shaded, with a depth buffer.

Small on purpose (Pillow and numpy only). Each joint supplies the shapes to
draw in its joint frame; given parts are grey, the designed part blue, a
loose item (a key, a contact point) orange. Colours are Okabe-Ito, which
stay apart for colour-blind readers.
"""
from __future__ import annotations

import math

import build123d as bd
import numpy as np
from PIL import Image, ImageDraw, ImageFont

GREY = (190, 190, 190)
BLUE = (0, 114, 178)
PALE = (150, 196, 228)
ORANGE = (230, 159, 0)
RED = (213, 94, 0)


def _mesh(shape, tol):
    v, t = shape.tessellate(tol, 0.3)
    V = np.array([tuple(p) for p in v], float)
    T = np.array(t, int)
    return V, T


def picture(items, path, title="", notes=(), view=(1.0, -1.4, 0.9), size=(1100, 760), tol=0.08):
    """items: [(shape, rgb)]. Orthographic view from direction ``view``, with a depth buffer."""
    v = np.array(view, float)
    v /= np.linalg.norm(v)
    up = np.array([0.0, 0.0, 1.0])
    right = np.cross(up, v)
    right /= np.linalg.norm(right)
    sup = np.cross(v, right)
    light = np.array([0.4, -0.5, 1.0])
    light /= np.linalg.norm(light)
    tris = []
    for shape, col in items:
        V, T = _mesh(shape, tol)
        if len(T) == 0:
            continue
        P = V[T]
        n = np.cross(P[:, 1] - P[:, 0], P[:, 2] - P[:, 0])
        ln = np.linalg.norm(n, axis=1)
        good = ln > 1e-12
        P, n = P[good], n[good] / ln[good, None]
        shade = 0.45 + 0.55 * np.abs(n @ light)
        shade = np.where(n @ v < 0, shade * 0.8, shade)  # inner faces seen in a cut-away: darker
        cols = np.clip(np.array(col, float)[None, :] * shade[:, None], 0, 255)
        Z = np.stack([P @ right, P @ sup, P @ v], axis=-1)
        Z[..., 2] += 1e-4 * len(tris)  # later items win where faces coincide
        tris.append((Z, cols))
    X = np.concatenate([t[0] for t in tris])
    Cc = np.concatenate([t[1] for t in tris])
    W, H = size
    top, bottom = 70, 30 + 22 * len(notes)
    lo, hi = X[..., :2].reshape(-1, 2).min(axis=0), X[..., :2].reshape(-1, 2).max(axis=0)
    s = min((W - 80) / (hi[0] - lo[0]), (H - top - bottom) / (hi[1] - lo[1]))
    ox = (W - s * (hi[0] - lo[0])) / 2 - s * lo[0]
    oy = top + (H - top - bottom - s * (hi[1] - lo[1])) / 2 + s * hi[1]
    px = ox + s * X[..., 0]
    py = oy - s * X[..., 1]
    pz = X[..., 2]
    img = np.full((H, W, 3), 255.0)
    zb = np.full((H, W), -np.inf)
    for k in range(len(X)):
        x0, y0, z0 = px[k], py[k], pz[k]
        xa, xb = int(max(0, np.floor(x0.min()))), int(min(W - 1, np.ceil(x0.max())))
        ya, yb = int(max(0, np.floor(y0.min()))), int(min(H - 1, np.ceil(y0.max())))
        if xb < xa or yb < ya:
            continue
        gx, gy = np.meshgrid(np.arange(xa, xb + 1) + 0.5, np.arange(ya, yb + 1) + 0.5)
        d = (y0[1] - y0[2]) * (x0[0] - x0[2]) + (x0[2] - x0[1]) * (y0[0] - y0[2])
        if abs(d) < 1e-12:
            continue
        l1 = ((y0[1] - y0[2]) * (gx - x0[2]) + (x0[2] - x0[1]) * (gy - y0[2])) / d
        l2 = ((y0[2] - y0[0]) * (gx - x0[2]) + (x0[0] - x0[2]) * (gy - y0[2])) / d
        l3 = 1 - l1 - l2
        inside = (l1 >= -1e-6) & (l2 >= -1e-6) & (l3 >= -1e-6)
        if not inside.any():
            continue
        z = l1 * z0[0] + l2 * z0[1] + l3 * z0[2]
        sub = zb[ya:yb + 1, xa:xb + 1]
        win = inside & (z > sub)
        sub[win] = z[win]
        img[ya:yb + 1, xa:xb + 1][win] = Cc[k]
    # dark outline where depth jumps: reads the shapes without edge lines
    finite = np.isfinite(zb)
    zz = np.where(finite, zb, zb[finite].min() - 50 if finite.any() else 0)
    jump = np.zeros_like(finite)
    tol_z = 1.5 / s
    jump[1:, :] |= np.abs(zz[1:, :] - zz[:-1, :]) > tol_z
    jump[:, 1:] |= np.abs(zz[:, 1:] - zz[:, :-1]) > tol_z
    img[jump] = img[jump] * 0.35
    im = Image.fromarray(img.astype(np.uint8))
    dr = ImageDraw.Draw(im)
    try:
        f1 = ImageFont.truetype("Helvetica.ttc", 26)
        f2 = ImageFont.truetype("Helvetica.ttc", 17)
    except OSError:
        f1 = f2 = ImageFont.load_default()
    dr.text((30, 22), title, fill=(30, 30, 30), font=f1)
    for i, line in enumerate(notes):
        dr.text((30, H - 22 * (len(notes) - i) - 12), line, fill=(60, 60, 60), font=f2)
    im.save(path)
    return path


def half(shape, keep_y_positive=True, big=1000.0):
    """The shape cut along the xz plane: a section view."""
    box = bd.Pos(0, big / 2 if keep_y_positive else -big / 2, 0) * bd.Box(big, big, big)
    return shape & box


def ball(p, r=0.8):
    return bd.Pos(*p) * bd.Sphere(r)

