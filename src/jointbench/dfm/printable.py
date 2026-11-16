"""Can a connecting part for these fixtures be printed on filament without support at all?

A quick test on the arrangement alone, before any part is built. At each fixture the part has
a seat face (pressed on the mounting face, its outward normal pointing into the fixture) and,
opposite it, the faces under the bolt heads (pointing away from the fixture). One of the two
faces down in any build direction unless the mounting face stands within the overhang limit of
vertical: |n . d| <= sin(limit). The one way out is for the seat itself to lie on the bed
(n = d, the fixture under the bed and the head faces looking up); only then must every other
mounting face lie at or above the bed.

Necessary, not sufficient: a direction that passes still needs a part that prints. The
generator proves that with its own reference part.
"""
from __future__ import annotations

import math

import numpy as np

ON_BED_TOL_DEG = 1.0  # a mounting face within this of the bed plane can be the bed face


def _sphere(n: int) -> np.ndarray:
    k = np.arange(n) + 0.5
    z = 1 - 2 * k / n
    phi = math.pi * (1 + 5 ** 0.5) * k
    rho = np.sqrt(1 - z * z)
    return np.stack([rho * np.cos(phi), rho * np.sin(phi), z], axis=1)


def print_directions(faces, overhang_deg: float = 45.0, samples: int = 4000) -> list[dict]:
    """Build directions in which no fixture forces a downward face past the limit.

    ``faces``: [(origin, normal, points)] per fixture, normal pointing from the fixture into the
    part's space, ``points`` the mounting face's corners or bolt positions (to see that a face
    lies above the bed). Returns [{"direction", "margin_deg", "bed"}], best margin first;
    ``bed`` names the index of the face lying on the bed, or None. Empty: not printable without
    support."""
    s = math.sin(math.radians(overhang_deg))
    N = np.array([np.asarray(f[1], float) / np.linalg.norm(f[1]) for f in faces])
    out = []
    # 1. every mounting face upright enough: no face on the bed
    D = _sphere(samples)
    c = np.abs(D @ N.T)  # |n . d| per direction and fixture
    ok = (c <= s + 1e-12).all(axis=1)
    if ok.any():
        margin = np.degrees(np.arcsin(s)) - np.degrees(np.arcsin(np.clip(c[ok].max(axis=1), 0, 1)))
        k = int(np.argmax(margin))
        out.append({"direction": [float(v) for v in D[ok][k]], "margin_deg": float(margin[k]), "bed": None})
    # 2. one seat on the bed: d = its normal; every other face upright enough and above the bed
    for i, (o, n, pts) in enumerate(faces):
        d = N[i]
        rest = [j for j in range(len(faces)) if j != i]
        if any(abs(float(N[j] @ d)) > s + 1e-12 and float(N[j] @ d) < math.cos(math.radians(ON_BED_TOL_DEG))
               for j in rest):
            continue
        z0 = float(np.asarray(o, float) @ d)
        if any(float(np.asarray(p, float) @ d) < z0 - 1e-6 for j in rest for p in faces[j][2]):
            continue
        worst = max((abs(float(N[j] @ d)) for j in rest), default=0.0)
        out.append({"direction": [float(v) for v in d],
                    "margin_deg": float(overhang_deg - math.degrees(math.asin(min(worst, 1.0)))),
                    "bed": i})
    return sorted(out, key=lambda x: -x["margin_deg"])


def task_print_directions(task, pose: int = 0, overhang_deg: float = 45.0) -> list[dict]:
    """``print_directions`` for a connecting-part task: its mounting faces and bolt positions."""
    bolts = task.bolts(pose)
    faces = []
    for f in task.faces(pose):
        pts = [b.position for b in bolts if b.id.startswith(f.part)] + [f.origin]
        faces.append((f.origin, f.normal, pts))
    return print_directions(faces, overhang_deg)
