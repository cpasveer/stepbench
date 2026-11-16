"""Fast plane-section areas on a triangle mesh, used only to rank cutting
planes for SECTION_MIN. The areas it reports to the agent are always
measured again exactly on the solid; this only decides which planes are
worth measuring exactly.

For a closed mesh with outward-facing triangles, the section by a plane is
the union of closed polylines where triangles cross the plane. Each crossing
triangle contributes one segment, from where an edge goes down through the
plane to where an edge comes up; the area of the section is half the sum of
(start x end) . normal over all segments (the shoelace formula in 3D).
"""
from __future__ import annotations

import numpy as np


def _triangulate(solid, tolerance: float = 0.01):
    """Triangles with consistent outward orientation: each face's triangles
    are flipped when the face is reversed in the solid (OpenCASCADE stores a
    face's triangulation in the orientation of its surface, not of the
    solid)."""
    from OCP.BRep import BRep_Tool
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.TopAbs import TopAbs_REVERSED
    from OCP.TopLoc import TopLoc_Location

    BRepMesh_IncrementalMesh(solid.wrapped, tolerance, False, 0.1, True)
    verts, tris = [], []
    for f in solid.faces():
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(f.wrapped, loc)
        if tri is None:
            continue
        trsf = loc.Transformation()
        base = len(verts)
        for i in range(1, tri.NbNodes() + 1):
            p = tri.Node(i).Transformed(trsf)
            verts.append((p.X(), p.Y(), p.Z()))
        rev = f.wrapped.Orientation() == TopAbs_REVERSED
        for i in range(1, tri.NbTriangles() + 1):
            a, b, c = tri.Triangle(i).Get()
            tris.append((base + a - 1, base + c - 1, base + b - 1) if rev else (base + a - 1, base + b - 1, base + c - 1))
    return np.array(verts, dtype=float), np.array(tris, dtype=np.int64)


def mesh(solid, tolerance: float = 0.01):
    V, F = _triangulate(solid, tolerance)
    return V[F]  # triangles x 3 corners x xyz


def areas(T: np.ndarray, u, ks) -> np.ndarray:
    """Section area of mesh T by the planes {x : x.u = k} for every k in ks."""
    u = np.asarray(u, dtype=float)
    ks = np.asarray(ks, dtype=float)
    h = T @ u  # triangles x 3
    d = h[:, :, None] - ks[None, None, :]  # triangles x 3 x K
    d = np.where(d == 0.0, 1e-12, d)  # a corner exactly on the plane counts as above
    out = np.zeros(len(ks))
    pts_up = np.zeros((T.shape[0], len(ks), 3))
    pts_dn = np.zeros((T.shape[0], len(ks), 3))
    for i, j in ((0, 1), (1, 2), (2, 0)):
        di, dj = d[:, i, :], d[:, j, :]
        cross = di * dj < 0
        w = np.where(cross, di / np.where(cross, di - dj, 1.0), 0.0)  # triangles x K
        p = T[:, i, None, :] + (T[:, j, None, :] - T[:, i, None, :]) * w[:, :, None]
        up = cross & (di < 0)
        dn = cross & (di > 0)
        pts_up = np.where(up[:, :, None], p, pts_up)
        pts_dn = np.where(dn[:, :, None], p, pts_dn)
    c = np.cross(pts_dn, pts_up) @ u  # triangles x K (zero where no crossing)
    out = 0.5 * np.abs(c.sum(axis=0))
    return out


def mesh_indexed(solid, tolerance: float = 0.01):
    """Vertices and triangle corner indices with coincident vertices merged
    (faces are triangulated separately), so a mesh edge has one integer key."""
    V, F = _triangulate(solid, tolerance)
    q = np.round(V / 1e-4).astype(np.int64)  # faces meet within 0.0001 mm
    Vu, inv = np.unique(q, axis=0, return_inverse=True)
    inv = inv.reshape(-1)
    Vm = np.zeros((len(Vu), 3))
    Vm[inv] = V
    return Vm, inv[F]


def _components(n: int, a: np.ndarray, b: np.ndarray) -> np.ndarray:
    try:
        from scipy.sparse import coo_matrix
        from scipy.sparse.csgraph import connected_components

        g = coo_matrix((np.ones(len(a)), (a, b)), shape=(n, n))
        return connected_components(g, directed=False)[1]
    except ImportError:  # label propagation, plain numpy
        lab = np.arange(n)
        while True:
            m = np.minimum(lab[a], lab[b])
            new = lab.copy()
            np.minimum.at(new, a, m)
            np.minimum.at(new, b, m)
            new = new[new]
            if np.array_equal(new, lab):
                return lab
            lab = new


def regions(M, u, k: float, tol: float = 1e-9, with_tris: bool = False):
    """The separate regions of the section by the plane x.u = k, as
    [(area, centre)]: each closed loop of the section, with loops lying
    inside it (holes) subtracted. Loops are found exactly from the mesh:
    every crossing point lies on one mesh edge, and segments are joined
    where they share an edge. Used only to rank candidate cuts."""
    V, F = M
    u = np.asarray(u, dtype=float)
    h = V @ u - k
    h = np.where(h == 0.0, 1e-12, h)
    H = h[F]  # triangles x 3
    cross = ~(np.all(H > 0, axis=1) | np.all(H < 0, axis=1))
    if not cross.any():
        return []
    cross_idx = np.nonzero(cross)[0]
    Fc, Hc = F[cross], H[cross]
    nV = len(V)
    up_key = np.zeros(len(Fc), dtype=np.int64)
    dn_key = np.zeros(len(Fc), dtype=np.int64)
    up_p = np.zeros((len(Fc), 3))
    dn_p = np.zeros((len(Fc), 3))
    for i, j in ((0, 1), (1, 2), (2, 0)):
        di, dj = Hc[:, i], Hc[:, j]
        c = di * dj < 0
        w = np.where(c, di / np.where(c, di - dj, 1.0), 0.0)
        p = V[Fc[:, i]] + (V[Fc[:, j]] - V[Fc[:, i]]) * w[:, None]
        key = np.minimum(Fc[:, i], Fc[:, j]) * nV + np.maximum(Fc[:, i], Fc[:, j])
        up = c & (di < 0)
        dn = c & (di > 0)
        up_key = np.where(up, key, up_key)
        dn_key = np.where(dn, key, dn_key)
        up_p = np.where(up[:, None], p, up_p)
        dn_p = np.where(dn[:, None], p, dn_p)
    keys, inv = np.unique(np.concatenate([dn_key, up_key]), return_inverse=True)
    ia, ib = inv[: len(Fc)], inv[len(Fc):]
    lab = _components(len(keys), ia, ib)[ia]
    loops, li = np.unique(lab, return_inverse=True)
    contrib = 0.5 * (np.cross(dn_p, up_p) @ u)
    area = np.bincount(li, weights=contrib, minlength=len(loops))
    cnt = np.bincount(li, minlength=len(loops))
    cen = np.stack([np.bincount(li, weights=dn_p[:, d], minlength=len(loops)) for d in range(3)], 1) / cnt[:, None]
    if len(area) == 0:
        return []
    sgn = np.sign(area[np.argmax(np.abs(area))]) or 1.0
    # holes: loops of the other sign, given to the outer loop whose plane-
    # projected bounding box holds them and is the smallest such
    a2 = np.array([1.0, 0, 0]) if abs(u[0]) < 0.9 else np.array([0, 1.0, 0])
    e1 = np.cross(u, a2)
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(u, e1)
    P2 = np.stack([dn_p @ e1, dn_p @ e2], 1)
    bmin = np.stack([np.full(len(loops), np.inf)] * 2, 1)
    bmax = -bmin
    np.minimum.at(bmin, li, P2)
    np.maximum.at(bmax, li, P2)
    out_area = {int(i): abs(area[i]) for i in range(len(loops)) if np.sign(area[i]) == sgn}
    members = {o: [o] for o in out_area}
    for i in range(len(loops)):
        if np.sign(area[i]) == sgn:
            continue
        hosts = [o for o in out_area if np.all(bmin[o] <= bmin[i] + 1e-9) and np.all(bmax[i] <= bmax[o] + 1e-9)]
        if hosts:
            o = min(hosts, key=lambda o: out_area[o])
            out_area[o] -= abs(area[i])
            members[o].append(i)
    if not with_tris:
        return [(a, cen[o]) for o, a in sorted(out_area.items()) if a > tol]
    return [(a, cen[o], cross_idx[np.isin(li, members[o])]) for o, a in sorted(out_area.items()) if a > tol]


class Surface:
    """Triangle adjacency of a mesh, to test whether cutting along one
    section loop disconnects two sets of points on the surface."""

    def __init__(self, M):
        V, F = M
        self.n = len(F)
        e = np.concatenate([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]])
        e.sort(axis=1)
        tri = np.tile(np.arange(len(F)), 3)
        key = e[:, 0] * len(V) + e[:, 1]
        order = np.argsort(key, kind="stable")
        ks, ts = key[order], tri[order]
        same = ks[1:] == ks[:-1]
        self.a, self.b = ts[:-1][same], ts[1:][same]
        self.cent = V[F].mean(axis=1)

    def nearest(self, pts) -> np.ndarray:
        P = np.asarray(pts, dtype=float)
        d = ((self.cent[None, :, :] - P[:, None, :]) ** 2).sum(axis=2)
        return d.argmin(axis=1)

    def separates(self, removed: np.ndarray, ta: np.ndarray, tb: np.ndarray) -> bool:
        keep = np.ones(self.n, dtype=bool)
        keep[removed] = False
        m = keep[self.a] & keep[self.b]
        lab = _components(self.n, self.a[m], self.b[m])
        la = {int(lab[t]) for t in ta if keep[t]}
        lb = {int(lab[t]) for t in tb if keep[t]}
        return bool(la and lb and not (la & lb))
