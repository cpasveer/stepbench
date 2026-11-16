"""Mesh a STEP solid and solve linear static elasticity on it.

Meshing is gmsh (reads the STEP through the same OpenCASCADE kernel as the
rest of the benchmark); the finite-element model is scikit-fem with
quadratic (10-node) tetrahedra; the equations are solved with SciPy's
conjugate-gradient solver to a relative residual of 1e-10, preconditioned by
a two-level cycle over the part's own corner nodes (``_Preconditioner``). All are ordinary pip packages, installed with the
``stiffness`` extra. Everything runs single-threaded with fixed settings,
so the same file always gives the same numbers.

Mounts are attached through a *distributing coupling*: a load is spread
over the mount's seat and bolt-hole surfaces as if through a rigid washer
(force evenly per area, moment as a linear spread about the mount's origin),
and the mount's movement is read back as the best-fit rigid motion of those
same surfaces. Load and measurement are each other's mirror image, so the
6 x 6 flexibility matrix of a mount comes out symmetric -- a check the
self-test makes.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


class MeshError(RuntimeError):
    pass


def available() -> str | None:
    """None if the solver can run, else what to install."""
    try:
        import gmsh  # noqa: F401
        import pyamg  # noqa: F401
        import threadpoolctl  # noqa: F401
        import skfem  # noqa: F401
    except ImportError:
        return "the stiffness score needs gmsh, scikit-fem, pyamg and threadpoolctl: uv sync --extra stiffness (or pip install 'jointbench[stiffness]')"
    return None


@dataclass
class Mesh:
    p: np.ndarray  # (n, 3) corner node positions, mm
    t: np.ndarray  # (m, 4) tetrahedra (corner nodes)
    solids: int
    # curved (second-order) elements: for every element edge (a, b), a < b,
    # the position of its middle node, which gmsh places on the true surface
    mid_pairs: np.ndarray | None = None  # (k, 2)
    mid_xyz: np.ndarray | None = None  # (k, 3)

    @property
    def curved(self) -> bool:
        return self.mid_pairs is not None


def mesh_step(path: str, h: float, curvature_points: int = 12, order: int = 2, max_retries: int | None = None) -> Mesh:
    """Tetrahedral mesh of the solid. ``order`` 2: curved elements -- every
    element edge gets a middle node on the true surface, so a round member
    is not meshed as a polygon (12 straight sides make a rod about 6 %
    too flexible; curved, within 1 %). If gmsh cannot curve a mesh, it is
    used straight, and the Mesh says so (``curved`` False)."""
    import gmsh

    gmsh.initialize(interruptible=False)
    try:
        gmsh.option.setNumber("General.Terminal", 0)
        gmsh.option.setNumber("General.Verbosity", 0)
        gmsh.option.setNumber("General.NumThreads", 1)
        gmsh.option.setNumber("Geometry.OCCImportLabels", 0)
        gmsh.option.setNumber("Mesh.MeshSizeMax", h)
        gmsh.option.setNumber("Mesh.MeshSizeMin", 0.0)
        gmsh.option.setNumber("Mesh.MeshSizeFromCurvature", curvature_points)
        gmsh.option.setNumber("Mesh.Algorithm", 6)  # 2D frontal-Delaunay
        gmsh.option.setNumber("Mesh.Algorithm3D", 1)  # 3D Delaunay
        gmsh.option.setNumber("Mesh.Optimize", 1)
        gmsh.option.setNumber("Mesh.RandomSeed", 1)
        if max_retries is None:
            from . import settings

            max_retries = settings()["mesh"].get("max_retries", 10)
        # a surface whose edge mesh crosses itself is re-split and retried; each retry
        # costs about four times the last, so a part the mesher cannot handle would
        # otherwise take an hour to fail. A count, not a clock: the same for every run
        gmsh.option.setNumber("Mesh.MaxRetries", int(max_retries))
        try:
            gmsh.model.occ.importShapes(path)
            gmsh.model.occ.synchronize()
        except Exception as e:  # noqa: BLE001
            raise MeshError(f"gmsh could not read the STEP file: {e}") from None
        vols = gmsh.model.getEntities(3)
        if not vols:
            raise MeshError("the STEP file holds no solid")
        try:
            gmsh.model.mesh.generate(3)
        except Exception as e:  # noqa: BLE001
            raise MeshError(f"gmsh could not mesh the solid: {e}") from None
        etype, nper = 4, 4
        if order == 2:
            try:
                gmsh.model.mesh.setOrder(2)
                etype, nper = 11, 10
            except Exception:  # noqa: BLE001 -- keep the straight mesh
                pass
        tags, xyz, _ = gmsh.model.mesh.getNodes()
        _, ntags = gmsh.model.mesh.getElementsByType(etype)
    finally:
        gmsh.finalize()
    if len(ntags) == 0:
        raise MeshError("gmsh produced no tetrahedra (the surface mesh did not close)")
    tags = tags.astype(np.int64)
    idx = np.zeros(int(tags.max()) + 1, np.int64)
    idx[tags] = np.arange(len(tags))
    p = xyz.reshape(-1, 3)
    full = idx[ntags.astype(np.int64)].reshape(-1, nper)
    t = full[:, :4]
    used = np.unique(t)
    remap = -np.ones(len(p), np.int64)
    remap[used] = np.arange(len(used))
    t2 = remap[t]
    mesh = Mesh(np.ascontiguousarray(p[used]), np.ascontiguousarray(t2), len(vols))
    if nper == 10:
        # gmsh's 10-node tetrahedron: nodes 4-9 sit on edges 0-1, 1-2, 0-2, 0-3, 2-3, 1-3
        pairs = np.concatenate([t2[:, [a, b]] for a, b in ((0, 1), (1, 2), (0, 2), (0, 3), (2, 3), (1, 3))])
        mids = np.concatenate([full[:, k] for k in range(4, 10)])
        pairs.sort(axis=1)
        key = pairs[:, 0] * len(used) + pairs[:, 1]
        key, first = np.unique(key, return_index=True)
        mesh.mid_pairs = pairs[first]
        mesh.mid_xyz = p[mids[first]]
    return mesh


def tet_volume(m: Mesh) -> float:
    a, b, c, d = (m.p[m.t[:, k]] for k in range(4))
    return float(np.abs(np.einsum("ij,ij->i", np.cross(b - a, c - a), d - a)).sum() / 6)


def _skew(r):
    z = np.zeros(len(r))
    return np.stack([np.stack([z, -r[:, 2], r[:, 1]], -1),
                     np.stack([r[:, 2], z, -r[:, 0]], -1),
                     np.stack([-r[:, 1], r[:, 0], z], -1)], 1)


def _untangle(straight, doflocs, worst: float = 0.1, rounds: int = 6):
    """Keep curved elements valid. Bending an element's edges onto a curved
    surface can fold a thin element inside out (its volume turns negative
    somewhere), which would silently corrupt the stiffness. Wherever the
    local volume scale of a curved element falls below ``worst`` of its
    straight shape, at a quadrature point or a corner, that element's edges
    are made straight again; repeated, since a straightened edge also
    changes its neighbours. Returns the corrected nodes and how many
    elements were straightened."""
    from skfem import MeshTet2
    from skfem.quadrature import get_quadrature_tet

    X = np.hstack([get_quadrature_tet(4)[0], np.array([[0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1.0]])])

    def det(nodes, X):
        mp = MeshTet2(nodes.T.copy(), straight.t.copy())._mapping()
        J = [[mp.J(i, j, X) for j in range(3)] for i in range(3)]
        return (J[0][0] * (J[1][1] * J[2][2] - J[1][2] * J[2][1]) - J[0][1] * (J[1][0] * J[2][2] - J[1][2] * J[2][0])
                + J[0][2] * (J[1][0] * J[2][1] - J[1][1] * J[2][0]))

    P = straight.p.T
    n = len(P)
    flat = det(np.vstack([P, P[straight.edges.T].mean(axis=1)]), X[:, :1])[:, 0]  # straight shape, constant
    fixed = np.zeros(straight.t.shape[1], bool)
    for _ in range(rounds):
        bad = (det(doflocs, X) / flat[:, None]).min(axis=1) < worst
        if not bad.any():
            break
        fixed |= bad
        e = np.unique(straight.t2e[:, bad])
        doflocs = doflocs.copy()
        doflocs[n + e] = P[straight.edges[:, e].T].mean(axis=1)
    return doflocs, int(fixed.sum())


def _elasticity(basis, lam: float, mu: float, chunk: int = 4000):
    """Stiffness matrix of linear elasticity: the same integral as scikit-fem's
    ``linear_elasticity`` form (sum over quadrature points of
    2 mu eps(u):eps(v) + lam div u div v), on the same basis and quadrature,
    computed element by element as one matrix product instead of one pass per
    pair of basis functions (900 for a quadratic tetrahedron). The entries
    agree with scikit-fem's to rounding (1e-16)."""
    import scipy.sparse as sp

    nb, dofs, dx = basis.Nbfun, basis.element_dofs, basis.dx  # dx: (elements, points), weight x volume scale
    ne = dofs.shape[1]
    data = np.empty((ne, nb, nb))
    for s in range(0, ne, chunk):
        sl = slice(s, min(s + chunk, ne))
        g = np.stack([basis.basis[i][0].grad[:, :, sl] for i in range(nb)])  # (nb, 3, 3, e, points)
        eps = 0.5 * (g + g.transpose(0, 2, 1, 3, 4))
        w = dx[sl]  # some quadrature weights are negative: weight one factor only
        A = eps.transpose(3, 0, 1, 2, 4).reshape(eps.shape[3], nb, 9, -1)
        Aw = (A * w[:, None, None, :]).reshape(A.shape[0], nb, -1)
        A = A.reshape(A.shape[0], nb, -1)
        tr = (eps[:, 0, 0] + eps[:, 1, 1] + eps[:, 2, 2]).transpose(1, 0, 2)  # (e, nb, points)
        Ke = 2 * mu * (Aw @ A.transpose(0, 2, 1)) + lam * ((tr * w[:, None, :]) @ tr.transpose(0, 2, 1))
        data[sl] = 0.5 * (Ke + Ke.transpose(0, 2, 1))  # exactly symmetric
    rows = np.broadcast_to(dofs.T[:, :, None], (ne, nb, nb)).ravel()
    cols = np.broadcast_to(dofs.T[:, None, :], (ne, nb, nb)).ravel()
    K = sp.coo_matrix((data.ravel(), (rows, cols)), shape=(basis.N, basis.N)).tocsr()
    K.sum_duplicates()
    return K


class Model:
    """The assembled stiffness matrix of one meshed part."""

    def __init__(self, mesh: Mesh, E: float, nu: float):
        from skfem import Basis, ElementTetP2, ElementVector, MeshTet
        from skfem.models.elasticity import lame_parameters

        straight = MeshTet(mesh.p.T.copy(), mesh.t.T.copy())
        if mesh.curved:
            from skfem import MeshTet2

            # skfem's curved mesh takes the edge middles in its own edge order
            e = np.sort(straight.edges.T, axis=1)
            n = len(mesh.p)
            want = e[:, 0] * n + e[:, 1]
            have = mesh.mid_pairs[:, 0] * n + mesh.mid_pairs[:, 1]
            pos = np.searchsorted(have, want)
            if not np.array_equal(have[np.minimum(pos, len(have) - 1)], want):
                raise MeshError("curved mesh: an element edge has no middle node")
            doflocs = np.vstack([mesh.p, mesh.mid_xyz[pos]])
            doflocs, self.straightened = _untangle(straight, doflocs)
            self.mesh = MeshTet2(doflocs.T.copy(), mesh.t.T.copy())
            order = 4  # curved elements: the integrand is no longer a polynomial of low degree
        else:
            self.mesh = straight
            order = 2  # straight quadratic elements: degree 2 integrates the stiffness exactly
        self.curved = mesh.curved
        if not mesh.curved:
            self.straightened = 0
        self.lam, self.mu = lame_parameters(E, nu)
        self.sbasis = Basis(self.mesh, ElementTetP2(), intorder=order)
        self.basis = Basis(self.mesh, ElementVector(ElementTetP2()), intorder=order)
        self.K = _elasticity(self.basis, self.lam, self.mu)
        # scalar dof -> its three vector dofs (x, y, z)
        n = self.sbasis.N
        self.vec = np.zeros((3, n), np.int64)
        self.vec[:, self.sbasis.nodal_dofs[0]] = self.basis.nodal_dofs
        self.vec[:, self.sbasis.edge_dofs[0]] = self.basis.edge_dofs
        self.loc = self.sbasis.doflocs.T  # (n, 3)
        self._bf = self.mesh.boundary_facets()
        self._free = None

    # --- regions on the surface
    def boundary_triangles(self):
        """Outward-oriented surface triangles (corner node indices)."""
        m = self.mesh
        f = m.facets[:, self._bf].T.copy()
        e = m.f2t[0, self._bf]
        opp = np.array([np.setdiff1d(m.t[:, k], tri)[0] for k, tri in zip(e, f)])
        P = m.p.T
        n = np.cross(P[f[:, 1]] - P[f[:, 0]], P[f[:, 2]] - P[f[:, 0]])
        flip = np.einsum("ij,ij->i", n, P[opp] - P[f[:, 0]]) > 0
        f[flip] = f[flip][:, [0, 2, 1]]
        return f

    def facets_where(self, pred) -> np.ndarray:
        """Boundary facets whose three corners all satisfy ``pred(points)``."""
        P = self.mesh.p.T
        corners = self.mesh.facets[:, self._bf]
        ok = np.ones(len(P), bool)
        ok[:] = pred(P)
        return self._bf[ok[corners].all(axis=0)]

    def region(self, facets: np.ndarray, centre) -> "Region":
        from skfem import FacetBasis, LinearForm, asm
        from skfem.element import ElementTetP2

        fb = FacetBasis(self.mesh, ElementTetP2(), facets=facets)

        @LinearForm
        def one(v, w):
            return v

        w = asm(one, fb)
        dofs = np.flatnonzero(np.abs(w) > 0)
        allq = np.unique(fb.get_dofs(facets=facets).flatten()) if len(facets) else np.zeros(0, int)
        area = float(w.sum())
        return Region(self, dofs, w[dofs], np.asarray(centre, float), allq, area, facets)

    # --- solving
    def clamp(self, region: "Region"):
        """Hold ``region`` still and prepare the solver: conjugate gradients,
        preconditioned by a two-level cycle over the same part (see
        ``_Preconditioner``)."""
        fixed = self.vec[:, region.all].ravel()
        free = np.setdiff1d(np.arange(self.K.shape[0]), fixed)
        self._Kf = self.K[free][:, free].tocsr()
        self._free = free
        self._pre = _Preconditioner(self, self._Kf, free)
        self.iterations = []

    def solve(self, f: np.ndarray, rtol: float = 1e-10, maxiter: int = 1500) -> np.ndarray:
        """Displacement for nodal forces ``f``, to a relative residual of
        ``rtol``. The preconditioner only decides how fast the iteration gets
        there; the answer is the same to about ten digits whichever is used."""
        import scipy.sparse.linalg as sla

        res = []
        M = sla.LinearOperator(self._Kf.shape, matvec=self._pre, dtype=float)
        x, info = sla.cg(self._Kf, f[self._free], rtol=rtol, atol=0.0, maxiter=maxiter, M=M,
                         callback=lambda xk: res.append(0))
        if info != 0:
            raise RuntimeError(f"the solver did not converge ({len(res)} iterations)")
        self.iterations.append(len(res))
        u = np.zeros(self.K.shape[0])
        u[self._free] = x
        return u

    def von_mises(self, u: np.ndarray) -> np.ndarray:
        """Von Mises stress (MPa) at every mesh corner for displacement u. The
        quadratic element's stress is linear within each tetrahedron; it is
        read at the element's four corners and averaged over the elements
        meeting at each corner (the usual smoothed stress of an FE plot)."""
        from skfem import Basis

        X = np.array([[0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], float)  # the reference corners
        cb = Basis(self.mesh, self.basis.elem, quadrature=(X, np.full(4, 1 / 24)))
        g = cb.interpolate(u).grad  # (3, 3, elements, 4 corners)
        eps = 0.5 * (g + g.transpose(1, 0, 2, 3))
        tr = eps[0, 0] + eps[1, 1] + eps[2, 2]
        sig = 2 * self.mu * eps
        for i in range(3):
            sig[i, i] += self.lam * tr
        vm = np.sqrt(0.5 * ((sig[0, 0] - sig[1, 1]) ** 2 + (sig[1, 1] - sig[2, 2]) ** 2 + (sig[2, 2] - sig[0, 0]) ** 2)
                     + 3 * (sig[0, 1] ** 2 + sig[1, 2] ** 2 + sig[2, 0] ** 2))  # (elements, 4)
        t = self.mesh.t  # (4, elements): corner k of element e is mesh node t[k, e]
        n = self.mesh.p.shape[1]
        tot = np.bincount(t.ravel(), weights=vm.T.ravel(), minlength=n)
        cnt = np.bincount(t.ravel(), minlength=n)
        return tot / np.maximum(cnt, 1)

    def nodal(self, u: np.ndarray) -> np.ndarray:
        """Displacement (n_vertices, 3) at the mesh corners."""
        nd = self.basis.nodal_dofs
        return np.stack([u[nd[0]], u[nd[1]], u[nd[2]]], -1)


# The preconditioner. Quadratic elements put a node on every edge as well as
# at every corner; the corner nodes alone form the same part in straight
# (linear) elements, a system about eight times smaller that carries all of
# the part's large-scale bending and twisting. Each cycle smooths the local
# error on the full system and corrects the large-scale error exactly on the
# small one. Poorly shaped elements (slivers the mesher leaves at tiny
# features) defeat point-by-point smoothing, so their unknowns are solved
# exactly as one block. All three steps are fixed linear operations with
# fixed orderings: the same part always iterates the same way.
SLIVER_QUALITY = 0.1  # element shape (1 = regular tetrahedron) below which its unknowns form the exact block
SMOOTHING_SWEEPS = 2  # Gauss-Seidel sweeps before and after the coarse correction
COARSE_DIRECT_MAX = 150_000  # largest corner-node system factorised; above it, a multigrid cycle


def _quality(P: np.ndarray, T: np.ndarray) -> np.ndarray:
    """Shape of each tetrahedron: volume against its mean edge length, 1 for a
    regular one, near 0 for a sliver."""
    a, b, c, d = (P[T[:, k]] for k in range(4))
    vol = np.abs(np.einsum("ij,ij->i", np.cross(b - a, c - a), d - a)) / 6
    L2 = np.stack([(e ** 2).sum(1) for e in (b - a, c - a, d - a, c - b, d - b, d - c)], 1)
    return vol / np.sqrt(L2.mean(1)) ** 3 * 6 * np.sqrt(2)


class _Preconditioner:
    def __init__(self, model: "Model", Kf, free: np.ndarray):
        import scipy.sparse as sp
        import scipy.sparse.linalg as sla

        self.A = Kf
        N = model.K.shape[0]
        inv = -np.ones(N, np.int64)
        inv[free] = np.arange(len(free))
        # corner-node (straight element) space: every edge node takes the mean of its two corners
        nd, ed = model.sbasis.nodal_dofs[0], model.sbasis.edge_dofs[0]
        nv, E = len(nd), model.mesh.edges
        rows, cols, vals = [], [], []
        for c in range(3):
            rows += [model.vec[c, nd], model.vec[c, ed], model.vec[c, ed]]
            cols += [3 * np.arange(nv) + c, 3 * E[0] + c, 3 * E[1] + c]
            vals += [np.ones(nv), np.full(len(ed), 0.5), np.full(len(ed), 0.5)]
        P = sp.csr_matrix((np.concatenate(vals), (np.concatenate(rows), np.concatenate(cols))), shape=(N, 3 * nv))
        corner_free = np.zeros(3 * nv, bool)
        corner_free[3 * np.arange(nv)[:, None] + np.arange(3)] = inv[model.vec[:, nd].T] >= 0
        P = P[free][:, np.flatnonzero(corner_free)].tocsr()
        self.P, self.R = P, P.T.tocsr()
        Ac = (self.R @ Kf @ P).tocsr()
        if Ac.shape[0] <= COARSE_DIRECT_MAX:
            self.coarse = sla.splu(Ac.tocsc(), permc_spec="MMD_AT_PLUS_A", options={"SymmetricMode": True}).solve
        else:
            self.coarse = _amg_cycle(Ac, model.mesh.p.T[:nv], corner_free)
        # slivers: their unknowns, solved exactly
        T = model.mesh.t[:4].T
        bad = _quality(model.mesh.p.T, T) < SLIVER_QUALITY
        bd = inv[model.basis.element_dofs[:, bad].ravel()]
        self.bd = np.unique(bd[bd >= 0])
        self.block = (sla.splu(Kf[self.bd][:, self.bd].tocsc(), permc_spec="MMD_AT_PLUS_A",
                               options={"SymmetricMode": True}).solve if len(self.bd) else None)
        self.slivers = int(bad.sum())

    def _fix(self, x, r):
        if self.block is not None:
            x[self.bd] += self.block((r - self.A @ x)[self.bd])

    def __call__(self, r: np.ndarray) -> np.ndarray:
        from pyamg.relaxation.relaxation import gauss_seidel

        r = np.ascontiguousarray(r, dtype=float).ravel()
        x = np.zeros_like(r)
        # symmetric: block, smooth forward, coarse, smooth backward, block
        self._fix(x, r)
        gauss_seidel(self.A, x, r, iterations=SMOOTHING_SWEEPS, sweep="forward")
        x += self.P @ self.coarse(self.R @ (r - self.A @ x))
        gauss_seidel(self.A, x, r, iterations=SMOOTHING_SWEEPS, sweep="backward")
        self._fix(x, r)
        return x


def _amg_cycle(Ac, X: np.ndarray, corner_free: np.ndarray):
    """One smoothed-aggregation multigrid cycle on the corner-node system, built
    on the part's six rigid-body motions; for parts too large to factorise."""
    import pyamg

    X = X - X.mean(axis=0)
    B = np.zeros((3 * len(X), 6))
    for c in range(3):
        B[c::3, c] = 1.0
    B[1::3, 3], B[2::3, 3] = -X[:, 2], X[:, 1]
    B[0::3, 4], B[2::3, 4] = X[:, 2], -X[:, 0]
    B[0::3, 5], B[1::3, 5] = -X[:, 1], X[:, 0]
    # pyamg estimates eigenvalues from a random start vector: fix the seed (and
    # leave the caller's random state as it was), or two runs differ in the 12th digit
    state = np.random.get_state()
    np.random.seed(0)
    try:
        ml = pyamg.smoothed_aggregation_solver(Ac, B=B[corner_free], symmetry="hermitian", max_coarse=500)
    finally:
        np.random.set_state(state)
    return lambda r: ml.solve(r, x0=np.zeros_like(r), maxiter=1, cycle="V", tol=1e-300)


@dataclass
class Region:
    model: Model
    dofs: np.ndarray  # scalar dofs carrying weight
    w: np.ndarray  # their weights (area shares, mm2)
    centre: np.ndarray
    all: np.ndarray  # every scalar dof on the region (for clamping)
    area: float
    facets: np.ndarray

    def _G(self):
        r = self.model.loc[self.dofs] - self.centre
        G = np.zeros((len(r), 3, 6))
        G[:, :, :3] = np.eye(3)
        G[:, :, 3:] = -_skew(r)
        return G

    def _A(self, G):
        return np.einsum("i,iak,ial->kl", self.w, G, G)

    def load(self, q6) -> np.ndarray:
        """Nodal force vector for resultant (Fx, Fy, Fz, Mx, My, Mz) about the
        centre, spread as through a rigid washer."""
        G = self._G()
        a = np.linalg.solve(self._A(G), np.asarray(q6, float))
        f_i = self.w[:, None] * np.einsum("iak,k->ia", G, a)
        f = np.zeros(self.model.K.shape[0])
        for c in range(3):
            f[self.model.vec[c, self.dofs]] = f_i[:, c]
        return f

    def motion(self, u: np.ndarray) -> np.ndarray:
        """Best-fit rigid motion (ux, uy, uz, rx, ry, rz) of the region."""
        G = self._G()
        ui = np.stack([u[self.model.vec[c, self.dofs]] for c in range(3)], -1)
        rhs = np.einsum("i,iak,ia->k", self.w, G, ui)
        return np.linalg.solve(self._A(G), rhs)
