"""Exact geometry -> something a browser can draw.

The browser never sees B-rep. Each solid is turned into two things:

* a triangle mesh per face (for shading), tessellated by OpenCASCADE;
* the true model edges as polylines (for the lines a CAD drawing shows).

Drawing the edges from the B-rep, rather than guessing them from triangle
angles, is what makes a hole read as a hole and a chamfer as a chamfer.
Seam edges (the line where a cylinder's surface is closed on itself) are
left out, as every CAD viewer does, because they are not real edges.

Arrays are packed as base64 little-endian Float32 / Uint32 so a mesh is a
plain JSON file that loads without any parser library.
"""
from __future__ import annotations

import base64
import hashlib
import json
import os
from array import array

from OCP.BRep import BRep_Tool
from OCP.BRepAdaptor import BRepAdaptor_Curve
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.GCPnts import GCPnts_TangentialDeflection
from OCP.TopAbs import TopAbs_REVERSED
from OCP.TopLoc import TopLoc_Location

MESH_VERSION = 1
LINEAR_DEFLECTION = 0.04  # mm: how far a triangle may sit from the true surface
ANGULAR_DEFLECTION = 0.25  # rad
THUMB_LINEAR = 0.4         # mm: coarse meshes for the small thumbnails
THUMB_ANGULAR = 0.6        # rad


def _b64(values, typecode: str) -> str:
    a = array(typecode, values)
    if a.itemsize != 4:  # pragma: no cover - every supported platform has 4-byte f and I
        raise RuntimeError("unexpected array item size")
    if os.sys.byteorder != "little":  # pragma: no cover
        a.byteswap()
    return base64.b64encode(a.tobytes()).decode("ascii")


def _wrapped(shape):
    return getattr(shape, "wrapped", shape)


def mesh_shape(shape, linear: float = LINEAR_DEFLECTION, angular: float = ANGULAR_DEFLECTION) -> dict:
    """Mesh one shape (solid, compound, anything with faces)."""
    import build123d as bd

    s = _wrapped(shape)
    bshape = shape if isinstance(shape, bd.Shape) else bd.Shape.cast(s)
    BRepMesh_IncrementalMesh(s, linear, False, angular, True)

    pos: list[float] = []
    idx: list[int] = []
    faces: list[list[int]] = []  # [first index, index count] per B-rep face
    for bface in bshape.faces():
        face = bface.wrapped
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, loc)
        if tri is None:
            continue
        trsf = loc.Transformation()
        base = len(pos) // 3
        for i in range(1, tri.NbNodes() + 1):
            p = tri.Node(i).Transformed(trsf)
            pos.extend((p.X(), p.Y(), p.Z()))
        rev = face.Orientation() == TopAbs_REVERSED
        first = len(idx)
        for i in range(1, tri.NbTriangles() + 1):
            a, b, c = tri.Triangle(i).Get()
            if rev:
                b, c = c, b
            idx.extend((base + a - 1, base + b - 1, base + c - 1))
        faces.append([first, len(idx) - first])

    # seam edges: an edge that a face meets on both sides of itself
    seams = set()
    for face in bshape.faces():
        for e in face.edges():
            if BRep_Tool.IsClosed_s(e.wrapped, face.wrapped):
                seams.add(e)
    seg: list[float] = []
    for e in bshape.edges():
        edge = e.wrapped
        if BRep_Tool.Degenerated_s(edge) or e in seams:
            continue
        try:
            curve = BRepAdaptor_Curve(edge)
            disc = GCPnts_TangentialDeflection(curve, angular / 2, linear)
        except Exception:  # noqa: BLE001 - an edge we cannot sample is just not drawn
            continue
        pts = [disc.Value(k) for k in range(1, disc.NbPoints() + 1)]
        for p, q in zip(pts, pts[1:]):
            seg.extend((p.X(), p.Y(), p.Z(), q.X(), q.Y(), q.Z()))

    if pos:
        xs, ys, zs = pos[0::3], pos[1::3], pos[2::3]
        bbox = [[min(xs), min(ys), min(zs)], [max(xs), max(ys), max(zs)]]
    else:
        bbox = None
    return {
        "v": MESH_VERSION,
        "positions": _b64(pos, "f"),
        "indices": _b64(idx, "I"),
        "faces": faces,
        "edges": _b64(seg, "f"),
        "bbox": [[round(v, 4) for v in c] for c in bbox] if bbox else None,
        "counts": {"vertices": len(pos) // 3, "triangles": len(idx) // 3,
                   "edge_segments": len(seg) // 6, "faces": len(faces)},
    }


def mesh_step(path: str) -> dict:
    """Mesh a STEP file as it is, whatever it contains (several solids,
    a shell, nothing at all). The viewer must be able to show a broken
    answer; it is the grader's job, not the viewer's, to reject it."""
    from ..geometry import read_step

    shape = read_step(path)
    return mesh_shape(shape)


def file_sha256(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


class MeshCache:
    """Meshes of STEP files, stored by the file's sha256.

    The attempts log records each graded file's sha256. Keying the cache by
    content means an attempt stays replayable after its ``output.step`` has
    been overwritten by the next attempt, as long as the mesh was made while
    the file still had that content."""

    def __init__(self, root: str):
        self.root = root
        os.makedirs(root, exist_ok=True)

    def path(self, sha: str) -> str:
        return os.path.join(self.root, f"{sha}.json")

    def has(self, sha: str) -> bool:
        return os.path.exists(self.path(sha))

    def get(self, sha: str) -> bytes | None:
        p = self.path(sha)
        if not os.path.exists(p):
            return None
        with open(p, "rb") as f:
            return f.read()

    def put_step(self, step_path: str, sha: str | None = None) -> str | None:
        """Mesh a STEP file into the cache; return its sha, or None when the
        file is missing or cannot be read at all."""
        if not os.path.exists(step_path):
            return None
        sha = sha or file_sha256(step_path)
        if self.has(sha):
            return sha
        try:
            m = mesh_step(step_path)
        except Exception as e:  # noqa: BLE001 - unreadable files get an empty mesh with a reason
            m = {"v": MESH_VERSION, "error": f"{type(e).__name__}: {e}"}
        # unique temporary name: two requests may mesh the same file at once
        import threading

        tmp = f"{self.path(sha)}.{os.getpid()}.{threading.get_ident()}.tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(m, f, separators=(",", ":"))
        os.replace(tmp, self.path(sha))
        return sha
