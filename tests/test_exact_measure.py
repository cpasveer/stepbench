"""Areas and volumes stay right on Rhino's one-edge closed loops (see jointbench/exact_measure.py).

tests/data/rhino_single_edge_loop.step is an answer exported by Rhino 9: its flange face is
bounded by one closed B-spline edge. OpenCASCADE's default integration gave that face
-2307 mm2 and the solid 218 000 mm3; a fine triangle mesh gives 2680 mm2 and 87 800 mm3.
"""
import os

import numpy as np

import jointbench  # noqa: F401  (installs the adaptive measurement)
import build123d as bd

STEP = os.path.join(os.path.dirname(__file__), "data", "rhino_single_edge_loop.step")


def test_rhino_face_areas_positive_and_volume_matches_mesh():
    s = bd.import_step(STEP)
    areas = [f.area for f in s.faces()]
    assert min(areas) > 1.0
    assert any(abs(a - 2679.0) < 2.0 for a in areas)
    v, t = s.tessellate(0.005, 0.05)
    P = np.array([[p.X, p.Y, p.Z] for p in v])
    T = np.array(t)
    mesh = abs(np.einsum("ij,ij->i", P[T[:, 0]], np.cross(P[T[:, 1]], P[T[:, 2]])).sum() / 6)
    assert abs(s.volume - mesh) / mesh < 0.002


def test_simple_box_unchanged():
    b = bd.Box(10, 20, 30)
    assert abs(b.volume - 6000) < 1e-6
    assert abs(sum(f.area for f in b.faces()) - 2200) < 1e-6
