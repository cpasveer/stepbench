"""Downloads of the geometry on the page: the fixed parts of a task, and the
part of each attempt, as STEP, STL and 3MF.

STEP is always the exact file: the fixed part's STEP from the task folder,
or the attempt's archived file that the grader graded (byte for byte). STL
and 3MF are meshed from that STEP by OpenCASCADE, in millimetres, with the
tolerances below, so a person can load the task into any CAD or slicer and
model against it.

Names inside a task (the same in the live server and the static export):

    fixed_<name>.step | .stl | .3mf     a fixed part
    obstacles.step | .stl | .3mf        the keep-out boxes (level 3)
    attempt-<n>.step | .stl | .3mf      the part of attempt n
"""
from __future__ import annotations

import os
import re
import shutil
import tempfile

from . import data

FORMATS = ("step", "stl", "3mf")
LINEAR_TOLERANCE = 0.01  # mm: largest distance between a mesh triangle and the true surface
ANGULAR_TOLERANCE = 0.1  # rad (about 5.7 degrees) between neighbouring triangles on a curve
CONTENT_TYPES = {"step": "model/step", "stl": "model/stl", "3mf": "model/3mf"}
NAME_RE = re.compile(r"^(fixed_[A-Za-z0-9]+|obstacles|attempt-(\d+))\.(step|stl|3mf)$")


def tolerance_note() -> str:
    return (f"STL and 3MF meshed from the STEP in millimetres, linear tolerance {LINEAR_TOLERANCE} mm, "
            f"angular tolerance {ANGULAR_TOLERANCE} rad")


def available(task_dir: str, attempts: list[dict]) -> dict:
    """What can be downloaded for a task: {"fixed": [names], "attempts": {n: True}}."""
    fixed = [f[len("fixed_"):-len(".step")] for f in data.task_files(task_dir)
             if f.startswith("fixed_") and f.lower().endswith(".step")]
    return {"fixed": fixed, "obstacles": "obstacles.step" in data.task_files(task_dir),
            "formats": list(FORMATS), "note": tolerance_note(),
            "attempts": [a["n"] for a in attempts if data.find_step(task_dir, a)]}


def source(task_dir: str, name: str) -> tuple[str, str] | None:
    """(STEP path, format) for a download name, or None."""
    m = NAME_RE.match(name)
    if not m:
        return None
    base, n, fmt = m.group(1), m.group(2), m.group(3)
    if n is None:
        step = os.path.join(task_dir, f"{base}.step")
        if f"{base}.step" not in data.task_files(task_dir):
            return None
    else:
        att = next((a for a in data.read_attempts(task_dir) if a.get("n") == int(n)), None)
        step = att and data.find_step(task_dir, att)
        if not step:
            return None
    return step, fmt


def convert(step: str, fmt: str, out: str) -> None:
    """Write `out` in format `fmt` from the STEP file (STEP is copied as is)."""
    if fmt == "step":
        shutil.copyfile(step, out)
        return
    import build123d as bd

    from ..geometry import read_step

    shape = read_step(step)
    if fmt == "stl":
        ok = bd.export_stl(shape, out, tolerance=LINEAR_TOLERANCE, angular_tolerance=ANGULAR_TOLERANCE)
        if not ok:
            raise RuntimeError("STL export failed")
    elif fmt == "3mf":
        m = bd.Mesher(unit=bd.Unit.MM)
        m.add_shape(shape, linear_deflection=LINEAR_TOLERANCE, angular_deflection=ANGULAR_TOLERANCE)
        m.write(out)
    else:
        raise ValueError(fmt)


def get(task_dir: str, name: str, cache_dir: str) -> str | None:
    """Path of the file for a download name, made once and cached by the
    source file's content. None if there is no such download."""
    from .mesh import file_sha256

    src = source(task_dir, name)
    if src is None:
        return None
    step, fmt = src
    if fmt == "step":
        return step
    os.makedirs(cache_dir, exist_ok=True)
    path = os.path.join(cache_dir, f"{file_sha256(step)}.{fmt}")
    if not os.path.exists(path):
        fd, tmp = tempfile.mkstemp(suffix=f".{fmt}", dir=cache_dir)
        os.close(fd)
        try:
            convert(step, fmt, tmp)
            os.replace(tmp, path)
        finally:
            if os.path.exists(tmp):
                os.remove(tmp)
    return path


def file_name(task_id: str, run: str | None, name: str) -> str:
    """The name a downloaded file gets: L1-s0_fixed_A.stl, opus_L1-s300_attempt-2.step."""
    return f"{run + '_' if run else ''}{task_id}_{name}"
