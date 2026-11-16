"""The tool layer: let an agent answer a task in a CAD tool of its choice.

The grader reads only ``output.step``, so any tool can be graded. What a tool
needs is a way *to* that file: a starting file in the tool's own language, a
one-command build into ``output.step`` (with any conversion), and a short
note for the brief. This package provides exactly that, per tool, and
nothing about grading changes.

    python -m jointbench.tools check                         # which tools work here
    python -m jointbench.tools prepare <task> --tool freecad # starter + TOOL.md
    python -m jointbench.tools build <task> --tool freecad   # -> <task>/output.step

Routes (see docs/tools.md for the evidence):

* build123d -- Python, OpenCASCADE; writes STEP directly.
* freecad   -- a FreeCAD Python script run headless by ``freecadcmd``;
  writes STEP directly.
* openscad  -- an OpenSCAD ``.scad`` file. OpenSCAD itself writes only meshes
  for 3D, so the build exports CSG (the flattened tree of primitives) with
  OpenSCAD and rebuilds it as exact solids with FreeCAD's OpenSCAD importer.
  Needs both programs.
* solidcraft -- a SolidCraft script (a JSON list of its commands) run
  headless by ``solidcraft-cli run``; SolidCraft writes STEP directly. Its
  MCP server is the other way to drive it; see docs/tools.md.
"""
from __future__ import annotations

import json
import os
import shutil
import subprocess
import sys
from dataclasses import dataclass
from typing import Callable

HERE = os.path.dirname(os.path.abspath(__file__))
CSG2STEP = os.path.join(HERE, "_csg2step.py")
FCRUN = os.path.join(HERE, "_fcrun.py")
NOTE_FILE = "TOOL.md"


class ToolMissing(RuntimeError):
    pass


class BuildFailed(RuntimeError):
    pass


# ---------------------------------------------------------------- programs

def _find(env: str, names: tuple[str, ...], paths: tuple[str, ...]) -> str | None:
    v = os.environ.get(env)
    if v and os.path.exists(v):
        return v
    for n in names:
        p = shutil.which(n)
        if p:
            return p
    for p in paths:
        if os.path.exists(p):
            return p
    return None


def freecadcmd() -> str | None:
    return _find("JOINTBENCH_FREECADCMD", ("freecadcmd", "FreeCADCmd"),
                 ("/Applications/FreeCAD.app/Contents/Resources/bin/freecadcmd",
                  "/Applications/FreeCAD.app/Contents/MacOS/FreeCADCmd",
                  "/usr/bin/freecadcmd", "/usr/local/bin/freecadcmd"))


def openscad() -> str | None:
    return _find("JOINTBENCH_OPENSCAD", ("openscad",),
                 ("/Applications/OpenSCAD.app/Contents/MacOS/OpenSCAD",
                  "/Applications/OpenSCAD-2021.01.app/Contents/MacOS/OpenSCAD",
                  "/usr/bin/openscad", "/usr/local/bin/openscad"))


def solidcraft() -> str | None:
    return _find("JOINTBENCH_SOLIDCRAFT", ("solidcraft-cli",),
                 ("/Applications/SolidCraft.app/Contents/MacOS/solidcraft-cli",
                  os.path.expanduser("~/Applications/SolidCraft.app/Contents/MacOS/solidcraft-cli"),
                  "/opt/homebrew/bin/solidcraft-cli", "/usr/local/bin/solidcraft-cli"))


def _need(prog: str | None, what: str, install: str) -> str:
    if not prog:
        raise ToolMissing(f"{what} not found. {install}")
    return prog


FREECAD_INSTALL = ("Install FreeCAD 1.x (https://www.freecad.org; on macOS `brew install --cask freecad`) "
                   "or set JOINTBENCH_FREECADCMD to its freecadcmd.")
SOLIDCRAFT_INSTALL = ("Install SolidCraft (on macOS `brew install --cask solidcraft`; "
                      "https://github.com/tangency-labs/solidcraft/releases) or set JOINTBENCH_SOLIDCRAFT "
                      "to its solidcraft-cli.")
OPENSCAD_INSTALL = ("Install OpenSCAD (on macOS `brew install --cask openscad@snapshot`; "
                    "https://openscad.org/downloads.html) or set JOINTBENCH_OPENSCAD.")


def _run(cmd: list[str], cwd: str, env: dict | None = None, timeout: int = 600) -> subprocess.CompletedProcess:
    e = dict(os.environ)
    e.update(env or {})
    return subprocess.run(cmd, cwd=cwd, env=e, capture_output=True, text=True, timeout=timeout)


def _tail(r: subprocess.CompletedProcess, n: int = 3000) -> str:
    return ((r.stdout or "") + (r.stderr or ""))[-n:]


# ---------------------------------------------------------------- starters

BUILD123D_STARTER = None  # build123d uses the task folder's own submission.py

FREECAD_STARTER = '''"""Your answer to this task, as a FreeCAD script.

Write ONE solid to output.step (next to this file) that bolts to every fixed
part. README.md is the brief; every number you need is in task.json.

    BUILD_CMD      # runs this file in FreeCAD, writes output.step
    GRADE_CMD      # grades it

FreeCAD's Part API: Part.makeBox(l, w, h, base, dir), Part.makeCylinder(r, h,
base, dir), a.fuse(b), a.cut(b), shape.transformShape(matrix),
App.Placement / App.Rotation / App.Vector.
"""
import json
import os

import FreeCAD as App
import Part

HERE = os.environ.get("JB_TASK") or os.path.dirname(os.path.abspath(__file__))
TASK = json.load(open(os.path.join(HERE, "task.json")))
POSE = TASK["poses"][0]["resolved"]
FACES = POSE["mounting_faces"]  # [{"part", "origin", "normal"}]
BOLTS = POSE["bolts"]  # [{"id", "size", "position", "axis", "clearance_diameter_range", ...}]
V = App.Vector


def build():
    # Placeholder: a 10 mm cube. It will fail. Replace it with your connector.
    return Part.makeBox(10, 10, 10)


shape = build()
if len(shape.Solids) == 1:
    shape = shape.Solids[0].removeSplitter()
shape.exportStep(os.path.join(HERE, "output.step"))
print("wrote", os.path.join(HERE, "output.step"))
'''

OPENSCAD_STARTER = '''// Your answer to this task, in OpenSCAD.
//
// Model ONE solid that bolts to every fixed part. README.md is the brief;
// every number is in task.json, and the same numbers are in task.scad
// (included below) as OpenSCAD variables, in world coordinates.
//
//   BUILD_CMD      // OpenSCAD -> CSG -> FreeCAD -> output.step
//   GRADE_CMD      // grades it
//
// Round holes must stay round: leave $fn unset, or set it to 0 or to 32 or
// more. $fn from 3 to 16 makes polygons, which the grader does not accept
// as bolt holes.

include <task.scad>

// Placeholder: a 10 mm cube. It will fail. Replace it with your connector.
cube(10);
'''


# A SolidCraft script is a JSON list of the commands its user interface runs.
# JSON has no comments, so the starter says only what it must; TOOL.md says the rest.
SOLIDCRAFT_STARTER = '''[
  {"command": "solid.box",
   "params": {"length": 10, "width": 10, "height": 10, "corner": [0, 0, 0], "body_name": "part"}}
]
'''


def _scad_value(v) -> str:
    if isinstance(v, str):
        return json.dumps(v)
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, (int, float)):
        return repr(float(v)) if isinstance(v, float) else str(v)
    if isinstance(v, (list, tuple)):
        return "[" + ", ".join(_scad_value(x) for x in v) + "]"
    raise TypeError(type(v))


def task_scad(task: dict) -> str:
    """task.json's resolved world geometry as OpenSCAD variables, full precision."""
    pose = task["poses"][0]["resolved"]
    lines = [
        "// Generated from task.json (pose 0). World coordinates, millimetres.",
        "// Do not edit; your model goes in submission.scad.",
        "//",
        "// FACES[i] = [part, origin, outward normal]",
        "// BOLTS[i] = [id, size, position, axis, [clearance dia min, max],",
        "//             [grip min, max], head diameter, head height]",
        "//   position: where the bolt axis meets the mounting face",
        "//   axis: unit vector from the fixed part into your part",
        "",
        "FACES = [",
    ]
    lines += [f"  {_scad_value([f['part'], f['origin'], f['normal']])}," for f in pose["mounting_faces"]]
    lines += ["];", "", "BOLTS = ["]
    for b in pose["bolts"]:
        row = [b["id"], b["size"], b["position"], b["axis"], b["clearance_diameter_range"],
               b["grip_range"], b["head_diameter"], b["head_height"]]
        lines.append(f"  {_scad_value(row)},")
    lines += ["];", ""]
    for i, f in enumerate(pose["mounting_faces"]):
        lines.append(f"FACE_{f['part']}_ORIGIN = {_scad_value(f['origin'])};")
        lines.append(f"FACE_{f['part']}_NORMAL = {_scad_value(f['normal'])};")
    obstacles = pose.get("obstacles") or []
    if obstacles:
        lines += ["", "// OBSTACLES[i] = [name, centre, size along axes, [axis1, axis2, axis3]]", "OBSTACLES = ["]
        for o in obstacles:
            lines.append(f"  {_scad_value([o.get('name', ''), o['center'], o['size'], o['axes']])},")
        lines.append("];")
    return "\n".join(lines) + "\n"


# ---------------------------------------------------------------- notes

COMMON_NOTE = """# Tool: {title}

You answer this task in **{title}**. The brief is `README.md`; the numbers
are in `task.json`. {readme_hint}

Only `output.step` is graded. The grader measures it exactly; it does not
read your code.

## Build and grade

From this folder:

```
{build}
{grade}
```

The first command turns your `{starter}` into `output.step`{conversion}. The
second grades it and prints each check with the number it turned on; fix
what it names and repeat. Each grading is logged and counts as a try.

## What the grader needs from the file

- exactly one closed solid;
- every bolt hole a true cylinder (not a polygon or a mesh), on the bolt
  axis within the stated tolerance;
- every seat a true flat face lying on the mounting face.

{tips}
"""

NOTES = {
    "build123d": dict(
        title="build123d",
        starter="submission.py",
        conversion="",
        readme_hint="Your starting file is `submission.py`, a build123d script.",
        tips="""## Tips for build123d

- `Box`, `Cylinder`, `Pos(...) * shape`, `Location`, `+` (union),
  `-` (cut), `export_step(part, path)`.
- Union pieces that overlap a little rather than ones that only touch face
  to face; the kernel's union of touching solids is fragile.""",
    ),
    "freecad": dict(
        title="FreeCAD",
        starter="submission_freecad.py",
        conversion=", running it headless in FreeCAD (`freecadcmd`)",
        readme_hint=("Your starting file is `submission_freecad.py`, a FreeCAD Python "
                     "script. Where `README.md` mentions `submission.py` (a build123d "
                     "file), read `submission_freecad.py` instead."),
        tips="""## Tips for FreeCAD

- Use the Part module: `Part.makeBox(l, w, h, App.Vector(base), App.Vector(dir))`,
  `Part.makeCylinder(r, h, base, dir)` (a true cylinder along `dir`),
  `a.fuse(b)`, `a.cut(b)`, `shape.removeSplitter()`. Shapes are values;
  no document or GUI is needed.
- To place a box in a tilted face's frame, build it at the origin and apply
  an `App.Placement(origin, App.Rotation(u, v, n, "ZXY"))` or a `App.Matrix`
  with `shape.transformShape(m)` / `shape.Placement = ...`.
- `freecadcmd` passes no arguments to the script; the starter finds the task
  folder from its own location (or `JB_TASK`).
- Union pieces that overlap a little rather than ones that only touch face
  to face.""",
    ),
    "openscad": dict(
        title="OpenSCAD",
        starter="submission.scad",
        conversion=(": OpenSCAD exports the CSG tree (`build/part.csg`), and FreeCAD's "
                    "OpenSCAD importer rebuilds it as exact solids"),
        readme_hint=("Your starting file is `submission.scad`. `task.scad` holds the "
                     "task's numbers as OpenSCAD variables (`FACES`, `BOLTS`, ...), included "
                     "by the starter. Where `README.md` mentions `submission.py` (a "
                     "build123d file), read `submission.scad` instead."),
        tips="""## Tips for OpenSCAD

OpenSCAD itself only writes meshes, and a mesh has no true cylinders, so
the build does not use OpenSCAD's STL or 3MF export. It exports the CSG tree
and FreeCAD rebuilds each primitive exactly. That sets the rules:

- **Round stays round only if `$fn` is unset, 0, or above 16.** With `$fn`
  from 3 to 16 a cylinder becomes a polygonal prism and the hole check
  fails. `$fa` and `$fs` do not matter.
- Use `cube`, `cylinder`, `sphere`, `translate`, `rotate`, `multmatrix`,
  `mirror`, `union`, `difference`, `intersection`, and `linear_extrude` /
  `rotate_extrude` of `square`, `circle`, `polygon`. These become exact
  solids.
- `polyhedron` works and gives flat faces; good for a bridge between two
  plates. Do not cut bolt holes or seats out of a polyhedron's own faces
  by approximation; cut them with `cylinder` and `cube`.
- Avoid `hull()`, `minkowski()`, `offset()` with rounding, `import()`,
  `resize()` and `surface()`: FreeCAD hands these back to OpenSCAD and gets a
  mesh, so their faces are triangles. Build a hull of boxes yourself as a
  `polyhedron` instead.
- `multmatrix(m)` takes a 4x4 (or 3x4) matrix whose first three columns are
  the new x, y, z axes and the fourth the origin, which is the easiest way
  to place something in a tilted face's frame.""",
    ),
    "solidcraft": dict(
        title="SolidCraft",
        starter="submission_solidcraft.json",
        conversion=", running it headless in SolidCraft (`solidcraft-cli run`)",
        readme_hint=("Your starting file is `submission_solidcraft.json`, a SolidCraft script. "
                     "Where `README.md` mentions `submission.py` (a build123d file), read "
                     "`submission_solidcraft.json` instead."),
        tips="""## Tips for SolidCraft

A SolidCraft script is a JSON list of the same commands its user interface
runs, in order: `[{"command": "<id>", "params": {...}}, ...]`. The file that
is built and archived is this JSON. How you work out the numbers in it is
up to you.

SolidCraft's command-line program is

```
SOLIDCRAFT_CLI
```

- `SOLIDCRAFT_CLI commands` prints every command with its parameters, as
  JSON (long; search it for the ids you need).
- `SOLIDCRAFT_CLI eval submission_solidcraft.json` builds without exporting
  and prints each body's faces by type, volume, bounding box and each
  step's warnings.
- Useful commands: `solid.box` (`corner` or `center`), `solid.cylinder`
  (`base` = centre of the bottom, `axis` = any direction, a true cylinder),
  `solid.move` (`translate`, `axis` + `angle`, e.g. `"30 deg"`),
  `solid.combine` (join / cut / intersect bodies), `solid.extrude` of a
  sketch (`sketch.create` on any plane given by origin, x_dir, y_dir).
- Name the first body with `"body_name"`; later solids join to it or cut
  from it with `"operation": "join"` or `"cut"` and `"targets": ["<name>"]`.
  The export writes every body, so end with exactly one.
- **Holes must be square to the flat faces they pass through.** SolidCraft
  writes a hole that meets a flat face at a slant to the STEP file as a
  free-form surface, not a cylinder, and the hole check rejects it. A bolt
  hole square to its plate is written as a true cylinder, also when the
  plate is tilted.
- If a step fails, the run stops, names the step and writes nothing.
- Union pieces that overlap a little rather than ones that only touch face
  to face.""",
    ),
}


def _with_cmds(text: str, build: str, grade: str) -> str:
    return text.replace("BUILD_CMD", build).replace("GRADE_CMD", grade)


# ---------------------------------------------------------------- builders

def _remove_output(task: str) -> str:
    out = os.path.join(task, "output.step")
    if os.path.exists(out):
        os.remove(out)  # never grade a stale file after a failed build
    return out


def build_build123d(task: str) -> str:
    out = _remove_output(task)
    r = _run([sys.executable, "submission.py"], cwd=task)
    if r.returncode != 0 or not os.path.exists(out):
        if os.path.exists(out):
            os.remove(out)
        raise BuildFailed(f"submission.py failed (exit {r.returncode}):\n{_tail(r)}")
    return out


def build_freecad(task: str) -> str:
    fc = _need(freecadcmd(), "FreeCAD (freecadcmd)", FREECAD_INSTALL)
    out = _remove_output(task)
    script = os.path.abspath(os.path.join(task, "submission_freecad.py"))
    r = _run([fc, FCRUN], cwd=task, env={"JB_TASK": os.path.abspath(task), "JB_SCRIPT": script})
    # freecadcmd exits 0 even when the script raised; the runner's marker is the proof
    if r.returncode != 0 or not os.path.exists(out) or "JB_DONE" not in r.stdout:
        if os.path.exists(out):
            os.remove(out)  # the script failed after writing; do not leave a file to grade
        raise BuildFailed(f"submission_freecad.py failed in FreeCAD:\n{_clean_fc(_tail(r))}")
    return out


def build_openscad(task: str) -> str:
    osc = _need(openscad(), "OpenSCAD", OPENSCAD_INSTALL)
    fc = _need(freecadcmd(), "FreeCAD (freecadcmd), needed to convert OpenSCAD's CSG", FREECAD_INSTALL)
    out = _remove_output(task)
    bdir = os.path.join(task, "build")
    os.makedirs(bdir, exist_ok=True)
    csg = os.path.join(bdir, "part.csg")
    if os.path.exists(csg):
        os.remove(csg)
    r = _run([osc, "-o", csg, "submission.scad"], cwd=task)
    if r.returncode != 0 or not os.path.exists(csg):
        raise BuildFailed(f"OpenSCAD failed (exit {r.returncode}):\n{_tail(r)}")
    r = _run([fc, CSG2STEP], cwd=bdir, env={"JB_CSG": os.path.abspath(csg), "JB_STEP": os.path.abspath(out)})
    if "JB_OK" not in r.stdout or not os.path.exists(out):
        raise BuildFailed(f"FreeCAD could not convert OpenSCAD's CSG:\n{_clean_fc(_tail(r))}")
    return out


def build_solidcraft(task: str) -> str:
    sc = _need(solidcraft(), "SolidCraft (solidcraft-cli)", SOLIDCRAFT_INSTALL)
    out = _remove_output(task)
    r = _run([sc, "run", "submission_solidcraft.json", "--out", "output.step", "--quiet"], cwd=task)
    if r.returncode != 0 or not os.path.exists(out):
        if os.path.exists(out):
            os.remove(out)
        raise BuildFailed(f"SolidCraft failed (exit {r.returncode}):\n{_tail(r)}")
    return out


def _clean_fc(text: str) -> str:
    """Drop FreeCAD's banner, STEP-writer statistics, colour codes and the CSG
    parser's grammar warnings; FreeCAD echoes to both streams, so each line once."""
    import re
    noise = ("WARNING: Token", "WARNING: There are", "FreeCAD ", "(C) 20", "****", "** WorkSession",
             "Step File Name", "Transfer Mode", "Transferring Shape")
    keep, seen = [], set()
    for ln in text.splitlines():
        ln = re.sub(r"\x1b\[[0-9;]*m", "", ln).rstrip()
        if not ln.strip() or ln.strip().startswith(noise) or "free and open-source software" in ln:
            continue
        if ln in seen:
            continue
        seen.add(ln)
        keep.append(ln)
    return "\n".join(keep)


# ---------------------------------------------------------------- registry

@dataclass(frozen=True)
class Tool:
    name: str
    starter: str  # file name in the task folder
    starter_text: str | None  # None: the task folder's own file is used
    build: Callable[[str], str]
    available: Callable[[], str | None]  # None if usable, else why not

    @property
    def note(self) -> dict:
        return NOTES[self.name]


def _avail_freecad():
    return None if freecadcmd() else "FreeCAD not found. " + FREECAD_INSTALL


def _avail_openscad():
    if not openscad():
        return "OpenSCAD not found. " + OPENSCAD_INSTALL
    if not freecadcmd():
        return "FreeCAD not found (converts OpenSCAD's CSG). " + FREECAD_INSTALL
    return None


TOOLS: dict[str, Tool] = {
    "build123d": Tool("build123d", "submission.py", BUILD123D_STARTER, build_build123d, lambda: None),
    "freecad": Tool("freecad", "submission_freecad.py", FREECAD_STARTER, build_freecad, _avail_freecad),
    "openscad": Tool("openscad", "submission.scad", OPENSCAD_STARTER, build_openscad, _avail_openscad),
    "solidcraft": Tool("solidcraft", "submission_solidcraft.json", SOLIDCRAFT_STARTER, build_solidcraft,
                       lambda: None if solidcraft() else "SolidCraft not found. " + SOLIDCRAFT_INSTALL),
}

NOT_PROVIDED = {
    "anvil": "not provided: Anvil cannot export STEP yet.",
}


def get(name: str) -> Tool:
    if name in TOOLS:
        return TOOLS[name]
    if name in NOT_PROVIDED:
        raise ToolMissing(f"{name}: {NOT_PROVIDED[name]}")
    raise KeyError(f"unknown tool {name!r}; known: {', '.join(TOOLS)}")


# ---------------------------------------------------------------- prepare / build

DEFAULT_RUNNER = "uv run"


def commands(tool: str, runner: str = DEFAULT_RUNNER) -> tuple[str, str]:
    t = get(tool)
    from .. import NAME
    build = f"{runner} python -m {NAME}.tools build . --tool {tool}"
    grade = f"{runner} {NAME} verify . --tool {tool} --script {t.starter}"
    return build, grade


def _is_stock_build123d_stub(path: str) -> bool:
    from .. import NAME
    from ..workspace import SUBMISSION_STUB
    try:
        with open(path) as f:
            return f.read() == SUBMISSION_STUB.replace("CLI_NAME", NAME)
    except OSError:
        return False


def prepare(task: str, tool: str, runner: str = DEFAULT_RUNNER, force: bool = False) -> list[str]:
    """Write the tool's starter, its helper files and TOOL.md into a task folder.

    For a tool other than build123d the untouched build123d stub
    ``submission.py`` is removed, so the folder offers one starting point.
    An edited ``submission.py`` is left alone. Returns the files written.
    """
    t = get(tool)
    if not os.path.exists(os.path.join(task, "task.json")):
        raise FileNotFoundError(f"{task}: no task.json")
    build, grade = commands(tool, runner)
    written = []

    def put(name, text):
        p = os.path.join(task, name)
        if os.path.exists(p) and not force and name not in (NOTE_FILE, "task.scad"):
            return
        with open(p, "w") as f:
            f.write(text)
        written.append(p)

    if t.starter_text is not None:
        put(t.starter, _with_cmds(t.starter_text, build, grade))
        stub = os.path.join(task, "submission.py")
        if _is_stock_build123d_stub(stub):
            os.remove(stub)
    if tool == "openscad":
        with open(os.path.join(task, "task.json")) as f:
            put("task.scad", task_scad(json.load(f)))
    n = t.note
    note = COMMON_NOTE.format(build=build, grade=grade, **n)
    if tool == "solidcraft":
        note = note.replace("SOLIDCRAFT_CLI", solidcraft() or "solidcraft-cli")
    put(NOTE_FILE, note)
    return written


def summary(step: str) -> str:
    """One neutral line about what the STEP holds (same for every tool)."""
    try:
        from build123d import import_step
        shape = import_step(step)
        solids = shape.solids()
        kinds: dict[str, int] = {}
        for f in shape.faces():
            k = str(f.geom_type).split(".")[-1].lower()
            kinds[k] = kinds.get(k, 0) + 1
        faces = ", ".join(f"{k} {v}" for k, v in sorted(kinds.items()))
        return f"output.step: {len(solids)} solid(s), {len(shape.faces())} faces ({faces})"
    except Exception as e:  # the grader will say what is wrong
        return f"output.step written (could not summarise: {e})"


def build(task: str, tool: str) -> str:
    t = get(tool)
    why = t.available()
    if why:
        raise ToolMissing(why)
    if not os.path.exists(os.path.join(task, t.starter)):
        raise FileNotFoundError(f"{task}: no {t.starter}; run `prepare {task} --tool {tool}` first")
    return t.build(task)
