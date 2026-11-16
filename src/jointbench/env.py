"""The environment record of a run: what the run was made with, captured
by the machine at the start of the run, not typed in by the participant.

    jointbench env <run folder> --model claude-opus-5-5 --provider anthropic \
        --tool freecad --mcp freecad-mcp=0.4.1 --agent-loop my-harness=1.2

writes ``<run folder>/environment.json``. What can be read from the machine
is read (Python packages, the CAD programs found, their add-ons, the
operating system, the benchmark's own version). What cannot (the model and
its provider, a connector or MCP server running elsewhere, the agent loop)
is passed in by the harness that starts the run, and is marked as such.

Not recorded on purpose: the machine's name and the user's name. They add
nothing to reproducing a run and would publish personal details.
"""
from __future__ import annotations

import datetime as _dt
import json
import os
import platform
import subprocess
import sys

FORMAT = 1
FILE = "environment.json"

# What a STEP file's header says about the program that wrote it, for each
# declared tool. A tool that exports through another program (OpenSCAD's
# CSG converted by FreeCAD) lists that program too.
HEADER_NAMES = {
    "build123d": ("build123d",),
    "cadquery": ("cadquery",),
    "freecad": ("freecad",),
    "openscad": ("openscad", "freecad"),
    "solidcraft": ("solidcraft",),
    "onshape": ("onshape", "parasolid"),
    "solidworks": ("solidworks",),
    "rhino": ("rhino", "rhinoceros"),
    "fusion": ("fusion", "autodesk"),
}


def tool_vs_header(tool: str | None, header: dict | None) -> str | None:
    """Compare the declared tool with the STEP header's writing program:
    "consistent", "inconsistent" (the header names a different known tool),
    or "cannot tell". Recorded next to the attempt; never graded on."""
    if not tool:
        return None
    if not header:
        return "cannot tell"
    text = " ".join(str(header.get(k) or "") for k in ("originating_system", "preprocessor_version",
                                                      "file_name")).lower()
    own = HEADER_NAMES.get(tool.lower(), (tool.lower(),))
    if any(n in text for n in own):
        return "consistent"
    others = {n for t, names in HEADER_NAMES.items() if t != tool.lower() for n in names} - set(own)
    if any(n in text for n in others):
        return "inconsistent"
    return "cannot tell"


def _home(path: str | None) -> str | None:
    """A path with the user's home folder written as ~, so no user name is recorded."""
    if not path:
        return path
    home = os.path.expanduser("~")
    return "~" + path[len(home):] if path.startswith(home + os.sep) else path


def _run(cmd: list[str], timeout: int = 60) -> str | None:
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    except (OSError, subprocess.SubprocessError):
        return None
    return ((r.stdout or "") + (r.stderr or "")).strip() or None


def python_packages() -> dict:
    """Every installed distribution and its version (a pip freeze)."""
    from importlib import metadata

    out = {}
    for d in metadata.distributions():
        name = d.metadata["Name"]
        if name:
            out[name.lower()] = d.version
    return dict(sorted(out.items()))


def benchmark() -> dict:
    """The benchmark's own version, and the exact source if it is a git checkout."""
    from . import NAME, __version__

    out = {"name": NAME, "version": __version__}
    here = os.path.dirname(os.path.abspath(__file__))
    commit = _run(["git", "-C", here, "rev-parse", "HEAD"])
    if commit and len(commit) == 40:
        out["commit"] = commit
        out["modified"] = bool(_run(["git", "-C", here, "status", "--porcelain", "--untracked-files=no"]))
    return out


_FREECAD_PROBE = r'''
import json, os, re, FreeCAD
v = FreeCAD.Version()
mods = {}
u = FreeCAD.getUserAppDataDir().rstrip("/\\")
for root in sorted({os.path.join(u, "Mod"), os.path.join(os.path.dirname(u), "Mod")}):
    if not os.path.isdir(root):
        continue
    for m in sorted(os.listdir(root)):
        if not os.path.isdir(os.path.join(root, m)):
            continue
        p = os.path.join(root, m, "package.xml")
        ver = None
        if os.path.exists(p):
            t = open(p, encoding="utf-8", errors="replace").read()
            g = re.search(r"<version>([^<]+)</version>", t)
            ver = g.group(1).strip() if g else None
        mods[m] = ver
print("JBENV" + json.dumps({"version": ".".join(v[:3]), "build": " ".join(str(x) for x in v[3:5]), "addons": mods}))
'''


def freecad() -> dict | None:
    from .tools import freecadcmd

    exe = freecadcmd()
    if not exe:
        return None
    out = {"path": _home(exe)}
    import tempfile

    with tempfile.TemporaryDirectory() as tmp:
        probe = os.path.join(tmp, "jbenv_probe.py")
        with open(probe, "w", encoding="utf-8") as f:
            f.write(_FREECAD_PROBE)
        text = _run([exe, probe], timeout=120) or ""
    for line in text.splitlines():
        if line.startswith("JBENV"):
            out.update(json.loads(line[5:]))
            break
    else:
        out["probe"] = "FreeCAD did not report its version"
    return out


def openscad() -> dict | None:
    from .tools import openscad as find

    exe = find()
    if not exe:
        return None
    return {"path": _home(exe), "version": _run([exe, "--version"])}


def solidcraft() -> dict | None:
    from .tools import solidcraft as find

    exe = find()
    if not exe:
        return None
    return {"path": _home(exe), "version": _run([exe, "--version"])}


def cad_programs() -> dict:
    """Every CAD program the benchmark knows how to find, with its version.
    Python libraries (build123d, CadQuery, OCP) are in the package list."""
    out = {}
    for name, probe in (("freecad", freecad), ("openscad", openscad), ("solidcraft", solidcraft)):
        found = probe()
        if found:
            out[name] = found
    return out


def machine() -> dict:
    out = {"os": platform.system(), "os_version": platform.release(), "platform": platform.platform(),
           "cpu": platform.machine(), "cpus": os.cpu_count(),
           "python": sys.version.split()[0], "python_implementation": platform.python_implementation()}
    # set by the official box's launcher; absent elsewhere
    image, digest = os.environ.get("JOINTBENCH_BOX_IMAGE"), os.environ.get("JOINTBENCH_BOX_DIGEST")
    out["box"] = {"image": image, "digest": digest} if image or digest else None
    return out


def task_info(task_dir: str) -> dict:
    with open(os.path.join(task_dir, "task.json"), encoding="utf-8") as f:
        t = json.load(f)
    from .freeze import membership

    return {"id": t.get("id"), "seed": t.get("seed"), "level": t.get("level"),
            "generator_version": t.get("generator_version"), "format_version": t.get("format_version"),
            "task_set": membership(task_dir)}


def _pairs(items: list[str] | None) -> dict:
    out = {}
    for it in items or []:
        name, _, ver = it.partition("=")
        out[name.strip()] = ver.strip() or None
    return out


def record(*, model: str | None = None, provider: str | None = None, tool: str | None = None,
           tool_versions: list[str] | None = None, mcp: list[str] | None = None,
           agent_loop: list[str] | None = None, tasks: list[str] | None = None,
           track: str | None = None, probe_programs: bool = True) -> dict:
    return {
        "format": FORMAT,
        "captured": _dt.datetime.now(_dt.timezone.utc).isoformat(timespec="seconds"),
        "given_by_harness": {  # what the machine cannot read; stored as given
            "model": model, "provider": provider, "track": track, "tool": tool,
            "tool_versions": _pairs(tool_versions), "mcp_servers": _pairs(mcp),
            "agent_loop": _pairs(agent_loop),
        },
        "measured": {
            "benchmark": benchmark(),
            "machine": machine(),
            "cad_programs": cad_programs() if probe_programs else None,
            "python_packages": python_packages(),
            "tasks": [task_info(t) for t in tasks or []],
        },
    }


def write(run_dir: str, rec: dict) -> str:
    os.makedirs(run_dir, exist_ok=True)
    p = os.path.join(run_dir, FILE)
    with open(p, "w", encoding="utf-8", newline="\n") as f:
        f.write(json.dumps(rec, indent=2) + "\n")
    return p
