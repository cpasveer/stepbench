"""Frozen task sets: tasks fixed as files, so scores stay comparable when the
generator changes (a new generator version gives the same seed a different
task).

    jointbench freeze sets/trial --name trial --version 1 --level 1:4 --level 2:4 \\
        --level 3:2 --knob 3:parts=2 --knob 3:obstacles=2 --seed 1000
    jointbench freeze --check sets/trial

A set folder holds the task folders and ``manifest.json``: the set's name,
version and visibility, and for every task its id, level, seed, knobs,
generator version and a fingerprint -- the SHA-256 of every file in the
task folder, and one hash over all of them. Each task folder also gets
``set.json`` (name, version, fingerprint), so ``verify``, the attempt
archive and ``traces`` record which set a task belongs to. A set is used as
files; it is never regenerated from its seeds.

Public, hidden and training tasks are kept apart by seed range and by where
the files live: a public set is committed and published; a hidden set uses
the same sizes and knobs with seeds derived from a secret, lives outside the
repository (``sets/hidden*`` is ignored by git) and is never published;
training tasks are generated freshly from any seeds not used by a set.
``freeze`` refuses seeds that overlap a set listed with ``--avoid``.
"""
from __future__ import annotations

import hashlib
import json
import os

from .catalogue import GENERATOR_VERSION
from . import families
from .workspace import write_workspace

MANIFEST = "manifest.json"
MARK = "set.json"


def _sha(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        h.update(f.read())
    return h.hexdigest()


def fingerprint(task_dir: str) -> tuple[str, dict]:
    """Hash of every file in a task folder except the set mark and anything
    written later by attempts (attempts.jsonl, attempts/, errors.jsonl,
    errors/, output files)."""
    skip_dirs = {"attempts", "errors", ".play"}
    skip = {MARK, "attempts.jsonl", "errors.jsonl", "output.step", ".verify.lock"}
    files = {}
    for root, dirs, names in os.walk(task_dir):
        dirs[:] = sorted(d for d in dirs if d not in skip_dirs)
        for n in sorted(names):
            rel = os.path.relpath(os.path.join(root, n), task_dir)
            if n in skip or n.startswith("baseline_"):
                continue
            files[rel] = _sha(os.path.join(root, n))
    whole = hashlib.sha256("\n".join(f"{k} {v}" for k, v in sorted(files.items())).encode()).hexdigest()
    return whole, files


def hidden_seed(secret: str, level: int, i: int) -> int:
    """Seeds for a hidden set: derived from a secret, far from public ranges."""
    h = hashlib.sha256(f"{secret}/{level}/{i}".encode()).hexdigest()
    return 10_000_000 + int(h[:12], 16) % 1_000_000_000


def _used_seeds(avoid: list[str]) -> set:
    used = set()
    for a in avoid or []:
        with open(os.path.join(a, MANIFEST), encoding="utf-8") as f:
            for t in json.load(f)["tasks"]:
                used.add((t["level"], t["seed"]))
    return used


def _make(args):
    family, lv, s, k, out = args
    mod = families.level(family, lv)
    task, d, ref = mod.generate(s, k) if k else mod.generate(s)
    ws = write_workspace(task, ref, out, {"level": lv, "seed": s, "draw": d})
    return lv, s, task.id, ws, task.meta.get("knobs"), task.generator_version


def freeze(out: str, name: str, version: int, counts: dict, knobs: dict, seed: int = 0,
           secret: str | None = None, avoid: list[str] | None = None, progress=None, jobs: int = 1,
           family: str = "connect") -> dict:
    from .jobs import pmap

    if os.path.exists(os.path.join(out, MANIFEST)):
        raise FileExistsError(f"{out} already holds a set; a set is never overwritten")
    used = _used_seeds(avoid)
    items = []
    for lv in sorted(counts):
        k = knobs.get(lv) or {}
        for i in range(counts[lv]):
            s = hidden_seed(secret, lv, i) if secret else seed + i
            if (lv, s) in used:
                raise ValueError(f"level {lv} seed {s} is already used by a set given with --avoid")
            items.append((family, lv, s, k, out))
    os.makedirs(out, exist_ok=True)
    tasks = []
    for lv, s, tid, ws, tknobs, gv in pmap(_make, items, jobs):
        if True:
            fp, files = fingerprint(ws)
            mark = {"set": name, "version": version, "task": tid, "fingerprint": fp}
            with open(os.path.join(ws, MARK), "w", encoding="utf-8", newline="\n") as f:
                f.write(json.dumps(mark, indent=2) + "\n")
            tasks.append({"id": tid, "folder": os.path.basename(ws), "level": lv, "seed": s,
                          "knobs": tknobs, "generator_version": gv, "fingerprint": fp, "files": files})
            if progress:
                progress(tid)
    man = {"set": name, "version": version, "family": family, "visibility": "hidden" if secret else "public",
           "generator_version": GENERATOR_VERSION, "counts": {str(k): v for k, v in counts.items()},
           "knobs": {str(k): v for k, v in knobs.items()}, "tasks": tasks}
    with open(os.path.join(out, MANIFEST), "w", encoding="utf-8", newline="\n") as f:
        f.write(json.dumps(man, indent=2) + "\n")
    return man


def check(set_dir: str) -> list[str]:
    """Problems found (empty when every task folder still matches the manifest)."""
    with open(os.path.join(set_dir, MANIFEST), encoding="utf-8") as f:
        man = json.load(f)
    problems = []
    for t in man["tasks"]:
        d = os.path.join(set_dir, t["folder"])
        if not os.path.isdir(d):
            problems.append(f"{t['id']}: folder missing")
            continue
        fp, files = fingerprint(d)
        if fp != t["fingerprint"]:
            changed = sorted(k for k in set(files) | set(t["files"]) if files.get(k) != t["files"].get(k))
            problems.append(f"{t['id']}: files changed: {', '.join(changed)}")
    return problems


def membership(task_dir: str) -> dict | None:
    """The set a task folder belongs to, from its set.json; None if none."""
    p = os.path.join(task_dir, MARK)
    if not os.path.exists(p):
        return None
    with open(p, encoding="utf-8") as f:
        m = json.load(f)
    return {"set": m.get("set"), "version": m.get("version"), "fingerprint": m.get("fingerprint")}
