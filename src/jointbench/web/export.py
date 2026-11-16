"""`jointbench site --out <dir>` -- the same front end as a static folder.

Writes the front end from ``site/`` and, under ``data/``, exactly the JSON
documents the live server would answer with (see ``data.py``), for a chosen
set of tasks. The result opens from any static web host, or from a local
``python -m http.server``, with no Python package installed. Uploading is
switched off there (``site.json`` says ``mode: static``); the page says how
to run the live version instead.

Only meshes of files that can still be found with the graded content are
included; other attempts keep their verdict and show no shape.
"""
from __future__ import annotations

import json
import os
import shutil

from .. import NAME
from . import data
from .mesh import MeshCache
from .server import site_dir

SKIP = {"screenshots", "README.md", "data"}


def _write(path: str, obj) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(obj, f, separators=(",", ":"))


def export(targets: list[str], out: str, include_reference: bool = True, log=print,
           scores_file: str | None = None, dev_runs: bool = False, precompute: list[str] | None = None) -> int:
    """dev_runs: also put runs graded under earlier rules on the scoreboard
    (marked as development data). Off by default: a hostable copy presents
    only runs under the current rules as results.

    precompute: processes (e.g. fdm, pbf, moulding) to measure for every
    model run's submission that has no field file yet, so the compare page
    can switch process without a live server. Written into the copy only,
    never into the task folders. Slow: roughly 20 s to several minutes per
    part and process (moulding on a level 3 part took over 10 minutes)."""
    keyed, scores_src = data.collect(list(targets), scores_file)
    if not keyed:
        log("no task folders found")
        return 2
    src = site_dir()
    out = os.path.abspath(out)
    os.makedirs(out, exist_ok=True)
    # the front end goes under v/<fingerprint>/ so a redeploy is never mixed
    # with cached old files; index.html names the current fingerprint
    from .assets import ASSET_DIRS, fingerprint, index_html

    fp = fingerprint(src)
    vdir = os.path.join(out, "v")
    if os.path.isdir(vdir):
        for old in os.listdir(vdir):
            if old != fp:
                shutil.rmtree(os.path.join(vdir, old), ignore_errors=True)
    for d in ASSET_DIRS:
        shutil.copytree(os.path.join(src, d), os.path.join(vdir, fp, d), dirs_exist_ok=True)
        shutil.rmtree(os.path.join(out, d), ignore_errors=True)  # unversioned copies from older exports
    with open(os.path.join(out, "index.html"), "w", encoding="utf-8") as f:
        f.write(index_html(src, fp))

    ddir = os.path.join(out, "data")
    if os.path.isdir(os.path.join(ddir, "tasks")):
        shutil.rmtree(os.path.join(ddir, "tasks"))
    # the cache can live in the export itself: meshes are named by content
    cache = MeshCache(os.path.join(ddir, "mesh"))
    used: set[str] = set()
    index = []
    for tid, d in keyed:
        sc = data.scene(d, cache)
        if not include_reference:
            sc["reference"] = None
        if sc["reference"]:
            used.add(sc["reference"])
        _write(os.path.join(ddir, "tasks", tid, "scene.json"), sc)
        atts = data.attempts_doc(d, cache)
        for a in atts:
            if a.get("mesh"):
                used.add(a["mesh"])
            if a.get("marks"):
                step = data.find_step(d, a)
                m = data.marks(d, a, step) if step else {"regions": []}
                _write(os.path.join(ddir, "tasks", tid, "marks", f"{a['n']}.json"), m)
        _write(os.path.join(ddir, "tasks", tid, "attempts.json"), atts)
        # thumbnails: the task's fixed parts, and each kept attempt's part
        _write(os.path.join(ddir, "tasks", tid, "thumb.json"), data.task_thumb(d))
        for a in atts:
            if a.get("mesh"):
                m = data.attempt_thumb(d, a, os.path.join(out, ".thumbcache"))
                if m:
                    _write(os.path.join(ddir, "tasks", tid, "thumb", f"{a['n']}.json"), m)
        # downloads: every fixed part and every kept attempt, as STEP, STL and 3MF
        from . import downloads
        dl_dir = os.path.join(ddir, "tasks", tid, "download")
        names = [f"fixed_{f}" for f in sc["downloads"]["fixed"]] + \
            (["obstacles"] if sc["downloads"].get("obstacles") else []) + \
            [f"attempt-{a['n']}" for a in atts if a.get("mesh")]
        missing = []
        for base in names:
            for fmt in downloads.FORMATS:
                # a STEP the mesher cannot triangulate gives no STL/3MF; that
                # file is left out (and listed), never the whole export
                try:
                    fp = downloads.get(d, f"{base}.{fmt}", os.path.join(out, ".dlcache"))
                except Exception as e:  # noqa: BLE001
                    log(f"  {tid}: {base}.{fmt} not made ({type(e).__name__}); the STEP is still offered")
                    missing.append(f"{base}.{fmt}")
                    continue
                if fp:
                    os.makedirs(dl_dir, exist_ok=True)
                    shutil.copy2(fp, os.path.join(dl_dir, f"{base}.{fmt}"))
        if missing:
            _write(os.path.join(dl_dir, "missing.json"), missing)
        for a in atts:  # field files already made (run here only when asked: precompute)
            for proc in a.get("fields") or a.get("dfm") or []:
                fp = data.dfm_fields_path(d, a, proc)
                dst = os.path.join(ddir, "tasks", tid, "dfm", f"{a['n']}.{proc}.json")
                os.makedirs(os.path.dirname(dst), exist_ok=True)
                shutil.copy2(fp, dst)
        if precompute and "__" in tid:
            # the part the compare page shows: the first pass, else the last attempt
            sub = next((a for a in atts if a.get("passed") and a.get("mesh")), None) or next((a for a in reversed(atts) if a.get("mesh")), None)
            changed = False
            for proc in precompute if sub else []:
                if proc in (sub.get("fields") or []):
                    continue
                dst = os.path.join(ddir, "tasks", tid, "dfm", f"{sub['n']}.{proc}.json")
                res = data.run_dfm(d, sub, proc, out_path=dst)
                log(f"  {tid} #{sub['n']} {proc}: {'measured' if res.get('ok') else 'not measured: ' + str(res.get('error'))[:80]}")
                if res.get("ok"):
                    sub["fields"] = sorted(set(sub.get("fields") or []) | {proc})
                    changed = True
            if changed:
                _write(os.path.join(ddir, "tasks", tid, "attempts.json"), atts)
        for a in atts:  # archived code and grader text, for the timeline
            arc = a.get("archive")
            for name in (arc or {}).get("files", []):
                src_f = os.path.join(d, arc["dir"], name)
                if name.lower().endswith(data.TEXT_SUFFIXES) and os.path.getsize(src_f) <= data.MAX_TEXT:
                    dst = os.path.join(ddir, "tasks", tid, "archive", str(a["n"]), name)
                    os.makedirs(os.path.dirname(dst), exist_ok=True)
                    shutil.copy2(src_f, dst)
        for name in sc["files"]:
            fdir = os.path.join(ddir, "tasks", tid, "files")
            os.makedirs(fdir, exist_ok=True)
            shutil.copy2(os.path.join(d, name), os.path.join(fdir, name))
        index.append(data.index_entry(d, tid))
        log(f"  {tid}: {len(atts)} attempt(s), {sum(1 for a in atts if a.get('mesh'))} with a shape")
    shutil.rmtree(os.path.join(out, ".dlcache"), ignore_errors=True)
    shutil.rmtree(os.path.join(out, ".thumbcache"), ignore_errors=True)
    # drop meshes no task refers to (left from an earlier export)
    for name in os.listdir(cache.root):
        if name.endswith(".json") and name[:-5] not in used:
            os.remove(os.path.join(cache.root, name))
    _write(os.path.join(ddir, "index.json"), index)
    # the front page's showcase: only attempts whose meshes are in this copy
    root = targets[0] if len(targets) == 1 and os.path.isdir(targets[0]) else None
    def in_copy(e):
        tdir = os.path.join(ddir, "tasks", e["key"])
        if e.get("reference"):
            with open(os.path.join(tdir, "scene.json"), encoding="utf-8") as f:
                ref = json.load(f).get("reference")   # None when the reference is left out
            return bool(ref) and os.path.exists(os.path.join(cache.root, f"{ref}.json"))
        with open(os.path.join(tdir, "attempts.json"), encoding="utf-8") as f:
            att = next((a for a in json.load(f) if a["n"] == e["n"]), None)
        return bool(att and att.get("mesh")) and os.path.exists(os.path.join(cache.root, f"{att['mesh']}.json"))

    show = [e for e in data.showcase(keyed, root) if in_copy(e)]
    _write(os.path.join(ddir, "showcase.json"), show)
    has_scores = scores_src is not None
    if has_scores:
        from .score import score
        sc = dict(scores_src) if isinstance(scores_src, dict) else score(scores_src)
        from .rules import refresh_rules
        sc = refresh_rules(sc, dict(keyed).get)   # the label follows the data, not the code that scored it
        sc.pop("root", None)  # a local path means nothing on a public host
        early = [r for r in sc.get("runs", []) if not (r.get("rules") or {}).get("current")]
        if early and not dev_runs:
            sc["runs"] = [r for r in sc["runs"] if r not in early]
            sc["left_out"] = {"runs": len(early), "why": "graded under earlier rules (development data)"}
            log(f"  scoreboard: {len(early)} run(s) under earlier rules left out (--dev-runs to include)")
        _write(os.path.join(ddir, "scores.json"), sc)
    elif os.path.exists(os.path.join(ddir, "scores.json")):
        os.remove(os.path.join(ddir, "scores.json"))
    _write(os.path.join(ddir, "site.json"), {"mode": "static", "name": NAME, "tasks": len(index),
                                             "single": None, "scores": has_scores, "dfm_processes": []})
    log(f"wrote {out}  ({len(index)} tasks, {len(used)} meshes)")
    log(f"  view it: python3 -m http.server -d {out}  then open http://localhost:8000/")
    return 0

