"""`jointbench play` -- the local server behind the viewer.

The browser only draws. Every verdict it shows comes from the Python grader
that grades agents: an uploaded STEP is graded by ``grade.grade`` and logged
by ``grade.log_attempt``, exactly as ``jointbench verify`` does, and the
page then reads it back from ``attempts.jsonl`` like any other attempt. A
person and an agent are therefore scored by the same measurement, and an
agent working in the same folder shows up on the page as it goes.

Standard library only: ``http.server`` with one thread per request, and
Server-Sent Events (one long-lived HTTP response the server keeps writing
lines to) for the live log.
"""
from __future__ import annotations

import json
import mimetypes
import os
import posixpath
import re
import threading
import time
import urllib.parse
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from .. import NAME
from . import data, human
from .mesh import MeshCache

MAX_UPLOAD = 64 * 1024 * 1024
SHA_RE = re.compile(r"^[0-9a-f]{64}$")


def site_dir() -> str:
    """The front end lives in ``site/`` at the repository root."""
    here = os.path.dirname(os.path.abspath(__file__))
    for up in range(1, 6):
        cand = os.path.normpath(os.path.join(here, *[".."] * up, "site"))
        if os.path.exists(os.path.join(cand, "index.html")):
            return cand
    raise FileNotFoundError("site/index.html not found; run from a checkout of the repository")


class State:
    def __init__(self, root: str, tasks: list, scores=None):
        """tasks: task dirs, or (key, dir) pairs. scores: a runs folder to
        score live, a loaded results dict, or None."""
        self.root = root
        pairs = [(t if isinstance(t, tuple) else (os.path.basename(t), t)) for t in tasks]
        self.tasks = dict(pairs)
        self.order = [k for k, _ in pairs]
        self.scores = scores
        dirs = [d for _, d in pairs]
        cache_base = dirs[0] if len(dirs) == 1 and os.path.abspath(root) == dirs[0] else root
        self.meshes = MeshCache(os.path.join(cache_base, data.VIEW_DIR, "mesh"))
        self.lock = threading.Lock()
        self.grade_lock = threading.Lock()
        self.dfm_lock = threading.Lock()
        # where a person's tries are kept (see human.py)
        self.human_root = human.runs_root(root, self.tasks, scores if isinstance(scores, str) else None)
        self.targets: list[str] = []   # folders `play` was given: rescanned for new runs and tasks
        self.scores_file: str | None = None
        self._scene: dict[str, tuple[float, bytes]] = {}
        self.marks: dict = {}
        self.sizes: dict[str, int] = {}
        self.version = 0  # bumped whenever any attempts log changes
        self.cond = threading.Condition()

    def scene(self, tid: str) -> bytes:
        d = self.tasks[tid]
        mt = os.path.getmtime(os.path.join(d, "task.json"))
        with self.lock:
            hit = self._scene.get(tid)
            if hit and hit[0] == mt:
                return hit[1]
        body = json.dumps(data.scene(d, self.meshes), separators=(",", ":")).encode()
        with self.lock:
            self._scene[tid] = (mt, body)
        return body

    def rescan(self) -> list[str]:
        """Add task folders that appeared since the start; returns their keys."""
        keyed, src = data.collect(self.targets, self.scores_file)
        new = []
        with self.lock:
            for k, d in keyed:
                if k not in self.tasks:
                    self.tasks[k] = d
                    self.order.append(k)
                    self.sizes[k] = self.log_size(k)
                    new.append(k)
            if self.scores is None and isinstance(src, str):
                self.scores = src   # a runs folder: scores are computed live from it
        return new

    def log_size(self, tid: str) -> int:
        p = os.path.join(self.tasks[tid], "attempts.jsonl")
        try:
            return os.path.getsize(p)
        except OSError:
            return 0

    def watch(self, interval: float = 0.4) -> None:
        """Tail every attempts log. On growth: mesh the new attempt's file
        straight away (before an agent overwrites it with the next try),
        then wake every open event stream."""
        for tid in self.order:
            self.sizes[tid] = self.log_size(tid)
        last_scan = time.monotonic()
        while True:
            time.sleep(interval)
            changed = []
            # new runs and tasks appear without a restart (rescanned every 2 s)
            if self.targets and time.monotonic() - last_scan > 2.0:
                last_scan = time.monotonic()
                try:
                    changed += self.rescan()
                except Exception:  # noqa: BLE001 - a half-written folder is picked up next time
                    pass
            for tid in self.order:
                s = self.log_size(tid)
                if s != self.sizes.get(tid):
                    self.sizes[tid] = s
                    changed.append(tid)
            for tid in changed:
                try:
                    data.attempts_doc(self.tasks[tid], self.meshes)
                except Exception:  # noqa: BLE001 - meshing failure must not stop the tail
                    pass
            if changed:
                with self.cond:
                    self.version += 1
                    self.changed = changed
                    self.cond.notify_all()


_GZ_CACHE: dict = {}


def _compressible(ctype: str) -> bool:
    return ctype.startswith(("text/", "application/json", "image/svg", "application/javascript"))


def _gzip(body: bytes) -> bytes:
    """gzip, remembered by content: the same script or mesh is compressed once."""
    import gzip
    import hashlib

    key = hashlib.blake2b(body, digest_size=16).digest()
    hit = _GZ_CACHE.get(key)
    if hit is None:
        hit = gzip.compress(body, compresslevel=6, mtime=0)
        if len(_GZ_CACHE) > 512:
            _GZ_CACHE.clear()
        _GZ_CACHE[key] = hit
    return hit


def make_handler(state: State, static_root: str):
    class H(BaseHTTPRequestHandler):
        server_version = f"{NAME}-play"

        def log_message(self, fmt, *args):  # quiet: one line per upload only
            pass

        # ---- helpers ---------------------------------------------------

        def send_body(self, body: bytes, ctype: str, code: int = 200, cache: bool = False):
            # text, scripts and meshes shrink to a quarter or less with gzip
            # (three.js 1.3 MB -> 0.26 MB, a mesh 80 KB -> 28 KB)
            gz = (len(body) > 1400 and _compressible(ctype)
                  and "gzip" in (self.headers.get("Accept-Encoding") or ""))
            if gz:
                body = _gzip(body)
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            if gz:
                self.send_header("Content-Encoding", "gzip")
                self.send_header("Vary", "Accept-Encoding")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "max-age=31536000, immutable" if cache else "no-store")
            self.end_headers()
            self.wfile.write(body)

        def send_json(self, obj, code: int = 200):
            self.send_body(json.dumps(obj, separators=(",", ":")).encode(), "application/json", code)

        def not_found(self, what="not found"):
            self.send_json({"error": what}, 404)

        # ---- GET ---------------------------------------------------------

        def do_GET(self):
            u = urllib.parse.urlparse(self.path)
            path = urllib.parse.unquote(u.path)
            q = urllib.parse.parse_qs(u.query)
            try:
                if path.startswith("/data/"):
                    return self.get_data(path[len("/data/"):])
                if path == "/api/events":
                    return self.events(q.get("task", [None])[0])
                if path == "/api/clock":
                    tid, who = q.get("task", [""])[0], (q.get("who", [""])[0] or "").strip()
                    if tid not in state.tasks or not who:
                        return self.send_json({"clock": None})
                    return self.send_json({"clock": human.peek(state.human_root, state.tasks[tid], who)})
                return self.get_static(path)
            except (BrokenPipeError, ConnectionResetError):
                return
            except KeyError:
                return self.not_found()
            except Exception as e:  # noqa: BLE001 - answer with the reason, never drop the connection
                import traceback
                traceback.print_exc()
                try:
                    return self.send_json({"error": f"{type(e).__name__}: {e}", "path": path}, 500)
                except (BrokenPipeError, ConnectionResetError):
                    return

        def get_static(self, path: str):
            from .assets import fingerprint, index_html

            if path in ("", "/", "/index.html"):
                # the page names the current fingerprint; it is never cached
                body = index_html(static_root, fingerprint(static_root)).encode()
                return self.send_body(body, "text/html; charset=utf-8")
            rel = posixpath.normpath(path).lstrip("/")
            immutable = False
            if rel.startswith("v/"):  # v/<fingerprint>/<asset>: same bytes for good
                rel = rel.split("/", 2)[2] if rel.count("/") >= 2 else ""
                immutable = True
            if any(seg.startswith(".") for seg in rel.split("/")):
                return self.not_found()
            fp = os.path.join(static_root, *rel.split("/"))
            if not os.path.isfile(fp):
                return self.not_found()
            ctype = mimetypes.guess_type(fp)[0] or "application/octet-stream"
            if fp.endswith(".js"):
                ctype = "text/javascript"
            with open(fp, "rb") as f:
                self.send_body(f.read(), ctype, cache=immutable)

        def get_data(self, rel: str):
            parts = rel.split("/")
            if rel == "site.json":
                return self.send_json({"mode": "live", "name": NAME, "tasks": len(state.order),
                                       "single": state.order[0] if len(state.order) == 1 else None,
                                       "scores": state.scores is not None,
                                       "dfm_processes": data.dfm_processes(),
                                       "field_sources": data.field_sources()})
            if rel == "scores.json":
                if state.scores is None:
                    return self.not_found("no scores: give a runs folder or --scores <file>")
                if isinstance(state.scores, dict):
                    # a results file keeps its scorer's label: recompute it from the data
                    from .rules import refresh_rules
                    return self.send_json(refresh_rules(state.scores, state.tasks.get))
                from .score import score
                return self.send_json(score(state.scores))
            if rel == "showcase.json":
                return self.send_json(data.showcase(list(state.tasks.items()), state.root))
            if rel == "index.json":
                return self.send_json([data.index_entry(state.tasks[t], t) for t in state.order])
            if parts[0] == "mesh" and len(parts) == 2 and parts[1].endswith(".json"):
                sha = parts[1][:-5]
                if not SHA_RE.match(sha):
                    return self.not_found()
                body = state.meshes.get(sha)
                if body is None:
                    return self.not_found("mesh not made")
                return self.send_body(body, "application/json", cache=True)
            if parts[0] == "tasks" and len(parts) >= 3:
                tid = parts[1]
                d = state.tasks[tid]
                if parts[2] == "scene.json":
                    return self.send_body(state.scene(tid), "application/json")
                if parts[2] == "files" and len(parts) == 4 and parts[3] in data.task_files(d):
                    fp = os.path.join(d, parts[3])
                    with open(fp, "rb") as f:
                        body = f.read()
                    ctype = "application/json" if fp.endswith(".json") else "text/plain; charset=utf-8"
                    if fp.lower().endswith((".step", ".stp")):
                        ctype = "application/step"
                    return self.send_body(body, ctype)
                if parts[2] == "archive" and len(parts) == 5 and parts[3].isdigit():
                    att = next((a for a in data.read_attempts(d) if a.get("n") == int(parts[3])), None)
                    fp = att and data.archive_file(d, att, parts[4])
                    if not fp or not fp.lower().endswith(data.TEXT_SUFFIXES) or os.path.getsize(fp) > data.MAX_TEXT:
                        return self.not_found()
                    with open(fp, "rb") as f:
                        return self.send_body(f.read(), "text/plain; charset=utf-8")
                if parts[2] == "dfm" and len(parts) == 4 and parts[3].endswith(".json"):
                    n_s, _, proc = parts[3][:-5].partition(".")
                    att = next((a for a in data.read_attempts(d) if str(a.get("n")) == n_s), None)
                    fp = att and data.dfm_fields_path(d, att, proc)
                    if not fp:
                        return self.not_found("no manufacturing check for this attempt yet")
                    with open(fp, "rb") as f:
                        return self.send_body(f.read(), "application/json")
                if parts[2] == "thumb.json":
                    key = ("thumb", tid, os.path.getmtime(os.path.join(d, "task.json")))
                    if key not in state.marks:
                        state.marks[key] = json.dumps(data.task_thumb(d), separators=(",", ":")).encode()
                    return self.send_body(state.marks[key], "application/json")
                if parts[2] == "thumb" and len(parts) == 4 and parts[3].endswith(".json") and parts[3][:-5].isdigit():
                    att = next((a for a in data.read_attempts(d) if a.get("n") == int(parts[3][:-5])), None)
                    m = att and data.attempt_thumb(d, att, os.path.join(state.meshes.root, "..", "thumb"))
                    if not m:
                        return self.not_found("no thumbnail")
                    return self.send_json(m)
                if parts[2] == "download" and len(parts) == 4:
                    from . import downloads
                    try:
                        fp = downloads.get(d, parts[3], os.path.join(state.meshes.root, "..", "dl"))
                    except Exception as e:  # noqa: BLE001 - a STEP the mesher cannot triangulate
                        return self.send_json({"error": f"this file could not be converted ({type(e).__name__}); "
                                                        "the STEP download is exact and always works"}, 422)
                    if not fp:
                        return self.not_found("no such download")
                    with open(fp, "rb") as f:
                        body = f.read()
                    fmt = parts[3].rsplit(".", 1)[1]
                    name = downloads.file_name(os.path.basename(d), data.task_key_run(tid), parts[3])
                    self.send_response(200)
                    self.send_header("Content-Type", downloads.CONTENT_TYPES[fmt])
                    self.send_header("Content-Length", str(len(body)))
                    self.send_header("Content-Disposition", f'attachment; filename="{name}"')
                    self.send_header("Cache-Control", "no-store")
                    self.end_headers()
                    self.wfile.write(body)
                    return
                if parts[2] == "attempts.json":
                    return self.send_json(data.attempts_doc(d, state.meshes))
                if parts[2] == "marks" and len(parts) == 4 and parts[3].endswith(".json"):
                    n = int(parts[3][:-5])
                    att = next((a for a in data.read_attempts(d) if a.get("n") == n), None)
                    if att is None:
                        return self.not_found()
                    step = data.find_step(d, att)
                    if step is None:
                        return self.send_json({"regions": []})
                    key = (tid, n, att.get("sha256"))
                    if key not in state.marks:
                        state.marks[key] = data.marks(d, att, step)
                    return self.send_json(state.marks[key])
            return self.not_found()

        def events(self, tid):
            """Server-Sent Events: one 'attempts' message whenever a log
            grows. The page then re-reads attempts.json."""
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "keep-alive")
            self.end_headers()
            seen = state.version
            self.wfile.write(b": connected\n\n")
            self.wfile.flush()
            while True:
                with state.cond:
                    state.cond.wait(timeout=15)
                    v, changed = state.version, list(getattr(state, "changed", []))
                if v == seen:
                    self.wfile.write(b": keep-alive\n\n")
                else:
                    seen = v
                    for t in changed:
                        if tid is None or t == tid:
                            self.wfile.write(f"event: attempts\ndata: {json.dumps({'task': t})}\n\n".encode())
                self.wfile.flush()

        # ---- POST: grade an upload ----------------------------------------

        def do_POST(self):
            u = urllib.parse.urlparse(self.path)
            q = urllib.parse.parse_qs(u.query)
            if u.path == "/api/dfm":
                return self.post_dfm(q)
            if u.path == "/api/start":
                return self.post_start(q)
            if u.path != "/api/upload":
                return self.not_found()
            tid = q.get("task", [""])[0]
            if tid not in state.tasks:
                return self.not_found("unknown task")
            n = int(self.headers.get("Content-Length") or 0)
            if n <= 0 or n > MAX_UPLOAD:
                return self.send_json({"error": f"file must be 1 byte to {MAX_UPLOAD >> 20} MB"}, 400)
            body = self.rfile.read(n)
            who = (q.get("who", ["person"])[0] or "person").strip()[:40] or "person"
            tool = (q.get("tool", [""])[0] or "").strip()[:40] or None
            try:
                key = human_key(state, tid, who, tool)
                rec = grade_upload(state, key, body, who, tool)
                rdir = os.path.dirname(state.tasks[key])
                rec["clock"] = human.graded(rdir, state.tasks[key], rec["n"], bool(rec.get("passed")))
                rec["key"] = key
            except Exception as e:  # noqa: BLE001
                return self.send_json({"error": f"{type(e).__name__}: {e}"}, 500)
            print(f"  graded upload by {who} for {key}: {'PASS' if rec['passed'] else 'FAIL'} (attempt #{rec['n']})")
            return self.send_json(rec)

        def post_start(self, q):
            tid = q.get("task", [""])[0]
            if tid not in state.tasks:
                return self.not_found("unknown task")
            who = (q.get("who", [""])[0] or "").strip()[:40]
            if not who:
                return self.send_json({"error": "a name is needed to keep your tries apart"}, 400)
            tool = (q.get("tool", [""])[0] or "").strip()[:40] or None
            key = human_key(state, tid, who, tool)
            clock = human.start(os.path.dirname(state.tasks[key]), state.tasks[key])
            return self.send_json({"key": key, "clock": clock})

        def post_dfm(self, q):
            tid = q.get("task", [""])[0]
            if tid not in state.tasks:
                return self.not_found("unknown task")
            d = state.tasks[tid]
            n = q.get("n", [""])[0]
            att = next((a for a in data.read_attempts(d) if str(a.get("n")) == n), None)
            if att is None:
                return self.not_found("unknown attempt")
            proc = q.get("process", [""])[0]
            with state.dfm_lock:  # one manufacturing run at a time: each takes ~20 s of CPU
                if data.dfm_fields_path(d, att, proc):
                    return self.send_json({"ok": True, "cached": True})
                res = data.run_dfm(d, att, proc)
            print(f"  manufacturing check {proc} for {tid} #{n}: {'done' if res['ok'] else 'failed'}")
            return self.send_json(res, 200 if res["ok"] else 422)

    return H


def human_key(state: State, tid: str, who: str, tool: str | None) -> str:
    """The key of this person's copy of the task, made and registered on
    first use (see human.py)."""
    run_id, rdir, tdir = human.task_dir(state.human_root, state.tasks[tid], who, tool)
    key = f"{run_id}__{os.path.basename(tdir)}"
    with state.lock:
        if key not in state.tasks:
            state.tasks[key] = tdir
            state.order.append(key)
            state.sizes[key] = state.log_size(key)
        # a folder of tasks has no scores of its own: the people's runs become them
        if state.scores is None:
            state.scores = state.human_root
    return key


def grade_upload(state: State, tid: str, body: bytes, who: str, tool: str | None = None) -> dict:
    """Store the file by content, grade it with the agents' grader, log it."""
    import hashlib

    from ..grade import grade, log_attempt
    from ..task import Task

    d = state.tasks[tid]
    sha = hashlib.sha256(body).hexdigest()
    up = os.path.join(d, data.VIEW_DIR, "uploads")
    os.makedirs(up, exist_ok=True)
    path = os.path.join(up, f"{sha}.step")
    with open(path, "wb") as f:
        f.write(body)
    with state.grade_lock:  # one grading at a time keeps attempt numbers in order
        task = Task.load(os.path.join(d, "task.json"))
        rep = grade(task, path)
        n = log_attempt(d, path, rep, who=who, tool=tool)
    state.meshes.put_step(path, sha)
    rec = next((a for a in data.attempts_doc(d, state.meshes, make_missing=False) if a.get("n") == n), None)
    return rec or {"n": n, **rep.to_json()}


def serve(target: str, host: str = "127.0.0.1", port: int = 8765, open_browser: bool = True,
          scores_file: str | None = None) -> int:
    tasks, scores = data.collect([target], scores_file)
    if not tasks:
        print(f"no task.json in {target}, its sub-folders, or <run>/<task> folders")
        return 2
    from . import score as _score

    runs_folder = scores if isinstance(scores, str) else None
    hr = human.runs_root(os.path.abspath(target), dict(tasks), runs_folder)
    # a folder of tasks: people's earlier tries (human-runs) are shown and scored too
    if runs_folder is None and _score.is_runs_folder(hr):
        tasks = list(tasks) + _score.run_tasks(hr)
        scores = scores if scores is not None else hr
    state = State(os.path.abspath(target), tasks, scores)
    state.targets, state.scores_file = [target], scores_file
    state.human_root = hr
    static_root = site_dir()
    httpd = None
    for p in range(port, port + 20):
        try:
            httpd = ThreadingHTTPServer((host, p), make_handler(state, static_root))
            break
        except OSError:
            continue
    if httpd is None:
        print(f"no free port in {port}..{port + 19}")
        return 2
    httpd.daemon_threads = True
    threading.Thread(target=state.watch, daemon=True).start()
    url = f"http://{host}:{httpd.server_port}/"
    if len(tasks) == 1:
        url += f"#/task/{tasks[0][0]}"
    elif scores is not None:
        url += "#/scores"
    print(f"{NAME} play: {len(tasks)} task(s) from {target}")
    print(f"  open {url}")
    print("  uploads are graded by the same grader as `verify`; each person's tries are kept as their own run in")
    print(f"  {state.human_root}")
    print("  Ctrl-C to stop")
    if open_browser:
        threading.Timer(0.5, lambda: webbrowser.open(url)).start()
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        httpd.server_close()
    return 0
