"""Fingerprinted asset paths, so a browser never mixes old and new files.

The page's scripts, styles and vendored three.js are served from
``v/<fingerprint>/...``, where the fingerprint is a hash of their contents.
Any change to any of them gives a new path, which no cache can have seen;
unchanged files keep their path and can be cached for good. The scripts
import each other with relative paths, so they all inherit the versioned
folder. Only ``index.html`` (which names the fingerprint) is never cached.
"""
from __future__ import annotations

import hashlib
import os

ASSET_DIRS = ("js", "css", "vendor")


def fingerprint(site: str) -> str:
    h = hashlib.sha256()
    for d in ASSET_DIRS:
        root = os.path.join(site, d)
        for base, dirs, files in sorted(os.walk(root)):
            dirs.sort()
            for f in sorted(files):
                p = os.path.join(base, f)
                h.update(os.path.relpath(p, site).encode())
                with open(p, "rb") as fh:
                    h.update(fh.read())
    return h.hexdigest()[:12]


def index_html(site: str, fp: str) -> str:
    """index.html with its asset references moved under v/<fp>/."""
    with open(os.path.join(site, "index.html"), encoding="utf-8") as f:
        html = f.read()
    return html.replace('href="css/', f'href="v/{fp}/css/').replace('src="js/', f'src="v/{fp}/js/')
