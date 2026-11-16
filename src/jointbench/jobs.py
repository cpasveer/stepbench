"""Running independent tasks on several processor cores.

Every task (one seed of one level) is generated and graded on its own, so
they can run in separate processes. Results come back in the order the
items were given, so output and files are the same as with one process.
"""
from __future__ import annotations

import os
from concurrent.futures import ProcessPoolExecutor


def n_jobs(jobs: int | None) -> int:
    """0 or None: all cores; otherwise as given (at least 1)."""
    if not jobs:
        return os.cpu_count() or 1
    return max(1, int(jobs))


def pmap(fn, items, jobs: int | None = 1):
    """``map(fn, items)`` across ``jobs`` processes, results in item order."""
    items = list(items)
    j = min(n_jobs(jobs) if jobs != 1 else 1, len(items) or 1)
    if j <= 1:
        for x in items:
            yield fn(x)
        return
    with ProcessPoolExecutor(max_workers=j) as ex:
        yield from ex.map(fn, items)
