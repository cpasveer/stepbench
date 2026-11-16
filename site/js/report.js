// Reading the grader's report: no 3D here, so pages without a drawing can use it.

// Which bolts the grader says are wrong, and its words for each. Read from
// the report only: HOLES and SEATING list problems as "<bolt>: <why>",
// NO_INTERFERENCE lists rows "bolt <id> vs ..." with a volume.
export function boltsFromReport(att, tol) {
  const out = {};
  if (!att) return out;
  const lim = (tol && tol.overlap_volume_max) || 0.01;
  for (const c of att.checks || []) {
    if (c.status !== 'fail') continue;
    for (const p of (c.details && c.details.problems) || []) {
      const m = /^([A-Za-z]+\d+):\s*(.*)$/.exec(p);
      if (m) (out[m[1]] = out[m[1]] || []).push(`${c.name}: ${m[2]}`);
    }
    for (const r of (c.details && c.details.overlaps) || []) {
      const m = /^bolt (\S+) vs (.*)$/.exec(r.what || '');
      if (m && r.volume > lim) (out[m[1]] = out[m[1]] || []).push(`${c.name}: vs ${m[2]} ${r.volume.toFixed(1)} mm³`);
    }
  }
  return out;
}
