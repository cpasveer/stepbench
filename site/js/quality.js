// Quality figures that are not part of the joint verdict.
//
// Material: the task's par volume divided by the part's volume, for
// passing attempts -- the benchmark's one score. 1.00 = as much material as
// par; higher is leaner (par is a deliberately lean answer, not an optimum,
// so above 1 is possible). The grader writes it into the attempt (info.material: ratio, and
// whether it counts -- a part below the minimum wall does not); tasks
// without a par volume have none.
//
// Geometry cleanliness (sliver faces, tiny steps, knife edges) is to be
// marked in the scene the way the manufacturing fields are; it waits for the
// quality module's output format.

// the ratio when it counts, else null
export function materialRatio(att) {
  const m = att && att.info && att.info.material;
  return m && m.counts !== false && typeof m.ratio === 'number' ? m.ratio : null;
}

export function fmtRatio(r) {
  return r === null || r === undefined ? null : `${r.toFixed(2)}`;
}

export function median(values) {
  const v = values.filter((x) => x !== null && x !== undefined).sort((a, b) => a - b);
  if (!v.length) return null;
  const m = Math.floor(v.length / 2);
  return v.length % 2 ? v[m] : (v[m - 1] + v[m]) / 2;
}
