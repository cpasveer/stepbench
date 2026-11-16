// The check list and the manufacturing setup (site/js/inspect.js), on made-up
// data whose answers are known. Run by tests/test_web.py (Node 22 or later).
import { groupChecks, limitOf, rowHTML, sourceOf, processSetup } from '../../site/js/inspect.js';
import { creaseFaces } from '../../site/js/fieldview.js';

const out = {};
const tol = { hole_axis_offset_max: 0.1, seat_contact_min: 0.75, overlap_volume_max: 0.01 };
const checks = [
  { name: 'HOLES', status: 'pass', required: true, group: 'fits', value: 0, unit: 'mm', reason: 'ok' },
  { name: 'SEATING', status: 'fail', required: true, group: 'fits', value: 0.6, unit: 'ratio', reason: 'ring 60 %' },
  { name: 'Q_TINY_FACE', status: 'pass', required: false, group: 'clean', value: 1.5, unit: 'mm2', reason: 'smallest face 1.5 mm2 (min 0.1 mm2)', details: { limit: 0.1 } },
  { name: 'Q_CREASE', status: 'fail', required: false, group: 'clean', value: 0.08, unit: 'deg', reason: '8 creases', details: { limit: 2 } },
  { name: 'FDM_OVERHANG', status: 'fail', required: false, group: 'makeable', value: 82, unit: 'deg', reason: 'overhang 82 deg (max 45 deg)', details: { limit: 45, process: 'fdm', direction: [0, 0, 1] } },
  { name: 'FDM_SEALED_HOLLOW', status: 'pass', required: false, group: 'makeable', value: 0, unit: 'voids', reason: 'no sealed hollows', details: { limit: 0, process: 'fdm' } },
  { name: 'FDM_BED_CONTACT', status: 'pass', required: false, group: 'makeable', value: 100, unit: 'mm2', reason: '100 mm2 flat on the bed (min 50 mm2)',
    details: { limit: 50, process: 'fdm', direction: [0, 0, 1], direction_chosen: 'best of 3 candidates (face 4 down)', where: [{ point: [5, 5, 0], face: 4 }] } },
  { name: 'SECTION_MIN', status: 'pass', required: true, group: 'sturdy', value: 1.3, unit: 'ratio', reason: 'ok' },
  { name: 'KEY_FIT', status: 'pass', required: true, group: 'keyed', value: 0.02, unit: 'mm', reason: 'ok' },
];
out.groups = groupChecks(checks).map((g) => [g.name, `${g.passed}/${g.total}`, g.failedRequired, g.decides]);
out.limits = Object.fromEntries(checks.map((c) => { const l = limitOf(c, tol); return [c.name, l ? `${l.op} ${l.v}` : null]; }));
out.sources = checks.map(sourceOf);
out.failRowHasReason = rowHTML(checks[3], tol).includes('class="why"');
out.passRowHasReason = rowHTML(checks[2], tol).includes('class="why"');
out.openRowHasReason = rowHTML(checks[2], tol, { open: true }).includes('class="why"');

// a 10 mm cube standing on its bottom face (face 4)
const V = [[0, 0, 0], [10, 0, 0], [10, 10, 0], [0, 10, 0], [0, 0, 10], [10, 0, 10], [10, 10, 10], [0, 10, 10]];
const T = [[0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7], [0, 1, 5], [0, 5, 4], [1, 2, 6], [1, 6, 5], [2, 3, 7], [2, 7, 6], [3, 0, 4], [3, 4, 7]];
const F = [4, 4, 5, 5, 0, 0, 1, 1, 2, 2, 3, 3];
const file = { process: 'fdm', direction: { name: 'build direction', vector: [0, 0, 1], chosen: 'best of 3 candidates (face 4 down)' },
  mesh: { vertices: V, triangles: T, triangle_face: F } };
const colours = { datum: '#36c', rule: '#888', ink3: '#888', ink: '#000', bad: '#c33' };
const fdm = processSetup({ checks, file, md: null, process: 'fdm', colours });
out.fdm = { text: fdm.text, face: fdm.face, labels: fdm.labels.map((l) => l.text) };
const mould = processSetup({ checks: [], file: { ...file, process: 'moulding', direction: { name: 'pull direction', vector: [0, 0, 1], chosen: 'stated' } }, md: null, process: 'moulding', colours });
out.mould = { text: mould.text, labels: mould.labels.map((l) => l.text) };
// a view-only line (STRESS) is listed but never counted
out.stressCounted = groupChecks([...checks, { name: 'STRESS', group: 'sturdy', status: 'info', required: false }]).find((g) => g.name === 'sturdy');
out.stressCounted = [out.stressCounted.passed, out.stressCounted.total, out.stressCounted.checks.length];
out.stressSource = sourceOf({ name: 'STRESS' });
// a surface without face numbers: shading groups by crease angle (a cube: 6 faces, 12 edges)
const cr = creaseFaces(V, T);
out.crease = [new Set(cr.F).size, cr.edges.length];
console.log(JSON.stringify(out));
