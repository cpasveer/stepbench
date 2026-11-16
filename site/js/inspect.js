// The check list in the side panel, and reviewing the part by one check in
// the 3D view.
//
// List: one dense line per check (mark, name, measured value, limit), grouped
// by requirement (fits, clean, makeable, sturdy, and a family's own checks).
// A group with a failure opens; one where everything passes stays closed.
//
// Inspection: clicking a check shows the part by that criterion. Every figure
// drawn comes from what the checkers wrote; this file only draws it:
//   * a field file (the manufacturing processes, the quality checker, the
//     stiffness analysis; layout in docs/dfm.md, "Field file"): the part
//     coloured per triangle, failing triangles striped, legend from the limit;
//   * otherwise the places a check names (`details.where`: point and face),
//     drawn as highlighted faces on the part;
//   * fits checks from their own details: bolt axes with their offsets,
//     ring contact, overlap regions, obstacles, tool paths;
//   * SECTION_MIN: each pair's thinnest cut drawn through the part;
//   * the manufacturing setup: build direction, the bed under the part and
//     the face it stands on, or for moulding the pull and a parting plane.
// Each inspection returns what to draw, labels, where to frame the camera,
// a legend and a sentence for the panel; app.js applies it.

import * as THREE from '../vendor/three/three.module.js';
import { viridis } from './dfm.js';
import { fieldMesh, newState, autoProbes, fmtValue, creaseFaces } from './fieldview.js';

const esc = (s) => String(s ?? '').replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));

// ---------------------------------------------------------------- grouping

export const GROUP_ORDER = ['fits', 'clean', 'makeable', 'sturdy'];
export const GROUP_TITLE = { fits: 'fits', clean: 'clean', makeable: 'makeable', sturdy: 'sturdy' };

export function groupOf(c) {
  if (c.group) return c.group;
  if (/^Q_/.test(c.name)) return 'clean';
  if (/^(FDM|PBF|IM|MILL|MOULD|CNC)_/.test(c.name)) return 'makeable';
  if (c.name === 'SECTION_MIN' || c.name === 'STIFFNESS') return 'sturdy';
  return 'fits';
}

export function groupChecks(checks) {
  const by = new Map();
  for (const c of checks || []) {
    const g = groupOf(c);
    if (!by.has(g)) by.set(g, []);
    by.get(g).push(c);
  }
  const names = [...GROUP_ORDER.filter((g) => by.has(g)), ...[...by.keys()].filter((g) => !GROUP_ORDER.includes(g))];
  return names.map((g) => {
    const cs = by.get(g);
    const counted = cs.filter((c) => c.status !== 'info');   // view-only lines (STRESS) are not checks
    const failed = counted.filter((c) => c.status === 'fail');
    const decides = counted.some((c) => c.required !== false);
    return { name: g, checks: cs, passed: counted.filter((c) => c.status === 'pass').length, total: counted.length,
      failed: failed.length, decides, failedRequired: failed.some((c) => c.required !== false) };
  });
}

// ---------------------------------------------------------------- one line

const UNIT = { mm2: 'mm²', mm3: 'mm³', deg: '°', ratio: '', solids: 'solid', voids: 'voids', pairs: 'pairs' };
const unitText = (u) => (u in UNIT ? UNIT[u] : (u || ''));

export function num(v) {
  if (typeof v !== 'number' || !isFinite(v)) return String(v ?? '');
  const a = Math.abs(v);
  if (a === 0) return '0';
  if (Number.isInteger(v) && a < 1e5) return String(v);
  if (a >= 1000) return v.toFixed(0);
  if (a >= 100) return v.toFixed(1);
  if (a >= 1) return v.toFixed(2);
  if (a >= 0.01) return v.toFixed(3);
  return v.toPrecision(2);
}

function withUnit(v, unit) {
  if (unit === 'ratio') return `${(v * 100).toFixed(0)} %`;
  const u = unitText(unit);
  return `${num(v)}${u === '°' ? '°' : u ? ' ' + u : ''}`;
}

// The limit a check is held to, from the task's tolerances or the check's own
// details; which side is good from the checker's own words ("min", "need",
// "max", "tol", "limit"), else from the value and the verdict.
export function limitOf(c, tol = {}) {
  const d = c.details || {};
  const t = (v, op, unit) => (v === undefined || v === null ? null : { v, op, unit });
  switch (c.name) {
    case 'FILE_VALID': return { v: 1, op: '=', unit: 'solids' };
    case 'HOLES': return t(tol.hole_axis_offset_max ?? 0.1, '≤', 'mm');
    case 'SEATING': return t(tol.seat_contact_min ?? 0.75, '≥', 'ratio');
    case 'NO_INTERFERENCE': case 'CLEAR_PATH': case 'TOOL_ACCESS': return t(tol.overlap_volume_max ?? 0.01, '≤', 'mm3');
    case 'SECTION_MIN': return { v: 1, op: '≥', unit: 'ratio' };
    default: break;
  }
  if (typeof d.limit !== 'number') return null;
  const r = c.reason || '';
  let op = /\((?:min|need|at least)\b/.test(r) ? '≥' : /\((?:max|tol|limit|at most)\b/.test(r) ? '≤' : null;
  if (!op && typeof c.value === 'number' && c.value !== d.limit) {
    const above = c.value >= d.limit;
    op = (c.status === 'pass') === above ? '≥' : '≤';
  }
  // no word and no value to compare: state the limit without a side
  return { v: d.limit, op: op || 'limit', unit: c.unit };
}

export function rowHTML(c, tol, { open = false, active = false } = {}) {
  const lim = limitOf(c, tol);
  const val = c.value === null || c.value === undefined ? '' : withUnit(c.value, c.unit);
  const limTxt = lim ? `${lim.op} ${withUnit(lim.v, lim.unit)}` : '';
  const adv = c.required === false;
  const mk = c.status === 'pass' ? '✓' : c.status === 'fail' ? '✕' : c.status === 'info' ? '◦' : '–';
  return `<div class="check ${esc(c.status)}${adv ? ' adv' : ''}${active ? ' active' : ''}${open ? ' open' : ''}" role="button" tabindex="0"
      aria-pressed="${active}" data-check="${esc(c.name)}" title="${esc(c.reason)}${adv ? ' (reported, does not decide the verdict)' : ''}">
    <span class="mk" aria-label="${esc(c.status)}">${mk}</span><span class="nm">${esc(c.name)}</span><span class="val num">${esc(val)}</span><span class="lim num">${esc(limTxt)}</span>
    ${c.status === 'fail' || open ? `<div class="why">${esc(c.reason)}</div>` : ''}
  </div>`;
}

// ---------------------------------------------------------------- sources

// which field file a check is drawn from, if any
export function sourceOf(c) {
  if (/^Q_/.test(c.name)) return 'quality';
  if (c.name === 'STIFFNESS' || c.name === 'STRESS') return 'stiffness';
  const p = c.details && c.details.process;
  if (p) return p;
  if (/^FDM_/.test(c.name)) return 'fdm';
  if (/^PBF_/.test(c.name)) return 'pbf';
  if (/^IM_/.test(c.name)) return 'moulding';
  return null;
}

// the part's display mesh (the viewer's own): positions, triangles, faces
export function decodeDisplay(m) {
  if (!m || !m.positions) return null;
  const b64 = (s, T) => { const bin = atob(s); const u = new Uint8Array(bin.length); for (let i = 0; i < bin.length; i++) u[i] = bin.charCodeAt(i); return new T(u.buffer); };
  const P = b64(m.positions, Float32Array), I = b64(m.indices, Uint32Array);
  return { P, I, faces: m.faces || [] };
}

// triangle corner positions of the given B-rep faces, from the field file
// (triangle_face) when there is one, else the display mesh (faces ranges)
function facePositions(faces, md, file) {
  const want = new Set(faces.filter((f) => f !== null && f !== undefined));
  const out = [];
  if (file && file.mesh && file.mesh.triangle_face) {
    const V = file.mesh.vertices, T = file.mesh.triangles, F = file.mesh.triangle_face;
    for (let t = 0; t < T.length; t++) if (want.has(F[t])) for (const k of T[t]) out.push(...V[k]);
  } else if (md) {
    for (const f of want) {
      const r = md.faces[f];
      if (!r) continue;
      for (let i = r[0]; i < r[0] + r[1]; i++) { const v = md.I[i]; out.push(md.P[3 * v], md.P[3 * v + 1], md.P[3 * v + 2]); }
    }
  }
  return new Float32Array(out);
}

function areaOf(pos) {
  let a = 0;
  const u = new THREE.Vector3(), v = new THREE.Vector3(), w = new THREE.Vector3();
  for (let i = 0; i + 8 < pos.length; i += 9) {
    u.set(pos[i], pos[i + 1], pos[i + 2]); v.set(pos[i + 3], pos[i + 4], pos[i + 5]); w.set(pos[i + 6], pos[i + 7], pos[i + 8]);
    a += v.sub(u).cross(w.sub(u)).length() / 2;
  }
  return a;
}

function partPoints(md, file) {
  if (file && file.mesh && file.mesh.vertices) return file.mesh.vertices;
  const out = [];
  if (md) for (let i = 0; i < md.P.length; i += 3) out.push([md.P[i], md.P[i + 1], md.P[i + 2]]);
  return out;
}

function boxOf(points) {
  const b = new THREE.Box3();
  for (const p of points) b.expandByPoint(new THREE.Vector3(p[0], p[1], p[2]));
  return b;
}

// ---------------------------------------------------------------- drawing helpers

function overlayFaces(pos, color, opacity = 0.85) {
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  g.computeVertexNormals();
  const m = new THREE.Mesh(g, new THREE.MeshBasicMaterial({ color, transparent: true, opacity, side: THREE.DoubleSide,
    polygonOffset: true, polygonOffsetFactor: -2, polygonOffsetUnits: -2, depthWrite: false }));
  m.renderOrder = 4;
  return m;
}

function dot(at, r, color) {
  const m = new THREE.Mesh(new THREE.SphereGeometry(r, 16, 12), new THREE.MeshBasicMaterial({ color, depthTest: false, transparent: true, opacity: 0.95 }));
  m.position.copy(at);
  m.renderOrder = 9;
  return m;
}

function line(points, color, { dashed = false, depthTest = false, opacity = 1 } = {}) {
  const g = new THREE.BufferGeometry().setFromPoints(points);
  const mat = dashed ? new THREE.LineDashedMaterial({ color, dashSize: 2, gapSize: 1.4, depthTest, transparent: true, opacity })
    : new THREE.LineBasicMaterial({ color, depthTest, transparent: true, opacity });
  const l = new THREE.Line(g, mat);
  if (dashed) l.computeLineDistances();
  l.renderOrder = 8;
  return l;
}

function perpBasis(n) {
  const a = Math.abs(n.z) < 0.9 ? new THREE.Vector3(0, 0, 1) : new THREE.Vector3(1, 0, 0);
  const u = new THREE.Vector3().crossVectors(n, a).normalize();
  const v = new THREE.Vector3().crossVectors(n, u).normalize();
  return { u, v };
}

// a square patch of a plane, filled faintly, outlined, with a grid
function planePatch(center, normal, size, color, { opacity = 0.16, grid = 0 } = {}) {
  const grp = new THREE.Group();
  const n = normal.clone().normalize(), { u, v } = perpBasis(n), h = size / 2;
  const c = (a, b) => center.clone().addScaledVector(u, a).addScaledVector(v, b);
  const corners = [c(-h, -h), c(h, -h), c(h, h), c(-h, h)];
  const g = new THREE.BufferGeometry().setFromPoints(corners);
  g.setIndex([0, 1, 2, 0, 2, 3]);
  const fill = new THREE.Mesh(g, new THREE.MeshBasicMaterial({ color, transparent: true, opacity, side: THREE.DoubleSide, depthWrite: false }));
  fill.renderOrder = 2;
  grp.add(fill, line([...corners, corners[0]], color, { depthTest: true, opacity: 0.8 }));
  if (grid > 0) {
    const seg = [];
    for (let k = -h + grid; k < h - 1e-6; k += grid) seg.push(c(k, -h), c(k, h), c(-h, k), c(h, k));
    const lg = new THREE.BufferGeometry().setFromPoints(seg);
    const ls = new THREE.LineSegments(lg, new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.3 }));
    grp.add(ls);
  }
  return grp;
}

// a thin arrow of a fixed size on screen (the viewer scales userData.px), drawn over everything
function arrow(from, dir, px, color) {
  const d = dir.clone().normalize(), grp = new THREE.Group();
  const mat = new THREE.MeshBasicMaterial({ color, depthTest: false, transparent: true, opacity: 0.9 });
  const len = 1, r = 0.012, head = 0.18;
  const shaft = new THREE.Mesh(new THREE.CylinderGeometry(r, r, len - head, 8), mat);
  const cone = new THREE.Mesh(new THREE.ConeGeometry(r * 3.5, head, 16), mat);
  shaft.position.y = (len - head) / 2; cone.position.y = len - head / 2;
  grp.add(shaft, cone);
  grp.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), d);
  grp.position.copy(from);
  grp.userData.px = px;
  grp.traverse((o) => { o.renderOrder = 9; });
  return grp;
}

// the outline of a set of triangles (edges used by one triangle only)
function outline(pos) {
  const key = (i) => `${pos[i].toFixed(3)},${pos[i + 1].toFixed(3)},${pos[i + 2].toFixed(3)}`;
  const count = new Map(), seg = new Map();
  for (let t = 0; t + 8 < pos.length; t += 9) {
    for (const [a, b] of [[0, 3], [3, 6], [6, 0]]) {
      const ka = key(t + a), kb = key(t + b), k = ka < kb ? ka + '|' + kb : kb + '|' + ka;
      count.set(k, (count.get(k) || 0) + 1);
      seg.set(k, [pos[t + a], pos[t + a + 1], pos[t + a + 2], pos[t + b], pos[t + b + 1], pos[t + b + 2]]);
    }
  }
  const out = [];
  for (const [k, n] of count) if (n === 1) out.push(...seg.get(k));
  return new Float32Array(out);
}

const niceStep = (x) => { const k = Math.pow(10, Math.floor(Math.log10(x))); return [1, 2, 5, 10].map((m) => m * k).find((s) => s >= x) || 10 * k; };
const vec = (p) => new THREE.Vector3(p[0], p[1], p[2]);
const fmtDir = (d) => `(${d.map((x) => x.toFixed(2)).join(', ')})`;

// ---------------------------------------------------------------- manufacturing setup

// How the part would be made, as far as the checks say: the build (or pull)
// direction, the bed and the face the part stands on, the parting plane.
export function processSetup({ checks, file, md, process, colours }) {
  // the direction as graded: the MANUFACTURING check records the declared one ("as declared")
  // or the checker's choice; else any of the process's own rule lines
  const hasDir = (c) => c.details && Array.isArray(c.details.direction) && c.details.direction.length === 3;
  const ck = (checks || []).find((c) => c.name === 'MANUFACTURING' && hasDir(c) && (!c.details.process || c.details.process === process))
    || (checks || []).find((c) => sourceOf(c) === process && hasDir(c));
  const vector = (file && file.direction && file.direction.vector && file.direction.vector.length === 3 ? file.direction.vector : null) || (ck && ck.details.direction);
  if (!vector || !vector.some((x) => x)) {
    // powder bed fusion: the part floats in the powder; the checker states no build direction
    if (process === 'pbf') return { group: new THREE.Group(), labels: [], text: 'Powder bed: printed in loose powder, so the checker needs no build direction and no bed face.' };
    return null;
  }
  const chosen = (file && file.direction && file.direction.chosen) || (ck && ck.details.direction_chosen) || '';
  const d = vec(vector).normalize();
  const pts = partPoints(md, file);
  if (!pts.length) return null;
  let lo = Infinity, hi = -Infinity;
  for (const p of pts) { const h = p[0] * d.x + p[1] * d.y + p[2] * d.z; if (h < lo) lo = h; if (h > hi) hi = h; }
  const box = boxOf(pts), ctr = box.getCenter(new THREE.Vector3()), size = box.getSize(new THREE.Vector3()).length();
  const onPlane = (h) => ctr.clone().addScaledVector(d, h - ctr.dot(d));
  const grp = new THREE.Group(), labels = [];
  let text = '';
  if (process === 'moulding') {
    const mid = onPlane((lo + hi) / 2);
    const side = perpBasis(d).u.multiplyScalar(size * 0.5);
    grp.add(planePatch(mid, d, size * 1.1, colours.datum, { opacity: 0.07 }));
    grp.add(arrow(onPlane(hi).add(side), d, 60, colours.datum), arrow(onPlane(lo).add(side), d.clone().negate(), 60, colours.datum));
    labels.push({ at: onPlane(hi).add(side), text: 'pull', bad: false, small: true },
      { at: onPlane(lo).add(side), text: 'pull', bad: false, small: true },
      { at: mid.clone().addScaledVector(perpBasis(d).u, size * 0.55), text: 'parting plane (assumed, mid-height)', bad: false, small: true });
    text = `Pull direction ${fmtDir(vector)}${chosen ? `, ${esc(chosen)}` : ''}. The checker states no parting line; the plane drawn is assumed through the middle of the part along the pull.`;
    return { group: grp, labels, text, direction: d };
  }
  // printed: the bed is the plane under the part, the build goes up from it
  const bed = onPlane(lo);
  grp.add(planePatch(bed, d, size * 1.1, colours.ink3, { opacity: 0.08, grid: niceStep(size / 8) }));
  const beside = bed.clone().addScaledVector(perpBasis(d).u, size * 0.5).addScaledVector(perpBasis(d).v, size * 0.5);
  grp.add(arrow(beside, d, 60, colours.datum));
  labels.push({ at: beside, text: 'build', bad: false, small: true });
  // the face it stands on: the bed-contact check's face, else "face N down"
  const bc = (checks || []).find((c) => /BED_CONTACT$/.test(c.name) && sourceOf(c) === process);
  const fromCheck = bc && bc.details && (bc.details.where || [])[0];
  const m = /face (\d+) down/.exec(chosen);
  const face = fromCheck && fromCheck.face !== null && fromCheck.face !== undefined ? fromCheck.face : m ? +m[1] : null;
  if (face !== null) {
    const pos = facePositions([face], md, file);
    if (pos.length) {
      const ol = new THREE.BufferGeometry(); ol.setAttribute('position', new THREE.BufferAttribute(outline(pos), 3));
      const ls = new THREE.LineSegments(ol, new THREE.LineBasicMaterial({ color: colours.datum, depthTest: false }));
      ls.renderOrder = 8;
      grp.add(ls);
      const area = areaOf(pos);
      const stated = bc && typeof bc.value === 'number' && fromCheck && fromCheck.face === face ? bc.value : null;
      const a = stated !== null ? stated : area;
      // the label hangs just under the face, below the bed, clear of labels on the part
      const fc = new THREE.Vector3(); for (let i = 0; i < pos.length; i += 3) fc.add(new THREE.Vector3(pos[i], pos[i + 1], pos[i + 2]));
      fc.divideScalar(pos.length / 3);
      const at = fc.clone().addScaledVector(d, -size * 0.09);
      grp.add(line([fc, at], colours.datum, { dashed: true }));
      labels.push({ at, text: `stands on face ${face}, ${num(a)} mm²`, bad: false, small: true });
      text = `Stands on face ${face}, ${num(a)} mm²${stated !== null ? ' on the bed' : ''}; build direction ${fmtDir(vector)}${chosen ? ` (${esc(chosen.replace(/\s*\(face \d+ down\)/, ''))})` : ''}.`;
    }
  }
  if (!text) text = `Build direction ${fmtDir(vector)}${chosen ? ` (${esc(chosen)})` : ''}.`;
  return { group: grp, labels, text, direction: d, face };
}

// ---------------------------------------------------------------- one check in 3D

// ctx: {c, att, checks, scene, pose, tol, md (decoded display mesh), files
// ({source: field file}), colours, regions (viewer's overlap regions), live}
export function inspection(ctx) {
  const { c } = ctx;
  const src = sourceOf(c);
  if (c.name === 'SECTION_MIN') return sectionView(ctx);
  if (c.name === 'STIFFNESS' || c.name === 'STRESS') return stiffnessView(ctx);
  if (['HOLES', 'SEATING', 'TOOL_ACCESS'].includes(c.name)) return boltView(ctx);
  if (c.name === 'NO_INTERFERENCE') return overlapView(ctx);
  if (c.name === 'CLEAR_PATH') return obstacleView(ctx);
  if (c.name === 'FILE_VALID') return solidView(ctx);
  if (src) return fieldView(ctx, src);
  return placesView(ctx, null);
}

const partSize = (ctx) => {
  const pts = partPoints(ctx.md, null);
  return pts.length ? boxOf(pts).getSize(new THREE.Vector3()).length() : 100;
};

// one field drawn smooth (fieldview.js): the coloured part, its true edges,
// the checker's marks, and value tags. state: the shared scale (one legend).
export function smoothField(file, f, colours, state) {
  const group = new THREE.Group(), labels = [];
  const fm = fieldMesh(file, f, { neutral: colours.neutral || '#d9dbde', contour: colours.contour || '#14161a' }, state);
  group.add(fm.mesh);
  const seg = [];
  // the solid's true edges; a surface without them (the stiffness mesh): its creases
  const lines = file.mesh.edges || (file.mesh._crease || (file.mesh._crease = creaseFaces(file.mesh.vertices, file.mesh.triangles))).edges;
  for (const ln of lines) for (let i = 0; i + 1 < ln.length; i++) seg.push(...ln[i], ...ln[i + 1]);
  const eg = new THREE.BufferGeometry();
  eg.setAttribute('position', new THREE.BufferAttribute(new Float32Array(seg), 3));
  group.add(new THREE.LineSegments(eg, new THREE.LineBasicMaterial({ color: colours.edgeLine || '#2a2f38', transparent: true, opacity: 0.75 })));
  for (const mk of f.marks || []) {
    if (mk.type !== 'point' && mk.points && mk.points.length > 1) group.add(line(mk.points.map(vec), mk.fail ? colours.bad : colours.ink));
  }
  for (const p of autoProbes(file, f)) labels.push({ at: p.at, text: `${['worst', 'least margin', 'peak'].includes(p.label) ? p.label + ' ' : ''}${fmtValue(p.value, f.unit)}`, probe: true, bad: p.label === 'worst' });
  return { group, labels, set: fm.set, mesh: fm.mesh };
}

function fieldView(ctx, src) {
  const { c, files, colours } = ctx;
  const file = files[src];
  const f = file && file.fields.find((x) => x.rule === c.name);
  if (!f) return placesView(ctx, src);
  const state = (ctx.fieldStates && ctx.fieldStates(f)) || newState(f);
  const sf = smoothField(file, f, colours, state);
  const { group, labels } = sf;
  const worst = (f.where || []).find((w) => w.point);
  const size = boxOf(file.mesh.vertices).getSize(new THREE.Vector3()).length();
  const setup = file.process && file.process !== 'quality' ? processSetup({ checks: ctx.checks, file, md: ctx.md, process: file.process, colours }) : null;
  if (setup) { group.add(setup.group); labels.push(...setup.labels); }
  return {
    group, labels, keepPart: false, ghostFixed: true,
    field: { f, state, set: sf.set },
    legend: setup ? `<div class="legend lg-setup">${setup.text}</div>` : '',
    frame: worst ? { at: vec(worst.point), r: size * 0.35 } : null,
    text: `${esc(f.note || '')}${setup ? `<br>${setup.text}` : ''}`,
  };
}

// no field file: the places the check names, as highlighted faces on the part
function placesView(ctx, src) {
  const { c, md, colours } = ctx;
  const where = ((c.details && c.details.where) || []).filter((w) => w.point || (w.face !== null && w.face !== undefined));
  const grp = new THREE.Group(), labels = [];
  const size = partSize(ctx);
  const bad = c.status === 'fail';
  const faces = where.map((w) => w.face).filter((f) => f !== null && f !== undefined);
  const pos = facePositions(faces, md, null);
  if (pos.length) grp.add(overlayFaces(pos, bad ? colours.bad : colours.datum, 0.7));
  where.forEach((w, i) => {
    if (!w.point) return;
    grp.add(dot(vec(w.point), size * 0.006, bad ? colours.bad : colours.ink));
    labels.push({ at: vec(w.point), text: i === 0 ? (bad ? 'worst' : 'least margin') : `${i + 1}`, bad });
  });
  const setup = src && src !== 'quality' && src !== 'stiffness' ? processSetup({ checks: ctx.checks, file: null, md, process: src, colours }) : null;
  if (setup) { grp.add(setup.group); labels.push(...setup.labels); }
  const lim = limitOf(c, ctx.tol);
  const need = src ? (ctx.live ? 'the coloured field is being measured' : 'the coloured field is made by the live viewer (jointbench play)') : 'this check writes no field';
  return {
    group: grp, labels, keepPart: true, ghostFixed: true,
    legend: `<div class="legend"><div class="lt">${esc(c.name)}</div><div class="lnote">${where.length ? `${where.length} place${where.length === 1 ? '' : 's'} named by the checker` : 'no place named'}${lim ? ` · limit ${lim.op} ${esc(withUnit(lim.v, lim.unit))}` : ''}; ${need}</div></div>${setup ? `<div class="legend lg-setup">${setup.text}</div>` : ''}`,
    frame: where[0] && where[0].point ? { at: vec(where[0].point), r: size * 0.35 } : null,
    text: setup ? setup.text : '',
    pending: !!src,
  };
}

function solidView(ctx) {
  const { c, md, colours } = ctx;
  const pts = partPoints(md, null);
  const ctr = pts.length ? boxOf(pts).getCenter(new THREE.Vector3()) : new THREE.Vector3();
  return { group: new THREE.Group(), labels: [{ at: ctr, text: c.reason, bad: c.status === 'fail' }], keepPart: true, ghostFixed: true,
    legend: `<div class="legend"><div class="lt">FILE_VALID</div><div class="lnote">${esc(c.reason)}</div></div>`, frame: null, text: '' };
}

function boltsOf(ctx) {
  const P = ctx.scene && ctx.scene.poses[ctx.pose || 0];
  return P ? P.bolts : [];
}

function boltView(ctx) {
  const { c, tol, colours } = ctx;
  const d = c.details || {}, labels = [], grp = new THREE.Group();
  const bolts = boltsOf(ctx), byId = Object.fromEntries(bolts.map((b) => [b.id, b]));
  let worst = null, worstScore = -Infinity, legend = '';
  const grips = Object.fromEntries(((ctx.checks.find((x) => x.name === 'HOLES') || {}).details?.holes || []).map((h) => [h.bolt, h.grip]));
  const at = (b, lift = 2) => vec(b.position).addScaledVector(vec(b.axis), (grips[b.id] || 0) + b.head_height + lift);
  if (c.name === 'HOLES') {
    const lim = tol.hole_axis_offset_max ?? 0.1;
    for (const h of d.holes || []) {
      const b = byId[h.bolt]; if (!b) continue;
      const bad = h.offset > lim;
      labels.push({ at: at(b), text: `${h.bolt} ${h.offset.toFixed(3)} mm`, bad });
      if (h.offset > worstScore) { worstScore = h.offset; worst = b; }
    }
    legend = `axis offset of each hole from its bolt, limit ≤ ${num(lim)} mm; also diameter and grip (in the panel)`;
  } else if (c.name === 'SEATING') {
    const lim = tol.seat_contact_min ?? 0.75;
    for (const r of d.bolts || []) {
      const b = byId[r.bolt]; if (!b) continue;
      labels.push({ at: at(b), text: `${r.bolt} ${(r.contact * 100).toFixed(0)} %`, bad: r.contact < lim });
      if (-r.contact > worstScore) { worstScore = -r.contact; worst = b; }
    }
    legend = `share of each bolt-head ring flat on its mounting face, limit ≥ ${(lim * 100).toFixed(0)} %`;
  } else {  // TOOL_ACCESS: the straight tool path above each head
    const lim = tol.overlap_volume_max ?? 0.01, per = {};
    for (const r of d.envelopes || []) if (!per[r.bolt] || r.volume > per[r.bolt].volume) per[r.bolt] = r;
    for (const [id, r] of Object.entries(per)) {
      const b = byId[id]; if (!b) continue;
      const bad = r.volume > lim;
      labels.push({ at: at(b, (tol.tool_length || 30) + 4), text: bad ? `${id} blocked by ${r.blocked_by}` : `${id} clear`, bad });
      if (r.volume > worstScore) { worstScore = r.volume; worst = b; }
    }
    legend = `a straight tool (${num(tol.tool_length || 30)} mm) from each bolt head along its axis; overlap limit ≤ ${num(lim)} mm³`;
  }
  return { group: grp, labels, keepPart: true, ghostFixed: false, showBolts: true, showFaces: c.name === 'SEATING',
    focusBolts: bolts.map((b) => b.id), hideBoltLabels: true,
    legend: `<div class="legend"><div class="lt">${esc(c.name)}</div><div class="lnote">${legend}</div></div>`,
    // a passing bolt check has no worst bolt worth flying to
    frame: worst && c.status === 'fail' ? { at: vec(worst.position), r: 60 } : null, text: '' };
}

function overlapView(ctx) {
  const { c, regions } = ctx;
  const rs = (regions || []).slice().sort((a, b) => b.volume - a.volume);
  const n = ((c.details || {}).overlaps || []).length;
  return { group: new THREE.Group(), labels: [], keepPart: true, ghostFixed: c.status === 'fail', showBolts: true, focusRegions: true,
    legend: `<div class="legend"><div class="lt">NO_INTERFERENCE</div><div class="lnote">${rs.length ? `${rs.length} overlap region${rs.length === 1 ? '' : 's'} in red, largest ${num(rs[0].volume)} mm³` : `no overlap in any of ${n} pairs: the part against each fixed part, and each bolt's shank and head space`}; limit ≤ ${num(ctx.tol.overlap_volume_max ?? 0.01)} mm³</div></div>`,
    frame: rs[0] ? { at: rs[0].at.clone(), r: 60 } : null, text: '' };
}

function obstacleView(ctx) {
  const { c, colours } = ctx;
  const P = ctx.scene && ctx.scene.poses[ctx.pose || 0];
  const obs = (P && P.obstacles) || [];
  const vol = Object.fromEntries((((c.details || {}).overlaps) || []).map((o) => [o.obstacle, o.volume]));
  const lim = ctx.tol.overlap_volume_max ?? 0.01;
  const labels = obs.map((o) => ({ at: vec(o.center), text: `${o.name} ${vol[o.name] !== undefined ? num(vol[o.name]) + ' mm³' : ''}`, bad: (vol[o.name] || 0) > lim }));
  const worst = obs.slice().sort((a, b) => (vol[b.name] || 0) - (vol[a.name] || 0))[0];
  return { group: new THREE.Group(), labels, keepPart: true, ghostFixed: true, obstacles: 'solid',
    legend: `<div class="legend"><div class="lt">CLEAR_PATH</div><div class="lnote">overlap of the part with each obstacle, limit ≤ ${num(lim)} mm³</div></div>`,
    frame: worst && (vol[worst.name] || 0) > lim ? { at: vec(worst.center), r: 80 } : null, text: '' };
}

function sectionView(ctx) {
  const { c, colours } = ctx;
  const secs = ((c.details || {}).sections) || [];
  const grp = new THREE.Group(), labels = [];
  const size = partSize(ctx);
  let worst = null, wr = Infinity;
  for (const s of secs) {
    const ratio = s.area / s.required, bad = ratio < 1;
    // a disc of the measured area would hide the cut; a square patch the
    // part's width marks the plane, the circle of equal area its size
    const n = vec(s.normal), at = vec(s.at);
    grp.add(planePatch(at, n, size * 0.28, bad ? colours.bad : colours.datum, { opacity: 0.28 }));
    if (s.outline && s.outline.length) {
      // the cut itself: its real edges, solid, on top of the part
      for (const w of s.outline) grp.add(line(w.map(vec), bad ? colours.bad : colours.datum));
    } else {
      // older records: a circle of the measured area
      const r = Math.sqrt(s.area / Math.PI), { u, v } = perpBasis(n);
      const ring = []; for (let k = 0; k <= 48; k++) { const t = (k / 48) * Math.PI * 2; ring.push(at.clone().addScaledVector(u, r * Math.cos(t)).addScaledVector(v, r * Math.sin(t))); }
      grp.add(line(ring, bad ? colours.bad : colours.datum, { dashed: true }));
    }
    labels.push({ at, text: `${s.pair} ${num(s.area)} mm² (need ${num(s.required)})`, bad });
    if (ratio < wr) { wr = ratio; worst = s; }
  }
  return { group: grp, labels, keepPart: true, ghostFixed: true,
    legend: `<div class="legend"><div class="lt">SECTION_MIN</div><div class="lnote">the thinnest cut between each pair of mounts, drawn where it cuts the part. Need ≥ the stated area (section factor ${num(ctx.tol.section_factor ?? 2)}).</div></div>`,
    frame: worst ? { at: vec(worst.at), r: size * 0.4 } : null, text: '', cut: worst ? { at: worst.at, normal: worst.normal } : null };
}

// the stiffness field file. Version 2 carries von Mises stress (MPa) for
// viewing, never a requirement: the envelope over the load cases first, each
// case on request, optionally on its deformed shape (displacement x design load
// x the file's exaggeration). Version 1: the weakest case's deformed shape.
function stiffnessView(ctx) {
  const { c, files, colours } = ctx;
  const file = files.stiffness;
  if (!file || !file.cases || !file.cases.length) return placesView(ctx, 'stiffness');
  if (file.stress_envelope) return stressView(ctx, file);
  return deformedView(ctx, file);
}

function stressView(ctx, file) {
  const { colours } = ctx;
  const key = { rule: 'STIFFNESS_STRESS', id: 'stress' };
  const state = (ctx.fieldStates && ctx.fieldStates(key)) || newState({ ...key, scale: file.stress_envelope.scale });
  const pick = state.choice === undefined || state.choice === 'envelope' ? null : file.cases[+state.choice];
  const src = pick ? pick.stress : file.stress_envelope;
  const T = file.mesh.triangles;
  let V = file.mesh.vertices;
  let factor = null;
  if (pick && state.toggle && pick.displacement) {
    // one exaggeration for every part shown together (state.exaggeration), so their shapes compare
    const ex = state.exaggeration || (pick.deformed && pick.deformed.exaggeration) || 1;
    factor = ((pick.design_load && pick.design_load.value) || 1) * ex;
    V = V.map((p, i) => { const d = pick.displacement[i]; return [p[0] + d[0] * factor, p[1] + d[1] * factor, p[2] + d[2] * factor]; });
  }
  const vv = src.vertex_values;
  const f = { id: 'stress', rule: 'STIFFNESS_STRESS', title: pick ? `von Mises stress, ${pick.label}` : 'von Mises stress, envelope of the load cases',
    type: 'scalar', unit: 'MPa', kind: 'max', limit: null, status: 'pass', values: src.values,
    corner_values: vv ? T.map((tri) => tri.map((i) => vv[i])) : undefined, range: src.range, scale: src.scale,
    probes: (src.probes || []).map((p) => ({ ...p, point: p.point || (p.vertex !== undefined ? V[p.vertex] : null) })) };
  if (state.auto && src.scale) state.range = src.scale.slice();
  const pseudo = { mesh: { vertices: V, triangles: T, _crease: file.mesh._crease || (file.mesh._crease = creaseFaces(file.mesh.vertices, T)) } };
  const sf = smoothField(pseudo, f, colours, state);
  if (factor) {
    const gg = new THREE.BufferGeometry();
    gg.setAttribute('position', new THREE.BufferAttribute(new Float32Array(T.flatMap((tri) => tri.flatMap((i) => file.mesh.vertices[i]))), 3));
    sf.group.add(new THREE.Mesh(gg, new THREE.MeshBasicMaterial({ color: colours.ink3, transparent: true, opacity: 0.12, depthWrite: false })));
  }
  // design_loads: {mount: {force_N, moment_Nmm, span_mm, basis}}
  const loads = Object.entries(file.design_loads || {}).map(([m, d]) => `at ${m}: ${d.force_N !== undefined ? I_num(d.force_N) + ' N' : ''}${d.moment_Nmm !== undefined ? ', ' + I_num(d.moment_Nmm) + ' N·mm' : ''}`);
  return {
    group: sf.group, labels: sf.labels, keepPart: false, ghostFixed: true,
    field: { f, state, set: sf.set,
      choices: [{ value: 'envelope', label: 'envelope' }, ...file.cases.map((cs, i) => ({ value: String(i), label: cs.label || `case ${i + 1}` }))],
      toggle: pick && pick.displacement ? 'on the deformed shape' : null,
      // the legend says it in one line; the panel carries the checker's full note
      note: 'For viewing only: not a requirement, in no verdict. The scale ends at the 99th percentile; a sharp inside corner has no finite peak.' },
    legend: '', frame: null,
    // side by side: one line; the full note stays on the part's own page
    short: `${pick && pick.design_load ? `${I_num(pick.design_load.value)} ${esc(pick.design_load.unit || '')}${factor ? `, shape ×${I_num(state.exaggeration || (pick.deformed || {}).exaggeration || 1)}` : ''}` : `loads: ${loads.map(esc).join('; ')}`}`,
    text: `${esc(file.stress_note || '')}${pick && pick.design_load ? `<br>Under the design load: ${I_num(pick.design_load.value)} ${esc(pick.design_load.unit || '')}${factor ? `; shape exaggerated ${I_num(state.exaggeration || (pick.deformed || {}).exaggeration || 1)}×` : ''}.` : ''}${loads.length ? `<br>Design loads: ${loads.map(esc).join('; ')}.` : ''}`,
  };
}
const I_num = (v) => num(v);

function deformedView(ctx, file) {
  const { c, colours } = ctx;
  const cs = file.cases.slice().sort((a, b) => (b.range ? b.range[1] : 0) - (a.range ? a.range[1] : 0))[0];
  const V = file.mesh.vertices, T = file.mesh.triangles, D = cs.displacement;
  let dmax = 0; for (const d of D) dmax = Math.max(dmax, Math.hypot(d[0], d[1], d[2]));
  const box = boxOf(V), span = box.getSize(new THREE.Vector3()).length();
  const k = dmax > 0 ? (0.08 * span) / dmax : 0;
  const pos = new Float32Array(T.length * 9), ghost = new Float32Array(T.length * 9), col = new Float32Array(T.length * 9);
  const [lo, hi] = cs.range || [0, 1];
  for (let t = 0; t < T.length; t++) {
    const cc = viridis((cs.magnitude[t] - lo) / ((hi - lo) || 1));
    for (let j = 0; j < 3; j++) {
      const i = T[t][j], p = V[i], d = D[i];
      pos.set([p[0] + d[0] * k, p[1] + d[1] * k, p[2] + d[2] * k], t * 9 + j * 3);
      ghost.set(p, t * 9 + j * 3);
      col.set(cc, t * 9 + j * 3);
    }
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.BufferAttribute(pos, 3)); g.setAttribute('color', new THREE.BufferAttribute(col, 3));
  g.computeVertexNormals();
  const gg = new THREE.BufferGeometry(); gg.setAttribute('position', new THREE.BufferAttribute(ghost, 3));
  const grp = new THREE.Group();
  grp.add(new THREE.Mesh(g, new THREE.MeshLambertMaterial({ vertexColors: true })));
  grp.add(new THREE.Mesh(gg, new THREE.MeshBasicMaterial({ color: colours.ink3, transparent: true, opacity: 0.15, depthWrite: false })));
  return { group: grp, labels: [], keepPart: false, ghostFixed: true,
    legend: `<div class="legend"><div class="lt">STIFFNESS · ${esc(cs.label || 'weakest case')}</div><div class="lnote">deformed shape, exaggerated ${num(k)}×; colour: displacement ${num(lo)} to ${num(hi)} ${esc((file.units || {}).displacement || '')}; grey: unloaded shape</div></div>`,
    frame: null, text: esc(c.reason) };
}
