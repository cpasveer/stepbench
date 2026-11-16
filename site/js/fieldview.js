// A measured field drawn smooth and exact-looking on the part (after the
// FreeCAD DFM Workbench's thickness map, which the maintainer pointed to).
//
// * Colour per pixel. Sampled fields (wall thickness, gap, section,
//   displacement) interpolate a value at each triangle corner: the field
//   file's `corner_values` (version 2), or for older files the same rule
//   computed here (area-weighted mean of the measured triangles of the same
//   face sharing the corner), so values never blend across a real edge.
// * Fields that are a function of the surface direction (overhang against
//   the build direction, draft against the pull) are computed in the shader
//   from the smooth surface normal and the stated direction: a cylinder shows
//   a continuous band and the limit is a clean line. Which faces are judged
//   at all (not the bed face, holes, bridges, faces across the pull) still
//   comes from the checker, per face.
// * Normals are smoothed within each face (curved faces show no facets) and
//   kept sharp across the solid's edges.
// * The limit is a crisp contour where the field crosses it; the failing
//   side is hatched in screen space, so the hatch does not follow triangles.
// * Turbo (an evenly bright rainbow) by default, viridis as the
//   colour-blind-safe switch; banded or smooth; only the failing range.

import * as THREE from '../vendor/three/three.module.js';

const esc = (s) => String(s ?? '').replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));

// ---------------------------------------------------------------- colour scales

const TURBO = { r4: [0.13572138, 4.61539260, -42.66032258, 132.13108234], r2: [-152.94239396, 59.28637943],
  g4: [0.09140261, 2.19418839, 4.84296658, -14.18503333], g2: [4.27729857, 2.82956604],
  b4: [0.10667330, 12.64194608, -60.58204836, 110.36276771], b2: [-89.90310912, 27.34824973] };
const VIR = [[0.2777273272234177, 0.005407344544966578, 0.3340998053353061], [0.1050930431085774, 1.404613529898575, 1.384590162594685],
  [-0.3308618287255563, 0.214847559468213, 0.09509516302823659], [-4.634230498983486, -5.799100973351585, -19.33244095627987],
  [6.228269936347081, 14.17993336680509, 56.69055260068105], [4.776384997670288, -13.74514537774601, -65.35303263337234],
  [-5.435455855934631, 4.645852612178535, 26.3124352495832]];

export function colourAt(map, t) {
  t = Math.min(1, Math.max(0, t));
  if (map === 'viridis') {
    const c = [0, 1, 2].map((k) => VIR.reduceRight((acc, row) => acc * t + row[k], 0));
    return c.map((x) => Math.min(1, Math.max(0, x)));
  }
  const p4 = [1, t, t * t, t * t * t], p2 = [t ** 4, t ** 5];
  const d = (a, b) => a.reduce((s, x, i) => s + x * b[i], 0);
  return [d(p4, TURBO.r4) + d(p2, TURBO.r2), d(p4, TURBO.g4) + d(p2, TURBO.g2), d(p4, TURBO.b4) + d(p2, TURBO.b2)].map((x) => Math.min(1, Math.max(0, x)));
}

export function gradientCSS(map, bands = 0) {
  const n = bands || 24, stops = [];
  for (let i = 0; i <= n; i++) {
    const t = bands ? (Math.min(i, n - 1) + 0.5) / n : i / n;
    const c = colourAt(map, t).map((x) => Math.round(x * 255)).join(',');
    if (bands) { if (i < n) stops.push(`rgb(${c}) ${(100 * i) / n}%`, `rgb(${c}) ${(100 * (i + 1)) / n}%`); } else stops.push(`rgb(${c}) ${(100 * i) / n}%`);
  }
  return `linear-gradient(90deg, ${stops.join(', ')})`;
}

const GLSL = `
uniform float uLo, uHi, uLimit, uKind, uMap, uBands, uOutside, uDirMode, uFlag, uHasLimit;
uniform vec3 uDir, uNeutral, uContour, uFlagCol;
varying float vF; varying float vM; varying float vB; varying vec3 vN;
vec3 jbTurbo(float x) {
  vec4 v4 = vec4(1.0, x, x * x, x * x * x); vec2 v2 = v4.zw * v4.z;
  return clamp(vec3(dot(v4, vec4(${TURBO.r4.join(',')})) + dot(v2, vec2(${TURBO.r2.join(',')})),
    dot(v4, vec4(${TURBO.g4.join(',')})) + dot(v2, vec2(${TURBO.g2.join(',')})),
    dot(v4, vec4(${TURBO.b4.join(',')})) + dot(v2, vec2(${TURBO.b2.join(',')}))), 0.0, 1.0);
}
vec3 jbViridis(float t) {
  ${VIR.map((c, i) => `const vec3 c${i} = vec3(${c.join(',')});`).join('\n  ')}
  return clamp(c0 + t * (c1 + t * (c2 + t * (c3 + t * (c4 + t * (c5 + t * c6))))), 0.0, 1.0);
}
`;

function fieldMaterial(u) {
  const mat = new THREE.MeshLambertMaterial({ color: 0xffffff, polygonOffset: true, polygonOffsetFactor: -1, polygonOffsetUnits: -1 });   // in front of a ghosted fixed part sharing a seat face
  mat.userData.uniforms = u;
  mat.onBeforeCompile = (sh) => {
    Object.assign(sh.uniforms, u);
    sh.vertexShader = sh.vertexShader
      .replace('#include <common>', '#include <common>\nattribute float fval; attribute float fmask; attribute float fbad;\nvarying float vF; varying float vM; varying float vB; varying vec3 vN;')
      .replace('#include <beginnormal_vertex>', '#include <beginnormal_vertex>\nvF = fval; vM = fmask; vB = fbad; vN = objectNormal;');
    sh.fragmentShader = sh.fragmentShader
      .replace('#include <common>', '#include <common>\n' + GLSL)
      .replace('vec4 diffuseColor = vec4( diffuse, opacity );', `
  float v = vF; float m = vM;
  if (uDirMode > 0.5) {
    float c = dot(normalize(vN), normalize(uDir));
    // faces turning up fade into the neutral colour over the last 2 degrees
    if (uDirMode < 1.5) { m *= smoothstep(0.0, 0.035, -c); v = degrees(asin(clamp(-c, 0.0, 1.0))); }
    else { v = degrees(asin(clamp(abs(c), 0.0, 1.0))); }
  }
  vec3 col = uNeutral;
  float fade = uDirMode > 0.5 ? m : 1.0;
  if (m > 0.001) {
    if (uFlag > 0.5) {
      col = v > 0.5 ? uFlagCol : uNeutral;
      if (vB > 0.5 && mod(gl_FragCoord.x + gl_FragCoord.y, 7.0) < 1.8) col = mix(col, vec3(0.0), 0.65);
    } else {
      float t = clamp((v - uLo) / max(1e-9, uHi - uLo), 0.0, 1.0);
      if (uBands > 0.5) t = (floor(min(t, 0.9999) * uBands) + 0.5) / uBands;
      col = uMap < 0.5 ? jbTurbo(t) : jbViridis(t);
      bool bad = uHasLimit > 0.5 ? (uKind > 0.5 ? v < uLimit : v > uLimit) : vB > 0.5;
      if (uOutside > 0.5 && !bad) col = mix(uNeutral, col, 0.15);
      if (bad && mod(gl_FragCoord.x + gl_FragCoord.y, 7.0) < 1.8) col = mix(col, vec3(0.0), 0.65);
      if (uHasLimit > 0.5) {
        float w = fwidth(v);
        float d = abs(v - uLimit) / max(w, 1e-6);
        col = mix(uContour, col, smoothstep(0.6, 1.6, d));
      }
    }
    col = mix(uNeutral, col, fade);
  }
  vec4 diffuseColor = vec4(col, opacity);`);
  };
  return mat;
}

// ---------------------------------------------------------------- geometry

const DIR_FIELDS = { overhang_angle: 1, draft_angle: 2 };

// corner values for a version 1 file: per face and corner, the area-weighted
// mean of the measured triangles of that face sharing the corner
function cornerValues(V, T, F, vals) {
  const area = (t) => { const [a, b, c] = T[t].map((i) => V[i]); const u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]], w = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
    return Math.hypot(u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0]) / 2; };
  const sum = new Map(), wt = new Map();
  for (let t = 0; t < T.length; t++) {
    const v = vals[t];
    if (v === null || v === undefined) continue;
    const a = area(t);
    for (const i of T[t]) { const k = `${i}:${F[t]}`; sum.set(k, (sum.get(k) || 0) + v * a); wt.set(k, (wt.get(k) || 0) + a); }
  }
  return T.map((tri, t) => tri.map((i) => { const k = `${i}:${F[t]}`; return wt.get(k) ? sum.get(k) / wt.get(k) : vals[t]; }));
}

// smooth normals within each face, sharp across faces
function faceNormals(V, T, F) {
  const acc = new Map();
  // weighted by the triangle's angle at the corner: closer to the true surface
  // normal than area weights on an irregular mesh
  const sub = (p, q) => [p[0] - q[0], p[1] - q[1], p[2] - q[2]];
  const ang = (u, w) => { const d = (u[0] * w[0] + u[1] * w[1] + u[2] * w[2]) / ((Math.hypot(...u) * Math.hypot(...w)) || 1); return Math.acos(Math.max(-1, Math.min(1, d))); };
  T.forEach((tri, t) => {
    const [a, b, c] = tri.map((i) => V[i]);
    const u = sub(b, a), w = sub(c, a);
    const n0 = [u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0]];
    const l = Math.hypot(...n0) || 1, n = n0.map((x) => x / l);
    const wts = [ang(sub(b, a), sub(c, a)), ang(sub(a, b), sub(c, b)), ang(sub(a, c), sub(b, c))];
    tri.forEach((i, k) => { const key = `${i}:${F[t]}`; const s = acc.get(key) || [0, 0, 0]; acc.set(key, [s[0] + n[0] * wts[k], s[1] + n[1] * wts[k], s[2] + n[2] * wts[k]]); });
  });
  return T.map((tri, t) => tri.map((i) => { const n = acc.get(`${i}:${F[t]}`); const l = Math.hypot(...n) || 1; return [n[0] / l, n[1] / l, n[2] / l]; }));
}

const cache = new WeakMap();   // per field file: geometry positions and normals

// a mesh without face numbers (the stiffness analysis's surface): smooth
// shading groups are made by crease angle, so sharp edges stay sharp
export function creaseFaces(V, T, deg = 30) {
  const n = T.map(([i, j, k]) => { const a = V[i], b = V[j], c = V[k]; const u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]], w = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
    const x = [u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0]]; const l = Math.hypot(...x) || 1; return x.map((q) => q / l); });
  const cos = Math.cos((deg * Math.PI) / 180), edge = new Map();
  T.forEach((tri, t) => { for (let k = 0; k < 3; k++) { const a = tri[k], b = tri[(k + 1) % 3], key = a < b ? `${a},${b}` : `${b},${a}`; (edge.get(key) || edge.set(key, []).get(key)).push(t); } });
  const parent = T.map((_, i) => i);
  const find = (x) => { while (parent[x] !== x) { parent[x] = parent[parent[x]]; x = parent[x]; } return x; };
  const creases = [];
  for (const [key, ts] of edge) {
    if (ts.length === 2) {
      const [a, b] = ts, d = n[a][0] * n[b][0] + n[a][1] * n[b][1] + n[a][2] * n[b][2];
      if (d >= cos) parent[find(a)] = find(b); else creases.push(key);
    } else creases.push(key);
  }
  const F = T.map((_, i) => find(i));
  const seg = [];
  for (const key of creases) { const [a, b] = key.split(',').map(Number); seg.push([V[a], V[b]]); }
  return { F, edges: seg };
}

function baseGeometry(file) {
  if (cache.has(file)) return cache.get(file);
  const V = file.mesh.vertices, T = file.mesh.triangles;
  const F = file.mesh.triangle_face || (file.mesh._crease || (file.mesh._crease = creaseFaces(V, T))).F;
  const pos = new Float32Array(T.length * 9), nor = new Float32Array(T.length * 9);
  const N = faceNormals(V, T, F);
  for (let t = 0; t < T.length; t++) for (let k = 0; k < 3; k++) { pos.set(V[T[t][k]], t * 9 + k * 3); nor.set(N[t][k], t * 9 + k * 3); }
  const out = { pos, nor, F };
  cache.set(file, out);
  return out;
}

// the default range shown: angles on fixed ranges so pictures compare;
// thicknesses and gaps capped at a few times the limit; always holding the limit
const FIXED = { overhang_angle: [0, 90], draft_angle: [0, 10] };
const CAP = { wall_thickness: 5, gap: 10, section_thickness: 2, bridge_span: 3, hole_diameter: 5, pin_diameter: 5 };
export function defaultRange(fs) {
  const list = Array.isArray(fs) ? fs : [fs];
  const f = list[0];
  if (FIXED[f.id]) return FIXED[f.id].slice();
  // a field that states its colour scale (stress: 0 to the 99th percentile, so one
  // sharp corner does not wash out the picture); above it is the top colour
  if (list.every((g) => g.scale)) return [Math.min(...list.map((g) => g.scale[0])), Math.max(...list.map((g) => g.scale[1]))];
  let lo = Infinity, hi = -Infinity;
  for (const g of list) if (g.range) { lo = Math.min(lo, g.range[0]); hi = Math.max(hi, g.range[1]); }
  if (!isFinite(lo)) { lo = 0; hi = 1; }
  if (CAP[f.id] && f.limit) hi = Math.min(hi, f.limit * CAP[f.id]);
  if (f.limit !== null && f.limit !== undefined) { lo = Math.min(lo, f.limit); hi = Math.max(hi, f.limit); }
  if (hi <= lo) hi = lo + 1;
  return [lo, hi];
}

// the shown state shared by every view of one field (one legend, one scale)
export function newState(f) {
  return { map: 'turbo', banded: false, outside: false, range: defaultRange(f), auto: true };
}

// the coloured part for one field; returns {mesh, set(state)}
export function fieldMesh(file, f, colours, state) {
  const { pos, nor, F } = baseGeometry(file);
  const T = file.mesh.triangles;
  const n = T.length;
  const fval = new Float32Array(n * 3), fmask = new Float32Array(n * 3), fbad = new Float32Array(n * 3);
  const dirMode = DIR_FIELDS[f.id] || 0;
  const vals = f.values || [];
  const fail = new Set(f.fail || []);
  const flag = f.type === 'flag';
  let corners = null;
  if (!flag && !dirMode) corners = f.corner_values || cornerValues(file.mesh.vertices, T, F, vals);
  // direction fields: judged per face (any measured triangle of the face)
  const judgedFace = new Set();
  if (dirMode) for (let t = 0; t < n; t++) if (vals[t] !== null && vals[t] !== undefined) judgedFace.add(F[t]);
  for (let t = 0; t < n; t++) {
    const measured = vals[t] !== null && vals[t] !== undefined;
    const m = dirMode ? (judgedFace.has(F[t]) ? 1 : 0) : measured ? 1 : 0;
    for (let k = 0; k < 3; k++) {
      const i = t * 3 + k;
      fmask[i] = m; fbad[i] = fail.has(t) ? 1 : 0;
      fval[i] = flag ? (measured ? vals[t] : 0) : corners ? (corners[t] && corners[t][k] !== null && corners[t][k] !== undefined ? corners[t][k] : (measured ? vals[t] : 0)) : 0;
    }
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  g.setAttribute('normal', new THREE.BufferAttribute(nor, 3));
  g.setAttribute('fval', new THREE.BufferAttribute(fval, 1));
  g.setAttribute('fmask', new THREE.BufferAttribute(fmask, 1));
  g.setAttribute('fbad', new THREE.BufferAttribute(fbad, 1));
  g.computeBoundingBox(); g.computeBoundingSphere();
  const dir = file.direction && file.direction.vector && file.direction.vector.length ? file.direction.vector : [0, 0, 1];
  const hasLimit = !flag && f.limit !== null && f.limit !== undefined && (f.kind === 'min' || f.kind === 'max');
  const u = {
    uLo: { value: 0 }, uHi: { value: 1 }, uLimit: { value: hasLimit ? f.limit : 0 }, uKind: { value: f.kind === 'min' ? 1 : -1 },
    uMap: { value: 0 }, uBands: { value: 0 }, uOutside: { value: 0 }, uDirMode: { value: dirMode }, uFlag: { value: flag ? 1 : 0 },
    uHasLimit: { value: hasLimit ? 1 : 0 }, uDir: { value: new THREE.Vector3(...dir) },
    uNeutral: { value: new THREE.Color(colours.neutral || '#d8dadd') }, uContour: { value: new THREE.Color(colours.contour || '#111111') },
    uFlagCol: { value: new THREE.Color(230 / 255, 159 / 255, 0) },
  };
  const mesh = new THREE.Mesh(g, fieldMaterial(u));
  mesh.userData.field = { file, f, corners, dirMode };
  const set = (st) => {
    u.uLo.value = st.range[0]; u.uHi.value = st.range[1];
    u.uMap.value = st.map === 'viridis' ? 1 : 0; u.uBands.value = st.banded ? 8 : 0; u.uOutside.value = st.outside ? 1 : 0;
  };
  set(state);
  return { mesh, set };
}

// the value drawn at a hit on the field mesh (interpolated as the shader does)
export function valueAt(hit) {
  const fd = hit.object.userData.field;
  if (!fd || !hit.face) return null;
  const t = Math.floor(hit.faceIndex);
  const { f, corners, dirMode, file } = fd;
  const vals = f.values || [];
  if (dirMode) {
    const nr = hit.object.geometry.attributes.normal, b = barycentric(hit);
    const n = new THREE.Vector3();
    for (let k = 0; k < 3; k++) n.addScaledVector(new THREE.Vector3().fromBufferAttribute(nr, t * 3 + k), b[k]);
    const d = new THREE.Vector3(...(file.direction && file.direction.vector && file.direction.vector.length ? file.direction.vector : [0, 0, 1])).normalize();
    const c = n.normalize().dot(d);
    if (dirMode === 1) return c >= 0 ? null : (Math.asin(Math.min(1, -c)) * 180) / Math.PI;
    return (Math.asin(Math.min(1, Math.abs(c))) * 180) / Math.PI;
  }
  if (vals[t] === null || vals[t] === undefined) return null;
  if (!corners || !corners[t]) return vals[t];
  const b = barycentric(hit);
  return corners[t].reduce((s, v, k) => s + (v ?? vals[t]) * b[k], 0);
}

function barycentric(hit) {
  const p = hit.object.geometry.attributes.position, t = hit.faceIndex;
  const a = new THREE.Vector3().fromBufferAttribute(p, t * 3), b = new THREE.Vector3().fromBufferAttribute(p, t * 3 + 1), c = new THREE.Vector3().fromBufferAttribute(p, t * 3 + 2);
  const out = new THREE.Vector3();
  THREE.Triangle.getBarycoord(hit.point, a, b, c, out);
  return [out.x, out.y, out.z];
}

export function fmtValue(v, unit) {
  if (v === null || v === undefined || !isFinite(v)) return '—';
  const a = Math.abs(v);
  const s = a >= 100 ? v.toFixed(0) : a >= 10 ? v.toFixed(1) : a >= 1 ? v.toFixed(2) : v.toFixed(3);
  return unit === 'deg' ? `${s}°` : `${s} ${unit === 'mm2' ? 'mm²' : unit || ''}`.trim();
}

// value tags placed automatically: the checker's probes (version 2), else
// the worst place and the lowest, median and highest measured triangle
export function autoProbes(file, f) {
  if (f.type === 'flag') return [];
  if (f.probes && f.probes.length) return spread(f.probes.filter((p) => p.point).map((p) => ({ at: new THREE.Vector3(...p.point), value: p.value, label: p.label })), file);
  const V = file.mesh.vertices, T = file.mesh.triangles, vals = f.values || [];
  const idx = [];
  for (let t = 0; t < T.length; t++) if (vals[t] !== null && vals[t] !== undefined) idx.push(t);
  if (!idx.length) return [];
  idx.sort((a, b) => vals[a] - vals[b]);
  const ctr = (t) => new THREE.Vector3(...[0, 1, 2].map((k) => (V[T[t][0]][k] + V[T[t][1]][k] + V[T[t][2]][k]) / 3));
  const out = [];
  const worst = (f.where || []).find((w) => w.point);
  if (worst) out.push({ at: new THREE.Vector3(...worst.point), value: f.kind === 'max' ? vals[idx[idx.length - 1]] : vals[idx[0]], label: f.status === 'fail' ? 'worst' : 'least margin' });
  const pick = [['lowest', 0], ['median', 0.5], ['highest', 1]];
  for (const [label, q] of pick) {
    const t = idx[Math.round(q * (idx.length - 1))];
    if (worst && out[0].at.distanceTo(ctr(t)) < 1e-6) continue;
    out.push({ at: ctr(t), value: vals[t], label });
  }
  return spread(out, file);
}

// one tag per spot: probes closer than 4 % of the part's size are one (the first kept: worst first)
function spread(list, file) {
  const b = new THREE.Box3(); for (const v of file.mesh.vertices) b.expandByPoint(new THREE.Vector3(...v));
  const min = b.getSize(new THREE.Vector3()).length() * 0.04, out = [];
  const order = list.slice().sort((a, c) => (c.label === 'worst' || c.label === 'least margin') - (a.label === 'worst' || a.label === 'least margin'));
  for (const p of order) if (!out.some((q) => q.at.distanceTo(p.at) < min)) out.push(p);
  return out;
}

// ---------------------------------------------------------------- the legend across the top

const nice = (span) => { const k = Math.pow(10, Math.floor(Math.log10(span / 5))); return [1, 2, 2.5, 5, 10].map((m) => m * k).find((s) => span / s <= 6) || 10 * k; };

// el: the container; fs: the field(s) shown (several in compare); state; onChange(state)
export function legendBar(el, fs, state, onChange, { title, choices, note, toggle, vertical = false } = {}) {
  // vertical: a slim upright bar for the corner of a view (0 at the bottom); otherwise across the top
  const S = vertical ? 'bottom' : 'left', L = vertical ? 'height' : 'width', DIR = vertical ? '0deg' : '90deg';
  el.classList.toggle('fb-vertical', !!vertical);
  const list = Array.isArray(fs) ? fs : [fs];
  const f = list[0];
  const flag = f.type === 'flag';
  let mn = Infinity, mx = -Infinity;
  for (const g of list) if (g.range) { mn = Math.min(mn, g.range[0]); mx = Math.max(mx, g.range[1]); }
  const [lo, hi] = state.range;
  // the axis stays put: it spans the full range (and any wider range chosen);
  // the handles narrow the coloured stretch, below it the bottom colour, above it the top
  const full = defaultRange(list);
  const A = Math.min(full[0], lo), B = Math.max(full[1], hi);
  const at = (v) => (100 * (v - A)) / (B - A);
  const unit = f.unit === 'deg' ? '°' : f.unit === 'mm2' ? 'mm²' : f.unit || '';
  if (flag) {
    el.innerHTML = `<div class="fb-head"><b>${esc(title || f.title || f.rule)}</b></div>
      <div class="fb-flag"><i class="sw" style="background:rgb(230,159,0)"></i> in the region <i class="sw fbn"></i> not <i class="sw hatch"></i> failing</div>`;
    el.hidden = false;
    return;
  }
  const edge = (t) => `rgb(${colourAt(state.map, t).map((x) => Math.round(x * 255)).join(',')})`;
  const step = nice(B - A), ticks = [];
  for (let v = Math.ceil(A / step) * step; v <= B + 1e-9; v += step) ticks.push(v);
  const lim = f.limit;
  const badSide = lim === null || lim === undefined ? '' : f.kind === 'min'
    ? `<i class="fb-bad" style="${S}:0;${L}:${Math.max(0, Math.min(100, at(lim)))}%"></i>` : `<i class="fb-bad" style="${S}:${Math.max(0, Math.min(100, at(lim)))}%;${vertical ? 'top' : 'right'}:0"></i>`;
  const fmt = (v) => (Math.abs(step) >= 1 ? (Math.round(v * 100) / 100).toString() : v.toFixed(Math.max(0, -Math.floor(Math.log10(step)) + 1)));
  el.innerHTML = `
    <div class="fb-head"><b>${esc(title || f.title || f.rule)}</b> <span class="dim">${esc(unit)}</span>
      <span class="fb-ctl">
        <button data-fb="banded" aria-pressed="${state.banded}" title="banded or smooth colouring">${state.banded ? 'banded' : 'smooth'}</button>
        <button data-fb="outside" aria-pressed="${state.outside}" title="colour only what fails the limit">${state.outside ? 'outside limit' : 'all values'}</button>
        <button data-fb="map" title="Turbo rainbow, or the colour-blind-safe viridis">${state.map === 'turbo' ? 'rainbow' : 'colour-blind safe'}</button>
        ${state.auto ? '' : '<button data-fb="reset" title="back to the full range">reset range</button>'}
      </span></div>
    ${choices && vertical ? `<div class="fb-choices"><select data-choicesel aria-label="load case">${choices.map((c) => `<option value="${esc(c.value)}" ${String(c.value) === String(state.choice ?? choices[0].value) ? 'selected' : ''}>${esc(c.label)}</option>`).join('')}</select>${toggle ? `<label class="fb-toggle"><input type="checkbox" data-toggle ${state.toggle ? 'checked' : ''}> ${esc(toggle)}</label>` : ''}</div>` : ''}
    ${choices && !vertical ? `<div class="fb-choices">${choices.map((c) => `<button data-choice="${esc(c.value)}" aria-pressed="${String(c.value) === String(state.choice ?? choices[0].value)}">${esc(c.label)}</button>`).join('')}${toggle ? `<label class="fb-toggle"><input type="checkbox" data-toggle ${state.toggle ? 'checked' : ''}> ${esc(toggle)}</label>` : ''}</div>` : ''}
    <div class="fb-body"><div class="fb-bar" style="background:linear-gradient(${DIR}, ${edge(0)} 0 ${at(lo)}%, ${edge(1)} ${at(hi)}% 100%)">
      <i class="fb-grad" style="${S}:${at(lo)}%;${L}:${at(hi) - at(lo)}%;background:${gradientCSS(state.map, state.banded ? 8 : 0).replace('90deg', DIR)}"></i>${badSide}
      ${lim !== null && lim !== undefined && lim >= A && lim <= B ? `<i class="fb-lim" style="${S}:${at(lim)}%"></i>` : ''}
      ${isFinite(mn) && mn >= A && mn <= B ? `<i class="fb-mm" style="${S}:${at(mn)}%" title="measured minimum ${fmt(mn)}"></i>` : ''}
      ${isFinite(mx) && mx >= A && mx <= B ? `<i class="fb-mm" style="${S}:${at(mx)}%" title="measured maximum ${fmt(mx)}"></i>` : ''}
      <i class="fb-h" data-h="0" style="${S}:${at(lo)}%" role="slider" tabindex="0" aria-label="lower end of the coloured range" aria-valuenow="${lo}"></i>
      <i class="fb-h" data-h="1" style="${S}:${at(hi)}%" role="slider" tabindex="0" aria-label="upper end of the coloured range" aria-valuenow="${hi}"></i>
    </div>
    <div class="fb-ticks">${ticks.filter((v) => lim === null || lim === undefined || Math.abs(at(v) - at(lim)) > 9).map((v) => `<span style="${S}:${at(v)}%">${fmt(v)}</span>`).join('')}${lim !== null && lim !== undefined && lim >= A && lim <= B ? `<span class="fb-limt" style="${S}:${at(lim)}%">limit ${fmt(lim)}</span>` : ''}</div></div>
    <div class="fb-mmtxt num">${vertical ? `max ${isFinite(mx) ? fmt(mx) : '—'} ${esc(unit)}` : `measured ${isFinite(mn) ? fmt(mn) : '—'} to ${isFinite(mx) ? fmt(mx) : '—'} ${esc(unit)}${state.auto ? '' : ` · coloured ${fmt(lo)} to ${fmt(hi)}`}${mx > hi ? ' · above: top colour' : ''}`}</div>
    ${note ? `<div class="fb-note">${esc(note)}</div>` : ''}`;
  el.hidden = false;
  el.onchange = (e) => {
    if (e.target.matches('[data-toggle]')) { state.toggle = e.target.checked; onChange(state, { rebuild: true }); }
    else if (e.target.matches('[data-choicesel]')) { state.choice = e.target.value; state.auto = true; onChange(state, { rebuild: true }); }
  };
  el.onclick = (e) => {
    const ch = e.target.closest('[data-choice]');
    if (ch) { state.choice = ch.dataset.choice; state.auto = true; onChange(state, { rebuild: true }); return; }
    const b = e.target.closest('[data-fb]');
    if (!b) return;
    const k = b.dataset.fb;
    if (k === 'banded') state.banded = !state.banded;
    else if (k === 'outside') state.outside = !state.outside;
    else if (k === 'map') state.map = state.map === 'turbo' ? 'viridis' : 'turbo';
    else if (k === 'reset') { state.range = defaultRange(list); state.auto = true; }
    onChange(state);
  };
  // dragging a handle narrows the coloured stretch; the axis and its numbers stay
  const bar = el.querySelector('.fb-bar'), grad = el.querySelector('.fb-grad');
  let pending = false;
  for (const hEl of el.querySelectorAll('.fb-h')) {
    const which = +hEl.dataset.h;
    const move = (cx, cy) => {
      const r = bar.getBoundingClientRect();
      const k = Math.min(1, Math.max(0, vertical ? (r.bottom - cy) / r.height : (cx - r.left) / r.width));
      const v = A + k * (B - A);
      const rg = state.range.slice();
      rg[which] = which ? Math.max(v, rg[0] + (B - A) * 0.02) : Math.min(v, rg[1] - (B - A) * 0.02);
      state.range = rg; state.auto = false;
      hEl.style[S] = `${at(rg[which])}%`;
      grad.style[S] = `${at(rg[0])}%`; grad.style[L] = `${at(rg[1]) - at(rg[0])}%`;
      bar.style.background = `linear-gradient(${DIR}, ${edge(0)} 0 ${at(rg[0])}%, ${edge(1)} ${at(rg[1])}% 100%)`;
      // the 3D views follow the handle as it moves (once per frame); the bar is redrawn on release
      if (!pending) { pending = true; requestAnimationFrame(() => { pending = false; onChange(state, { live: true }); }); }
    };
    hEl.onpointerdown = (e) => {
      e.preventDefault(); hEl.setPointerCapture(e.pointerId);
      hEl.onpointermove = (ev) => move(ev.clientX, ev.clientY);
      hEl.onpointerup = () => { hEl.onpointermove = null; onChange(state); };
    };
    hEl.onkeydown = (e) => {
      const up = e.key === 'ArrowRight' || e.key === 'ArrowUp', down = e.key === 'ArrowLeft' || e.key === 'ArrowDown';
      if (!up && !down) return;
      e.preventDefault();
      const d = (up ? 1 : -1) * (B - A) * 0.05, rg = state.range.slice();
      rg[which] += d;
      if (rg[1] - rg[0] > (B - A) * 0.02) { state.range = rg; state.auto = false; onChange(state); }
    };
  }
}
