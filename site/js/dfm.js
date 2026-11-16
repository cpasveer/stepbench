// "Made how?": the manufacturing check of one attempt, drawn from the field
// file that `python -m jointbench.dfm --fields` writes (format: docs/dfm.md,
// "Field file"). Every pass/fail and every value here comes from that file;
// this module only chooses colours.
//
// Colour: viridis (dark purple to yellow), whose lightness rises steadily,
// so it reads for colour-blind viewers and in grey. Failing triangles are
// striped black and white on top of their colour. Triangles a rule does not
// measure are plain light grey; region rules (flags) mark their region in
// Okabe-Ito orange. These follow the checker's own pictures.

import * as THREE from '../vendor/three/three.module.js';

const h = (html) => { const t = document.createElement('template'); t.innerHTML = html.trim(); return t.content.firstElementChild; };
const esc = (s) => String(s ?? '').replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));

// viridis, 9 stops (matplotlib)
const VIRIDIS = [[68, 1, 84], [71, 44, 122], [59, 81, 139], [44, 113, 142], [33, 144, 141], [39, 173, 129], [92, 200, 99], [170, 220, 50], [253, 231, 37]];
export function viridis(t) {
  t = Math.min(1, Math.max(0, t)) * (VIRIDIS.length - 1);
  const i = Math.min(VIRIDIS.length - 2, Math.floor(t)), f = t - i;
  return VIRIDIS[i].map((c, k) => (c + (VIRIDIS[i + 1][k] - c) * f) / 255);
}
const VIRIDIS_CSS = `linear-gradient(90deg, ${VIRIDIS.map((c, i) => `rgb(${c.join(',')}) ${(100 * i) / (VIRIDIS.length - 1)}%`).join(', ')})`;
const GREY = [0.87, 0.87, 0.87];
const ORANGE = [230 / 255, 159 / 255, 0];

// the checker's scale choices (src/jointbench/dfm/viz.py): angles on a fixed
// range so pictures compare; thicknesses and gaps capped at a few times the limit
const FIXED = { overhang_angle: [0, 90], draft_angle: [0, 10] };
const CAP = { wall_thickness: 5, gap: 10, section_thickness: 2, bridge_span: 3, hole_diameter: 5, pin_diameter: 5 };

export function scaleOf(f) {
  if (FIXED[f.id]) return { lo: FIXED[f.id][0], hi: FIXED[f.id][1], capped: false };
  if (!f.range) return { lo: 0, hi: 1, capped: false };
  let [lo, hi] = f.range;
  let capped = false;
  if (CAP[f.id] && f.limit) {
    const cap = f.limit * CAP[f.id];
    if (hi > cap) { hi = cap; capped = true; }
    lo = Math.min(lo, f.limit);
  }
  if (f.limit !== null && f.limit !== undefined) { lo = Math.min(lo, f.limit); hi = Math.max(hi, f.limit); }
  if (hi <= lo) hi = lo + 1;
  return { lo, hi, capped };
}

let stripeTex = null;
function stripes() {
  if (stripeTex) return stripeTex;
  const c = document.createElement('canvas');
  c.width = 8; c.height = 1;
  const g = c.getContext('2d');
  g.fillStyle = '#000'; g.fillRect(0, 0, 4, 1);
  g.fillStyle = '#fff'; g.fillRect(4, 0, 4, 1);
  stripeTex = new THREE.CanvasTexture(c);
  stripeTex.wrapS = THREE.RepeatWrapping;
  stripeTex.magFilter = THREE.NearestFilter;
  return stripeTex;
}

// one field drawn on the checker's own mesh: per-triangle colour, failing
// triangles striped, true edges, marks and the worst place, the direction arrow
export function fieldGroup(file, f, colours) {
  const V = file.mesh.vertices, T = file.mesh.triangles;
  const pos = new Float32Array(T.length * 9), col = new Float32Array(T.length * 9);
  const sc = scaleOf(f);
  for (let t = 0; t < T.length; t++) {
    const v = f.values ? f.values[t] : null;
    let c;
    if (v === null || v === undefined) c = GREY;
    else if (f.type === 'flag') c = v ? ORANGE : GREY;
    else c = viridis((v - sc.lo) / (sc.hi - sc.lo));
    for (let k = 0; k < 3; k++) {
      const p = V[T[t][k]];
      pos.set(p, t * 9 + k * 3);
      col.set(c, t * 9 + k * 3);
    }
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  g.setAttribute('color', new THREE.BufferAttribute(col, 3));
  g.computeVertexNormals();
  g.computeBoundingBox();
  const group = new THREE.Group();
  group.add(new THREE.Mesh(g, new THREE.MeshLambertMaterial({
    vertexColors: true, polygonOffset: true, polygonOffsetFactor: 1, polygonOffsetUnits: 1,
  })));

  // failing triangles: black and white stripes, 1.2 mm wide, laid in world space
  if (f.fail && f.fail.length) {
    const fp = new Float32Array(f.fail.length * 9), uv = new Float32Array(f.fail.length * 6);
    f.fail.forEach((t, i) => {
      for (let k = 0; k < 3; k++) {
        const p = V[T[t][k]];
        fp.set(p, i * 9 + k * 3);
        uv.set([(p[0] + p[1] + p[2]) / 2.4, 0], i * 6 + k * 2);
      }
    });
    const fg = new THREE.BufferGeometry();
    fg.setAttribute('position', new THREE.BufferAttribute(fp, 3));
    fg.setAttribute('uv', new THREE.BufferAttribute(uv, 2));
    const m = new THREE.Mesh(fg, new THREE.MeshBasicMaterial({ map: stripes(), transparent: true, opacity: 0.6, polygonOffset: true, polygonOffsetFactor: -1, polygonOffsetUnits: -1 }));
    group.add(m);
  }

  // the solid's true edges
  const seg = [];
  for (const line of file.mesh.edges || []) for (let i = 0; i + 1 < line.length; i++) seg.push(...line[i], ...line[i + 1]);
  const eg = new THREE.BufferGeometry();
  eg.setAttribute('position', new THREE.BufferAttribute(new Float32Array(seg), 3));
  group.add(new THREE.LineSegments(eg, new THREE.LineBasicMaterial({ color: colours.edge })));

  // marks (bridges, sharp corners, points) and the worst place
  const labels = [];
  for (const mk of f.marks || []) {
    const c = mk.fail ? colours.bad : colours.ink;
    if (mk.type === 'point') {
      labels.push({ at: new THREE.Vector3(...mk.point), text: mk.label || '', bad: !!mk.fail });
    } else if (mk.points && mk.points.length > 1) {
      const lg = new THREE.BufferGeometry().setFromPoints(mk.points.map((p) => new THREE.Vector3(...p)));
      const line = new THREE.Line(lg, new THREE.LineBasicMaterial({ color: c, depthTest: false }));
      line.renderOrder = 5;
      group.add(line);
      if (mk.label) labels.push({ at: new THREE.Vector3(...mk.points[Math.floor(mk.points.length / 2)]), text: mk.label, bad: !!mk.fail });
    }
  }
  for (const w of f.where || []) {
    if (!w.point) continue;
    labels.push({ at: new THREE.Vector3(...w.point), text: f.status === 'fail' ? 'worst' : 'least margin', bad: f.status === 'fail' });
  }

  // build / pull direction
  const box = g.boundingBox;
  const size = box.getSize(new THREE.Vector3()).length();
  const ctr = box.getCenter(new THREE.Vector3());
  if (file.direction && file.direction.vector) {
    const d = new THREE.Vector3(...file.direction.vector).normalize();
    const start = ctr.clone().addScaledVector(d, size * 0.3);
    const arrow = new THREE.ArrowHelper(d, start, size * 0.2, colours.datum, size * 0.05, size * 0.025);
    arrow.traverse((o) => { if (o.material) { o.material.depthTest = false; } o.renderOrder = 6; });
    group.add(arrow);
    labels.push({ at: start.clone().addScaledVector(d, size * 0.24), text: file.direction.name || 'direction', bad: false });
  }
  return { group, labels };
}

export function legendHTML(file, f) {
  if (f.type === 'flag') {
    return `<div class="legend"><div class="lt">${esc(f.title || f.rule)}</div>
      <div class="lrow"><i class="sw" style="background:rgb(230,159,0)"></i> in the region <i class="sw" style="background:#ddd;margin-left:10px"></i> not</div>
      ${f.fail && f.fail.length ? '<div class="lrow"><i class="sw hatch"></i> failing</div>' : ''}</div>`;
  }
  if (!f.range) return `<div class="legend"><div class="lt">${esc(f.title || f.rule)}</div><div class="lrow dim">not measured anywhere on this part</div></div>`;
  const sc = scaleOf(f);
  const at = (v) => (100 * (v - sc.lo)) / (sc.hi - sc.lo);
  const lim = f.limit;
  // the failing side of the limit, hatched on the bar
  const bad = lim === null || lim === undefined ? '' : f.kind === 'min'
    ? `<i class="lbad" style="left:0;width:${at(lim)}%"></i>` : `<i class="lbad" style="left:${at(lim)}%;right:0"></i>`;
  const fmt = (v) => (Math.abs(v) >= 100 ? v.toFixed(0) : Math.abs(v) >= 10 ? v.toFixed(1) : v.toFixed(2));
  return `<div class="legend"><div class="lt">${esc(f.title || f.rule)} <span class="dim">${esc(f.unit)}</span></div>
    <div class="lbar" style="background:${VIRIDIS_CSS}">${bad}${lim !== null && lim !== undefined ? `<i class="llim" style="left:${at(lim)}%"></i>` : ''}</div>
    <div class="lax"><span>${fmt(sc.lo)}</span>${lim !== null && lim !== undefined ? `<span class="llab" style="left:${Math.min(80, Math.max(20, at(lim)))}%">limit ${fmt(lim)}</span>` : ''}<span>${sc.capped ? '≥ ' : ''}${fmt(sc.hi)}</span></div>
    <div class="lrow"><i class="sw hatch"></i> failing <i class="sw" style="background:#ddd;margin-left:10px"></i> not measured</div></div>`;
}

// the panel: rule list for one process; clicking a rule colours the part
export function dfmPanel({ file, process, processes, available, canRun, running, loading, onProcess, onRun, onField, current }) {
  const opts = processes.map((p) => `<option ${p === process ? 'selected' : ''}>${esc(p)}</option>`).join('');
  const head = `<div class="dfm-head">
    <label class="sel">process <select id="dfmproc">${opts}</select></label>
    ${file ? `<span class="stamp ${file.passed ? 'ok' : 'bad'}">${file.passed ? 'makeable' : 'not as is'}</span>` : ''}</div>`;
  if (!file) {
    const body = loading
      ? `<p class="note"><span class="spin"></span>loading the manufacturing check for ${esc(process)}…</p>`
      : running
      ? `<p class="note"><span class="spin"></span>running the manufacturing check for ${esc(process)} (about 20 s)…</p>`
      : canRun
        ? `<p class="note">No manufacturing check has been run on this attempt for <b>${esc(process)}</b>.</p><button class="btn small" id="dfmrun">Run it</button>
           <p class="note dim">Runs <code>python -m jointbench.dfm --process ${esc(process)} --fields</code> on the graded file; about 20 s. The result is kept.</p>`
        : `<p class="note dim">${available === null ? 'The graded file of this attempt was not kept, so it cannot be checked.' : 'No manufacturing check was recorded for this attempt in this copy.'}</p>`;
    const el = h(`<div class="dfm">${head}${body}</div>`);
    wire(el);
    return el;
  }
  const rows = file.fields.map((f) => `<div class="check ${f.status === 'pass' ? 'pass' : 'fail'} ${current === f.id ? 'cur' : ''}" data-f="${esc(f.id)}">
      <div class="mk">${f.status === 'pass' ? '✓' : '✕'}</div>
      <div class="nm"><span>${esc(f.rule)}</span><span class="val num">${f.limit === null || f.limit === undefined ? '' : `${f.kind === 'min' ? 'min' : 'max'} ${f.limit} ${esc(f.unit)}`}</span></div>
      <div class="why">${esc(f.reason || f.title || '')}</div><div></div></div>`).join('');
  const el = h(`<div class="dfm">${head}
    <p class="note">${esc(file.process_name || file.process)}. ${esc(file.direction?.name || 'direction')} <span class="num">(${(file.direction?.vector || []).map((v) => +v.toFixed(3)).join(', ')})</span>, ${esc(file.direction?.chosen || '')}. Click a rule to colour the part by it.</p>
    <div class="dfm-rules">${rows}</div>
    <p class="note dim">From the project's manufacturing checker, not from the joint grader: it does not change the verdict above.</p></div>`);
  el.querySelector('.dfm-rules').addEventListener('click', (e) => { const r = e.target.closest('[data-f]'); if (r) onField(r.dataset.f === current ? null : r.dataset.f); });
  wire(el);
  return el;

  function wire(root) {
    const s = root.querySelector('#dfmproc');
    if (s) s.addEventListener('change', () => onProcess(s.value));
    const b = root.querySelector('#dfmrun');
    if (b) b.addEventListener('click', onRun);
  }
}
