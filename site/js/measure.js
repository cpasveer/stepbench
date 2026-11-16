// Measuring: distance between two picked points, in millimetres.
//
// A pick snaps, in this order, to a corner of the part (a point where three
// or more true B-rep edges meet, within 9 px), to an edge (the nearest point
// on a B-rep edge, within 7 px), or to the face under the cursor. The
// cursor shows which: a square for a corner, a diamond for an edge, a dot
// for a face. While the second point is chosen a dashed line and a live
// length follow the cursor. A measurement is a line with a label at its
// middle; hovering the label shows dx, dy, dz and a cross to remove it.
// Esc drops a half-made measurement, then puts the tool down.
// (Behaviour after the maintainer's model viewer; written for JointBench.)

import * as THREE from '../vendor/three/three.module.js';

const VERTEX_PX = 9;
const EDGE_PX = 7;
const NS = 'http://www.w3.org/2000/svg';

function el(tag, attrs = {}, parent) {
  const e = document.createElementNS(NS, tag);
  for (const [k, v] of Object.entries(attrs)) e.setAttribute(k, v);
  if (parent) parent.appendChild(e);
  return e;
}

// corners of a body: points where 3+ edge segments end (curves continue through 2)
function corners(edgesGeo) {
  const a = edgesGeo.attributes.position.array, count = new Map(), pts = new Map();
  for (let i = 0; i < a.length; i += 3) {
    const k = `${a[i].toFixed(3)},${a[i + 1].toFixed(3)},${a[i + 2].toFixed(3)}`;
    count.set(k, (count.get(k) || 0) + 1);
    if (!pts.has(k)) pts.set(k, new THREE.Vector3(a[i], a[i + 1], a[i + 2]));
  }
  return [...count].filter(([, n]) => n >= 3).map(([k]) => pts.get(k));
}

export class Measure {
  constructor(viewer, svg) {
    this.v = viewer;
    this.svg = svg;
    this.on = false;
    this.items = [];     // {id, a, b}
    this.first = null;   // the first point of a measurement being made
    this.hover = null;   // {p, kind}
    this.cornerCache = new WeakMap();
    this.nextId = 1;
    this.listeners = new Set();
  }

  onChange(fn) { this.listeners.add(fn); }
  changed() { for (const f of this.listeners) f(); }

  start() { this.on = true; this.changed(); }
  stop() { this.on = false; this.first = null; this.hover = null; this.changed(); }
  escape() { if (this.first) { this.first = null; return true; } if (this.on) { this.stop(); return true; } return false; }
  clear() { this.items = []; this.first = null; this.changed(); }

  bodies() {
    const B = this.v.bodies;
    return [...B.fixed, B.part, B.ref].filter((b) => b && b.mode !== 'hidden' && b.group.visible);
  }

  toScreen(p) {
    const v = p.clone().project(this.v.camera);
    return [((v.x + 1) / 2) * this.v.width, ((1 - v.y) / 2) * this.v.height, v.z];
  }

  // the snapped point under the pointer
  snap(clientX, clientY) {
    const r = this.v.renderer.domElement.getBoundingClientRect();
    const sx = clientX - r.left, sy = clientY - r.top;
    const ndc = new THREE.Vector2((sx / r.width) * 2 - 1, -(sy / r.height) * 2 + 1);
    const hit = this.v.pickHit(ndc);
    const near = (p) => { const [x, y, z] = this.toScreen(p); return z < 1 ? Math.hypot(x - sx, y - sy) : Infinity; };
    // corners first, then edges, of the bodies drawn
    let best = null;
    for (const b of this.bodies()) {
      let cs = this.cornerCache.get(b);
      if (!cs) { cs = corners(b.lines.geometry); this.cornerCache.set(b, cs); }
      for (const c of cs) {
        if (!this.v.keptByClip(c)) continue;
        const d = near(c);
        if (d < VERTEX_PX && (!best || d < best.d)) best = { p: c.clone(), kind: 'corner', d };
      }
    }
    if (best) return best;
    for (const b of this.bodies()) {
      const a = b.lines.geometry.attributes.position.array;
      for (let i = 0; i < a.length; i += 6) {
        const P = new THREE.Vector3(a[i], a[i + 1], a[i + 2]), Q = new THREE.Vector3(a[i + 3], a[i + 4], a[i + 5]);
        const [px, py, pz] = this.toScreen(P), [qx, qy, qz] = this.toScreen(Q);
        if (pz >= 1 || qz >= 1) continue;
        const L2 = (qx - px) ** 2 + (qy - py) ** 2;
        const t = L2 ? Math.max(0, Math.min(1, ((sx - px) * (qx - px) + (sy - py) * (qy - py)) / L2)) : 0;
        const d = Math.hypot(px + t * (qx - px) - sx, py + t * (qy - py) - sy);
        if (d < EDGE_PX && (!best || d < best.d)) {
          const w = P.clone().lerp(Q, t);
          if (this.v.keptByClip(w)) best = { p: w, kind: 'edge', d };
        }
      }
    }
    if (best) return best;
    return hit ? { p: hit.point.clone(), kind: 'face', d: 0 } : null;
  }

  move(e) { if (this.on) this.hover = this.snap(e.clientX, e.clientY); }

  // a click while the tool is on: first point, then second point
  click(e) {
    if (!this.on) return false;
    const s = this.snap(e.clientX, e.clientY);
    if (!s) return true;
    if (!this.first) this.first = s.p;
    else {
      this.items.push({ id: this.nextId++, a: this.first, b: s.p });
      this.first = null;
      this.changed();
    }
    return true;
  }

  remove(id) { this.items = this.items.filter((m) => m.id !== id); this.changed(); }

  fmt(d) { return `${d.toFixed(2)} mm`; }

  draw() {
    const svg = this.svg;
    svg.replaceChildren();
    const line = (a, b, cls) => {
      const [ax, ay, az] = this.toScreen(a), [bx, by, bz] = this.toScreen(b);
      if (az >= 1 || bz >= 1) return null;
      el('line', { x1: ax, y1: ay, x2: bx, y2: by, class: cls }, svg);
      for (const [x, y] of [[ax, ay], [bx, by]]) el('circle', { cx: x, cy: y, r: 3, class: 'ms-end' }, svg);
      return [(ax + bx) / 2, (ay + by) / 2];
    };
    for (const m of this.items) {
      const mid = line(m.a, m.b, 'ms-line');
      if (!mid) continue;
      const d = m.a.distanceTo(m.b), dv = m.b.clone().sub(m.a);
      const g = el('g', { class: 'ms-label', transform: `translate(${mid[0]} ${mid[1]})` }, svg);
      const text = this.fmt(d);
      const w = text.length * 6.6 + 12;
      el('rect', { x: -w / 2, y: -10, width: w, height: 20, rx: 4 }, g);
      el('text', { 'text-anchor': 'middle', dy: '0.35em' }, g).textContent = text;
      const more = el('g', { class: 'ms-more', transform: 'translate(0 14)' }, g);
      const detail = `dx ${dv.x.toFixed(2)}  dy ${dv.y.toFixed(2)}  dz ${dv.z.toFixed(2)} mm`;
      const w2 = detail.length * 6 + 34;
      el('rect', { x: -w2 / 2, y: 0, width: w2, height: 20, rx: 4 }, more);
      el('text', { 'text-anchor': 'middle', dy: '0.35em', y: 10, x: -8 }, more).textContent = detail;
      const x = el('text', { class: 'ms-x', x: w2 / 2 - 12, y: 10, dy: '0.35em', 'text-anchor': 'middle', role: 'button', 'aria-label': 'Remove this measurement' }, more);
      x.textContent = '×';
      x.addEventListener('click', (ev) => { ev.stopPropagation(); this.remove(m.id); });
    }
    if (this.on && this.first && this.hover) {
      const mid = line(this.first, this.hover.p, 'ms-line live');
      if (mid) {
        const g = el('g', { class: 'ms-label live', transform: `translate(${mid[0]} ${mid[1]})` }, svg);
        const text = this.fmt(this.first.distanceTo(this.hover.p));
        const w = text.length * 6.6 + 12;
        el('rect', { x: -w / 2, y: -10, width: w, height: 20, rx: 4 }, g);
        el('text', { 'text-anchor': 'middle', dy: '0.35em' }, g).textContent = text;
      }
    }
    if (this.on && this.hover) {  // snap cursor: square corner, diamond edge, dot face
      const [x, y, z] = this.toScreen(this.hover.p);
      if (z < 1) {
        if (this.hover.kind === 'corner') el('rect', { x: x - 5, y: y - 5, width: 10, height: 10, class: 'ms-snap' }, svg);
        else if (this.hover.kind === 'edge') el('rect', { x: x - 4, y: y - 4, width: 8, height: 8, class: 'ms-snap', transform: `rotate(45 ${x} ${y})` }, svg);
        else el('circle', { cx: x, cy: y, r: 3.5, class: 'ms-snap' }, svg);
      }
    }
  }
}
