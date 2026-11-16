// Clipping: section planes and a clipping box, with hatched cut faces.
//
// A plane removes what lies on its normal side: it is a unit normal n and a
// position d, and keeps the points with n.p <= d. Planes come three ways:
//   axis  -- across X, Y or Z through the view's pivot; the half nearer the
//            viewer is taken away, so one looks at the cut
//   face  -- click a face of a part: the plane takes the face's position and
//            outward normal (snapped to an axis when within 4 degrees) and
//            starts 0.5 mm outside it, so nothing is cut until it is dragged
//   box   -- six planes around the model; each face is dragged on its own,
//            never past the opposite face, to isolate a region
// Each plane has a handle in the drawing: a dot on the plane and an arrow to
// the side being cut away; dragging moves it along its normal (0.1 mm steps,
// 1 mm with Ctrl). (Tools and handle behaviour follow the maintainer's model
// viewer; this is a re-implementation for JointBench.)
//
// Cut faces are filled with the stencil buffer: per body and plane, faces
// behind the plane are counted (back +1, front -1), and a hatched cap is
// painted where the count is not zero -- exact for any closed solid. The
// cut outline is the plane's intersection with each mesh, clipped by the
// other planes.

import * as THREE from '../vendor/three/three.module.js';

const SNAP_DEG = 4;
const FACE_OFFSET = 0.5;   // mm outside a picked face
const MIN_BOX = 1;         // mm between opposite box faces
const AXES = ['x', 'y', 'z'];
const AXV = { x: new THREE.Vector3(1, 0, 0), y: new THREE.Vector3(0, 1, 0), z: new THREE.Vector3(0, 0, 1) };

// hatch per kind of body: the tile carries a 45 degree line, the rotation
// picks the direction (0 = /, pi/2 = \), step the pitch
export const HATCH = {
  fixed: { angle: 0, step: 1.0 },
  part: { angle: Math.PI / 2, step: 0.75 },
  ref: { angle: Math.PI / 4, step: 1.0 },
  obstacle: { angle: -Math.PI / 4, step: 1.2 },
};
const hatchCache = new Map();
function hatchTexture(fill, ink) {
  const key = fill + ink;
  if (hatchCache.has(key)) return hatchCache.get(key);
  const c = document.createElement('canvas');
  c.width = c.height = 32;
  const g = c.getContext('2d');
  g.fillStyle = fill; g.fillRect(0, 0, 32, 32);
  g.strokeStyle = ink; g.lineWidth = 2.2;
  for (const o of [-32, 0, 32]) { g.beginPath(); g.moveTo(o, 32); g.lineTo(o + 32, 0); g.stroke(); }
  const t = new THREE.CanvasTexture(c);
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.anisotropy = 4;
  t.center.set(0.5, 0.5);
  hatchCache.set(key, t);
  return t;
}

// snap a normal to the nearest world axis when within SNAP_DEG
export function snapNormal(n) {
  const out = n.clone().normalize();
  for (const a of AXES) {
    const v = AXV[a];
    for (const s of [1, -1]) {
      if (out.angleTo(v.clone().multiplyScalar(s)) < (SNAP_DEG * Math.PI) / 180) return v.clone().multiplyScalar(s);
    }
  }
  return out;
}

// the part of segment a-b kept by all planes (THREE.Plane, keep >= 0), or null
function clipSegment(a, b, planes) {
  let p = a.clone(), q = b.clone();
  for (const pl of planes) {
    const da = pl.distanceToPoint(p), db = pl.distanceToPoint(q);
    if (da < 0 && db < 0) return null;
    if (da < 0) p.lerp(q, da / (da - db));
    else if (db < 0) q.lerp(p, db / (db - da));
  }
  return [p, q];
}

// where `plane` cuts the triangles of `geom`, kept by `others`
function outline(geom, plane, others) {
  const pos = geom.attributes.position.array;
  const idx = geom.index ? geom.index.array : null;
  const n = idx ? idx.length : pos.length / 3;
  const out = [];
  const P = (i) => new THREE.Vector3(pos[3 * i], pos[3 * i + 1], pos[3 * i + 2]);
  for (let t = 0; t < n; t += 3) {
    const v = idx ? [idx[t], idx[t + 1], idx[t + 2]] : [t, t + 1, t + 2];
    const pts = v.map(P), d = pts.map((p) => plane.distanceToPoint(p));
    const cut = [];
    for (let k = 0; k < 3; k++) {
      const da = d[k], db = d[(k + 1) % 3];
      if ((da < 0) !== (db < 0)) cut.push(pts[k].clone().lerp(pts[(k + 1) % 3], da / (da - db)));
    }
    if (cut.length === 2) {
      const s = clipSegment(cut[0], cut[1], others);
      if (s) out.push(s[0].x, s[0].y, s[0].z, s[1].x, s[1].y, s[1].z);
    }
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.BufferAttribute(new Float32Array(out), 3));
  return g;
}

let nextId = 1;

export class Clipper {
  constructor(viewer) {
    this.v = viewer;
    this.planes = [];   // {id, kind, axis?, side?, n: Vector3, d: number, anchor: Vector3}
    this.picking = false;
    this.listeners = new Set();
  }

  onChange(fn) { this.listeners.add(fn); return () => this.listeners.delete(fn); }
  changed() { this.v.applyClip(); for (const f of this.listeners) f(); }

  get active() { return this.planes.length > 0; }
  bounds() { return this.v.box ? this.v.box.clone() : new THREE.Box3(new THREE.Vector3(-50, -50, -50), new THREE.Vector3(50, 50, 50)); }

  // THREE planes (keep where n.p <= d)
  three(only) {
    return (only || this.planes).map((p) => new THREE.Plane(p.n.clone().negate(), p.d));
  }

  // ---- making planes ----

  addAxis(axis, t = null) {
    const b = this.bounds(), i = AXES.indexOf(axis);
    const lo = b.min.getComponent(i), hi = b.max.getComponent(i);
    const pivot = this.v.nav.target.getComponent(i);
    const at = t === null ? Math.min(hi, Math.max(lo, pivot)) : lo + (hi - lo) * t;
    // take away the half nearer the viewer
    const eye = this.v.nav.camera.position.getComponent(i) - at;
    const n = AXV[axis].clone().multiplyScalar(eye >= 0 ? 1 : -1);
    const anchor = b.getCenter(new THREE.Vector3()); anchor.setComponent(i, at);
    const p = { id: nextId++, kind: 'axis', axis, n, d: n.dot(anchor), anchor };
    this.planes.push(p);
    this.changed();
    return p;
  }

  addFace(point, normal) {
    const n = snapNormal(normal);
    const p = { id: nextId++, kind: 'face', n, d: n.dot(point) + FACE_OFFSET, d0: n.dot(point), anchor: point.clone().addScaledVector(n, FACE_OFFSET) };
    this.planes.push(p);
    this.changed();
    return p;
  }

  setBox(on) {
    this.planes = this.planes.filter((p) => p.kind !== 'box');
    if (on) {
      const b = this.bounds().expandByScalar(1);
      const c = b.getCenter(new THREE.Vector3());
      for (const a of AXES) {
        for (const s of [1, -1]) {
          const n = AXV[a].clone().multiplyScalar(s);
          const anchor = c.clone(); anchor.setComponent(AXES.indexOf(a), s > 0 ? b.max.getComponent(AXES.indexOf(a)) : b.min.getComponent(AXES.indexOf(a)));
          this.planes.push({ id: nextId++, kind: 'box', axis: a, side: s, n, d: n.dot(anchor), anchor });
        }
      }
    }
    this.changed();
  }
  hasBox() { return this.planes.some((p) => p.kind === 'box'); }

  flip(id) {
    const p = this.planes.find((x) => x.id === id);
    if (!p || p.kind === 'box') return;
    p.n.negate(); p.d = -p.d; if (p.d0 !== undefined) p.d0 = -p.d0;
    this.changed();
  }
  remove(id) { this.planes = this.planes.filter((p) => p.id !== id); this.changed(); }
  clear() { this.planes = []; this.changed(); }

  // move a plane along its normal by `delta` mm (box faces stop short of their opposite)
  move(id, delta) {
    const p = this.planes.find((x) => x.id === id);
    if (!p) return;
    if (p.kind === 'box') {
      const opp = this.planes.find((x) => x.kind === 'box' && x.axis === p.axis && x.side === -p.side);
      // p keeps n.x <= d; opposite keeps -n.x <= d' i.e. n.x >= -d'
      const lowest = -opp.d + MIN_BOX;
      delta = Math.max(delta, lowest - p.d);
    }
    p.d += delta;
    p.anchor.addScaledVector(p.n, delta);
    this.changed();
  }

  // the signed position shown to the reader: along the world axis for axis
  // planes and box faces, along the normal for face planes
  readout(p) {
    if (p.kind === 'face') { const o = p.d - p.d0; return o >= 0 ? `${o.toFixed(1)} mm outside the face` : `${(-o).toFixed(1)} mm into the part`; }
    const i = AXES.indexOf(p.axis || 'x');
    return `${(p.axis || '').toUpperCase()} ${p.anchor.getComponent(i) >= 0 ? '+' : ''}${p.anchor.getComponent(i).toFixed(1)} mm`;
  }

  // ---- caps and outlines ----

  buildCaps(layer, bodies, col, radius) {
    for (const c of [...layer.children]) {
      layer.remove(c);
      c.traverse((o) => { if (o.material) o.material.dispose(); if (o.userData.ownGeo) o.geometry.dispose(); });
    }
    if (!this.planes.length) return;
    const all = this.three();
    const size = radius * 8;
    const spacing = Math.max(0.6, radius / 45);
    let order = 20;
    bodies.forEach(([b, kind]) => {
      const geo = b.mesh.geometry;
      all.forEach((plane, j) => {
        const others = all.filter((_, k) => k !== j);
        // the count for this plane sees the body cut by this plane only; the
        // cap is then trimmed to what the other planes keep
        const base = { depthWrite: false, depthTest: false, colorWrite: false, stencilWrite: true,
          stencilFunc: THREE.AlwaysStencilFunc, clippingPlanes: [plane] };
        const back = new THREE.Mesh(geo, new THREE.MeshBasicMaterial({ ...base, side: THREE.BackSide,
          stencilFail: THREE.IncrementWrapStencilOp, stencilZFail: THREE.IncrementWrapStencilOp, stencilZPass: THREE.IncrementWrapStencilOp }));
        const front = new THREE.Mesh(geo, new THREE.MeshBasicMaterial({ ...base, side: THREE.FrontSide,
          stencilFail: THREE.DecrementWrapStencilOp, stencilZFail: THREE.DecrementWrapStencilOp, stencilZPass: THREE.DecrementWrapStencilOp }));
        back.renderOrder = front.renderOrder = order;
        const h = HATCH[kind] || HATCH.fixed;
        const tex = hatchTexture(kind === 'part' ? col.hatchPart : col.hatchFixed, kind === 'part' ? col.partEdge : col.hatchInk).clone();
        tex.needsUpdate = true;
        tex.rotation = h.angle;
        const rep = size / (spacing * 3.2 * h.step);
        tex.repeat.set(rep, rep);
        const capMat = new THREE.MeshBasicMaterial({
          map: tex, side: THREE.DoubleSide, stencilWrite: true, stencilRef: 0, clippingPlanes: others,
          stencilFunc: THREE.NotEqualStencilFunc, stencilFail: THREE.ReplaceStencilOp,
          stencilZFail: THREE.ReplaceStencilOp, stencilZPass: THREE.ReplaceStencilOp,
          polygonOffset: true, polygonOffsetFactor: 1, polygonOffsetUnits: 1,
        });
        const cap = new THREE.Mesh(new THREE.PlaneGeometry(size, size), capMat);
        cap.userData.ownGeo = true;
        cap.position.copy(plane.coplanarPoint(new THREE.Vector3()));
        cap.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), plane.normal);
        cap.renderOrder = order + 1;
        cap.onAfterRender = (r) => r.clearStencil();
        const line = new THREE.LineSegments(outline(geo, plane, others),
          new THREE.LineBasicMaterial({ color: kind === 'part' ? col.partEdge : col.fixedEdge }));
        line.userData.ownGeo = true;
        line.renderOrder = order + 2;
        layer.add(back, front, cap, line);
        order += 3;
      });
    });
  }

  // ---- handles (SVG over the drawing) ----

  // where a plane crosses the model's box, padded: a polygon in world space
  crossing(p) {
    const b = this.bounds().expandByScalar(Math.max(2, this.v.radius * 0.08));
    const c = [];
    for (let i = 0; i < 8; i++) c.push(new THREE.Vector3(i & 1 ? b.max.x : b.min.x, i & 2 ? b.max.y : b.min.y, i & 4 ? b.max.z : b.min.z));
    const E = [[0, 1], [2, 3], [4, 5], [6, 7], [0, 2], [1, 3], [4, 6], [5, 7], [0, 4], [1, 5], [2, 6], [3, 7]];
    const pts = [];
    for (const [i, j] of E) {
      const da = p.n.dot(c[i]) - p.d, db = p.n.dot(c[j]) - p.d;
      if ((da < 0) !== (db < 0)) pts.push(c[i].clone().lerp(c[j], da / (da - db)));
    }
    if (pts.length < 3) return [];
    const ctr = pts.reduce((a, q) => a.add(q), new THREE.Vector3()).multiplyScalar(1 / pts.length);
    const u = new THREE.Vector3().subVectors(pts[0], ctr).normalize(), w = new THREE.Vector3().crossVectors(p.n, u);
    return pts.sort((a, q) => Math.atan2(w.dot(a.clone().sub(ctr)), u.dot(a.clone().sub(ctr))) - Math.atan2(w.dot(q.clone().sub(ctr)), u.dot(q.clone().sub(ctr))));
  }
}
