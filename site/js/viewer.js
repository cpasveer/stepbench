// The 3D drawing. It draws what Python sends -- meshes, true B-rep edges,
// mounting faces, bolts -- and decides nothing. Pass/fail marks come from
// the grader's report; this file only knows where to put the colour.
//
// World axes follow CAD, not three.js: Z is up. Units are millimetres.
// Navigation is in nav.js (perspective by default, parallel for the axis
// views), clipping in clip.js, measuring in measure.js.

import * as THREE from '../vendor/three/three.module.js';
import { Nav, VIEWS, uprightQ } from './nav.js';
import { Clipper } from './clip.js';
import { Measure } from './measure.js';
import { boltsFromReport } from './report.js';

export { boltsFromReport };

const UP = new THREE.Vector3(0, 0, 1);

function css(name) {
  return getComputedStyle(document.documentElement).getPropertyValue(name).trim() || '#888';
}

function b64(str, Type) {
  const bin = atob(str);
  const bytes = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i);
  return new Type(bytes.buffer);
}

export function decodeMesh(m) {
  if (!m || m.error || !m.positions) return null;
  const pos = b64(m.positions, Float32Array);
  const idx = b64(m.indices, Uint32Array);
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  g.setIndex(new THREE.BufferAttribute(idx, 1));
  g.computeVertexNormals();
  g.computeBoundingBox();
  const e = new THREE.BufferGeometry();
  e.setAttribute('position', new THREE.BufferAttribute(b64(m.edges, Float32Array), 3));
  return { faces: g, edges: e };
}

// A keep-out box from the task file: centre, size along its own axes.
export function boxGeometry(o) {
  const g = new THREE.BoxGeometry(o.size[0], o.size[1], o.size[2]);
  const [ax, ay, az] = o.axes.map((v) => new THREE.Vector3(...v));
  const m = new THREE.Matrix4().makeBasis(ax, ay, az).setPosition(...o.center);
  g.applyMatrix4(m);
  g.computeBoundingBox();
  const e = new THREE.EdgesGeometry(g);
  return { faces: g, edges: e };
}

// A solid drawn as a CAD body: shaded faces and true edges. Where a section
// plane cuts it, the viewer adds a hatched cap (see clip.js).
class Body {
  constructor(geo, role, name) {
    this.role = role;
    this.name = name;
    this.group = new THREE.Group();
    this.group.userData.body = this;
    this.mat = new THREE.MeshStandardMaterial({
      roughness: 0.62, metalness: 0.08, side: THREE.FrontSide,
      polygonOffset: true, polygonOffsetFactor: 1, polygonOffsetUnits: 1,
    });
    this.edgeMat = new THREE.LineBasicMaterial({ transparent: true });
    this.mesh = new THREE.Mesh(geo.faces, this.mat);
    this.mesh.userData.body = this;
    this.lines = new THREE.LineSegments(geo.edges, this.edgeMat);
    this.group.add(this.mesh, this.lines);
    this.mode = 'solid';
  }
  colours(face, edge, cap) {
    this.baseColour = new THREE.Color(face);
    this.mat.color.set(face);
    this.edgeMat.color.set(edge);
    this.capColour = cap;
    this.edgeColour = edge;
  }
  setMode(mode) {  // solid | ghost | hidden
    this.mode = mode;
    this.group.visible = mode !== 'hidden';
    const ghost = mode === 'ghost';
    // a ghost must still read as a solid: faces see-through, edges near full,
    // so what lies behind shows through without the part dissolving (tuned on the dark sheet)
    this.mat.transparent = ghost;
    this.mat.opacity = ghost ? 0.42 : 1;
    this.mat.depthWrite = !ghost;
    // a dimmed grey sinks into a dark background: a ghost is lightened (less
    // so on paper) and shows its back faces too, so it keeps its volume
    this.mat.side = ghost ? THREE.DoubleSide : THREE.FrontSide;
    const light = document.documentElement.dataset.themeResolved === 'light';
    if (this.baseColour) this.mat.color.copy(this.baseColour).lerp(new THREE.Color('#ffffff'), ghost ? (light ? 0.15 : 0.22) : 0);
    this.edgeMat.opacity = ghost ? 0.9 : 1;
    this.mat.needsUpdate = true;
  }
  clip(planes) {
    for (const m of [this.mat, this.edgeMat]) { m.clippingPlanes = planes; m.needsUpdate = true; }
  }
  dispose() {
    this.mesh.geometry.dispose(); this.lines.geometry.dispose();
    this.mat.dispose(); this.edgeMat.dispose();
  }
}

function lineGeo(points) {
  return new THREE.BufferGeometry().setFromPoints(points);
}

function cylinderBetween(r, a, b, mat, seg = 28) {
  const dir = new THREE.Vector3().subVectors(b, a);
  const len = dir.length();
  const g = new THREE.CylinderGeometry(r, r, len, seg, 1, false);
  const m = new THREE.Mesh(g, mat);
  m.position.copy(a).addScaledVector(dir, 0.5);
  m.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir.normalize());
  return m;
}

function circle(center, axis, r, seg = 48) {
  const q = new THREE.Quaternion().setFromUnitVectors(UP, axis.clone().normalize());
  const pts = [];
  for (let i = 0; i <= seg; i++) {
    const t = (i / seg) * Math.PI * 2;
    pts.push(new THREE.Vector3(Math.cos(t) * r, Math.sin(t) * r, 0).applyQuaternion(q).add(center));
  }
  return pts;
}

export class Viewer {
  constructor(stage, { labels, gizmo, probe, scalebar, scaletext, overlay, viewLabel }) {
    this.stage = stage;
    this.labelsEl = labels;
    this.probeEl = probe;
    this.scalebar = scalebar;
    this.scaletext = scaletext;
    this.viewLabelEl = viewLabel;
    this.renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true, stencil: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.renderer.localClippingEnabled = true;
    stage.prepend(this.renderer.domElement);

    this.scene = new THREE.Scene();
    this.nav = new Nav(this.renderer.domElement, {
      pick: (ndc) => { const h = this.pickHit(ndc); return h ? h.point : null; },
      onChange: () => { this.viewChanged = true; this.overlayDirty = true; },
    });
    this.nav.onClick = (e) => this.onClick(e);

    this.hemi = new THREE.HemisphereLight(0xffffff, 0x8890a0, 1.6);
    this.scene.add(this.hemi);
    // a soft light that travels with the eye: faces shade by how they face the viewer
    this.key = new THREE.DirectionalLight(0xffffff, 1.6);
    this.scene.add(this.key);

    this.root = new THREE.Group();
    this.scene.add(this.root);
    this.layers = {};
    for (const k of ['fixed', 'obstacle', 'part', 'ref', 'prev', 'dfm', 'caps', 'faces', 'bolts', 'marks', 'grid']) {
      this.layers[k] = new THREE.Group();
      this.root.add(this.layers[k]);
    }
    this.bodies = { fixed: [], obstacle: [], part: null, ref: null, prev: null };
    this.modes = { fixed: 'solid', obstacle: 'ghost', part: 'solid', ref: 'hidden', prev: 'ghost' };
    this.show = { faces: true, bolts: true };
    this.labels = [];
    this.badBolts = {};
    this.focus = null;
    this.radius = 100;

    this.clip = new Clipper(this);
    this.clip.onChange(() => { this.overlayDirty = true; });
    this.overlay = overlay;                 // SVG layer for handles and measurements
    this.measure = new Measure(this, overlay ? overlay.querySelector('.ms') : null);
    this.measure.onChange(() => { this.overlayDirty = true; });

    this.gizmoEl = gizmo;
    if (gizmo) this.buildGizmo(gizmo);

    this.raycaster = new THREE.Raycaster();
    this.renderer.domElement.addEventListener('pointermove', (e) => { this.onMove(e); if (this.measure.on) { this.measure.move(e); this.overlayDirty = true; } });
    this.renderer.domElement.addEventListener('pointerleave', () => { if (this.probeEl) this.probeEl.innerHTML = ''; });
    this._onKey = (e) => this.onKey(e);
    window.addEventListener('keydown', this._onKey);
    this.ro = new ResizeObserver(() => this.resize());
    this.ro.observe(stage);
    this.applyTheme();
    this.resize();
    this.running = true;
    const tick = () => { if (!this.running) return; this.nav.tick(); this.render(); requestAnimationFrame(tick); };
    requestAnimationFrame(tick);
  }

  get camera() { return this.nav.camera; }

  destroy() {
    this.running = false;
    this.ro.disconnect();
    this.renderer.dispose();
    this.renderer.domElement.remove();
    if (this.gizmoEl) this.gizmoEl.innerHTML = '';
    window.removeEventListener('keydown', this._onKey);
  }

  // ---------- theme ----------

  applyTheme() {
    this.col = {
      sheet: css('--sheet'), fixed: css('--fixed'), fixedEdge: css('--fixed-edge'),
      part: css('--part'), obstacle: css('--obstacle'), partEdge: css('--part-edge'), partCap: css('--part-cap'), ref: css('--ref'),
      datum: css('--datum'), bad: css('--bad'), ok: css('--ok'), cap: css('--cap'), ink2: css('--ink-2'),
      ink3: css('--ink-3'), rule: css('--rule'), ink: css('--ink'),
      hatchFixed: css('--hatch-fixed'), hatchPart: css('--hatch-part'), hatchInk: css('--hatch-ink'),
      axX: css('--ax-x'), axY: css('--ax-y'), axZ: css('--ax-z'),
    };
    this.renderer.setClearColor(this.col.sheet, 1);
    // lighting per theme: on the slate sheet enough light that grey parts
    // read as solids without washing out the light edges; on paper as before
    const light = document.documentElement.dataset.themeResolved === 'light';
    this.hemi.color.set(light ? '#ffffff' : '#e6edf6');
    this.hemi.groundColor.set(light ? '#8890a0' : '#4a5468');
    this.hemi.intensity = light ? 1.6 : 1.3;
    this.key.intensity = light ? 1.5 : 1.4;
    for (const b of this.bodies.fixed) b.colours(this.col.fixed, this.col.fixedEdge, this.col.cap);
    for (const b of this.bodies.obstacle) b.colours(this.col.obstacle, this.col.obstacle, this.col.obstacle);
    if (this.bodies.part) this.bodies.part.colours(this.col.part, this.col.partEdge, this.col.partCap);
    if (this.bodies.ref) this.bodies.ref.colours(this.col.ref, this.col.ref, this.col.ref);
    if (this.bodies.prev) this.bodies.prev.colours(this.col.datum, this.col.datum, this.col.datum);
    for (const b of [...this.bodies.fixed, ...this.bodies.obstacle, this.bodies.part, this.bodies.ref, this.bodies.prev]) if (b) b.setMode(b.mode);  // ghosts follow the theme
    if (this.bodies.part && this.layers.dfm.children.length && !this.dfmKeepPart) this.bodies.part.group.visible = false;  // a field is shown instead
    this.applyClip();
    this.rebuildOverlays();
  }

  // ---------- content ----------

  setScene(scene, pose = 0) {
    this.sceneData = scene;
    this.pose = pose;
    for (const b of this.bodies.fixed) { this.layers.fixed.remove(b.group); b.dispose(); }
    this.bodies.fixed = [];
    const P = scene.poses[pose];
    for (const [name, m] of Object.entries(P.fixed)) {
      const g = decodeMesh(m);
      if (!g) continue;
      const b = new Body(g, 'fixed', name);
      b.colours(this.col.fixed, this.col.fixedEdge, this.col.cap);
      b.setMode(this.modes.fixed);
      this.layers.fixed.add(b.group);
      this.bodies.fixed.push(b);
    }
    // level 3: keep-out boxes the answer must stay clear of (CLEAR_PATH)
    for (const b of this.bodies.obstacle) { this.layers.obstacle.remove(b.group); b.dispose(); }
    this.bodies.obstacle = [];
    for (const o of P.obstacles || []) {
      const b = new Body(boxGeometry(o), 'obstacle', `obstacle ${o.name}`);
      b.colours(this.col.obstacle, this.col.obstacle, this.col.obstacle);
      b.setMode(this.modes.obstacle);
      this.layers.obstacle.add(b.group);
      this.bodies.obstacle.push(b);
    }
    this.computeBounds();
    this.applyClip();
    this.rebuildOverlays();
    this.applyPartMotion();
  }

  // a mechanism: the answer (modelled in pose 1) moves with the pose, as the
  // missing part moves in the task's motion (scene.part_motion, from data.py)
  applyPartMotion() {
    const pm = this.sceneData && this.sceneData.part_motion && this.sceneData.part_motion[this.pose];
    const m = new THREE.Matrix4();
    if (pm) m.fromArray(pm);
    const groups = [this.bodies.part && this.bodies.part.group, this.bodies.prev && this.bodies.prev.group, this.layers.dfm];
    for (const g of groups) {
      if (!g) continue;
      g.matrixAutoUpdate = false;
      g.matrix.copy(m);
      g.matrixWorldNeedsUpdate = true;
    }
  }

  setBody(slot, mesh) {  // slot: part | ref | prev (the previous attempt, drawn as a ghost)
    const old = this.bodies[slot];
    if (old) { this.layers[slot].remove(old.group); old.dispose(); }
    this.bodies[slot] = null;
    const g = mesh && decodeMesh(mesh);
    if (g) {
      const label = { part: 'submitted part', ref: 'reference answer', prev: 'previous attempt' }[slot];
      const b = new Body(g, slot, label);
      if (slot === 'part') b.colours(this.col.part, this.col.partEdge, this.col.partCap);
      else if (slot === 'prev') b.colours(this.col.datum, this.col.datum, this.col.datum);
      else b.colours(this.col.ref, this.col.ref, this.col.ref);
      b.setMode(this.modes[slot]);
      this.layers[slot].add(b.group);
      this.bodies[slot] = b;
      if (slot === 'part' && this.layers.dfm.children.length && !this.dfmKeepPart) b.group.visible = false;  // a field is shown instead
    }
    this.computeBounds();
    this.applyClip();
    this.applyPartMotion();
  }

  // verdict: the grader's report for the shown attempt (or null)
  setVerdict(att, tol) {
    this.verdict = att;
    this.tol = tol || {};
    this.badBolts = boltsFromReport(att, this.tol);
    this.grips = {};
    this.toolBlocked = {};
    const ta = att && (att.checks || []).find((c) => c.name === 'TOOL_ACCESS');
    const lim = (this.tol && this.tol.overlap_volume_max) || 0.01;
    const worst = {};   // per bolt, what blocks its tool path most
    for (const r of (ta && ta.details && ta.details.envelopes) || []) {
      if (r.volume > lim && !(worst[r.bolt] >= r.volume)) { worst[r.bolt] = r.volume; this.toolBlocked[r.bolt] = r.blocked_by; }
    }
    const holes = att && (att.checks || []).find((c) => c.name === 'HOLES');
    for (const r of (holes && holes.details && holes.details.holes) || []) this.grips[r.bolt] = r.grip;
    this.rebuildOverlays();
  }

  setMarks(regions) {
    for (const c of [...this.layers.marks.children]) { this.layers.marks.remove(c); c.geometry && c.geometry.dispose(); }
    this.regions = [];
    for (const r of regions || []) {
      if (this.sceneData && r.pose !== undefined && r.pose !== this.pose) continue;
      const g = decodeMesh(r.mesh);
      if (!g) continue;
      const mat = new THREE.MeshBasicMaterial({ color: this.col.bad, transparent: true, opacity: 0.78, depthTest: false });
      const mesh = new THREE.Mesh(g.faces, mat);
      mesh.renderOrder = 10;
      const lm = new THREE.LineBasicMaterial({ color: this.col.bad, depthTest: false });
      const lines = new THREE.LineSegments(g.edges, lm);
      lines.renderOrder = 11;
      this.layers.marks.add(mesh, lines);
      const c = new THREE.Vector3();
      g.faces.boundingBox.getCenter(c);
      this.regions.push({ what: r.what, volume: r.volume, at: c });
    }
    this.rebuildLabels();
  }

  // a manufacturing field drawn in place of the answer (built by dfm.js);
  // null puts the answer back. Labels: [{at: Vector3, text, bad}]
  // opts.keepPart: draw the group on top of the answer (places, sections,
  // the bed) instead of in its place; opts.hideBoltLabels: the group labels
  // the bolts itself
  setDfm(group, labels = [], opts = {}) {
    this.dfmKeepPart = !!opts.keepPart;
    this.hideBoltLabels = !!(group && opts.hideBoltLabels);
    for (const c of [...this.layers.dfm.children]) {
      this.layers.dfm.remove(c);
      c.traverse((o) => { o.geometry && o.geometry.dispose(); o.material && o.material.dispose(); });
    }
    this.dfmLabels = labels;
    this.userProbes = [];
    // objects drawn at a fixed size on screen (userData.px pixels long)
    this.screenSized = [];
    if (group) { this.layers.dfm.add(group); group.traverse((o) => { if (o.userData && o.userData.px) this.screenSized.push(o); }); }
    if (this.bodies.part) this.bodies.part.group.visible = (!group || this.dfmKeepPart) && this.modes.part !== 'hidden';
    this.applyClip();
    this.rebuildLabels();
  }

  setMode(layer, mode) {
    this.modes[layer] = mode;
    const list = layer === 'fixed' || layer === 'obstacle' ? this.bodies[layer] : [this.bodies[layer]];
    for (const b of list) if (b) b.setMode(mode);
    this.applyClip();
  }

  setShow(what, on) {
    this.show[what] = on;
    this.layers[what].visible = on;
    this.rebuildLabels();
  }

  setFocus(boltIds, regions = false) {  // highlight these bolts (hover on a check); null clears
    this.focus = boltIds && boltIds.length ? new Set(boltIds) : null;
    this.focusRegions = regions;
    this.rebuildOverlays();
  }

  // ---------- overlays: mounting faces, bolts ----------

  rebuildOverlays() {
    if (!this.sceneData) return;
    for (const k of ['faces', 'bolts']) {
      for (const c of [...this.layers[k].children]) {
        this.layers[k].remove(c);
        c.traverse((o) => { o.geometry && o.geometry.dispose(); o.material && o.material.dispose(); });
      }
    }
    const P = this.sceneData.poses[this.pose];
    const datum = new THREE.Color(this.col.datum);
    // mounting faces: outline, tint, outward normal
    for (const f of P.faces) {
      const pts = f.outline.map((p) => new THREE.Vector3(...p));
      const n = new THREE.Vector3(...f.normal);
      const lift = n.clone().multiplyScalar(0.02);
      const outline = new THREE.LineLoop(lineGeo(pts.map((p) => p.clone().add(lift))),
        new THREE.LineBasicMaterial({ color: datum }));
      const sh = new THREE.BufferGeometry().setFromPoints(pts.map((p) => p.clone().add(lift)));
      sh.setIndex([0, 1, 2, 0, 2, 3]);
      const fill = new THREE.Mesh(sh, new THREE.MeshBasicMaterial({ color: datum, transparent: true, opacity: 0.12, side: THREE.DoubleSide, depthWrite: false }));
      const o = new THREE.Vector3(...f.origin);
      const size = pts[0].distanceTo(pts[2]);
      const arrow = new THREE.ArrowHelper(n, o, Math.max(6, size * 0.14), datum, Math.max(2, size * 0.04), Math.max(1.2, size * 0.022));
      this.layers.faces.add(outline, fill, arrow);
    }
    // bolts: dash-dot centre line, hole circle on the face, and the screw
    // itself at the grip the grader measured
    const steel = new THREE.MeshStandardMaterial({ color: this.col.ink3, roughness: 0.4, metalness: 0.5, transparent: true, opacity: 0.55, depthWrite: false });
    const badSteel = new THREE.MeshBasicMaterial({ color: this.col.bad, transparent: true, opacity: 0.75, depthWrite: false });
    for (const b of P.bolts) {
      const p = new THREE.Vector3(...b.position);
      const a = new THREE.Vector3(...b.axis).normalize();
      const bad = !!this.badBolts[b.id];
      const dim = this.focus && !this.focus.has(b.id);
      const colour = bad ? this.col.bad : this.col.datum;
      const grip = this.grips && this.grips[b.id];
      const reach = (grip || b.grip_range[0]) + b.head_height + 6;
      const cl = new THREE.Line(lineGeo([p.clone().addScaledVector(a, -10), p.clone().addScaledVector(a, reach)]),
        new THREE.LineDashedMaterial({ color: colour, dashSize: 3, gapSize: 1.2, transparent: true, opacity: dim ? 0.25 : 1 }));
      cl.computeLineDistances();
      const dia = b.clearance_diameter_range[0];
      const ring = new THREE.Line(lineGeo(circle(p.clone().addScaledVector(a, 0.03), a, dia / 2)),
        new THREE.LineBasicMaterial({ color: colour, transparent: true, opacity: dim ? 0.25 : 1 }));
      const headRing = new THREE.Line(lineGeo(circle(p.clone().addScaledVector(a, 0.03), a, b.head_diameter / 2, 40)),
        new THREE.LineDashedMaterial({ color: colour, dashSize: 0.8, gapSize: 0.8, transparent: true, opacity: dim ? 0.15 : 0.6 }));
      headRing.computeLineDistances();
      this.layers.bolts.add(cl, ring, headRing);
      if (grip) {
        const d = parseFloat(String(b.size).replace(/[^0-9.]/g, '')) || dia * 0.85;
        const mat = bad ? badSteel : steel;
        const shank = cylinderBetween(d / 2, p.clone().addScaledVector(a, -Math.min(6, d * 1.5)), p.clone().addScaledVector(a, grip), mat, 20);
        const head = cylinderBetween(b.head_diameter / 2, p.clone().addScaledVector(a, grip), p.clone().addScaledVector(a, grip + b.head_height), mat, 32);
        shank.renderOrder = head.renderOrder = 3;
        if (!dim) this.layers.bolts.add(shank, head);
      }
      // level 3 (TOOL_ACCESS): the straight path a hex key needs, as the
      // grader builds it -- a cylinder from the top of the head, at the
      // measured grip, outward along the axis: tool length long, head
      // diameter (+ the allowance) wide. Red when the grader found it blocked.
      const tol = (this.sceneData.task && this.sceneData.task.tolerances) || {};
      if (tol.tool_length) {
        const g0 = (grip || b.grip_range[0]) + b.head_height;
        const r = (b.head_diameter + (tol.tool_diameter_extra || 0)) / 2;
        const blocked = (this.toolBlocked || {})[b.id];
        // a blocked path usually runs through solid material: drawn through it, so it shows
        const mat = new THREE.LineDashedMaterial({ color: blocked ? this.col.bad : this.col.ink3, dashSize: 1.6, gapSize: 1.2, transparent: true, opacity: dim ? 0.2 : (blocked ? 1 : 0.7), depthTest: !blocked });
        const c0 = p.clone().addScaledVector(a, g0), c1 = p.clone().addScaledVector(a, g0 + tol.tool_length);
        const q = new THREE.Quaternion().setFromUnitVectors(UP, a);
        const side = [];
        for (let i = 0; i < 4; i++) {
          const t = (i / 4) * Math.PI * 2, off = new THREE.Vector3(Math.cos(t) * r, Math.sin(t) * r, 0).applyQuaternion(q);
          side.push(c0.clone().add(off), c1.clone().add(off));
        }
        for (const L of [new THREE.Line(lineGeo(circle(c1, a, r, 40)), mat), new THREE.LineSegments(lineGeo(side), mat)]) {
          L.computeLineDistances();
          L.userData.envelope = true;
          if (blocked) L.renderOrder = 12;
          this.layers.bolts.add(L);
        }
      }
    }
    this.applyClip();
    this.rebuildLabels();
  }

  rebuildLabels() {
    this.labelsEl.innerHTML = '';
    this.labels = [];
    if (!this.sceneData) return;
    const P = this.sceneData.poses[this.pose];
    if (this.show.bolts && !this.hideBoltLabels && !this.noBoltLabels) {
      for (const b of P.bolts) {
        const bad = this.badBolts[b.id];
        if (this.focus && !this.focus.has(b.id) && !bad) continue;
        const el = document.createElement('div');
        el.className = 'blabel' + (bad ? ' bad' : '');
        el.textContent = b.id;
        const nBad = Object.keys(this.badBolts).length;
        const wordy = this.focus ? this.focus.has(b.id) : nBad <= 2;
        if (bad && wordy) {
          const s = document.createElement('span');
          const t = bad[0].replace(/^[A-Z_]+: /, '');
          s.textContent = t.length > 44 ? t.slice(0, 43) + '…' : t;
          el.appendChild(s);
        } else if (bad) el.textContent = b.id + ' ✕';
        const grip = (this.grips && this.grips[b.id]) || 0;
        const at = new THREE.Vector3(...b.position).addScaledVector(new THREE.Vector3(...b.axis), grip + b.head_height + 2);
        this.labelsEl.appendChild(el);
        this.labels.push({ el, at });
      }
    }
    for (const d of [...(this.dfmLabels || []), ...(this.userProbes || [])]) {
      const el = document.createElement('div');
      el.className = d.probe ? 'ptag' + (d.bad ? ' bad' : '') + (d.user ? ' user' : '') : 'blabel dfm' + (d.bad ? ' bad' : '') + (d.small ? ' small' : '');
      el.textContent = d.text;
      this.labelsEl.appendChild(el);
      this.labels.push({ el, at: d.at });
    }
    const wordy = this.focusRegions || (this.regions || []).length <= 2;
    for (const r of this.regions || []) {
      const el = document.createElement('div');
      el.className = 'blabel region';
      el.textContent = wordy ? `${r.what}: ${r.volume.toFixed(1)} mm³` : `${r.volume.toFixed(0)} mm³`;
      el.title = r.what;
      this.labelsEl.appendChild(el);
      this.labels.push({ el, at: r.at });
    }
  }

  placeLabels() {
    const w = this.width, h = this.height;
    const v = new THREE.Vector3();
    const placed = [];  // a label that would cover an earlier one is left out (the first listed wins)
    for (const l of this.labels) {
      v.copy(l.at).project(this.camera);
      const clipped = !this.keptByClip(l.at);
      let vis = v.z > -1 && v.z < 1 && !clipped;
      const x = ((v.x + 1) / 2) * w, y = ((1 - v.y) / 2) * h;
      if (vis) {
        const bw = l.el.offsetWidth || 60, bh = l.el.offsetHeight || 18;
        const r = { x0: x - 2, y0: y - bh / 2 - 2, x1: x + bw + 2, y1: y + bh / 2 + 2 };
        if (placed.some((q) => r.x0 < q.x1 && q.x0 < r.x1 && r.y0 < q.y1 && q.y0 < r.y1)) vis = false;
        else placed.push(r);
      }
      l.el.style.display = vis ? '' : 'none';
      l.el.style.left = x + 'px';
      l.el.style.top = y + 'px';
    }
  }

  // ---------- camera ----------

  computeBounds() {
    const box = new THREE.Box3();
    for (const b of [...this.bodies.fixed, ...this.bodies.obstacle, this.bodies.part, this.bodies.ref, this.bodies.prev]) {
      if (b && b.mesh.geometry.boundingBox) box.union(b.mesh.geometry.boundingBox);
    }
    if (box.isEmpty()) box.set(new THREE.Vector3(-50, -50, -50), new THREE.Vector3(50, 50, 50));
    this.box = box;
    this.center = box.getCenter(new THREE.Vector3());
    this.radius = box.getSize(new THREE.Vector3()).length() / 2;
    if (!this.fitted) { this.nav.fit(box, { animate: false }); this.fitted = true; }
  }

  // the first view of a task with an answer: the answer and its mounts,
  // from the three-quarter direction (four corners, above and below) where
  // the fewest of their points hide behind a fixed part or an obstacle.
  // Ties keep the usual south-west view from above.
  frameAnswer() {
    const part = this.bodies.part;
    if (!part || !part.mesh.geometry.boundingBox) return false;
    const pos = part.mesh.geometry.attributes.position;
    const pts = [];
    const step = Math.max(1, Math.floor(pos.count / 80));
    for (let i = 0; i < pos.count; i += step) pts.push(new THREE.Vector3().fromBufferAttribute(pos, i));
    const box = part.mesh.geometry.boundingBox.clone();
    const P = this.sceneData && this.sceneData.poses[this.pose];
    for (const f of (P && P.faces) || []) { const o = new THREE.Vector3(...f.origin); pts.push(o, o, o); box.expandByPoint(o); }   // a mount weighs as much as a few points of the part
    const ctr = box.getCenter(new THREE.Vector3()), R = Math.max(1, box.getSize(new THREE.Vector3()).length() / 2);
    const occ = [...this.bodies.fixed, ...this.bodies.obstacle].filter((b) => b && b.mode !== 'hidden').map((b) => b.mesh);
    const ray = new THREE.Raycaster();
    let best = null;
    for (const pitch of [0.6, -0.6]) {
      for (const yaw of [-2.36, -0.79, 0.79, 2.36]) {   // south-west first
        const q = uprightQ(yaw, pitch), dir = new THREE.Vector3(0, 0, 1).applyQuaternion(q);
        const eye = ctr.clone().addScaledVector(dir, R * 4);
        let hidden = 0;
        for (const p of pts) {
          const d = p.clone().sub(eye), len = d.length();
          ray.set(eye, d.normalize()); ray.far = len - 0.5;
          if (occ.length && ray.intersectObjects(occ, false).length) hidden++;
        }
        if (!best || hidden < best.hidden) best = { hidden, q };
      }
    }
    this.nav.q.copy(best.q);
    this.nav.view = null;
    this.nav.fit(box, { animate: false, keepDirection: true });
    this.fitted = true;
    return true;
  }

  // named views: iso (home) frames the model from the south-west; the axis
  // views keep the distance and go parallel
  view(name, animate = true) {
    if (!name || name === 'iso' || name === 'home') { this.nav.fit(this.box, { animate }); return; }
    if (name === 'side') name = 'right';
    if (VIEWS[name]) this.nav.snap(name, animate);
  }

  fit() { this.nav.fit(this.box); }

  resize() {
    const r = this.stage.getBoundingClientRect();
    this.width = Math.max(1, r.width); this.height = Math.max(1, r.height);
    this.renderer.setSize(this.width, this.height, false);
    this.nav.resize(this.width, this.height);
    if (this.overlay) this.overlay.setAttribute('viewBox', `0 0 ${this.width} ${this.height}`);
    this.overlayDirty = true;
  }

  // ---------- keyboard: views (Blender-like) ----------

  onKey(e) {
    const tag = (e.target && e.target.tagName) || '';
    if (/INPUT|TEXTAREA|SELECT/.test(tag) || e.altKey || e.metaKey) return;
    const code = e.code || '', key = e.key;
    const num = code.startsWith('Numpad');
    const opp = e.ctrlKey;
    const go = (f) => { e.preventDefault(); f(); };
    if (num && code === 'Numpad4') return go(() => this.nav.turn(Math.PI / 12));
    if (num && code === 'Numpad6') return go(() => this.nav.turn(-Math.PI / 12));
    if (num && code === 'Numpad8') return go(() => this.nav.turn(0, Math.PI / 12));
    if (num && code === 'Numpad2') return go(() => this.nav.turn(0, -Math.PI / 12));
    if (e.ctrlKey && !['1', '3', '7'].includes(key)) return;
    if (key === '1') return go(() => this.nav.snap(opp ? 'back' : 'front'));
    if (key === '3') return go(() => this.nav.snap(opp ? 'left' : 'right'));
    if (key === '7') return go(() => this.nav.snap(opp ? 'bottom' : 'top'));
    if (key === '5') return go(() => this.nav.toggleProjection());
    if (key === '9') return go(() => this.nav.opposite());
    if (key === '0' || key === 'Home') return go(() => this.fit());
    if (key === 'l' || key === 'L') return go(() => this.nav.level());   // upright again after free orbit
    if (key === 'Escape') {
      if (this.measure.escape()) return;
      if (this.clip.picking) { this.clip.picking = false; this.clip.changed(); }
    }
  }

  // ---------- picking ----------

  keptByClip(p) {
    for (const pl of this.clip.planes) if (pl.n.dot(p) > pl.d + 1e-6) return false;
    return true;
  }

  // the nearest visible surface point under a screen position, not cut away
  pickHit(ndc) {
    this.raycaster.setFromCamera(ndc, this.camera);
    const targets = [...this.bodies.fixed, this.bodies.part, this.bodies.ref]
      .filter((b) => b && b.mode !== 'hidden' && b.group.visible).map((b) => b.mesh);
    const hit = this.raycaster.intersectObjects(targets, false).find((h) => this.keptByClip(h.point));
    if (!hit) return null;
    return { point: hit.point, normal: hit.face ? hit.face.normal.clone() : null, body: hit.object.userData.body };
  }

  // a click (no drag): measuring, or laying a section plane on a face
  onClick(e) {
    if (this.measure.click(e)) return;
    // a click on a coloured field: tag the value there
    if (this.onFieldClick && this.layers.dfm.children.length && !this.clip.picking) {
      this.raycaster.setFromCamera(this.nav.ndc(e), this.camera);
      const meshes = []; this.layers.dfm.traverse((o) => { if (o.isMesh && o.userData.field) meshes.push(o); });
      const hit = this.raycaster.intersectObjects(meshes, false).find((h) => this.keptByClip(h.point));
      if (hit) { this.onFieldClick(hit); return; }
    }
    if (this.clip.picking) {
      const h = this.pickHit(this.nav.ndc(e));
      if (h && h.normal) {
        this.clip.picking = false;
        this.clip.addFace(h.point, h.normal);
      }
    }
  }

  // ---------- clipping ----------

  applyClip() {
    const planes = this.clip.three();
    for (const b of [...this.bodies.fixed, ...this.bodies.obstacle, this.bodies.part, this.bodies.ref, this.bodies.prev]) if (b) b.clip(planes);
    for (const k of ['faces', 'bolts', 'marks', 'dfm']) {
      this.layers[k].traverse((o) => { if (o.material) { o.material.clippingPlanes = planes; o.material.needsUpdate = true; } });
    }
    if (!this.col) return;
    const bodies = [...this.bodies.fixed.map((b) => [b, 'fixed']), ...this.bodies.obstacle.map((b) => [b, 'obstacle']), [this.bodies.part, 'part'], [this.bodies.ref, 'ref']]
      .filter(([b]) => b && b.mode !== 'hidden' && b.group.visible);
    this.clip.buildCaps(this.layers.caps, bodies, this.col, this.radius || 100);
  }

  // ---------- probe: what is under the pointer, in mm ----------

  onMove(e) {
    if (!this.probeEl) return;
    const h = this.pickHit(this.nav.ndc(e));
    if (!h) { this.probeEl.innerHTML = ''; return; }
    const b = h.body;
    const who = b.role === 'fixed' ? `fixed ${b.name}` : b.name;
    const f = (v) => (Math.abs(v) < 0.005 ? 0 : v).toFixed(2);
    this.probeEl.innerHTML = `<b>${who}</b> &nbsp;x ${f(h.point.x)} &nbsp;y ${f(h.point.y)} &nbsp;z ${f(h.point.z)} mm`;
  }

  // ---------- drawing ----------

  updateScale() {
    if (!this.scalebar) return;
    const mmPerPx = this.nav.mmPerPx();
    const target = 110 * mmPerPx;
    const k = Math.pow(10, Math.floor(Math.log10(target)));
    const nice = [1, 2, 5, 10].map((m) => m * k).reduce((a, b) => (Math.abs(b - target) < Math.abs(a - target) ? b : a));
    const px = nice / mmPerPx;
    if (this._lastScale !== nice || Math.abs((this._lastPx || 0) - px) > 0.5 || this._lastPar !== this.nav.parallel) {
      this._lastPar = this.nav.parallel;
      this.scalebar.style.width = px.toFixed(1) + 'px';
      this.scaletext.textContent = `${nice >= 1 ? nice : nice.toFixed(1)} mm${this.nav.parallel ? '' : ' at the pivot'}`;
      this._lastScale = nice; this._lastPx = px;
    }
  }

  // ---------- navigation ball ----------
  // A centre with short stubs to the positive X, Y, Z ends (solid, lettered)
  // and paler balls for the negative ends (lettered on hover). The ends turn
  // with the scene; clicking one looks from that side, clicking it again
  // from the opposite side; dragging the ball orbits like dragging the
  // drawing. (After the maintainer's model viewer; drawn for JointBench.)

  buildGizmo(el) {
    el.innerHTML = '';
    const NS = 'http://www.w3.org/2000/svg';
    const svg = document.createElementNS(NS, 'svg');
    svg.setAttribute('viewBox', '-50 -50 100 100');
    svg.setAttribute('role', 'group');
    svg.setAttribute('aria-label', 'View orientation: click an axis end to look from that side, again for the opposite side; drag to orbit. Keys 1, 3, 7 (Ctrl: opposite), 5 projection, 9 other side, 0 home.');
    el.appendChild(svg);
    const bg = document.createElementNS(NS, 'circle');
    bg.setAttribute('r', 48); bg.setAttribute('class', 'gz-bg');
    svg.appendChild(bg);
    this.gz = { svg, items: [] };
    const ends = [['X', [1, 0, 0], 'axX', 'right', 'left'], ['Y', [0, 1, 0], 'axY', 'back', 'front'], ['Z', [0, 0, 1], 'axZ', 'top', 'bottom']];
    for (const [name, v, col, posView, negView] of ends) {
      for (const sign of [1, -1]) {
        const view = sign > 0 ? posView : negView;
        const g = document.createElementNS(NS, 'g');
        g.setAttribute('class', `gz-end ${sign > 0 ? 'pos' : 'neg'}`);
        g.setAttribute('tabindex', '0');
        g.setAttribute('aria-label', VIEWS[view].name);
        const title = document.createElementNS(NS, 'title'); title.textContent = VIEWS[view].name; g.appendChild(title);
        let line = null;
        if (sign > 0) { line = document.createElementNS(NS, 'line'); line.setAttribute('class', 'gz-line'); svg.appendChild(line); }
        const c = document.createElementNS(NS, 'circle');
        c.setAttribute('r', sign > 0 ? 8 : 7);
        g.appendChild(c);
        const t = document.createElementNS(NS, 'text');
        t.setAttribute('text-anchor', 'middle'); t.setAttribute('dy', '0.35em');
        t.textContent = sign > 0 ? name : `−${name}`;
        g.appendChild(t);
        svg.appendChild(g);
        const snap = () => this.nav.snap(view);
        g.dataset.view = view;
        g.addEventListener('keydown', (e) => { if (e.key === 'Enter' || e.key === ' ') { e.preventDefault(); snap(); } });
        this.gz.items.push({ g, c, t, line, dir: new THREE.Vector3(...v).multiplyScalar(sign), col, view, sign });
      }
    }
    // press on the ball: a release without moving is a click on the axis end
    // that was pressed (snap to that view, or to the opposite one if it is
    // already shown); moving more than 3 px orbits instead. The end is
    // remembered on press, because once the pointer is captured for the
    // drag, the release is reported on the ball, not on the end.
    let last = null, pressed = null;
    svg.addEventListener('pointerdown', (e) => {
      const end = e.target.closest && e.target.closest('.gz-end');
      pressed = end ? end.dataset.view : null;
      last = [e.clientX, e.clientY, e.clientX, e.clientY];
      this.gz.dragged = false;
      e.preventDefault();
    });
    svg.addEventListener('pointermove', (e) => {
      if (!last) return;
      if (!this.gz.dragged && Math.hypot(e.clientX - last[2], e.clientY - last[3]) < 3) return;
      if (!this.gz.dragged) { try { svg.setPointerCapture(e.pointerId); } catch { /* not a live pointer */ } }
      this.gz.dragged = true;
      this.nav.orbit(e.clientX - last[0], e.clientY - last[1]);
      last[0] = e.clientX; last[1] = e.clientY;
    });
    const release = (e) => {
      if (last && !this.gz.dragged && pressed) this.nav.snap(pressed);
      last = null; pressed = null; this.gz.dragged = false;
      try { svg.releasePointerCapture(e.pointerId); } catch { /* not captured */ }
    };
    svg.addEventListener('pointerup', release);
    svg.addEventListener('pointercancel', () => { last = null; pressed = null; this.gz.dragged = false; });
  }

  renderGizmo() {
    if (!this.gz || !this.col) return;
    const inv = this.camera.quaternion.clone().invert();
    const R = 34;
    const placed = this.gz.items.map((it) => {
      const v = it.dir.clone().applyQuaternion(inv);
      return { it, x: v.x * R, y: -v.y * R, z: v.z };
    });
    placed.sort((a, b) => a.z - b.z);
    for (const p of placed) {
      const { it } = p;
      const col = this.col[it.col];
      it.g.setAttribute('transform', `translate(${p.x.toFixed(2)} ${p.y.toFixed(2)})`);
      it.c.setAttribute('fill', it.sign > 0 ? col : `color-mix(in srgb, ${col} 38%, var(--sheet))`);
      it.c.setAttribute('stroke', it.sign > 0 ? 'none' : col);
      it.g.classList.toggle('current', this.nav.view === it.view);
      if (it.line) {
        it.line.setAttribute('x2', p.x.toFixed(2)); it.line.setAttribute('y2', p.y.toFixed(2));
        it.line.setAttribute('stroke', col);
      }
      it.g.parentNode.appendChild(it.g);
    }
  }

  // ---------- clip handles: a dot on each plane, an arrow to the side cut away ----------

  drawHandles() {
    const svg = this.overlay && this.overlay.querySelector('.hd');
    if (!svg) return;
    svg.replaceChildren();
    const NS = 'http://www.w3.org/2000/svg';
    const mk = (tag, a, parent = svg) => { const e = document.createElementNS(NS, tag); for (const [k, v] of Object.entries(a)) e.setAttribute(k, v); parent.appendChild(e); return e; };
    const scr = (p) => { const v = p.clone().project(this.camera); return [((v.x + 1) / 2) * this.width, ((1 - v.y) / 2) * this.height, v.z]; };
    const planes = this.clip.planes;
    const box = planes.filter((p) => p.kind === 'box');
    if (box.length === 6) {  // the box's twelve edges
      const g = (a, s) => box.find((p) => p.axis === a && p.side === s).d * s;
      const xs = [g('x', -1), g('x', 1)], ys = [g('y', -1), g('y', 1)], zs = [g('z', -1), g('z', 1)];
      const C = (i, j, k) => new THREE.Vector3(xs[i], ys[j], zs[k]);
      const E = [[[0, 0, 0], [1, 0, 0]], [[0, 1, 0], [1, 1, 0]], [[0, 0, 1], [1, 0, 1]], [[0, 1, 1], [1, 1, 1]],
        [[0, 0, 0], [0, 1, 0]], [[1, 0, 0], [1, 1, 0]], [[0, 0, 1], [0, 1, 1]], [[1, 0, 1], [1, 1, 1]],
        [[0, 0, 0], [0, 0, 1]], [[1, 0, 0], [1, 0, 1]], [[0, 1, 0], [0, 1, 1]], [[1, 1, 0], [1, 1, 1]]];
      for (const [a, b] of E) {
        const [ax, ay] = scr(C(...a)), [bx, by] = scr(C(...b));
        mk('line', { x1: ax, y1: ay, x2: bx, y2: by, class: 'hd-box' });
      }
      // keep each face's handle at the centre of that face
      const cx = (xs[0] + xs[1]) / 2, cy = (ys[0] + ys[1]) / 2, cz = (zs[0] + zs[1]) / 2;
      for (const p of box) { p.anchor.set(cx, cy, cz); p.anchor.setComponent('xyz'.indexOf(p.axis), p.d * p.side); }
    }
    for (const p of planes) {
      if (p.kind !== 'box') {
        const poly = this.clip.crossing(p).map(scr);
        if (poly.length > 2) mk('polygon', { points: poly.map(([x, y]) => `${x},${y}`).join(' '), class: 'hd-plane' });
      }
      const [x, y, z] = scr(p.anchor);
      if (z >= 1) continue;
      // screen direction of the normal (the side being cut away)
      const k = this.radius * 0.1;
      const [x2, y2] = scr(p.anchor.clone().addScaledVector(p.n, k));
      let dx = x2 - x, dy = y2 - y; const L = Math.hypot(dx, dy) || 1;
      const ux = dx / L, uy = dy / L;
      const colour = this.col[{ x: 'axX', y: 'axY', z: 'axZ' }[p.axis] || 'datum'] || this.col.datum;
      const g = mk('g', { class: `hd-h ${this.dragging && this.dragging.id === p.id ? 'on' : ''}`, 'data-id': p.id });
      if (p.kind !== 'box') {
        mk('line', { x1: x, y1: y, x2: x + ux * 26, y2: y + uy * 26, class: 'hd-arrow', stroke: colour }, g);
        mk('circle', { cx: x + ux * 26, cy: y + uy * 26, r: 2.5, fill: colour, class: 'hd-tip' }, g);
      }
      mk('circle', { cx: x, cy: y, r: 13, class: 'hd-hit' }, g);
      mk('circle', { cx: x, cy: y, r: p.kind === 'box' ? 5 : 6, fill: colour, class: 'hd-dot' }, g);
      g.addEventListener('pointerdown', (e) => this.startHandleDrag(e, p, L / k, ux, uy));
      if (this.dragging && this.dragging.id === p.id) {
        const t = this.clip.readout(p);
        const bw = t.length * 6.4 + 14;
        const bub = mk('g', { class: 'hd-read', transform: `translate(${x + 14} ${y - 30})` });
        mk('rect', { width: bw, height: 22, rx: 6 }, bub);
        mk('text', { x: 7, y: 11, dy: '0.35em' }, bub).textContent = t;
      }
    }
  }

  // drag a handle along its plane's normal: screen movement along the
  // normal's on-screen direction (vertical when the normal faces the eye)
  startHandleDrag(e, p, pxPerMm, ux, uy) {
    e.stopPropagation(); e.preventDefault();
    const facing = pxPerMm < 0.05;
    let x0 = e.clientX, y0 = e.clientY;
    this.dragging = { id: p.id };
    const move = (ev) => {
      const dx = ev.clientX - x0, dy = ev.clientY - y0;
      const along = facing ? -dy : dx * ux + dy * uy;
      const per = facing ? this.nav.mmPerPx() : 1 / pxPerMm;
      let mm = along * per;
      const step = ev.ctrlKey ? 1 : 0.1;
      mm = Math.round(mm / step) * step;
      if (mm !== 0) { this.clip.move(p.id, mm); x0 = ev.clientX; y0 = ev.clientY; }
      this.overlayDirty = true;
    };
    const up = () => { this.dragging = null; this.overlayDirty = true; window.removeEventListener('pointermove', move); window.removeEventListener('pointerup', up); this.clip.changed(); };
    window.addEventListener('pointermove', move);
    window.addEventListener('pointerup', up);
  }

  // a value tag the reader added by clicking the field
  addProbe(at, text) {
    this.userProbes = [...(this.userProbes || []).slice(-5), { at, text, probe: true, user: true }];
    this.rebuildLabels();
  }

  render() {
    this.key.position.copy(this.camera.position);   // the light travels with the eye
    if (this.screenSized && this.screenSized.length) { const k = this.nav.mmPerPx(); for (const o of this.screenSized) o.scale.setScalar(k * o.userData.px); }
    this.renderer.render(this.scene, this.camera);
    // timing marks, read by the speed measurements (performance.getEntriesByType('mark'))
    if (!this.markedFirst && this.bodies.fixed.length) { this.markedFirst = true; performance.mark('jb-first-3d'); }
    this.placeLabels();
    this.updateScale();
    this.renderGizmo();
    // handles and measurements are redrawn only when something moved
    if (this.overlayDirty) {
      this.overlayDirty = false;
      this.drawHandles();
      if (this.measure.svg) this.measure.draw();
    }
    if (this.viewLabelEl && this.viewChanged) { this.viewLabelEl.textContent = this.nav.label(); this.viewChanged = false; }
  }
}
