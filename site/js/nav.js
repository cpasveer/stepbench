// Navigation: how the camera moves. The view is four numbers -- a target
// point, a heading (yaw) and elevation (pitch) of the eye around it, and a
// distance -- plus the choice of projection. Every camera move changes those
// numbers; the camera is rebuilt from them each frame. Z is up.
//
// Behaviour (after the maintainer's own model viewer, re-implemented here, except
// that orbit is free: his is a building viewer and keeps the world upright;
// mechanical parts are turned every way):
//   left drag      free orbit about the target (trackball): the view turns
//                  about the screen's own axes, 0.005 rad per pixel, no pole,
//                  no clamp, the part can be turned upside down
//   Alt + drag     roll about the line of sight (sideways motion)
//   level()        back to upright (Z up on screen), same line of sight
//   right / Shift  pan; the point under the cursor follows the cursor
//   wheel          zoom toward the point under the cursor; trackpad pinch
//                  (ctrl + wheel, or Safari's gesture events) the same;
//                  the point is picked once per gesture and the zoom is an
//                  exact scaling about it, so in-then-out returns the view
//   double click   make the clicked point the new pivot (glides there)
//   axis views     always parallel; orbiting away returns to perspective
//                  unless parallel was chosen with the projection button
//   moves          glide with cubic ease-in-out; rotation by heading and
//                  elevation, distance geometric, so a jump to the opposite
//                  side swings round the part rather than through it

import * as THREE from '../vendor/three/three.module.js';

const FOV = 50;                    // degrees, vertical, perspective
const ORBIT = 0.005;               // rad per pixel
const WHEEL = 0.0015;              // zoom factor exp(deltaY * WHEEL)
const GLIDE = 320;                 // ms for views, framing, turning round
const HOME_DIR = new THREE.Vector3(-0.6, -0.65, 0.47).normalize();  // from the south-west, a little above

export const VIEWS = {
  front: { yaw: -Math.PI / 2, pitch: 0, name: 'Front (−Y)', axis: [0, -1, 0] },
  back: { yaw: Math.PI / 2, pitch: 0, name: 'Back (+Y)', axis: [0, 1, 0] },
  right: { yaw: 0, pitch: 0, name: 'Right (+X)', axis: [1, 0, 0] },
  left: { yaw: Math.PI, pitch: 0, name: 'Left (−X)', axis: [-1, 0, 0] },
  top: { yaw: -Math.PI / 2, pitch: Math.PI / 2, name: 'Top (+Z)', axis: [0, 0, 1] },
  bottom: { yaw: -Math.PI / 2, pitch: -Math.PI / 2, name: 'Bottom (−Z)', axis: [0, 0, -1] },
};
const OPPOSITE = { front: 'back', back: 'front', right: 'left', left: 'right', top: 'bottom', bottom: 'top' };

const ease = (k) => (k < 0.5 ? 4 * k * k * k : 1 - Math.pow(-2 * k + 2, 3) / 2);

const X = new THREE.Vector3(1, 0, 0), Y = new THREE.Vector3(0, 1, 0), Z = new THREE.Vector3(0, 0, 1);

// the orientation of an upright camera looking from heading yaw and elevation
// pitch (Z up on screen; at the poles the heading sets which way is up)
export function uprightQ(yaw, pitch) {
  const cp = Math.cos(pitch), sp = Math.sin(pitch), cy = Math.cos(yaw), sy = Math.sin(yaw);
  const dir = new THREE.Vector3(cp * cy, cp * sy, sp);
  const right = new THREE.Vector3(-sy, cy, 0);
  const up = new THREE.Vector3().crossVectors(dir, right);
  return new THREE.Quaternion().setFromRotationMatrix(new THREE.Matrix4().makeBasis(right, up, dir));
}

export class Nav {
  constructor(canvas, { pick, onChange } = {}) {
    this.canvas = canvas;
    this.pick = pick || (() => null);         // (ndc) -> world point or null
    this.onChange = onChange || (() => {});
    this.persp = new THREE.PerspectiveCamera(FOV, 1, 0.5, 1e6);
    this.ortho = new THREE.OrthographicCamera(-1, 1, 1, -1, -1e6, 1e6);
    for (const c of [this.persp, this.ortho]) c.up.set(0, 0, 1);
    this.target = new THREE.Vector3();
    this.q = uprightQ(0, 0.4); this.dist = 300;   // camera orientation: any, no pole
    this.parallel = false;       // what is drawn now
    this.parallelChosen = false; // the user asked for parallel with the button
    this.view = null;            // name of the axis view, or null (free view)
    this.width = 1; this.height = 1;
    this.anim = null;
    this.tool = null;            // 'zoom' | 'pan' while a tool button drags
    this.attach();
    this.apply();
  }

  get camera() { return this.parallel ? this.ortho : this.persp; }

  // eye direction from target, and the camera's own right and up
  basis() {
    return { dir: Z.clone().applyQuaternion(this.q), right: X.clone().applyQuaternion(this.q), up: Y.clone().applyQuaternion(this.q) };
  }

  // heading and elevation of the line of sight; setting them gives an upright view
  get yaw() { const d = Z.clone().applyQuaternion(this.q); return Math.abs(d.z) > 0.999999 ? Math.atan2(-X.clone().applyQuaternion(this.q).x, X.clone().applyQuaternion(this.q).y) : Math.atan2(d.y, d.x); }
  get pitch() { const d = Z.clone().applyQuaternion(this.q); return Math.asin(Math.max(-1, Math.min(1, d.z))); }
  set yaw(v) { this.q = uprightQ(v, this.pitch); }
  set pitch(v) { this.q = uprightQ(this.yaw, v); }

  apply() {
    const { dir } = this.basis();
    const m = new THREE.Matrix4().makeRotationFromQuaternion(this.q);
    const pos = this.target.clone().addScaledVector(dir, this.dist);
    const aspect = this.width / Math.max(1, this.height);
    this.persp.aspect = aspect;
    this.persp.near = Math.max(0.05, this.dist / 2000); this.persp.far = this.dist * 50 + 1e4;
    const hh = this.dist * Math.tan((FOV * Math.PI) / 360);  // same size on screen at the target plane
    Object.assign(this.ortho, { left: -hh * aspect, right: hh * aspect, top: hh, bottom: -hh });
    for (const c of [this.persp, this.ortho]) {
      c.position.copy(pos);
      c.quaternion.setFromRotationMatrix(m);
      c.updateProjectionMatrix();
      c.updateMatrixWorld();
    }
    this.onChange();
  }

  resize(w, h) { this.width = w; this.height = h; this.apply(); }

  // millimetres per screen pixel at the target plane (scale bar, panning)
  mmPerPx() { return (2 * this.dist * Math.tan((FOV * Math.PI) / 360)) / Math.max(1, this.height); }

  label() {
    const v = this.view && VIEWS[this.view];
    return `${v ? v.name : 'User'} · ${this.parallel ? 'parallel' : 'perspective'}`;
  }

  // ---- moves ----

  freeView() {  // the user moved away from an axis view
    if (this.view) { this.view = null; if (!this.parallelChosen) this.parallel = false; }
  }

  // trackball: turn about the camera's own up axis (sideways drag) and right
  // axis (up-down drag); the pivot stays where it is on screen
  orbit(dx, dy) {
    this.stop();
    this.freeView();
    const r = new THREE.Quaternion().setFromAxisAngle(Y, -dx * ORBIT)
      .multiply(new THREE.Quaternion().setFromAxisAngle(X, -dy * ORBIT));
    this.q.multiply(r).normalize();
    this.apply();
  }

  // deliberate roll about the line of sight
  roll(dx) {
    this.stop();
    this.freeView();
    this.q.multiply(new THREE.Quaternion().setFromAxisAngle(Z, dx * ORBIT)).normalize();
    this.apply();
  }

  // back to an upright view along the same line of sight
  level(ms = GLIDE) {
    this.glide({ target: this.target.clone(), q: uprightQ(this.yaw, this.pitch), dist: this.dist }, ms);
  }

  pan(dx, dy) {
    this.stop();
    const { right, up } = this.basis();
    const k = this.mmPerPx();
    this.target.addScaledVector(right, -dx * k).addScaledVector(up, dy * k);
    this.apply();
  }

  // scale the whole view about a world point (zoom toward the cursor)
  zoomAbout(factor, point) {
    this.stop();
    factor = Math.max(0.05, factor);
    // target' = point + (target - point) * factor: the offset is taken before
    // the target moves (taking it after made every gesture recentre the view
    // on the cursor point, a sideways jump). Zooming in and back out by the
    // same amount about the same point returns the identical view.
    if (point) {
      const off = this.target.clone().sub(point);
      this.target.copy(point).addScaledVector(off, factor);
    }
    this.dist *= factor;
    this.apply();
  }

  // glide to a view {target, q | yaw+pitch, dist}: orientation by the shortest
  // rotation, distance geometric
  glide(to, ms = GLIDE) {
    const from = { target: this.target.clone(), q: this.q.clone(), dist: this.dist };
    const q1 = to.q ? to.q.clone() : uprightQ(to.yaw, to.pitch);
    const t0 = performance.now();
    this.anim = () => {
      const k = Math.min(1, (performance.now() - t0) / ms), e = ease(k);
      this.target.lerpVectors(from.target, to.target, e);
      this.q.slerpQuaternions(from.q, q1, e);
      this.dist = from.dist * Math.pow(to.dist / from.dist, e);
      this.apply();
      if (k >= 1) this.anim = null;
    };
  }
  stop() { this.anim = null; }
  tick() { if (this.anim) this.anim(); }

  // frame a box: the bounding sphere fits the smaller field of view, 5 % margin
  fit(box, { animate = true, keepDirection = false } = {}) {
    const c = box.getCenter(new THREE.Vector3());
    const r = Math.max(1, box.getSize(new THREE.Vector3()).length() / 2);
    const aspect = this.width / Math.max(1, this.height);
    const half = Math.min(FOV, 2 * Math.atan(Math.tan((FOV * Math.PI) / 360) * aspect) * 180 / Math.PI) * Math.PI / 360;
    const dist = (r / Math.sin(half)) * 1.05;
    let q = this.q.clone();
    if (!keepDirection) { q = uprightQ(Math.atan2(HOME_DIR.y, HOME_DIR.x), Math.asin(HOME_DIR.z)); this.view = null; if (!this.parallelChosen) this.parallel = false; }
    const to = { target: c, q, dist };
    if (animate) this.glide(to, 450); else { this.q.copy(q); this.dist = dist; this.target.copy(c); this.apply(); }
  }

  // an axis view; asking for the one already shown turns to the opposite
  snap(name, animate = true) {
    if (name === this.view) name = OPPOSITE[name];
    const v = VIEWS[name];
    if (!v) return;
    this.view = name;
    this.parallel = true;
    const to = { target: this.target.clone(), q: uprightQ(v.yaw, v.pitch), dist: this.dist };
    if (animate) this.glide(to); else { this.q.copy(to.q); this.apply(); }
  }

  // the other side: half a turn about the camera's up axis
  opposite() { if (this.view) this.snap(this.view); else { this.glide({ target: this.target.clone(), q: this.q.clone().multiply(new THREE.Quaternion().setFromAxisAngle(Y, Math.PI)), dist: this.dist }); } }

  // a step turn: about the world vertical (dyaw), then the camera's right axis (dpitch)
  turn(dyaw, dpitch = 0) {
    this.freeView();
    const q = new THREE.Quaternion().setFromAxisAngle(Z, dyaw).multiply(this.q).multiply(new THREE.Quaternion().setFromAxisAngle(X, -dpitch));
    this.glide({ target: this.target.clone(), q, dist: this.dist }, 160);
  }

  toggleProjection() {
    this.parallel = !this.parallel;
    this.parallelChosen = this.parallel;
    this.apply();
  }

  // ---- input ----

  ndc(e) {
    const r = this.canvas.getBoundingClientRect();
    return new THREE.Vector2(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1);
  }

  // the world point under the cursor, or the point on the target plane
  pointAt(ndc) {
    const hit = this.pick(ndc);
    if (hit) return hit;
    const ray = new THREE.Raycaster(); ray.setFromCamera(ndc, this.camera);
    const { dir } = this.basis();
    const plane = new THREE.Plane().setFromNormalAndCoplanarPoint(dir, this.target);
    return ray.ray.intersectPlane(plane, new THREE.Vector3()) || this.target.clone();
  }

  attach() {
    const c = this.canvas;
    this.pointers = new Map();
    c.addEventListener('contextmenu', (e) => e.preventDefault());
    c.addEventListener('pointerdown', (e) => {
      try { c.setPointerCapture(e.pointerId); } catch { /* not a live pointer */ }
      this.pointers.set(e.pointerId, { x: e.clientX, y: e.clientY, x0: e.clientX, y0: e.clientY,
        mode: this.tool || (e.button === 2 || e.shiftKey ? 'pan' : e.altKey ? 'roll' : 'orbit') });
      if (this.pointers.size === 2) this.pinch = null;
    });
    c.addEventListener('pointermove', (e) => {
      const p = this.pointers.get(e.pointerId);
      if (!p) return;
      const dx = e.clientX - p.x, dy = e.clientY - p.y;
      p.x = e.clientX; p.y = e.clientY;
      if (this.pointers.size === 2) {  // two fingers: pinch zooms, moving together pans
        const [a, b] = [...this.pointers.values()];
        const d = Math.hypot(a.x - b.x, a.y - b.y);
        if (this.pinch) this.zoomAbout(this.pinch / Math.max(1, d), null);
        this.pinch = d;
        this.pan(dx / 2, dy / 2);
        return;
      }
      if (Math.hypot(e.clientX - p.x0, e.clientY - p.y0) < 4 && !p.moved) return;  // under 4 px is a click
      p.moved = true;
      if (p.mode === 'orbit') this.orbit(dx, dy);
      else if (p.mode === 'roll') this.roll(dx);
      else if (p.mode === 'pan') this.pan(dx, dy);
      else if (p.mode === 'zoom') this.zoomAbout(Math.exp(dy * 0.01), null);
    });
    const end = (e) => {
      const p = this.pointers.get(e.pointerId);
      this.pointers.delete(e.pointerId); this.pinch = null;
      if (p && !p.moved && e.type === 'pointerup' && e.button === 0 && this.onClick) this.onClick(e);  // a click, not a drag
    };
    c.addEventListener('pointerup', end);
    c.addEventListener('pointercancel', end);
    c.addEventListener('wheel', (e) => {
      if (this.wheelEnabled === false) return;  // e.g. the front page: the wheel scrolls the page until the view is touched
      e.preventDefault();
      const dy = e.deltaMode === 1 ? e.deltaY * 16 : e.deltaY;
      // one pick per gesture: the zoom point stays put while the wheel turns
      const now = performance.now();
      if (!this.wheelPoint || now - this.wheelAt > 250) this.wheelPoint = this.pointAt(this.ndc(e));
      this.wheelAt = now;
      this.zoomAbout(Math.exp(dy * WHEEL), this.wheelPoint);
    }, { passive: false });
    // Safari sends a pinch as its own gesture events (scale since the start),
    // not as ctrl + wheel; left alone it zooms the whole page. Same zoom,
    // about the point picked where the pinch began. The horizontal part of a
    // wheel event (deltaX) is never used: no stray pan during a zoom.
    let gScale = 1, gPoint = null;
    c.addEventListener('gesturestart', (e) => { e.preventDefault(); gScale = 1; gPoint = this.pointAt(this.ndc(e)); });
    c.addEventListener('gesturechange', (e) => {
      e.preventDefault();
      if (!(e.scale > 0) || this.wheelEnabled === false) return;
      if (performance.now() - (this.wheelAt || -1e9) < 250) { gScale = e.scale; return; }  // the same pinch, already seen as ctrl + wheel
      this.zoomAbout(gScale / e.scale, gPoint);
      gScale = e.scale;
    });
    c.addEventListener('gestureend', (e) => e.preventDefault());
    c.addEventListener('dblclick', (e) => {
      const p = this.pick(this.ndc(e));
      if (p) this.glide({ target: p, q: this.q.clone(), dist: this.dist }, 280);
    });
  }
}
