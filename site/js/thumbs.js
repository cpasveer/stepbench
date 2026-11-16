// Small 3D pictures for the task cards and the compare grid.
//
// One hidden WebGL context draws every thumbnail, one at a time, and turns
// each drawing into an ordinary image; a page with a hundred tasks has one
// 3D context, not a hundred, and a hundred plain pictures. Cards are drawn when they scroll into
// view, and redrawn when the theme changes. On hover a card turns slowly:
// only that card is redrawn, about 30 times a second.
//
// Look: the viewer's -- isometric from the south-west, parallel projection,
// grey fixed parts with their true edges; in the compare grid the fixed
// parts are see-through and the submission is drawn in the answer colour.

import * as THREE from '../vendor/three/three.module.js';
import { decodeMesh, boxGeometry } from './viewer.js';

const css = (n) => getComputedStyle(document.documentElement).getPropertyValue(n).trim() || '#888';

let shared = null;
function ctx() {
  if (shared) return shared;
  const canvas = document.createElement('canvas');
  let renderer;
  try { renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true, preserveDrawingBuffer: true }); } catch { return null; }
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  const scene = new THREE.Scene();
  const hemi = new THREE.HemisphereLight(0xffffff, 0x8890a0, 1.5);
  const key = new THREE.DirectionalLight(0xffffff, 1.4);
  scene.add(hemi, key);
  const camera = new THREE.OrthographicCamera(-1, 1, 1, -1, -1e5, 1e5);
  camera.up.set(0, 0, 1);
  shared = { canvas, renderer, scene, hemi, key, camera, root: null };
  return shared;
}

const dataCache = new Map();   // url -> Promise<json>
function getJSON(url) {
  if (!dataCache.has(url)) dataCache.set(url, fetch(url).then((r) => (r.ok ? r.json() : null)).catch(() => null));
  return dataCache.get(url);
}

function body(mesh, face, edge, ghost) {
  const g = decodeMesh(mesh);
  if (!g) return null;
  const grp = new THREE.Group();
  const mat = new THREE.MeshStandardMaterial({ color: face, roughness: 0.62, metalness: 0.08,
    polygonOffset: true, polygonOffsetFactor: 1, polygonOffsetUnits: 1 });
  if (ghost) Object.assign(mat, { transparent: true, opacity: 0.3, depthWrite: false, side: THREE.DoubleSide });
  grp.add(new THREE.Mesh(g.faces, mat));
  grp.add(new THREE.LineSegments(g.edges, new THREE.LineBasicMaterial({ color: edge, transparent: true, opacity: ghost ? 0.6 : 1 })));
  return grp;
}

// build the scene for one picture: {fixed: [mesh], part: mesh|null}
function build(spec) {
  const light = document.documentElement.dataset.themeResolved === 'light';
  const fixedCol = css('--fixed'), fixedEdge = css('--fixed-edge');
  const grp = new THREE.Group();
  const ghost = !!spec.part;
  for (const m of spec.fixed || []) {
    const b = body(m, ghost ? new THREE.Color(fixedCol).lerp(new THREE.Color('#ffffff'), light ? 0.15 : 0.25) : fixedCol, fixedEdge, ghost);
    if (b) grp.add(b);
  }
  // obstacles: see-through, in their own colour, in every picture
  const oc = css('--obstacle');
  for (const o of spec.obstacles || []) {
    const g = boxGeometry(o);
    grp.add(new THREE.Mesh(g.faces, new THREE.MeshStandardMaterial({ color: oc, transparent: true, opacity: 0.28, depthWrite: false, side: THREE.DoubleSide })));
    grp.add(new THREE.LineSegments(g.edges, new THREE.LineBasicMaterial({ color: oc })));
  }
  if (spec.part) { const b = body(spec.part, css('--part'), css('--part-edge'), false); if (b) grp.add(b); }
  const box = new THREE.Box3().setFromObject(grp);
  return { grp, box };
}

function dispose(grp) {
  grp.traverse((o) => { if (o.geometry) o.geometry.dispose(); if (o.material) o.material.dispose(); });
}

// draw `spec` at heading `yaw` into the image `target`
function draw(spec, target, yaw = 0) {
  const c = ctx();
  if (!c) return false;
  const w = target.clientWidth || 320, h = target.clientHeight || 200;
  c.renderer.setSize(w, h, false);
  if (!spec._built) spec._built = build(spec);
  const { grp, box } = spec._built;
  if (c.root !== grp) { if (c.root) c.scene.remove(c.root); c.scene.add(grp); c.root = grp; }
  const light = document.documentElement.dataset.themeResolved === 'light';
  c.hemi.intensity = light ? 1.6 : 1.3;
  c.hemi.groundColor.set(light ? '#8890a0' : '#4a5468');
  const center = box.getCenter(new THREE.Vector3());
  const r = Math.max(1, box.getSize(new THREE.Vector3()).length() / 2);
  const base = Math.atan2(-0.65, -0.6) + yaw, pitch = Math.asin(0.47);
  const dir = new THREE.Vector3(Math.cos(pitch) * Math.cos(base), Math.cos(pitch) * Math.sin(base), Math.sin(pitch));
  c.camera.position.copy(center).addScaledVector(dir, r * 4);
  c.camera.lookAt(center);
  const aspect = w / h, hh = (aspect >= 1 ? r : r / aspect) * 1.02;
  Object.assign(c.camera, { left: -hh * aspect, right: hh * aspect, top: hh, bottom: -hh });
  c.camera.updateProjectionMatrix();
  c.key.position.copy(c.camera.position);
  c.renderer.setClearColor(0x000000, 0);
  c.renderer.render(c.scene, c.camera);
  target.src = c.canvas.toDataURL('image/png');
  if (!draw.marked) { draw.marked = true; performance.mark('jb-first-thumb'); }
  return true;
}

// a thumbnail element: <canvas> that fills in when visible and turns on hover
const live = new Set();
let observer = null;
// load a card when it comes near the screen. A card already in view when
// it is placed is loaded straight away (checked once it is in the page);
// the watcher only handles cards further down.
function observe(el) {
  if (!observer) {
    observer = new IntersectionObserver((entries) => {
      for (const e of entries) if (e.isIntersecting && !e.target._started) { e.target._started = true; observer.unobserve(e.target); e.target._load(); }
    }, { rootMargin: '200px' });
  }
  const check = (tries) => {
    if (el._started) return;
    if (!el.isConnected) { if (tries > 0) setTimeout(() => check(tries - 1), 50); return; }
    const r = el.getBoundingClientRect();
    if (r.bottom > -200 && r.top < innerHeight + 200) { el._started = true; el._load(); }
    else observer.observe(el);
  };
  setTimeout(() => check(40), 0);
}

export function thumb(urls, { hoverTurn = true } = {}) {
  const canvas = document.createElement('img');
  canvas.className = 'thumb3d';
  canvas.alt = '';
  canvas.setAttribute('aria-hidden', 'true');
  let spec = null, turning = null;
  canvas._load = async () => {
    canvas.dataset.state = 'loading';
    const fixed = urls.fixed ? await getJSON(urls.fixed) : null;
    const part = urls.part ? await getJSON(urls.part) : null;
    if (!fixed && !part) { canvas.classList.add('empty'); return; }
    spec = { fixed: fixed ? fixed.fixed : [], obstacles: fixed ? fixed.obstacles || [] : [], part: part || null };
    canvas._redraw();
    canvas.dataset.state = canvas.isConnected ? 'drawn' : 'detached';
    live.add(canvas);
  };
  canvas._redraw = () => { if (spec && canvas.isConnected) { if (spec._built) { dispose(spec._built.grp); spec._built = null; } draw(spec, canvas, 0); } };
  if (hoverTurn) {
    canvas.addEventListener('pointerenter', () => {
      if (!spec) return;
      const t0 = performance.now();
      // about 20 pictures a second, for the one card under the pointer
      let last = 0;
      const step = (now) => {
        if (!turning) return;
        if (now - last > 50) { draw(spec, canvas, ((now - t0) / 1000) * 0.6); last = now; }
        turning = requestAnimationFrame(step);
      };
      turning = requestAnimationFrame(step);
    });
    canvas.addEventListener('pointerleave', () => { cancelAnimationFrame(turning); turning = null; if (spec) draw(spec, canvas, 0); });
  }
  observe(canvas);
  return canvas;
}

// redraw every visible thumbnail (theme change)
export function redrawThumbs() {
  for (const c of [...live]) { if (!c.isConnected) live.delete(c); else c._redraw(); }
}

export function thumbsAvailable() { return !!ctx(); }
