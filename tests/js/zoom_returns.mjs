// Zoom in and back out with trackpad-style wheel events; the camera must
// return to where it was. Run by tests/test_web.py (needs Node 22 or later).
import { Nav } from '../../site/js/nav.js';

const listeners = {};
const canvas = {
  addEventListener: (t, f) => { listeners[t] = f; },
  getBoundingClientRect: () => ({ left: 0, top: 0, width: 800, height: 600 }),
  setPointerCapture() {},
};
let pickAt = null;                       // (ndc) -> point on the model, or null
const nav = new Nav(canvas, { pick: (ndc) => (pickAt ? pickAt(ndc) : null) });
nav.resize(800, 600);
nav.target.set(10, -20, 5); nav.yaw = -2.3; nav.pitch = 0.5; nav.dist = 350; nav.apply();

let clock = 0;
globalThis.performance = { now: () => clock };
const wheel = (x, y, dy, o = {}) => listeners.wheel({ clientX: x, clientY: y, deltaY: dy, deltaX: o.dx || 0,
  deltaMode: 0, ctrlKey: !!o.ctrl, preventDefault() {} });
const state = () => [...nav.target.toArray(), nav.dist, nav.yaw, nav.pitch, ...nav.camera.position.toArray()];
const gesture = (x, y, dy, n, o) => { for (let i = 0; i < n; i++) { clock += 16; wheel(x, y, dy, o); } clock += 1000; };
const screen = (p) => { const q = p.clone().project(nav.camera); return [(q.x + 1) * 400, (1 - q.y) * 300]; };

const out = [];
const check = (name, fn) => {
  const a = state(); const extra = fn() || {}; const b = state();
  out.push({ name, err: Math.max(...a.map((v, i) => Math.abs(v - b[i]))), ...extra });
};
// pinch (ctrl + small fractional deltas, stray sideways component), off the model
check('pinch off the model', () => { gesture(90, 70, -3.7, 40, { ctrl: true, dx: 1.3 }); gesture(90, 70, 3.7, 40, { ctrl: true, dx: -0.9 }); });
// two-finger scroll with a horizontal component, over the model
const P = nav.target.clone().add({ x: 30, y: 12, z: -8 });
pickAt = () => P.clone();
check('scroll over the model', () => { gesture(500, 260, -6.25, 30, { dx: 2.5 }); gesture(500, 260, 6.25, 30, { dx: -2.5 }); });
// the point under the cursor stays under the cursor while zooming
pickAt = null;
const cursor = [620, 180];
const ndcOf = ([x, y]) => ({ x: (x / 800) * 2 - 1, y: -(y / 600) * 2 + 1 });
const before = nav.pointAt(ndcOf(cursor));
check('stays under the cursor', () => {
  gesture(cursor[0], cursor[1], -8, 25);
  const s = screen(before);
  const drift = Math.hypot(s[0] - cursor[0], s[1] - cursor[1]);
  gesture(cursor[0], cursor[1], 8, 25);
  return { pixels: drift };
});
console.log(JSON.stringify(out));
