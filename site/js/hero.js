// The front page's 3D showcase: real attempts, one after another.
//
// Each attempt turns slowly for a few seconds under a quiet caption (task,
// who submitted it, passed or failed, which try it was, and the check that
// failed, with its mark in the scene), then the next one comes. Failures
// are put in on purpose: they show what the benchmark measures. The first
// view names the roles: the grey parts are given, the coloured one is what
// had to be modelled.
//
// The list comes from data/showcase.json: a curated list when the folder
// has one, otherwise every attempt whose geometry is kept, picked at random
// with passes and failures alternating. With no attempts at all, tasks are
// shown with their reference answer.
//
// The visitor is in charge as soon as they touch the view: cycling stops,
// the view can be turned and zoomed, and the caption links to the attempt.
// Until then the mouse wheel scrolls the page, not the model. With reduced
// motion asked for, one attempt is shown, still. One 3D context in all;
// it is started only after the text of the page is in place.

import { Viewer } from './viewer.js';
import { getJSON } from './fetch.js';
import { devTag } from './rules.js';

const HOLD_MS = 7000;     // how long each attempt is shown
const TURN = 0.11;        // radians per second while shown
const esc = (s) => String(s ?? '').replace(/[&<>"']/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));


function shuffle(a) {
  for (let i = a.length - 1; i > 0; i--) { const j = Math.floor(Math.random() * (i + 1)); [a[i], a[j]] = [a[j], a[i]]; }
  return a;
}

// random order, failures and passes taking turns, a failure first
function order(list) {
  if (list.some((e) => e.curated)) return list;
  const f = shuffle(list.filter((e) => e.passed === false)), p = shuffle(list.filter((e) => e.passed !== false));
  const out = [];
  while (f.length || p.length) { if (f.length) out.push(f.shift()); if (p.length) out.push(p.shift()); }
  return out;
}

const CHECK_WORDS = {
  FILE_VALID: 'not one valid solid',
  HOLES: 'a bolt does not go in',
  SEATING: 'does not sit flat',
  NO_INTERFERENCE: 'runs into a given part',
};

export async function mountHero(host) {
  let list;
  try { list = order(await getJSON('data/showcase.json')); } catch { list = []; }
  if (!list.length) return null;
  const still = matchMedia('(prefers-reduced-motion: reduce)').matches;

  host.classList.add('hero3d');
  host.innerHTML = `
    <div class="h3-stage" tabindex="0" aria-label="A real attempt in 3D. Drag to turn it.">
      <div class="labels"></div>
      <svg class="overlay" aria-hidden="true"><g class="hd"></g><g class="ms"></g></svg>
      <div class="h3-roles" aria-hidden="true"><span class="given">given</span><span class="tomodel">to model</span></div>
      <button class="h3-next" type="button" aria-label="Next attempt" title="Next attempt">›</button>
    </div>
    <p class="h3-cap" aria-live="polite"></p>`;
  const stage = host.querySelector('.h3-stage'), cap = host.querySelector('.h3-cap');
  const roles = host.querySelector('.h3-roles');
  let viewer;
  try {
    viewer = new Viewer(stage, { labels: stage.querySelector('.labels'), overlay: stage.querySelector('.overlay') });
  } catch { host.classList.add('no3d'); host.innerHTML = ''; return null; }
  window.removeEventListener('keydown', viewer._onKey);   // the page's keys are not the viewer's here
  viewer.nav.wheelEnabled = false;
  viewer.setShow('faces', false);

  let i = -1, timer = null, touched = false, alive = true, shownAt = 0, lastT = 0;
  const scenes = new Map();       // task key -> scene.json
  const atts = new Map();         // task key -> attempts.json
  const load = async (e) => {
    if (!scenes.has(e.key)) scenes.set(e.key, getJSON(`data/tasks/${e.key}/scene.json`));
    if (!e.reference && !atts.has(e.key)) atts.set(e.key, getJSON(`data/tasks/${e.key}/attempts.json`));
    const scene = await scenes.get(e.key);
    const att = e.reference ? null : (await atts.get(e.key)).find((a) => a.n === e.n);
    const meshId = e.reference ? scene.reference : att && att.mesh;
    const mesh = meshId ? await getJSON(`data/mesh/${meshId}.json`) : null;
    const marks = att && att.marks ? await getJSON(`data/tasks/${e.key}/marks/${att.n}.json`).catch(() => null) : null;
    return { scene, att, mesh, marks };
  };

  const caption = (e, att) => {
    const link = e.reference ? `#/task/${encodeURIComponent(e.key)}` : `#/task/${encodeURIComponent(e.key)}/${e.n}`;
    if (e.reference) return `<span class="mono">${esc(e.task)}</span> · reference answer · no attempts yet <a href="${link}">open the task</a>`;
    const by = att && att.by ? esc(att.by) : 'unknown';
    const verdict = e.passed ? '<b class="pass">passed</b>' : '<b class="fail">failed</b>';
    const fail = !e.passed && e.failed.length
      ? ` · <span class="mono">${esc(e.failed[0])}</span> ${esc(CHECK_WORDS[e.failed[0]] || '')}${e.failed.length > 1 ? ` (+${e.failed.length - 1})` : ''}`
      : '';
    // attempts graded before the current rules are sample data, not results
    const dev = e.rules_current === false ? ` ${devTag()}` : '';
    return `<span class="mono">${esc(e.task)}</span> · ${by} · ${verdict} on try ${e.n}${fail}${dev} <a href="${link}">open this attempt</a>`;
  };

  const show = async (k) => {
    const e = list[k % list.length];
    let d;
    try { d = await load(e); } catch { return false; }
    if (!alive) return false;
    viewer.fitted = false;
    viewer.setScene(d.scene, 0);
    viewer.setBody('part', e.reference ? null : d.mesh);
    viewer.setBody('ref', e.reference ? d.mesh : null);
    viewer.setMode('ref', e.reference ? 'solid' : 'hidden');
    viewer.focus = new Set();     // only the bolts the grader complains about are named
    viewer.setVerdict(d.att, d.scene.task.tolerances || {});
    viewer.setMarks(d.marks ? d.marks.regions : []);
    viewer.nav.yaw += (k % 2 ? -1 : 1) * 0.25;   // each one from a slightly different side
    viewer.nav.dist *= 0.72;                      // closer than the task page: there is no panel to leave room for
    viewer.nav.apply();
    cap.innerHTML = caption(e, d.att);
    stage.dataset.state = e.reference ? 'reference' : e.passed ? 'pass' : 'fail';
    roles.hidden = k !== 0 || touched;
    shownAt = performance.now();
    // fetch the next one while this one turns
    if (!still && list.length > 1) load(list[(k + 1) % list.length]).catch(() => {});
    return true;
  };

  const next = async () => {
    clearTimeout(timer);
    for (let tries = 0; tries < Math.min(list.length, 5); tries++) {
      i = (i + 1) % list.length;
      if (await show(i)) break;
    }
    if (!touched && !still && alive && list.length > 1) timer = setTimeout(next, HOLD_MS);
  };

  // the slow turn, and the role labels on the first view
  const place = () => {
    if (roles.hidden) return;
    const B = viewer.bodies, w = viewer.width, h = viewer.height;
    const at = (b, top) => {
      if (!b || !b.mesh.geometry.boundingBox) return null;
      const bb = b.mesh.geometry.boundingBox, c = bb.getCenter(bb.min.clone());
      if (top) c.z = bb.max.z;
      const v = c.project(viewer.camera);
      return [((v.x + 1) / 2) * w, ((1 - v.y) / 2) * h];
    };
    // "given" on the largest given part, "to model" under the middle of the answer
    const size = (b) => b.mesh.geometry.boundingBox ? b.mesh.geometry.boundingBox.getSize(b.mesh.geometry.boundingBox.min.clone()).length() : 0;
    const big = [...B.fixed].sort((a, b) => size(b) - size(a))[0];
    const g = at(big, true), p = at(B.part || B.ref, false);
    const [eg, ep] = [roles.querySelector('.given'), roles.querySelector('.tomodel')];
    eg.style.display = g ? '' : 'none'; ep.style.display = p ? '' : 'none';
    if (g) Object.assign(eg.style, { left: g[0] + 'px', top: g[1] + 'px' });
    if (p) Object.assign(ep.style, { left: p[0] + 'px', top: p[1] + 'px' });
    if (B.ref && !B.part) ep.textContent = 'to model (the reference answer)';
  };
  const tick = (t) => {
    if (!alive) return;
    const dt = lastT ? Math.min(0.05, (t - lastT) / 1000) : 0;
    lastT = t;
    if (!touched && !still && !document.hidden && t - shownAt > 400) { viewer.nav.yaw += TURN * dt; viewer.nav.apply(); }
    place();
    requestAnimationFrame(tick);
  };
  requestAnimationFrame(tick);

  // the visitor takes over: no more cycling, the wheel zooms the model
  const takeOver = () => {
    if (touched) return;
    touched = true;
    clearTimeout(timer);
    viewer.nav.wheelEnabled = true;
    roles.hidden = true;
    stage.classList.add('held');
  };
  stage.addEventListener('pointerdown', (e) => { if (!e.target.closest('.h3-next')) takeOver(); });
  stage.addEventListener('keydown', (e) => { if (e.key !== 'Tab') takeOver(); });
  host.querySelector('.h3-next').addEventListener('click', () => { takeOver(); next(); });
  if (list.length < 2) host.querySelector('.h3-next').hidden = true;

  await next();
  return {
    viewer,
    destroy() { alive = false; clearTimeout(timer); viewer.destroy(); },
  };
}
