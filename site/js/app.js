// JointBench front end: overview, task page, attempts, upload.
// Reads only the JSON documents under data/ (see src/jointbench/web/data.py);
// the live server and the static export serve the same paths.

import { boltsFromReport } from './report.js';
import { markdown } from './md.js';
import { renderScores } from './scores.js';
import { renderAbout } from './about.js';
import { timelineTab, previousStep } from './timeline.js';
import { materialRatio, fmtRatio } from './quality.js';
import { ICONS } from './icons.js';
import { getJSON } from './fetch.js';
import { devTag } from './rules.js';
import { ladderHTML } from './ladder.js';

// The 3D code (three.js and the modules built on it, about 1.3 MB before
// compression) is loaded only by pages that draw: the front page after its
// text is shown, a task page, the compare pages. The scores page never
// loads it. G holds those modules once loaded.
let G = null, G3 = null;
function load3d() {
  if (!G3) {
    G3 = Promise.all([import('./viewer.js'), import('./thumbs.js'), import('./dfm.js'), import('./compare.js'), import('./hero.js'), import('./inspect.js').then((m) => ({ inspect: m })), import('./fieldview.js')])
      .then((ms) => (G = Object.assign({}, ...ms)));
  }
  return G3;
}

const $ = (sel, el = document) => el.querySelector(sel);
const h = (html) => { const t = document.createElement('template'); t.innerHTML = html.trim(); return t.content.firstElementChild; };
const esc = (s) => String(s ?? '').replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));

const app = $('#app');
let site = { mode: 'static' };
let current = null; // the mounted page, with .destroy()


// ---------------------------------------------------------------- theme
// Dark is the default; after that the reader's last choice is kept (in
// this browser only). A link may ask for a theme with ?theme= (screenshots).

function storedTheme() {
  const q = new URLSearchParams((location.hash.split('?')[1]) || '').get('theme');
  if (q === 'light' || q === 'dark') return q;
  try { return localStorage.getItem('jb-theme') === 'light' ? 'light' : 'dark'; } catch { return 'dark'; }
}
function applyTheme(t) {
  const root = document.documentElement;
  if (t === 'light') root.dataset.theme = 'light'; else delete root.dataset.theme;
  root.dataset.themeResolved = t;
  const meta = document.querySelector('meta[name="theme-color"]');
  if (meta) meta.content = t === 'light' ? '#f2f0e9' : '#384154';
  const b = $('#theme');
  if (b) {
    b.title = t === 'light' ? 'Light theme (click for dark)' : 'Dark theme (click for light)';
    b.innerHTML = t === 'light'
      ? '<svg viewBox="0 0 16 16"><circle cx="8" cy="8" r="3.2" fill="currentColor"/><g stroke="currentColor" stroke-width="1.4"><path d="M8 1v2M8 13v2M1 8h2M13 8h2M3 3l1.4 1.4M11.6 11.6L13 13M3 13l1.4-1.4M11.6 4.4L13 3"/></g></svg>'
      : '<svg viewBox="0 0 16 16"><path d="M13 10.5A6 6 0 0 1 5.5 3 6 6 0 1 0 13 10.5z" fill="currentColor"/></svg>';
  }
  if (current && current.viewer) current.viewer.applyTheme();
  if (current && current.viewers) for (const v of current.viewers) v.applyTheme();   // the compare page
  if (current && current.retheme) current.retheme();   // overlays drawn in theme colours
  if (G) G.redrawThumbs();
}
$('#theme').addEventListener('click', () => {
  const t = document.documentElement.dataset.themeResolved === 'light' ? 'dark' : 'light';
  try { localStorage.setItem('jb-theme', t); } catch { /* private mode: the choice lasts this page only */ }
  applyTheme(t);
});
applyTheme(storedTheme());

// ---------------------------------------------------------------- helpers

function ago(iso) {
  if (!iso) return '';
  const s = (Date.now() - Date.parse(iso)) / 1000;
  if (s < 50) return 'just now';
  if (s < 3600) return `${Math.round(s / 60)} min ago`;
  if (s < 86400) return `${Math.round(s / 3600)} h ago`;
  return new Date(iso).toISOString().slice(0, 10);
}

const KIND = { nema17: 'NEMA 17 motor', flange2: '2-hole flange', flange4: '4-hole flange' };
const kindName = (k) => KIND[k] || k.replace(/_/g, ' ');
const partLine = (p) => `${kindName(p.kind)} · ${p.holes}×${p.bolt}`;

// the verdict as an inspection report's header row: plain words in ruled
// cells, colour kept to the verdict word only
function verdictBlock(a) {
  const n = (a.checks || []).length, ok = (a.checks || []).filter((c) => c.status === 'pass').length;
  // the verdict rests on the required checks; the rest (geometry quality,
  // printing) are advice. Counted apart when the report says which is which.
  const req = (a.checks || []).filter((c) => c.required === true);
  const reqOk = req.filter((c) => c.status === 'pass').length;
  const checksCell = req.length && req.length < n
    ? `<span class="num">${reqOk} of ${req.length}</span> required <span class="dim num">· ${ok} of ${n} in all</span>`
    : `<span class="num">${ok} of ${n}</span>`;
  return `<div class="vblock">
    <div class="v"><small>verdict</small><b class="vword ${a.passed ? 'ok' : 'bad'}">${a.passed ? 'pass' : 'fail'}</b></div>
    <div><small>checks passed</small>${checksCell}</div>
    <div><small>attempt</small><span class="num">#${a.n}</span></div>
    <div class="wide"><small>submitted by</small>${esc(a.by)}</div>
    <div class="wide"><small>graded</small><span class="num" title="${esc(a.time)}">${esc((a.time || '').replace('T', ' ').slice(0, 16))} UTC · ${esc(ago(a.time))}</span></div>
    <div class="wide"><small>file</small><span class="num">${esc(a.file || '—')}</span></div>
    ${materialCell(a)}
  </div>`;
}

function materialCell(a) {
  const m = a.passed && a.info && a.info.material;
  if (!m || m.ratio === null || m.ratio === undefined) return '';
  const why = m.counts === false ? ` <span class="dim">· not counted${m.why_not ? `: ${esc(m.why_not)}` : ''}</span>` : '';
  return `<div class="wide"><small title="par volume ÷ part volume: 1 = as lean as the task's lean par answer, higher is leaner">material score (par ÷ volume, higher is leaner)</small><span class="num">${fmtRatio(m.ratio)}</span>${why}</div>`;
}

function stamp(passed, big = false) {
  if (passed === undefined || passed === null) return `<span class="stamp none${big ? ' big' : ''}">open</span>`;
  return `<span class="stamp ${passed ? 'ok' : 'bad'}${big ? ' big' : ''}">${passed ? 'pass' : 'fail'}</span>`;
}

function dots(checks) {
  return `<span class="dots">${(checks || []).map((c) => `<i class="${esc(c.status)}" title="${esc(c.name)}: ${esc(c.status)}"></i>`).join('')}</span>`;
}

// plan view thumbnail: mounting faces and bolts seen along the first face normal
function planSVG(plan) {
  if (!plan || !plan.faces || !plan.faces.length) return '<div class="noplan dim">no mounting faces to draw</div>';
  const pts = plan.faces.flatMap((f) => f.outline);
  const xs = pts.map((p) => p[0]), ys = pts.map((p) => p[1]);
  const pad = 10;
  const x0 = Math.min(...xs) - pad, x1 = Math.max(...xs) + pad, y0 = Math.min(...ys) - pad, y1 = Math.max(...ys) + pad;
  const w = x1 - x0, hgt = y1 - y0;
  const s = Math.min(320 / w, 200 / hgt);
  const ox = (320 - w * s) / 2, oy = (200 - hgt * s) / 2;
  const X = (x) => (ox + (x - x0) * s).toFixed(1), Y = (y) => (oy + (y1 - y) * s).toFixed(1);
  const faces = plan.faces.map((f, i) => {
    const d = f.outline.map((p, k) => `${k ? 'L' : 'M'}${X(p[0])} ${Y(p[1])}`).join('') + 'Z';
    const c = f.outline.reduce((a, p) => [a[0] + p[0] / 4, a[1] + p[1] / 4], [0, 0]);
    return `<path d="${d}" fill="${i ? 'none' : 'var(--datum-soft)'}" stroke="var(--datum)" stroke-width="1.2" ${i ? 'stroke-dasharray="5 3"' : ''}/>
      <text x="${X(c[0])}" y="${Y(c[1])}" font-size="11" font-family="var(--font-mono)" fill="var(--ink-3)" text-anchor="middle" dy="4">${esc(f.part)}</text>`;
  }).join('');
  const bolts = plan.bolts.map((b) => {
    const r = Math.max(2.2, (b.d / 2) * s);
    const x = X(b.at[0]), y = Y(b.at[1]);
    return `<circle cx="${x}" cy="${y}" r="${r.toFixed(1)}" fill="var(--sheet)" stroke="var(--ink-2)" stroke-width="1"/>
      <path d="M${x - r - 3} ${y}h${2 * r + 6}M${x} ${y - r - 3}v${2 * r + 6}" stroke="var(--ink-3)" stroke-width="0.6"/>`;
  }).join('');
  return `<svg class="plan" viewBox="0 0 320 200" aria-hidden="true">${faces}${bolts}</svg>`;
}

// the explanatory elevation on the front page: two given parts, the part to
// model. Shown only where the 3D showcase cannot be (no 3D in the browser).
const HERO_SVG = `
<svg class="diagram" viewBox="0 0 560 370" role="img" aria-label="Section through a task: fixed part B above, fixed part A below, the connecting part to model between them, bolted to both">
  <defs>
    <pattern id="hatch" width="7" height="7" patternUnits="userSpaceOnUse" patternTransform="rotate(45)">
      <line x1="0" y1="0" x2="0" y2="7" stroke="var(--ink-3)" stroke-width="1"/>
    </pattern>
  </defs>
  <g stroke="var(--ink)" stroke-width="1.4">
    <rect x="200" y="28" width="300" height="56" fill="url(#hatch)"/>
    <rect x="70" y="282" width="260" height="56" fill="url(#hatch)"/>
    <rect x="176" y="272" width="48" height="10" fill="url(#hatch)"/>
    <rect x="195" y="200" width="10" height="72" fill="var(--paper-2)"/>
  </g>
  <g fill="var(--part)" fill-opacity="0.18" stroke="var(--part)" stroke-width="2" stroke-dasharray="6 4">
    <path d="M86 270 H168 V258 H232 V270 H352 V98 H486 V84 H330 V256 H86 Z" fill="none"/>
  </g>
  <path d="M86 270 H168 V258 H232 V270 H352 V98 H486 V84 H330 V256 H86 Z" fill="var(--part)" fill-opacity="0.16" stroke="none"/>
  <g stroke="var(--datum)" stroke-width="1" stroke-dasharray="10 3 2 3">
    <line x1="112" y1="300" x2="112" y2="226"/><line x1="300" y1="300" x2="300" y2="226"/>
    <line x1="390" y1="60" x2="390" y2="128"/><line x1="462" y1="60" x2="462" y2="128"/>
  </g>
  <g fill="var(--paper)" stroke="var(--ink-2)" stroke-width="1.2">
    <rect x="102" y="244" width="20" height="12"/><rect x="290" y="244" width="20" height="12"/>
    <rect x="380" y="98" width="20" height="12"/><rect x="452" y="98" width="20" height="12"/>
  </g>
  <g stroke="var(--datum)" stroke-width="2.4"><line x1="62" y1="282" x2="340" y2="282"/><line x1="192" y1="84" x2="508" y2="84"/></g>
  <g stroke="var(--ink-2)" stroke-width="0.8" fill="var(--ink-2)">
    <line x1="528" y1="84" x2="528" y2="282"/><line x1="512" y1="84" x2="534" y2="84"/><line x1="346" y1="282" x2="534" y2="282"/>
    <path d="M528 84 l-3.5 9 h7z M528 282 l-3.5 -9 h7z"/>
  </g>
  <text class="lab" x="542" y="187" transform="rotate(-90 542 187)" text-anchor="middle">gap</text>
  <text class="big" x="210" y="20">Given · fixed part B</text>
  <text class="lab" x="210" y="58" dx="8" dy="6" style="fill:var(--ink)">4 bolts, M4</text>
  <text class="big" x="70" y="360">Given · fixed part A</text>
  <text class="lab" x="78" y="316" style="fill:var(--ink)">4 bolts, M3</text>
  <text class="lab" x="212" y="214">boss</text>
  <text class="big" x="368" y="166" style="fill:var(--part)">To model ·</text>
  <text class="big" x="368" y="185" style="fill:var(--part)">the joining part</text>
  <text class="lab" x="368" y="206">any shape that</text>
  <text class="lab" x="368" y="221">bolts to both</text>
  <text class="cap" x="62" y="296">mounting face</text>
  <text class="cap" x="420" y="76">mounting face</text>
</svg>`;

// ---------------------------------------------------------------- overview

async function renderOverview(scrollTo) {
  load3d().catch(() => {});   // start fetching the 3D code now, alongside the list; the text does not wait for it
  const index = await getJSON('data/index.json');
  const solved = index.filter((t) => t.solved).length;
  const tried = index.reduce((a, t) => a + t.attempts, 0);
  const el = h(`<div class="page">
    <section class="hero">
      <div>
        <div class="kicker">A benchmark for 3D CAD agents · units: mm</div>
        <h1>The parts are given.<br>The part that <em>joins them</em> is not.</h1>
        <p>Every task places a few parts in space, each with a flat mounting face and a pattern of bolts. The job is to model the one part that bolts to all of them. It exists nowhere: no drawing, no reference shape to copy.</p>
        <p>The answer is a STEP file. It is graded by measurement on exact geometry, the same way for an AI agent writing code and for a person in any CAD program.</p>
        <div class="cta">
          <a class="btn" href="#/task/${esc(index[0] ? index[0].key : '')}">Open a task</a>
          <a class="btn ghost" href="#/about">How it is graded</a>
        </div>
      </div>
      <div class="hero-side" id="hero-side">${HERO_SVG}</div>
    </section>

    <div class="section-title" id="about"><h2>Four measurements, no judge</h2><span class="n">every check reports a number</span></div>
    <div class="checks-explain">
      <div><code>FILE_VALID</code><h3>One real solid</h3><p>The file holds exactly one closed, valid solid. No loose surfaces, no mesh.</p></div>
      <div><code>HOLES</code><h3>Every bolt goes in</h3><p>A clearance hole on each bolt axis, within 0.1 mm, of a standard size, with a stock screw length of material under the head.</p></div>
      <div><code>SEATING</code><h3>Sits flat where clamped</h3><p>A flat face lies on each mounting face and covers at least 75 % of the ring each bolt head presses on.</p></div>
      <div><code>NO_INTERFERENCE</code><h3>Nothing collides</h3><p>No overlap with the fixed parts, and room for every screw: its shank, and its head with clearance to turn it.</p></div>
    </div>
    <div class="prose" style="margin-top:16px">
      <p><strong>Graded on function, not likeness.</strong> A connector is right if every bolt goes in, it sits flat where it is clamped, and nothing collides. Those are facts about geometry that can be measured exactly, so a differently shaped body with the same interfaces passes. That is the definition of correct, not a loophole.</p>
      <p><strong>Generated from a seed.</strong> Each task is built backwards: a valid connector first, then the fixed parts placed where it fits. So every task is known to be solvable before anyone sees it. Level 1 is two parts facing each other with nothing in between; later levels add angles, obstacles, motion and printability.</p>
    </div>

    <div class="section-title"><h2>Tasks</h2><span class="n">${index.length} tasks · ${solved} solved · ${tried} attempts${index.some((t) => t.attempts && t.rules && !t.rules.current) ? ` ${devTag()}` : ''}</span>
      <span class="r" id="cards-note"></span></div>
    ${[...new Set(index.map((t) => t.level))].length > 1 || [...new Set(index.map((t) => t.run))].length > 1 ? `<div class="filters">
      <div class="seg" id="flv"><span class="lbl">level</span><button data-l="all" class="on">all</button>${[...new Set(index.map((t) => t.level))].sort().map((l) => `<button data-l="${l}">${l}</button>`).join('')}</div>
      ${[...new Set(index.map((t) => t.run))].length > 1 ? `<label class="sel">run <select id="frun"><option value="all">all</option>${[...new Set(index.map((t) => t.run))].map((r) => `<option value="${esc(r ?? '')}">${esc(r ?? 'single tasks')}</option>`).join('')}</select></label>` : ''}
    </div>` : ''}
    <div class="grid" id="cards"></div>

    <div class="section-title"><h2>Try it yourself</h2></div>
    <div class="runbox">
      ${site.mode === 'live'
        ? '<div class="note">Open a task, download its fixed parts, model the connector in any CAD program, export a STEP file and drop it on the <b>Try it</b> tab. It is graded by the same Python grader that grades the agents.</div>'
        : '<div class="note">This is a static copy: attempts are recorded, uploading is off. To grade your own STEP file, run it locally:</div>'}
      <pre><span class="c"># in a checkout of the repository</span>
uv sync
uv run jointbench gen --seed 0 --n 10 --out tasks
uv run jointbench play tasks</pre>
    </div>
    <footer class="foot"><span>JointBench · Apache-2.0</span><span>every verdict on this page was written by the Python grader, not by this page</span></footer>
  </div>`);
  const cards = $('#cards', el);
  const flt = { level: 'all', run: 'all' };
  let last = index;
  const fill = (idx) => {
    last = idx;
    cards.innerHTML = '';
    // process versions of one challenge (L2-s3000-fdm, -pbf, -moulding) sit together
    const PORDER = { fdm: 0, pbf: 1, moulding: 2 };
    const vb = (t) => (t.version && t.version.versioned ? `${t.run ?? ''}|${t.version.base}` : null);
    const groups = new Map();
    for (const t of idx) { const k = vb(t); if (k) { if (!groups.has(k)) groups.set(k, []); groups.get(k).push(t); } }
    const placed = new Set();
    const ordered = [];
    for (const t of idx) {
      const k = vb(t);
      if (k && groups.get(k).length > 1) {
        if (placed.has(k)) continue;
        placed.add(k);
        ordered.push({ group: groups.get(k).slice().sort((a, b) => (PORDER[a.version.process] ?? 9) - (PORDER[b.version.process] ?? 9)) });
      } else ordered.push(t);
    }
    const PNAME = { fdm: 'filament', pbf: 'powder bed', moulding: 'moulding' };
    for (const item of ordered) {
      if (item.group) {
        const g = item.group.filter((t) => (flt.level === 'all' || String(t.level) === flt.level) && (flt.run === 'all' || (t.run ?? '') === flt.run));
        if (!g.length) continue;
        const base = g[0].version.base;
        const box = h(`<div class="vgroup"><div class="vg-head"><b>${esc(base)}</b> <span class="dim">the same situation, made ${g.length} ways</span>${g[0].run ? ` <a class="vg-cmp" href="#/compare/${encodeURIComponent(base)}?v=${encodeURIComponent(g[0].run)}">see the ${g.length} side by side</a>` : ''}</div><div class="vg-cards"></div></div>`);
        for (const t of g) $('.vg-cards', box).appendChild(cardOf(t, PNAME));
        cards.appendChild(box);
        continue;
      }
      const t = item;
      if (flt.level !== 'all' && String(t.level) !== flt.level) continue;
      if (flt.run !== 'all' && (t.run ?? '') !== flt.run) continue;
      cards.appendChild(cardOf(t, PNAME));
    }
  };
  function cardOf(t, PNAME) {
    {
      const st = t.solved ? true : (t.attempts ? false : null);
      const card = h(`<a class="card" href="#/task/${esc(t.key)}">
        <div class="thumbslot">${G && !G.thumbsAvailable() ? planSVG(t.plan) : ''}</div>
        <div class="tb"><div><div class="tid">${esc(t.id)}${t.run ? ` <span class="runtag">${esc(t.run)}</span>` : ''}${t.version && t.version.versioned ? ` <span class="ptag-proc">${esc(PNAME[t.version.process] || t.version.process)}</span>` : ''}</div>
          <div class="meta">level ${t.level} · seed ${t.seed} · ${t.poses} pose${t.poses > 1 ? 's' : ''}${t.gap != null ? ` · gap ${t.gap} mm` : t.face_angle != null ? ` · faces ${t.face_angle}° off, ${t.face_distance} mm apart` : ''}${t.parts.length > 2 ? ` · ${t.parts.length} parts` : ''}${t.obstacles ? ` · ${t.obstacles} obstacle${t.obstacles > 1 ? 's' : ''}` : ''}</div></div>
          <div>${t.solved ? stamp(true) : t.attempts ? stamp(false) : stamp(null)}</div></div>
        <div class="desc">${t.parts.map((p) => `<b>${esc(p.name)}</b> ${esc(partLine(p))}`).join('<br>')}</div>
        <div class="foot"><span>${t.attempts} attempt${t.attempts === 1 ? '' : 's'}</span><span>${t.last ? esc(ago(t.last.time)) : '—'}</span></div>
      </a>`);
      if (G && G.thumbsAvailable()) $('.thumbslot', card).appendChild(G.thumb({ fixed: `data/tasks/${encodeURIComponent(t.key)}/thumb.json` }));
      void st;
      return card;
    }
  }
  fill(index);
  const flv = $('#flv', el), frun = $('#frun', el);
  if (flv) flv.addEventListener('click', (e) => {
    const b = e.target.closest('button[data-l]'); if (!b) return;
    flt.level = b.dataset.l;
    for (const x of flv.querySelectorAll('button')) x.classList.toggle('on', x === b);
    fill(last);
  });
  if (frun) frun.addEventListener('change', () => { flt.run = frun.value; fill(last); });
  app.replaceChildren(el);
  let es = null;
  if (site.mode === 'live') {
    es = new EventSource('api/events');
    es.addEventListener('attempts', async () => fill(await getJSON('data/index.json')));
  }
  if (scrollTo) requestAnimationFrame(() => { const t = document.getElementById(scrollTo); if (t) t.scrollIntoView({ behavior: 'smooth' }); });
  else window.scrollTo(0, 0);
  performance.mark('jb-text');
  // the 3D showcase replaces the drawing once the text is on screen
  let hero = null, gone = false;
  const side = $('#hero-side', el);
  setTimeout(async () => {
    try { await load3d(); } catch { return; }
    if (gone) return;
    // the cards' 3D pictures, or plan drawings where the browser has no 3D
    $('#cards-note', el).textContent = G.thumbsAvailable() ? 'the fixed parts of each task, from the south-west; hover to turn' : 'plan view: mounting faces and bolts, far face dashed';
    fill(last);
    const holder = document.createElement('div');
    side.prepend(holder);
    let m = null;
    try { m = await G.mountHero(holder); } catch { m = null; }
    if (gone) { if (m) m.destroy(); return; }
    if (m) { hero = m; for (const s of side.querySelectorAll('svg.diagram')) s.remove(); } else holder.remove();
  }, 0);
  return { get viewer() { return hero && hero.viewer; }, destroy() { gone = true; if (es) es.close(); if (hero) hero.destroy(); } };
}

// ---------------------------------------------------------------- task page

async function renderTask(tid, wantN, params) {
  const [scene, attempts] = await Promise.all([
    getJSON(`data/tasks/${tid}/scene.json`),
    getJSON(`data/tasks/${tid}/attempts.json`),
    load3d(),
  ]);
  const tol = scene.task.tolerances || {};
  const parts = scene.task.fixed_parts;
  const el = h(`<div class="task">
    <div class="stage" id="stage">
      <div class="labels" id="labels"></div>
      <svg class="overlay" id="overlay" aria-hidden="true"><g class="hd"></g><g class="ms"></g></svg>
      <div class="rail" id="show" role="toolbar" aria-orientation="vertical" aria-label="Display">
        <button data-l="fixed" class="tb st-solid">${ICONS.fixed}<span class="sr"></span></button>
        <button data-l="part" class="tb st-solid">${ICONS.part}<span class="sr"></span></button>
        ${scene.reference ? `<button data-l="ref" class="tb st-hidden">${ICONS.ref}<span class="sr"></span></button>` : ''}
        ${scene.poses.some((p) => p.obstacles && p.obstacles.length) ? `<button data-l="obstacle" class="tb st-ghost">${ICONS.obstacle}<span class="sr"></span></button>` : ''}
        <i class="rsep"></i>
        <button data-t="faces" class="tb on" aria-pressed="true">${ICONS.faces}<span class="sr"></span></button>
        <button data-t="bolts" class="tb on" aria-pressed="true">${ICONS.bolts}<span class="sr"></span></button>
        <i class="rsep"></i>
        <button data-dl="1" class="tb" aria-haspopup="dialog" aria-expanded="false">${ICONS.download}<span class="sr"></span></button>
      </div>
      <section class="float-panel dl-panel" id="dlpanel" hidden aria-label="Download">
        <header><span>Download</span><button class="x" data-close="dl" aria-label="Close">${ICONS.close}</button></header>
        <div class="fp-body" id="dlbody"></div>
      </section>
      <div class="tr">
        <div class="gizmo" id="gizmo"></div>
        <div class="toolcol" role="toolbar" aria-orientation="vertical" aria-label="View">
          <button class="rb" data-tool="fit">${ICONS.fit}<span class="sr"></span></button>
          <button class="rb" data-tool="zoom">${ICONS.zoom}<span class="sr"></span></button>
          <button class="rb" data-tool="pan">${ICONS.pan}<span class="sr"></span></button>
          <button class="rb" data-tool="proj">${ICONS.projection}<span class="sr"></span></button>
          <i class="cgap"></i>
          <button class="rb" data-tool="section">${ICONS.section}<span class="sr"></span><i class="dot"></i></button>
          <button class="rb" data-tool="measure">${ICONS.measure}<span class="sr"></span><i class="dot"></i></button>
        </div>
        <div class="viewlabel num" id="viewlabel" aria-live="polite"></div>
      </div>
      <section class="float-panel sec-panel" id="secpanel" hidden aria-label="Section">
        <header><span>Section</span><button class="x" data-close="sec" aria-label="Close">${ICONS.close}</button></header>
        <div class="fp-body">
          <div class="sec-tools">
            <button data-cut="x" title="Cut across X through the pivot">X</button>
            <button data-cut="y" title="Cut across Y through the pivot">Y</button>
            <button data-cut="z" title="Cut across Z through the pivot">Z</button>
            <button data-cut="face" title="Click a face of a part: the cut takes its position and direction">${ICONS.onface}<span>On face</span></button>
            <button data-cut="box" title="Six faces around the model, each dragged on its own">${ICONS.clipbox}<span>Box</span></button>
          </div>
          <div class="sec-list" id="seclist"></div>
          <p class="fp-note" id="sechint">Drag a cut's dot in the drawing to move it along its arrow (Ctrl: 1 mm steps).</p>
        </div>
      </section>
      ${scene.poses.length > 1 ? `<div class="pose-ctl"><button class="rb" id="poseplay" title="Play the motion" aria-label="Play the motion" aria-pressed="false">▶</button><span>pose</span><input type="range" min="0" max="${scene.poses.length - 1}" step="1" value="0" id="pose"><span class="num" id="posev">1 / ${scene.poses.length}</span></div>` : ''}
      <div class="readout"><div class="scalebar"><div class="bar" id="sbar"></div><span id="stext" class="num"></span></div><div class="probe" id="probe"></div></div>
      <div class="gizmo" id="gizmo"></div>
      <div class="legendbox" id="legend" hidden></div>
      <div class="fieldbar" id="fieldbar" hidden></div>
    </div>
    <button class="panel-toggle" id="ptoggle" aria-controls="sidepanel"></button>
    <aside class="panel" id="sidepanel">
      <div class="panel-head">
        <div class="tid">${esc(scene.id)}${tid.includes('__') ? ` <a class="runtag" href="#/scores/${encodeURIComponent(tid.split('__')[0])}" title="model run">${esc(tid.split('__')[0])}</a>` : ''}${tid.includes('__') && scene.rules && !scene.rules.current ? devTag() : ''}</div>
        <div class="sub">${parts.map((p) => `<b>${esc(p.name)}</b> ${esc(p.interface.description.split(':')[0])}`).join(' &nbsp;↔&nbsp; ')}</div>
        <button class="btn small ghost try-cta" data-goto="try">Try it yourself</button>
      </div>
      <div class="tabs" id="tabs">
        <button data-tab="verdict">Verdict</button>
        <button data-tab="timeline">Timeline</button>
        <button data-tab="attempts">Attempts<span class="c" id="acount"></span></button>
        <button data-tab="made">Made how?</button>
        <button data-tab="brief">Brief</button>
        <button data-tab="try">Try it yourself</button>
      </div>
      <div class="panel-body" id="pbody"></div>
    </aside>
  </div>`);
  app.replaceChildren(el);
  window.scrollTo(0, 0);

  // the side panel folds away so the model can fill the window; the choice
  // is kept in this browser (key ]: toggle)
  const panelOpen = (() => { try { return localStorage.getItem('jb-panel') !== 'closed'; } catch { return true; } })();
  const setPanel = (open) => {
    el.classList.toggle('collapsed', !open);
    const b = $('#ptoggle', el);
    const text = open ? 'Hide the side panel (])' : 'Show the side panel (])';
    b.title = text; b.setAttribute('aria-label', text); b.setAttribute('aria-expanded', String(open));
    b.innerHTML = `<svg viewBox="0 0 20 20" aria-hidden="true"><path d="${open ? 'M8 5 L13 10 L8 15' : 'M12 5 L7 10 L12 15'}"/></svg>`;
    try { localStorage.setItem('jb-panel', open ? 'open' : 'closed'); } catch { /* private mode */ }
  };
  setPanel(panelOpen);
  $('#ptoggle', el).addEventListener('click', () => setPanel(el.classList.contains('collapsed')));

  const viewer = new G.Viewer($('#stage', el), {
    labels: $('#labels', el), gizmo: $('#gizmo', el), probe: $('#probe', el), scalebar: $('#sbar', el), scaletext: $('#stext', el),
    overlay: $('#overlay', el), viewLabel: $('#viewlabel', el),
  });
  viewer.setScene(scene, 0);

  const page = { viewer, attempts, selected: null, tab: null, follow: true, es: null, refShown: false, compare: false, who: null, playing: false, timer: null,
    dfm: { process: null, file: null, key: null, field: null, running: false },
    // reviewing the part by one check (inspect.js): the check's name, the
    // field files loaded per source for the shown attempt, what to restore
    params,            // the link's settings (a ?view= there wins over the automatic first view)
    fieldStates: {},   // per rule: the scale shown (range, colours), kept while switching
    insp: { name: null, key: null, files: {}, loading: {}, running: {}, saved: null, framed: null, md: null, mdKey: null } };

  // ---- toolbar
  const cycle = { fixed: ['solid', 'ghost', 'hidden'], part: ['solid', 'ghost', 'hidden'], ref: ['hidden', 'ghost', 'solid'], obstacle: ['ghost', 'solid', 'hidden'] };
  // every control says what it is and what state it is in, to the eye
  // (the icon), on hover (tooltip) and to screen readers (hidden text)
  const NAMES = { fixed: 'Fixed parts', part: 'Answer', ref: 'Reference answer', obstacle: 'Obstacles' };
  const KEYS = { fixed: 'F', part: 'A', ref: 'R', obstacle: 'O', faces: 'P', bolts: 'B', sec: 'S', dl: 'D' };
  const STATE = { solid: 'solid', ghost: 'see-through', hidden: 'hidden' };
  function label(b, text) {
    b.title = text;
    b.setAttribute('aria-label', text);
    $('.sr', b).textContent = text;
  }
  function setLayer(l, mode) {
    const b = $(`#show button[data-l="${l}"]`, el);
    if (!b || !cycle[l].includes(mode)) return;
    if (l === 'ref' && mode !== 'hidden' && !page.refShown) { page.refShown = true; loadRef(); }
    viewer.setMode(l, mode);
    b.classList.remove('st-solid', 'st-ghost', 'st-hidden');
    b.classList.add(`st-${mode}`);
    label(b, `${NAMES[l]}: ${STATE[mode]} (${KEYS[l]} cycles solid, see-through, hidden)`);
  }
  function setToggle(t, on) {
    const b = $(`#show button[data-t="${t}"]`, el);
    b.classList.toggle('on', on);
    b.setAttribute('aria-pressed', String(on));
    label(b, `${t === 'faces' ? 'Mounting faces' : 'Bolts'}: ${on ? 'shown' : 'hidden'} (${KEYS[t === 'faces' ? 'faces' : 'bolts']})`);
    viewer.setShow(t, on);
  }
  function setView(v) {
    viewer.view(v);
  }
  $('#show', el).addEventListener('click', (e) => {
    const b = e.target.closest('button'); if (!b) return;
    if (b.dataset.l) {
      const l = b.dataset.l;
      if (l === 'ref' && !page.refShown &&
          !confirm('Show the reference answer? It is one shape that passes -- a spoiler if you mean to try the task yourself.')) return;
      const c = cycle[l];
      setLayer(l, c[(c.indexOf(viewer.modes[l]) + 1) % c.length]);
    } else if (b.dataset.t) {
      setToggle(b.dataset.t, !b.classList.contains('on'));
    } else if (b.dataset.dl) {
      toggleDownloads();
    }
  });
  for (const l of ['fixed', 'part', 'ref', 'obstacle']) setLayer(l, viewer.modes[l]);
  setToggle('faces', true); setToggle('bolts', true);
  label($('#show button[data-dl]', el), 'Download the parts shown (D)');

  // ---- downloads: the exact STEP, and STL / 3MF meshed from it (a floating
  // panel beside the rail)
  function toggleDownloads(force) {
    const m = $('#dlpanel', el), btn = $('#show button[data-dl]', el);
    const open = force !== undefined ? force : m.hidden;
    m.hidden = !open;
    btn.classList.toggle('on', open);
    btn.setAttribute('aria-expanded', String(open));
    if (!open) return;
    if (site.mode !== 'live' && page.dlMissing === undefined) {
      page.dlMissing = [];
      getJSON(`data/tasks/${tid}/download/missing.json`).then((m) => { page.dlMissing = m; if (!$('#dlpanel', el).hidden) toggleDownloads(true); }).catch(() => {});
    }
    const D = scene.downloads || { fixed: [], formats: [] };
    const run = tid.includes('__') ? tid.split('__')[0] + '_' : '';
    const gone = page.dlMissing || [];   // static copy: files the export could not make
    const row = (title, base) => `<div class="dl-row"><span>${title}</span>${D.formats.map((f) => gone.includes(`${base}.${f}`)
      ? `<span class="dim" title="this STEP could not be meshed for ${f.toUpperCase()}; the STEP itself is exact">${f.toUpperCase()} –</span>`
      : `<a href="data/tasks/${encodeURIComponent(tid)}/download/${base}.${f}" download="${esc(run + scene.id + '_' + base + '.' + f)}">${f.toUpperCase()}</a>`).join('')}</div>`;
    const a = page.selected;
    $('#dlbody', el).innerHTML = `${a && a.mesh ? row(`answer, attempt #${a.n}`, `attempt-${a.n}`) : `<div class="dl-row dim">${a ? `attempt #${a.n}: geometry not kept` : 'no answer shown'}</div>`}
      ${D.fixed.map((f) => row(`fixed part ${esc(f)}`, `fixed_${f}`)).join('')}
      ${D.obstacles ? row('obstacles (keep-out boxes)', 'obstacles') : ''}
      <p class="fp-note">STEP is the exact file (the graded one for an answer). ${esc(D.note || '')}.</p>`;
  }

  // ---- the round tools at the top right
  const tools = $('.toolcol', el);
  const TOOLTIP = {
    fit: 'Show the whole model (Home, 0)', zoom: 'Zoom: press and drag up or down (or the wheel)',
    pan: 'Pan: press and drag (or right-drag, Shift-drag)', section: 'Section (S)', measure: 'Measure: click two points (M)',
  };
  const syncTools = () => {
    const parallel = viewer.nav.parallel;
    const pb = $('[data-tool="proj"]', tools);
    pb.classList.toggle('on', parallel);
    label(pb, `Projection: ${parallel ? 'parallel' : 'perspective'} (5)`);
    for (const [k, v] of Object.entries(TOOLTIP)) label($(`[data-tool="${k}"]`, tools), v);
    const secOpen = !$('#secpanel', el).hidden;
    const sb = $('[data-tool="section"]', tools);
    sb.classList.toggle('on', secOpen || viewer.clip.picking);
    sb.classList.toggle('live', viewer.clip.active && !secOpen);  // a cut is in effect while the panel is closed
    const mb = $('[data-tool="measure"]', tools);
    mb.classList.toggle('on', viewer.measure.on);
    mb.classList.toggle('live', viewer.measure.items.length > 0 && !viewer.measure.on);
    $('.dot', mb).hidden = false;
    $('#stage', el).classList.toggle('picking', viewer.clip.picking || viewer.measure.on);
  };
  // zoom and pan: press the button and drag
  for (const k of ['zoom', 'pan']) {
    const b = $(`[data-tool="${k}"]`, tools);
    b.addEventListener('pointerdown', (e) => {
      try { b.setPointerCapture(e.pointerId); } catch { /* not a live pointer */ }
      let x = e.clientX, y = e.clientY;
      b.classList.add('on');
      const move = (ev) => {
        const dx = ev.clientX - x, dy = ev.clientY - y; x = ev.clientX; y = ev.clientY;
        if (k === 'zoom') viewer.nav.zoomAbout(Math.exp(dy * 0.01), null); else viewer.nav.pan(dx, dy);
      };
      const up = () => { b.classList.remove('on'); b.removeEventListener('pointermove', move); b.removeEventListener('pointerup', up); };
      b.addEventListener('pointermove', move); b.addEventListener('pointerup', up);
    });
  }
  function toggleSection(force) {
    const p = $('#secpanel', el);
    p.hidden = force !== undefined ? !force : !p.hidden;
    if (p.hidden) viewer.clip.picking = false;
    fillSection(); syncTools();
  }
  function toggleMeasure(force) {
    const on = force !== undefined ? force : !viewer.measure.on;
    if (on) viewer.measure.start(); else viewer.measure.stop();
    syncTools();
  }
  tools.addEventListener('click', (e) => {
    const b = e.target.closest('[data-tool]'); if (!b) return;
    const t = b.dataset.tool;
    if (t === 'fit') viewer.fit();
    else if (t === 'proj') { viewer.nav.toggleProjection(); syncTools(); }
    else if (t === 'section') toggleSection();
    else if (t === 'measure') toggleMeasure();
  });
  // section panel: ways to cut, and the cuts in effect
  function fillSection() {
    const list = $('#seclist', el), C = viewer.clip;
    $('[data-cut="face"]', el).classList.toggle('on', C.picking);
    $('[data-cut="box"]', el).classList.toggle('on', C.hasBox());
    $('#sechint', el).textContent = C.picking ? 'Click a face of a part. Esc cancels.'
      : C.active ? "Drag a cut's dot in the drawing to move it along its arrow (Ctrl: 1 mm steps)." : 'Choose a way to cut.';
    const free = C.planes.filter((p) => p.kind !== 'box');
    list.innerHTML = (C.hasBox() ? `<div class="sec-row"><span>box</span><span class="num dim">6 faces</span><span></span><button data-rm="box" title="Remove the box">${ICONS.close}</button></div>` : '') +
      free.map((p) => `<div class="sec-row"><span>${p.kind === 'face' ? 'on face' : `across ${p.axis.toUpperCase()}`}</span><span class="num">${esc(C.readout(p))}</span>
        <button data-flip="${p.id}" title="Cut the other side away">${ICONS.flip}</button><button data-rm="${p.id}" title="Remove this cut">${ICONS.close}</button></div>`).join('') +
      (C.active ? '<div class="sec-row end"><button class="link" data-clear="1">clear all</button></div>' : '');
  }
  $('#secpanel', el).addEventListener('click', (e) => {
    const C = viewer.clip;
    const b = e.target.closest('button'); if (!b) return;
    if (b.dataset.close) return toggleSection(false);
    if (b.dataset.cut) {
      const c = b.dataset.cut;
      if (c === 'face') { C.picking = !C.picking; C.changed(); }
      else if (c === 'box') C.setBox(!C.hasBox());
      else C.addAxis(c);
    } else if (b.dataset.flip) C.flip(+b.dataset.flip);
    else if (b.dataset.rm) { if (b.dataset.rm === 'box') C.setBox(false); else C.remove(+b.dataset.rm); }
    else if (b.dataset.clear) C.clear();
  });
  viewer.clip.onChange(() => { if (!$('#secpanel', el).hidden) fillSection(); syncTools(); });
  viewer.measure.onChange(() => { syncTools(); viewer.viewChanged = true; });
  $('#dlpanel', el).addEventListener('click', (e) => { if (e.target.closest('[data-close]')) toggleDownloads(false); });
  syncTools();
  const syncProj = setInterval(() => { const pb = $('[data-tool="proj"]', tools); if (pb && pb.classList.contains('on') !== viewer.nav.parallel) syncTools(); }, 300);

  // keyboard shortcuts (views are on the navigation ball: 1 3 7, 5, 9, 0)
  window.addEventListener('keydown', page.keys = (e) => {
    if (/INPUT|TEXTAREA|SELECT/.test((e.target && e.target.tagName) || '') || e.ctrlKey || e.metaKey || e.altKey) return;
    const k = e.key.toUpperCase();
    const click = (sel) => { const b = $(sel, el); if (b) { e.preventDefault(); b.click(); } };
    if (k === 'F') click('#show button[data-l="fixed"]');
    if (k === 'O') click('#show button[data-l="obstacle"]');
    else if (k === 'A') click('#show button[data-l="part"]');
    else if (k === 'R') click('#show button[data-l="ref"]');
    else if (k === 'P') click('#show button[data-t="faces"]');
    else if (k === 'B') click('#show button[data-t="bolts"]');
    else if (k === 'S') { e.preventDefault(); toggleSection(); }
    else if (k === 'M') { e.preventDefault(); toggleMeasure(); }
    else if (k === 'D') click('#show button[data-dl]');
    else if (e.key === ']') { e.preventDefault(); $('#ptoggle', el).click(); }
    else if (e.key === 'Escape') { if (page.insp.name) { if (document.activeElement && document.activeElement.closest('.checklist')) page.insp.focusName = page.insp.name; endInspect(); } toggleDownloads(false); setTimeout(syncTools, 0); }
  });
  page.syncProj = syncProj;
  const pose = $('#pose', el);
  if (pose) pose.addEventListener('input', () => {
    const k = +pose.value;
    viewer.setScene(scene, k);
    $('#posev', el).textContent = `${k + 1} / ${scene.poses.length}`;
    // a mechanism moves the answer with the pose; reloading the attempt is only needed for the bolt overlays
    if (page.selected && !scene.part_motion) showAttempt(page.selected.n, false);
  });
  // play the motion: step through the poses, round and round, until pressed again
  const play = $('#poseplay', el);
  if (play) play.addEventListener('click', () => {
    const on = play.getAttribute('aria-pressed') !== 'true';
    play.setAttribute('aria-pressed', String(on)); play.textContent = on ? '❚❚' : '▶';
    clearInterval(page.poseTimer);
    if (on) page.poseTimer = setInterval(() => { pose.value = String((+pose.value + 1) % scene.poses.length); pose.dispatchEvent(new Event('input')); }, 110);
  });

  async function loadRef() {
    try { viewer.setBody('ref', await getJSON(`data/mesh/${scene.reference}.json`)); viewer.setMode('ref', viewer.modes.ref); } catch { /* not exported */ }
  }

  // ---- tabs
  const tabs = $('#tabs', el);
  tabs.addEventListener('click', (e) => { const b = e.target.closest('button'); if (b) setTab(b.dataset.tab); });
  $('.try-cta', el).addEventListener('click', () => { setTab('try'); $('#tabs', el).scrollLeft = 1e4; });
  function setTab(t) {
    const was = page.tab;
    page.tab = t;
    if (was === 'made' && t !== 'made') showField();
    if (t === 'made' && page.insp.name) endInspect(false);
    for (const b of tabs.querySelectorAll('button')) b.classList.toggle('on', b.dataset.tab === t);
    const body = $('#pbody', el);
    body.replaceChildren((({ verdict: verdictTab, timeline: timelineView, made: madeTab, attempts: attemptsTab, brief: briefTab, try: tryTab })[t] || verdictTab)());
  }

  function verdictTab() {
    const a = page.selected;
    if (!a) {
      return h(`<div><div class="vblock"><div class="v"><small>verdict</small><b class="vword none">none</b></div><div><small>attempt</small>—</div><div class="wide"><small>status</small>no attempt shown</div></div>
        <p class="note">Nobody has attempted this task yet${site.mode === 'live' ? ', or the log is empty. An agent running <code>jointbench verify</code> in this folder will show up here live.' : '.'}</p>
        <p class="note">The grey parts are given. Blue marks each mounting face and bolt axis. The answer, when there is one, is drawn in orange.</p></div>`);
    }
    const bad = boltsFromReport(a, tol);
    const wrap = h(`<div>
      ${verdictBlock(a)}
      ${a.ladder ? `<div class="ladder-cap"><small>requirement ladder</small>${ladderHTML(a.ladder)}</div>` : ''}
      <div id="checks"></div>
      ${a.info && a.info.volume !== undefined ? `<div class="info">volume ${a.info.volume.toFixed(1)} mm³ · bounding box ${a.info.bbox_size.map((v) => v.toFixed(1)).join(' × ')} mm<br><span style="color:var(--ink-3)">measured, not graded</span></div>` : ''}
      ${a.mesh ? '' : '<p class="note">The file for this attempt has been overwritten since it was graded, so its shape cannot be shown. The verdict is as the grader wrote it.</p>'}
    </div>`);
    fillChecks($('#checks', wrap), a, bad);
    return wrap;
  }


  // ---- the check list: one line per check, grouped by requirement, each
  // group open when something in it fails (or as the reader last left it);
  // a click reviews the part by that check in 3D (inspect.js)
  const GROUPS_KEY = 'jb.checkGroups';
  const groupOpen = (() => { try { return JSON.parse(localStorage.getItem(GROUPS_KEY) || '{}') || {}; } catch { return {}; } })();
  const saveGroups = () => { try { localStorage.setItem(GROUPS_KEY, JSON.stringify(groupOpen)); } catch { /* private window */ } };

  // von Mises stress, for viewing only (the stiffness analysis's field file): a
  // line of its own in the sturdy group, never a check and never counted
  const STRESS = { name: 'STRESS', group: 'sturdy', status: 'info', required: false, value: null, unit: 'MPa',
    reason: 'von Mises stress under the design loads, for viewing only: not a requirement, in no verdict' };
  const withStress = (a) => {
    const fam = scene.task.family || 'connect';
    const can = (a.fields || []).includes('stiffness') || (site.mode === 'live' && fam === 'connect' && a.mesh);
    return can ? [...(a.checks || []), STRESS] : (a.checks || []);
  };

  function fillChecks(list, a, bad) {
    const I = G.inspect;
    list.replaceChildren();
    list.classList.add('checklist');
    for (const g of I.groupChecks(withStress(a))) {
      const open = g.name in groupOpen ? groupOpen[g.name] : g.failed > 0;
      const state = g.failedRequired ? 'fail' : g.failed ? 'adv' : 'pass';
      const sec = h(`<section class="cgroup ${state}${open ? ' open' : ''}">
        <button class="cghead" aria-expanded="${open}" data-group="${esc(g.name)}">
          <span class="mk">${state === 'pass' ? '✓' : '✕'}</span><b>${esc(g.name)}</b>
          <span class="num cnt">${g.passed}/${g.total}</span>
          <span class="role">${g.decides ? 'decides' : 'reported'}</span><span class="chev" aria-hidden="true">▸</span>
        </button>
        <div class="cgbody"${open ? '' : ' hidden'}></div></section>`);
      const body = $('.cgbody', sec);
      for (const c of g.checks) {
        const active = page.insp.name === c.name;
        const row = h(I.rowHTML(c, tol, { open: active, active }));
        if (active) row.appendChild(inspectDetails(c, a, bad));
        row.addEventListener('click', (e) => { if (!e.target.closest('details, a, button')) { if (e.detail) page.insp.focusName = null; toggleInspect(c.name); } });
        row.addEventListener('keydown', (e) => { if (e.key === 'Enter' || e.key === ' ') { e.preventDefault(); page.insp.focusName = c.name; toggleInspect(c.name); } });
        if (!page.insp.name) {
          const ids = boltIdsOf(c);
          const regions = c.name === 'NO_INTERFERENCE' && c.status === 'fail';
          row.addEventListener('mouseenter', () => { if (!page.insp.name) viewer.setFocus(ids, regions); });
          row.addEventListener('mouseleave', () => { if (!page.insp.name) viewer.setFocus(null); });
        }
        body.appendChild(row);
        // keep the keyboard on the same line after the list is redrawn
        if (page.insp.focusName === c.name) setTimeout(() => { row.focus({ preventScroll: true }); }, 0);
      }
      $('.cghead', sec).addEventListener('click', () => {
        const now = body.hidden;
        body.hidden = !now; sec.classList.toggle('open', now);
        $('.cghead', sec).setAttribute('aria-expanded', String(now));
        groupOpen[g.name] = now; saveGroups();
      });
      list.appendChild(sec);
    }
  }

  // under the inspected line: what is drawn, the setup sentence, the measurements
  function inspectDetails(c, a, bad) {
    const I = G.inspect, src = I.sourceOf(c), P = page.insp;
    const status = src && !(P.files[src]) ? (P.running[src] ? `<p class="insp-note">Measuring the ${src === 'quality' ? 'modelling-quality' : src === 'stiffness' ? 'stiffness and stress' : src} field, ${src === 'stiffness' ? 'about half a minute to a few minutes' : 'about 20 s'}; the places the checker named are shown meanwhile.</p>`
      : P.loading[src] ? '<p class="insp-note">Loading the field…</p>' : '') : '';
    const d = h(`<div class="insp">${status}<div class="insp-text"></div>${checkDetails(c, bad)}<p class="insp-hint">Click again or press Esc for the plain view.</p></div>`);
    if (P.text) $('.insp-text', d).innerHTML = P.text;
    return d;
  }


  // the legend across the top of the view for a smooth field, and click-to-probe
  function fieldStateOf(f) { return page.fieldStates[f.rule || f.id] || (page.fieldStates[f.rule || f.id] = G.newState(f)); }
  function showFieldBar(fd) {
    const bar = $('#fieldbar', el);
    if (!fd) { bar.hidden = true; bar.innerHTML = ''; viewer.onFieldClick = null; return; }
    const redraw = () => G.legendBar(bar, fd.f, fd.state, (st, o) => { if (o && o.rebuild) refreshInspect(false); else { fd.set(st); if (!(o && o.live)) redraw(); } },
      { choices: fd.choices, toggle: fd.toggle, note: fd.note });
    redraw();
    viewer.onFieldClick = (hit) => {
      const v = G.valueAt(hit);
      viewer.addProbe(hit.point.clone(), v === null ? 'not judged here' : G.fmtValue(v, fd.f.unit));
    };
  }

  function toggleInspect(name) {
    const P = page.insp;
    if (P.name === name) { endInspect(); return; }
    P.name = name; P.framed = null;
    if (page.tab === 'made') setTab('verdict');
    refreshInspect(true);
  }

  function endInspect(rerender = true) {
    const P = page.insp;
    if (!P.name) return;
    P.name = null; P.text = '';
    viewer.setDfm(null); viewer.setFocus(null); showFieldBar(null);
    $('#legend', el).hidden = true; $('#legend', el).classList.remove('insp-on');
    if (P.saved) {
      if (P.saved.bolts !== undefined && P.saved.bolts !== viewer.show.bolts) setToggle('bolts', P.saved.bolts);
      if (P.saved.faces !== undefined && P.saved.faces !== viewer.show.faces) setToggle('faces', P.saved.faces);
      if (P.saved.fixed && viewer.modes.fixed !== P.saved.fixed) setLayer('fixed', P.saved.fixed);
      if (P.saved.obstacle && viewer.modes.obstacle !== P.saved.obstacle) setLayer('obstacle', P.saved.obstacle);
      // back to where the camera was, unless the reader has moved since the framing
      const n = viewer.nav, f = P.framed;
      if (P.saved.view && f && n.target.distanceTo(f.target) < 1e-6 && Math.abs(n.dist - f.dist) < 1e-6 && n.q.angleTo(f.q) < 1e-9) n.glide(P.saved.view, 280);
      P.saved = null;
    }
    P.framed = null;
    if (rerender && page.tab === 'verdict') setTab('verdict');
  }

  async function partMesh(a) {
    const P = page.insp;
    if (!a || !a.mesh) return null;
    if (P.mdKey !== a.mesh) { P.md = G.inspect.decodeDisplay(await getJSON(`data/mesh/${a.mesh}.json`)); P.mdKey = a.mesh; }
    return P.md;
  }

  // load (or, live, make) the field file a check is drawn from
  async function ensureField(a, src) {
    const P = page.insp, key = `${a.n}`;
    if (P.key !== key) { P.key = key; P.files = {}; P.loading = {}; P.running = {}; }
    if (P.files[src] || P.loading[src] || P.running[src]) return;
    const have = (a.fields || a.dfm || []).includes(src);
    if (have) {
      P.loading[src] = true;
      try { P.files[src] = await getJSON(`data/tasks/${tid}/dfm/${a.n}.${src}.json`); } catch { /* not in this copy */ }
      P.loading[src] = false;
    } else if (site.mode === 'live' && a.mesh && (site.field_sources || []).includes(src)) {
      P.running[src] = true;
      if (page.tab === 'verdict') setTab('verdict');
      try {
        const r = await fetch(`api/dfm?task=${encodeURIComponent(tid)}&n=${a.n}&process=${encodeURIComponent(src)}`, { method: 'POST' });
        const res = await r.json();
        if (res.ok) {
          a.fields = [...new Set([...(a.fields || []), src])];
          if (src !== 'quality') a.dfm = [...new Set([...(a.dfm || []), src])];
          P.files[src] = await getJSON(`data/tasks/${tid}/dfm/${a.n}.${src}.json`);
        }
      } catch { /* the places stay shown */ }
      P.running[src] = false;
    } else return;
    if (page.selected === a && P.name) refreshInspect(false);
  }

  // draw the inspected check for the shown attempt; frame = move the camera to the worst place
  async function refreshInspect(frame) {
    const P = page.insp, a = page.selected;
    const c = a && P.name && withStress(a).find((x) => x.name === P.name);
    if (!c) { if (P.name && !a) endInspect(); else if (P.name) { viewer.setDfm(null); $('#legend', el).hidden = true; } return; }
    const I = G.inspect, src = I.sourceOf(c);
    const md = await partMesh(a).catch(() => null);
    if (page.selected !== a || P.name !== c.name) return;
    if (src) ensureField(a, src);
    // processes are drawn with their setup even for the quality checks' sake: no
    const files = { ...P.files };
    const insp = I.inspection({ c, att: a, checks: a.checks || [], scene, pose: viewer.pose || 0, tol, md, files, fieldStates: fieldStateOf,
      colours: viewer.col, regions: viewer.regions, live: site.mode === 'live' });
    if (!P.saved) P.saved = { bolts: viewer.show.bolts, faces: viewer.show.faces, fixed: viewer.modes.fixed, obstacle: viewer.modes.obstacle,
      view: { target: viewer.nav.target.clone(), q: viewer.nav.q.clone(), dist: viewer.nav.dist } };
    // layers this check needs; put back on exit
    if (insp.ghostFixed && viewer.modes.fixed === 'solid') setLayer('fixed', 'ghost');
    if (!insp.ghostFixed && P.saved.fixed && viewer.modes.fixed !== P.saved.fixed) setLayer('fixed', P.saved.fixed);
    if (insp.showBolts && !viewer.show.bolts) setToggle('bolts', true);
    if (insp.showFaces && !viewer.show.faces) setToggle('faces', true);
    if (insp.obstacles && viewer.modes.obstacle !== insp.obstacles) setLayer('obstacle', insp.obstacles);
    viewer.setDfm(insp.group, insp.labels, { keepPart: insp.keepPart, hideBoltLabels: insp.hideBoltLabels });
    showFieldBar(insp.field || null);
    viewer.setFocus(insp.focusBolts || null, !!insp.focusRegions);
    const leg = $('#legend', el);
    leg.innerHTML = insp.legend || ''; leg.hidden = !insp.legend; leg.classList.add('insp-on');
    P.text = insp.text || '';
    const t = $(`.check.active .insp-text`, el);
    if (t) t.innerHTML = P.text;
    const note = $(`.check.active .insp-note`, el);
    if (note && P.files[src]) note.remove();
    if (frame && insp.frame) {
      const n = viewer.nav, dist = Math.min(n.dist, Math.max(insp.frame.r * 2.2, 30));
      const to = { target: insp.frame.at.clone(), q: n.q.clone(), dist };
      n.glide(to, 420);
      P.framed = { target: to.target.clone(), q: to.q.clone(), dist: to.dist };
    }
    if (frame && page.tab === 'verdict') setTab('verdict');
  }

  // ---- manufacturing check ("made how?"): drawn from the dfm field file
  function madeTab() {
    const a = page.selected;
    const procs = (site.dfm_processes && site.dfm_processes.length ? site.dfm_processes : null) || (a && a.dfm) || [];
    const D = page.dfm;
    if (!a) return h('<div><p class="note">Select an attempt first.</p></div>');
    if (!procs.length) return h('<div><p class="note dim">No manufacturing check is available here.</p></div>');
    if (!D.process || !procs.includes(D.process)) D.process = (a.dfm && a.dfm[0]) || procs[0];
    const key = `${a.n}.${D.process}`;
    if (D.key !== key) { D.key = key; D.file = null; D.field = null; showField(); loadDfm(a, D.process); } else showField();
    return G.dfmPanel({
      file: D.file, process: D.process, processes: procs, current: D.field,
      available: a.mesh ? [] : null,
      canRun: site.mode === 'live' && !!a.mesh, running: D.running, loading: !D.file && (a.dfm || []).includes(D.process),
      onProcess: (p) => { D.process = p; setTab('made'); },
      onRun: () => runDfm(a, D.process),
      onField: (f) => { D.field = f; showField(); setTab('made'); },
    });
  }

  async function loadDfm(a, proc) {
    if (!(a.dfm || []).includes(proc)) return;
    try {
      const f = await getJSON(`data/tasks/${tid}/dfm/${a.n}.${proc}.json`);
      if (page.dfm.key !== `${a.n}.${proc}`) return;
      page.dfm.file = f;
      const firstFail = f.fields.find((x) => x.status === 'fail');
      const want = page.dfm.wantField && f.fields.find((x) => x.id === page.dfm.wantField);
      page.dfm.field = (want || firstFail || f.fields[0] || {}).id || null;
      showField();
      if (page.tab === 'made') setTab('made');
    } catch { /* not present in this copy */ }
  }

  async function runDfm(a, proc) {
    page.dfm.running = true; setTab('made');
    try {
      const r = await fetch(`api/dfm?task=${encodeURIComponent(tid)}&n=${a.n}&process=${encodeURIComponent(proc)}`, { method: 'POST' });
      const res = await r.json();
      if (!res.ok) throw new Error(res.error || r.status);
      a.dfm = [...new Set([...(a.dfm || []), proc])];
      page.dfm.key = null;
    } catch (e) { alert(`The manufacturing check failed: ${e.message}`); }
    page.dfm.running = false;
    if (page.tab === 'made') setTab('made');
  }

  function showField() {
    const D = page.dfm, leg = $('#legend', el);
    const f = D.file && D.field && D.file.fields.find((x) => x.id === D.field);
    if (!f || page.tab !== 'made') {
      viewer.setDfm(null); leg.hidden = true;
      if (!page.insp.name) showFieldBar(null);
      if (D.fixedWas) { setLayer('fixed', D.fixedWas); D.fixedWas = null; }
      return;
    }
    // the fixed parts are ghosted while a field is shown, so the part is seen whole
    if (!D.fixedWas) { D.fixedWas = viewer.modes.fixed; if (viewer.modes.fixed === 'solid') setLayer('fixed', 'ghost'); }
    const cs = getComputedStyle(document.documentElement);
    const c = (n) => cs.getPropertyValue(n).trim();
    const state = fieldStateOf(f);
    const sf = G.inspect.smoothField(D.file, f, viewer.col, state);
    const { group, labels } = sf;
    // how it would be made: build direction, the bed and the face it stands on (or pull and parting plane)
    const setup = G.inspect.processSetup({ checks: (page.selected && page.selected.checks) || [], file: D.file, md: null, process: D.file.process, colours: viewer.col });
    if (setup) { group.add(setup.group); labels.push(...setup.labels); }
    viewer.setDfm(group, labels);
    showFieldBar({ f, state, set: sf.set });
    leg.innerHTML = setup ? `<div class="legend lg-setup">${setup.text}</div>` : '';
    leg.hidden = !setup;
  }

  function timelineView() {
    return timelineTab({
      attempts: page.attempts, selected: page.selected, tid,
      compare: page.compare, who: page.who, playing: page.playing,
      go: (n) => { history.replaceState(null, '', `#/task/${encodeURIComponent(tid)}/${n}?tab=timeline`); showAttempt(n, false); },
      setCompare: (on) => { page.compare = on; showPrev(); },
      setWho: (w) => { page.who = w; },
      setPlaying: (on, steps) => {
        page.playing = on;
        clearInterval(page.timer);
        if (on) {
          const advance = () => {
            const i = steps.findIndex((x) => page.selected && x.n === page.selected.n);
            if (i < 0 || i >= steps.length - 1) { page.playing = false; clearInterval(page.timer); setTab('timeline'); return; }
            history.replaceState(null, '', `#/task/${encodeURIComponent(tid)}/${steps[i + 1].n}?tab=timeline`);
            showAttempt(steps[i + 1].n, false);
          };
          const i = steps.findIndex((x) => page.selected && x.n === page.selected.n);
          if (i >= steps.length - 1) showAttempt(steps[0].n, false);
          page.timer = setInterval(advance, 1800);
        }
        setTab('timeline');
      },
    });
  }

  async function showPrev() {
    const p = page.compare ? previousStep(page.attempts, page.selected) : null;
    if (p && p.mesh) {
      try { viewer.setBody('prev', await getJSON(`data/mesh/${p.mesh}.json`)); return; } catch { /* fall through */ }
    }
    viewer.setBody('prev', null);
  }

  function attemptsTab() {
    const wrap = h(`<div>
      ${site.mode === 'live' ? '<label class="follow"><input type="checkbox" id="follow"> follow new attempts as they arrive</label>' : ''}
      <ul class="alist" id="alist"></ul></div>`);
    const f = $('#follow', wrap);
    if (f) { f.checked = page.follow; f.addEventListener('change', () => { page.follow = f.checked; }); }
    fillAttempts($('#alist', wrap));
    if (!page.attempts.length) wrap.appendChild(h('<p class="note">No attempts yet.</p>'));
    return wrap;
  }

  function fillAttempts(ul, fresh = new Set()) {
    if (!ul) return;
    ul.innerHTML = '';
    for (const a of [...page.attempts].reverse()) {
      const li = h(`<li class="${page.selected && page.selected.n === a.n ? 'on' : ''} ${fresh.has(a.n) ? 'fresh' : ''}">
        <span class="an">#${a.n}</span>
        <span class="aw">${esc(a.by)}<small>${esc(ago(a.time))}${a.mesh ? '' : ' · no shape'}</small></span>
        <span>${dots(a.checks)}${stamp(a.passed)}</span></li>`);
      li.addEventListener('click', () => { location.hash = `#/task/${tid}/${a.n}`; });
      ul.appendChild(li);
    }
    $('#acount', el).textContent = page.attempts.length ? ` ${page.attempts.length}` : '';
  }

  function briefTab() {
    return h(`<div class="brief">${markdown(scene.brief || '')}</div>`);
  }

  function tryTab() {
    const files = scene.files.map((f) => `<li><span>${esc(f)}</span><a href="data/tasks/${esc(tid)}/files/${esc(f)}" download>download</a></li>`).join('');
    if (site.mode !== 'live') {
      return h(`<div>
        <p class="note">Model the part in any CAD program from these files. Units are millimetres; the fixed parts are in world coordinates.</p>
        <ul class="files">${files}</ul>
        <div class="h3">Viewing a saved copy</div>
        <p class="note">Uploads are not graded here: this is a saved copy of the site. To have a STEP file graded by the same grader the agents face, run it on your computer:</p>
        <div class="note"><pre>uv sync
uv run jointbench gen --seed ${scene.seed} --level ${scene.level} --out tasks
uv run jointbench play tasks/${esc(tid)}</pre></div></div>`);
    }
    // Try it yourself, live: three steps. A person's tries are kept as a run
    // of their own (human-<name>), never in this task's log; the server
    // measures the time from starting the clock to the first passing upload.
    const keep = (k, v) => { try { if (v === undefined) return localStorage.getItem(k) || ''; localStorage.setItem(k, v); } catch { return ''; } return v; };
    const steps = scene.files.filter((f) => /\.(step|stp)$/i.test(f));
    const other = scene.files.filter((f) => !steps.includes(f));
    const fileLink = (f) => `<li><span>${esc(f)}</span><a class="dl" href="data/tasks/${esc(tid)}/files/${esc(f)}" download>download</a></li>`;
    const wrap = h(`<div class="tryit">
      <p class="note">Model the part yourself in any CAD program. Your tries are graded by the same grader as the agents and kept as your own run on the scores page, with the time you took.</p>
      <ol class="steps">
        <li><b>Download the parts</b>
          <div class="who-row">
            <label class="field">your name<input id="who" maxlength="40" placeholder="as it should appear on the scores page"></label>
            <label class="field">your CAD program<input id="tool" maxlength="40" placeholder="e.g. FreeCAD"></label>
          </div>
          <ul class="files" id="stepfiles">${steps.map(fileLink).join('')}</ul>
          <details class="more"><summary>brief and task files</summary><ul class="files">${other.map(fileLink).join('')}</ul></details>
          <p class="note" id="clock">Your clock starts with your first download.</p></li>
        <li><b>Model the part</b>
          <p class="note">One closed solid that bolts to every fixed part${scene.poses.some((p) => p.obstacles && p.obstacles.length) ? ' and stays clear of the obstacles' : ''}. Millimetres, in the parts' own world coordinates: import the STEP files as they are and do not move them. The <a href="#" data-tab-link="brief">brief</a> lists the bolts and what is measured.</p></li>
        <li><b>Upload your STEP</b>
          <div class="drop" id="drop"><b>Drop a .step file</b><small>or click to choose · graded on this computer, by the same grader as the agents</small>
            <input type="file" id="file" accept=".step,.stp,.STEP,.STP" hidden></div>
          <div class="status" id="ustatus"></div>
          <p class="note">See the verdict with its marks in the drawing, fix, and upload again: every try is kept and counted.</p></li>
      </ol>
    </div>`);
    const who = $('#who', wrap), tool = $('#tool', wrap), clockEl = $('#clock', wrap), st = $('#ustatus', wrap);
    // on a person's own copy of the task, the name is theirs already
    const own = tid.startsWith('human-') ? page.attempts.find((a) => a.by) : null;
    who.value = (own && own.by) || keep('jb-who');
    tool.value = (own && own.tool) || keep('jb-tool');
    const ready = () => !!who.value.trim();
    const sync = () => { $('#stepfiles', wrap).classList.toggle('locked', !ready()); if (!ready() && !page.clock) clockEl.textContent = 'Enter your name first: it keeps your tries apart from everyone else\'s, and starts your clock with your first download.'; };
    who.addEventListener('input', () => { keep('jb-who', who.value.trim()); sync(); });
    tool.addEventListener('input', () => keep('jb-tool', tool.value.trim()));
    const fmtDur = (s) => (s >= 3600 ? `${(s / 3600).toFixed(1)} h` : s >= 60 ? `${Math.round(s / 60)} min` : `${Math.round(s)} s`);
    const showClock = (c) => {
      if (!c || !c.started) return;
      page.clock = c;
      const t0 = new Date(c.started);
      clockEl.innerHTML = c.wall_seconds !== undefined
        ? `Clock: <b>first pass after ${fmtDur(c.wall_seconds)}</b> (try ${c.first_pass}), measured by the server from your first download.`
        : `Clock started at ${t0.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}; ${fmtDur((Date.now() - t0) / 1000)} so far. It stops at your first passing upload.`;
    };
    const qs = () => `task=${encodeURIComponent(tid)}&who=${encodeURIComponent(who.value.trim())}&tool=${encodeURIComponent(tool.value.trim())}`;
    async function startClock() {
      if (!ready()) return false;
      try {
        const r = await fetch(`api/start?${qs()}`, { method: 'POST' });
        const j = await r.json();
        if (r.ok) showClock(j.clock);
      } catch { /* the download still works; the time is then missing, never guessed */ }
      return true;
    }
    wrap.addEventListener('click', (e) => {
      const a = e.target.closest('a.dl');
      if (a && !ready()) { e.preventDefault(); who.focus(); sync(); return; }
      if (a) startClock();
      const t = e.target.closest('[data-tab-link]');
      if (t) { e.preventDefault(); setTab(t.dataset.tabLink); }
    });
    if (page.clock) showClock(page.clock);
    else if (ready()) fetch(`api/clock?task=${encodeURIComponent(tid)}&who=${encodeURIComponent(who.value.trim())}`).then((r) => r.json()).then((j) => { if (j.clock) showClock(j.clock); }).catch(() => {});
    sync();
    const tick = setInterval(() => { if (!wrap.isConnected) clearInterval(tick); else if (page.clock && page.clock.wall_seconds === undefined) showClock(page.clock); }, 30000);
    // the result of the last upload, carried over to the person's own task page
    try { const last = JSON.parse(sessionStorage.getItem('jb-last') || 'null'); if (last && last.key === tid) { st.innerHTML = last.html; showClock(last.clock); sessionStorage.removeItem('jb-last'); } } catch { /* nothing to show */ }

    const drop = $('#drop', wrap), input = $('#file', wrap);
    drop.addEventListener('click', () => input.click());
    drop.addEventListener('dragover', (e) => { e.preventDefault(); drop.classList.add('over'); });
    drop.addEventListener('dragleave', () => drop.classList.remove('over'));
    drop.addEventListener('drop', (e) => { e.preventDefault(); drop.classList.remove('over'); if (e.dataTransfer.files[0]) upload(e.dataTransfer.files[0]); });
    input.addEventListener('change', () => { if (input.files[0]) upload(input.files[0]); });
    async function upload(file) {
      if (!ready()) { who.focus(); sync(); st.className = 'status err'; st.textContent = 'Enter your name first.'; return; }
      st.className = 'status';
      st.innerHTML = `<span class="spin"></span>grading ${esc(file.name)} (${(file.size / 1024).toFixed(0)} KB)…`;
      try {
        const r = await fetch(`api/upload?${qs()}`, { method: 'POST', body: file });
        const rec = await r.json();
        if (!r.ok) throw new Error(rec.error || r.status);
        const failed = (rec.checks || []).filter((c) => c.status === 'fail' && c.required !== false).map((c) => c.name);
        const html = rec.passed
          ? `<b class="ok-t">Pass</b> on try ${rec.n}. Every required check passed; see the measurements in the verdict.`
          : `<b class="bad-t">Fail</b> on try ${rec.n}: ${failed.map((n) => `<code>${esc(n)}</code>`).join(', ') || 'see the verdict'}. The failing bolts and overlaps are marked in the drawing; fix them and upload again.`;
        st.innerHTML = html;
        try { sessionStorage.setItem('jb-last', JSON.stringify({ key: rec.key, html, clock: rec.clock })); } catch { /* shown here only */ }
        if (rec.key === tid) { showClock(rec.clock); await refreshAttempts(); location.hash = `#/task/${tid}/${rec.n}?tab=try`; }
        else location.hash = `#/task/${encodeURIComponent(rec.key)}/${rec.n}?tab=try`;
      } catch (e) {
        st.className = 'status err';
        st.textContent = `could not grade: ${e.message}`;
      }
    }
    return wrap;
  }

  // ---- attempts
  let token = 0;
  // tab: which tab to show ('verdict' when true, the current tab when false,
  // or a tab name). The panel is filled at once; the shape loads after it,
  // so a slow mesh never leaves the panel blank.
  async function showAttempt(n, tab = true) {
    const a = page.attempts.find((x) => x.n === n) || null;
    page.selected = a;
    const my = ++token;
    viewer.setVerdict(a, tol);
    viewer.setMarks([]);
    const want = tab === true ? 'verdict' : tab === false ? (page.tab || 'verdict') : tab;
    if (want === 'attempts' && page.tab === 'attempts') fillAttempts($('#alist', el));
    else setTab(want);
    showPrev();
    if (a && a.marks) {
      getJSON(`data/tasks/${tid}/marks/${a.n}.json`).then((m) => { if (my === token) { viewer.setMarks(m.regions); if (page.insp.name === 'NO_INTERFERENCE') refreshInspect(false); } }).catch(() => {});
    }
    if (a && a.mesh) {
      try {
        const m = await getJSON(`data/mesh/${a.mesh}.json`);
        if (my !== token) return;
        viewer.setBody('part', m);
        // the first answer shown on this page: frame it with its mounts, from the clearest side
        // (not when the link sets a view)
        if (!page.framedAnswer && !(page.params && (page.params.get('view') || page.params.get('cam')))) { page.framedAnswer = viewer.frameAnswer(); }
      } catch { if (my === token) viewer.setBody('part', null); }
    } else viewer.setBody('part', null);
    if (my === token && page.insp.name) refreshInspect(false);
  }

  async function refreshAttempts() {
    const before = new Set(page.attempts.map((a) => a.n));
    page.attempts = await getJSON(`data/tasks/${tid}/attempts.json`);
    const fresh = new Set(page.attempts.filter((a) => !before.has(a.n)).map((a) => a.n));
    if (page.tab === 'attempts') fillAttempts($('#alist', el), fresh);
    else $('#acount', el).textContent = ` ${page.attempts.length}`;
    return fresh;
  }

  page.route = async (n, q) => {
    const tab = q && q.get('tab');
    if (n !== undefined && n !== null) { await showAttempt(n, tab || true); return; }
    const last = page.attempts[page.attempts.length - 1];
    if (last) await showAttempt(last.n, true);
    else setTab('brief');
  };

  if (site.mode === 'live') {
    page.es = new EventSource(`api/events?task=${encodeURIComponent(tid)}`);
    page.es.addEventListener('attempts', async () => {
      const fresh = await refreshAttempts();
      if (page.follow && fresh.size) {
        const n = Math.max(...fresh);
        history.replaceState(null, '', `#/task/${tid}/${n}`);
        await showAttempt(n, page.tab !== 'attempts');
        if (page.tab === 'attempts') fillAttempts($('#alist', el), fresh);
      }
    });
  }

  $('#acount', el).textContent = page.attempts.length ? ` ${page.attempts.length}` : '';
  await page.route(wantN);
  applyParams(params);

  // view settings carried in the link, e.g. #/task/L1-s0/2?fixed=ghost&view=front&sec=y,0.4&tab=brief
  function applyParams(q) {
    if (!q) return;
    for (const l of ['fixed', 'part', 'ref', 'obstacle']) if (q.get(l)) setLayer(l, q.get(l) === 'off' ? 'hidden' : q.get(l));
    for (const t of ['faces', 'bolts']) if (q.get(t)) setToggle(t, q.get(t) !== 'off');
    if (q.get('view')) setView(q.get('view'));
    if (q.get('sec')) {
      const [ax, t] = q.get('sec').split(',');
      if (['x', 'y', 'z'].includes(ax)) viewer.clip.addAxis(ax, Math.min(1, Math.max(0, parseFloat(t) || 0.5)));
    }
    if (q.get('box')) viewer.clip.setBox(true);
    if (q.get('secpanel')) toggleSection(true);
    if (q.get('proj') === 'parallel' && !viewer.nav.parallel) viewer.nav.toggleProjection();
    if (q.get('cmp')) { page.compare = true; showPrev(); }
    if (q.get('proc')) page.dfm.process = q.get('proc');
    if (q.get('field')) page.dfm.wantField = q.get('field');
    if (q.get('tab')) setTab(q.get('tab'));
    if (q.get('dl')) toggleDownloads(true);
    if (q.get('pose') && pose) { pose.value = q.get('pose'); pose.dispatchEvent(new Event('input')); }
  }
  return {
    tid, viewer, page, route: page.route,
    retheme() { if (page.insp.name) refreshInspect(false); else if (page.tab === 'made') showField(); },
    destroy() {
      clearInterval(page.timer); clearInterval(page.poseTimer); if (page.es) page.es.close(); viewer.destroy();
      window.removeEventListener('keydown', page.keys); clearInterval(page.syncProj);
    },
  };
}

function fmtVal(v, unit = '') {
  if (typeof v !== 'number') return esc(v) + ' ' + esc(unit);
  if (unit === 'ratio') return `${(v * 100).toFixed(0)} %`;
  if (unit === 'mm3') return `${v >= 100 ? v.toFixed(1) : v.toFixed(4)} mm³`;
  return `${unit === "mm" ? v.toFixed(3) : fmtNum(v)} ${esc(unit)}`;
}

function fmtNum(v) {
  if (Number.isInteger(v)) return String(v);
  return Math.abs(v) >= 100 ? v.toFixed(1) : v.toFixed(3);
}

function boltIdsOf(c) {
  const d = c.details || {};
  const ids = new Set();
  for (const r of d.holes || []) ids.add(r.bolt);
  for (const r of d.bolts || []) ids.add(r.bolt);
  if (c.status === 'fail') {
    for (const p of d.problems || []) { const m = /^([A-Za-z]+\d+):/.exec(p); if (m) ids.add(m[1]); }
    for (const r of d.overlaps || []) { const m = /^bolt (\S+) /.exec(r.what || ''); if (m && r.volume > 0.01) ids.add(m[1]); }
  }
  // for a passing check, highlighting every bolt says nothing; only a fail focuses
  return c.status === 'fail' ? [...ids].filter((id) => (d.problems || []).some((p) => p.startsWith(id + ':')) ||
    (d.overlaps || []).some((r) => r.what.startsWith(`bolt ${id} `) && r.volume > 0.01)) : null;
}

function checkDetails(c, bad) {
  const d = c.details || {};
  let inner = '';
  if (d.problems && d.problems.length > 1) inner += `<ul class="probs">${d.problems.map((p) => `<li>${esc(p)}</li>`).join('')}</ul>`;
  if (d.holes && d.holes.length) {
    inner += `<table class="dtable"><tr><th>bolt</th><th class="r">axis off mm</th><th class="r">dia mm</th><th class="r">grip mm</th></tr>${
      d.holes.map((r) => `<tr class="${bad[r.bolt] ? 'bad' : ''}"><td>${esc(r.bolt)}</td><td class="r">${r.offset.toFixed(3)}</td><td class="r">${r.diameter.toFixed(2)}</td><td class="r">${r.grip.toFixed(2)}</td></tr>`).join('')}</table>`;
  }
  if (d.bolts && d.bolts.length) {
    inner += `<table class="dtable"><tr><th>bolt</th><th class="r">ring in contact</th></tr>${
      d.bolts.map((r) => `<tr class="${r.contact < 0.75 ? 'bad' : ''}"><td>${esc(r.bolt)}</td><td class="r">${(r.contact * 100).toFixed(0)} %</td></tr>`).join('')}</table>`;
  }
  if (d.overlaps && d.overlaps.length) {
    const rows = d.overlaps.filter((r) => r.volume > 0 || c.status === 'pass');
    inner += `<table class="dtable"><tr><th>between</th><th class="r">overlap mm³</th></tr>${
      (rows.length ? rows : d.overlaps).map((r) => `<tr class="${r.volume > 0.01 ? 'bad' : ''}"><td>${esc(r.what)}</td><td class="r">${r.volume.toFixed(r.volume >= 100 ? 1 : 4)}</td></tr>`).join('')}</table>`;
  }
  if (!inner) return '';
  return `<details ${c.status === 'fail' ? 'open' : ''}><summary>measurements</summary>${inner}</details>`;
}


// ---------------------------------------------------------------- router

async function route() {
  const [hash, query] = location.hash.replace(/^#\/?/, '').split('?');
  const params = new URLSearchParams(query || '');
  const [view, a0, b] = hash.split('/');
  const a = a0 ? decodeURIComponent(a0) : a0;
  for (const n of document.querySelectorAll('.top nav a')) n.classList.toggle('on', n.dataset.nav === (view || 'tasks'));
  try {
    if (view === 'task' && a) {
      const n = b ? parseInt(b, 10) : null;
      if (current && current.tid === a) { await current.route(n, params); return; }
      if (current) current.destroy();
      current = null;
      app.innerHTML = '<div class="empty-note" style="position:static;padding:80px 0">loading…</div>';
      current = await renderTask(a, n, params);
      return;
    }
    if (current) current.destroy();
    if (view === 'compare') { await load3d(); current = a ? await G.renderComparePage(app, a, params) : await G.renderCompareGrid(app, params); return; }
    if (view === 'scores') { current = await renderScores(app, a || null, Object.fromEntries(params)); return; }
    if (view === 'about') { current = await renderAbout(app); return; }
    current = await renderOverview(null);
  } catch (e) {
    app.replaceChildren(h(`<div class="page"><div class="section-title"><h2>Could not load</h2></div><p class="note mono">${esc(e.message)}</p><p><a href="#/">back to the tasks</a></p></div>`));
  }
}

// for inspection from the browser console (and the page tests): the open page's 3D view
Object.defineProperty(window, 'jointbench', { value: { get viewer() { return current && current.viewer; }, get page() { return current && current.page; } } });

async function start() {
  try { site = await getJSON('data/site.json'); } catch { site = { mode: 'static' }; }
  if (!location.hash && site.single) location.hash = `#/task/${site.single}`;
  window.addEventListener('hashchange', route);
  route();
}

start();
