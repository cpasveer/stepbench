// Comparing models on the same tasks.
//
// Rules (stated on the page):
// * A model's answer to a task is its *submission*: the first passing
//   attempt within the allowed tries, or, if none passed, its last allowed
//   attempt -- never the best attempt picked afterwards (score.py).
// * A model with several runs of a task is shown by its *typical* run: the
//   median by tries to pass (a run that never passed counts as the most
//   tries; with an even count the less flattering middle run), ties broken
//   by the earlier run. All runs stay reachable, with
//   how many there are and how many passed. Never only the best run.
//
// Two pages:
// * the grid (per level): tasks down, models across; each cell a small 3D
//   picture of the submission with the fixed parts see-through, pass or
//   fail and tries. Tasks that separate the models come first.
// * the compare page (per task): two to four submissions side by side,
//   each in its own 3D view; cameras and section cuts are linked, so all
//   views move together. Or two submissions overlaid in one scene.

import { Viewer } from './viewer.js';
import { thumb, thumbsAvailable } from './thumbs.js';
import { materialRatio, fmtRatio } from './quality.js';
import { ICONS } from './icons.js';
import { getJSON } from './fetch.js';
import { devNotice, noResults, devTag, isDevRun } from './rules.js';
import * as I from './inspect.js';
import * as FV from './fieldview.js';
import { ladderHTML } from './ladder.js';
import { ladderChart } from './charts.js';

// the processes a reader can judge every part by; "as graded" is the task's own
const PROCESSES = [['graded', 'as graded'], ['fdm', 'filament (FDM)'], ['pbf', 'powder (SLS)'], ['moulding', 'moulding']];
// the material each process file states ([material] in src/jointbench/dfm/processes/*.toml)
const MATERIAL = { fdm: ['PLA', 1.24], pbf: ['PA12', 1.01], moulding: ['ABS', 1.05] };

const $ = (sel, el = document) => el.querySelector(sel);
const h = (html) => { const t = document.createElement('template'); t.innerHTML = html.trim(); return t.content.firstElementChild; };
const esc = (s) => String(s ?? '').replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
const NONE = '<span class="missing" title="not reported">—</span>';


// ---- models, tasks, typical runs --------------------------------------------

function triesToPass(t, run) {
  const s = t.submission;
  if (!s) return Infinity;
  return s.passed ? s.n : (run.tries_allowed || t.attempts || 99) + 1;
}

// {models: [name], tasks: [{id, level, cells: {model: cell}}]}
export function compareTable(scores) {
  const models = [], byKey = new Map();
  scores.runs.forEach((run, order) => {
    if (!models.includes(run.model)) models.push(run.model);
    for (const t of run.tasks) {
      if (!t.attempts) continue;
      const k = `${t.id}\u0000${run.model}`;
      if (!byKey.has(k)) byKey.set(k, []);
      byKey.get(k).push({ run, t, order, tries: triesToPass(t, run) });
    }
  });
  const tasks = new Map();
  for (const [k, entries] of byKey) {
    const [id, model] = k.split('\u0000');
    entries.sort((a, b) => a.tries - b.tries || a.order - b.order);
    // the median run; with an even count the less flattering of the two
    // middle runs, so the choice never leans toward the best run
    const med = entries[Math.floor(entries.length / 2)].tries;
    const typical = entries.find((x) => x.tries === med);  // ties: the earlier run
    if (!tasks.has(id)) tasks.set(id, { id, level: typical.t.level, cells: {} });
    tasks.get(id).cells[model] = {
      typical, runs: entries.length, passed: entries.filter((e) => e.t.submission && e.t.submission.passed).length, all: entries,
    };
  }
  return { models, tasks: [...tasks.values()] };
}

// the tasks that separate the models come first:
//   0. some passed and some did not (most evenly split first);
//   1. all passed, in different numbers of tries (widest spread first);
//   2. none passed;
//   3. all passed in the same number of tries.
function separation(task, models) {
  const ms = models.filter((m) => task.cells[m]);
  const sub = (m) => task.cells[m].typical.t.submission;
  const p = ms.filter((m) => sub(m) && sub(m).passed).length;
  if (p > 0 && p < ms.length) return [0, Math.abs(p - ms.length / 2)];
  if (p === 0) return [2, 0];
  const tries = ms.map((m) => task.cells[m].typical.tries);
  const spread = Math.max(...tries) - Math.min(...tries);
  return spread > 0 ? [1, -spread] : [3, 0];
}

const fmtSec = (s) => (s >= 60 ? `${(s / 60).toFixed(1)} min` : `${Math.round(s)} s`);
const fmtTok = (t) => (t >= 1e3 ? `${(t / 1e3).toFixed(1)} k` : `${t}`);

// ---- the grid -----------------------------------------------------------------

export async function renderCompareGrid(app, params) {
  let scores;
  try { scores = await getJSON('data/scores.json'); } catch {
    app.replaceChildren(h('<div class="page"><div class="section-title"><h2>Compare</h2></div><p class="note">No model runs are loaded. Start <code>jointbench play &lt;runs folder&gt;</code> or export with <code>jointbench site &lt;runs folder&gt;</code>.</p></div>'));
    return { destroy() {} };
  }
  if (!scores.runs.length) { app.replaceChildren(h(noResults(scores, 'Compare'))); return { destroy() {} }; }
  const { models, tasks } = compareTable(scores);
  const levels = [...new Set(tasks.map((t) => t.level))].sort();
  const level = params.get('level') || String(levels[0] ?? '');
  const shown = tasks.filter((t) => String(t.level) === level)
    .sort((a, b) => { const x = separation(a, models), y = separation(b, models); return x[0] - y[0] || x[1] - y[1] || a.id.localeCompare(b.id, undefined, { numeric: true }); });
  const el = h(`<div class="page compare">
    <div class="section-title"><h2>Compare</h2><span class="n">${shown.length} tasks · ${models.length} models · level ${esc(level)}</span>
      <span class="r">${levels.length > 1 ? `<span class="seg">${levels.map((l) => `<a class="segl ${String(l) === level ? 'on' : ''}" href="#/compare?level=${l}">level ${l}</a>`).join('')}</span>` : ''}</span></div>
    ${devNotice(scores, scores.runs)}
    <p class="prose">Each cell is the model's <b>submission</b>: its first passing attempt within the allowed tries, or its last attempt if none passed. Where a model ran a task more than once, the cell shows its <b>typical run</b> (the median by tries) and says how many runs passed. Tasks that separate the models come first: where some passed and some did not, then where all passed but in different numbers of tries. Click a cell to compare the submissions in 3D.</p>
    <div class="tablewrap"><table class="cgrid" style="--n:${models.length}"><thead><tr><th>task</th>${models.map((m) => `<th>${esc(m)}</th>`).join('')}</tr></thead><tbody></tbody></table></div>
  </div>`);
  const tb = $('tbody', el);
  for (const t of shown) {
    const tr = h(`<tr><th class="tname"><a href="#/compare/${encodeURIComponent(t.id)}"><b>${esc(t.id)}</b></a></th></tr>`);
    for (const m of models) {
      const c = t.cells[m];
      if (!c) { tr.appendChild(h('<td class="cell none"><span class="dim">not attempted</span></td>')); continue; }
      const { run, t: tt } = c.typical, s = tt.submission;
      const key = tt.key;
      const td = h(`<td class="cell"><a class="cg" href="#/compare/${encodeURIComponent(t.id)}?focus=${encodeURIComponent(m)}">
        <div class="cthumb"></div>
        <div class="cmeta"><span class="stamp ${s.passed ? 'ok' : 'bad'}">${s.passed ? 'pass' : 'fail'}</span>
          <span class="num">${s.passed ? `try ${s.n}` : `${tt.attempts} tries`}</span>
          ${s.passed ? '' : `<code class="bad-t">${esc((s.failed || [])[0] || '')}</code>`}</div>
        ${c.runs > 1 ? `<div class="cruns dim">typical of ${c.runs} runs · ${c.passed} passed</div>` : ''}
      </a></td>`);
      if (thumbsAvailable()) {
        $('.cthumb', td).appendChild(thumb({ fixed: `data/tasks/${encodeURIComponent(key)}/thumb.json`, part: `data/tasks/${encodeURIComponent(key)}/thumb/${s.n}.json` }));
      }
      void run;
      tr.appendChild(td);
    }
    tb.appendChild(tr);
  }
  // how far up the requirement ladder each model's parts get, on the tasks shown
  const lad = models.map((m) => {
    const rungs = [];
    for (const t of shown) {
      const c = t.cells[m], L = c && c.typical.t.submission && c.typical.t.submission.ladder;
      if (!L) continue;
      if (!rungs.length) L.forEach((x) => rungs.push({ label: x.label, reached: 0, measured: 0, required: 0 }));
      let ok = true;
      L.forEach((x, i) => {
        if (x.required) rungs[i].required++;
        if (x.state === 'not measured') return;
        rungs[i].measured++;
        if (x.state === 'met') { if (ok) rungs[i].reached++; } else ok = false;
      });
    }
    return { label: m, rungs };
  }).filter((x) => x.rungs.length);
  if (lad.length) {
    el.appendChild(h(`<div class="section-title"><h2>How far up the ladder</h2><span class="n">level ${esc(level)}: share of each model's parts reaching each rung</span></div>`));
    el.appendChild(ladderChart(lad));
  }
  app.replaceChildren(el);
  window.scrollTo(0, 0);
  return { destroy() {} };
}

// ---- the compare page -------------------------------------------------------------

export async function renderComparePage(app, taskId, params) {
  const scores = await getJSON('data/scores.json');
  const { models, tasks } = compareTable(scores);
  // process versions: the same challenge made for several processes (ids end
  // in -fdm, -pbf, -moulding). ?v=<model or run> shows that model's versions
  // side by side, each part with its own build or pull direction drawn.
  // the version record the scores file carries per task (rules.task_version: "situation", else the id's suffix)
  const SUF = /-(fdm|pbf|moulding)$/;
  const verOf = (t) => { const c = Object.values(t.cells)[0]; return (c && c.typical.t.version) || { base: t.id.replace(SUF, ''), process: (SUF.exec(t.id) || [])[1], versioned: SUF.test(t.id) }; };
  const here = tasks.find((t) => t.id === taskId);
  const baseOf = (id) => { const t = tasks.find((x) => x.id === id); return t ? verOf(t).base : id.replace(SUF, ''); };
  const siblings = tasks.filter((t) => verOf(t).versioned && verOf(t).base === (here ? verOf(here).base : taskId.replace(SUF, '')));
  const procOf = (t) => verOf(t).process || (SUF.exec(t.id) || [])[1];
  let vModel = params.get('v');
  if (vModel && !models.includes(vModel)) { const r = scores.runs.find((x) => x.id === vModel); vModel = r ? r.model : vModel; }
  let task = vModel ? null : tasks.find((t) => t.id === taskId);
  if (vModel && siblings.length) {
    const ORDER = { fdm: 0, pbf: 1, moulding: 2 };
    task = { id: verOf(siblings[0]).base, level: siblings[0].level, versions: true, cells: {}, ids: {} };
    for (const t of siblings.slice().sort((a, b) => (ORDER[procOf(a)] ?? 9) - (ORDER[procOf(b)] ?? 9))) if (t.cells[vModel]) { task.cells[procOf(t)] = t.cells[vModel]; task.ids[procOf(t)] = t.id; }
  }
  if (!task || !Object.keys(task.cells).length) { app.replaceChildren(h(`<div class="page"><p class="note">No model attempted ${esc(taskId)}. <a href="#/compare">Back to the grid</a></p></div>`)); return { destroy() {} }; }
  if (task.versions) models.splice(0, models.length, ...Object.keys(task.cells));
  const have = models.filter((m) => task.cells[m]);
  let chosen = (params.get('m') ? params.get('m').split(',') : have).filter((m) => have.includes(m)).slice(0, 4);
  if (params.get('focus') && !chosen.includes(params.get('focus'))) chosen = [params.get('focus'), ...chosen].slice(0, 4);
  const mode = params.get('mode') === 'overlay' && chosen.length >= 2 ? 'overlay' : 'side';
  const runPick = {};  // model -> chosen run index (default: typical)
  for (const m of have) { const c = task.cells[m]; runPick[m] = c.all.indexOf(c.typical); }
  if (params.get('run')) for (const kv of params.get('run').split(',')) { const [m, i] = kv.split(':'); if (task.cells[m] && task.cells[m].all[+i]) runPick[m] = +i; }

  const url = (o = {}) => {
    const q = new URLSearchParams();
    q.set('m', (o.m || chosen).join(','));
    if ((o.mode || mode) === 'overlay') q.set('mode', 'overlay');
    const rp = Object.entries(o.run || runPick).filter(([m, i]) => i !== task.cells[m].all.indexOf(task.cells[m].typical)).map(([m, i]) => `${m}:${i}`);
    if (rp.length) q.set('run', rp.join(','));
    const c = o.c !== undefined ? o.c : crit, p = o.p !== undefined ? o.p : proc;
    if (c) q.set('c', c);
    if (p && p !== 'graded') q.set('p', p);
    if (task.versions) q.set('v', vModel);
    return `#/compare/${encodeURIComponent(taskId)}?${q}`;
  };
  let crit = params.get('c') || '', proc = params.get('p') || 'graded';
  const site = await getJSON('data/site.json').catch(() => ({}));
  const live = site.mode === 'live';

  const entry = (m) => task.cells[m].all[runPick[m]];
  const el = h(`<div class="cmp${mode === 'side' ? ' has-review' : ''}">
    <div class="cmp-head">
      <div><a class="crumb" href="#/compare?level=${task.level}">Compare</a> <b class="tid">${esc(task.versions ? task.id : taskId)}</b> <span class="dim">level ${task.level}</span>${task.versions ? ` <span class="ptag-proc">${esc(vModel)}: ${Object.keys(task.cells).length} processes</span>` : ''}${scores.runs.some(isDevRun) ? devTag() : ''}
        ${!task.versions && siblings.length > 1 ? `<span class="vlinks dim">same situation, other processes: ${siblings.filter((t) => t.id !== taskId).map((t) => `<a href="#/compare/${encodeURIComponent(t.id)}">${esc(procOf(t))}</a>`).join(' · ')} · per model: ${have.map((m) => `<a href="#/compare/${encodeURIComponent(taskId)}?v=${encodeURIComponent(m)}">${esc(m)}</a>`).join(' · ')}</span>` : ''}
        ${task.versions ? `<span class="vlinks dim">one process across models: ${Object.keys(task.cells).map((p) => `<a href="#/compare/${encodeURIComponent(task.ids[p])}">${esc(p)}</a>`).join(' · ')}</span>` : ''}</div>
      <div class="cmp-models">${have.map((m) => `<button class="chip ${chosen.includes(m) ? 'on' : ''}" data-m="${esc(m)}" aria-pressed="${chosen.includes(m)}">${esc(m)}</button>`).join('')}</div>
      <div class="seg cmp-mode"><button data-mode="side" class="${mode === 'side' ? 'on' : ''}">side by side</button><button data-mode="overlay" class="${mode === 'overlay' ? 'on' : ''}" ${chosen.length < 2 ? 'disabled' : ''}>overlay</button></div>
      <div class="cmp-tools">
        <button class="rb" data-a="fit" title="Show the whole model (all views)" aria-label="Show the whole model">${ICONS.fit}</button>
        <button class="rb" data-a="proj" title="Parallel / perspective (all views)" aria-label="Parallel or perspective">${ICONS.projection}</button>
        <span class="seg">${['x', 'y', 'z'].map((a) => `<button data-cut="${a}" title="Cut across ${a.toUpperCase()} in all views">${a.toUpperCase()}</button>`).join('')}<button data-cut="clear" title="Remove the cuts">clear</button></span>
      </div>
    </div>
    ${mode === 'side' ? `<div class="cmp-review">
      <label class="sel">review by <select id="crit"></select></label>
      <span class="seg" id="procs" role="group" aria-label="process">${PROCESSES.map(([k, t]) => `<button data-proc="${k}" class="${k === proc ? 'on' : ''}" aria-pressed="${k === proc}">${t}</button>`).join('')}</span>
      <span class="dim" id="review-note"></span>
    </div>
` : ''}
    <div class="cmp-cols mode-${mode}" style="--n:${mode === 'overlay' ? 1 : chosen.length}"></div>
    <div class="cmp-info" style="--n:${chosen.length}"></div>
    ${mode === 'side' ? '<div class="tablewrap cmp-sum"><table class="cmp-table"></table></div>' : ''}
  </div>`);
  app.replaceChildren(el);
  window.scrollTo(0, 0);

  const cols = $('.cmp-cols', el);
  const viewers = [];
  const stage = (withGizmo) => {
    const st = h(`<div class="stage cmp-stage"><div class="labels"></div><svg class="overlay" aria-hidden="true"><g class="hd"></g><g class="ms"></g></svg>${withGizmo ? '<div class="tr"><div class="gizmo"></div><div class="viewlabel num"></div></div>' : ''}<div class="readout"></div></div>`);  // no scale bar: an overview
    return st;
  };
  const makeViewer = (st, first) => new Viewer(st, {
    labels: $('.labels', st), overlay: $('.overlay', st), gizmo: first ? $('.gizmo', st) : null,
    viewLabel: first ? $('.viewlabel', st) : null, scalebar: null, scaletext: null,
  });

  // per model: scene, attempt record, mesh
  const load = async (m) => {
    const e = entry(m), key = e.t.key, n = e.t.submission.n;
    const [scene, atts] = await Promise.all([getJSON(`data/tasks/${encodeURIComponent(key)}/scene.json`), getJSON(`data/tasks/${encodeURIComponent(key)}/attempts.json`)]);
    const att = atts.find((a) => a.n === n);
    const mesh = att && att.mesh ? await getJSON(`data/mesh/${att.mesh}.json`).catch(() => null) : null;
    return { m, e, key, scene, att, mesh, md: mesh ? I.decodeDisplay(mesh) : null, files: {}, loading: {}, running: {}, failed: {} };
  };
  const data = await Promise.all(chosen.map(load));

  if (mode === 'overlay') {
    const st = stage(true);
    cols.appendChild(st);
    const v = makeViewer(st, true);
    viewers.push(v);
    v.setScene(data[0].scene, 0);
    v.setMode('fixed', 'ghost');
    v.setShow('bolts', false);
    v.setBody('part', data[0].mesh);
    v.modes.ref = 'ghost';
    v.setBody('ref', data[1].mesh);
    st.appendChild(h(`<div class="cmp-legend"><span><i style="background:var(--part)"></i>${esc(data[0].m)} (solid)</span><span><i style="background:var(--ref)"></i>${esc(data[1].m)} (see-through)</span>${chosen.length > 2 ? '<span class="dim">overlay shows the first two chosen</span>' : ''}</div>`));
  } else {
    data.forEach((d, i) => {
      const last = i === data.length - 1;  // the orientation gizmo sits in the top-right view
      const st = stage(last);
      st.appendChild(h(`<div class="cmp-name">${esc(d.m)}</div>`));
      cols.appendChild(st);
      const v = makeViewer(st, last);
      v.noBoltLabels = true;  // an overview: the parts, not the bolt names
      viewers.push(v);
      v.setScene(d.scene, 0);
      v.setVerdict(d.att, d.scene.task.tolerances || {});
      v.setBody('part', d.mesh);
    });
  }

  // ---- linking: every camera move and every cut is copied to the other views
  let syncing = false;
  for (const v of viewers) {
    if (v !== viewers[0]) window.removeEventListener('keydown', v._onKey);  // keys drive the first view; links do the rest
    const own = v.nav.onChange;
    v.nav.onChange = () => {
      own();
      if (syncing) return;
      syncing = true;
      for (const o of viewers) {
        if (o === v) continue;
        o.nav.target.copy(v.nav.target);
        o.nav.q.copy(v.nav.q); Object.assign(o.nav, { dist: v.nav.dist, parallel: v.nav.parallel, parallelChosen: v.nav.parallelChosen, view: v.nav.view });
        o.nav.apply();
      }
      syncing = false;
    };
    v.clip.onChange(() => {
      if (syncing) return;
      syncing = true;
      for (const o of viewers) {
        if (o === v) continue;
        o.clip.planes = v.clip.planes.map((p) => ({ ...p, n: p.n.clone(), anchor: p.anchor.clone() }));
        o.applyClip(); o.overlayDirty = true;
      }
      syncing = false;
    });
  }
  viewers[0].nav.apply();
  // frame again once the columns have their final size (the links copy it)
  requestAnimationFrame(() => requestAnimationFrame(() => { for (const v of viewers) v.resize(); viewers[0].nav.fit(viewers[0].box, { animate: false }); }));

  // ---- info under each submission
  const info = $('.cmp-info', el);
  for (const d of data) {
    const e = d.e, s = e.t.submission, rep = d.att ? d.att.reported || {} : {};
    const c = task.cells[d.m];
    const fig = (k) => { const f = e.t.reported[k]; return f && f.total !== null && f.of === e.t.tries ? f.total : null; };
    const secs = fig('seconds'), tin = fig('tokens_in'), tout = fig('tokens_out'), ttot = fig('tokens_total');
    const mr = s.passed && d.att ? fmtRatio(materialRatio(d.att)) : null;
    info.appendChild(h(`<div class="ci">
      <div class="ci-head"><b>${esc(d.m)}</b> <span class="stamp ${s.passed ? 'ok' : 'bad'}">${s.passed ? 'pass' : 'fail'}</span></div>
      <dl>
        <dt>submission</dt><dd class="num">attempt #${s.n}${s.passed ? ` (try ${s.n})` : ` · ${e.t.attempts} tries, none passed`}</dd>
        ${s.passed ? '' : `<dt>failed</dt><dd><code>${esc((s.failed || []).join(', ') || '—')}</code></dd>`}
        <dt>time</dt><dd class="num rep">${secs === null ? NONE : fmtSec(secs)}</dd>
        <dt>tokens</dt><dd class="num rep">${tin === null && tout === null ? (ttot === null ? NONE : `${fmtTok(ttot)} total`) : `${tin === null ? '—' : fmtTok(tin)} in / ${tout === null ? '—' : fmtTok(tout)} out`}</dd>
        ${mr ? `<dt>material</dt><dd class="num">${mr}</dd>` : ''}
        <dt>tool</dt><dd class="rep">${esc(rep.tool || e.run.tools[0] || '') || NONE}</dd>
        <dt>run</dt><dd>${c.runs > 1 ? `<select data-run="${esc(d.m)}">${c.all.map((x, i) => `<option value="${i}" ${i === runPick[d.m] ? 'selected' : ''}>${esc(x.run.name)} · ${x.t.submission && x.t.submission.passed ? `pass, try ${x.t.submission.n}` : 'fail'}${x === c.typical ? ' (typical)' : ''}</option>`).join('')}</select> <span class="dim">${c.passed} of ${c.runs} runs passed</span>` : `${esc(e.run.name)} <span class="dim">(1 run)</span>`}</dd>
      </dl>
      ${ladderHTML(s.ladder, { compact: true })}
      <div class="ci-check" data-ci="${esc(d.m)}"></div>
      <dl class="ci-figs" data-figs="${esc(d.m)}"></dl>
      <a class="link" href="#/task/${encodeURIComponent(d.key)}/${s.n}?tab=timeline">open the timeline</a>
    </div>`));
  }

  // ---- controls
  el.addEventListener('click', (ev) => {
    const b = ev.target.closest('button'); if (!b) return;
    if (b.dataset.m) {
      const m = b.dataset.m;
      let next = chosen.includes(m) ? chosen.filter((x) => x !== m) : [...chosen, m];
      if (next.length < 1) return;
      if (next.length > 4) next = next.slice(-4);
      location.hash = url({ m: next });
    } else if (b.dataset.mode) location.hash = url({ mode: b.dataset.mode });
    else if (b.dataset.a === 'fit') viewers[0].fit();
    else if (b.dataset.a === 'proj') viewers[0].nav.toggleProjection();
    else if (b.dataset.cut) { if (b.dataset.cut === 'clear') viewers[0].clip.clear(); else viewers[0].clip.addAxis(b.dataset.cut); }
  });
  el.addEventListener('change', (ev) => {
    if (ev.target.id === 'crit') { setCrit(ev.target.value); return; }
    const s = ev.target.closest('select[data-run]'); if (!s) return;
    location.hash = url({ run: { ...runPick, [s.dataset.run]: +s.value } });
  });
  el.addEventListener('click', (ev) => {
    const pb = ev.target.closest('[data-proc]');
    if (pb) { proc = pb.dataset.proc; history.replaceState(null, '', url()); for (const b of el.querySelectorAll('[data-proc]')) { b.classList.toggle('on', b === pb); b.setAttribute('aria-pressed', String(b === pb)); } review(); return; }
    const cell = ev.target.closest('[data-pick]');
    if (cell) { setCrit(cell.dataset.pick); return; }
    const sortBy = ev.target.closest('[data-sort]');
    if (sortBy) { location.hash = url({ m: sortParts(sortBy.dataset.sort) }); return; }
    const tg = ev.target.closest('[data-tgroup]');
    if (tg) { const body = el.querySelector(`tbody[data-group="${tg.dataset.tgroup}"]`); if (body) { body.hidden = !body.hidden; tg.setAttribute('aria-expanded', String(!body.hidden)); } }
  });

  // ---- reviewing every part by one criterion, under one process -----------------
  const tol = (d) => d.scene.task.tolerances || {};
  const gradedProc = (d) => ((tol(d).manufacturing || {}).process) || 'fdm';
  const effProc = (d) => (proc === 'graded' ? gradedProc(d) : proc);
  // a field turned into a check line, for a process the part was not graded under
  const synth = (f, file, process) => ({ name: f.rule, status: f.status, required: false, group: 'makeable', unit: f.unit, reason: f.reason,
    // the measured extreme of a scalar field; a region flag has no single value
    value: f.type !== 'flag' && f.range ? (f.kind === 'min' ? f.range[0] : f.range[1]) : null,
    details: { limit: f.limit, process, direction: file.direction && file.direction.vector, direction_chosen: file.direction && file.direction.chosen, where: f.where || [] } });
  // the attempt's own lines when it was graded under this process; else the process's field file
  const gradedUnder = (d, p) => ((d.att && d.att.checks) || []).some((c) => I.groupOf(c) === 'makeable' && I.sourceOf(c) === p);
  // the stiffness analysis, for viewing, where it was run but is no check of the task
  const stiffLine = (d) => {
    const f = d.files.stiffness, w = f && f.summary && f.summary.weakest;
    return { name: 'STIFFNESS', status: 'info', required: false, group: 'sturdy', unit: 'N/mm',
      value: w ? w.stiffness : null, details: {},
      reason: w ? `at mount ${esc(w.mount)} · ${I.num(w.per_cm3)} N/mm per cm³ · not graded here` : 'not graded here' };
  };
  const hasStiffFile = (d) => !!d.att && ((d.att.fields || []).includes('stiffness'));
  const checksOf = (d) => {
    let base = (d.att && d.att.checks) || [];
    if (hasStiffFile(d) && !base.some((c) => c.name === 'STIFFNESS')) base = [...base, stiffLine(d)];
    const p = effProc(d);
    if (gradedUnder(d, p) || (proc === 'graded' && !d.files[p])) return base;
    const file = d.files[p];
    return [...base.filter((c) => I.groupOf(c) !== 'makeable'), ...(file ? file.fields.map((f) => synth(f, file, p)) : [])];
  };
  const shared = {};   // per rule: one scale for every part
  const sharedState = (f) => {
    // a field file's fields; a stiffness file carries its stress as an envelope instead
    const fieldsOf = (fl) => fl.fields || (fl.stress_envelope ? [{ ...fl.stress_envelope, rule: 'STIFFNESS_STRESS', id: 'stress' }] : []);
    const all = data.map((d) => Object.values(d.files).flatMap((fl) => fieldsOf(fl).filter((x) => x.rule === f.rule))).flat();
    if (!shared[f.rule]) shared[f.rule] = FV.newState(all.length ? all : f);
    else if (shared[f.rule].auto && all.length) shared[f.rule].range = FV.defaultRange(all);
    return shared[f.rule];
  };

  async function ensure(d, src) {
    if (!src || d.files[src] || d.loading[src] || d.running[src] || d.failed[src]) return;
    const a = d.att;
    if (!a) return;
    if ((a.fields || a.dfm || []).includes(src)) {
      d.loading[src] = true;
      try { d.files[src] = await getJSON(`data/tasks/${encodeURIComponent(d.key)}/dfm/${a.n}.${src}.json`); } catch { d.failed[src] = true; }
      d.loading[src] = false;
    } else if (live && a.mesh && (site.field_sources || []).includes(src) && src !== 'stiffness') {
      d.running[src] = true; review();
      try {
        const r = await fetch(`api/dfm?task=${encodeURIComponent(d.key)}&n=${a.n}&process=${encodeURIComponent(src)}`, { method: 'POST' });
        const res = await r.json();
        if (res.ok) { a.fields = [...new Set([...(a.fields || []), src])]; d.files[src] = await getJSON(`data/tasks/${encodeURIComponent(d.key)}/dfm/${a.n}.${src}.json`); } else d.failed[src] = true;
      } catch { d.failed[src] = true; }
      d.running[src] = false;
    } else { d.failed[src] = true; return; }
    review();
  }

  function setCrit(c) { crit = c; history.replaceState(null, '', url()); review(); }

  // the selector: every check any shown part has, grouped by requirement
  function fillSelect() {
    const sel = $('#crit', el);
    const names = new Map();
    for (const d of data) for (const c of checksOf(d)) if (!names.has(c.name)) names.set(c.name, I.groupOf(c));
    const groups = [...new Set([...I.GROUP_ORDER, ...names.values()])].filter((g) => [...names.values()].includes(g));
    sel.innerHTML = `<option value="">plain view</option>${groups.map((g) => `<optgroup label="${esc(g)}">${[...names].filter(([, gg]) => gg === g).map(([n]) => `<option ${n === crit ? 'selected' : ''}>${esc(n)}</option>`).join('')}</optgroup>`).join('')}`;
    if (crit && !names.has(crit)) sel.insertAdjacentHTML('beforeend', `<option selected>${esc(crit)}</option>`);
  }

  async function review() {
    if (mode !== 'side') return;
    // the process's field files are needed for its rules (and the setup drawing)
    // another process than graded: its field file gives its rules (made on demand, live)
    for (const d of data) { const p = effProc(d); if (proc !== 'graded' && !gradedUnder(d, p)) ensure(d, p); }
    fillSelect();
    const note = $('#review-note', el);
    const busy = data.filter((d) => d.running[effProc(d)]).map((d) => `${effProc(d)} for ${d.m}`);
    note.textContent = busy.length ? `measuring ${busy.join(', ')}…` : '';
    const fields = [];
    // the deformed shape: one exaggeration for all parts (the smallest any part asks for), so their movements compare
    const ss = shared.STIFFNESS_STRESS;
    if (ss) {
      const ex = data.map((d) => { const cs = d.files.stiffness && d.files.stiffness.cases && d.files.stiffness.cases[+ss.choice]; return cs && cs.deformed && cs.deformed.exaggeration; }).filter(Boolean);
      ss.exaggeration = ss.choice !== undefined && ss.choice !== 'envelope' && ex.length ? Math.min(...ex) : undefined;
    }
    data.forEach((d, i) => {
      const v = viewers[i], line = el.querySelector(`[data-ci="${CSS.escape(d.m)}"]`);
      const c = crit && checksOf(d).find((x) => x.name === crit);
      if (!crit && task.versions) {
        if (!gradedUnder(d, effProc(d))) ensure(d, effProc(d));   // its direction and bed come from the process's field file
        // the versions, plain: each part with its process's direction, bed and standing face (filament) or pull and parting plane
        const setup = I.processSetup({ checks: checksOf(d), file: d.files[effProc(d)] || null, md: d.md, process: effProc(d), colours: v.col });
        v.setDfm(setup ? setup.group : null, setup ? setup.labels : [], { keepPart: true }); v.setFocus(null);
        line.innerHTML = setup ? `<div class="dim ci-text">${setup.text}</div>` : `<span class="dim">no ${esc(effProc(d))} direction recorded for this part</span>`;
        return;
      }
      if (!crit || !c) {
        v.setDfm(null); v.setFocus(null); v.setMode('fixed', 'solid'); v.setShow('bolts', true);
        line.innerHTML = crit ? `<span class="dim">${d.running[effProc(d)] ? 'measuring…' : `${esc(crit)}: not measured for this part`}</span>` : '';
        return;
      }
      const src = I.sourceOf(c);
      if (src) ensure(d, src);
      const insp = I.inspection({ c, att: d.att, checks: checksOf(d), scene: d.scene, pose: 0, tol: tol(d), md: d.md, files: d.files,
        colours: v.col, regions: v.regions, live, fieldStates: sharedState });
      v.setMode('fixed', insp.ghostFixed ? 'ghost' : 'solid');
      v.setShow('bolts', insp.showBolts !== undefined ? !!insp.showBolts : !insp.ghostFixed);
      if (insp.obstacles) v.setMode('obstacle', insp.obstacles);
      v.setDfm(insp.group, insp.labels, { keepPart: insp.keepPart, hideBoltLabels: insp.hideBoltLabels });
      v.setFocus(insp.focusBolts || null, !!insp.focusRegions);
      if (insp.field) fields.push({ d, ...insp.field });
      const lim = I.limitOf(c, tol(d));
      const val = c.value === null || c.value === undefined ? '' : I.num(c.value) + (c.unit === 'deg' ? '°' : c.unit === 'ratio' ? '' : ' ' + (c.unit === 'mm2' ? 'mm²' : c.unit === 'mm3' ? 'mm³' : c.unit || ''));
      const mark = c.status === 'info' ? '·' : c.status === 'pass' ? '✓' : '✕';
      line.innerHTML = `<span class="mk ${c.status}">${mark}</span> <code>${esc(c.name)}</code> <span class="num">${c.unit === 'ratio' && typeof c.value === 'number' ? (c.value * 100).toFixed(0) + ' %' : esc(val)}</span> <span class="dim num">${lim ? `${lim.op} ${esc(lim.unit === 'ratio' ? (lim.v * 100).toFixed(0) + ' %' : I.num(lim.v))}` : ''}</span>${c.status === 'info' ? `<div class="dim ci-text">${c.reason}</div>` : ''}${(insp.short ?? insp.text) ? `<div class="dim ci-text">${insp.short ?? insp.text}</div>` : ''}`;
    });
    // one legend for all parts
    // the colour bar stands upright in the top-right view, under the orientation gizmo
    const lastStage = cols.lastElementChild;
    let bar = lastStage && lastStage.querySelector('#cmpbar');
    if (!bar && lastStage) { bar = h('<div class="fieldbar cmp-fieldbar" id="cmpbar" hidden></div>'); lastStage.appendChild(bar); }
    if (fields.length) {
      const st = fields[0].state;
      // a choice (a load case) or a toggle (the deformed shape) rebuilds every view; the shared state carries it to all
      const redraw = () => FV.legendBar(bar, fields.map((x) => x.f), st, (s2, o) => {
        if (o && o.rebuild) { review(); return; }
        for (const x of fields) x.set(s2); if (!(o && o.live)) redraw();
      }, { title: String(fields[0].f.title || fields[0].f.rule).split(',')[0], choices: fields[0].choices, toggle: fields[0].toggle ? 'deformed' : null, vertical: true });  // no note side by side: the part's own page has it
      redraw();
      for (const x of fields) x.set(st);
    } else { bar.hidden = true; bar.innerHTML = ''; }
    figures();
    table();
  }

  // ---- figures per part, and the summary table ------------------------------------
  const ROWS = [
    { k: 'verdict', t: 'verdict' },
    { k: 'ladder', t: 'ladder' },
    { k: 'tries', t: 'tries to pass', better: -1 },
    { k: 'time', t: 'time', better: -1, rep: true },
    { k: 'tokens', t: 'tokens', better: -1, rep: true },
    { k: 'volume', t: 'volume', better: -1 },
    { k: 'mass', t: 'mass', better: -1 },
    { k: 'material', t: 'material vs par', better: 1 },
    { k: 'bbox', t: 'bounding box' },
    { k: 'faces', t: 'faces' },
    { k: 'stiffness', t: 'stiffness', better: 1 },
  ];
  const figsOf = (d) => {
    const e = d.e, s = e.t.submission, info = (d.att && d.att.info) || {};
    const fig = (k) => { const f = e.t.reported[k]; return f && f.total !== null && f.of === e.t.tries ? f.total : null; };
    const p = effProc(d), [mat, rho] = MATERIAL[p] || MATERIAL.fdm;
    const fv = ((d.att && d.att.checks) || []).find((c) => c.name === 'FILE_VALID');
    const st = ((d.att && d.att.checks) || []).find((c) => c.name === 'STIFFNESS');
    const tok = fig('tokens_total') ?? ((fig('tokens_in') ?? 0) + (fig('tokens_out') ?? 0) || null);
    const mr = d.att ? materialRatio(d.att) : null;
    return {
      verdict: { v: s.passed ? 1 : 0, txt: s.passed ? '<span class="ok">pass</span>' : '<span class="bad">fail</span>' },
      ladder: { v: null, txt: s.ladder ? ladderHTML(s.ladder, { compact: true }) : NONE },
      tries: { v: s.passed ? s.n : null, txt: s.passed ? String(s.n) : `— (${e.t.attempts}, none passed)` },
      time: { v: fig('seconds'), txt: fig('seconds') === null ? NONE : fmtSec(fig('seconds')) },
      tokens: { v: tok, txt: tok === null ? NONE : fmtTok(tok) },
      volume: { v: info.volume ?? null, txt: info.volume === undefined ? NONE : `${(info.volume / 1000).toFixed(1)} cm³` },
      mass: { v: info.volume !== undefined ? (info.volume / 1000) * rho : null, txt: info.volume === undefined ? NONE : `${((info.volume / 1000) * rho).toFixed(0)} g <span class="dim">solid ${mat}</span>` },
      material: { v: mr, txt: mr === null || mr === undefined ? NONE : fmtRatio(mr) },
      bbox: { v: null, txt: info.bbox_size ? info.bbox_size.map((x) => x.toFixed(0)).join(' × ') + ' mm' : NONE },
      faces: { v: null, txt: fv && /(\d+) faces/.test(fv.reason || '') ? /(\d+) faces/.exec(fv.reason)[1] : NONE },
      stiffness: { v: st && typeof st.value === 'number' ? st.value : null, txt: st && typeof st.value === 'number' ? `${I.num(st.value)} ${esc(st.unit || '')}` : NONE },
    };
  };
  const best = (k, better) => {
    const vals = data.map((d) => figsOf(d)[k].v).filter((x) => x !== null && x !== undefined && isFinite(x));
    if (vals.length < 2 || !better || vals.every((x) => x === vals[0])) return null;   // a tie marks nothing
    return better > 0 ? Math.max(...vals) : Math.min(...vals);
  };
  function sortParts(k) {
    const r = ROWS.find((x) => x.k === k);
    const val = (m) => { const d = data.find((x) => x.m === m); const v = d ? figsOf(d)[k].v : null; return v === null || v === undefined ? (r.better > 0 ? -Infinity : Infinity) : v; };
    return chosen.slice().sort((a, b) => (r.better > 0 ? val(b) - val(a) : val(a) - val(b)));
  }
  function figures() {
    for (const d of data) {
      const dl = el.querySelector(`[data-figs="${CSS.escape(d.m)}"]`);
      const f = figsOf(d);
      const groups = I.groupChecks(checksOf(d));
      dl.innerHTML = ROWS.filter((r) => !['verdict', 'ladder', 'tries', 'time', 'tokens'].includes(r.k)).map((r) => {
        const b = best(r.k, r.better);
        return `<dt>${esc(r.t)}</dt><dd class="num${b !== null && f[r.k].v === b ? ' best' : ''}">${f[r.k].txt}</dd>`;
      }).join('') + groups.map((g) => `<dt>${esc(g.name)}</dt><dd><span class="${g.failed ? (g.failedRequired ? 'bad' : 'adv') : 'ok'}">${g.failed ? '✕' : '✓'}</span> <span class="num">${g.passed}/${g.total}</span></dd>`).join('');
    }
  }
  function table() {
    const tb = $('.cmp-table', el);
    const head = `<thead><tr><th></th>${data.map((d) => `<th>${esc(d.m)}</th>`).join('')}</tr></thead>`;
    const rows = ROWS.map((r) => {
      const b = best(r.k, r.better);
      return `<tr><th>${r.better ? `<button class="linkish" data-sort="${r.k}" title="sort the parts by ${esc(r.t)}, best first">${esc(r.t)}</button>` : esc(r.t)}</th>${data.map((d) => { const f = figsOf(d)[r.k]; return `<td class="num${b !== null && f.v === b ? ' best' : ''}${r.rep ? ' rep' : ''}">${f.txt}</td>`; }).join('')}</tr>`;
    }).join('');
    // one row per check, grouped; a click shows that check on every part
    const names = new Map();
    for (const d of data) for (const c of checksOf(d)) if (!names.has(c.name)) names.set(c.name, I.groupOf(c));
    const groups = [...new Set([...I.GROUP_ORDER, ...names.values()])].filter((g) => [...names.values()].includes(g));
    const gbodies = groups.map((g) => {
      const anyFail = data.some((d) => checksOf(d).some((c) => I.groupOf(c) === g && c.status === 'fail'));
      const gRow = `<tr class="tg"><th><button class="linkish" data-tgroup="${esc(g)}" aria-expanded="${anyFail}">${esc(g)} ▸</button></th>${data.map((d) => {
        const gg = I.groupChecks(checksOf(d)).find((x) => x.name === g);
        return `<td>${gg ? `<span class="${gg.failed ? (gg.failedRequired ? 'bad' : 'adv') : 'ok'}">${gg.failed ? '✕' : '✓'}</span> <span class="num">${gg.passed}/${gg.total}</span>` : NONE}</td>`;
      }).join('')}</tr>`;
      const crows = [...names].filter(([, gg]) => gg === g).map(([n]) => `<tr class="${n === crit ? 'on' : ''}"><th><button class="linkish" data-pick="${esc(n)}">${esc(n)}</button></th>${data.map((d) => {
        const c = checksOf(d).find((x) => x.name === n);
        if (!c) return '<td class="dim">—</td>';
        const v = c.value === null || c.value === undefined ? '' : c.unit === 'ratio' ? `${(c.value * 100).toFixed(0)} %` : I.num(c.value);
        if (c.status === 'info') return `<td data-pick="${esc(n)}" class="num pick"><span class="dim">·</span> ${esc(v)}</td>`;
        return `<td data-pick="${esc(n)}" class="num pick"><span class="${c.status === 'pass' ? 'ok' : c.required === false ? 'adv' : 'bad'}">${c.status === 'pass' ? '✓' : '✕'}</span> ${esc(v)}</td>`;
      }).join('')}</tr>`).join('');
      return `<tbody class="tgh">${gRow}</tbody><tbody data-group="${esc(g)}"${anyFail || crows.includes('class="on"') ? '' : ' hidden'}>${crows}</tbody>`;
    }).join('');
    tb.innerHTML = head + `<tbody>${rows}</tbody>` + gbodies;
  }

  if (mode === 'side') review();
  return { viewers, retheme() { if (mode === 'side') review(); }, destroy() { for (const v of viewers) v.destroy(); } };
}
