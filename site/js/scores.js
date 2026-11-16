// Scores: one row per model run, grouped and filtered in the browser from
// data/scores.json (written by `jointbench score`). Every count here comes
// from the grader's attempts logs; time, tokens, cost and CAD tool are as the
// submitter reported them and are marked as such. A missing figure is shown
// as missing, never as zero.

const $ = (sel, el = document) => el.querySelector(sel);
const h = (html) => { const t = document.createElement('template'); t.innerHTML = html.trim(); return t.content.firstElementChild; };
const esc = (s) => String(s ?? '').replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));

import { median } from './quality.js';
import { getJSON } from './fetch.js';
import { devNotice, noResults, devTag, isDevRun } from './rules.js';
import { frontierChart, levelMultiples, materialChart, taskStrip, ladderChart, fmtSec as cSec, fmtTok as cTok, fmtMoney, TIME_TICKS } from './charts.js';

const NONE = '<span class="missing" title="not reported">—</span>';
const NO_TOOL = 'not reported';


// ---- aggregation ----------------------------------------------------------

function taskTool(t, run) {
  return (t.tools && t.tools[0]) || (run.tools && run.tools[0]) || NO_TOOL;
}

// one flat row per (run, task) so any grouping is a reduce
let DEV_MIXED = false;

function flatten(scores) {
  const rows = [];
  for (const run of scores.runs) {
    for (const t of run.tasks) rows.push({ run, t, tool: taskTool(t, run) });
  }
  return rows;
}

function aggregate(rows) {
  const att = rows.filter((r) => r.t.attempts > 0);
  const passed = att.filter((r) => r.t.passed_within);
  const fails = {};
  for (const r of att) for (const [k, v] of Object.entries(r.t.failed_checks)) fails[k] = (fails[k] || 0) + v;
  const sumRep = (key, subset) => {
    let total = 0, n = 0;
    for (const r of subset) {
      const f = r.t.reported[key];
      if (f && f.total !== null && f.of === r.t.tries) { total += f.total; n++; }
    }
    return { total: n ? total : null, n, of: subset.length };
  };
  const byLevel = {};
  for (const r of att) {
    const b = byLevel[r.t.level] = byLevel[r.t.level] || { attempted: 0, first: 0, within: 0 };
    b.attempted++; b.first += r.t.passed_first ? 1 : 0; b.within += r.t.passed_within ? 1 : 0;
  }
  return {
    attempted: att.length,
    first: att.filter((r) => r.t.passed_first).length,
    within: passed.length,
    avgTries: passed.length ? passed.reduce((a, r) => a + r.t.tries, 0) / passed.length : null,
    fails,
    byLevel,
    timeToPass: sumRep('seconds', passed),
    tokensIn: sumRep('tokens_in', att),
    tokensOut: sumRep('tokens_out', att),
    tokensTotal: sumRep('tokens_total', att),
    cost: sumRep('cost', att),
    currency: (att.find((r) => r.run.currency) || {}).run?.currency || null,
    // median material ratio over passing attempts (null until par volumes exist)
    material: median(passed.flatMap((r) => r.t.rows.filter((x) => x.passed && x.counted).map((x) => x.material_ratio))),
  };
}

// ---- formatting -------------------------------------------------------------

const pct = (a, b) => (b ? Math.round((100 * a) / b) : null);
function frac(a, b) {
  if (!b) return NONE;
  return `<span class="num">${a}/${b}</span> <span class="pc num">${pct(a, b)} %</span>`;
}
function bar(a, b) {
  const p = b ? (100 * a) / b : 0;
  return `<span class="pbar" title="${a} of ${b}"><i style="width:${p.toFixed(1)}%"></i></span>`;
}
function rep(v, n, of, fmt) {
  // reported figure: shown only when every counted task reported it
  if (v === null || v === undefined) return NONE;
  const part = n < of ? ` <span class="partial" title="only ${n} of ${of} tasks reported this">(${n}/${of})</span>` : '';
  return `<span class="rep num">${fmt(v)}</span>${part}`;
}
const fmtSec = (s) => (s >= 3600 ? `${(s / 3600).toFixed(1)} h` : s >= 60 ? `${(s / 60).toFixed(1)} min` : `${s.toFixed(0)} s`);
const fmtTok = (t) => (t >= 1e6 ? `${(t / 1e6).toFixed(2)} M` : t >= 1e3 ? `${(t / 1e3).toFixed(1)} k` : `${t}`);
const fmtCost = (cur) => (c) => `${c.toFixed(c < 1 ? 3 : 2)}${cur ? ' ' + esc(cur) : ''}`;

// tokens: input / output when split; otherwise the total, labelled as such
// (a total is never shown in the input column)
function tokCell(tin, tout, ttot) {
  if (tin.total !== null || tout.total !== null) return `${rep(tin.total, tin.n, tin.of, fmtTok)} / ${rep(tout.total, tout.n, tout.of, fmtTok)}`;
  if (ttot && ttot.total !== null && ttot.total !== undefined) return `${rep(ttot.total, ttot.n, ttot.of, fmtTok)} <span class="dim" title="input and output together; no split was reported">total</span>`;
  return NONE;
}

function topFail(fails) {
  const e = Object.entries(fails).sort((a, b) => b[1] - a[1]);
  if (!e.length) return '<span class="dim">none</span>';
  return e.slice(0, 2).map(([k, v]) => `<code>${esc(k)}</code> <span class="num dim">×${v}</span>`).join('<br>');
}

function levelCells(byLevel) {
  const ks = Object.keys(byLevel).sort();
  if (!ks.length) return NONE;
  return ks.map((k) => `<span class="lv" title="level ${k}: ${byLevel[k].within} of ${byLevel[k].attempted} passed">L${k} <b class="num">${byLevel[k].within}/${byLevel[k].attempted}</b></span>`).join(' ');
}

// ---- page -------------------------------------------------------------------

export async function renderScores(app, runId, state) {
  let scores;
  try { scores = await getJSON('data/scores.json'); } catch {
    app.replaceChildren(h(`<div class="page"><div class="section-title"><h2>Scores</h2></div>
      <p class="note">No results file is loaded. Score a folder of model runs and show it:</p>
      <div class="runbox"><pre>uv run jointbench score runs --out scores.json
uv run jointbench play runs                      <span class="c"># or: play tasks --scores scores.json</span>
uv run jointbench site runs --out public         <span class="c"># static copy with the scores page</span></pre></div>
      <p class="note">A runs folder holds one folder per model run, each with task folders and their <code>attempts.jsonl</code>; an optional <code>run.json</code> per run gives its display name, date, tries allowed and notes.</p></div>`));
    return { destroy() {} };
  }
  if (!scores.runs.length) { app.replaceChildren(h(noResults(scores, 'Scores'))); return { destroy() {} }; }
  if (runId) return renderRun(app, scores, runId);

  const all = flatten(scores);
  DEV_MIXED = scores.runs.some(isDevRun) && !scores.runs.every(isDevRun);
  const levels = [...new Set(all.map((r) => r.t.level))].sort();
  const tools = [...new Set(all.map((r) => r.tool))].sort();
  const sets = [...new Set(scores.runs.map(runSet).filter(Boolean))].sort();
  const procs = [...new Set(scores.runs.map(runProc).filter(Boolean))].sort();
  const st = Object.assign({ group: 'model', level: 'all', tool: 'all', set: 'all', proc: 'all', x: null, pass: 'within' }, state || {});
  // open the chart on the first figure any run reported: cost, else tokens, else time
  if (!st.x) st.x = ['cost', 'tokens', 'time'].find((k) => all.some((r) => r.t.attempts && taskFigure(r, XAXES[k]) !== null)) || 'cost';

  const el = h(`<div class="page scores">
    <div class="section-title"><h2>Scores</h2><span class="n">${scores.runs.length} runs · ${all.filter((r) => r.t.attempts).length} task attempts · scored ${esc((scores.generated || '').slice(0, 16).replace('T', ' '))} UTC</span></div>
    ${devNotice(scores, scores.runs)}
    <p class="prose" style="margin-top:0">Every count is read from the grader's attempts logs. A task is <b>passed first try</b> when attempt 1 passed, <b>passed</b> when an attempt within the allowed tries passed. <b>Tries</b> counts attempts up to the first pass. Figures in <span class="rep">italic</span> were <b>reported by the submitter</b> (time, tokens, cost, CAD tool), not measured; <span class="missing">—</span> means not reported.</p>
    <div class="filters">
      <div class="seg" id="grp"><span class="lbl">group by</span>
        <button data-g="model">Model</button><button data-g="tool">CAD tool</button><button data-g="grid">Model × tool</button></div>
      <label class="sel">level <select id="flevel"><option value="all">all</option>${levels.map((l) => `<option value="${l}">${l}</option>`).join('')}</select></label>
      <label class="sel">tool <select id="ftool"><option value="all">all</option>${tools.map((t) => `<option>${esc(t)}</option>`).join('')}</select></label>
      ${sets.length ? `<label class="sel">set <select id="fset"><option value="all">all</option>${sets.map((t) => `<option>${esc(t)}</option>`).join('')}</select></label>` : ''}
      ${procs.length ? `<label class="sel">process <select id="fproc"><option value="all">all</option>${procs.map((t) => `<option>${esc(t)}</option>`).join('')}</select></label>` : ''}
    </div>
    <div id="board"></div>
    <div class="section-title"><h2>Score against cost</h2><span class="n">one point per model and CAD tool, per level; each level has its own line of best score for the price</span></div>
    <div class="filters">
      <div class="seg" id="fx"><span class="lbl">across</span><button data-x="cost">Cost</button><button data-x="tokens">Tokens</button><button data-x="time">Time</button></div>
      <div class="seg" id="fpass"><span class="lbl">score</span><button data-p="within">Passed within the allowed tries</button><button data-p="first">Passed at the first try</button></div>
    </div>
    <div id="scatter"></div>
    <div class="section-title"><h2>Where each model falls off</h2><span class="n">share passed by level</span></div>
    <div id="levels"></div>
    <div class="section-title"><h2>How far up the ladder</h2><span class="n">valid solid, fits, cleanly modelled, makeable, stiff: share of parts reaching each rung</span></div>
    <div id="ladder"></div>
    <div class="section-title"><h2>Which task, which model</h2><span class="n">every attempted task, easiest left; click a square to open the task</span></div>
    <div id="strip"></div>
    <div class="section-title"><h2>Material among passed tasks</h2><span class="n">how lean the passing parts are</span></div>
    <div id="material"></div>
    <p class="note">${sets.length ? '' : 'No run names a task set (<code>run.json</code> "set"), so there is no set switch. '}${procs.length ? '' : 'No run names a manufacturing process (<code>run.json</code> "process"), so there is no process switch.'}</p>
    <div class="section-title"><h2>Which check failed</h2><span class="n">failed attempts per check that decides the verdict, counted up to each task's first pass</span></div>
    <div id="fails"></div>
  </div>`);
  app.replaceChildren(el);

  const draw = () => {
    for (const b of $('#grp', el).querySelectorAll('button')) b.classList.toggle('on', b.dataset.g === st.group);
    for (const b of $('#fx', el).querySelectorAll('button')) b.classList.toggle('on', b.dataset.x === st.x);
    for (const b of $('#fpass', el).querySelectorAll('button')) b.classList.toggle('on', b.dataset.p === st.pass);
    const rows = all.filter((r) => (st.level === 'all' || String(r.t.level) === st.level) && (st.tool === 'all' || r.tool === st.tool)
      && (st.set === 'all' || runSet(r.run) === st.set) && (st.proc === 'all' || runProc(r.run) === st.proc));
    // the level charts ignore the level filter: they are about levels
    const rowsAllLevels = all.filter((r) => (st.tool === 'all' || r.tool === st.tool)
      && (st.set === 'all' || runSet(r.run) === st.set) && (st.proc === 'all' || runProc(r.run) === st.proc));
    const board = $('#board', el);
    if (st.group === 'grid') board.replaceChildren(gridTable(rows));
    else board.replaceChildren(boardTable(rows, st.group));
    const X = XAXES[st.x], Yname = st.pass === 'first' ? 'passed at the first try' : 'passed within the allowed tries';
    const cur = (rows.find((r) => r.run.currency) || {}).run?.currency || '';
    $('#scatter', el).replaceChildren(frontierChart(points(rows, st, X), { name: X.name, fmt: X.fmt(cur), ticks: X.ticks }, { name: Yname }));
    $('#levels', el).replaceChildren(levelMultiples(levelData(rowsAllLevels, st), levels.map(String), Yname));
    $('#ladder', el).replaceChildren(ladderChart(ladderData(rows)));
    $('#strip', el).replaceChildren(taskStrip(...stripData(rows)));
    $('#material', el).replaceChildren(materialChart(materialData(rows)));
    $('#fails', el).replaceChildren(failTable(rows));
  };
  $('#grp', el).addEventListener('click', (e) => { const b = e.target.closest('button'); if (b) { st.group = b.dataset.g; draw(); } });
  $('#flevel', el).addEventListener('change', (e) => { st.level = e.target.value; draw(); });
  $('#ftool', el).addEventListener('change', (e) => { st.tool = e.target.value; draw(); });
  if ($('#fset', el)) $('#fset', el).addEventListener('change', (e) => { st.set = e.target.value; draw(); });
  if ($('#fproc', el)) $('#fproc', el).addEventListener('change', (e) => { st.proc = e.target.value; draw(); });
  $('#fx', el).addEventListener('click', (e) => { const b = e.target.closest('button'); if (b) { st.x = b.dataset.x; draw(); } });
  $('#fpass', el).addEventListener('click', (e) => { const b = e.target.closest('button'); if (b) { st.pass = b.dataset.p; draw(); } });
  if (st.level !== 'all') $('#flevel', el).value = st.level;
  draw();
  performance.mark('jb-text');
  window.scrollTo(0, 0);
  return { destroy() {} };
}

function groups(rows, by) {
  const m = new Map();
  for (const r of rows) {
    // by model: the runs of one model form one row (run.json "model", else its name)
    const k = by === 'tool' ? r.tool : r.run.model || r.run.id;
    if (!m.has(k)) m.set(k, { key: k, run: by === 'tool' ? null : r.run, runs: [], rows: [] });
    const g = m.get(k);
    g.rows.push(r);
    if (!g.runs.includes(r.run)) g.runs.push(r.run);
  }
  return [...m.values()];
}

// repeated runs of one model: the mean rate over its runs, with the lowest
// and highest run beside it -- never the best run alone
function spread(g, k) {
  const rates = g.runs.map((run) => {
    const a = aggregate(g.rows.filter((r) => r.run === run));
    return a.attempted ? a[k] / a.attempted : null;
  }).filter((x) => x !== null);
  if (!rates.length) return NONE;
  const mean = rates.reduce((x, y) => x + y, 0) / rates.length;
  const lo = Math.min(...rates), hi = Math.max(...rates);
  return `<span class="num" title="mean over ${rates.length} runs; range lowest to highest run">${Math.round(100 * mean)} %</span> <span class="pc num">mean, ${Math.round(100 * lo)}–${Math.round(100 * hi)} % over ${rates.length} runs</span>`;
}

function repeats(g) {
  const seen = new Map();
  for (const r of g.rows) {
    if (!r.t.attempts) continue;
    const s = seen.get(r.t.id);
    if (s && s !== r.run) return true;
    seen.set(r.t.id, r.run);
  }
  return false;
}

function boardTable(rows, by) {
  const gs = groups(rows, by).map((g) => ({ ...g, a: aggregate(g.rows) }));
  gs.sort((x, y) => (y.a.within / (y.a.attempted || 1)) - (x.a.within / (x.a.attempted || 1)) ||
    (y.a.first - x.a.first) || ((x.a.avgTries ?? 99) - (y.a.avgTries ?? 99)));
  const hasMat = gs.some((g) => g.a.material !== null);
  if (hasMat && boardTable.sortMat) gs.sort((x, y) => (y.a.material ?? -1) - (x.a.material ?? -1));  // leanest (highest) first
  const t = h(`<div class="tablewrap"><table class="board">
    <thead><tr>
      <th class="r">#</th><th>${by === 'tool' ? 'CAD tool' : 'model'}</th>
      <th class="r">tasks</th><th title="share of attempted tasks whose attempt 1 passed">passed at the first try</th><th title="share of attempted tasks with a pass within the allowed tries">passed within the allowed tries</th><th class="r">avg tries</th>
      <th>by level</th><th>most failed</th>
      ${hasMat ? '<th class="r sortable" id="matsort" title="material score, par volume ÷ part volume, median over counted passes; higher is leaner; click to sort, leanest first">material ⇅</th>' : ''}
      <th class="r rep-h">time to pass</th><th class="r rep-h" title="input / output; or the total where no split was reported">tokens</th><th class="r rep-h">cost</th>
    </tr></thead><tbody></tbody></table></div>`);
  const tb = $('tbody', t);
  const ms = $('#matsort', t);
  if (ms) ms.addEventListener('click', () => { boardTable.sortMat = !boardTable.sortMat; t.replaceWith(boardTable(rows, by)); });
  gs.forEach((g, i) => {
    const a = g.a;
    // repeated runs: the same model run again over (some of) the same tasks.
    // Runs over different tasks (e.g. one per level) are parts of one
    // result and are added up, never averaged.
    const multi = g.run && g.runs.length > 1 && repeats(g);
    const lvls = (run) => [...new Set(g.rows.filter((r) => r.run === run && r.t.attempts).map((r) => `L${r.t.level}`))].join('+');
    const name = g.run && g.runs.length === 1
      ? `<a href="#/scores/${encodeURIComponent(g.run.id)}"><b>${esc(g.run.name)}</b></a><small>${[g.run.date, g.run.tries_allowed ? `${g.run.tries_allowed} tries allowed` : 'no try limit', g.run.tools.length ? g.run.tools.join(', ') : null].filter(Boolean).map(esc).join(' · ')}</small>`
      : g.run
      ? `<b>${esc(g.key)}</b><small>${g.runs.length} runs${multi ? '' : ', added up'}: ${g.runs.map((r) => `<a href="#/scores/${encodeURIComponent(r.id)}">${esc(multi ? (r.date || r.id) : lvls(r) || r.id)}</a>`).join(', ')}</small>`
      : `<b class="${g.key === NO_TOOL ? 'dim' : 'rep'}">${esc(g.key)}</b><small>${new Set(g.rows.map((r) => r.run.name)).size} model runs</small>`;
    const tokens = tokCell(a.tokensIn, a.tokensOut, a.tokensTotal);
    // only needed where current and earlier-rules runs are shown together
    const dev = DEV_MIXED && [...new Set(g.rows.map((r) => r.run))].some(isDevRun) ? devTag() : '';
    tb.appendChild(h(`<tr>
      <td class="r num dim">${i + 1}</td><td class="name">${name}${dev}</td>
      <td class="r num">${a.attempted}</td>
      <td>${multi ? spread(g, 'first') : frac(a.first, a.attempted)}</td>
      <td>${bar(a.within, a.attempted)} ${multi ? spread(g, 'within') : frac(a.within, a.attempted)}</td>
      <td class="r num">${a.avgTries === null ? NONE : a.avgTries.toFixed(2)}</td>
      <td>${levelCells(a.byLevel)}</td>
      <td>${topFail(a.fails)}</td>
      ${hasMat ? `<td class="r num">${a.material === null ? NONE : a.material.toFixed(2)}</td>` : ''}
      <td class="r">${rep(a.timeToPass.total, a.timeToPass.n, a.timeToPass.of, fmtSec)}</td>
      <td class="r">${tokens}</td>
      <td class="r">${rep(a.cost.total, a.cost.n, a.cost.of, fmtCost(a.currency))}</td>
    </tr>`));
  });
  if (!gs.length) tb.appendChild(h(`<tr><td colspan="${hasMat ? 12 : 11}" class="dim">nothing matches the filter</td></tr>`));
  return t;
}

function gridTable(rows) {
  const runs = [...new Map(rows.map((r) => [r.run.id, r.run])).values()];
  const tools = [...new Set(rows.map((r) => r.tool))].sort();
  const t = h(`<div class="tablewrap"><table class="board grid-mt">
    <thead><tr><th>model run ↓ / CAD tool →</th>${tools.map((x) => `<th class="${x === NO_TOOL ? 'dim' : 'rep'}">${esc(x)}</th>`).join('')}</tr></thead><tbody></tbody></table></div>`);
  const tb = $('tbody', t);
  for (const run of runs) {
    const cells = tools.map((tool) => {
      const a = aggregate(rows.filter((r) => r.run.id === run.id && r.tool === tool));
      if (!a.attempted) return '<td class="cell empty">·</td>';
      const p = a.within / a.attempted;
      return `<td class="cell" style="--p:${p.toFixed(3)}" title="${a.within} of ${a.attempted} passed, ${a.first} first try"><b class="num">${Math.round(100 * p)} %</b><small class="num">${a.within}/${a.attempted}</small></td>`;
    }).join('');
    tb.appendChild(h(`<tr><td class="name"><a href="#/scores/${encodeURIComponent(run.id)}"><b>${esc(run.name)}</b></a></td>${cells}</tr>`));
  }
  return t;
}

// ---- chart data ------------------------------------------------------------------

const runSet = (run) => (run.run_json && (run.run_json.set || run.run_json.task_set)) || null;
const runProc = (run) => (run.run_json && run.run_json.process) || null;
const isHuman = (run) => !!(run.run_json && run.run_json.human) || /^human\b/i.test(run.model || run.name || '');
// a person is shown by name ("human" is their model field); a model by its model name
const combo = (r) => `${isHuman(r.run) ? r.run.name : r.run.model || r.run.name}${r.tool === NO_TOOL ? '' : ' · ' + r.tool}`;

// what a task cost, per axis: from the attempts log (complete over the
// counted tries), else null
const XAXES = {
  cost: { name: 'cost', keys: ['cost'], fmt: (cur) => fmtMoney(cur) },
  tokens: { name: 'tokens, total', keys: ['tokens_in', 'tokens_out'], alt: ['tokens_total'], fmt: () => cTok },
  time: { name: 'time', keys: ['seconds'], fmt: () => cSec, ticks: TIME_TICKS },
};
function taskFigure(r, X) {
  const sum = (keys) => {
    let v = 0;
    for (const k of keys) {
      const f = r.t.reported[k];
      if (!f || f.total === null || f.of !== r.t.tries) return null;
      v += f.total;
    }
    return v;
  };
  const v = sum(X.keys);
  return v === null && X.alt ? sum(X.alt) : v;   // tokens: in + out, else the reported total
}
// a run's mean per task from run.json (when its log has none)
function runFigure(run, X) {
  const rr = run.summary && run.summary.run_reported;
  if (!rr || run.summary.run_reported_basis !== 'mean per task') return null;  // older results files: unknown basis, not used
  const sum = (keys) => { let v = 0; for (const k of keys) { if (rr[k] === null || rr[k] === undefined) return null; v += rr[k]; } return v; };
  const v = sum(X.keys);
  return v === null && X.alt ? sum(X.alt) : v;
}

function points(rows, st, X) {
  // levels are never pooled: with more than one level in the selection,
  // each model and tool gets one point per level, and each level its own frontier
  const multi = new Set(rows.filter((r) => r.t.attempts).map((r) => r.t.level)).size > 1;
  const m = new Map();
  for (const r of rows) {
    if (!r.t.attempts) continue;
    const k = multi ? `${combo(r)} · L${r.t.level}` : combo(r);
    if (!m.has(k)) m.set(k, { label: k, rows: [], runs: new Set(), series: multi ? `level ${r.t.level}` : null });
    m.get(k).rows.push(r); m.get(k).runs.add(r.run);
  }
  return [...m.values()].map((g) => {
    const of = g.rows.length;
    const n = g.rows.filter((r) => (st.pass === 'first' ? r.t.passed_first : r.t.passed_within)).length;
    const figs = g.rows.map((r) => taskFigure(r, X)).filter((v) => v !== null);
    let x = null, xPartial = null, src = '';
    // 1. every task reported it in the log; 2. else whole runs that give a
    // total in run.json; 3. else the mean over the tasks that did report it,
    // marked as partial
    const runs0 = [...g.runs];
    const whole = runs0.every((run) => g.rows.filter((r) => r.run === run).length === run.summary.attempted);
    const tot = runs0.map((run) => runFigure(run, X));
    const fromRunJson = g.rows.some((r) => [...X.keys, ...(X.alt || [])].some((k) => r.t.reported[k] && r.t.reported[k].source === 'run.json'));
    if (figs.length === of) {
      x = figs.reduce((a, b) => a + b, 0) / of; src = fromRunJson ? 'per task, from run.json' : 'from the attempts log';
    } else if (whole && tot.every((v) => v !== null)) {
      // each run's mean per task, weighted by its tasks
      x = runs0.reduce((a, run, i) => a + tot[i] * run.summary.attempted, 0) / of; src = 'run mean per task from run.json';
    } else if (figs.length) {
      x = figs.reduce((a, b) => a + b, 0) / figs.length;
      xPartial = `mean over the ${figs.length} of ${of} tasks that reported it`;
      src = 'from the attempts log';
    }
    const runs = [...g.runs];
    return { label: g.label, series: g.series, x, y: of ? n / of : 0, n, of, xPartial, human: runs.some(isHuman),
      detail: `${runs.length === 1 ? runs[0].name : `${runs.length} runs, pooled`}${src ? ` · ${src}` : ''}${xPartial ? ` · ${xPartial}` : ''}` };
  });
}

function levelData(rows, st) {
  const m = new Map();
  for (const r of rows) {
    if (!r.t.attempts) continue;
    const k = combo(r);
    if (!m.has(k)) m.set(k, { label: k, levels: {} });
    const b = m.get(k).levels[r.t.level] = m.get(k).levels[r.t.level] || { n: 0, of: 0 };
    b.of++; if (st.pass === 'first' ? r.t.passed_first : r.t.passed_within) b.n++;
  }
  return [...m.values()];
}

// per model (and tool): the submissions' ladders added up (counts as ladder.py reach())
function ladderData(rows) {
  const m = new Map();
  for (const r of rows) {
    const lad = r.t.attempts && r.t.submission && r.t.submission.ladder;
    if (!lad) continue;
    const k = combo(r);
    if (!m.has(k)) m.set(k, { label: k, rungs: lad.map((x) => ({ label: x.label, reached: 0, measured: 0, required: 0 })) });
    const rs = m.get(k).rungs;
    let ok = true;
    lad.forEach((x, i) => {
      if (x.required) rs[i].required++;
      if (x.state === 'not measured') return;
      rs[i].measured++;
      if (x.state === 'met') { if (ok) rs[i].reached++; } else ok = false;
    });
  }
  return [...m.values()];
}

function stripData(rows) {
  // one row per model and tool (its runs at different levels share the row);
  // a run that repeats a task already in the row gets a row of its own
  const out = [], tasks = new Map(), rowOf = new Map();
  for (const r of rows) {
    if (!r.t.attempts) continue;
    tasks.set(r.t.id, { id: r.t.id, level: r.t.level });
    const k = combo(r);
    let row = rowOf.get(`${k}\u0000${r.run.id}`);
    if (!row) {
      row = out.find((x) => x.key === k && !Object.keys(x.cells).some((id) => rows.some((q) => q.run === r.run && q.t.id === id)));
      if (!row) { row = { key: k, label: k, cells: {} }; out.push(row); }
      rowOf.set(`${k}\u0000${r.run.id}`, row);
    }
    row.cells[r.t.id] = { state: r.t.passed_first ? 'first' : r.t.passed_within ? 'within' : 'fail', tries: r.t.tries, key: r.t.key };
  }
  // repeated rows of one model and tool are numbered
  const seen = {};
  for (const row of out) { seen[row.key] = (seen[row.key] || 0) + 1; if (seen[row.key] > 1) row.label = `${row.key} (run ${seen[row.key]})`; }
  return [out, [...tasks.values()]];
}

function materialData(rows) {
  const m = new Map();
  for (const r of rows) {
    const k = combo(r);
    if (!m.has(k)) m.set(k, { label: k, values: [], excluded: [] });
    const sub = r.t.submission;
    if (!sub || !sub.passed) continue;
    const row = r.t.rows.find((x) => x.n === sub.n);
    if (row && row.material_ratio !== null && row.material_ratio !== undefined) m.get(k).values.push({ v: row.material_ratio, task: r.t.id });
    else if (row && row.material_note) m.get(k).excluded.push({ task: r.t.id, note: row.material_note });
  }
  return [...m.values()];
}

function failTable(rows) {
  const gs = groups(rows, 'model').map((g) => ({ run: g.run, a: aggregate(g.rows) }));
  const names = [...new Set(gs.flatMap((g) => Object.keys(g.a.fails)))].sort();
  if (!names.length) return h('<p class="note">No counted attempt failed a check in this selection.</p>');
  const max = Math.max(1, ...gs.flatMap((g) => Object.values(g.a.fails)));
  const t = h(`<div class="tablewrap"><table class="board fails"><thead><tr><th>model run</th>${names.map((n) => `<th><code>${esc(n)}</code></th>`).join('')}</tr></thead><tbody></tbody></table></div>`);
  for (const g of gs) {
    $('tbody', t).appendChild(h(`<tr><td class="name">${esc(g.run.name)}</td>${names.map((n) => {
      const v = g.a.fails[n] || 0;
      return `<td><span class="fbar"><i style="width:${(100 * v) / max}%"></i></span> <span class="num ${v ? '' : 'dim'}">${v}</span></td>`;
    }).join('')}</tr>`));
  }
  return t;
}

// ---- one run: its tasks ---------------------------------------------------------

function renderRun(app, scores, runId) {
  const run = scores.runs.find((r) => r.id === runId);
  if (!run) {
    app.replaceChildren(h(`<div class="page"><p class="note">No run named ${esc(runId)}. <a href="#/scores">All scores</a></p></div>`));
    return { destroy() {} };
  }
  const a = aggregate(run.tasks.map((t) => ({ run, t, tool: taskTool(t, run) })));
  const cur = run.currency;
  const el = h(`<div class="page scores">
    <p class="crumb"><a href="#/scores">Scores</a> /</p>
    <div class="section-title" style="margin-top:4px"><h2>${esc(run.name)}</h2>
      <span class="n">${[run.date, run.model_version, run.tries_allowed ? `${run.tries_allowed} tries allowed` : 'no try limit'].filter(Boolean).map(esc).join(' · ')}</span></div>
    ${devNotice(scores, [run])}
    ${run.notes ? `<p class="prose">${esc(run.notes)}</p>` : ''}
    ${run.reported_note ? `<p class="note"><span class="rep">Reported figures:</span> ${esc(run.reported_note)}</p>` : ''}
    <div class="kpis">
      <div><small>passed</small><b class="num">${a.within}/${a.attempted}</b></div>
      <div><small>first try</small><b class="num">${a.first}/${a.attempted}</b></div>
      <div><small>avg tries</small><b class="num">${a.avgTries === null ? '—' : a.avgTries.toFixed(2)}</b></div>
      <div><small>CAD tool <i>reported</i></small><b class="rep">${run.tools.length ? esc(run.tools.join(', ')) : '—'}</b></div>
      <div><small>cost <i>reported</i></small><b>${rep(a.cost.total, a.cost.n, a.cost.of, fmtCost(cur))}</b></div>
    </div>
    <div class="tablewrap"><table class="board">
      <thead><tr><th>task</th><th class="r">level</th><th>attempts, in order</th><th class="r">first pass</th><th>failed checks</th>
        <th class="r rep-h">time</th><th class="r rep-h" title="input / output; or the total where no split was reported">tokens</th><th class="r rep-h">cost</th><th></th></tr></thead>
      <tbody></tbody></table></div>
  </div>`);
  const tb = $('tbody', el);
  for (const t of run.tasks) {
    const seq = t.rows.map((r) => `<a class="tick ${r.passed ? 'ok' : 'bad'} ${r.counted ? '' : 'after'}" href="#/task/${encodeURIComponent(t.key)}/${r.n}?tab=timeline" title="attempt ${r.n}: ${r.passed ? 'pass' : 'fail'}${r.counted ? '' : ' (after the first pass, not counted)'}">${r.n}</a>`).join('');
    const f = t.reported;
    const target = t.first_pass || (t.rows.length ? t.rows[t.rows.length - 1].n : '');
    tb.appendChild(h(`<tr>
      <td class="name"><a href="#/task/${encodeURIComponent(t.key)}/${target}?tab=timeline"><b>${esc(t.id)}</b></a></td>
      <td class="r num">${t.level}</td>
      <td><span class="ticks">${seq || '<span class="dim">not attempted</span>'}</span></td>
      <td class="r num">${t.first_pass ? `#${t.first_pass}` : '<span class="bad-t">never</span>'}</td>
      <td>${Object.keys(t.failed_checks).length ? Object.entries(t.failed_checks).map(([k, v]) => `<code>${esc(k)}</code> <span class="dim num">×${v}</span>`).join('<br>') : '<span class="dim">none</span>'}</td>
      <td class="r">${rep(f.seconds.total, f.seconds.of, t.tries, fmtSec)}</td>
      <td class="r">${tokCell({ total: f.tokens_in.total, n: f.tokens_in.of, of: t.tries }, { total: f.tokens_out.total, n: f.tokens_out.of, of: t.tries }, f.tokens_total ? { total: f.tokens_total.total, n: f.tokens_total.of, of: t.tries } : { total: null })}</td>
      <td class="r">${rep(f.cost.total, f.cost.of, t.tries, fmtCost(cur))}</td>
      <td><a class="btn small ghost" href="#/task/${encodeURIComponent(t.key)}/${target}?tab=timeline">timeline</a></td>
    </tr>`));
  }
  app.replaceChildren(el);
  window.scrollTo(0, 0);
  return { destroy() {} };
}
