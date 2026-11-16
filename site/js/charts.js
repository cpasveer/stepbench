// Score charts, drawn as plain SVG (no chart library).
//
// Four pictures of the same results file, each with a table of exactly the
// numbers it draws underneath:
//
// * frontier: one point per model and CAD tool, the share of tasks passed
//   against what a task cost (money, tokens or time, logarithmic), with the
//   line of best score for the price drawn through the leading points;
// * levels: pass rate by level, one small chart per model, to see where
//   each falls off;
// * material: how lean the passing parts are, per model (once the grader
//   reports a par volume);
// * tasks: which model passed which task, hardest tasks to the right.
//
// Nothing is plotted that was not reported. A point without the figure on
// the horizontal axis is left off that chart, and the chart says how many.
//
// Idea after the ARC Prize leaderboard's score-against-cost chart; drawn and
// written for JointBench.

const esc = (s) => String(s ?? '').replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
const h = (html) => { const t = document.createElement('template'); t.innerHTML = html.trim(); return t.content.firstElementChild; };

export const fmtSec = (s) => (s >= 3600 ? `${Number((s / 3600).toFixed(1))} h` : s >= 60 ? `${Number((s / 60).toFixed(1))} min` : `${s.toFixed(0)} s`);
// round values for a time axis, seconds
export const TIME_TICKS = [1, 2, 5, 10, 15, 30, 60, 120, 300, 600, 900, 1800, 3600, 7200, 18000, 36000];
export const fmtTok = (t) => (t >= 1e6 ? `${Number((t / 1e6).toFixed(2))} M` : t >= 1e3 ? `${Number((t / 1e3).toFixed(t >= 1e4 ? 0 : 1))} k` : `${Math.round(t)}`);
export const fmtMoney = (cur) => (c) => `${c < 1 ? String(Number(c.toPrecision(2))) : c.toFixed(2)}${cur ? ' ' + cur : ''}`;
const pc = (v) => `${Math.round(100 * v)} %`;

// hover: one small box per chart, filled from the element's data-tip
function hoverable(wrap) {
  const tip = h('<div class="ctip" hidden></div>');
  wrap.appendChild(tip);
  const show = (e) => {
    const t = e.target.closest('[data-tip]');
    if (!t || !wrap.contains(t)) { tip.hidden = true; return; }
    tip.innerHTML = t.dataset.tip;
    tip.hidden = false;
    const r = wrap.getBoundingClientRect();
    let x = e.clientX - r.left + 14, y = e.clientY - r.top + 14;
    if (x + tip.offsetWidth > r.width) x = e.clientX - r.left - tip.offsetWidth - 10;
    tip.style.left = `${Math.max(0, x)}px`; tip.style.top = `${y}px`;
  };
  wrap.addEventListener('pointermove', show);
  wrap.addEventListener('pointerleave', () => { tip.hidden = true; });
  // keyboard: points can be focused, the box follows
  wrap.addEventListener('focusin', (e) => {
    const t = e.target.closest('[data-tip]'); if (!t) return;
    const b = t.getBoundingClientRect(); show({ target: t, clientX: b.right, clientY: b.bottom });
  });
}

// ticks on a logarithmic axis: 1, 2, 5 per decade, thinned to fit
function logTicks(lo, hi) {
  const out = [];
  for (let e = Math.floor(Math.log10(lo)); e <= Math.ceil(Math.log10(hi)); e++) {
    for (const m of [1, 2, 5]) { const v = m * 10 ** e; if (v >= lo * 0.999 && v <= hi * 1.001) out.push(v); }
  }
  const lead = (v) => Math.round(v / 10 ** Math.floor(Math.log10(v) + 1e-9));
  return out.length > 9 ? out.filter((v) => lead(v) === 1) : out;
}

// labels beside points, moved down until they do not overlap
function placeLabels(pts, W, R) {
  const taken = pts.map((q) => ({ x: q.x - 7, y: q.y + 4, w: 14 }));
  return pts.map((q) => {
    const w = q.label.length * 6.9;
    const fitsRight = q.x + 9 + w <= W - R;
    // candidate spots: above the point, right side first (left if no room),
    // then the other side, then higher, then below
    const sides = fitsRight ? [false, true] : [true, false];
    const cand = [];
    for (const dy of [-9, -23, 17, -37, 31]) for (const left of sides) cand.push({ left, ly: q.y + dy });
    const hit = (lx, ly) => taken.some((b) => lx < b.x + b.w && lx + w > b.x && Math.abs(ly - b.y) < 13);
    let pick = cand.find((c) => { const lx = c.left ? q.x - 9 - w : q.x + 9; return lx > 0 && !hit(lx, c.ly); }) || cand[0];
    const lx = pick.left ? q.x - 9 - w : q.x + 9;
    taken.push({ x: lx, y: pick.ly, w });
    return { lx, ly: pick.ly, w, right: pick.left };
  });
}

// ---- 1. score against cost ------------------------------------------------------
//
// points: [{label, x (per task) | null, y (0..1), n, of, human, detail}]
// x: {name, fmt}; y: {name}
export function frontierChart(points, x, y) {
  const have = points.filter((p) => p.x !== null && p.x > 0);
  const missing = points.length - have.length;
  const wrap = h('<div class="chartwrap frontier"></div>');
  const note = missing
    ? `<p class="note">Not drawn, no reported ${esc(x.name)}: ${points.filter((p) => !have.includes(p)).map((p) => esc(p.label)).join(', ')} (${missing} of ${points.length}).</p>`
    : '';
  if (!have.length) {
    wrap.appendChild(h(`<p class="note">No point in this selection reported its ${esc(x.name)}. Figures are never estimated here; add them to the attempts log or <code>run.json</code>.</p>`));
    wrap.appendChild(dataTable(points, x, y));
    return wrap;
  }
  const W = 760, H = 380, L = 56, B = 46, T = 30, R = 20;
  const xs = have.map((p) => p.x);
  let lo = Math.min(...xs), hi = Math.max(...xs);
  // pad half a decade either side (a single point gets a whole decade around it)
  lo = lo / (hi / lo < 3 ? 3.2 : 1.8); hi = hi * (hi / lo < 9 ? 3.2 : 1.8);
  const X = (v) => L + ((Math.log10(v) - Math.log10(lo)) / (Math.log10(hi) - Math.log10(lo))) * (W - L - R);
  const Y = (v) => T + (1 - v) * (H - T - B);
  let svg = `<svg class="chart" viewBox="0 0 ${W} ${H}" role="img" aria-label="share of tasks passed against ${esc(x.name)} per task, logarithmic">`;
  for (const v of [0, 0.25, 0.5, 0.75, 1]) {
    svg += `<line x1="${L}" x2="${W - R}" y1="${Y(v)}" y2="${Y(v)}" class="gl"/><text x="${L - 8}" y="${Y(v) + 4}" text-anchor="end" class="ax">${v * 100} %</text>`;
  }
  const ticks = x.ticks ? x.ticks.filter((v) => v >= lo && v <= hi) : logTicks(lo, hi);
  for (const v of (ticks.length >= 2 ? ticks : logTicks(lo, hi))) {
    svg += `<line x1="${X(v)}" x2="${X(v)}" y1="${T}" y2="${H - B}" class="gl v"/><text x="${X(v)}" y="${H - B + 17}" text-anchor="middle" class="ax">${esc(x.fmt(v))}</text>`;
  }
  svg += `<text x="${(W + L) / 2}" y="${H - 6}" text-anchor="middle" class="ax">${esc(x.name)} per task (reported, log scale)</text>`;
  svg += `<text x="12" y="${(T + H - B) / 2}" text-anchor="middle" class="ax" transform="rotate(-90 12 ${(T + H - B) / 2})">${esc(y.name)}</text>`;
  // human baseline: a line across at its score
  for (const p of points.filter((q) => q.human)) {
    svg += `<line x1="${L}" x2="${W - R}" y1="${Y(p.y)}" y2="${Y(p.y)}" class="human-line"/><text x="${L + 4}" y="${Y(p.y) + 14}" class="ax hl">person: ${esc(p.label)}, ${pc(p.y)}</text>`;
  }
  // frontier: best score at or below each cost, as a staircase through the
  // leaders -- one per series (level), never across levels
  const series = [...new Set(have.map((p) => p.series ?? ''))].sort();
  const shapeOf = (p) => series.indexOf(p.series ?? '') % 3;   // 0 circle, 1 square, 2 triangle
  const front = [];
  series.forEach((sname, si) => {
    const byX = have.filter((p) => (p.series ?? '') === sname).sort((a, b) => a.x - b.x || b.y - a.y);
    const f = [];
    for (const p of byX) if (!f.length || p.y > f[f.length - 1].y) f.push(p);
    let d = `M${X(f[0].x)} ${Y(f[0].y)}`;
    for (let i = 1; i < f.length; i++) d += ` H${X(f[i].x)} V${Y(f[i].y)}`;
    d += ` H${W - R}`;
    svg += `<path d="${d}" class="front s${si % 3}"/>`;
    front.push(...f);
  });
  const pts = have.map((p) => ({ p, x: X(p.x), y: Y(p.y), label: p.label })).sort((a, b) => a.x - b.x);
  const labels = placeLabels(pts, W, R);
  pts.forEach((q, i) => {
    const l = labels[i];
    const onFront = front.includes(q.p);
    const tip = `<b>${esc(q.p.label)}</b><br>${esc(y.name)}: <span class="num">${q.p.n}/${q.p.of} = ${pc(q.p.y)}</span><br>${esc(x.name)} per task: <span class="num">${esc(x.fmt(q.p.x))}</span>${q.p.detail ? `<br><span class="dim">${esc(q.p.detail)}</span>` : ''}${onFront ? '<br><i>on the frontier</i>' : ''}`;
    if (Math.abs(l.ly - q.y + 9) > 1) svg += `<line x1="${q.x}" y1="${q.y}" x2="${l.right ? l.lx + l.w : l.lx}" y2="${l.ly - 4}" class="lead"/>`;
    const cls = `pt ${onFront ? 'on' : ''}`, at = `tabindex="0" data-tip="${esc(tip)}"`, sh = shapeOf(q.p);
    svg += q.p.human
      ? `<path d="M${q.x} ${q.y - 7} l7 7 -7 7 -7 -7z" class="${cls} human" ${at}/>`
      : sh === 1 ? `<rect x="${q.x - 5.5}" y="${q.y - 5.5}" width="11" height="11" class="${cls}" ${at}/>`
      : sh === 2 ? `<path d="M${q.x} ${q.y - 7} l6.5 11.5 h-13z" class="${cls}" ${at}/>`
      : `<circle cx="${q.x}" cy="${q.y}" r="6" class="${cls}" ${at}/>`;
    svg += `<text x="${l.lx}" y="${l.ly}" class="pl">${esc(q.p.label)}</text>`;
  });
  wrap.appendChild(h(`<div class="svgbox">${svg}</svg></div>`));
  if (series.length > 1) {
    const sym = ['<svg viewBox="0 0 14 14"><circle cx="7" cy="7" r="5"/></svg>', '<svg viewBox="0 0 14 14"><rect x="2" y="2" width="10" height="10"/></svg>', '<svg viewBox="0 0 14 14"><path d="M7 1.5 l6 10.5 h-12z"/></svg>'];
    wrap.appendChild(h(`<p class="legend-line series">${series.map((s, i) => `<span>${sym[i % 3]} ${esc(s)}, with its own frontier</span>`).join('')}</p>`));
  }
  if (note) wrap.appendChild(h(note));
  wrap.appendChild(dataTable(points, x, y, front));
  hoverable(wrap);
  return wrap;
}

function dataTable(points, x, y, front = []) {
  const rows = [...points].sort((a, b) => b.y - a.y || (a.x ?? Infinity) - (b.x ?? Infinity));
  return h(`<div class="tablewrap ctable"><table class="board">
    <thead><tr><th>model · CAD tool</th><th class="r">${esc(y.name)}</th><th class="r">tasks</th><th class="r rep-h">${esc(x.name)} per task</th><th>frontier</th><th>runs</th></tr></thead>
    <tbody>${rows.map((p) => `<tr><td class="name">${p.human ? '◇ ' : ''}${esc(p.label)}</td><td class="r num">${pc(p.y)}</td><td class="r num">${p.n}/${p.of}</td>
      <td class="r">${p.x === null ? '<span class="missing" title="not reported">—</span>' : `<span class="rep num">${esc(x.fmt(p.x))}</span>${p.xPartial ? ` <span class="partial">(${esc(p.xPartial)})</span>` : ''}`}</td>
      <td>${front.includes(p) ? 'yes' : ''}</td><td class="dim">${esc(p.detail || '')}</td></tr>`).join('')}</tbody></table></div>`);
}

// ---- 2. pass rate by level, one small chart per model ---------------------------
//
// models: [{label, levels: {level: {n, of}}}], levels: sorted list
export function levelMultiples(models, levels, yName) {
  const wrap = h('<div class="chartwrap"></div>');
  if (!models.length || !levels.length) { wrap.appendChild(h('<p class="note">Nothing attempted in this selection.</p>')); return wrap; }
  const grid = h('<div class="multiples"></div>');
  const W = 220, H = 140, L = 34, B = 24, T = 10, R = 12;
  const X = (i) => (levels.length === 1 ? (L + W - R) / 2 : L + (i / (levels.length - 1)) * (W - L - R));
  const Y = (v) => T + (1 - v) * (H - T - B);
  for (const m of models) {
    let svg = `<svg class="chart small" viewBox="0 0 ${W} ${H}" role="img" aria-label="${esc(m.label)}: ${esc(yName)} by level">`;
    for (const v of [0, 0.5, 1]) svg += `<line x1="${L}" x2="${W - R}" y1="${Y(v)}" y2="${Y(v)}" class="gl"/><text x="${L - 6}" y="${Y(v) + 4}" text-anchor="end" class="ax">${v * 100}</text>`;
    levels.forEach((lv, i) => { svg += `<text x="${X(i)}" y="${H - 6}" text-anchor="middle" class="ax">L${esc(lv)}</text>`; });
    const pts = levels.map((lv, i) => ({ lv, i, b: m.levels[lv] })).filter((p) => p.b && p.b.of);
    // the line joins neighbouring levels only; a level not attempted breaks it
    let d = '';
    pts.forEach((p, k) => { d += `${k && pts[k - 1].i === p.i - 1 ? 'L' : 'M'}${X(p.i)} ${Y(p.b.n / p.b.of)} `; });
    if (d) svg += `<path d="${d}" class="lvline"/>`;
    for (const p of pts) {
      const tip = `<b>${esc(m.label)}</b>, level ${esc(p.lv)}<br>${esc(yName)}: <span class="num">${p.b.n}/${p.b.of} = ${pc(p.b.n / p.b.of)}</span>`;
      svg += `<circle cx="${X(p.i)}" cy="${Y(p.b.n / p.b.of)}" r="4.5" class="pt" tabindex="0" data-tip="${esc(tip)}"/>`;
    }
    grid.appendChild(h(`<figure class="mult"><figcaption>${esc(m.label)}</figcaption>${svg}</svg></figure>`));
  }
  wrap.appendChild(grid);
  wrap.appendChild(h(`<div class="tablewrap ctable"><table class="board"><thead><tr><th>model · CAD tool</th>${levels.map((l) => `<th class="r">level ${esc(l)}</th>`).join('')}</tr></thead>
    <tbody>${models.map((m) => `<tr><td class="name">${esc(m.label)}</td>${levels.map((l) => { const b = m.levels[l]; return `<td class="r num">${b && b.of ? `${b.n}/${b.of} <span class="pc">${pc(b.n / b.of)}</span>` : '<span class="dim">·</span>'}</td>`; }).join('')}</tr>`).join('')}</tbody></table></div>`));
  hoverable(wrap);
  return wrap;
}

// ---- the requirement ladder: how far up each model's parts get ----------------------
//
// models: [{label, rungs: [{label, reached, measured, required}]}]: per rung,
// of the parts where it was measured, how many met it and every measured
// rung below (src/jointbench/web/ladder.py). One stepped bar per model; a
// rung no part of this model was measured on is a gap, never a zero.
export function ladderChart(models) {
  const wrap = h('<div class="chartwrap"></div>');
  if (!models.length) { wrap.appendChild(h('<p class="note">Nothing attempted in this selection.</p>')); return wrap; }
  const rungs = models[0].rungs.map((r) => r.label);
  const W = 300, H = 46, L = 4, R = 4, T = 4, B = 4, step = (W - L - R) / rungs.length;
  const grid = h(`<div class="ladders" style="--n:${rungs.length}"><span></span><div class="lad-cols lad-head">${rungs.map((r) => `<span>${esc(r)}</span>`).join('')}</div></div>`);
  for (const m of models) {
    let svg = `<svg class="chart lad" viewBox="0 0 ${W} ${H}" preserveAspectRatio="none" role="img" aria-label="${esc(m.label)}: share of parts reaching each rung">`;
    svg += `<line x1="${L}" x2="${W - R}" y1="${H - B}" y2="${H - B}" class="gl"/>`;
    let d = '', prev = null;
    m.rungs.forEach((r, i) => {
      const x0 = L + i * step, x1 = x0 + step;
      if (!r.measured) { prev = null; svg += `<rect x="${x0 + 1}" y="${T}" width="${step - 2}" height="${H - T - B}" class="lad-na"/>`; return; }
      const y = T + (1 - r.reached / r.measured) * (H - T - B);
      svg += `<rect x="${x0 + 1}" y="${y}" width="${step - 2}" height="${H - B - y}" class="lad-bar${r.required ? ' req' : ''}" tabindex="0" data-tip="${esc(`<b>${esc(m.label)}</b> · ${esc(r.label)}<br><span class="num">${r.reached}/${r.measured}</span> of the parts where it was measured reached it${r.required ? `<br>required in ${r.required} task${r.required === 1 ? '' : 's'}` : '<br>reported, not required'}`)}"/>`;
      d += `${prev === null ? 'M' : 'L'}${x0} ${y} L${x1} ${y} `;
      prev = y;
    });
    if (d) svg += `<path d="${d}" class="lad-line"/>`;
    svg += '</svg>';
    grid.appendChild(h(`<span class="lad-name">${esc(m.label)}</span>`));
    grid.appendChild(h(`<div class="lad-svg">${svg}<div class="lad-cols lad-nums">${m.rungs.map((r) => `<span class="num">${r.measured ? `${r.reached}/${r.measured}` : '<span class="dim">—</span>'}</span>`).join('')}</div></div>`));
  }
  wrap.appendChild(grid);
  wrap.appendChild(h('<p class="note">A part reaches a rung when it meets that rung and every measured rung below it. A rung not measured for a part neither counts nor blocks; its share is over the parts where it was measured. Solid bars: the rung was required in at least one of these tasks; light bars: only reported. Passing still means every required rung, as the headline figure counts it.</p>'));
  hoverable(wrap);
  return wrap;
}

// ---- 3. material among passed tasks ------------------------------------------------
//
// models: [{label, values: [{v, task}]}]; v = par volume / part volume (1 = par, higher is leaner)
export function materialChart(models) {
  const wrap = h('<div class="chartwrap"></div>');
  const withV = models.filter((m) => m.values.length);
  const excl = models.flatMap((m) => (m.excluded || []).map((x) => ({ ...x, label: m.label })));
  const exclNote = excl.length
    ? `<details class="ctable"><summary>${excl.length} passing ${excl.length === 1 ? 'part is' : 'parts are'} not counted (the grader says why)</summary><ul class="plain">${excl.map((x) => `<li>${esc(x.label)} · ${esc(x.task)}: ${esc(x.note)}</li>`).join('')}</ul></details>`
    : '';
  if (!withV.length) {
    wrap.appendChild(h(`<div><p class="note">No passing part in this selection has a material figure that counts. Only tasks with a par volume (level 3 on) have one; nothing is estimated.</p>${exclNote}</div>`));
    return wrap;
  }
  const all = withV.flatMap((m) => m.values.map((x) => x.v));
  const lo = Math.min(0.9, ...all) * 0.95, hi = Math.max(1.1, ...all) * 1.05;
  const W = 720, row = 30, L = 150, R = 18, T = 10, B = 34, H = T + B + row * withV.length;
  const X = (v) => L + ((v - lo) / (hi - lo)) * (W - L - R);
  let svg = `<svg class="chart" viewBox="0 0 ${W} ${H}" role="img" aria-label="material used by passing parts, per model">`;
  const step = (hi - lo) > 2 ? 0.5 : 0.25;
  for (let v = Math.ceil(lo / step) * step; v <= hi; v += step) svg += `<line x1="${X(v)}" x2="${X(v)}" y1="${T}" y2="${H - B}" class="gl v ${Math.abs(v - 1) < 1e-9 ? 'par' : ''}"/><text x="${X(v)}" y="${H - B + 16}" text-anchor="middle" class="ax">${v.toFixed(2)}×</text>`;
  svg += `<text x="${(W + L) / 2}" y="${H - 4}" text-anchor="middle" class="ax">material score: par volume ÷ part volume (1 = par; higher is leaner)</text>`;
  withV.forEach((m, i) => {
    const y = T + row * i + row / 2;
    const vs = m.values.map((x) => x.v).sort((a, b) => a - b);
    const med = vs.length % 2 ? vs[(vs.length - 1) / 2] : (vs[vs.length / 2 - 1] + vs[vs.length / 2]) / 2;
    svg += `<text x="${L - 10}" y="${y + 4}" text-anchor="end" class="pl">${esc(m.label)}</text>`;
    for (const x of m.values) svg += `<circle cx="${X(x.v)}" cy="${y}" r="4" class="pt dot" tabindex="0" data-tip="${esc(`<b>${esc(m.label)}</b> · ${esc(x.task)}<br>material score <span class="num">${x.v.toFixed(3)}</span>`)}"/>`;
    svg += `<line x1="${X(med)}" x2="${X(med)}" y1="${y - 10}" y2="${y + 10}" class="med" data-tip="${esc(`<b>${esc(m.label)}</b><br>median <span class="num">${med.toFixed(3)}</span> over ${vs.length} passes`)}"/>`;
  });
  wrap.appendChild(h(`<div class="svgbox">${svg}</svg></div>`));
  if (exclNote) wrap.appendChild(h(exclNote));
  wrap.appendChild(h(`<div class="tablewrap ctable"><table class="board"><thead><tr><th>model · CAD tool</th><th class="r">passes measured</th><th class="r">heaviest</th><th class="r">median</th><th class="r">leanest</th></tr></thead>
    <tbody>${withV.map((m) => { const v = m.values.map((x) => x.v).sort((a, b) => a - b); const med = v.length % 2 ? v[(v.length - 1) / 2] : (v[v.length / 2 - 1] + v[v.length / 2]) / 2; return `<tr><td class="name">${esc(m.label)}</td><td class="r num">${v.length}</td><td class="r num">${v[0].toFixed(3)}</td><td class="r num">${med.toFixed(3)}</td><td class="r num">${v[v.length - 1].toFixed(3)}</td></tr>`; }).join('')}</tbody></table></div>`));
  hoverable(wrap);
  return wrap;
}

// ---- 4. which model passed which task -------------------------------------------------
//
// models: [{label, cells: {taskId: {state: 'first'|'within'|'fail'|null, tries, key}}}]
// tasks: [{id, level}] ; sorted here by how many models passed, fewest last
export function taskStrip(models, tasks) {
  const wrap = h('<div class="chartwrap"></div>');
  if (!models.length || !tasks.length) { wrap.appendChild(h('<p class="note">Nothing attempted in this selection.</p>')); return wrap; }
  const passedBy = (t) => models.filter((m) => { const c = m.cells[t.id]; return c && (c.state === 'first' || c.state === 'within'); }).length;
  const tried = (t) => models.filter((m) => m.cells[t.id] && m.cells[t.id].state).length;
  const order = [...tasks].map((t) => ({ ...t, p: passedBy(t), a: tried(t) }))
    .sort((a, b) => (b.a ? b.p / b.a : -1) - (a.a ? a.p / a.a : -1) || String(a.id).localeCompare(String(b.id), undefined, { numeric: true }));
  const C = order.length > 60 ? 9 : 14, L = 150, T = 30, R = 10, B = 10, row = C + 4;
  const W = Math.max(L + order.length * C + R, L + 200), H = T + models.length * row + B;
  let svg = `<svg class="chart strip" viewBox="0 0 ${W} ${H}" width="${W}" height="${H}" role="img" aria-label="which model passed which task, tasks sorted from easiest to hardest">`;
  svg += `<text x="${L}" y="12" class="ax">easiest</text><text x="${Math.max(L + order.length * C, L + 200)}" y="12" text-anchor="end" class="ax">hardest →</text>`;
  order.forEach((t, j) => { if (C >= 14 || j % 5 === 0) svg += `<text x="${L + j * C + C / 2}" y="${T - 6}" text-anchor="middle" class="ax tiny">${j + 1}</text>`; });
  models.forEach((m, i) => {
    const y = T + i * row;
    svg += `<text x="${L - 10}" y="${y + C - 3}" text-anchor="end" class="pl">${esc(m.label)}</text>`;
    order.forEach((t, j) => {
      const c = m.cells[t.id], x = L + j * C;
      const what = !c || !c.state ? 'not attempted' : c.state === 'first' ? 'passed at the first try' : c.state === 'within' ? `passed on try ${c.tries}` : `failed (${c.tries} tries)`;
      const tip = `<b>${esc(t.id)}</b> · level ${esc(t.level)} · ${esc(m.label)}<br>${what}<br><span class="dim">passed by ${t.p} of ${t.a} that tried it</span>`;
      const cls = !c || !c.state ? 'na' : c.state;
      const shape = `<rect x="${x + 1}" y="${y + 1}" width="${C - 2}" height="${C - 2}" rx="1.5" class="cell ${cls}" tabindex="${c && c.state ? 0 : -1}" data-tip="${esc(tip)}"/>`;
      svg += c && c.key ? `<a href="#/task/${encodeURIComponent(c.key)}">${shape}</a>` : shape;
    });
  });
  wrap.appendChild(h(`<div class="svgbox scroll">${svg}</svg></div>`));
  wrap.appendChild(h(`<p class="legend-line"><span class="sw first"></span> passed at the first try <span class="sw within"></span> passed in later tries <span class="sw fail"></span> failed <span class="sw na"></span> not attempted</p>`));
  wrap.appendChild(h(`<details class="ctable"><summary>Table: every task, every model</summary><div class="tablewrap"><table class="board"><thead><tr><th class="r">#</th><th>task</th><th class="r">level</th><th class="r">passed by</th>${models.map((m) => `<th>${esc(m.label)}</th>`).join('')}</tr></thead>
    <tbody>${order.map((t, j) => `<tr><td class="r num dim">${j + 1}</td><td class="name">${esc(t.id)}</td><td class="r num">${esc(t.level)}</td><td class="r num">${t.p}/${t.a}</td>${models.map((m) => { const c = m.cells[t.id]; return `<td>${!c || !c.state ? '<span class="dim">·</span>' : c.state === 'fail' ? `<span class="bad-t">fail</span> <span class="dim num">${c.tries}</span>` : `pass <span class="dim num">${c.tries}</span>`}</td>`; }).join('')}</tr>`).join('')}</tbody></table></div></details>`));
  hoverable(wrap);
  return wrap;
}
