// The attempt timeline: step or play through what one submitter did on a
// task, attempt by attempt. Each step shows the part (drawn by the viewer),
// the checks, the grader's feedback as the submitter got it, the code that
// made the part when it was archived, and what changed since the previous
// step (the previous part as a blue ghost in 3D, and a line diff of the code).
//
// Attempts graded before the per-attempt archive existed keep only their
// verdict; their shape is shown as "geometry not kept".

import { materialRatio, fmtRatio } from './quality.js';

const h = (html) => { const t = document.createElement('template'); t.innerHTML = html.trim(); return t.content.firstElementChild; };
const esc = (s) => String(s ?? '').replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
const NONE = '<span class="missing" title="not reported">—</span>';

async function getText(url) {
  const r = await fetch(url, { cache: 'no-store' });
  if (!r.ok) throw new Error(String(r.status));
  return r.text();
}

// the report `jointbench verify` printed for this attempt, rebuilt from the
// logged result (same lines, same order) when no archived text exists
export function reportText(a) {
  const checks = a.checks || [];
  const ok = checks.filter((c) => c.status === 'pass').length;
  const lines = [`${a.passed ? 'PASS' : 'FAIL'}  (${ok}/${checks.length} checks)  ${a.task || ''}`];
  for (const c of checks) {
    const mark = c.status === 'pass' ? 'ok  ' : c.status === 'skipped' ? '--  ' : 'FAIL';
    lines.push(`  ${mark} ${c.name.padEnd(16)} ${c.reason}`);
  }
  const i = a.info || {};
  if (i.volume !== undefined && i.bbox_size) {
    lines.push(`  info volume ${i.volume.toFixed(1)} mm3, bounding box ${i.bbox_size.map((v) => v.toFixed(1)).join(' x ')} mm`);
  }
  return lines.join('\n');
}

// line diff (longest common subsequence); small scripts only
export function lineDiff(a, b) {
  const A = a.split('\n'), B = b.split('\n');
  if (A.length * B.length > 4e6) return null;
  const n = A.length, m = B.length;
  const L = Array.from({ length: n + 1 }, () => new Uint32Array(m + 1));
  for (let i = n - 1; i >= 0; i--) for (let j = m - 1; j >= 0; j--) L[i][j] = A[i] === B[j] ? L[i + 1][j + 1] + 1 : Math.max(L[i + 1][j], L[i][j + 1]);
  const out = [];
  let i = 0, j = 0;
  while (i < n && j < m) {
    if (A[i] === B[j]) { out.push([' ', A[i]]); i++; j++; } else if (L[i + 1][j] >= L[i][j + 1]) out.push(['-', A[i++]]); else out.push(['+', B[j++]]);
  }
  while (i < n) out.push(['-', A[i++]]);
  while (j < m) out.push(['+', B[j++]]);
  return out;
}

function diffHTML(d) {
  // show changes with 2 lines of context, folding the rest
  const keep = new Set();
  d.forEach((x, k) => { if (x[0] !== ' ') for (let q = k - 2; q <= k + 2; q++) keep.add(q); });
  if (!keep.size) return '<div class="dim">no change in the code</div>';
  let out = '', gap = false;
  d.forEach((x, k) => {
    if (!keep.has(k)) { if (!gap) { out += '<div class="fold">⋯</div>'; gap = true; } return; }
    gap = false;
    out += `<div class="dl ${x[0] === '+' ? 'add' : x[0] === '-' ? 'del' : ''}"><span>${x[0]}</span>${esc(x[1]) || ' '}</div>`;
  });
  return out;
}

const fmtSec = (s) => (s >= 60 ? `${(s / 60).toFixed(1)} min` : `${s.toFixed(0)} s`);
const fmtTok = (t) => (t >= 1e3 ? `${(t / 1e3).toFixed(1)} k` : `${t}`);

export function timelineTab(ctx) {
  // ctx: { attempts, selected, tid, go(n), compare, setCompare(on), who, setWho(w), playing, setPlaying(on) }
  const all = ctx.attempts;
  const whos = [...new Set(all.map((a) => a.by || '?'))];
  const who = ctx.who && whos.includes(ctx.who) ? ctx.who : (ctx.selected ? ctx.selected.by : whos[0]);
  const steps = all.filter((a) => (a.by || '?') === who);
  if (!steps.length) return h('<div><p class="note">No attempts yet.</p></div>');
  let k = Math.max(0, steps.findIndex((a) => ctx.selected && a.n === ctx.selected.n));
  const a = steps[k], prev = k > 0 ? steps[k - 1] : null;
  const rp = a.reported || {};
  const arc = a.archive;

  const repLine = [
    rp.tool ? `<span class="rep">${esc(rp.tool)}</span>` : null,
    rp.seconds !== null && rp.seconds !== undefined ? `<span class="rep num">${fmtSec(rp.seconds)}</span>` : null,
    rp.tokens_in !== null && rp.tokens_in !== undefined ? `<span class="rep num">${fmtTok(rp.tokens_in)} in</span>` : null,
    rp.tokens_out !== null && rp.tokens_out !== undefined ? `<span class="rep num">${fmtTok(rp.tokens_out)} out</span>` : null,
    rp.cost !== null && rp.cost !== undefined ? `<span class="rep num">${rp.cost}${rp.currency ? ' ' + esc(rp.currency) : ''}</span>` : null,
    rp.tool_calls !== null && rp.tool_calls !== undefined ? `<span class="rep num">${rp.tool_calls} tool calls</span>` : null,
    rp.model ? `<span class="rep">${esc(rp.model)}${rp.model_version ? ' ' + esc(rp.model_version) : ''}</span>` : null,
  ].filter(Boolean);
  const LINK = { ran: 'ran: verify --build ran it', named: 'named: given with --script', found: 'found: an edited submission.py lay in the task folder; not proof it made the file' };

  const el = h(`<div class="timeline">
    ${whos.length > 1 ? `<label class="sel">submitter <select id="tlwho">${whos.map((w) => `<option ${w === who ? 'selected' : ''}>${esc(w)}</option>`).join('')}</select></label>` : ''}
    <div class="tl-ctl">
      <span class="seg"><button data-s="first" title="first">⏮</button><button data-s="prev" title="previous">◀</button><button data-s="play" title="play through">${ctx.playing ? '❚❚' : '▶'}</button><button data-s="next" title="next">▶︎▶︎</button><button data-s="last" title="last">⏭</button></span>
      <span class="num dim">step ${k + 1} of ${steps.length}</span>
    </div>
    <div class="ticks tl">${steps.map((s, i) => `<button class="tick ${s.passed ? 'ok' : 'bad'} ${i === k ? 'cur' : ''}" data-n="${s.n}" title="attempt ${s.n}: ${s.passed ? 'pass' : 'fail'}">${s.n}</button>`).join('')}</div>
    <div class="tl-head">
      <span class="stamp ${a.passed ? 'ok' : 'bad'} big">${a.passed ? 'pass' : 'fail'}</span>
      <div><b>attempt #${a.n}</b> by <b>${esc(a.by)}</b><br><span class="mono dim">${esc((a.time || '').replace('T', ' ').slice(0, 19))}</span></div>
    </div>
    <div class="tl-rep">${repLine.length ? repLine.join(' · ') + ' <span class="dim">· reported by the submitter</span>' : '<span class="dim">no time, tokens or tool reported for this step</span>'}</div>
    ${a.mesh ? '' : '<p class="note warn">Geometry not kept: this attempt was graded before attempts were archived, and its file has since been overwritten. The verdict below is as the grader logged it.</p>'}
    ${a.step_origin ? `<div class="tl-rep">STEP file header says it was written by <b>${esc(a.step_origin)}</b> <span class="dim">· read from the file, not graded on</span></div>` : ''}
    <div id="tlnote">${(arc && (arc.meta.note || arc.meta.notes)) || (typeof a.note === 'string' && a.note) ? `<p class="note"><b>Note:</b> ${esc((arc && (arc.meta.note || arc.meta.notes)) || a.note)}</p>` : ''}</div>
    <label class="follow"><input type="checkbox" id="tlcmp" ${ctx.compare ? 'checked' : ''} ${prev ? '' : 'disabled'}> show the previous step (#${prev ? prev.n : '–'}) as a blue ghost ${prev && !prev.mesh ? '<span class="dim">(its geometry was not kept)</span>' : ''}</label>
    <div class="h3">What changed</div>
    <div id="tlchg"></div>
    <div class="h3">Feedback ${arc && arc.texts.length ? '<span class="dim">as archived</span>' : '<span class="dim">as <code>verify</code> printed it, rebuilt from the log</span>'}</div>
    <pre class="fb" id="tlfb">${esc(reportText(a))}</pre>
    <div class="h3">Code ${arc && arc.script_link ? `<span class="dim" title="${esc(LINK[arc.script_link] || arc.script_link)}">· linked by ${esc(arc.script_link)}</span>` : ''}</div>
    <div id="tlcode"><p class="note dim">${arc ? (arc.code.length ? 'loading…' : 'no code was archived with this attempt') : 'no archive for this attempt: the code that made it was not kept'}</p></div>
  </div>`);

  // what changed: checks that flipped, plus volume
  const chg = el.querySelector('#tlchg');
  if (!prev) chg.innerHTML = '<p class="note dim">first step</p>';
  else {
    const before = Object.fromEntries((prev.checks || []).map((c) => [c.name, c.status]));
    const rows = (a.checks || []).map((c) => {
      const b = before[c.name];
      const flip = b && b !== c.status;
      return `<tr class="${flip ? (c.status === 'pass' ? 'up' : 'down') : ''}"><td><code>${esc(c.name)}</code></td><td>${esc(b || '–')}</td><td>→</td><td><b>${esc(c.status)}</b></td></tr>`;
    }).join('');
    const v0 = prev.info && prev.info.volume, v1 = a.info && a.info.volume;
    const vol = v0 !== undefined && v1 !== undefined ? `<p class="note num">volume ${v0.toFixed(0)} → ${v1.toFixed(0)} mm³ (${v1 >= v0 ? '+' : ''}${(v1 - v0).toFixed(0)})</p>` : '';
    const mr = a.passed ? fmtRatio(materialRatio(a)) : null;
    chg.innerHTML = `<table class="dtable chg"><tr><th>check</th><th>#${prev.n}</th><th></th><th>#${a.n}</th></tr>${rows}</table>${vol}${mr ? `<p class="note num">material ${mr}</p>` : ''}`;
  }

  // the caller's note, stored as given
  if (arc && arc.note) {
    getText(`data/tasks/${ctx.tid}/archive/${a.n}/${encodeURIComponent(arc.note)}`)
      .then((t) => { el.querySelector('#tlnote').innerHTML = `<p class="note"><b>Note</b> <span class="dim">(reported)</span>: ${esc(t.trim())}</p>`; }).catch(() => {});
  }
  // archived text replaces the rebuilt feedback
  if (arc && arc.texts.length) {
    getText(`data/tasks/${ctx.tid}/archive/${a.n}/${encodeURIComponent(arc.texts[0])}`)
      .then((t) => { el.querySelector('#tlfb').textContent = t; }).catch(() => {});
  }
  // code and its diff from the previous step
  if (arc && arc.code.length) {
    const name = arc.code[0];
    const pa = prev && prev.archive && prev.archive.code.length ? prev.archive : null;
    Promise.all([
      getText(`data/tasks/${ctx.tid}/archive/${a.n}/${encodeURIComponent(name)}`),
      pa ? getText(`data/tasks/${ctx.tid}/archive/${prev.n}/${encodeURIComponent(pa.code[0])}`).catch(() => null) : Promise.resolve(null),
    ]).then(([cur, old]) => {
      const box = el.querySelector('#tlcode');
      const d = old !== null ? lineDiff(old, cur) : null;
      box.innerHTML = `<div class="seg codeseg"><button data-c="diff" class="${d ? 'on' : ''}" ${d ? '' : 'disabled'}>diff vs #${prev ? prev.n : '–'}</button><button data-c="full" class="${d ? '' : 'on'}">${esc(name)}</button></div>
        <div class="code" id="cdiff" ${d ? '' : 'hidden'}>${d ? diffHTML(d) : ''}</div>
        <pre class="code" id="cfull" ${d ? 'hidden' : ''}>${esc(cur)}</pre>`;
      box.querySelector('.codeseg').addEventListener('click', (e) => {
        const b = e.target.closest('button'); if (!b || b.disabled) return;
        for (const x of box.querySelectorAll('.codeseg button')) x.classList.toggle('on', x === b);
        box.querySelector('#cdiff').hidden = b.dataset.c !== 'diff';
        box.querySelector('#cfull').hidden = b.dataset.c !== 'full';
      });
    }).catch(() => { el.querySelector('#tlcode').innerHTML = '<p class="note dim">the archived code could not be read</p>'; });
  }

  el.querySelector('.tl-ctl').addEventListener('click', (e) => {
    const b = e.target.closest('button'); if (!b) return;
    const s = b.dataset.s;
    if (s === 'play') { ctx.setPlaying(!ctx.playing, steps); return; }
    const j = { first: 0, prev: Math.max(0, k - 1), next: Math.min(steps.length - 1, k + 1), last: steps.length - 1 }[s];
    ctx.go(steps[j].n);
  });
  el.querySelector('.ticks.tl').addEventListener('click', (e) => { const b = e.target.closest('button[data-n]'); if (b) ctx.go(+b.dataset.n); });
  const w = el.querySelector('#tlwho');
  if (w) w.addEventListener('change', () => { ctx.setWho(w.value); const first = all.find((x) => (x.by || '?') === w.value); if (first) ctx.go(first.n); });
  el.querySelector('#tlcmp').addEventListener('change', (e) => ctx.setCompare(e.target.checked));
  return el;
}

export function previousStep(attempts, a) {
  if (!a) return null;
  const same = attempts.filter((x) => (x.by || '?') === (a.by || '?') && x.n < a.n);
  return same.length ? same[same.length - 1] : null;
}

export { NONE };
