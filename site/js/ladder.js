// One part's requirement ladder, as a compact row of rungs (valid solid,
// fits, cleanly modelled, makeable, stiff), read from the record
// src/jointbench/web/ladder.py writes. It shows how far a part got; it never
// calls a part a pass: that is the grader's verdict (every required rung met).

const esc = (s) => String(s ?? '').replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));

export function ladderHTML(lad, { compact = false } = {}) {
  if (!lad || !lad.length) return '';
  return `<div class="part-ladder" role="list" aria-label="requirement ladder">${lad.map((r) => {
    const cls = r.state === 'met' ? 'met' : r.state === 'not met' ? 'notmet' : 'na';
    const mk = r.state === 'met' ? '✓' : r.state === 'not met' ? '✕' : '·';
    const tip = `${r.label}: ${r.state}${r.required ? ', required' : ', reported, not required'}`;
    return `<span class="rung ${cls}" role="listitem" title="${esc(tip)}">${mk} ${esc(compact ? r.label.split(' ')[0] : r.label)}${r.required ? '<span class="rq">req</span>' : ''}</span>`;
  }).join('')}</div>`;
}
