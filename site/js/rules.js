// Runs graded before the current rules are development data: sample data
// for building and checking the pages, never the benchmark's results. The
// server decides which runs those are (src/jointbench/web/rules.py: the
// task's generator version and the requirements it makes decide the pass);
// this file only words the notice the same way on every page.

const esc = (s) => String(s ?? '').replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));

export const DEV_TAG = 'development data, earlier rules';
export const isDevRun = (r) => !(r && r.rules && r.rules.current);

const ruleText = (scores) => (scores && scores.current_rules && scores.current_rules.label)
  || 'each part graded against exactly the requirements its task states (fits always; clean, makeable, sturdy where the task requires them), by the current grader, on a task from a generator that states them';

// the notice over scores and compare when any run shown is development data
export function devNotice(scores, runs) {
  const dev = (runs || []).filter(isDevRun);
  if (!dev.length) return '';
  const all = dev.length === runs.length;
  // "generator version 1, current 3" and "... 2, current 3" read as one line
  const raw = [...new Set(dev.flatMap((r) => (r.rules && r.rules.why) || []))];
  const gens = raw.map((w) => /^generator version (\d+), the rules hold from (\d+)$/.exec(w)).filter(Boolean);
  const why = raw.filter((w) => !/^generator version/.test(w));
  if (gens.length) {
    const vs = [...new Set(gens.map((m) => +m[1]))].sort((a, b) => a - b);
    why.unshift(`made by generator version ${vs.join(' and ')} (the rules hold from ${gens[0][2]})`);
  }
  return `<div class="devnote" role="note"><b>${all ? 'Development data, earlier rules.' : `${dev.length} of ${runs.length} runs are development data, earlier rules.`}</b>
    ${all ? 'These runs were' : 'Those runs were'} graded under earlier rules. <i>Current rules</i> means: ${esc(ruleText(scores))}. They are kept to build and check these pages; they are not the benchmark's results. Results come from runs on a frozen task set under the current rules, and there are none yet.${why.length ? ` <span class="dim">Why: ${why.map(esc).join('; ')}.</span>` : ''}</div>`;
}

// the scores and compare pages of a copy that left every run out
export function noResults(scores, title) {
  const n = scores && scores.left_out ? scores.left_out.runs : 0;
  return `<div class="page"><div class="section-title"><h2>${esc(title)}</h2></div>
    <p class="prose">No results yet. Results come from runs on a frozen task set under the current rules, and none exist yet. <i>Current rules</i> means: ${esc(ruleText(scores))}.</p>
    ${n ? `<p class="note">${n} run${n === 1 ? '' : 's'} graded under earlier rules ${n === 1 ? 'is' : 'are'} left out of this copy as development data. Their attempts can still be seen on the task pages, marked as such.</p>` : ''}</div>`;
}

export const devTag = () => `<span class="devtag" title="graded before the current rules; not a result">${DEV_TAG}</span>`;
