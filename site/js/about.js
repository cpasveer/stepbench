// About: what the benchmark is, what is built and proven, what is not yet,
// and how to run it. The facts are taken from the repository's README.md and
// the checkers' documents (docs/*.md); when those change, change this page.
// Counts of tasks and runs are read from the data this copy carries.

import { getJSON } from './fetch.js';
import { isDevRun } from './rules.js';

const h = (html) => { const t = document.createElement('template'); t.innerHTML = html.trim(); return t.content.firstElementChild; };
const esc = (s) => String(s ?? '').replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));

export async function renderAbout(app) {
  let index = [], scores = null;
  try { index = await getJSON('data/index.json'); } catch { /* counts left out */ }
  try { scores = await getJSON('data/scores.json'); } catch { /* no runs in this copy */ }
  const levels = [...new Set(index.map((t) => t.level))].sort();
  const runs = scores ? scores.runs : [];
  const dev = runs.filter(isDevRun), current = runs.filter((r) => !isDevRun(r));
  const leftOut = scores && scores.left_out ? scores.left_out.runs : 0;
  const devN = dev.length + leftOut;

  const el = h(`<div class="page about">
    <div class="section-title"><h2>About JointBench</h2><span class="n">what it is, what is proven, what is not yet</span></div>

    <div class="about-grid">
    <section>
      <h3>What it tests</h3>
      <p>How well an agent models a 3D part that exists nowhere, for a real situation, with nothing to look up. The situation is one designers meet every day: <b>design the connecting part</b>. Fixed parts are placed in space, each with a flat mounting face and a real bolt pattern, and the answer is one solid that bolts to all of them.</p>
      <p>Every task is generated from a seed, so its answer is in no dataset, and the brief states every position, size and tolerance. What is left to test is modelling and spatial reasoning. Difficulty comes from the situation, not from a description of the shape: parts at angles, obstacles in the way, a third part.</p>
      <p>The answer is a STEP file, graded by measurement on exact geometry (OpenCASCADE): never by resemblance to a reference shape, never by a judge model. Any shape that works passes. The grader reads only the file, so a person in any CAD program and an agent writing code are measured the same way.</p>
    </section>

    <section>
      <h3>How a part is judged</h3>
      <div class="tablewrap"><table class="board plain-t">
        <thead><tr><th>requirement</th><th>checks</th><th>decides the verdict</th></tr></thead>
        <tbody>
          <tr><td>it fits</td><td><code>FILE_VALID</code> <code>HOLES</code> <code>SEATING</code> <code>NO_INTERFERENCE</code>; at level 3 also <code>CLEAR_PATH</code> <code>TOOL_ACCESS</code></td><td>always</td></tr>
          <tr><td>it is sturdy enough</td><td><code>SECTION_MIN</code></td><td>from level 3; reported at 1 and 2</td></tr>
          <tr><td>it is cleanly modelled</td><td><code>Q_*</code> (modelling-quality checker)</td><td>reported only</td></tr>
          <tr><td>it can be made</td><td>the stated process's rules, <code>FDM_*</code> by default</td><td>where the task requires it (connecting parts: at every level since generator 4); else reported</td></tr>
        </tbody>
      </table></div>
      <p>Each check reports pass or fail, a one-line reason and the number it turned on; a failure names the bolt and the amount. Among parts that pass, the one score is <b>material</b>: par volume divided by the part's volume, where par is a deliberately lean answer the generator builds for the task (higher is leaner). A part counts for it only if it also keeps the minimum section and the process's minimum wall.</p>
    </section>

    <section>
      <h3>What is built and proven</h3>
      <ul>
        <li><b>Three levels.</b> Level 1: two parts facing each other. Level 2: faces at an angle and offset, nothing axis-aligned. Level 3: two or three parts, obstacles exactly where a straight connection would run, and every bolt reachable by a straight tool.</li>
        <li><b>Every task is solvable.</b> Tasks are built answer-first: a valid connector, then the fixed parts placed where it fits. The answer and a 1 mm thicker variant are graded by the same grader before a task is kept.</li>
        <li><b>The grader is checked.</b> <code>selftest</code> grades the reference, a differently shaped body and broken copies (holes shifted 0.25 to 2 mm, holes too small, a body pushed into a part, a lifted seat) on seeds 0 to 19: every one fails or passes as it must, 20 of 20, at levels 1 and 2.</li>
        <li><b>Independent evidence.</b> A FreeCAD script that builds connectors from <code>task.json</code> alone, a design the generator never sees, passes 15 of 20 level 1 and 17 of 20 level 2 tasks; every failure is a real clash.</li>
        <li><b>A difficulty floor.</b> A block that fills the gap passes 0 of 20 at levels 1 and 2. The level 1 answer applied blindly passes all of level 1 and none of level 2.</li>
        <li><b>Reproducible.</b> A seed gives byte-identical task folders; the same file gives the same verdict and numbers.</li>
        <li><b>Every attempt is kept</b>, with the graded file, the script if linked, and the feedback as it was printed, so each run can be replayed step by step.</li>
        ${index.length ? `<li><b>In this copy:</b> ${index.length} task${index.length === 1 ? '' : 's'} (level ${levels.join(', ')})${current.length ? `, ${current.length} run${current.length === 1 ? '' : 's'} under the current rules (<a href="#/scores">Scores</a>)` : ''}${devN ? `, and ${devN} run${devN === 1 ? '' : 's'} kept as development data` : ''}.</li>` : ''}
      </ul>
    </section>

    <section>
      <h3>What is not yet</h3>
      <ul>
        <li><b>No frozen task set.</b> Scores are comparable only on the same tasks; "set 1" is frozen once the levels are final. Until then a seed fixes a task only for one generator version.</li>
        <li><b>No results yet.</b> Results come from runs on a frozen task set under the current rules: each part graded against exactly the requirements its own task states, by the current grader. None exist yet. Runs graded under earlier rules are kept only as development data to build and check these pages, and are marked so wherever they appear.</li>
        <li><b>No human baseline yet.</b> Anyone can try a task with <i>Try it yourself</i> on a task page, in their own CAD program; the baseline is set on the frozen set.</li>
        <li><b>Strength and stiffness are not judged.</b> The minimum section is a proxy; a stiffness score is written (<code>docs/stiffness.md</code>) but is not part of the verdict.</li>
        <li><b>Clean modelling is reported, not required.</b></li>
        <li><b>Holes must be true cylinders and seats true planes.</b> Files that approximate them with facets fail. Tested with build123d and FreeCAD only.</li>
        <li><b>Tolerances are stated, not simulated.</b> The 0.1 mm hole-position tolerance is the same for every bolt size; bolts are simplified (tapped holes in the fixed parts, no nuts or washers).</li>
        <li><b>Planned:</b> a mode that withholds the grading rules (testing engineering knowledge apart from modelling), motion (more poses) and further task families.</li>
      </ul>
    </section>

    <section class="wide">
      <h3>Run it</h3>
      <div class="runbox"><pre>uv sync
uv run jointbench gen --seed 0 --n 5 --level 3 --out tasks   <span class="c"># tasks from a seed</span>
uv run jointbench verify tasks/L3-s0                        <span class="c"># grade tasks/L3-s0/output.step</span>
uv run jointbench selftest --level 2 --n 20                 <span class="c"># proof the grader can be trusted</span>
uv run jointbench play tasks                                <span class="c"># this site, live: try a task yourself</span>
uv run jointbench score runs --out scores.json              <span class="c"># score runs; earlier-rules runs are marked</span>
uv run jointbench site runs --out public                    <span class="c"># a static copy; earlier-rules runs left off its scoreboard</span></pre></div>
      <p class="note">The full description, every definition and the known limits are in the repository's <code>README.md</code>; the checkers are described in <code>docs/quality.md</code>, <code>docs/dfm.md</code> and <code>docs/stiffness.md</code>. Licence: Apache-2.0.</p>
      <p class="note">The score chart's idea, score against cost per task, comes from the ARC Prize leaderboard; JointBench applies ARC-AGI's idea, tasks that cannot be memorised, to CAD.</p>
    </section>
    </div>
  </div>`);
  app.replaceChildren(el);
  window.scrollTo(0, 0);
  return { destroy() {} };
}
