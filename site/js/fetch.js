// Loading the page's data.
//
// Meshes are stored under the hash of their STEP file, so a mesh URL always
// means the same bytes: those are kept in memory for the visit (switching
// attempts or tabs never fetches one twice) and the browser may cache them.
// Everything else (task lists, attempts, scores) can change while the page
// is open and is always fetched fresh.

const kept = new Map();   // url -> Promise<json>, content-addressed files only

export function getJSON(url) {
  const fixed = /(^|\/)data\/mesh\//.test(url);
  if (fixed && kept.has(url)) return kept.get(url);
  // a failure says why: the server's own reason when it gave one, else what failed to arrive
  const p = fetch(url, fixed ? {} : { cache: 'no-store' }).catch((e) => {
    throw new Error(`${url}: no answer from the server (${e.message}); is it still running?`);
  }).then(async (r) => {
    if (!r.ok) {
      let why = '';
      try { const b = await r.json(); why = b.error || ''; } catch { /* not JSON */ }
      throw new Error(`${url}: ${r.status}${why ? ` ${why}` : ''}`);
    }
    return r.json();
  });
  if (fixed) { kept.set(url, p); p.catch(() => kept.delete(url)); }
  return p;
}
