//! Point-in-solid for many points at once, on a fine triangle copy of the
//! part's surface, with a bounding-volume tree; points in parallel (rayon).
//! Points near the surface, or where three skew rays disagree, are marked
//! unsure and the caller classifies them exactly (OpenCASCADE).

use pyo3::prelude::*;
use pyo3::types::PyBytes;
use rayon::prelude::*;

type V3 = [f64; 3];

#[inline] fn sub(a: V3, b: V3) -> V3 { [a[0]-b[0], a[1]-b[1], a[2]-b[2]] }
#[inline] fn add(a: V3, b: V3) -> V3 { [a[0]+b[0], a[1]+b[1], a[2]+b[2]] }
#[inline] fn mul(a: V3, s: f64) -> V3 { [a[0]*s, a[1]*s, a[2]*s] }
#[inline] fn dot(a: V3, b: V3) -> f64 { a[0]*b[0] + a[1]*b[1] + a[2]*b[2] }

/// Closest point on triangle abc to p (Ericson, Real-Time Collision Detection 5.1.5).
fn closest_on_tri(p: V3, a: V3, b: V3, c: V3) -> V3 {
    let ab = sub(b, a); let ac = sub(c, a); let ap = sub(p, a);
    let d1 = dot(ab, ap); let d2 = dot(ac, ap);
    if d1 <= 0.0 && d2 <= 0.0 { return a; }
    let bp = sub(p, b); let d3 = dot(ab, bp); let d4 = dot(ac, bp);
    if d3 >= 0.0 && d4 <= d3 { return b; }
    let vc = d1*d4 - d3*d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 { let v = d1/(d1-d3); return add(a, mul(ab, v)); }
    let cp = sub(p, c); let d5 = dot(ab, cp); let d6 = dot(ac, cp);
    if d6 >= 0.0 && d5 <= d6 { return c; }
    let vb = d5*d2 - d1*d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 { let w = d2/(d2-d6); return add(a, mul(ac, w)); }
    let va = d3*d6 - d5*d4;
    if va <= 0.0 && (d4-d3) >= 0.0 && (d5-d6) >= 0.0 {
        let w = (d4-d3)/((d4-d3)+(d5-d6)); return add(b, mul(sub(c, b), w));
    }
    let denom = 1.0/(va+vb+vc); let v = vb*denom; let w = vc*denom;
    add(a, add(mul(ab, v), mul(ac, w)))
}

struct Node { lo: V3, hi: V3, left: usize, right: usize, start: usize, count: usize }

struct Bvh { nodes: Vec<Node>, tris: Vec<[V3; 3]>, order: Vec<usize> }

impl Bvh {
    fn new(tris: Vec<[V3; 3]>) -> Bvh {
        let mut order: Vec<usize> = (0..tris.len()).collect();
        let cen: Vec<V3> = tris.iter().map(|t| mul(add(add(t[0], t[1]), t[2]), 1.0/3.0)).collect();
        let mut bvh = Bvh { nodes: Vec::new(), tris, order: Vec::new() };
        let n = order.len();
        bvh.build(&mut order, &cen, 0, n);
        bvh.order = order;
        bvh
    }
    fn build(&mut self, order: &mut Vec<usize>, cen: &Vec<V3>, start: usize, end: usize) -> usize {
        let mut lo = [f64::INFINITY; 3]; let mut hi = [f64::NEG_INFINITY; 3];
        for &i in &order[start..end] { for v in &self.tris[i] { for k in 0..3 { lo[k] = lo[k].min(v[k]); hi[k] = hi[k].max(v[k]); } } }
        let id = self.nodes.len();
        self.nodes.push(Node { lo, hi, left: 0, right: 0, start, count: 0 });
        if end - start <= 4 { self.nodes[id].count = end - start; return id; }
        let mut clo = [f64::INFINITY; 3]; let mut chi = [f64::NEG_INFINITY; 3];
        for &i in &order[start..end] { for k in 0..3 { clo[k] = clo[k].min(cen[i][k]); chi[k] = chi[k].max(cen[i][k]); } }
        let ax = (0..3).max_by(|&a, &b| (chi[a]-clo[a]).partial_cmp(&(chi[b]-clo[b])).unwrap()).unwrap();
        let mid = (start + end) / 2;
        order[start..end].sort_by(|&a, &b| cen[a][ax].partial_cmp(&cen[b][ax]).unwrap().then(a.cmp(&b)));
        let l = self.build(order, cen, start, mid);
        let r = self.build(order, cen, mid, end);
        self.nodes[id].left = l; self.nodes[id].right = r;
        id
    }
    #[inline] fn box_d2(&self, n: &Node, p: V3) -> f64 {
        let mut d = 0.0;
        for k in 0..3 { let g = (n.lo[k]-p[k]).max(p[k]-n.hi[k]).max(0.0); d += g*g; }
        d
    }
    /// (squared distance, closest point) from p to the mesh.
    fn nearest(&self, p: V3) -> (f64, V3) {
        let mut best = f64::INFINITY; let mut bq = p;
        let mut stack: Vec<usize> = vec![0];
        while let Some(id) = stack.pop() {
            let n = &self.nodes[id];
            if self.box_d2(n, p) >= best { continue; }
            if n.count > 0 {
                for &t in &self.order[n.start..n.start+n.count] {
                    let tr = &self.tris[t];
                    let q = closest_on_tri(p, tr[0], tr[1], tr[2]);
                    let d = sub(q, p); let d2 = dot(d, d);
                    if d2 < best { best = d2; bq = q; }
                }
            } else {
                let (a, b) = (n.left, n.right);
                let (da, db) = (self.box_d2(&self.nodes[a], p), self.box_d2(&self.nodes[b], p));
                if da < db { stack.push(b); stack.push(a); } else { stack.push(a); stack.push(b); }
            }
        }
        (best, bq)
    }
}

impl Bvh {
    /// Number of triangles a ray from o along d crosses (t > 0).
    fn crossings(&self, o: V3, d: V3) -> usize {
        let inv = [1.0/d[0], 1.0/d[1], 1.0/d[2]];
        let mut n = 0usize;
        let mut stack: Vec<usize> = vec![0];
        while let Some(id) = stack.pop() {
            let nd = &self.nodes[id];
            // slab test
            let mut t0 = 0.0f64; let mut t1 = f64::INFINITY;
            let mut hit = true;
            for k in 0..3 {
                let (mut a, mut b) = ((nd.lo[k]-o[k])*inv[k], (nd.hi[k]-o[k])*inv[k]);
                if a > b { std::mem::swap(&mut a, &mut b); }
                if a.is_nan() || b.is_nan() { continue; }
                t0 = t0.max(a); t1 = t1.min(b);
                if t0 > t1 { hit = false; break; }
            }
            if !hit { continue; }
            if nd.count > 0 {
                for &t in &self.order[nd.start..nd.start+nd.count] {
                    let tr = &self.tris[t];
                    // Moller-Trumbore
                    let e1 = sub(tr[1], tr[0]); let e2 = sub(tr[2], tr[0]);
                    let pv = [d[1]*e2[2]-d[2]*e2[1], d[2]*e2[0]-d[0]*e2[2], d[0]*e2[1]-d[1]*e2[0]];
                    let det = dot(e1, pv);
                    if det.abs() < 1e-14 { continue; }
                    let id_ = 1.0/det; let tv = sub(o, tr[0]);
                    let u = dot(tv, pv)*id_; if u < 0.0 || u > 1.0 { continue; }
                    let qv = [tv[1]*e1[2]-tv[2]*e1[1], tv[2]*e1[0]-tv[0]*e1[2], tv[0]*e1[1]-tv[1]*e1[0]];
                    let v = dot(d, qv)*id_; if v < 0.0 || u+v > 1.0 { continue; }
                    let tt = dot(e2, qv)*id_; if tt > 1e-12 { n += 1; }
                }
            } else { stack.push(nd.left); stack.push(nd.right); }
        }
        n
    }
}

fn read_f64(b: &[u8]) -> Vec<f64> {
    b.chunks_exact(8).map(|c| f64::from_le_bytes(c.try_into().unwrap())).collect()
}

/// Point-in-solid for many points: 1 inside, 0 outside, -1 not sure (the
/// point is within ``guard`` of the surface, or three skew rays disagree).
/// The caller decides the unsure ones exactly.
#[pyfunction]
fn inside<'py>(py: Python<'py>, tris: &[u8], pts: &[u8], guard: f64) -> Bound<'py, PyBytes> {
    let tf = read_f64(tris);
    let tri: Vec<[V3; 3]> = tf.chunks_exact(9).map(|c| [[c[0],c[1],c[2]],[c[3],c[4],c[5]],[c[6],c[7],c[8]]]).collect();
    let pf = read_f64(pts);
    let out: Vec<i8> = py.allow_threads(|| {
        let bvh = Bvh::new(tri);
        let dirs: [V3; 3] = [[0.5773502691896258, 0.5773502691896257, 0.5773502691896259],
                             [-0.2672612419124244, 0.8017837257372732, 0.5345224838248488],
                             [0.6666666666666666, -0.3333333333333333, 0.6666666666666667]];
        (0..pf.len()/3).into_par_iter().map(|i| {
            let p = [pf[3*i], pf[3*i+1], pf[3*i+2]];
            let (d2, _) = bvh.nearest(p);
            if d2.sqrt() <= guard { return -1i8; }
            let votes: Vec<usize> = dirs.iter().map(|&d| bvh.crossings(p, d) % 2).collect();
            if votes[0] == votes[1] && votes[1] == votes[2] { votes[0] as i8 } else { -1i8 }
        }).collect()
    });
    let bytes: Vec<u8> = out.iter().map(|&v| v as u8).collect();
    PyBytes::new_bound(py, &bytes)
}

/// A fine triangle copy with its tree, built once and queried many times.
#[pyclass]
struct Mesh { bvh: Bvh }

#[pymethods]
impl Mesh {
    #[new]
    fn new(py: Python<'_>, tris: &[u8]) -> Mesh {
        let tf = read_f64(tris);
        let tri: Vec<[V3; 3]> = tf.chunks_exact(9).map(|c| [[c[0],c[1],c[2]],[c[3],c[4],c[5]],[c[6],c[7],c[8]]]).collect();
        Mesh { bvh: py.allow_threads(|| Bvh::new(tri)) }
    }
    fn inside<'py>(&self, py: Python<'py>, pts: &[u8], guard: f64) -> Bound<'py, PyBytes> {
        let pf = read_f64(pts);
        let bvh = &self.bvh;
        let out: Vec<i8> = py.allow_threads(|| (0..pf.len()/3).into_par_iter().map(|i| inside_one(bvh, &pf, i, guard)).collect());
        let bytes: Vec<u8> = out.iter().map(|&v| v as u8).collect();
        PyBytes::new_bound(py, &bytes)
    }
}

const SKEW: [V3; 3] = [[0.5773502691896258, 0.5773502691896257, 0.5773502691896259],
                       [-0.2672612419124244, 0.8017837257372732, 0.5345224838248488],
                       [0.6666666666666666, -0.3333333333333333, 0.6666666666666667]];

fn inside_one(bvh: &Bvh, pf: &[f64], i: usize, guard: f64) -> i8 {
    let p = [pf[3*i], pf[3*i+1], pf[3*i+2]];
    let (d2, _) = bvh.nearest(p);
    if d2.sqrt() <= guard { return -1; }
    let v: Vec<usize> = SKEW.iter().map(|&d| bvh.crossings(p, d) % 2).collect();
    if v[0] == v[1] && v[1] == v[2] { v[0] as i8 } else { -1 }
}

#[pymodule]
fn _fastball(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(inside, m)?)?;
    m.add_class::<Mesh>()?;
    Ok(())
}
