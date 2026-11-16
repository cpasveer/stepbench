"""A browser view of an answer moving: STL files, the motion as placements per step, and a page.

``export(task_dir, assembly_path, out_dir)`` writes, into ``out_dir``:

  <body>.stl        each body's parts (and its screws) at the start, in world coordinates
  target<k>.stl     fixture B at each required pose (shown as ghosts)
  motion.json       per drive step, each body's 4x4 placement from its start (row by row)
  index.html        three.js page: a slider over the drive, a play button

The motion comes from the same linkage solver the grader uses, so what the page
shows is what was graded.
"""
from __future__ import annotations

import json
import math
import os

import build123d as bd
import numpy as np

from ..geometry import read_step
from ..task import Task
from . import assembly as AS
from .generate import target_solids
from .grade import build_linkage, pose_error, targets

COLORS = {"A": 0x8d96a5, "B": 0x8d96a5}
PALETTE = [0x66a897, 0x3f9fb0, 0xd08a4e, 0xb77fc4, 0xc9b458, 0x6f8fd6]


class _Ans:
    def __init__(self, asm):
        self.asm = asm


def export(task_dir: str, assembly_path: str, out_dir: str, step_deg: float = 1.0) -> str:
    task = Task.load(os.path.join(task_dir, "task.json"))
    mo = task.meta["motion"]
    asm = AS.load(assembly_path, mo["fixed"], mo["moving"])
    folder = os.path.dirname(os.path.abspath(assembly_path))
    os.makedirs(out_dir, exist_ok=True)
    lk = build_linkage(_Ans(asm))
    path, stop = lk.sweep(asm.range_deg, 0.25)
    fx = task.fixed_solids(0)
    bodies = []
    for k, b in enumerate(lk.bodies):
        shapes = [read_step(os.path.join(folder, p.file)) for p in asm.parts if asm.body_of(p.name) == b]
        shapes += [s for f in asm.fasteners if asm.body_of(f.held_by) == b for s in f.bodies()]
        files = []
        if b in fx:
            bd.export_stl(fx[b], os.path.join(out_dir, f"fixture_{b}.stl"))
            files.append({"file": f"fixture_{b}.stl", "color": COLORS[b], "opacity": 0.55})
        if shapes:
            bd.export_stl(bd.Compound(shapes), os.path.join(out_dir, f"body_{b}.stl"))
            files.append({"file": f"body_{b}.stl", "color": PALETTE[k % len(PALETTE)], "opacity": 1.0})
        bodies.append({"name": b, "files": files})
    ghosts = []
    for k, s in enumerate(target_solids(task)):
        bd.export_stl(s, os.path.join(out_dir, f"target{k + 1}.stl"))
        ghosts.append(f"target{k + 1}.stl")
    frames = []
    every = max(1, round(step_deg / 0.25))
    b_idx = lk.bodies.index("B")
    tgs = targets(task)
    for i, (deg, q) in enumerate(path):
        if i % every and i != len(path) - 1:
            continue
        mats = {}
        for k, b in enumerate(lk.bodies):
            pl = lk.placement(q, k)
            m = np.eye(4)
            m[:3, :3] = np.array(pl.rot)
            m[:3, 3] = np.array(pl.origin)
            mats[b] = [round(float(v), 6) for v in m.flatten()]
        bpl = task.poses[0]["B"].then(lk.placement(q, b_idx))
        near = [round(pose_error(bpl, t)[0], 2) for t in tgs]
        frames.append({"deg": round(deg, 3), "m": mats, "to_targets_mm": near})
    meta = {"task": task.id, "bodies": bodies, "targets": ghosts, "frames": frames,
            "stopped": None if stop is None else stop[1], "range_deg": asm.range_deg,
            "pivots": [{"name": v.name, "point": [float(x) for x in v.point], "axis": [float(x) for x in v.axis]}
                       for v in asm.pivots]}
    with open(os.path.join(out_dir, "motion.json"), "w", encoding="utf-8", newline="\n") as f:
        json.dump(meta, f)
    with open(os.path.join(out_dir, "index.html"), "w", encoding="utf-8", newline="\n") as f:
        f.write(PAGE)
    return out_dir


PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Moves prototype</title>
<style>
:root{--bg:#2b3240;--panel:#343c4c;--ink:#e6eaf0;--ink2:#9aa6b8}
body{margin:0;background:var(--bg);color:var(--ink);font:14px/1.5 -apple-system,system-ui,sans-serif}
header{padding:16px 20px 4px}h1{font-size:18px;font-weight:500;margin:0}p{color:var(--ink2);margin:4px 0 0;max-width:960px}
.v{background:var(--panel);border-radius:10px;overflow:hidden;position:relative;height:560px;margin:12px 20px 8px}
.ctl{padding:0 20px 20px;color:var(--ink2)}input{vertical-align:middle;width:360px}button{margin-left:10px}
#near{margin-left:16px}
</style></head><body>
<header><h1 id="title">Moves prototype: a linkage that carries B through stated poses</h1>
<p id="about">Grey: fixture A (stays) and fixture B (carried). Colours: the answer's bodies (the frame on A, the carrier on B, and
each link). Orange ghosts: B at the poses it must reach, in order. Thin orange lines: the declared pivot axes, where they are at the start.
Slide to drive the mechanism; the placements come from the grader's own solver. Drag to turn the view, scroll to zoom.</p></header>
<div class="v" id="v"></div>
<div class="ctl">drive <input type="range" id="a" min="0" value="0" step="1"> <span id="deg">0°</span><button id="play">play</button><span id="near"></span></div>
<script type="importmap">{"imports":{"three":"https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.js","three/addons/":"https://cdn.jsdelivr.net/npm/three@0.160.0/examples/jsm/"}}</script>
<script type="module">
import * as THREE from 'three';
import {STLLoader} from 'three/addons/loaders/STLLoader.js';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
const M=await (await fetch('motion.json')).json();
if(M.title) document.getElementById('title').textContent=M.title; document.getElementById('title').textContent+=' -- '+M.task;
if(M.about) document.getElementById('about').textContent=M.about;
const load=(u)=>new Promise(r=>new STLLoader().load(u,r));
const el=document.getElementById('v'), w=el.clientWidth||900, h=el.clientHeight||560;
const r=new THREE.WebGLRenderer({antialias:true}); r.setPixelRatio(devicePixelRatio); r.setSize(w,h); r.setClearColor(0x343c4c); el.appendChild(r.domElement);
const s=new THREE.Scene(); s.add(new THREE.HemisphereLight(0xffffff,0x445566,1.6)); const dl=new THREE.DirectionalLight(0xffffff,1.2); dl.position.set(1,2,3); s.add(dl);
async function mesh(f,color,op){const geo=await load(f); geo.computeVertexNormals(); const g=new THREE.Group();
  g.add(new THREE.Mesh(geo,new THREE.MeshStandardMaterial({color,roughness:.6,transparent:op<1,opacity:op,depthWrite:op>=1})));
  g.add(new THREE.LineSegments(new THREE.EdgesGeometry(geo,25),new THREE.LineBasicMaterial({color:0x1d2430,transparent:true,opacity:.5}))); return g;}
const groups={};
for(const b of M.bodies){const g=new THREE.Group(); g.matrixAutoUpdate=false; for(const f of b.files) g.add(await mesh(f.file,f.color,f.opacity)); s.add(g); groups[b.name]=g;}
for(const t of M.targets){s.add(await mesh(t,0xe0903a,.25));}
for(const p of M.pivots){const P=new THREE.Vector3(...p.point), D=new THREE.Vector3(...p.axis).normalize().multiplyScalar(40);
  const geo=new THREE.BufferGeometry().setFromPoints([P.clone().sub(D),P.clone().add(D)]);
  const ln=new THREE.Line(geo,new THREE.LineBasicMaterial({color:0xe0903a})); ln.userData.body=null; s.add(ln);}
const box=new THREE.Box3().setFromObject(s), c=box.getCenter(new THREE.Vector3()), size=box.getSize(new THREE.Vector3()).length();
const cam=new THREE.PerspectiveCamera(40,w/h,size/100,size*10); cam.up.set(0,0,1); cam.position.copy(c).add(new THREE.Vector3(size*.9,-size*.9,size*.6));
const ctl=new OrbitControls(cam,r.domElement); ctl.target.copy(c); ctl.update();
new ResizeObserver(()=>{const W=el.clientWidth,H=el.clientHeight; if(W&&H){r.setSize(W,H); cam.aspect=W/H; cam.updateProjectionMatrix();}}).observe(el);
const a=document.getElementById('a'), deg=document.getElementById('deg'), near=document.getElementById('near');
a.max=M.frames.length-1;
function show(i){const f=M.frames[i]; for(const n in f.m){const m=new THREE.Matrix4(); m.set(...f.m[n]); groups[n].matrix.copy(m);}
  deg.textContent=f.label?'':f.deg.toFixed(1)+'°'; near.textContent=f.label?f.label:'B to '+f.to_targets_mm.map((v,k)=>'pose '+(k+1)+': '+v.toFixed(1)+' mm').join(', ')+(M.stopped&&i==M.frames.length-1?' -- '+M.stopped:'');}
a.oninput=()=>show(+a.value); show(0);
let playing=false; document.getElementById('play').onclick=()=>{playing=!playing;};
let dir=1; (function loop(){requestAnimationFrame(loop); if(playing){let v=+a.value+dir; if(v>=+a.max||v<=0) dir=-dir; a.value=v; show(v);} r.render(s,cam);})();
</script></body></html>
"""
