"""A 3D view of a level 4a (assembles) task and an answer, for people: STL files and a page.

Writes into ``out``: the fixed parts, the box (obstacles), the two parts in
place, and ``ways.json``: for each part, in the order it goes in, a list of
its poses along its way in (from just outside the opening to its place), as
4 x 4 matrices that move the part from where it ends up to where it is at
that moment. The page (``index.html``) shows the assembly and a slider that
takes it apart again, last part first, along those poses, with each part's
way in drawn as a line.
"""
from __future__ import annotations

import json
import math
import os

import build123d as bd
import numpy as np

from ..task import Task
from . import joint as J
from .sweep import _trsf_move, _trsf_turn, segment_trsf

FRAMES_PER_STRETCH = 40


def _matrix(trsf) -> list:
    """A gp_Trsf as a 4 x 4 matrix in three.js order (column by column)."""
    m = np.eye(4)
    for i in range(3):
        for j in range(4):
            m[i, j] = trsf.Value(i + 1, j + 1)
    return [round(float(v), 6) for v in m.T.ravel()]


def _poses(solid, entry: J.PartEntry, outside: float) -> list:
    """The part's poses, start to finish, as transforms of its final place."""
    from OCP.gp import gp_Trsf

    total = gp_Trsf()
    for seg in entry.path or []:
        total = segment_trsf(seg).Multiplied(total)
    start = total.Inverted()  # final place -> start of the path
    out = []
    if entry.direction is not None:
        d = np.asarray(entry.direction, float)
        for k in range(FRAMES_PER_STRETCH + 1):  # straight in from outside to the start of the path
            s = 1 - k / FRAMES_PER_STRETCH
            out.append(_trsf_move(-d * outside * s).Multiplied(start))
    else:
        out.append(start)  # the path starts outside the box
    cur = start
    for seg in entry.path or []:
        for k in range(1, FRAMES_PER_STRETCH + 1):
            f = k / FRAMES_PER_STRETCH
            if "move" in seg:
                step = _trsf_move(np.asarray(seg["move"], float) * f)
            else:
                t = seg["turn"]
                step = _trsf_turn(t["point"], t["axis"], float(t.get("degrees", t.get("deg"))) * f)
            out.append(step.Multiplied(cur))
        cur = segment_trsf(seg).Multiplied(cur)
    return out


def write(task: Task, answer_dir: str, out: str, title: str = "", heading: str = "", note: str = "",
          view_dir=None) -> str:
    """The view. ``heading`` and ``note`` replace the page's level 4a heading and explanation (a
    situation other than 4a's says what is shown); ``view_dir``, from the middle towards the camera,
    replaces 4a's."""
    os.makedirs(out, exist_ok=True)
    asm = J.load(os.path.join(answer_dir, "assembly.json"), list(task.parts))
    for n, s in task.fixed_solids(0).items():
        bd.export_stl(s, os.path.join(out, f"fixed_{n}.stl"), tolerance=0.05)
    walls = bd.Compound([o.solid() for o in task.obstacles_at(0)])
    bd.export_stl(walls, os.path.join(out, "box.stl"), tolerance=0.05)
    bb = walls.bounding_box()
    ways = []
    for p in asm.parts:
        solid = bd.import_step(os.path.join(answer_dir, p.file)).solids()[0]
        bd.export_stl(solid, os.path.join(out, f"{p.name}.stl"), tolerance=0.05)
        pb = solid.bounding_box()
        outside = float(np.linalg.norm(np.array(tuple(pb.size)))) * 1.2  # start well clear of the opening
        poses = _poses(solid, p, outside)
        c = np.array(tuple(solid.center()))
        trail = []
        for t in poses:
            v = t.TranslationPart()
            R = np.array([[t.Value(i + 1, j + 1) for j in range(3)] for i in range(3)])
            trail.append([round(float(x), 3) for x in R @ c + np.array([v.X(), v.Y(), v.Z()])])
        ways.append({"name": p.name, "frames": [_matrix(t) for t in poses], "trail": trail})
    with open(os.path.join(out, "ways.json"), "w", encoding="utf-8") as f:
        json.dump({"order": [p.name for p in asm.parts], "ways": ways,
                   "fixed": list(task.parts), "box": [bb.min.X, bb.min.Y, bb.min.Z, bb.max.X, bb.max.Y, bb.max.Z],
                   "opening": (task.meta or {}).get("opening"), "title": title or task.id,
                   **({"heading": heading} if heading else {}), **({"note": note} if note else {}),
                   **({"view_dir": [round(float(x), 4) for x in view_dir]} if view_dir is not None else {})}, f)
    with open(os.path.join(out, "index.html"), "w", encoding="utf-8") as f:
        f.write(PAGE)
    return out


PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Assembles prototype</title>
<style>
:root{--bg:#2b3240;--panel:#343c4c;--ink:#e6eaf0;--ink2:#9aa6b8}
body{margin:0;background:var(--bg);color:var(--ink);font:14px/1.5 -apple-system,system-ui,sans-serif}
header{padding:16px 20px 4px}h1{font-size:18px;font-weight:500;margin:0}p{color:var(--ink2);margin:4px 0 0;max-width:960px}
.v{background:var(--panel);border-radius:10px;overflow:hidden;position:relative;height:620px;margin:12px 20px 8px}
.ctl{padding:0 20px 20px;color:var(--ink2);display:flex;gap:20px;flex-wrap:wrap;align-items:center}
input[type=range]{vertical-align:middle;width:340px}label{cursor:pointer}
</style></head><body>
<header><h1 id="t">Assembles (level 4a) prototype</h1>
<p>A ship in a bottle. Grey: the two fixed parts. Red, see-through: the closed box round them, with its one
opening in the near end wall. Green: part 1, which bolts to the lower fixed part; teal: part 2, which bolts
to the upper one. Slide to take them out again the way they went in, last in first out; the lines are
each part's way in (its middle point). Drag to turn, scroll to zoom.</p></header>
<div class="v" id="v"></div>
<div class="ctl"><span>taken out <input type="range" id="ex" min="0" max="1000" value="0"></span>
<label><input type="checkbox" id="walls" checked> show the box</label>
<label><input type="checkbox" id="trails" checked> show the ways in</label></div>
<script type="importmap">{"imports":{"three":"https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.js","three/addons/":"https://cdn.jsdelivr.net/npm/three@0.160.0/examples/jsm/"}}</script>
<script type="module">
import * as THREE from 'three';
import {STLLoader} from 'three/addons/loaders/STLLoader.js';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
const W=await (await fetch('ways.json')).json();
document.getElementById('t').textContent=W.heading||('Assembles (level 4a) prototype: '+W.title);
if(W.note) document.querySelector('header p').textContent=W.note;
const load=(u)=>new Promise(r=>new STLLoader().load(u,r));
const el=document.getElementById('v'), w=el.clientWidth, h=el.clientHeight;
const r=new THREE.WebGLRenderer({antialias:true}); r.setPixelRatio(devicePixelRatio); r.setSize(w,h); r.setClearColor(0x343c4c); el.appendChild(r.domElement);
const s=new THREE.Scene(); s.add(new THREE.HemisphereLight(0xffffff,0x445566,1.6)); const dl=new THREE.DirectionalLight(0xffffff,1.2); dl.position.set(-1,-2,3); s.add(dl);
async function mesh(file,color,op){const geo=await load(file); geo.computeVertexNormals(); const g=new THREE.Group();
  g.add(new THREE.Mesh(geo,new THREE.MeshStandardMaterial({color,metalness:.05,roughness:.6,transparent:op<1,opacity:op,depthWrite:op>=1,side:THREE.DoubleSide})));
  g.add(new THREE.LineSegments(new THREE.EdgesGeometry(geo,25),new THREE.LineBasicMaterial({color:0x1d2430,transparent:true,opacity:.45})));
  g.matrixAutoUpdate=false; s.add(g); return g;}
for(const n of W.fixed) await mesh('fixed_'+n+'.stl',0x8d96a5,.75);
const box=await mesh('box.stl',0xe0605a,.18);
const colors={part1:0x66a897,part2:0x3f9fb0}; const parts={}; const trails=[];
for(const wy of W.ways){parts[wy.name]=await mesh(wy.name+'.stl',colors[wy.name]||0xcccccc,1);
  const g=new THREE.BufferGeometry().setFromPoints(wy.trail.map(p=>new THREE.Vector3(...p)));
  const L=new THREE.Line(g,new THREE.LineBasicMaterial({color:colors[wy.name]||0xffffff})); s.add(L); trails.push(L);}
const b=W.box, c=new THREE.Vector3((b[0]+b[3])/2,(b[1]+b[4])/2,(b[2]+b[5])/2), size=Math.hypot(b[3]-b[0],b[4]-b[1],b[5]-b[2]);
const cam=new THREE.PerspectiveCamera(35,w/h,size/100,size*30); cam.up.set(0,0,1);
cam.position.copy(c).add((W.view_dir?new THREE.Vector3(...W.view_dir):new THREE.Vector3(-1.4,-1.6,.9)).normalize().multiplyScalar(size*(W.view_dir?1.5:2.2)));
const ctl=new OrbitControls(cam,r.domElement); ctl.target.copy(c); ctl.update();
const draw=()=>{s.updateMatrixWorld(true); r.render(s,cam);}; ctl.addEventListener('change',draw);
const ex=document.getElementById('ex'), m=new THREE.Matrix4();
function place(){const t=+ex.value/1000, n=W.ways.length;
  W.ways.forEach((wy,i)=>{ // the last in comes out first
    const slot=n-1-i, f=Math.min(1,Math.max(0,t*n-slot)); const fr=wy.frames, k=Math.round((fr.length-1)*(1-f));
    parts[wy.name].matrix.copy(m.fromArray(fr[k]));}); draw();}
ex.oninput=place; place();
document.getElementById('walls').onchange=e=>{box.visible=e.target.checked; draw();};
document.getElementById('trails').onchange=e=>{trails.forEach(L=>L.visible=e.target.checked); draw();};
(function loop(){requestAnimationFrame(loop); draw();})();
</script></body></html>
"""
