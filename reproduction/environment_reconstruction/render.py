"""Read-only CPU rendering of frozen environment reconstruction inputs.

PNG uses matplotlib; the standalone interactive HTML embeds all geometry and a
small Canvas renderer, so viewing does not require a CDN, server or GPU runtime.
"""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
from functools import lru_cache

import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Patch
from matplotlib.colors import to_rgba
from matplotlib.lines import Line2D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection

ROOT = Path(__file__).resolve().parents[2]
ASSET = ROOT / 'reproduction/paper_constructed/assets/fr3_33.json'
TABLE = ROOT / 'csdecomp/tests/test_assets/table.obj'


def read(path):
    return json.loads(Path(path).read_text())


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def fk(q, asset):
    """URDF fixed-origin then local-z joint rotation; no native/CUDA imports."""
    q = np.asarray(q, dtype=float)
    if q.shape != (7,) or not np.isfinite(q).all():
        raise ValueError('FR3 configuration must contain seven finite coordinates')
    frames = {'fr3_link0': np.eye(4)}
    current = np.eye(4)
    for index in range(1, 9):
        origin = asset['kinematics'][f'joint{index}']['kinematic']
        fixed = np.eye(4)
        fixed[:3, :3] = Rotation.from_euler('xyz', [origin[k] for k in ['roll','pitch','yaw']]).as_matrix()
        fixed[:3, 3] = [origin[k] for k in ['x','y','z']]
        current = current @ fixed
        if index < 8:
            rotation = np.eye(4)
            rotation[:3,:3] = Rotation.from_rotvec([0,0,q[index-1]]).as_matrix()
            current = current @ rotation
        frames[f'fr3_link{index}'] = current.copy()
    hand = np.eye(4)
    hand[:3,:3] = Rotation.from_rotvec([0,0,-np.pi/4]).as_matrix()
    frames['fr3_hand'] = current @ hand
    for name, xyz, yaw in [('fr3_leftfinger',[0,.04,.0584],0),
                            ('fr3_rightfinger',[0,-.04,.0584],np.pi),
                            ('fr3_hand_tcp',[0,0,.1034],0)]:
        transform = np.eye(4)
        transform[:3,3] = xyz
        transform[:3,:3] = Rotation.from_rotvec([0,0,yaw]).as_matrix()
        frames[name] = frames['fr3_hand'] @ transform
    return frames


def sphere_mesh(center, radius, nu=16, nv=10):
    u,v = np.meshgrid(np.linspace(0,2*np.pi,nu,endpoint=False), np.linspace(0,np.pi,nv))
    vertices = np.column_stack([np.cos(u).ravel()*np.sin(v).ravel(),
                               np.sin(u).ravel()*np.sin(v).ravel(), np.cos(v).ravel()])
    vertices = vertices * radius + np.asarray(center)
    faces=[]
    for row in range(nv-1):
        for col in range(nu):
            a=row*nu+col; b=row*nu+(col+1)%nu; c=(row+1)*nu+col; d=(row+1)*nu+(col+1)%nu
            faces.extend([[a,c,b],[b,c,d]])
    return vertices, np.asarray(faces)


def combine_meshes(meshes):
    vertices=[]; faces=[]; offset=0
    for v,f in meshes:
        vertices.append(v); faces.append(f+offset); offset+=len(v)
    if not vertices:
        return np.empty((0,3)),np.empty((0,3),dtype=int)
    return np.concatenate(vertices),np.concatenate(faces)


def robot_mesh(q, asset, resolution=12):
    transforms=fk(q,asset); meshes=[]; centers=[]
    for name, balls in asset['spheres'].items():
        transform=transforms[name]
        for ball in balls:
            c=transform[:3,:3]@np.asarray(ball['center'])+transform[:3,3]
            meshes.append(sphere_mesh(c,ball['radius'],resolution,8))
            centers.append(dict(link=name,center=c.tolist(),radius=ball['radius']))
    return (*combine_meshes(meshes), centers)


@lru_cache(maxsize=1)
def official_meshes():
    from reproduction.paper_constructed.build_assets import stl, finger_triangles, SOURCE
    asset=read(ASSET)
    for source in asset['sources']:
        path=SOURCE/source['path']
        if not path.exists() or sha(path)!=source['sha256']:
            raise ValueError(f'Official geometry source identity mismatch: {path}')
    parts={f'fr3_link{i}':stl(SOURCE/f'meshes/robots/fr3/collision/link{i}.stl') for i in range(8)}
    parts['fr3_hand']=stl(SOURCE/'meshes/robot_ee/franka_hand_white/collision/hand.stl')
    parts['fr3_leftfinger']=finger_triangles(False)
    parts['fr3_rightfinger']=finger_triangles(True)
    return parts


def official_robot_mesh(q,asset):
    frames=fk(q,asset);pieces=[]
    for name,triangles in official_meshes().items():
        transform=frames[name]
        vertices=triangles.reshape(-1,3)@transform[:3,:3].T+transform[:3,3]
        pieces.append((vertices,np.arange(len(vertices)).reshape(-1,3)))
    return combine_meshes(pieces)


def table_mesh(translation):
    vertices=[]; faces=[]
    for line in TABLE.read_text().splitlines():
        fields=line.split()
        if not fields: continue
        if fields[0]=='v': vertices.append([float(x) for x in fields[1:4]])
        if fields[0]=='f':
            ids=[int(x.split('/')[0])-1 for x in fields[1:]]
            for j in range(1,len(ids)-1): faces.append([ids[0],ids[j],ids[j+1]])
    return np.asarray(vertices)+translation,np.asarray(faces)


def box_mesh(center,size):
    v=np.array([[x,y,z] for x in [-.5,.5] for y in [-.5,.5] for z in [-.5,.5]])*size+center
    f=np.array([[0,1,3],[0,3,2],[4,6,7],[4,7,5],[0,4,5],[0,5,1],
                [2,3,7],[2,7,6],[0,2,6],[0,6,4],[1,5,7],[1,7,3]])
    return v,f


def _valid_q(value):
    try:
        q=np.asarray(value,float)
        return q if q.shape==(7,) and np.isfinite(q).all() else None
    except (TypeError,ValueError): return None


def _valid_path(value,dimension):
    try:
        p=np.asarray(value,float)
        if p.ndim!=2 or not np.isfinite(p).all(): return None
        if p.shape[1]!=dimension and p.shape[0]==dimension: p=p.T
        return p if p.shape[1]==dimension and len(p)>0 else None
    except (TypeError,ValueError): return None


def _load_case(run,seed):
    candidates=[run/f'cases/fr3_{seed}/case.json',run/f'cases/{seed}/case.json']
    for path in candidates:
        if path.exists(): return read(path),path
    return {},None


def geometry(scene,case,cfg,asset):
    s=cfg.get('fr3_scene',cfg)
    # Public visual table pose is distinct from the collision-box center.
    center=np.asarray(s.get('table_center',[.4,0,-.1]),float)
    size=np.asarray(s.get('table_size',[2.5,2.5,.2]),float)
    translation=np.asarray(s.get('table_visual_translation',[center[0],center[1],center[2]+size[2]/2]),float)
    v,f=table_mesh(translation)
    pieces=[dict(name='Public visual table',group='visual',vertices=v,faces=f,color='#90714d',opacity=.88)]
    v,f=box_mesh(center,size)
    pieces.append(dict(name='Actual table collision box',group='collision',vertices=v,faces=f,color='#5a8bb0',opacity=.12))
    for side,color in [('right','#d83b4b'),('left','#355bd4')]:
        if side+'_ee_lower' in s and side+'_ee_upper' in s:
            lo=np.asarray(s[side+'_ee_lower'],float); hi=np.asarray(s[side+'_ee_upper'],float)
            v,f=box_mesh((lo+hi)/2,hi-lo)
            pieces.append(dict(name=f'{side.title()} TCP sampling box',group='taskboxes',vertices=v,faces=f,color=color,opacity=.025))
    obstacles=np.asarray(scene.get('obstacles',[]),float)
    radius=scene.get('obstacle_radius',s.get('obstacle_radius',.05))
    v,f=combine_meshes([sphere_mesh(c,radius,14,9) for c in obstacles])
    pieces.append(dict(name='Online obstacle spheres',group='obstacles',vertices=v,faces=f,color='#242424',opacity=1))
    qstart=_valid_q(scene.get('start'))
    qgoal=_valid_q(case.get('goal'))
    counts={}; lines=[]
    for name,q,color,group in [('Start configuration',qstart,'#d83b4b','start'),
                               ('Online IK goal',qgoal,'#355bd4','goal')]:
        if q is None: continue
        v,f,balls=robot_mesh(q,asset)
        pieces.append(dict(name=name,group=group,vertices=v,faces=f,color=color,opacity=.62))
        counts[group]=len(balls)
        v,f=official_robot_mesh(q,asset)
        pieces.append(dict(name=name+' official mesh context (not planner geometry)',group='mesh',pose=group,vertices=v,faces=f,color=color,opacity=.68))
    path=_valid_path(case.get('path'),7)
    if path is not None:
        dense=[]
        for a,b in zip(path[:-1],path[1:]):
            dense.extend(a+(b-a)*t for t in np.linspace(0,1,12,endpoint=False))
        dense.append(path[-1])
        tcp=[fk(q,asset)['fr3_hand_tcp'][:3,3] for q in dense]
        accepted=case.get('status')=='validated'
        lines.append(dict(name=('Validated' if accepted else 'FAILED / unvalidated')+' saved DRM path TCP projection',points=np.asarray(tcp),color='#de9900' if accepted else '#bc1e36',group='path',dash=None if accepted else [7,5],accepted=accepted))
    target=np.asarray(scene.get('target_pose',[]),float)
    if target.shape==(4,4) and np.isfinite(target).all():
        for axis,color in enumerate(['#ec5353','#24a064','#337bd9']):
            lines.append(dict(name=f'Target pose axis {axis}',points=np.array([target[:3,3],target[:3,3]+.12*target[:3,axis]]),color=color,group='target'))
    return pieces,lines,dict(obstacle_count=len(obstacles),robot_spheres=counts,
                             has_online_goal=qgoal is not None,has_start=qstart is not None,
                             has_saved_drm_path=path is not None,
                             scene_status=scene.get('status','unknown'),case_status=case.get('status','not_run_or_missing'),
                             table_visual_translation=translation.tolist(),table_collision_center=center.tolist(),table_collision_size=size.tolist())


def draw(ax,pieces,lines,collision=False,small=False,taskboxes=False,official_context=False):
    # One collection sorts ALL mesh faces together. Separate collections sort
    # whole objects and can falsely hide an above-table arm behind the table.
    faces=[];colors=[];wires=[]
    for item in pieces:
        if item['group']=='mesh' and not official_context: continue
        if item['group'] in ['start','goal'] and official_context: continue
        if item['group']=='taskboxes' and not taskboxes: continue
        if item['group']=='collision' and not collision: continue
        if item['group']=='visual' and collision: continue
        v=item['vertices'];f=item['faces']
        if not len(v): continue
        if item['group'] in ['collision','taskboxes']:
            edges=[]
            for i in range(len(v)):
                for j in range(i+1,len(v)):
                    if np.count_nonzero(np.abs(v[i]-v[j])>1e-8)==1:
                        edges.append([v[i],v[j]])
            wires.append((edges,item['color']))
            if item['group']=='collision':
                faces.extend(v[f]);colors.extend([to_rgba(item['color'],.08)]*len(f))
        else:
            faces.extend(v[f]);colors.extend([to_rgba(item['color'],item['opacity'])]*len(f))
    if faces:
        collection=Poly3DCollection(faces,facecolors=colors,edgecolors='none',linewidths=0,zsort='average')
        ax.add_collection3d(collection)
    for edges,color in wires:ax.add_collection3d(Line3DCollection(edges,colors=color,linewidths=.7,alpha=.7))
    for line in lines:
        p=line['points']; ax.plot(p[:,0],p[:,1],p[:,2],color=line['color'],lw=1.2,linestyle='--' if line.get('dash') else '-')
    if collision:
        ax.set(xlim=(-.95,1.75),ylim=(-1.35,1.35),zlim=(-1.05,1.25))
        ax.set_box_aspect((2.7,2.7,2.3))
    else:
        ax.set(xlim=(-.45,1.25),ylim=(-.9,.9),zlim=(-1.05,1.25))
        ax.set_box_aspect((1.7,1.8,2.3))
    ax.view_init(elev=25,azim=-50)
    if small:
        ax.set_xticks([]);ax.set_yticks([]);ax.set_zticks([])
    else:
        ax.set_xlabel('x [m]');ax.set_ylabel('y [m]');ax.set_zlabel('z [m]')
    ax.set_facecolor('#f5f7f9')


def render_forest(run,out):
    scene_path=run/'scenes/forest/figure6.json'
    if not scene_path.exists(): return None
    s=read(scene_path)
    centers=s.get('centers',s.get('obstacles',[]))
    # Accept the frozen digitizer record without inventing task data.
    if isinstance(centers,dict): centers=centers.get('centers',[])
    p=np.asarray(centers,float)
    if p.ndim!=2 or p.shape[1]!=2: return dict(status='unrecognized_forest_centers')
    fig,ax=plt.subplots(figsize=(6,6))
    radius=s.get('obstacle_radius',s.get('radius',.35))
    for c in p: ax.add_patch(Circle(c,radius,fc='#d17b7b',ec='#4b2828',alpha=.8))
    start=np.asarray(s.get('start',[]),float);goal=np.asarray(s.get('goal',[]),float)
    if start.shape==(2,):ax.scatter(*start,c='#161616',label='Saved start')
    if goal.shape==(2,):ax.scatter(*goal,c='#e2b126',marker='*',s=90,label='Saved goal')
    case_path=run/'cases/forest_n00200/case.json'
    if case_path.exists():
        case=read(case_path);path=_valid_path(case.get('path'),2)
        if path is not None:ax.plot(path[:,0],path[:,1],color='#2874a4',label='Saved DRM path')
    bounds=s.get('bounds',[-5,5])
    ax.set(xlim=(-5,5),ylim=(-5,5),xlabel='x [m]',ylabel='y [m]',title='Forest: frozen Figure 6 reconstruction')
    ax.set_aspect('equal');ax.legend(loc='lower right');fig.tight_layout()
    path=out/'forest_scene.png';fig.savefig(path,dpi=180);plt.close(fig)
    return dict(status='rendered',file=path.name,obstacle_count=len(p),source_sha256=sha(scene_path))


HTML = r'''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1"><title>Frozen FR3 environment</title>
<style>body{margin:0;font:15px system-ui;background:#f3f5f8;color:#192738}header{padding:16px 24px;background:white;border-bottom:1px solid #ccd4de}h1{font-size:20px;margin:0 0 7px}p{margin:4px 0;font-size:13px}#controls{display:flex;gap:16px;flex-wrap:wrap;padding:12px 24px;background:#e8eef5}canvas{display:block;width:100vw;height:calc(100vh - 185px);min-height:420px;cursor:grab;touch-action:none}label{font-size:13px}select,button{font:inherit}#status{padding:8px 24px;font-size:12px;background:white}</style>
<header><h1>FR3: saved inputs and actual collision geometry</h1><p>Drag to rotate · Wheel to zoom · Scene selection includes failed or unrun cases. Red: start; blue: online IK goal; black: obstacles; gold: validated DRM path; red dashed: failed or unvalidated DRM path. Lines show TCP projections.</p><p>The visual table and collision box have different dimensions in the public asset. A target frame does not imply an IK solution or successful plan. Official mesh context is display-only; the planner uses the 33 spheres.</p></header>
<div id="controls"><label>Scene <select id="scene"></select></label><label><input type="checkbox" id="visual" checked> Visual table</label><label><input type="checkbox" id="collision"> Collision box</label><label><input type="checkbox" id="start" checked> Start</label><label><input type="checkbox" id="goal" checked> Online IK goal</label><label><input type="checkbox" id="path" checked> DRM path</label><label><input type="checkbox" id="taskboxes"> TCP sampling boxes</label><label><input type="checkbox" id="mesh"> Official mesh context</label><label><input type="checkbox" id="spheres" checked> Planner spheres</label><button id="reset">Reset view</button></div><canvas id="view"></canvas><div id="status"></div>
<script>const DATA=__DATA__;const canvas=document.getElementById('view'),ctx=canvas.getContext('2d');const selector=document.getElementById('scene');Object.keys(DATA).forEach(k=>{let o=document.createElement('option');o.value=k;o.textContent=k;selector.appendChild(o)});let az=-50*Math.PI/180,el=25*Math.PI/180,zoom=1,drag=false,last=[0,0],pending=false;
function project(p,w,h){let x=p[0]-.4,y=p[1],z=p[2]-.1,c=Math.cos(az),s=Math.sin(az),ce=Math.cos(el),se=Math.sin(el);let u=-s*x+c*y,v=-se*(c*x+s*y)+ce*z,depth=ce*(c*x+s*y)+se*z;let scale=Math.min(w,h)*.7*zoom/(1-depth/6);return[w/2+scale*u,h/2-scale*v,depth]}
function rgba(hex,a){let n=parseInt(hex.slice(1),16);return`rgba(${n>>16},${(n>>8)&255},${n&255},${a})`}
function visible(g){let box=document.getElementById(g);return !box||box.checked}
function draw(){pending=false;let box=canvas.getBoundingClientRect(),ratio=window.devicePixelRatio||1,w=box.width,h=box.height;canvas.width=w*ratio;canvas.height=h*ratio;ctx.setTransform(ratio,0,0,ratio,0,0);ctx.fillStyle='#f3f5f8';ctx.fillRect(0,0,w,h);let scene=DATA[selector.value],tris=[];if(!scene)return;for(let mesh of scene.pieces){if(!visible(mesh.group)||(mesh.pose&&!visible(mesh.pose))||(['start','goal'].includes(mesh.group)&&!visible('spheres')))continue;let pts=mesh.vertices.map(p=>project(p,w,h));for(let f of mesh.faces){let a=pts[f[0]],b=pts[f[1]],c=pts[f[2]];tris.push({p:[a,b,c],z:(a[2]+b[2]+c[2])/3,color:mesh.color,alpha:mesh.opacity,wire:mesh.group==='collision'||mesh.group==='taskboxes'})}}tris.sort((a,b)=>a.z-b.z);for(let t of tris){ctx.beginPath();ctx.moveTo(t.p[0][0],t.p[0][1]);ctx.lineTo(t.p[1][0],t.p[1][1]);ctx.lineTo(t.p[2][0],t.p[2][1]);ctx.closePath();ctx.fillStyle=rgba(t.color,t.alpha);ctx.fill();if(t.wire){ctx.strokeStyle=rgba(t.color,.7);ctx.lineWidth=.7;ctx.stroke()}}for(let line of scene.lines){if(!visible(line.group))continue;ctx.beginPath();line.points.map(p=>project(p,w,h)).forEach((p,i)=>i?ctx.lineTo(p[0],p[1]):ctx.moveTo(p[0],p[1]));ctx.strokeStyle=line.color;ctx.lineWidth=2;ctx.setLineDash(line.dash||[]);ctx.stroke();ctx.setLineDash([])}document.getElementById('status').textContent=scene.label+' | '+JSON.stringify(scene.audit)}
function queue(){if(!pending){pending=true;requestAnimationFrame(draw)}}canvas.addEventListener('pointerdown',e=>{drag=true;last=[e.clientX,e.clientY];canvas.setPointerCapture(e.pointerId)});canvas.addEventListener('pointerup',()=>drag=false);canvas.addEventListener('pointermove',e=>{if(!drag)return;az+=(e.clientX-last[0])*.008;el=Math.max(-1.4,Math.min(1.4,el+(e.clientY-last[1])*.008));last=[e.clientX,e.clientY];queue()});canvas.addEventListener('wheel',e=>{e.preventDefault();zoom=Math.max(.2,Math.min(4,zoom*Math.exp(-e.deltaY*.001)));queue()},{passive:false});document.querySelectorAll('input,select').forEach(e=>e.addEventListener('change',queue));document.getElementById('reset').onclick=()=>{az=-50*Math.PI/180;el=25*Math.PI/180;zoom=1;queue()};window.addEventListener('resize',queue);queue();</script></html>'''


def _json(value):
    if isinstance(value,np.ndarray):return value.tolist()
    if isinstance(value,np.generic):return value.item()
    raise TypeError(type(value).__name__)


def render_run(run_dir, output_dir):
    run=Path(run_dir).resolve();out=Path(output_dir).resolve();out.mkdir(parents=True,exist_ok=True)
    manifest=read(run/'manifest.json')
    identity={}
    for path in [ASSET,TABLE,TABLE.with_suffix('.urdf')]:
        relative=str(path.relative_to(ROOT));expected=manifest.get('source_hashes',{}).get(relative)
        actual=sha(path)
        if expected is None or actual!=expected:
            raise ValueError(f'Render geometry differs from frozen run: {relative}; expected={expected}, actual={actual}')
        identity[relative]=actual
    if sha(run/'protocol.json')!=manifest['protocol_sha256']:
        raise ValueError('Frozen rendering protocol hash mismatch')
    cfg=read(run/'protocol.json');asset=read(ASSET)
    count=sum(len(x) for x in asset['spheres'].values())
    if count!=33:raise ValueError(f'Expected actual 33 sphere asset, found {count}')
    paths=sorted((run/'scenes/fr3').glob('*/scene.json'))
    payload={};sources={};audits={}
    for scene_path in paths:
        seed=scene_path.parent.name;scene=read(scene_path);case,case_path=_load_case(run,seed)
        try:
            pieces,lines,audit=geometry(scene,case,cfg,asset)
            payload[seed]=dict(pieces=pieces,lines=lines,audit=audit,label=f'{seed}: {audit["scene_status"]} / {audit["case_status"]}')
            sources[str(scene_path.relative_to(run))]=sha(scene_path)
            if case_path:sources[str(case_path.relative_to(run))]=sha(case_path)
            audits[seed]=audit
        except Exception as exc:
            audits[seed]=dict(status='render_failed',exception_type=type(exc).__name__,error=str(exc))
    if payload:
        cols=5;rows=int(np.ceil(len(payload)/cols))
        fig=plt.figure(figsize=(18,4.0*rows))
        for i,(seed,entry) in enumerate(payload.items(),1):
            ax=fig.add_subplot(rows,cols,i,projection='3d');draw(ax,entry['pieces'],entry['lines'],small=True)
            ax.set_title(entry['label'],fontsize=8)
        fig.suptitle('All frozen FR3 scenes · red start / blue online IK goal · absent goal means no saved IK result',fontsize=12)
        fig.tight_layout();fig.savefig(out/'fr3_all_scenes.png',dpi=150);plt.close(fig)
        key='e00' if 'e00' in payload else next(iter(payload));entry=payload[key]
        fig=plt.figure(figsize=(14,6.7))
        for i,collision in enumerate([False,True],1):
            ax=fig.add_subplot(1,2,i,projection='3d');draw(ax,entry['pieces'],entry['lines'],collision,taskboxes=collision)
            ax.set_title('Public OBJ visual table' if not collision else 'Actual collision table and TCP sampling boxes')
        fig.suptitle(f'{entry["label"]} · actual 33 collision spheres per saved robot pose',fontsize=12)
        fig.legend(handles=[Patch(color='#d83b4b',label='Saved start'),Patch(color='#355bd4',label='Saved online IK goal'),Patch(color='#242424',label='20 obstacles'),Line2D([0],[0],color='#de9900',label='Validated DRM path TCP'),Line2D([0],[0],color='#bc1e36',ls='--',label='Failed / unvalidated DRM path')],loc='lower center',ncol=3)
        fig.tight_layout(rect=(0,.06,1,.96));fig.savefig(out/'fr3_e00_geometry.png',dpi=180);plt.close(fig)
        fig=plt.figure(figsize=(8,8));ax=fig.add_subplot(111,projection='3d')
        draw(ax,entry['pieces'],entry['lines'],official_context=True)
        ax.set_title(f'{key}: official geometry context at actual saved start / goal\nDisplay only; planner geometry remains the 33 collision spheres',fontsize=10)
        fig.tight_layout();fig.savefig(out/'fr3_e00_official_mesh_context.png',dpi=180);plt.close(fig)
        raw=json.dumps(payload,default=_json,separators=(',',':')).replace('</','<\\/')
        (out/'fr3_interactive.html').write_text(HTML.replace('__DATA__',raw))
    forest=render_forest(run,out)
    result=dict(status='rendered' if payload or forest else 'no_renderable_scenes',run=str(run),
                source_sha256=sources,frozen_run_geometry_identity_verified=identity,asset_sha256=sha(ASSET),table_obj_sha256=sha(TABLE),
                protocol_sha256=sha(run/'protocol.json'),fr3=audits,forest=forest,
                official_mesh_source_hashes_verified=asset['sources'],
                no_native_or_gpu_imports=True,standalone_html_no_external_dependencies=True,
                files=[p.name for p in sorted(out.iterdir()) if p.suffix in ['.png','.html']])
    (out/'render_manifest.json').write_text(json.dumps(result,default=_json,indent=2)+'\n')
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('run_dir');parser.add_argument('output_dir')
    args=parser.parse_args();print(json.dumps(render_run(args.run_dir,args.output_dir),indent=2))
