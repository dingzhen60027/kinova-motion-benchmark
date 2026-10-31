"""Self-contained 3-D collision-model replay from saved numerical samples.

No CUDA calls, planner reruns, external JavaScript libraries or network assets.
"""
from __future__ import annotations

import hashlib
import html
import json
from pathlib import Path

import numpy as np
from ..common import resolve_case

BOX_EDGES = [(0, 1), (0, 2), (0, 4), (1, 3), (1, 5), (2, 3),
             (2, 6), (3, 7), (4, 5), (4, 6), (5, 7), (6, 7)]
SIGNS = np.asarray([[x, y, z] for x in (-1., 1.) for y in (-1., 1.) for z in (-1., 1.)])


def _read(path, default=None):
    path = Path(path)
    return json.loads(path.read_text()) if path.exists() else default


def _inputs(attempt):
    result = _read(attempt / 'result.json', {})
    process = _read(attempt / 'process.json', {})
    case = result.get('case') or _read(attempt / 'case.json', {})
    if not case:
        input_file = _read(attempt / 'input.json', {})
        case = input_file.get('case', input_file if 'q_start' in input_file else {})
    if not case:
        # .../run/jobs/<id>/attempts/0001
        for parent in attempt.parents:
            manifest = _read(parent / 'run_manifest.json')
            if manifest:
                job_id = attempt.parent.parent.name
                job = next((j for j in manifest['jobs'] if j['job_id'] == job_id), None)
                if job:
                    case = resolve_case(job)
                break
    if 'q_start' not in case or 'q_goal' not in case:
        raise ValueError('Replay needs the saved fixed input case with q_start/q_goal.')
    return case, result, process


def _trajectory(attempt, case):
    paths = list(attempt.glob('trajectory_samples.npz'))
    if not paths:
        paths = sorted(p for p in attempt.rglob('trajectory_samples.npz') if 'replay' not in p.parts)
    if paths:
        path = paths[-1]
        with np.load(path, allow_pickle=False) as arrays:
            q = arrays['q'].copy()
            time = arrays['time'].copy() if 'time' in arrays else np.linspace(0, 1, len(q))
            clearance = arrays['clearance'].copy() if 'clearance' in arrays else None
        if (q.ndim != 2 or q.shape[1] != 7 or not len(q) or not np.isfinite(q).all()
                or time.shape != (len(q),) or not np.isfinite(time).all() or np.any(np.diff(time) < 0)
                or (clearance is not None and (clearance.shape != (len(q),) or not np.isfinite(clearance).all()))):
            raise ValueError('Saved trajectory samples are malformed.')
        validation = _read(path.parent / 'validation.json', {})
        return q, time, clearance, validation, path, True
    endpoints = []
    for key in ('q_start', 'q_goal'):
        value = np.asarray(case.get(key), float)
        if value.shape == (7,) and np.isfinite(value).all():
            endpoints.append(value)
    states = np.asarray(endpoints) if endpoints else np.zeros((1, 7))
    return states, np.arange(len(states), dtype=float), None, {}, None, False


def _world_boxes(model, q):
    model.plant.SetPositions(model.plant_context, q)
    query = model._query()
    boxes = []
    for entry, geometry_id in zip(model.inventory['geometries'], model._drake_ids):
        if entry['shape'] != 'box':
            raise ValueError('This replay version expects the frozen all-box Kinova primitive model.')
        X = query.GetPoseInWorld(geometry_id)
        corners = (SIGNS * np.asarray(entry['dimensions_m']) / 2) @ X.rotation().matrix().T + X.translation()
        boxes.append(corners)
    return np.asarray(boxes)


def _plot(out, data, q, time, clearance, all_tcp):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    figure = plt.figure(figsize=(10, 8), constrained_layout=True)
    axis = figure.add_subplot(111, projection='3d')
    fixed = np.asarray(data['fixed_boxes'])
    moving = np.asarray(data['frames'][-1]['moving_boxes'])
    start = np.asarray(data['frames'][0]['moving_boxes'])
    for boxes, color, alpha in [(fixed, '#7c8797', .55), (start, '#a6b9dd', .35), (moving, '#1856a5', .95)]:
        for box in boxes:
            for a, b in BOX_EDGES:
                axis.plot(*box[[a, b]].T, color=color, alpha=alpha, linewidth=.8)
    voxels = np.asarray(data['voxels_xyz'])
    if len(voxels):
        axis.scatter(*voxels.T, c='#bd6b24', s=9, alpha=.45, label='Obstacle sphere centres (r=0.04 m)')
    for shape in data['source_shapes']:
        if shape['shape'] == 'box':
            corners = np.asarray(shape['corners'])
            for a, b in BOX_EDGES:
                axis.plot(*corners[[a, b]].T, c='#a34c13', linewidth=1.2)
        else:
            theta = np.linspace(0, 2 * np.pi, 49)
            for plane in ((0, 1), (0, 2), (1, 2)):
                points = np.tile(shape['centre'], (len(theta), 1))
                points[:, plane[0]] += shape['radius'] * np.cos(theta)
                points[:, plane[1]] += shape['radius'] * np.sin(theta)
                axis.plot(*points.T, c='#a34c13', linewidth=.7, alpha=.8)
    if data['has_trajectory']:
        axis.plot(*all_tcp.T, c='#008b8b', linewidth=2, label='Saved trajectory TCP trace')
    axis.scatter(*data['frames'][0]['tcp'], c='#16874b', s=55, label='First displayed TCP')
    axis.scatter(*data['frames'][-1]['tcp'], c='#a32e3f', s=55, label='Last displayed TCP')
    bounds = np.asarray(data['view_bounds'])
    axis.set_xlim(*bounds[:, 0]); axis.set_ylim(*bounds[:, 1]); axis.set_zlim(*bounds[:, 2])
    axis.set_box_aspect(bounds[1] - bounds[0])
    axis.set_xlabel('World x [m]'); axis.set_ylabel('World y [m]'); axis.set_zlabel('World z [m]')
    axis.view_init(elev=24, azim=-52)
    axis.set_title(f"{data['case_id']} | {data['status']}\nFull robot collision boxes; fixed world in grey")
    axis.legend(loc='upper left', fontsize=8)
    figure.savefig(out / 'scene.png', dpi=160)
    plt.close(figure)

    figure, axes = plt.subplots(2 if clearance is not None else 1, 1, figsize=(11, 7), sharex=True, squeeze=False, constrained_layout=True)
    first = axes[0, 0]
    if data['has_trajectory']:
        first.plot(time, q)
    else:
        first.scatter(np.repeat(time, 7), q.ravel(), s=25)
    first.set_ylabel('Joint position [rad]')
    first.set_title(f"{data['status']} — " + ('saved numerical trajectory samples' if data['has_trajectory'] else 'fixed endpoints only; no trajectory produced'))
    if data['has_trajectory']:
        first.legend([f'q{i + 1}' for i in range(7)], ncol=7, fontsize=8)
    first.grid(alpha=.2)
    if clearance is not None:
        axes[1, 0].plot(time, clearance * 1000, c='#7a456d')
        axes[1, 0].axhline(0, c='black', linewidth=.8)
        axes[1, 0].set_ylabel('Sampled clearance [mm]')
        axes[1, 0].grid(alpha=.2)
    axes[-1, 0].set_xlabel(data['time_axis'])
    figure.savefig(out / 'joints.png', dpi=160)
    plt.close(figure)


_HTML = r'''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Kinova motion benchmark replay</title><style>
:root{font-family:system-ui,sans-serif;color:#142037;background:#f3f5f8}body{max-width:1150px;margin:30px auto;padding:0 20px}h1{font-size:24px;margin-bottom:4px}.card{background:white;border:1px solid #d6deea;border-radius:10px;padding:20px;margin:18px 0}.status{font-weight:650;color:#a12c3c}.success{color:#16834c}canvas{width:100%;height:540px;background:#fcfdff;border:1px solid #e7ecf5}input[type=range]{width:68%;vertical-align:middle}button{padding:7px 15px;border:1px solid #bdc9dc;background:#eef3fb;border-radius:5px;margin-right:8px}.note{font-size:13px;color:#4c5d72;line-height:1.5}.row{display:flex;flex-wrap:wrap;gap:18px;align-items:center}pre{white-space:pre-wrap;font-size:12px}img{max-width:100%}</style>
<h1>Kinova 7-DoF · saved experiment replay</h1><div id="subtitle"></div><div class="card"><div class="row"><button id="play">Play</button><button id="focus">Full scene</button><input id="slider" type="range" min="0" value="0"><span id="stamp"></span></div><canvas id="view"></canvas><div class="row"><label>Azimuth <input id="yaw" type="range" min="-180" max="180" value="-52"></label><label>Elevation <input id="pitch" type="range" min="0" max="85" value="24"></label></div><p class="note">Blue: all moving robot collision boxes. Grey: fixed base, table and devices. Dark amber outlines: original geometric obstacles. Pale amber: conservative collision spheres, radius 0.04 m. Teal: saved TCP trace. Coordinates are world metres; joint positions are radians. Finite collision sampling is not a continuous-safety certificate.</p><pre id="state"></pre></div><div class="card"><h2>Saved numerical samples</h2><img src="joints.png" alt="Joint positions and sampled clearance"><p id="provenance" class="note"></p></div>
<script id="data" type="application/json">__DATA__</script><script>
const d=JSON.parse(document.getElementById('data').textContent),canvas=document.getElementById('view'),ctx=canvas.getContext('2d'),slider=document.getElementById('slider'),yaw=document.getElementById('yaw'),pitch=document.getElementById('pitch');slider.max=d.frames.length-1;const edges=__EDGES__;let timer=null,focusRobot=true;
document.getElementById('subtitle').textContent=d.case_id+' · '+d.method+' · '+d.objective+' · '+d.status;document.getElementById('subtitle').className=d.status==='success_validated_numerical'?'success':'status';document.getElementById('provenance').textContent=d.sample_source_note+' Source: '+(d.sample_source||'fixed input endpoints')+'. '+d.validation_note;
function draw(){const ratio=window.devicePixelRatio||1;canvas.width=canvas.clientWidth*ratio;canvas.height=canvas.clientHeight*ratio;ctx.setTransform(ratio,0,0,ratio,0,0);let w=canvas.clientWidth,h=canvas.clientHeight;ctx.clearRect(0,0,w,h);const bounds=focusRobot?d.robot_view_bounds:d.view_bounds,frame=d.frames[+slider.value],a=+yaw.value*Math.PI/180,e=+pitch.value*Math.PI/180,c=bounds[0].map((v,i)=>(v+bounds[1][i])/2),scale=Math.min(w,h)*.76/Math.max(...bounds[1].map((v,i)=>v-bounds[0][i]));function project(p){let x=p[0]-c[0],y=p[1]-c[1],z=p[2]-c[2],u=Math.cos(a)*x-Math.sin(a)*y,t=Math.sin(a)*x+Math.cos(a)*y;return[w/2+scale*u,h/2-scale*(Math.cos(e)*z-Math.sin(e)*t),Math.cos(e)*t+Math.sin(e)*z]}function line(p,q,col,width=1){let x=project(p),y=project(q);ctx.beginPath();ctx.moveTo(x[0],x[1]);ctx.lineTo(y[0],y[1]);ctx.strokeStyle=col;ctx.lineWidth=width;ctx.stroke()}for(let i=0;i<3;i++){let p=[0,0,0],q=[0,0,0];q[i]=.3;line(p,q,['#d34c49','#329654','#3b65c7'][i],2);let v=project(q);ctx.fillStyle='#20304a';ctx.fillText(['x','y','z'][i]+' [m]',v[0]+5,v[1])}for(const b of d.fixed_boxes)for(const edge of edges)line(b[edge[0]],b[edge[1]],'#8a94a4');for(const shape of d.source_shapes){if(shape.shape==='box'){for(const edge of edges)line(shape.corners[edge[0]],shape.corners[edge[1]],'#a34c13',1.5)}else{let v=project(shape.centre);ctx.beginPath();ctx.arc(v[0],v[1],shape.radius*scale,0,Math.PI*2);ctx.strokeStyle='#a34c13';ctx.lineWidth=1.2;ctx.stroke()}}let spheres=d.voxels_xyz.map(p=>({p,v:project(p)})).sort((a,b)=>a.v[2]-b.v[2]);for(const o of spheres){ctx.beginPath();ctx.arc(o.v[0],o.v[1],Math.max(1,d.voxel_radius_m*scale),0,2*Math.PI);ctx.fillStyle='rgba(186,108,35,.12)';ctx.fill();ctx.strokeStyle='rgba(164,85,24,.27)';ctx.lineWidth=.5;ctx.stroke()}if(d.has_trajectory)for(let i=1;i<d.frames.length;i++)line(d.frames[i-1].tcp,d.frames[i].tcp,'#2c9691',1.5);for(const b of frame.moving_boxes)for(const edge of edges)line(b[edge[0]],b[edge[1]],'#1e5ba5',1.5);let tip=project(frame.tcp);ctx.beginPath();ctx.arc(tip[0],tip[1],5,0,Math.PI*2);ctx.fillStyle='#b2344b';ctx.fill();document.getElementById('stamp').textContent=(+slider.value+1)+' / '+d.frames.length+' · '+Number(frame.time).toFixed(4)+' '+d.time_axis;document.getElementById('state').textContent='q = ['+frame.q.map(v=>v.toFixed(5)).join(', ')+'] rad\n'+(frame.clearance_m==null?'':'Saved sampled clearance = '+(1000*frame.clearance_m).toFixed(5)+' mm')+(d.has_trajectory?'':'\n'+d.sample_source_note);}
document.getElementById('focus').onclick=()=>{focusRobot=!focusRobot;document.getElementById('focus').textContent=focusRobot?'Full scene':'Focus robot';draw()};slider.oninput=draw;yaw.oninput=draw;pitch.oninput=draw;window.onresize=draw;document.getElementById('play').onclick=()=>{if(timer){clearInterval(timer);timer=null;document.getElementById('play').textContent='Play'}else{timer=setInterval(()=>{slider.value=(+slider.value+1)%d.frames.length;draw()},80);document.getElementById('play').textContent='Pause'}};draw();
</script></html>'''


def replay(attempt_dir, output=None):
    from ..model import Model

    attempt = Path(attempt_dir).resolve()
    evidence_integrity = {'verified': False, 'scope': 'standalone saved samples; not a sealed benchmark run'}
    for parent in attempt.parents:
        if (parent / 'run_manifest.json').exists():
            from ..batch import verify
            check = verify(parent)
            if not check['integrity_passed']:
                raise ValueError('Refusing to replay corrupted benchmark evidence: ' + json.dumps(check))
            evidence_integrity = {'verified': True, 'scope': 'sealed run receipts', 'run': str(parent)}
            break
    case, result, process = _inputs(attempt)
    q, time, clearance, validation, sample_path, has_trajectory = _trajectory(attempt, case)
    model = Model(case.get('voxels_xyz', []), case.get('voxel_radius_m', .04))
    if case.get('robot_sha256') and case['robot_sha256'] != model.robot_sha256:
        raise ValueError('Saved case robot hash differs from the current canonical model.')
    if case.get('collision_geometry_sha256') and case['collision_geometry_sha256'] != model.collision_geometry_sha256:
        raise ValueError('Saved case collision-model hash differs from the current canonical model.')
    indices = np.unique(np.linspace(0, len(q) - 1, min(180, len(q))).astype(int))
    moving = np.asarray(model.robot_geometry_ids, int)
    fixed = np.asarray([i for i in range(model.mplant.num_scene_geometries) if i not in set(moving)], int)
    frames = []
    all_tcp = model.tcp(q)
    fixed_boxes = None
    for index in indices:
        boxes = _world_boxes(model, q[index])
        if fixed_boxes is None:
            fixed_boxes = boxes[fixed]
        frames.append(dict(sample_index=int(index), time=float(time[index]), q=q[index].tolist(),
                           tcp=all_tcp[index].tolist(), moving_boxes=boxes[moving].round(7).tolist(),
                           clearance_m=float(clearance[index]) if clearance is not None else None))
    # Include the complete fixed scene; no world box may escape the axes.
    source_shapes = []
    for obstacle in case.get('obstacle_geometry', []) or []:
        centre = np.asarray(obstacle['center_m'])
        if obstacle['shape'] == 'sphere':
            source_shapes.append(dict(shape='sphere', centre=centre.tolist(), radius=float(obstacle['radius_m'])))
        elif obstacle['shape'] == 'box':
            yaw = float(obstacle.get('yaw_rad', 0.))
            c, s = np.cos(yaw), np.sin(yaw)
            rotation = np.asarray([[c, -s, 0], [s, c, 0], [0, 0, 1]])
            corners = (SIGNS * np.asarray(obstacle['size_m']) / 2) @ rotation.T + centre
            source_shapes.append(dict(shape='box', corners=corners.tolist()))
    endpoint_count = sum(np.asarray(case.get(k)).shape == (7,) and np.isfinite(np.asarray(case[k], float)).all() for k in ('q_start', 'q_goal'))
    no_trajectory_note = ('No trajectory was saved. Only valid original fixed endpoint states are displayed; no connecting path is fabricated.' if endpoint_count else 'No valid endpoints or trajectory were saved. The canonical zero-joint pose is shown only to locate the robot; it is not a planned state.')
    visible = np.vstack([fixed_boxes.reshape(-1, 3)] +
                        [np.asarray(f['moving_boxes']).reshape(-1, 3) for f in frames] +
                        ([model.voxels_xyz] if len(model.voxels_xyz) else []))
    robot_visible = np.vstack([np.asarray(f['moving_boxes']).reshape(-1, 3) for f in frames] + ([model.voxels_xyz] if len(model.voxels_xyz) else []))
    robot_low, robot_high = robot_visible.min(axis=0) - .12, robot_visible.max(axis=0) + .12
    low, high = visible.min(axis=0) - .12, visible.max(axis=0) + .12
    low[2] = min(0., low[2])
    data = dict(schema_version=1, case_id=case.get('problem_id', case.get('case_id', 'unknown')),
                evidence_integrity=evidence_integrity,
                scene_id=case.get('scene_id'), family=case.get('family'),
                method=result.get('method', 'unknown'), objective=result.get('objective', 'unknown'),
                status=result.get('status', process.get('status', 'unfinished')),
                has_trajectory=has_trajectory, time_axis=validation.get('time_axis', 'endpoint index; not a trajectory'),
                original_sample_count=len(q), displayed_frames=len(frames), frames=frames,
                fixed_boxes=fixed_boxes.round(7).tolist(), robot_geometry_names=[model.inventory['geometries'][i]['name'] for i in moving],
                voxels_xyz=model.voxels_xyz.tolist(), voxel_radius_m=model.voxel_radius,
                obstacle_geometry=case.get('obstacle_geometry'), source_shapes=source_shapes,
                view_bounds=[low.tolist(), high.tolist()], robot_view_bounds=[robot_low.tolist(), robot_high.tolist()],
                robot_sha256=model.robot_sha256, collision_geometry_sha256=model.collision_geometry_sha256,
                sample_source=str(sample_path.relative_to(attempt)) if sample_path else None,
                sample_source_sha256=hashlib.sha256(sample_path.read_bytes()).hexdigest() if sample_path else None,
                sample_source_note='Rendered frames are a deterministic subsample of saved numerical validator samples; no planning or CUDA recomputation.' if has_trajectory else no_trajectory_note,
                validation_note=validation.get('check', 'No complete numerical trajectory validation was saved.'))
    # Replays are derived artifacts, never appended to a sealed attempt.
    out = Path(output) if output else attempt.parent / (attempt.name + '_replay')
    if out.resolve().is_relative_to(attempt.resolve()):
        raise ValueError('Replay output must be outside the sealed attempt directory.')
    out.mkdir(parents=True, exist_ok=True)
    _plot(out, data, q, time, clearance, all_tcp)
    serialized = json.dumps(data, separators=(',', ':'), allow_nan=False)
    (out / 'replay.json').write_text(json.dumps(data, indent=2, allow_nan=False) + '\n')
    (out / 'replay.html').write_text(_HTML.replace('__DATA__', serialized.replace('<', '\\u003c'))
                                   .replace('__EDGES__', json.dumps(BOX_EDGES)))
    return dict(status='created', attempt_dir=str(attempt), output_dir=str(out),
                trajectory_status=data['status'], has_trajectory=has_trajectory,
                original_sample_count=len(q), displayed_frames=len(frames),
                html=str(out / 'replay.html'), scene_png=str(out / 'scene.png'),
                joints_png=str(out / 'joints.png'), data_json=str(out / 'replay.json'))
