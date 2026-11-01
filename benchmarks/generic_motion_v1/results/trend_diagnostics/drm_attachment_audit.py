"""Developer-only DRM failure attribution. No benchmark input/result is edited."""
from __future__ import annotations
import argparse
import collections
import fcntl
import json
from pathlib import Path
import sys
import time
import numpy as np

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT))
from benchmarks.generic_motion_v1.common import BASE, cases, sha
from benchmarks.generic_motion_v1.roadmap import load_config, write


def components(nodes, adjacency, blocked, edges):
    labels = {}
    for node in sorted(set(nodes) - blocked):
        if node in labels:
            continue
        label = len(set(labels.values()))
        labels[node] = label
        stack = [node]
        while stack:
            current = stack.pop()
            for nxt in adjacency.get(current, []):
                if nxt not in nodes or nxt in blocked or nxt in labels or (current, nxt) in edges:
                    continue
                labels[nxt] = label
                stack.append(nxt)
    return labels


def diagnose(case, output, candidate_limit):
    import csdecomp as csd
    from benchmarks.generic_motion_v1.model import Model
    from benchmarks.generic_motion_v1.runners.native_drm_query import load
    cfg = load_config(ROOT / 'configs/evaluation.json')
    model = Model(case['voxels_xyz'], case['voxel_radius_m'])
    module, receipt = load(ROOT / cfg['drm_query']['build_dir'])
    opts = csd.DrmPlannerOptions()
    for key, value in cfg['drm'].items():
        setattr(opts, key, value)
    planner = csd.DrmPlanner(model.csd_plant, opts, True)
    roadmap = BASE / 'roadmaps/rss2025_n6000_s000/native.map'
    planner.LoadRoadmap(str(roadmap))
    static_blocked = set(planner.collision_set)
    planner.BuildCollisionSet(model.voxels)
    cached = set(planner.collision_set)
    edges = set(tuple(e) for e in planner.forbidden_edge_set)
    nodes = planner.drm.id_to_node_map
    adjacency = planner.drm.node_adjacency_map
    ids = sorted(nodes)
    q = np.asarray([nodes[i] for i in ids])
    started = time.perf_counter()
    free = model.cuda(q)
    node_check_seconds = time.perf_counter() - started
    exact_blocked = {i for i, safe in zip(ids, free) if not safe}
    falsely_blocked = sorted(cached - exact_blocked)
    missed = sorted(exact_blocked - cached)
    # Independently check concrete witnesses; CUDA population counts stay labeled CUDA.
    witness_ids = falsely_blocked[:128]
    clearances = model.clearance(np.asarray([nodes[i] for i in witness_ids])) if witness_ids else []
    row = dict(problem_id=case['problem_id'], scene_id=case['scene_id'], category=case['family'],
               roadmap_sha256=sha(roadmap), build_binary_sha256=receipt['binary_sha256'],
               roadmap_nodes=len(ids), static_blocked=len(static_blocked), cached_blocked=len(cached),
               exact_cuda_blocked=len(exact_blocked), cache_false_positive_cuda=len(falsely_blocked),
               cache_false_negative_cuda=len(missed), exact_cuda_node_check_seconds=node_check_seconds,
               cache_false_positive_independent_witnesses=len(witness_ids),
               cache_false_positive_independently_clear=sum(float(c)>model.contact_band_m for c in clearances),
               cached_edge_exclusions=len(edges), asymmetric_cached_edges=sum((b,a) not in edges for a,b in edges),
               candidate_limit=candidate_limit, steering_radius_m=case['voxel_radius_m'],
               steering_step_rad=opts.online_edge_step_size, candidate_modes={})
    np.savez_compressed(output/'nodes.npz', node_ids=ids, configurations=q, cuda_free=free,
                        cached_blocked=np.asarray([i in cached for i in ids]),
                        independent_witness_ids=witness_ids, independent_clearance=clearances)
    for name, excluded in [('cached', cached), ('exact_nodes_diagnostic', exact_blocked)]:
        labels = components(nodes, adjacency, excluded, edges)
        sizes = collections.Counter(labels.values())
        mode = dict(components=len(sizes), largest_components=sorted(sizes.values(), reverse=True)[:10])
        for endpoint in ('q_start', 'q_goal'):
            target = np.asarray(case[endpoint])
            ordered = sorted(labels, key=lambda i: (float(np.linalg.norm(nodes[i]-target)), i))[:candidate_limit]
            entries=[]
            for rank, node in enumerate(ordered):
                safe = bool(module.steer(planner, target, nodes[node], np.asarray(model.voxels_xyz,np.float32).T, case['voxel_radius_m']))
                entries.append(dict(node_id=node, rank=rank, component=labels[node],
                                    distance_rad=float(np.linalg.norm(nodes[node]-target)), native_steer_safe=safe))
            mode[endpoint] = entries
        a = [x for x in mode['q_start'] if x['native_steer_safe']]
        b = [x for x in mode['q_goal'] if x['native_steer_safe']]
        common = sorted({x['component'] for x in a} & {x['component'] for x in b})
        mode['common_visible_components'] = common
        mode['first_visible_pair_connected'] = bool(a and b and a[0]['component']==b[0]['component'])
        mode['first_start_visible'] = a[0] if a else None
        mode['first_goal_visible'] = b[0] if b else None
        mode['alternate_visible_pair'] = next(([x['node_id'], y['node_id']] for x in a for y in b
                                               if x['component']==y['component']), None)
        row['candidate_modes'][name] = mode
    # Existing OMPL candidates are evidence only; do not call another planner.
    paths = list((ROOT/'results/repaired_dev/jobs').glob(case['problem_id']+'-RRTCONNECT*/attempts/0001/result.json'))
    if paths:
        folder=paths[0].parent; result=json.loads(paths[0].read_text())
        row['rrt_reference'] = dict(result=str(paths[0].relative_to(ROOT)), status=result['status'])
        pathfile=folder/'ompl_simplified_path.npz'
        if pathfile.exists() and result['status']=='success_validated_numerical':
            points=np.load(pathfile)['points']
            safe=[bool(module.steer(planner,a,b,np.asarray(model.voxels_xyz,np.float32).T,case['voxel_radius_m'])) for a,b in zip(points,points[1:])]
            row['rrt_reference'].update(path_sha256=sha(pathfile),native_steer_all_safe=all(safe),segments=len(safe))
    write(output/'record.json', row)
    return row


def rrt_witnesses(output):
    import csdecomp as csd
    from benchmarks.generic_motion_v1.model import Model
    from benchmarks.generic_motion_v1.runners.native_drm_query import load
    module,_=load(BASE/'native/build/drm_consistent_query')
    records=json.loads((output/'records.json').read_text())
    casebyid={c['problem_id']:c for c in cases(BASE/'manifests/dev.jsonl')}
    rows=[]
    for row in records:
        reference=row.get('rrt_reference',{})
        if reference.get('status')!='success_validated_numerical':continue
        case=casebyid[row['problem_id']];result=ROOT/reference['result'];folder=result.parent
        path=folder/'ompl_simplified_path.npz';points=np.load(path)['points']
        with (ROOT/'results/.gpu.lock').open('a+') as lock:
            fcntl.flock(lock,fcntl.LOCK_EX)
            model=Model(case['voxels_xyz'],case['voxel_radius_m'])
            options=csd.DrmPlannerOptions();options.online_edge_step_size=.01
            planner=csd.DrmPlanner(model.csd_plant,options,True)
            safe=[bool(module.steer(planner,a,b,np.asarray(model.voxels_xyz,np.float32).T,.04)) for a,b in zip(points,points[1:])]
        rows.append(dict(problem_id=case['problem_id'],scene_id=case['scene_id'],source_result=str(result.relative_to(ROOT)),
            path_sha256=sha(path),saved_status=reference['status'],native_step_rad=.01,online_radius_m=.04,
            native_steer_all_safe=all(safe),segments=len(safe),model_sha256=model.robot_sha256,
            geometry_sha256=model.collision_geometry_sha256))
    write(output/'rrt_witnesses.json',dict(scope='Existing numerically validated RRT paths checked only as feasibility/collision-model witnesses; never substituted for DRM output',rows=rows))
    print(json.dumps(dict(witnesses=len(rows),native_steer_passed=sum(r['native_steer_all_safe'] for r in rows))),flush=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--problem',action='append')
    parser.add_argument('--candidate-limit',type=int,default=50)
    parser.add_argument('--witnesses-only',action='store_true')
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    if args.witnesses_only:
        rrt_witnesses(args.output);return
    failures=json.loads((ROOT/'results/trend_diagnostics/frontend_native_limits/records.json').read_text())
    selected={x['problem_id'] for x in failures if x['status']!='success_numerical'}
    if args.problem:
        selected &= set(args.problem)
    rows=[]
    for case in cases(BASE/'manifests/dev.jsonl'):
        if case['problem_id'] not in selected:
            continue
        folder=args.output/case['problem_id']
        if (folder/'record.json').exists():
            rows.append(json.loads((folder/'record.json').read_text()));continue
        folder.mkdir(exist_ok=True)
        # Shared lock is released after each task so other diagnostic jobs can run.
        with (ROOT/'results/.gpu.lock').open('a+') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            started=time.perf_counter()
            row=diagnose(case,folder,args.candidate_limit)
            row['diagnostic_seconds']=time.perf_counter()-started
            write(folder/'record.json',row)
        rows.append(row)
        print(json.dumps({k:row[k] for k in ('problem_id','cache_false_positive_cuda','cache_false_negative_cuda','diagnostic_seconds')},sort_keys=True),flush=True)
        write(args.output/'records.json',rows)
    write(args.output/'summary.json',dict(scope='Dev-only diagnosis, not a planner score; exact-node population uses unchanged canonical CUDA; independent witnesses are separately reported',
        rows=len(rows),requested=len(selected),complete=len(rows)==len(selected),
        nearest_pair_disconnected_but_alternate_visible=sum(not r['candidate_modes']['cached']['first_visible_pair_connected'] and bool(r['candidate_modes']['cached']['alternate_visible_pair']) for r in rows),
        edge_cache_in_use=any(r['cached_edge_exclusions'] for r in rows),records='records.json'))


if __name__=='__main__':
    main()
