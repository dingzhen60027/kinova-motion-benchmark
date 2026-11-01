"""Serial dev-only paired native DRM regression with unchanged final validator."""
from __future__ import annotations
import argparse
import collections
import contextlib
import fcntl
import json
from pathlib import Path
import sys
import time
import traceback
import numpy as np
ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT))
from benchmarks.generic_motion_v1.common import BASE, cases, sha
from benchmarks.generic_motion_v1.roadmap import load_config, write
from benchmarks.generic_motion_v1.runners.native_drm_query import load, query


def run_case(case,folder,build,cfg):
    import csdecomp as csd
    from benchmarks.generic_motion_v1.model import Model
    from benchmarks.generic_motion_v1.runners.csdecomp_baseline import _validate_frontend_path, _checked_native_path
    began=time.perf_counter()
    model=Model(case['voxels_xyz'],case['voxel_radius_m'])
    opts=csd.DrmPlannerOptions()
    for name,value in cfg['drm'].items():setattr(opts,name,value)
    planner=csd.DrmPlanner(model.csd_plant,opts,True)
    roadmap=BASE/'roadmaps/rss2025_n6000_s000/native.map'
    planner.LoadRoadmap(str(roadmap));planner.BuildCollisionSet(model.voxels)
    before=set(planner.collision_set)
    row=dict(problem_id=case['problem_id'],scene_id=case['scene_id'],category=case['family'],
        status='running',cached_blocked_nodes=len(before),roadmap_nodes=len(planner.drm.id_to_node_map),
        setup_seconds=time.perf_counter()-began,drm_options=cfg['drm'])
    started=time.perf_counter()
    try:
        success,path,evidence=query(planner,case['q_start'],case['q_goal'],model.voxels_xyz,case['voxel_radius_m'],build_dir=build,out=folder)
        row['query_seconds']=evidence['elapsed_seconds'];row['native_success']=success
        if success:
            path=_checked_native_path(path,np.asarray([case['q_start'],case['q_goal']]),cfg['endpoint_tolerance_rad'])
            np.save(folder/'path.npy',path)
            valstart=time.perf_counter()
            validation=_validate_frontend_path(path,model,case,cfg,folder/'validation')
            row.update(validation_seconds=time.perf_counter()-valstart,
                       status='success_numerical' if validation['passed'] else 'validation_failed',
                       validation_status=validation['status'],validation=validation)
        else:row['status']='no_path'
    except Exception as error:
        row.update(status='native_exception',error=repr(error))
        (folder/'exception.txt').write_text(traceback.format_exc())
    row['query_and_validation_seconds']=time.perf_counter()-started
    row['diagnostic_cold_seconds']=time.perf_counter()-began
    row['remaining_excluded_nodes']=len(planner.collision_set)
    row['native_rejected_edges']=len(planner.forbidden_edge_set)
    if row['status']!='success_numerical':
        all_nodes=planner.drm.id_to_node_map
        all_ids=sorted(all_nodes)
        exact_free=model.cuda(np.asarray([all_nodes[i] for i in all_ids]))
        heuristic_ids=[i for i,free in zip(all_ids,exact_free) if free and i in planner.collision_set]
        forbidden=set(tuple(edge) for edge in planner.forbidden_edge_set)
        edges={(min(i,j),max(i,j)) for i,neighbors in planner.drm.node_adjacency_map.items() for j in neighbors}
        row['diagnostic_heuristic_free_node_exclusions']=len(heuristic_ids)
        row['diagnostic_rejected_internal_undirected_edges']=len({(min(a,b),max(a,b)) for a,b in forbidden if a>=0 and b>=0})
        row['diagnostic_total_undirected_edges']=len(edges)
        safe_neighbor_witnesses=[]
        module,_=load(build)
        for node in heuristic_ids:
            for neighbor in planner.drm.node_adjacency_map.get(node,[]):
                if neighbor in planner.collision_set or (node,neighbor) in forbidden:continue
                if module.steer(planner,all_nodes[node],all_nodes[neighbor],np.asarray(model.voxels_xyz,np.float32).T,
                                case['voxel_radius_m']+opts.voxel_padding):
                    safe_neighbor_witnesses.append([node,neighbor]);break
        row['diagnostic_heuristic_nodes_with_free_unblocked_neighbor']=safe_neighbor_witnesses
        np.savez_compressed(folder/'query_graph_state.npz',excluded_node_ids=sorted(planner.collision_set),
                            heuristic_free_node_ids=heuristic_ids,forbidden_edges=sorted(forbidden))
        row['post_query_diagnostic_seconds']=time.perf_counter()-began-row['diagnostic_cold_seconds']
    write(folder/'record.json',row)
    return row


def fixture(out,build):
    import csdecomp as csd
    from benchmarks.generic_motion_v1.model import Model
    model=Model();module,receipt=load(build)
    options=csd.DrmPlannerOptions()
    options.online_edge_step_size=.01
    planner=csd.DrmPlanner(model.csd_plant,options,True)
    planner.LoadRoadmap(str(BASE/'roadmaps/rss2025_n6000_s000/native.map'))
    q=np.asarray(cases(BASE/'manifests/dev.jsonl')[0]['q_start'],np.float32)
    direction=np.zeros(7,np.float32);direction[0]=.001
    nodes={0:q,1:q+direction,2:q+2*direction}
    assert model.cuda(np.asarray(list(nodes.values()))).all()
    planner.drm.id_to_node_map=nodes
    planner.drm.node_adjacency_map={0:[],1:[2],2:[1]}
    # Empty cache; nearest nodes 0 and 2 disconnect, alternate node 1 connects.
    empty=np.zeros((3,0),np.float32)
    checks={}
    try:
        ok,path=module.plan(planner,nodes[0],nodes[2],empty,.04)
        checks['alternate_attachment_success']=bool(ok)
        if ok:np.save(out/'alternate_attachment_path.npy',np.asarray(path))
    except Exception as error:checks['alternate_attachment_success']=False;checks['alternate_error']=repr(error)
    # One genuinely distant online sphere, but a deliberately overinclusive
    # quantized cache entry excludes every free graph node.
    sphere=np.array([[.7],[.7],[.7]],np.float32)
    sphere_id=csd.GetCollisionVoxelId(sphere[:,0],planner.drm.options)
    assert sphere_id>=0
    witness=Model(sphere.T,.04)
    assert witness.cuda(np.asarray(list(nodes.values()))).all()
    assert np.all(witness.clearance(np.asarray(list(nodes.values())))>witness.contact_band_m)
    planner.drm.collision_map={sphere_id:[0,1,2]}
    planner.BuildCollisionSet(csd.Voxels(sphere))
    checks['fixture_cache_excluded_all']=len(planner.collision_set)==3
    try:
        ok,path=module.plan(planner,nodes[0],nodes[2],sphere,.04)
        checks['cache_false_positive_reconciled']=bool(ok) and not planner.collision_set
    except Exception as error:checks['cache_false_positive_reconciled']=False;checks['cache_error']=repr(error)
    # Negative: move sphere onto robot, omit it from cache. Exact node update
    # must still reject collisions; no successful path can bypass validation.
    collision_sphere=None
    for geom in witness.inventory['geometries']:
        if geom['moving_robot']:
            geometry_id=witness._drake_ids[geom['index']]
            witness.plant.SetPositions(witness.plant_context,q)
            center=witness._query().GetPoseInWorld(geometry_id).translation()
            probe=Model([center],.04)
            if not probe.cuda(q)[0]:collision_sphere=np.asarray(center,np.float32).reshape(3,1);break
    assert collision_sphere is not None
    planner.drm.collision_map={}
    planner.BuildCollisionSet(csd.Voxels(collision_sphere))
    checks['fixture_cache_missed_collision']=not planner.collision_set
    try:
        ok,_=module.plan(planner,q,q+2*direction,collision_sphere,.04)
        checks['uncached_collision_rejected']=not ok
    except RuntimeError:checks['uncached_collision_rejected']=True
    checks['uncached_collision_nodes_excluded']=bool(planner.collision_set)
    result=dict(profile=receipt['profile'],checks=checks,
                all_checks_passed=all(x for x in checks.values() if isinstance(x,bool)),
                scope='Real canonical CUDA/Drake collision tests on deliberately tiny regression graphs, not benchmark tasks')
    write(out/'fixture.json',result);print(json.dumps(result),flush=True)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',required=True,type=Path)
    p.add_argument('--build',required=True,type=Path)
    p.add_argument('--config',type=Path,default=ROOT/'configs/evaluation.json')
    p.add_argument('--native-defaults',action='store_true')
    p.add_argument('--fixture',action='store_true')
    p.add_argument('--attempts',type=int)
    p.add_argument('--endpoint-trials',type=int)
    p.add_argument('--problem',action='append')
    p.add_argument('--external-gpu-lock',action='store_true',help='Only for invocation under flock results/.gpu.lock timeout ...')
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    if a.fixture:
        with (ROOT/'results/.gpu.lock').open('a+') as lock:
            fcntl.flock(lock,fcntl.LOCK_EX);fixture(a.output,a.build)
        return
    cfg=load_config(a.config)
    if a.native_defaults:
        cfg['drm'].update(max_number_planning_attempts=100,max_iterations_steering_to_node=50,
                          max_nodes_to_expand=100000,online_edge_step_size=.01)
    if a.attempts is not None:cfg['drm']['max_number_planning_attempts']=a.attempts
    if a.endpoint_trials is not None:cfg['drm']['max_iterations_steering_to_node']=a.endpoint_trials
    cfg['drm_query']['build_dir']=str(a.build)
    write(a.output/'evaluation.json',cfg)
    rows=[]
    for case in cases(BASE/'manifests/dev.jsonl'):
        if a.problem and case['problem_id'] not in a.problem:continue
        folder=a.output/case['problem_id']
        if (folder/'record.json').exists():
            rows.append(json.loads((folder/'record.json').read_text()));continue
        folder.mkdir(exist_ok=True)
        with (contextlib.nullcontext(None) if a.external_gpu_lock else (ROOT/'results/.gpu.lock').open('a+')) as lock:
            if lock is not None:fcntl.flock(lock,fcntl.LOCK_EX)
            row=run_case(case,folder,a.build,cfg)
        rows.append(row);write(a.output/'records.json',rows)
        print(json.dumps({k:row[k] for k in ('problem_id','status','query_and_validation_seconds')}),flush=True)
    write(a.output/'summary.json',dict(scope='Frontend-only paired dev diagnosis; unchanged canonical model/graph/endpoints/final independent validator; exact node checks count inside online query',
        rows=len(rows),status_counts=dict(collections.Counter(r['status'] for r in rows)),
        manifest_sha256=sha(BASE/'manifests/dev.jsonl'),roadmap_sha256=sha(BASE/'roadmaps/rss2025_n6000_s000/native.map'),
        config=cfg,build=str(a.build),records='records.json'))


if __name__=='__main__':main()
