"""Native GPU regressions and whole-dev frontend diagnosis, separate from scores."""
from __future__ import annotations
import argparse
import collections
import json
from pathlib import Path
import time
import traceback
import numpy as np
from ..common import BASE, ROOT, cases, sha
from ..roadmap import load_config, write
from .native_drm_query import load, query


def regression(build, out):
    import csdecomp as csd
    from ..model import Model
    model=Model()
    module,receipt=load(build)
    planner=csd.DrmPlanner(model.csd_plant,csd.DrmPlannerOptions(),True)
    planner.LoadRoadmap(str(BASE/'roadmaps/rss2025_n6000_s000/native.map'))
    case=next(c for c in cases(BASE/'manifests/dev.jsonl') if c['validation_status']=='valid_input')
    q=np.asarray(case['q_start'],np.float32)
    assert model.cuda(q).all() and model.clearance(q)[0]>model.contact_band_m
    planner.drm.id_to_node_map={0:q}
    planner.drm.node_adjacency_map={0:[]}
    checks={}
    try:
        success,path=module.plan(planner,q,q,np.zeros((3,0),np.float32),.04)
        checks['single_isolated_vertex_same_endpoint_success']=bool(success)
        np.save(out/'single_vertex_path.npy',np.asarray(path))
    except Exception as exc:
        checks['single_isolated_vertex_same_endpoint_success']=False
        checks['same_endpoint_exception']=repr(exc)
    if hasattr(module,'path_exists'):
        checks['same_vertex_reachable']=bool(module.path_exists(planner,0,0))
        checks['missing_vertex_rejected']=not module.path_exists(planner,0,1)
        checks['valid_zero_edge']=bool(module.steer(planner,q,q,np.zeros((3,0),np.float32),.04))
        rng=np.random.Generator(np.random.PCG64(20261101))
        samples=rng.uniform(model.lower+.02,model.upper-.02,(1024,7))
        colliding=samples[~model.cuda(samples)]
        assert len(colliding)
        bad=colliding[0]
        checks['colliding_zero_edge_rejected']=not module.steer(planner,bad,bad,np.zeros((3,0),np.float32),.04)
        # Bisect a real collision boundary, then keep a short edge whose start
        # is free and endpoint collides. It is shorter than the native step.
        free=q.astype(float); blocked=bad.copy()
        for _ in range(18):
            mid=(free+blocked)/2
            if model.cuda(mid)[0]: free=mid
            else: blocked=mid
        checks['short_collision_endpoint_rejected']=not module.steer(planner,free,blocked,np.zeros((3,0),np.float32),.04)
        checks['nonfinite_edge_rejected']=not module.steer(planner,q,np.full(7,np.nan),np.zeros((3,0),np.float32),.04)
        blocked_goal=None
        for goal in samples[model.cuda(samples)][:128]:
            count=int(np.ceil(np.linalg.norm(goal-q)/.005))+1
            segment=np.linspace(q,goal,count)
            if not model.cuda(segment).all():
                blocked_goal=goal;break
        assert blocked_goal is not None
        try: module.plan(planner,q,blocked_goal,np.zeros((3,0),np.float32),.04)
        except RuntimeError: pass
        checks['failed_endpoint_keeps_valid_vertex']=0 not in planner.collision_set
        checks['endpoint_rejection_is_tagged']=any(-2 in edge for edge in planner.forbidden_edge_set)
        retried,_=module.plan(planner,q,q,np.zeros((3,0),np.float32),.04)
        checks['new_query_clears_old_endpoint_rejection']=bool(retried)
        np.savez_compressed(out/'boundary_fixture.npz',free=free,colliding=blocked,zero_valid=q,zero_collision=bad)
    result=dict(profile=receipt.get('profile','failure_exits'),checks=checks,
                all_expected_positive_negative_checks_passed=all(v for v in checks.values() if isinstance(v,bool)),
                binary_sha256=receipt['binary_sha256'],scope='Real canonical-model CUDA queries; synthetic one-vertex graph is a regression fixture, not a benchmark task')
    write(out/'regression.json',result)
    return result


def frontend(build,out,seed):
    import csdecomp as csd
    from ..model import Model
    from .csdecomp_baseline import _validate_frontend_path, _checked_native_path
    cfg=load_config(ROOT/'configs/evaluation.json')
    manifest=BASE/'manifests/dev.jsonl'
    roadmap=BASE/f'roadmaps/rss2025_n6000_s{seed:03d}/native.map'
    records=[]
    for case in cases(manifest):
        folder=out/case['problem_id'];folder.mkdir()
        model=Model(case['voxels_xyz'],case['voxel_radius_m'])
        options=csd.DrmPlannerOptions()
        for name,value in cfg['drm'].items():setattr(options,name,value)
        planner=csd.DrmPlanner(model.csd_plant,options,True)
        planner.LoadRoadmap(str(roadmap));planner.BuildCollisionSet(model.voxels)
        before=set(planner.collision_set)
        row=dict(problem_id=case['problem_id'],scene_id=case['scene_id'],category=case['family'],
                 seed=seed,model_sha256=model.robot_sha256,geometry_sha256=model.collision_geometry_sha256,
                 status='running',cached_blocked_nodes=len(before),roadmap_nodes=len(planner.drm.id_to_node_map))
        started=time.perf_counter()
        try:
            ok,path,evidence=query(planner,case['q_start'],case['q_goal'],model.voxels_xyz,.04,
                                   build_dir=build,out=folder)
            row['native_success']=ok
            if ok:
                path=_checked_native_path(path,np.asarray([case['q_start'],case['q_goal']]),cfg['endpoint_tolerance_rad'])
                np.save(folder/'path.npy',path)
                validation=_validate_frontend_path(path,model,case,cfg,folder/'validation')
                row['status']='success_numerical' if validation['passed'] else 'validation_failed'
                row['validation_status']=validation['status']
            else:row['status']='no_path'
        except Exception as exc:
            row.update(status='native_exception',error=repr(exc))
            (folder/'exception.txt').write_text(traceback.format_exc())
        row['query_and_validation_seconds']=time.perf_counter()-started
        removed=sorted(set(planner.collision_set)-before)
        if removed:
            nodes=np.asarray([planner.drm.id_to_node_map[i] for i in removed])
            free=model.cuda(nodes)&(model.clearance(nodes)>model.contact_band_m)
            row['newly_excluded_valid_vertices']=int(free.sum())
            np.savez_compressed(folder/'excluded_nodes.npz',node_ids=removed,configurations=nodes,independently_free=free)
        else:row['newly_excluded_valid_vertices']=0
        row['rejected_endpoint_edges']=sum(any(n<0 for n in edge) for edge in planner.forbidden_edge_set)
        write(folder/'record.json',row);records.append(row)
        write(out/'records.json',records)
    result=dict(scope='Frontend diagnosis only; same native graph/model/options; not an end-to-end planner score',
                manifest_sha256=sha(manifest),roadmap_sha256=sha(roadmap),config=cfg,
                rows=len(records),status_counts=dict(collections.Counter(r['status'] for r in records)),
                records='records.json',excluded_valid_vertices=sum(r['newly_excluded_valid_vertices'] for r in records))
    write(out/'summary.json',result)
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--kind',choices=['regression','frontend'],required=True)
    p.add_argument('--build-dir',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--seed',type=int,default=0)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    result=regression(a.build_dir,a.output) if a.kind=='regression' else frontend(a.build_dir,a.output,a.seed)
    print(json.dumps(result,indent=2))

if __name__=='__main__':main()
