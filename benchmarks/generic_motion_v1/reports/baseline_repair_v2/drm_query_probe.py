"""Focused native/safe DRM comparison; only fixed dev/smoke inputs allowed."""
import argparse
from pathlib import Path
import json
import resource
import time
import numpy as np

from benchmarks.generic_motion_v1.common import BASE, cases, read, sha, write
from benchmarks.generic_motion_v1.model import Model
from benchmarks.generic_motion_v1.roadmap import load_config
from benchmarks.generic_motion_v1.runners.native_drm_query import load, layout_guard, query


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--case',required=True)
    p.add_argument('--seed',type=int,required=True)
    p.add_argument('--mode',choices=['native','safe'],required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--build-dir',type=Path,required=True)
    args=p.parse_args()
    split=args.case.split('-')[0]
    if split not in ('dev','smoke'):
        raise ValueError('This repair probe is restricted to dev and smoke')
    if args.output.exists():raise FileExistsError(args.output)
    args.output.mkdir(parents=True)
    resource.setrlimit(resource.RLIMIT_CORE,(0,0))
    data=cases(BASE/f'manifests/{split}.jsonl')
    case=next(c for c in data if c['problem_id']==args.case)
    if case['validation_status']!='valid_input':raise ValueError('Probe expects a known legal development input')
    import csdecomp as csd
    cfg=load_config(BASE/'configs/rss2025_eizo.json')
    road=BASE/f'roadmaps/rss2025_n6000_s{args.seed:03d}'
    metadata=read(road/'metadata.json')
    record=dict(mode=args.mode,case_id=args.case,case_sha256=case['problem_sha256'],
        roadmap_id=metadata['roadmap_id'],roadmap_sha256=sha(road/'native.map'),
        q_start=case['q_start'],q_goal=case['q_goal'],config=cfg['drm'],
        online_obstacle_radius_m=case['voxel_radius_m'],same_fixed_endpoints=True,
        robot_sha256=case['robot_sha256'],collision_geometry_sha256=case['collision_geometry_sha256'])
    write(args.output/'request.json',record)
    model=Model(case['voxels_xyz'],case['voxel_radius_m'])
    assert model.robot_sha256==case['robot_sha256'] and model.collision_geometry_sha256==case['collision_geometry_sha256']
    options=csd.DrmPlannerOptions()
    for k,v in cfg['drm'].items():setattr(options,k,v)
    planner=csd.DrmPlanner(model.csd_plant,options,True)
    planner.LoadRoadmap(str(road/'native.map'))
    planner.BuildCollisionSet(model.voxels)
    module,build=load(args.build_dir)
    record['layout_before']=layout_guard(module,planner)
    record['native_library_sha256']=build['native_library_sha256']
    record['sidecar_binary_sha256']=build['binary_sha256']
    write(args.output/'before_query.json',record)
    start=time.perf_counter()
    try:
        if args.mode=='native':
            success,path=planner.Plan(case['q_start'],case['q_goal'],model.voxels,model.voxel_radius)
        else:
            success,path,evidence=query(planner,case['q_start'],case['q_goal'],model.voxels_xyz,
                                       model.voxel_radius,build_dir=args.build_dir,out=args.output/'safe_query')
            record['safe_query_evidence']=evidence
        path=np.asarray(path,float)
        np.save(args.output/'path.npy',path)
        record.update(returned=True,success=bool(success),status='path_returned' if success else 'frontend_failed',
                      path_sha256=sha(args.output/'path.npy'),path_shape=list(path.shape))
    except RuntimeError as error:
        record.update(returned=True,success=False,status='frontend_failed',error=repr(error))
    record.update(query_seconds=time.perf_counter()-start,layout_after=layout_guard(module,planner))
    write(args.output/'result.json',record)
    print(json.dumps(record,indent=2))


if __name__=='__main__':main()
