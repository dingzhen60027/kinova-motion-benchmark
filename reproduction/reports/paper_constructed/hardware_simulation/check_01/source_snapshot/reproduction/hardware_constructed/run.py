"""Execute actual IK/DRM/EI-ZO/SCSTO on fifteen constructed hardware-like scenes.

No robot, camera, ROS, sensor driver or hardware execution is contacted. This
supplement remains separate from both the main simulation grid and paper Table II.
"""
import argparse
from collections import Counter
import datetime
import hashlib
from pathlib import Path
import shutil
import sys
import time
import traceback
import numpy as np
import csdecomp as csd
from reproduction.common import ROOT, atomic, digest, read
from reproduction.run import runtime_versions
from reproduction.paper_constructed.run import execute_child
from reproduction.paper_constructed.data import endpoint_audit, online_pose_ik, planner
from reproduction.paper_constructed.pipeline import Pipeline
from reproduction.paper_constructed.validation import polygon_samples
from .model import IIWA, robot_urdf, exact_box_flags
from .scenes import LAYOUTS, layout, surface_cloud, voxelize

PACKAGE=Path(__file__).parent
REPORTS=ROOT/'reproduction/reports/paper_constructed/hardware_simulation'


def identity():
    paths=[p for p in PACKAGE.rglob('*')if p.is_file()and p.suffix in {'.py','.json','.urdf'}and '__pycache__'not in p.parts and p.name not in {'plot.py','report.py','analyze.py'}]
    paths.extend(ROOT/'reproduction'/name for name in ['common.py','run.py','forest.py','optimization.py','pipeline.py','probe_public_iiwa.py','paper_protocol/model.py','paper_constructed/model.py','paper_constructed/run.py','paper_constructed/data.py','paper_constructed/pipeline.py','paper_constructed/optimization.py','paper_constructed/validation.py'])
    paths.append(ROOT/'local/python.sh')
    return {str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()for p in sorted(paths)}


def check_identity(manifest):
    if digest(identity())!=manifest['implementation_hash']or runtime_versions()!=manifest['runtime_versions']:
        raise ValueError('Hardware simulation source/environment changed; retain old run and create a new one')


def model_check(cfg,out):
    out.mkdir(parents=True,exist_ok=True);f=IIWA(cfg)
    points=np.vstack([cfg['initial_right_configuration'],cfg['left_target_generating_configuration'],
        np.random.default_rng(250410783).uniform(f.lower,f.upper,(256,7))])
    cuda=f.cuda(points);drake=f.independent(points)
    np.savez_compressed(out/'static_collision_points.npz',q=points,cuda=cuda,independent=drake)
    record=dict(status='passed'if np.array_equal(cuda,drake)else'failed',metadata=f.metadata(),
        samples=len(points),disagreements=int(np.count_nonzero(cuda!=drake)),
        declared_initial_free=bool(cuda[0]and drake[0]),declared_left_anchor_free=bool(cuda[1]and drake[1]),
        declared_anchor_limits_valid=bool(np.all(points[:2]>=f.lower)and np.all(points[:2]<=f.upper)),
        left_target_pose=f.pose(cfg['left_target_generating_configuration']),right_target_pose=f.pose(cfg['right_target_generating_configuration']))
    scene=layout(0,cfg);cloud,labels=surface_cloud(scene,cfg['cloud_surface_grid'])
    centers,keys,online=voxelize(cloud,labels,scene,cfg['online_voxel_side'])
    g=IIWA(cfg,centers);a=g.cuda(points[:34]);b=g.independent(points[:34])
    np.savez_compressed(out/'online_collision_points.npz',q=points[:34],cuda=a,independent=b,voxel_centers=centers)
    record.update(online_samples=34,online_disagreements=int(np.count_nonzero(a!=b)),online_model=g.metadata())
    if not np.array_equal(a,b):record['status']='failed'
    if record['status']=='passed'and not(record['declared_initial_free']and record['declared_left_anchor_free']and record['declared_anchor_limits_valid']):
        record.update(status='predicates_matched_but_fixed_task_invalid',reason='At least one frozen task anchor is not collision-free in the static model; no roadmap/planning is permitted for this run')
    atomic(out/'model_check.json',record)
    if record['status']!='passed':raise ValueError('Model/task preflight rejected: '+record['status']+'; raw points retained, no task reselection')
    return record


def build(root):
    manifest=read(root/'manifest.json');check_identity(manifest);cfg=manifest['config'];out=root/'roadmap';out.mkdir(exist_ok=True)
    record=dict(status='running',stage='model',requested_nodes=cfg['roadmap_nodes'])
    def persist():atomic(out/'roadmap.json',record)
    persist();start=time.perf_counter()
    try:
        f=IIWA(cfg);rng=np.random.default_rng(cfg['roadmap_seed']);selected=[];sampled=0;tic=time.perf_counter()
        while len(selected)<cfg['roadmap_nodes']and sampled<cfg['roadmap_sampling_limit']:
            batch=min(32768,cfg['roadmap_sampling_limit']-sampled)
            q=rng.uniform(f.lower,f.upper,(batch,7)).astype(np.float32);sampled+=batch
            selected.extend(q[f.cuda(q)])
            record.update(stage='sampling',sampled=sampled,accepted=len(selected));persist()
        if len(selected)<cfg['roadmap_nodes']:raise ValueError('Fixed offline sampling budget exhausted')
        nodes=np.asarray(selected[:cfg['roadmap_nodes']]);np.save(out/'samples.npy',nodes)
        record.update(sampling_seconds=time.perf_counter()-tic,model=f.metadata())
        options=csd.RoadmapOptions()
        options.robot_map_size_x,options.robot_map_size_y,options.robot_map_size_z=cfg['offline_map_size']
        options.map_center=np.asarray(cfg['offline_map_center'])
        options.offline_voxel_resolution=cfg['offline_voxel_side']/2
        options.max_task_space_distance_between_nodes=cfg['max_task_space_distance']
        options.max_configuration_distance_between_nodes=cfg['max_configuration_distance']
        options.edge_step_size=cfg['lazy_edge_step'];options.nodes_processed_before_debug_statement=100000
        builder=csd.RoadmapBuilder(f.native,'public_iiwa7_wsg_probe::'+cfg['tcp_link'],options)
        phases={}
        for name,operation in [('add_nodes',lambda:builder.add_nodes_manual(nodes.T)),('graph',lambda:builder.build_roadmap(cfg['roadmap_neighbors'])),('pose_map',builder.build_pose_map),('collision_map',builder.build_collision_map),('serialization',lambda:builder.write(str(out/'native.map')))]:
            record['stage']=name;persist();tic=time.perf_counter();operation();phases[name]=time.perf_counter()-tic
            record['phase_seconds']=phases;persist()
        record.update(status='completed',stage='completed',actual_nodes=len(builder.get_drm().id_to_node_map),
            native_voxel_half_side=options.offline_voxel_resolution,offline_voxel_side=cfg['offline_voxel_side'],
            offline_total_seconds=time.perf_counter()-start)
    except BaseException as error:
        record.update(status='failed',error=repr(error),traceback=traceback.format_exc());raise
    finally:persist()


def trial(root,index):
    manifest=read(root/'manifest.json');check_identity(manifest);cfg=manifest['config'];out=root/'trials'/f'{index:02d}'
    inputs=read(out/'input.json');scene=read(root/'scenes'/f'{index:02d}.json')
    record=dict(index=index,status='running',stage='synthetic_cloud',target_side=scene['target_side'],
        physical_execution=False,camera_perception_seconds=None,method='SCSTO',start=inputs['start'],target_pose=inputs['target_pose'])
    start=time.perf_counter()
    def checkpoint(stage):record['stage']=stage;atomic(out/'result.json',record)
    checkpoint('synthetic_cloud')
    try:
        tic=time.perf_counter();cloud,labels=surface_cloud(scene,cfg['cloud_surface_grid'])
        record['synthetic_surface_generation_seconds']=time.perf_counter()-tic
        tic=time.perf_counter();centers,keys,online=voxelize(cloud,labels,scene,cfg['online_voxel_side'])
        record.update(voxelization_seconds=time.perf_counter()-tic,raw_cloud_points=len(cloud),online_points=int(online.sum()),voxels=len(centers),online_voxel_radius=cfg['online_voxel_radius'])
        np.savez_compressed(out/'synthetic_cloud.npz',points=cloud,solid_labels=labels,online_mask=online,voxel_keys=keys,voxel_centers=centers)
        checkpoint('model');tic=time.perf_counter();f=IIWA(cfg,centers);record['model_setup_seconds']=time.perf_counter()-tic
        atomic(out/'model.json',f.metadata());f.start=np.asarray(inputs['start'],np.float32).astype(float)
        audit=endpoint_audit(f,f.start);atomic(out/'start_audit.json',audit)
        if not audit['canonical_valid']:
            record.update(status='start_invalid',reason='Fixed state collides after this predeclared scene modification; no state/target reselection');return
        checkpoint('load_roadmap');p=planner(f);tic=time.perf_counter();p.LoadRoadmap(str(root/'roadmap/native.map'))
        record['map_load_seconds']=time.perf_counter()-tic
        tic=time.perf_counter();p.BuildCollisionSet(f.voxels);record['collision_set_seconds']=time.perf_counter()-tic
        checkpoint('online_ik');ik=online_pose_ik(f,p.drm,inputs['target_pose'],out,collision_set=p.collision_set)
        record['ik_seconds']=ik['seconds']
        if ik['status']!='success':record.update(status='ik_failed',ik=ik);return
        f.goal=np.asarray(ik['goal']);f.cfg.update(start=f.start,goal=f.goal)
        record['goal']=f.goal;atomic(out/'endpoints.json',dict(start=audit,goal=endpoint_audit(f,f.goal)))
        checkpoint('drm');tic=time.perf_counter()
        try:success,path=p.Plan(f.start,f.goal,f.voxels,f.voxel_radius)
        finally:record['drm_seconds']=time.perf_counter()-tic
        record['drm_success']=bool(success);record['planner_options']=f.native_planner_options
        if not success:record['status']='drm_failed';return
        path=np.asarray(path);np.savez_compressed(out/'seed_path.npz',path=path)
        checkpoint('seed_validation');tic=time.perf_counter();samples=polygon_samples(path,cfg['validation_step']);cuda=f.cuda(samples);independent=f.independent(samples)
        np.savez_compressed(out/'seed_validation.npz',q=samples,cuda=cuda,independent=independent)
        valid=bool(cuda.all()and independent.all())
        record.update(seed_validation_seconds=time.perf_counter()-tic,drm_seed_valid=valid,seed_nodes=len(path),seed_samples=len(samples),seed_model_disagreements=int((cuda!=independent).sum()))
        if not valid:record['status']='invalid_seed_path';return
        if len(path)<2:record.update(status='degenerate_seed',reason='No motion path returned; not counted as optimized trajectory');return
        # A fixed independent warmup is recorded separately from the measured
        # SCS/TO calls. It cannot change the selected path or scene.
        checkpoint('eizo_warmup');tic=time.perf_counter()
        try:Pipeline(f,1337).inflate(path[0],path[1])
        except Exception as error:record['warmup_failure']=repr(error)
        record['eizo_warmup_seconds']=time.perf_counter()-tic
        checkpoint('scsto');method_out=out/'SCSTO';method_out.mkdir(exist_ok=True)
        answer=Pipeline(f,1337).solve('SCSTO',path,p.drm,method_out)
        candidate=answer.pop('candidate');regions=answer.pop('regions');validation=answer.pop('validation',None)
        record.update(answer,solver_success=bool(candidate['solver_success']),solver_status=candidate['solver_status'],regions=len(regions))
        if validation is not None:
            samples=validation.pop('_samples');np.savez_compressed(method_out/'trajectory_samples.npz',**samples)
            atomic(method_out/'validation.json',validation)
            checkpoint('exact_scene_box_validation');tic=time.perf_counter()
            truth,minimum=exact_box_flags(cfg,scene,samples['q'])
            np.savez_compressed(method_out/'exact_scene_validation.npz',q=samples['q'],free=truth,minimum_signed_distance=minimum)
            record.update(exact_scene_validation_seconds=time.perf_counter()-tic,exact_scene_collision_free=bool(truth.all()),exact_scene_collision_samples=int((~truth).sum()),exact_scene_minimum_signed_distance=float(minimum.min()),
                validation_passed=bool(validation['passed']),native_cuda_collision_free=validation['native_cuda_collision_free'],independent_voxel_collision_free=validation['independent_collision_free'],
                executed_in_virtual_state=bool(candidate['solver_success']and validation['passed']and truth.all()),duration=validation['duration'],cost=validation['cost'],model_disagreements=validation['model_disagreements'])
            if record['executed_in_virtual_state']:
                record.update(status='success_numerical',terminal_configuration=samples['q'][-1],virtual_state_transition='Set to independently validated trajectory endpoint; no physical dynamics/controller or hardware execution simulated')
            else:record['status']='validation_failed'
        else:record.update(validation_passed=False,executed_in_virtual_state=False)
    except BaseException as error:
        record.update(status='failed',error=repr(error),traceback=traceback.format_exc());traceback.print_exc()
    finally:
        record.update(wall_seconds=time.perf_counter()-start)
        # This explicitly excludes synthetic cloud generation/model loading/
        # warmup/final validation. It is not paper camera-to-motion latency.
        fields=[k for k in ['collision_set_seconds','ik_seconds','drm_seconds','construction_seconds','optimization_seconds']if k in record]
        record['planning_components_recorded']=fields
        record['planning_component_seconds']=sum(record[k]for k in fields)if fields else None
        atomic(out/'result.json',record)


def summarize(root):
    manifest=read(root/'manifest.json');rows=[read(root/'trials'/f'{i:02d}'/'result.json')for i in manifest['trials']]
    metrics={}
    for key in ['synthetic_surface_generation_seconds','voxelization_seconds','collision_set_seconds','ik_seconds','drm_seconds','construction_seconds','optimization_seconds','validation_seconds','exact_scene_validation_seconds','planning_component_seconds','wall_seconds','duration','regions','voxels']:
        values=[float(row[key])for row in rows if row.get(key)is not None]
        metrics[key]=dict(recorded=len(values),mean=float(np.mean(values))if values else None,sample_sd=float(np.std(values,ddof=1))if len(values)>1 else None)
    summary=dict(status='completed',run_dir=str(root),scope='constructed_hardware_condition_simulation_only',physical_experiment=False,sensors_simulated=False,
        original_paper_hardware_experiment_reproduced=False,requested=len(rows),strict_virtual_successes=sum(bool(x.get('executed_in_virtual_state'))for x in rows),
        case_status_counts=dict(Counter(x['status']for x in rows)),metrics=metrics,
        reference_comparison='No direct success-rate/speed comparison to physical Table II: synthetic scenes, calibration, collision pairs, hardware, sensing and execution differ.')
    atomic(root/'summary.json',summary);return summary


def execute(args):
    stamp=datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    root=(args.output or ROOT/'reproduction/runs'/(stamp+'_constructed_hardware_simulation')).resolve();root.mkdir(parents=True,exist_ok=False)
    cfg=read(PACKAGE/'config.json')
    if args.nodes is not None:cfg['roadmap_nodes']=args.nodes
    trials=list(range(len(LAYOUTS)))if args.trials is None else args.trials
    if not trials or len(set(trials))!=len(trials)or any(i<0 or i>=15 for i in trials):raise ValueError('Choose distinct trial indices in 0..14')
    source=identity();manifest=dict(status='running',protocol=cfg['protocol'],physical_experiment=False,config=cfg,trials=trials,
        matches_declared_full_size=cfg['roadmap_nodes']==35000 and trials==list(range(15)),source_files=source,implementation_hash=digest(source),
        runtime_versions=runtime_versions(),created_utc=stamp,command=[sys.executable,*sys.argv])
    atomic(root/'manifest.json',manifest);print('RUN_DIR='+str(root),flush=True)
    for name in source:
        dest=root/'source_snapshot'/name;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(ROOT/name,dest)
    try:
        check=model_check(cfg,root/'model_check');atomic(root/'model_check_summary.json',check)
        (root/'assembled_static.urdf').write_text(robot_urdf(cfg));(root/'scenes').mkdir()
        for index in range(15):atomic(root/'scenes'/f'{index:02d}.json',layout(index,cfg))
        targets={side:check[side+'_target_pose']for side in ['left','right']}
        atomic(root/'fixed_targets.json',targets)
        command=['bash','local/python.sh','-m','reproduction.hardware_constructed.run','_build','--run-dir',str(root)]
        outcome=execute_child(command,root/'offline_process',cfg['offline_timeout_seconds'])
        atomic(root/'offline_process/outcome.json',dict(command=command,**outcome))
        if outcome['exit_code']!=0 or outcome['timed_out']:
            partial=read(root/'roadmap/roadmap.json')if(root/'roadmap/roadmap.json').exists()else None
            atomic(root/'roadmap/roadmap.json',dict(status='timeout'if outcome['timed_out']else'failed',partial_record=partial,process=outcome))
            for index in trials:
                atomic(root/'trials'/f'{index:02d}'/'result.json',dict(index=index,status='offline_failed',executed_in_virtual_state=False))
        else:
            state=np.asarray(cfg['initial_right_configuration'],np.float32).astype(float)
            for completed_count,index in enumerate(trials,1):
                out=root/'trials'/f'{index:02d}';out.mkdir(parents=True)
                side='left'if index%2==0 else'right'
                atomic(out/'input.json',dict(index=index,start=state,target_pose=targets[side],target_side=side,
                    state_rule='Last strictly validated endpoint; hold state after failure. Fixed target schedule is never resampled.'))
                command=['bash','local/python.sh','-m','reproduction.hardware_constructed.run','_trial','--run-dir',str(root),'--index',str(index)]
                outcome=execute_child(command,out/'process',cfg['trial_timeout_seconds'])
                atomic(out/'process/outcome.json',dict(command=command,**outcome))
                result_path=out/'result.json';result=read(result_path)if result_path.exists()else{}
                if outcome['timed_out']or outcome['exit_code']!=0 or result.get('status')=='running':
                    result=dict(index=index,status='timeout'if outcome['timed_out']else'process_error',partial_record=result,process=outcome,executed_in_virtual_state=False)
                    atomic(result_path,result)
                if result.get('executed_in_virtual_state'):state=np.asarray(result['terminal_configuration'])
                manifest.update(completed_trials=completed_count,last_virtual_state=state);atomic(root/'manifest.json',manifest)
                print(index,result['status'],flush=True)
        summary=summarize(root);manifest.update(status='completed_with_failures_retained',summary=summary)
        REPORTS.mkdir(parents=True,exist_ok=True)
        # Each run receives a unique report pointer; no former result is replaced.
        atomic(REPORTS/(stamp+'_run.json'),dict(run_dir=str(root),summary=summary,implementation_hash=manifest['implementation_hash']))
    except BaseException as error:
        manifest.update(status='failed',error=repr(error),traceback=traceback.format_exc())
        for index in trials:
            path=root/'trials'/f'{index:02d}'/'result.json'
            partial=read(path)if path.exists()else None
            if partial is None or partial.get('status')=='running':
                atomic(path,dict(index=index,status='interrupted'if partial else'not_attempted_due_to_run_failure',partial_record=partial,executed_in_virtual_state=False))
        atomic(REPORTS/(stamp+'_failed_run.json'),dict(run_dir=str(root),status='failed',error=repr(error)))
        raise
    finally:atomic(root/'manifest.json',manifest)
    return root


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('command',choices=['check','run','_build','_trial'])
    parser.add_argument('--output',type=Path);parser.add_argument('--run-dir',type=Path);parser.add_argument('--index',type=int)
    parser.add_argument('--nodes',type=int);parser.add_argument('--trials',nargs='+',type=int)
    args=parser.parse_args()
    if args.command=='_build':build(args.run_dir.resolve())
    elif args.command=='_trial':trial(args.run_dir.resolve(),args.index)
    elif args.command=='check':
        if args.output is None:raise ValueError('Use a fresh --output directory for the model check')
        args.output.mkdir(parents=True,exist_ok=False)
        try:model_check(read(PACKAGE/'config.json'),args.output)
        except BaseException as error:
            atomic(args.output/'failure.json',dict(status='failed',error=repr(error),traceback=traceback.format_exc()));raise
    else:execute(args)


if __name__=='__main__':main()
