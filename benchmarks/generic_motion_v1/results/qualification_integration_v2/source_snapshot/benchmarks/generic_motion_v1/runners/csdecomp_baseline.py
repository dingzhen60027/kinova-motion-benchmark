"""Real native DRM/EI-ZO and installed author SCSPlanning optimizers.

No example or CUDA source is modified. The installed author SCSPlanning
release uses Clarabel; relative to the paper's commercial backend this is
a solver adaptation. Every failed stage and numerical candidate is retained.
"""
from __future__ import annotations
import importlib
import inspect
import json
import os
from pathlib import Path
import subprocess
import threading
import time
import traceback
import numpy as np
from ..roadmap import HERE, ROOT, load_config, sha, write
from .validate import validate, _numeric_array


class MemoryMonitor:
    """Sample this worker's nvidia-smi process allocation; never claim exact peak."""
    def __init__(self):
        self.stop=threading.Event();self.peak=None;self.samples=0;self.error=None
        self.thread=threading.Thread(target=self._run,daemon=True)
    def _run(self):
        while not self.stop.is_set():
            try:
                result=subprocess.run(['nvidia-smi','--query-compute-apps=pid,used_gpu_memory','--format=csv,noheader,nounits'],capture_output=True,text=True,timeout=2)
                if result.returncode:
                    self.error=result.stderr.strip();return
                for line in result.stdout.splitlines():
                    fields=line.split(',')
                    if len(fields)==2 and fields[0].strip()==str(os.getpid()):
                        memory=int(fields[1].strip());self.peak=max(self.peak or 0,memory);self.samples+=1
            except Exception as exc:
                self.error=repr(exc);return
            self.stop.wait(.2)
    def __enter__(self):
        self.thread.start();return self
    def __exit__(self,*args):
        self.stop.set();self.thread.join(timeout=2.2)
    def report(self):
        return dict(sampled_peak_process_allocation_mib=self.peak,samples=self.samples,
                    sampling_period_s=.2,method='nvidia-smi process allocation; sampled peak, not CUDA allocator exact high water mark',error=self.error)


def _save_candidate(out,candidate):
    arrays={k:v for k,v in candidate.items() if isinstance(v,np.ndarray)}
    np.savez_compressed(out/'candidate.npz',**arrays)
    write(out/'candidate.json',{k:v for k,v in candidate.items() if k not in arrays})


def _save_regions(out,regions):
    values={}
    for i,r in enumerate(regions):
        values[f'A_{i:04d}']=np.asarray(r.A());values[f'b_{i:04d}']=np.asarray(r.b())
    np.savez_compressed(out/'regions.npz',**values)
    write(out/'regions.json',dict(count=len(regions),facets=[len(r.b()) for r in regions],
                                 order='one region per fixed DRM edge, start to goal'))


def cover_region_anchors(region,anchors,maximum_normalized_correction,out=None):
    """Normalize float32 region handoff without moving the fixed endpoints.

    A plane's correction is divided by its normal norm, so row scaling cannot
    evade the fixed tolerance. Larger errors fail closed. Every saved native
    halfspace and every anchor remains inspectable before and after the repair.
    """
    from pydrake.all import HPolyhedron
    A=np.asarray(region.A(),dtype=float)
    b=np.asarray(region.b(),dtype=float)
    anchors=np.asarray(anchors,dtype=float)
    directory=None if out is None else Path(out)
    if directory is not None:
        directory.mkdir(parents=True,exist_ok=True)
        np.savez_compressed(directory/'before.npz',A=A,b=b,anchors=anchors)
    if (A.ndim!=2 or A.shape[1]!=7 or len(A)==0 or b.shape!=(len(A),) or anchors.ndim!=2
            or anchors.shape[1]!=7 or len(anchors)==0 or not np.isfinite(A).all()
            or not np.isfinite(b).all() or not np.isfinite(anchors).all()
            or not np.isfinite(maximum_normalized_correction) or maximum_normalized_correction<0):
        raise ValueError('Malformed region, anchors, or numerical coverage tolerance')
    with np.errstate(over='ignore',invalid='ignore'):
        residual=A@anchors.T-b[:,None]
        norms=np.linalg.norm(A,axis=1)
    if not np.isfinite(residual).all() or not np.isfinite(norms).all():
        raise ValueError('Region coverage arithmetic overflowed; cannot bound correction')
    correction=np.maximum(0.,np.max(residual,axis=1))
    normalized=np.divide(correction,norms,out=np.zeros_like(correction),where=norms>0)
    normalized[(norms==0)&(correction>0)]=np.inf
    largest=float(np.max(normalized,initial=0.))
    record=dict(status='accepted' if largest<=maximum_normalized_correction else 'rejected',
        maximum_normalized_correction=largest,allowed_normalized_correction=maximum_normalized_correction,
        corrected_planes=int(np.count_nonzero(correction)),normal_matrix_changed=False,
        fixed_endpoints_changed=False,maximum_raw_anchor_residual_before=float(np.max(residual,initial=0.)),
        note='Bounded numerical region handoff only; unchanged robot/obstacles; final collision validator still applies')
    if directory is not None:write(directory/'coverage.json',record)
    if largest>maximum_normalized_correction:
        raise ValueError(f'Region requires normalized endpoint correction {largest:.9g}, '
                         f'exceeding fixed numerical tolerance {maximum_normalized_correction:.9g}')
    corrected_b=b+correction
    result=HPolyhedron(A,corrected_b)
    after=A@anchors.T-corrected_b[:,None]
    record['maximum_raw_anchor_residual_after']=float(np.max(after,initial=0.))
    if directory is not None:
        np.savez_compressed(directory/'after.npz',A=A,b=corrected_b,anchors=anchors,
                            normalized_corrections=normalized,raw_corrections=correction)
        write(directory/'coverage.json',record)
    return result


def _region_anchors(path,index,endpoints):
    anchors=[path[index],path[index+1]]
    if index==0:anchors.append(endpoints[0])
    if index==len(path)-2:anchors.append(endpoints[1])
    return np.asarray(anchors,dtype=float)


def dependency_audit(out,method='EI_ZO'):
    """Inspect installed actual optimizer code and execute an open-source solve."""
    if method not in ('EI_ZO','NTO'):
        raise ValueError('Unsupported baseline method')
    from pydrake.all import ClarabelSolver,IpoptSolver,MathematicalProgram
    import csdecomp
    modules=[csdecomp]
    if method=='EI_ZO':
        import scsplanning,pybezier
        modules.extend([scsplanning,pybezier,importlib.import_module('scsplanning.polygonal'),
             importlib.import_module('scsplanning.base_program')])
    program=MathematicalProgram();x=program.NewContinuousVariables(1)
    program.AddBoundingBoxConstraint(1.,2.,x);program.AddLinearCost(x[0])
    solver=ClarabelSolver() if method=='EI_ZO' else IpoptSolver()
    name='clarabel' if method=='EI_ZO' else 'ipopt'
    record=dict(method=method,modules={m.__name__:dict(path=str(m.__file__),sha256=sha(m.__file__)) for m in modules},
                clarabel=dict(available=ClarabelSolver().available(),enabled=ClarabelSolver().enabled()),
                ipopt=dict(available=IpoptSolver().available(),enabled=IpoptSolver().enabled()),
                scsplanning_solver=('Clarabel' if 'ClarabelSolver' in inspect.getsource(modules[-1]) else 'inspect module receipt') if method=='EI_ZO' else None,
                fidelity='Native CSDecomp; author SCSPlanning with Clarabel for EI_ZO or declared Drake KTO/Ipopt adaptation for NTO; not exact FR3 reproduction')
    write(out/'dependencies.json',record)
    if not solver.available() or not solver.enabled():
        raise ImportError(f'{name} solver is unavailable or disabled')
    solution=solver.Solve(program)
    record[name].update(probe_success=bool(solution.is_success()),probe_solution=float(solution.GetSolution(x)[0]))
    write(out/'dependencies.json',record)
    if not solution.is_success():
        raise RuntimeError(f'{name} actual dependency probe failed')
    return record


def _nto(path,model,case,cfg,out,deadline):
    """Declared Drake KTO/Ipopt baseline with the same collision geometry.

    Distance uses an order-two B-spline, hence precisely a PWL objective.
    Time uses a cubic B-spline with endpoint copies and shape-preserving knot
    insertion. These are documented adapter choices, not recovered paper code.
    """
    from pydrake.all import (BsplineBasis,BsplineTrajectory,KnotVectorType,
        KinematicTrajectoryOptimization,MinimumDistanceLowerBoundConstraint,IpoptSolver)
    timed=case['_objective']=='minimum_time'
    options=cfg['nto'];start=time.perf_counter()
    native=model.nto_plant()
    controls=np.vstack([path[0],path,path[-1]]) if timed else path.copy()
    order=4 if timed else 2
    duration=float(np.linalg.norm(np.diff(path,axis=0),axis=1).sum()+2.) if timed else 1.
    initial=BsplineTrajectory(BsplineBasis(order,len(controls),KnotVectorType.kClampedUniform,0.,duration),[p[:,None] for p in controls])
    if timed:
        while len(initial.control_points())<options['minimum_control_points']:
            unique=np.unique(initial.basis().knots());i=np.argmax(np.diff(unique))
            initial.InsertKnots([float((unique[i]+unique[i+1])/2)])
    np.savez_compressed(out/'nto_initial.npz',controls=np.asarray([p.ravel() for p in initial.control_points()]),knots=initial.basis().knots(),original_path=path)
    traj=KinematicTrajectoryOptimization(initial);prog=traj.get_mutable_prog()
    traj.AddPositionBounds(model.lower,model.upper)
    traj.AddPathPositionConstraint(case['q_start'],case['q_start'],0.)
    traj.AddPathPositionConstraint(case['q_goal'],case['q_goal'],1.)
    if timed:
        traj.AddDurationCost(1.);traj.AddDurationConstraint(.05,100.)
        traj.AddVelocityBounds(-np.ones(7)*cfg['velocity_limit_rad_s'],np.ones(7)*cfg['velocity_limit_rad_s'])
        traj.AddAccelerationBounds(-np.ones(7)*cfg['acceleration_limit_rad_s2'],np.ones(7)*cfg['acceleration_limit_rad_s2'])
        traj.AddPathVelocityConstraint(np.zeros(7),np.zeros(7),0.)
        traj.AddPathVelocityConstraint(np.zeros(7),np.zeros(7),1.)
    else:
        traj.AddPathLengthCost();traj.AddDurationConstraint(1.,1.)
    constraint=MinimumDistanceLowerBoundConstraint(native.plant,options['clearance_m'],native.plant_context,influence_distance_offset=options['influence_distance_m'])
    locations=np.arange(1,options['collocation_points']+1)/(options['collocation_points']+1)
    for s in locations:
        traj.AddPathPositionConstraint(constraint,float(s))
    remaining=min(cfg['optimizer_budget_seconds'],deadline-time.perf_counter())
    if remaining<=0:
        raise TimeoutError('NTO construction exhausted online budget')
    solver=IpoptSolver()
    parameters=dict(max_iter=options['maximum_iterations'],max_cpu_time=remaining,tol=1e-6,
                    print_level=0,file_print_level=5,output_file=str((out/'ipopt.log').resolve()))
    for key,value in parameters.items():
        prog.SetSolverOption(solver.solver_id(),key,value)
    write(out/'nto_parameters.json',dict(parameters=parameters,collision_pairs=native.collision_pair_count,
          collocation_locations=locations,clearance_m=options['clearance_m'],
          transcription='Drake KTO + Ipopt adaptation; fixed DRM initialization; no success filtering',
          construction_seconds=time.perf_counter()-start))
    result=solver.Solve(prog)
    curve=traj.ReconstructTrajectory(result)
    return dict(kind='bspline',objective=case['_objective'],solver='IPOPT',solver_success=bool(result.is_success()),
                solver_status=str(result.get_solution_result()),spline_order=order,
                controls=np.asarray([p.ravel() for p in curve.control_points()]),knots=np.asarray(curve.basis().knots()),
                duration=float(curve.end_time()-curve.start_time()),provenance='adapted_Drake_KTO_NTO')


def _input_endpoints(case):
    """Reject recorded or malformed input before any conversion or GPU call."""
    if case.get('validation_status') != 'valid_input':
        return None
    try:
        endpoints=_numeric_array([case.get('q_start'),case.get('q_goal')],'Fixed endpoints')
    except (TypeError,ValueError):
        return None
    if endpoints.shape != (2,7) or not np.isfinite(endpoints).all():
        return None
    return endpoints


def _method_seed(seed):
    if isinstance(seed,(bool,np.bool_)) or not isinstance(seed,(int,np.integer)) or not 0<=seed<2**32:
        raise ValueError('Method seed must be an unsigned 32-bit integer')
    return int(seed)


def _checked_native_path(path,endpoints,tolerance):
    values=np.asarray(path,float)
    if values.ndim!=2 or values.shape[1]!=7 or len(values)<2 or not np.isfinite(values).all():
        raise ValueError('Native DRM returned a malformed or nonfinite Nx7 path')
    if not np.allclose(values[[0,-1]],endpoints,atol=tolerance,rtol=0):
        raise ValueError('Native DRM path does not connect the fixed requested endpoints')
    return values


def _assert_motion_limits(case,cfg):
    """The fixed manifest defines the task; evaluation cannot silently change it.

    This v1 runner supports uniform per-joint limits. A different vector or
    endpoint velocity requires a separately named and frozen input protocol.
    """
    for record_key,config_key in (
            ('velocity_limits_rad_s','velocity_limit_rad_s'),
            ('acceleration_limits_rad_s2','acceleration_limit_rad_s2')):
        try:
            manifest_limits=_numeric_array(case[record_key],record_key)
            configured=float(cfg[config_key])
        except (KeyError,TypeError,ValueError) as exc:
            raise ValueError(f'Missing or malformed frozen motion limits: {record_key}') from exc
        if (manifest_limits.shape != (7,) or not np.isfinite(manifest_limits).all()
                or np.any(manifest_limits<=0) or not np.isfinite(configured) or configured<=0
                or not np.array_equal(manifest_limits,np.full(7,configured))):
            raise ValueError(f'Evaluation {config_key} differs from frozen manifest {record_key}; '
                             'a different task requires a separately frozen experiment protocol')
    try:
        endpoint_velocity=_numeric_array(case['endpoint_velocity_rad_s'],'Endpoint velocity')
    except (KeyError,TypeError,ValueError) as exc:
        raise ValueError('Missing or malformed frozen endpoint velocity') from exc
    if endpoint_velocity.shape != (7,) or not np.array_equal(endpoint_velocity,np.zeros(7)):
        raise ValueError('This frozen protocol requires seven zero endpoint velocities')


def run_case(case,roadmap_dir,out,objective='minimum_distance',seed=0,method='EI_ZO',config=None):
    """Run a fixed query, saving every stage; caller enforces process watchdog.

    ``online_budget_seconds`` is checked between calls. A native kernel/solver
    cannot be interrupted safely in-process: the CLI worker must have the same
    total wall-clock subprocess deadline and retain these incremental files.
    """
    start=time.perf_counter();out=Path(out);out.mkdir(parents=True,exist_ok=True)
    if objective not in ('minimum_distance','minimum_time') or method not in ('EI_ZO','NTO'):
        raise ValueError('Unknown objective or method')
    if (out/'result.json').exists():
        raise FileExistsError('Refusing to overwrite an existing run result')
    try:
        cfg=load_config(config)
        seed=_method_seed(seed)
    except (ValueError,TypeError,OverflowError) as exc:
        result=dict(status='infrastructure_error',error_category='input_protocol_mismatch',
            failed_stage='input_contract',final_stage='input_contract',exception=repr(exc),
            problem_id=case.get('problem_id'),scene_id=case.get('scene_id'),family=case.get('family'),
            objective=objective,method=method,frontend_success=False,metrics={},
            timing={'cold_e2e_seconds':time.perf_counter()-start,'warm_query_seconds':None})
        write(out/'input.json',case);write(out/'result.json',result)
        (out/'exception.txt').write_text(traceback.format_exc())
        return result
    deadline=start+cfg['online_budget_seconds']
    case=dict(case);case['_objective']=objective
    result=dict(status='running',problem_id=case['problem_id'],scene_id=case['scene_id'],family=case['family'],
        objective=objective,method=method,method_seed=int(seed),roadmap_id=None,frontend_success=False,
        source_code_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        source_sha256={name:sha(path) for name,path in [('runner',__file__),('validator',Path(__file__).with_name('validate.py'))]},
        timing={},metrics={},budget_seconds=cfg['online_budget_seconds'],
        reproduction_fidelity='RSS-inspired Kinova-7D adaptation; local Clarabel backend; native EI-ZO unchanged by this benchmark',
        seed_semantics='EI-ZO EizoOptions.seed is passed to native cuRAND; DRM graph sampling uses separate saved PCG64 seed; optimizer initialization is deterministic fixed DRM path')
    write(out/'input.json',case);write(out/'evaluation.json',cfg);write(out/'result.json',result)
    phase='initializing';warm_started=None
    def stage(name):
        nonlocal phase
        phase=name
        write(out/'phase.json',dict(phase=name,elapsed_seconds=time.perf_counter()-start,timing=result['timing']))
        write(out/'result.json',result)
        if time.perf_counter()>=deadline:
            raise TimeoutError(f'Online deadline before {name}')
    memory=MemoryMonitor()
    with memory:
        try:
            # A frozen generation failure is an input outcome, never a CUDA
            # infrastructure error and never an invitation to resample.
            endpoints=_input_endpoints(case)
            if endpoints is None:
                result['status']='invalid_endpoint'
                result['input_validation_status']=case.get('validation_status')
                result['generation_rejections']=case.get('generation_rejections',case.get('endpoint_sampling'))
                result['error']='Frozen input has no valid finite endpoint pair; no resampling performed'
                return result
            stage('input_contract')
            _assert_motion_limits(case,cfg)
            import csdecomp as csd
            from pydrake.all import HPolyhedron
            from ..model import Model
            stage('model_setup');tic=time.perf_counter()
            model=Model(case['voxels_xyz'],case['voxel_radius_m'])
            for key in ('robot_sha256','collision_geometry_sha256'):
                if case[key]!=getattr(model,key):
                    raise ValueError(f'Frozen manifest {key} does not match canonical model')
            result['timing']['model_setup_seconds']=time.perf_counter()-tic
            stage('dependency_probe');dependency_audit(out,method)
            stage('gpu_initialization_warmup');tic=time.perf_counter()
            endpoints=np.asarray([case['q_start'],case['q_goal']],float)
            endpoint_native=model.cuda(endpoints)
            endpoint_clearance=model.clearance(endpoints)
            result['timing']['gpu_initialization_warmup_seconds']=time.perf_counter()-tic
            valid=bool(np.isfinite(endpoints).all() and np.all(endpoints>=model.lower) and
                np.all(endpoints<=model.upper) and endpoint_native.all() and np.all(endpoint_clearance>model.contact_band_m))
            write(out/'endpoint_audit.json',dict(valid=valid,cuda_free=endpoint_native,clearance_m=endpoint_clearance,
                                                q_start=endpoints[0],q_goal=endpoints[1],endpoints_changed=False))
            if not valid:
                result['status']='invalid_endpoint';return result
            road=Path(roadmap_dir);meta=json.loads((road/'metadata.json').read_text())
            if meta['status']!='ready' or meta['native_map_sha256']!=sha(road/'native.map'):
                raise ValueError('Roadmap not ready or map hash mismatch')
            for key in ('robot_sha256','collision_geometry_sha256'):
                if meta[key]!=case[key]:
                    raise ValueError('DRM and online model mismatch')
            low=np.asarray(meta['config']['map_center'])-np.asarray(meta['config']['map_extent'])/2
            high=np.asarray(meta['config']['map_center'])+np.asarray(meta['config']['map_extent'])/2
            if len(model.voxels_xyz) and (np.any(model.voxels_xyz<low) or np.any(model.voxels_xyz>=high)):
                raise ValueError('Scene voxel centres outside frozen roadmap cache workspace')
            result['roadmap_id']=meta['roadmap_id'];result['roadmap_sha256']=meta['native_map_sha256']
            result['offline_roadmap_seconds']=meta['offline_seconds']
            stage('roadmap_load');tic=time.perf_counter()
            options=csd.DrmPlannerOptions()
            for key,value in cfg['drm'].items():setattr(options,key,value)
            planner=csd.DrmPlanner(model.csd_plant,options,True);planner.LoadRoadmap(str(road/'native.map'))
            result['timing']['roadmap_load_seconds']=time.perf_counter()-tic
            warm_started=time.perf_counter()
            stage('online_collision_map');tic=time.perf_counter();planner.BuildCollisionSet(model.voxels)
            result['timing']['online_collision_map_seconds']=time.perf_counter()-tic
            stage('drm_path');tic=time.perf_counter()
            success,path=planner.Plan(endpoints[0],endpoints[1],model.voxels,model.voxel_radius)
            result['timing']['drm_path_seconds']=time.perf_counter()-tic
            path=np.asarray(path,float);np.save(out/'path.npy',path)
            write(out/'path_provenance.json',dict(native_success=bool(success),try_shortcutting=cfg['drm']['try_shortcutting'],
                representation='Native DrmPlanner.Plan returned path after configured shortcutting',
                internal_raw_path='not exposed by the upstream Python API; no fabricated pre-shortcut path'))
            if not success:
                result['status']='frontend_failed';return result
            path=_checked_native_path(path,endpoints,cfg['endpoint_tolerance_rad'])
            result['frontend_success']=True
            candidate=None;validation=None;regions=[]
            if method=='NTO':
                stage('trajectory_optimization');tic=time.perf_counter()
                candidate=_nto(path,model,case,cfg,out,deadline)
                result['timing']['trajectory_optimization_seconds']=time.perf_counter()-tic
                _save_candidate(out,candidate)
                stage('validation');tic=time.perf_counter()
                validation=validate(candidate,model,case,cfg,out=out)
                result['timing']['validation_seconds']=time.perf_counter()-tic
                result['status']=validation['status'] if candidate['solver_success'] else 'solver_failed'
            else:
                domain=csd.HPolyhedron();domain.MakeBox(model.lower,model.upper)
                eizo=csd.EizoOptions()
                for key,value in cfg['eizo'].items():setattr(eizo,key,value)
                eizo.seed=int(seed)
                write(out/'eizo_parameters.json',dict(cfg['eizo'],seed=int(eizo.seed)))
                inflator=csd.CudaEdgeInflator(model.mplant,model.robot_geometry_ids,eizo,domain)
                stage('eizo_inflation');tic=time.perf_counter()
                for i,(a,b) in enumerate(zip(path[:-1],path[1:])):
                    stage('eizo_inflation')
                    containing=next((r for r in regions if r.PointInSet(a,1e-7) and r.PointInSet(b,1e-7)),None)
                    if containing is not None:
                        region=containing
                    else:
                        native=inflator.inflateEdge(a,b,model.voxels,model.voxel_radius,False)
                        region=HPolyhedron(native.A(),native.b())
                    region=cover_region_anchors(region,_region_anchors(path,i,endpoints),
                        cfg['region_coverage_tolerance_rad'],out/'coverage'/'inflation'/f'{i:04d}')
                    regions.append(region)
                    _save_regions(out,regions)
                result['timing']['eizo_inflation_seconds']=time.perf_counter()-tic
                result['timing']['trajectory_optimization_seconds']=0.
                result['timing']['validation_seconds']=0.
                result['timing']['collision_recovery_seconds']=0.
                for attempt in range(cfg['max_recovery_attempts']+1):
                    round_out=out/'rounds'/f'{attempt:03d}';round_out.mkdir(parents=True,exist_ok=True)
                    _save_regions(round_out,regions)
                    stage('trajectory_optimization');tic=time.perf_counter()
                    if objective=='minimum_distance':
                        from scsplanning.polygonal import get_points
                        points=np.asarray(get_points(endpoints[0],endpoints[1],regions),float)
                        candidate=dict(kind='polygon',points=points,objective=objective,solver='Clarabel',solver_success=True,
                                       solver_status='author_scsplanning_get_points_returned',provenance='installed_author_LSCS_with_Clarabel')
                    else:
                        from scsplanning import biconvex
                        velocity=HPolyhedron.MakeBox(-np.ones(7)*cfg['velocity_limit_rad_s'],np.ones(7)*cfg['velocity_limit_rad_s'])
                        acceleration=HPolyhedron.MakeBox(-np.ones(7)*cfg['acceleration_limit_rad_s2'],np.ones(7)*cfg['acceleration_limit_rad_s2'])
                        curve=biconvex(endpoints[0],endpoints[1],regions,velocity,acceleration,cfg['bezier_degree'],cfg['scsto_relative_tolerance'])
                        candidate=dict(kind='bezier',controls=np.asarray([c.points for c in curve.curves],float),
                            durations=np.asarray([c.duration for c in curve.curves],float),objective=objective,
                            solver='Clarabel',solver_success=True,solver_status='author_scsplanning_biconvex_returned',
                            provenance='installed_author_SCSTO_with_Clarabel')
                    result['timing']['trajectory_optimization_seconds']+=time.perf_counter()-tic
                    _save_candidate(round_out,candidate);_save_candidate(out,candidate)
                    stage('validation');tic=time.perf_counter()
                    validation=validate(candidate,model,case,cfg,regions=regions,out=round_out)
                    result['timing']['validation_seconds']+=time.perf_counter()-tic
                    write(out/'validation.json',{k:v for k,v in validation.items() if k!='_samples'})
                    np.savez_compressed(out/'trajectory_samples.npz',**validation['_samples'])
                    result['status']=validation['status'];result['metrics']['recovery_attempts']=attempt
                    if validation['passed'] or validation['status']!='collision_failed' or attempt==cfg['max_recovery_attempts']:
                        break
                    stage('collision_recovery');tic=time.perf_counter()
                    points=validation['collision_points'];assignments=validation['collision_regions']
                    ids=np.unique(np.linspace(0,len(points)-1,min(len(points),cfg['maximum_recovery_witnesses']),dtype=int))
                    edit=csd.EditRegionsOptions();edit.configuration_margin=.01
                    native_regions=[]
                    for r in regions:
                        native=csd.HPolyhedron(r.A(),r.b());native_regions.append(native)
                    corrected=csd.EditRegionsCuda(points[ids].T,assignments[ids].tolist(),path[:-1].T,path[1:].T,
                        native_regions,model.mplant,model.robot_geometry_ids,model.voxels,model.voxel_radius,edit)
                    regions=[HPolyhedron(r.A(),r.b()) for r in corrected[0]]
                    for i,(a,b) in enumerate(zip(path[:-1],path[1:])):
                        regions[i]=cover_region_anchors(regions[i],_region_anchors(path,i,endpoints),
                            cfg['region_coverage_tolerance_rad'],
                            out/'coverage'/f'recovery_{attempt:03d}'/'edited'/f'{i:04d}')
                        if not regions[i].PointInSet(a,1e-6) or not regions[i].PointInSet(b,1e-6):
                            native=inflator.inflateEdge(a,b,model.voxels,model.voxel_radius,False)
                            regions[i]=cover_region_anchors(HPolyhedron(native.A(),native.b()),
                                _region_anchors(path,i,endpoints),cfg['region_coverage_tolerance_rad'],
                                out/'coverage'/f'recovery_{attempt:03d}'/'reinflated'/f'{i:04d}')
                    _save_regions(out,regions)
                    result['timing']['collision_recovery_seconds']+=time.perf_counter()-tic
            if validation is not None:
                result['metrics'].update({k:validation[k] for k in ('cost','cost_unit','path_length_rad','normalized_path_length','minimum_sampled_clearance_m')})
                result['metrics']['duration_s']=validation['duration_s']
            result['metrics'].update(regions=len(regions),facets=sum(len(r.b()) for r in regions))
            if time.perf_counter()>=deadline:result['status']='timeout'
        except Exception as exc:
            result['exception']=repr(exc);result['failed_stage']=phase
            (out/'exception.txt').write_text(traceback.format_exc())
            if isinstance(exc,TimeoutError):result['status']='timeout'
            elif 'cuda' in str(exc).lower():result['status']='infrastructure_error';result['error_category']='CUDA_error'
            elif phase=='input_contract':
                result['status']='infrastructure_error';result['error_category']='input_protocol_mismatch'
            elif phase=='dependency_probe':
                result['status']='infrastructure_error'
                result['error_category']='dependency_unsupported' if isinstance(exc,(ImportError,ModuleNotFoundError)) else 'dependency_probe_failed'
            elif phase=='drm_path':result['status']='frontend_failed'
            elif phase in ('trajectory_optimization','collision_recovery','eizo_inflation'):result['status']='solver_failed'
            elif phase=='validation':result['status']='validation_undetermined'
            else:result['status']='infrastructure_error'
        finally:
            result['timing']['cold_e2e_seconds']=time.perf_counter()-start
            if time.perf_counter()>=deadline:
                result['status']='timeout'
            result['timing']['warm_query_seconds']=time.perf_counter()-warm_started if warm_started is not None else None
            result['timing_scope']=dict(cold='worker function entry through final validation/failure; process startup measured separately by outer runner',
                                       warm='same actual cold execution minus one-time model/dependency/GPU/roadmap setup; not a second timed solve',
                                       offline='DRM construction is recorded separately and never added to query latency')
            result['gpu_memory']=memory.report()
            result['final_stage']=phase
            write(out/'result.json',result)
    return result
