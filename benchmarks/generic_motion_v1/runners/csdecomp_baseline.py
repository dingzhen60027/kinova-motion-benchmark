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
from .native_handoff import NativeHandoffError, native_region_with_anchors
from .methods import METHODS, INDEPENDENT, LOCAL_EXTENSIONS, objectives as supported_objectives
from .failure_classification import (HANDOFF_KIND, HANDOFF_SOURCE, KNOWN_DRM_NO_ROUTE,
    KNOWN_VALIDATION_REJECTIONS, handoff_failure_evidence, known_exception_category)



def _exception_outcome(exc, phase, *, out=None, config=None):
    """Classify failures without changing search, optimization or validation."""
    if 'cuda' in str(exc).lower():
        return dict(status='infrastructure_error', error_category='CUDA_error')
    if getattr(exc, 'dependency_unsupported', False):
        return dict(status='unsupported', error_category='dependency_unsupported')
    if type(exc) is TimeoutError:
        timeout = dict(status='timeout', exception=repr(exc), failed_stage=phase)
        if known_exception_category(timeout) == 'query_deadline':
            return dict(status='timeout')
        return dict(status='infrastructure_error', error_category='unexpected_timeout_exception')
    if phase in ('input_contract', 'roadmap_contract'):
        return dict(status='infrastructure_error', error_category='input_protocol_mismatch')
    if phase == 'dependency_probe':
        missing = isinstance(exc, (ImportError, ModuleNotFoundError))
        return dict(status='unsupported' if missing else 'infrastructure_error',
                    error_category='dependency_unsupported' if missing else 'dependency_probe_failed')
    if phase == 'sampling_planning':
        from .ompl_baselines import OmplNoExactSolution
        if type(exc) is OmplNoExactSolution:
            if exc.solver_status in ('Approximate solution', 'Timeout'):
                return dict(native_planner_status=exc.solver_status, status='planner_timeout')
            return dict(native_planner_status=exc.solver_status,
                        status='infrastructure_error', error_category='unexpected_native_planner_status')
    if phase == 'drm_path':
        if type(exc) is RuntimeError and str(exc) == KNOWN_DRM_NO_ROUTE:
            return dict(status='frontend_failed')
        return dict(status='infrastructure_error', error_category='unexpected_frontend_exception')
    if (phase in ('drm_path_validation', 'validation') and type(exc) is ValueError
            and str(exc) in KNOWN_VALIDATION_REJECTIONS):
        return dict(status='validation_undetermined')
    if type(exc) is NativeHandoffError and phase == 'collision_recovery' and out is not None:
        try:
            evidence = Path(exc.evidence_path).resolve().relative_to(Path(out).resolve()).as_posix()
            source_sha = sha(ROOT/HANDOFF_SOURCE)
            marker = dict(kind=HANDOFF_KIND, code=exc.code, source=HANDOFF_SOURCE,
                          source_sha256=source_sha, evidence=evidence)
            record = dict(status='solver_failed', exception=repr(exc), failed_stage=phase,
                          failure_classification=marker)
            if handoff_failure_evidence(out, record,
                    expected_tolerance=(config or {}).get('region_coverage_tolerance_rad'),
                    source_hashes={HANDOFF_SOURCE:source_sha}):
                return dict(status='solver_failed', error_category=HANDOFF_KIND, failure_classification=marker)
        except (OSError, TypeError, ValueError):
            pass  # Missing or inconsistent numerical evidence is not exempted.
    return dict(status='infrastructure_error', error_category='unexpected_backend_exception')


def _apply_deadline_outcome(result, deadline_exceeded):
    """Record deadline expiry without hiding an already observed backend error."""
    if deadline_exceeded:
        result['deadline_exceeded'] = True
        if result.get('status') not in ('infrastructure_error', 'unsupported', 'model_consistency_failed'):
            result['status'] = 'timeout'


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
                                 order='active ordered corridor, start to goal; original DRM edge mapping in corridor.json'))


class RegionCoverageError(ValueError):
    """A cut removed a required path anchor beyond numerical tolerance."""


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
        raise RegionCoverageError(f'Region requires normalized endpoint correction {largest:.9g}, '
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


def _corridor_anchors(path,entry,endpoints):
    indices=entry['covered_edge_indices']
    vertices=list(range(indices[0],indices[-1]+2))
    anchors=[path[i] for i in vertices]
    if indices[0]==0:anchors.append(endpoints[0])
    if indices[-1]==len(path)-2:anchors.append(endpoints[1])
    return np.asarray(anchors,float)


def _check_corridor(regions,mapping,path,endpoints):
    """Verify an ordered partition of the original path and every shared point."""
    if len(regions)!=len(mapping) or not regions:
        raise ValueError('Active corridor region/mapping count mismatch')
    flat=[];connections=[]
    for i,(region,entry) in enumerate(zip(regions,mapping)):
        edges=entry['covered_edge_indices']
        if (not edges or edges!=list(range(edges[0],edges[-1]+1))
                or entry['original_seed_edge_index'] not in edges):
            raise ValueError('Malformed original-edge corridor mapping')
        flat.extend(edges)
        if not all(region.PointInSet(q,1e-7) for q in _corridor_anchors(path,entry,endpoints)):
            raise RegionCoverageError('Active region lost a required original path or canonical endpoint anchor')
        if i:
            shared=edges[0]
            if mapping[i-1]['covered_edge_indices'][-1]+1!=shared:
                raise ValueError('Adjacent active regions have no common original waypoint')
            if not regions[i-1].PointInSet(path[shared],1e-7):
                raise RegionCoverageError('Adjacent active regions lost their shared original waypoint')
            connections.append(dict(left=i-1,right=i,original_waypoint_index=shared,q=path[shared]))
    if flat!=list(range(len(path)-1)):
        raise ValueError('Active corridor does not partition all fixed DRM edges exactly once')
    return connections


def _save_corridor(out,mapping,path,endpoints,events,complete=True):
    if out is None:return
    write(Path(out)/'corridor.json',dict(
        protocol='consecutive_active_corridor_v1',complete=complete,
        original_edge_count=len(path)-1,active_region_count=len(mapping),
        reuse_policy='Reuse only the last active region for consecutive covered original edges; never globally deduplicate a returning path',
        collision_region_indices='Zero-based indices into this active region sequence',
        regions=[dict(entry,active_region_index=i,anchors=_corridor_anchors(path,entry,endpoints))
                 for i,entry in enumerate(mapping)],events=events))


def build_active_corridor(path,endpoints,inflate,tolerance,out=None,on_edge=None):
    """Retain one optimization segment per active region, not per DRM edge."""
    regions=[];mapping=[];events=[]
    if out is not None:out=Path(out);out.mkdir(parents=True,exist_ok=True)
    for index,(a,b) in enumerate(zip(path[:-1],path[1:])):
        if on_edge is not None:on_edge()
        reused=bool(regions and regions[-1].PointInSet(a,1e-7) and regions[-1].PointInSet(b,1e-7))
        if reused:
            mapping[-1]['covered_edge_indices'].append(index)
            region=regions[-1]
        else:
            region=inflate(a,b)
            mapping.append(dict(original_seed_edge_index=index,covered_edge_indices=[index]))
            regions.append(region)
        active=len(regions)-1
        target=None if out is None else out/'coverage'/'inflation'/f'{index:04d}'
        regions[-1]=cover_region_anchors(region,_corridor_anchors(path,mapping[-1],endpoints),tolerance,target)
        events.append(dict(original_edge_index=index,active_region_index=active,
                           action='reuse_last_region' if reused else 'native_inflate_original_edge'))
        if out is not None:_save_regions(out,regions)
        _save_corridor(out,mapping,path,endpoints,events,complete=False)
    connections=_check_corridor(regions,mapping,path,endpoints)
    events.append(dict(action='verify_shared_waypoints',connections=connections))
    _save_corridor(out,mapping,path,endpoints,events)
    return regions,mapping


def recover_active_corridor(edited,mapping,path,endpoints,inflate,tolerance,out=None,on_edge=None):
    """Preserve valid cuts; rebuild only spans whose original anchors were cut.

    Every replacement uses an original DRM edge. No unverified chord or moved
    endpoint becomes a seed. A rebuilt span has one region per original edge
    until a later experiment explicitly implements another reuse policy.
    """
    if len(edited)!=len(mapping):raise ValueError('Native editor changed active region count')
    if out is not None:
        out=Path(out);out.mkdir(parents=True,exist_ok=True)
        _save_regions(out,edited)
        _save_corridor(out,mapping,path,endpoints,[dict(action='native_edited_before_anchor_check')])
    regions=[];updated=[];events=[]
    for old_index,(region,entry) in enumerate(zip(edited,mapping)):
        target=None if out is None else out/'anchor_checks'/f'{old_index:04d}'
        try:
            corrected=cover_region_anchors(region,_corridor_anchors(path,entry,endpoints),tolerance,target)
        except RegionCoverageError as exc:
            event=dict(action='reinflate_original_span',previous_active_region_index=old_index,
                original_edge_indices=entry['covered_edge_indices'],reason=str(exc),replacement_active_indices=[])
            events.append(event)
            if out is not None:write(out/'recovery_events.json',dict(complete=False,events=events))
            for edge in entry['covered_edge_indices']:
                if on_edge is not None:on_edge()
                replacement=inflate(path[edge],path[edge+1])
                new_entry=dict(original_seed_edge_index=edge,covered_edge_indices=[edge])
                target=None if out is None else out/'reinflated'/f'{edge:04d}'
                replacement=cover_region_anchors(replacement,_corridor_anchors(path,new_entry,endpoints),tolerance,target)
                event['replacement_active_indices'].append(len(regions))
                regions.append(replacement);updated.append(new_entry)
                if out is not None:write(out/'recovery_events.json',dict(complete=False,events=events))
        else:
            events.append(dict(action='retain_edited_region',previous_active_region_index=old_index,
                               active_region_index=len(regions)))
            regions.append(corrected);updated.append(dict(entry,covered_edge_indices=list(entry['covered_edge_indices'])))
    connections=_check_corridor(regions,updated,path,endpoints)
    events.append(dict(action='verify_shared_waypoints',connections=connections))
    if out is not None:
        after=out/'accepted';after.mkdir(parents=True,exist_ok=True);_save_regions(after,regions)
        _save_corridor(after,updated,path,endpoints,events)
        write(out/'recovery_events.json',dict(complete=True,events=events))
    return regions,updated,events


def dependency_audit(out,method='EI_ZO',objective='minimum_distance',config=None):
    """Inspect installed actual optimizer code and execute an open-source solve."""
    if method in INDEPENDENT:
        from .ompl_baselines import dependency_probe
        return dependency_probe(out)
    if method == 'STOMP':
        from .chomp_stomp import dependency_probe
        return dependency_probe(out)
    if method == 'CHOMP':
        from .chomp_stomp import acceleration_metric
        _, metric, inverse = acceleration_metric(8)
        error = float(np.max(np.abs(metric @ inverse-np.eye(6))))
        record = dict(method=method, probe_success=error < 1e-9, inverse_residual=error,
                      implementation='Covariant descent with canonical-geometry C-space line-integral cost')
        write(out/'chomp_dependencies.json',record)
        if not record['probe_success']:
            raise RuntimeError('CHOMP metric probe failed')
        return record
    if method == 'TRAJOPT':
        from .trajopt_baseline import dependency_probe, settings, DEFAULT_BUILD
        record = dependency_probe(settings(config or {}).get('build_dir', DEFAULT_BUILD))
        write(out/'trajopt_dependencies.json',record)
        if not record.get('available'):
            raise ImportError('TrajOpt author core unavailable: '+str(record.get('status')))
        return record
    if method in ('NTO_PAPER', 'NTO_REFINED'):
        from .paper_nto import dependency_probe
        record = dependency_probe(out, objective)
        if method == 'NTO_REFINED':
            from .paper_nto_refined import PROFILE
            record.update(method=method, profile=PROFILE['name'],
                fidelity='Drake NTO with published solvers/orders and explicit positive-clearance collocation refinement')
            write(out/'paper_nto_dependencies.json',record)
        return record
    if method == 'LGCS':
        from .paper_lgcs import dependency_probe
        return dependency_probe(out, backend=(config or {}).get('lgcs', {}).get('solver', 'gurobi'))
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
        KinematicTrajectoryOptimization,IpoptSolver)
    from .nto_collision_constraint import RobustMinimumDistanceConstraint, NativeFirstCollisionSolve
    from .paper_nto import _save_result
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
    collision_backend=RobustMinimumDistanceConstraint(native,options['clearance_m'],options['influence_distance_m'],out)
    locations=np.arange(1,options['collocation_points']+1)/(options['collocation_points']+1)
    collision_solve=NativeFirstCollisionSolve(traj,collision_backend,locations)
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
          collision_backend='native_first_Drake_exact_issue21675_guarded_retry',
          collision_backend_evidence='nto_collision_backend.json',
          transcription='Drake KTO + Ipopt adaptation; fixed DRM initialization; no success filtering',
          construction_seconds=time.perf_counter()-start))
    try:
        result=collision_solve.solve(solver,min(deadline,time.perf_counter()+remaining),
                                    timeout_option='max_cpu_time',log_path=parameters['output_file'])
    finally:
        collision_backend.save()
    controls,duration,normalized_knots,status=_save_result(result,traj,out)
    with np.errstate(invalid='ignore',over='ignore'):
        knots=normalized_knots*duration
    return dict(kind='bspline',objective=case['_objective'],solver='IPOPT',solver_success=bool(result.is_success()),
                solver_status=str(result.get_solution_result()),spline_order=order,
                controls=controls,knots=knots,duration=duration,provenance='adapted_Drake_KTO_NTO')


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


def _native_option_values(options):
    return {name:getattr(options,name) for name in dir(options)
            if not name.startswith('_') and isinstance(getattr(options,name),(bool,int,float,str))}


def _checked_native_path(path,endpoints,tolerance):
    values=np.asarray(path,float)
    if values.ndim!=2 or values.shape[1]!=7 or len(values)<2 or not np.isfinite(values).all():
        raise ValueError('Native DRM returned a malformed or nonfinite Nx7 path')
    if not np.allclose(values[[0,-1]],endpoints,atol=tolerance,rtol=0):
        raise ValueError('Native DRM path does not connect the fixed requested endpoints')
    return values


def _validate_frontend_path(path,model,case,cfg,out):
    # This is a geometric seed-path check for both objectives. It does not
    # reinterpret a timed candidate as PWL or change the requested solve task.
    seed_case=dict(case,_objective='minimum_distance')
    candidate=dict(kind='polygon',points=path,objective='minimum_distance')
    return validate(candidate,model,seed_case,cfg,out=out)


def plan_query(planner,model,endpoints,cfg,out):
    """Use the explicitly selected native host entry with unchanged CUDA."""
    selected=cfg.get('drm_query')
    if selected is None:
        return planner.Plan(endpoints[0],endpoints[1],model.voxels,model.voxel_radius)
    from .native_drm_query import query
    build_dir=Path(selected['build_dir'])
    if not build_dir.is_absolute():
        build_dir=ROOT/build_dir
    success,path,evidence=query(planner,endpoints[0],endpoints[1],model.voxels_xyz,
        model.voxel_radius,build_dir=build_dir,out=Path(out))
    return success,path


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


def run_case(case,roadmap_dir,out,objective='minimum_distance',seed=0,method='EI_ZO',config=None,*,shared_initialization=None):
    """Run a fixed query, saving every stage; caller enforces process watchdog.

    ``online_budget_seconds`` is checked between calls. A native kernel/solver
    cannot be interrupted safely in-process: the CLI worker must have the same
    total wall-clock subprocess deadline and retain these incremental files.
    """
    start=time.perf_counter();out=Path(out);out.mkdir(parents=True,exist_ok=True)
    if objective not in ('minimum_distance','minimum_time') or method not in METHODS:
        raise ValueError('Unknown objective or method')
    if objective not in supported_objectives(method):
        raise ValueError(f'{method} implements minimum_distance only')
    if shared_initialization is not None and (method not in ('EI_ZO','LGCS') or objective!='minimum_distance'):
        raise ValueError('Fixed initial regions support EI_ZO/LGCS path backends only')
    if (out/'result.json').exists():
        raise FileExistsError('Refusing to overwrite an existing run result')
    try:
        cfg=load_config(config)
        seed=_method_seed(seed)
    except (ValueError,TypeError,OverflowError) as exc:
        result=dict(status='infrastructure_error',error_category='input_protocol_mismatch',
            failed_stage='input_contract',final_stage='input_contract',exception=repr(exc),
            problem_id=case.get('problem_id'),scene_id=case.get('scene_id'),family=case.get('family'),
            objective=objective,method=method,roadmap_used=method not in INDEPENDENT,
            frontend_success=None if method in INDEPENDENT else False,
            frontend_native_success=None if method in INDEPENDENT else False,metrics={},
            timing={'cold_e2e_seconds':time.perf_counter()-start,'warm_query_seconds':None})
        write(out/'input.json',case);write(out/'result.json',result)
        (out/'exception.txt').write_text(traceback.format_exc())
        return result
    deadline=start+cfg['online_budget_seconds']
    case=dict(case);case['_objective']=objective
    result=dict(status='running',problem_id=case['problem_id'],scene_id=case['scene_id'],family=case['family'],
        objective=objective,method=method,method_seed=int(seed),roadmap_id=None,frontend_success=False,
        frontend_native_success=False, roadmap_used=method not in INDEPENDENT,
        source_code_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        source_sha256={name:sha(path) for name,path in [('runner',__file__),('validator',Path(__file__).with_name('validate.py'))]},
        timing={},metrics={},budget_seconds=cfg['online_budget_seconds'],
        reproduction_fidelity={
            'EI_ZO':'Kinova adaptation; native EI-ZO and author LSCS/SCSTO using Clarabel',
            'LGCS':'Kinova; native EI-ZO and perspective GCS MISOCP using '+cfg.get('lgcs',{}).get('solver','gurobi'),
            'NTO':'Kinova adaptation; Drake KTO/IPOPT with locally strengthened initialization',
            'NTO_PAPER':'Kinova adaptation; reconstructed Drake KTO using published SNOPT distance/IPOPT time profile',
            'NTO_REFINED':'Kinova; Drake KTO with SNOPT distance/IPOPT time, positive clearance and collision collocation refinement',
            'RRTCONNECT':'Official native OMPL RRTConnect with canonical CSD state/motion callbacks',
            'RRTSTAR':'Official native OMPL RRTstar with canonical CSD state/motion callbacks',
            'CHOMP':'Covariant CHOMP descent with canonical-geometry C-space line-integral obstacle cost',
            'STOMP':'Unchanged official STOMP core with canonical signed-distance task cost',
            'TRAJOPT':'Author TrajOpt trust-region SQP and BPMPD core with canonical signed-distance collocation'
        }[method]+'; not exact author FR3/hardware reproduction',
        seed_semantics=('method_seed seeds native OMPL sampling; roadmap identity only pairs the repetition' if method in INDEPENDENT
                        else 'method_seed seeds STOMP correlated PCG64 rollouts; deterministic fixed DRM initialization' if method == 'STOMP'
                        else 'deterministic optimizer from fixed validated DRM path; method_seed retained as pairing identity' if method in LOCAL_EXTENSIONS
                        else 'EI-ZO EizoOptions.seed is passed to native cuRAND; DRM graph sampling uses separate saved PCG64 seed; optimizer initialization is deterministic fixed DRM path'))
    if shared_initialization is not None:
        result.update(execution_mode='fixed_initial_regions',
            initialization_digest=shared_initialization['initialization_digest'],
            source_initialization=shared_initialization,
            initialization_generation_cost='Separate source-run preparation; not part of this conditioned backend clock',
            initialization_scope='Identical saved first corridor at backend handoff; normal EI-ZO repair and LGCS alternate-route/pool expansion/repair remain enabled and separately saved')
    write(out/'input.json',case);write(out/'evaluation.json',cfg);write(out/'result.json',result)
    if method in INDEPENDENT:
        result.update(frontend_success=None, frontend_native_success=None,
                      offline_roadmap_seconds=None,
                      initialization='Fixed endpoints; independent tree search; roadmap identifier is repetition pairing only')
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
            stage('dependency_probe');dependency_audit(out,method,objective,config=cfg)
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
            if method in INDEPENDENT:
                from .ompl_baselines import solve
                warm_started=time.perf_counter()
                stage('sampling_planning');tic=time.perf_counter()
                try:
                    candidate=solve(model,case,cfg,out,deadline,seed,method)
                finally:
                    result['timing']['sampling_planning_seconds']=time.perf_counter()-tic
                _save_candidate(out,candidate)
                stage('validation');tic=time.perf_counter()
                validation=validate(candidate,model,case,cfg,out=out)
                result['timing']['validation_seconds']=time.perf_counter()-tic
                result['status']=validation['status'] if candidate['solver_success'] else 'solver_failed'
                result['metrics'].update({k:validation[k] for k in ('cost','cost_unit','path_length_rad','normalized_path_length','minimum_sampled_clearance_m','duration_s')})
                result['metrics'].update(regions=0,facets=0)
                return result
            stage('roadmap_contract')
            road=Path(roadmap_dir);meta=json.loads((road/'metadata.json').read_text())
            if meta['status']!='ready' or meta['native_map_sha256']!=sha(road/'native.map'):
                raise ValueError('Roadmap not ready or map hash mismatch')
            for key in ('robot_sha256','collision_geometry_sha256'):
                if meta[key]!=case[key]:
                    raise ValueError('DRM and online model mismatch')
            if meta.get('config')!=cfg['roadmap']:
                raise ValueError('Frozen roadmap build configuration differs from requested evaluation profile')
            low=np.asarray(meta['config']['map_center'])-np.asarray(meta['config']['map_extent'])/2
            high=np.asarray(meta['config']['map_center'])+np.asarray(meta['config']['map_extent'])/2
            if len(model.voxels_xyz) and (np.any(model.voxels_xyz<low) or np.any(model.voxels_xyz>=high)):
                raise ValueError('Scene voxel centres outside frozen roadmap cache workspace')
            result['roadmap_id']=meta['roadmap_id'];result['roadmap_sha256']=meta['native_map_sha256']
            result['offline_roadmap_seconds']=meta['offline_seconds']
            shared_regions=shared_mapping=None
            if shared_initialization is not None:
                if shared_initialization.get('roadmap_native_sha256')!=meta['native_map_sha256']:
                    raise ValueError('Shared initialization and target native roadmap content differ')
                warm_started=time.perf_counter()
                stage('shared_initialization_load');tic=time.perf_counter()
                from .shared_initialization import load_bound
                path,shared_regions,shared_mapping=load_bound(shared_initialization,case,cfg,
                    seed,meta['roadmap_id'],out)
                result['timing']['shared_initialization_load_seconds']=time.perf_counter()-tic
                write(out/'path_provenance.json',dict(native_success=True,
                    representation='Exact saved native DRM path from the bound first-terminal EI_ZO source',
                    initialization_digest=shared_initialization['initialization_digest'],
                    source_attempt=shared_initialization['source_attempt']))
            else:
                stage('roadmap_load');tic=time.perf_counter()
                options=csd.DrmPlannerOptions()
                for key,value in cfg['drm'].items():setattr(options,key,value)
                planner=csd.DrmPlanner(model.csd_plant,options,True);planner.LoadRoadmap(str(road/'native.map'))
                result['timing']['roadmap_load_seconds']=time.perf_counter()-tic
                warm_started=time.perf_counter()
                stage('online_collision_map');tic=time.perf_counter();planner.BuildCollisionSet(model.voxels)
                result['timing']['online_collision_map_seconds']=time.perf_counter()-tic
                stage('drm_path');tic=time.perf_counter()
                success,path=plan_query(planner,model,endpoints,cfg,out)
                result['timing']['drm_path_seconds']=time.perf_counter()-tic
                path=np.asarray(path,float);np.save(out/'path.npy',path)
                write(out/'path_provenance.json',dict(native_success=bool(success),try_shortcutting=cfg['drm']['try_shortcutting'],
                    representation='Native DrmPlanner.Plan returned path after configured shortcutting',
                    internal_raw_path='not exposed by the upstream Python API; no fabricated pre-shortcut path'))
                if not success:
                    result['status']='frontend_failed';return result
            result['frontend_native_success']=True
            path=_checked_native_path(path,endpoints,cfg['endpoint_tolerance_rad'])
            stage('drm_path_validation');tic=time.perf_counter()
            seed_validation=_validate_frontend_path(path,model,case,cfg,out/'frontend_validation')
            result['timing']['drm_path_validation_seconds']=time.perf_counter()-tic
            result['frontend_validation_status']=seed_validation['status']
            if not seed_validation['passed']:
                result['failed_stage']='drm_path_validation'
                result['status']='validation_undetermined' if seed_validation['status']=='validation_undetermined' else 'frontend_failed'
                result['error']='Native DRM reported success, but its seed path failed the common numerical geometry check; no EI-ZO or optimizer call made'
                return result
            result['frontend_success']=True
            candidate=None;validation=None;regions=[]
            if method in LOCAL_EXTENSIONS or method in ('NTO','NTO_PAPER','NTO_REFINED'):
                stage('trajectory_optimization');tic=time.perf_counter()
                if method in ('CHOMP','STOMP'):
                    from .chomp_stomp import solve
                    candidate=solve(path,model,case,cfg,out,deadline-5.,seed,method)
                elif method == 'TRAJOPT':
                    from .trajopt_baseline import solve
                    candidate=solve(path,model,case,cfg,out,deadline-5.,seed)
                elif method == 'NTO_REFINED':
                    from .paper_nto_refined import paper_nto_refined
                    candidate=paper_nto_refined(path,model,case,cfg,out,deadline)
                elif method == 'NTO_PAPER':
                    from .paper_nto import paper_nto
                    candidate=paper_nto(path,model,case,cfg,out,deadline)
                else:
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
                native_defaults=_native_option_values(eizo)
                for key,value in cfg['eizo'].items():setattr(eizo,key,value)
                eizo.seed=int(seed)
                write(out/'eizo_parameters.json',dict(cfg['eizo'],seed=int(eizo.seed),
                    native_defaults=native_defaults,native_effective_options=_native_option_values(eizo),
                    parameter_source='Explicit frozen evaluation eizo fields plus method seed override; all other installed native defaults retained unchanged'))
                inflator=csd.CudaEdgeInflator(model.mplant,model.robot_geometry_ids,eizo,domain)
                def inflate_original_edge(a,b):
                    native=inflator.inflateEdge(a,b,model.voxels,model.voxel_radius,False)
                    return HPolyhedron(native.A(),native.b())
                if method == 'LGCS':
                    from .paper_graph import run_graph
                    candidate,validation,regions=run_graph(model,case,cfg,out,path,endpoints,road,seed,
                        inflate_original_edge,stage,result,deadline,
                        initial_corridor=(shared_regions,shared_mapping) if shared_regions is not None else None)
                else:
                    stage('eizo_inflation');tic=time.perf_counter()
                    if shared_regions is None:
                        regions,corridor=build_active_corridor(path,endpoints,inflate_original_edge,
                            cfg['region_coverage_tolerance_rad'],out,on_edge=lambda:stage('eizo_inflation'))
                    else:
                        regions,corridor=list(shared_regions),list(shared_mapping)
                        _check_corridor(regions,corridor,path,endpoints)
                    result['timing']['eizo_inflation_seconds']=time.perf_counter()-tic
                    result['timing']['trajectory_optimization_seconds']=0.
                    result['timing']['validation_seconds']=0.
                    result['timing']['collision_recovery_seconds']=0.
                    for attempt in range(cfg['max_recovery_attempts']+1):
                        round_out=out/'rounds'/f'{attempt:03d}';round_out.mkdir(parents=True,exist_ok=True)
                        _save_regions(round_out,regions)
                        _save_corridor(round_out,corridor,path,endpoints,[dict(action='optimization_input')])
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
                        recovery_out=out/'coverage'/f'recovery_{attempt:03d}'
                        recovery_out.mkdir(parents=True,exist_ok=True)
                        write(recovery_out/'edit_parameters.json',dict(
                            native_effective_options=_native_option_values(edit),
                            parameter_source='Installed EditRegionsOptions defaults with explicit configuration_margin=0.01, matching upstream RegionCorrector step_back'))
                        seeds=np.asarray([entry['original_seed_edge_index'] for entry in corridor],int)
                        np.savez_compressed(recovery_out/'native_edit_inputs.npz',
                            original_seed_edge_indices=seeds,edge_start_points=path[seeds],edge_end_points=path[seeds+1],
                            collision_points=points[ids],active_region_assignments=assignments[ids])
                        native_regions=[]
                        for region_index,(r,entry) in enumerate(zip(regions,corridor)):
                            native=native_region_with_anchors(r,_corridor_anchors(path,entry,endpoints),
                                cfg['region_coverage_tolerance_rad'],recovery_out/'native_handoff'/f'{region_index:04d}')
                            native_regions.append(native)
                        corrected=csd.EditRegionsCuda(points[ids].T,assignments[ids].tolist(),path[seeds].T,path[seeds+1].T,
                            native_regions,model.mplant,model.robot_geometry_ids,model.voxels,model.voxel_radius,edit)
                        np.savez_compressed(recovery_out/'native_edit_debug.npz',
                            projections=np.asarray(corrected[1][0]),optimized_collisions=np.asarray(corrected[1][1]))
                        regions,corridor,recovery_events=recover_active_corridor(
                            [HPolyhedron(r.A(),r.b()) for r in corrected[0]],corridor,path,endpoints,inflate_original_edge,
                            cfg['region_coverage_tolerance_rad'],recovery_out,on_edge=lambda:stage('collision_recovery'))
                        _save_regions(out,regions)
                        _save_corridor(out,corridor,path,endpoints,recovery_events)
                        result['timing']['collision_recovery_seconds']+=time.perf_counter()-tic
            if validation is not None:
                result['metrics'].update({k:validation[k] for k in ('cost','cost_unit','path_length_rad','normalized_path_length','minimum_sampled_clearance_m')})
                result['metrics']['duration_s']=validation['duration_s']
            result['metrics'].update(regions=len(regions),facets=sum(len(r.b()) for r in regions))
            if time.perf_counter()>=deadline:result['status']='timeout'
        except Exception as exc:
            result['exception']=repr(exc);result['failed_stage']=phase
            (out/'exception.txt').write_text(traceback.format_exc())
            if hasattr(exc, 'optimization_metadata'):
                write(out/'optimizer_failure_metadata.json',exc.optimization_metadata)
            result.update(_exception_outcome(exc,phase,out=out,config=cfg))
        finally:
            result['timing']['cold_e2e_seconds']=time.perf_counter()-start
            _apply_deadline_outcome(result,time.perf_counter()>=deadline)
            result['timing']['warm_query_seconds']=time.perf_counter()-warm_started if warm_started is not None else None
            result['timing_scope']=dict(cold='worker function entry through final validation/failure; process startup measured separately by outer runner',
                                       warm='same actual cold execution minus one-time model/dependency/GPU/roadmap setup; not a second timed solve',
                                       offline='DRM construction is recorded separately and never added to query latency')
            result['gpu_memory']=memory.report()
            result['final_stage']=phase
            write(out/'result.json',result)
    return result
