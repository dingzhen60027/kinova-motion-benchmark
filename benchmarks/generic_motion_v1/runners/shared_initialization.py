"""Byte-bound initial paths/regions for a separate paired backend comparison.

The source optimizer outcome is irrelevant. Only the original input, validated
DRM path and first complete region corridor define this conditional cohort.
"""
from pathlib import Path
import hashlib
import json
import shutil
import numpy as np
from ..common import ROOT, read, sha, resolve_case, recorded_path

MODE = 'fixed_initial_regions'
FILES = ('request.json', 'input.json', 'evaluation.json', 'result.json', 'process.json', 'path.npy',
         'frontend_validation/validation.json', 'frontend_validation/trajectory_samples.npz',
         'rounds/000/regions.npz', 'rounds/000/corridor.json', 'eizo_parameters.json')


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                    allow_nan=False).encode()).hexdigest()


def path(value):
    return recorded_path(value)


def reference(value):
    p = Path(value).resolve()
    return str(p.relative_to(ROOT)) if p.is_relative_to(ROOT) else str(p)


def object_json(p):
    value = read(p)
    if not isinstance(value, dict):
        raise ValueError(f'Expected object in {p}')
    return value


def terminal_attempt(run, job):
    for p in sorted((Path(run)/'jobs'/job['job_id']/'attempts').glob('[0-9][0-9][0-9][0-9]')):
        if (p/'receipt.json').is_file():
            receipt = object_json(p/'receipt.json')
            if receipt.get('retry_pending', False) is False:
                return p, receipt
    return None, None


def inspect_source(attempt, case, cfg, *, expected_seed, expected_roadmap):
    """Check source evidence without requiring its final candidate to succeed."""
    attempt = Path(attempt).resolve()
    receipt = object_json(attempt/'receipt.json')
    if receipt.get('retry_pending', False) is not False or not isinstance(receipt.get('files'), dict):
        raise ValueError('Source is not a sealed terminal attempt')
    files = receipt['files']
    # A missing source stage is honest unavailable coverage. An altered sealed
    # stage is an integrity failure, never a reason to silently remove a query.
    for name in FILES:
        if name in files and (not (attempt/name).is_file() or sha(attempt/name) != files[name]):
            raise ValueError('Source artifact changed: '+name)
    metadata = ('request.json', 'input.json', 'evaluation.json')
    if any(name not in files for name in metadata):
        raise ValueError('Source lacks sealed task/configuration identity')
    request = object_json(attempt/'request.json')
    original = object_json(attempt/'input.json'); original.pop('_objective', None)
    if (digest(original) != digest(case) or request.get('case_id') != case['problem_id']
            or request.get('method') != 'EI_ZO' or request.get('objective') != 'minimum_distance'
            or request.get('execution_mode','end_to_end') != 'end_to_end'
            or request.get('method_seed') != expected_seed or request.get('roadmap_id') != expected_roadmap
            or object_json(attempt/'evaluation.json') != cfg):
        raise ValueError('Source task, model, seed, objective or configuration differs')
    if not {'path.npy','frontend_validation/validation.json','frontend_validation/trajectory_samples.npz'} <= files.keys():
        return None, 'source_frontend_unavailable'
    validation = object_json(attempt/'frontend_validation/validation.json')
    if validation.get('passed') is not True:
        return None, 'source_frontend_not_validated'
    if not {'rounds/000/regions.npz','rounds/000/corridor.json','eizo_parameters.json'} <= files.keys():
        return None, 'source_initial_regions_unavailable'
    if not set(FILES)<=files.keys():
        raise ValueError('Complete source initialization lacks original sealed metadata/process evidence')
    q = np.load(attempt/'path.npy', allow_pickle=False)
    if q.dtype.kind not in 'fiu' or q.ndim != 2 or q.shape[1] != 7 or len(q)<2 or not np.isfinite(q).all():
        raise ValueError('Source path must be finite Nx7')
    # Reconstruct the existing saved frontend samples without another geometry
    # query. Collision masks and clearance must agree with the passing summary.
    from .validate import sample_candidate
    rebuilt = sample_candidate(dict(kind='polygon', objective='minimum_distance', points=q), cfg)
    with np.load(attempt/'frontend_validation/trajectory_samples.npz', allow_pickle=False) as z:
        for name in ('q','time','segment'):
            if name not in z or z[name].shape != rebuilt[name].shape or not np.allclose(z[name], rebuilt[name], rtol=1e-10, atol=1e-12):
                raise ValueError('Source frontend samples do not reproduce its path')
        n=len(rebuilt['q'])
        for name in ('native','independent'):
            if name not in z or z[name].dtype.kind!='b' or z[name].shape!=(n,) or not z[name].all():
                raise ValueError('Source frontend collision mask does not pass')
        if ('clearance' not in z or z['clearance'].shape!=(n,) or not np.isfinite(z['clearance']).all()
                or np.any(z['clearance']<=2e-6)):
            raise ValueError('Source frontend clearance does not pass')
        clearance_min=float(z['clearance'].min())
    required_true=('collision_free','joint_limits_passed','derivative_limits_passed')
    if (any(validation.get(k) is not True for k in required_true)
            or any(validation.get(k)!=0 for k in ('native_collisions','independent_collisions','model_disagreements','contact_band_samples'))
            or validation.get('sample_step_bound_rad')!=cfg['validation_step_rad']
            or validation.get('samples')!=len(rebuilt['q'])):
        raise ValueError('Source frontend validation contradicts its arrays/protocol')
    from ..common import BASE
    robot=object_json(BASE/'configs/robot.json')
    if any(case.get(k)!=robot.get(k) for k in ('robot_sha256','collision_geometry_sha256')):
        raise ValueError('Source case does not bind canonical model')
    if np.any(q<np.asarray(robot['lower'])-cfg['feasibility_tolerance']) or np.any(q>np.asarray(robot['upper'])+cfg['feasibility_tolerance']):
        raise ValueError('Source path violates canonical joint limits')
    if (validation.get('cost_unit')!='rad' or validation.get('time_axis')!='normalized_path_parameter'
            or not np.isclose(validation.get('cost',np.nan),rebuilt['cost'],rtol=1e-10,atol=1e-12)):
        raise ValueError('Source frontend summary contradicts reconstructed path')
    actual_step=float(np.linalg.norm(np.diff(rebuilt['q'],axis=0),axis=1).max())
    if actual_step>cfg['validation_step_rad']*(1+1e-9)+1e-12:
        raise ValueError('Source frontend sampling resolution differs')
    endpoint=float(np.max(np.linalg.norm(q[[0,-1]]-np.asarray([case['q_start'],case['q_goal']]),axis=1)))
    if endpoint>cfg['endpoint_tolerance_rad']:
        raise ValueError('Source path endpoints differ from fixed task')
    scalar_checks=dict(endpoint_error_rad=endpoint, actual_maximum_sample_joint_step_rad=actual_step,
        minimum_sampled_clearance_m=clearance_min, path_length_rad=rebuilt['cost'],
        normalized_path_length=float(np.linalg.norm(np.diff(rebuilt['q'],axis=0)/(np.asarray(robot['upper'])-np.asarray(robot['lower'])),axis=1).sum()))
    for key,value in scalar_checks.items():
        saved=validation.get(key)
        if isinstance(saved,bool) or not isinstance(saved,(int,float)) or not np.isfinite(saved) or not np.isclose(saved,value,rtol=1e-10,atol=1e-12):
            raise ValueError('Source frontend summary differs from arrays: '+key)
    if validation.get('duration_s') is not None:
        raise ValueError('Source path cannot claim physical trajectory duration')
    params=object_json(attempt/'eizo_parameters.json')
    if params.get('seed')!=expected_seed or any(params.get(k)!=v for k,v in cfg['eizo'].items()):
        raise ValueError('Source EI-ZO seed or explicit parameters differ')
    corridor=object_json(attempt/'rounds/000/corridor.json')
    if corridor.get('complete') is not True or corridor.get('protocol')!='consecutive_active_corridor_v1':
        raise ValueError('Source initial corridor is incomplete')
    mapping=corridor.get('regions')
    if not isinstance(mapping,list) or not mapping:
        raise ValueError('Source initial corridor lacks mapping')
    arrays=[];flat=[]
    with np.load(attempt/'rounds/000/regions.npz',allow_pickle=False) as z:
        expected={f'{prefix}_{i:04d}' for i in range(len(mapping)) for prefix in ('A','b')}
        if set(z.files)!=expected:raise ValueError('Source region keys differ from corridor mapping')
        for i,entry in enumerate(mapping):
            A,b=z[f'A_{i:04d}'],z[f'b_{i:04d}']
            if A.ndim!=2 or A.shape[1]!=7 or b.shape!=(len(A),) or not len(A) or not np.isfinite(A).all() or not np.isfinite(b).all():
                raise ValueError('Malformed source region arrays')
            edges=entry.get('covered_edge_indices'); seed=entry.get('original_seed_edge_index')
            if (not isinstance(edges,list) or not edges or any(type(e)!=int for e in edges)
                    or edges!=list(range(edges[0],edges[-1]+1)) or type(seed)!=int or seed not in edges
                    or edges[0]<0 or edges[-1]>=len(q)-1):
                raise ValueError('Malformed source corridor edge mapping')
            anchors=q[edges[0]:edges[-1]+2]
            if edges[0]==0:anchors=np.vstack([anchors,case['q_start']])
            if edges[-1]==len(q)-2:anchors=np.vstack([anchors,case['q_goal']])
            with np.errstate(over='ignore',invalid='ignore'):
                residual=A@anchors.T-b[:,None]
                norms=np.linalg.norm(A,axis=1)
            if not np.isfinite(residual).all() or not np.isfinite(norms).all():
                raise ValueError('Source initial region coverage arithmetic is nonfinite')
            if np.any(residual>1e-7):
                raise ValueError('Source initial region does not cover its fixed anchors')
            flat.extend(edges);arrays.append((A.copy(),b.copy()))
    if flat!=list(range(len(q)-1)):
        raise ValueError('Source corridor does not partition every original path edge')
    return dict(path=q.copy(), arrays=arrays, mapping=[dict(original_seed_edge_index=e['original_seed_edge_index'],covered_edge_indices=list(e['covered_edge_indices'])) for e in mapping]), None


def source_metadata(attempt):
    metadata={}
    original_result=object_json(attempt/'result.json')
    metadata['roadmap_native_sha256']=original_result.get('roadmap_sha256')
    if not isinstance(metadata['roadmap_native_sha256'],str) or len(metadata['roadmap_native_sha256'])!=64:
        raise ValueError('Source lacks original roadmap content hash')
    timings=original_result.get('timing',{})
    fields=('model_setup_seconds','gpu_initialization_warmup_seconds','roadmap_load_seconds',
            'online_collision_map_seconds','drm_path_seconds','drm_path_validation_seconds','eizo_inflation_seconds')
    recorded={k:timings.get(k) for k in fields}
    metadata['source_initialization_recorded_stages']=recorded
    present=[v for v in recorded.values() if isinstance(v,(int,float)) and not isinstance(v,bool) and np.isfinite(v) and v>=0]
    metadata['source_initialization_recorded_stage_seconds']=sum(present) if present else None
    process=object_json(attempt/'process.json')
    cold=process.get('cold_e2e_seconds')
    metadata['source_attempt_cold_seconds']=cold if isinstance(cold,(int,float)) and not isinstance(cold,bool) and np.isfinite(cold) and cold>=0 else None
    metadata['source_initialization_timing_scope']='Saved measured source stages only; excludes uninstrumented startup/dependency overhead and is not a separate exact frontend wall clock'
    return metadata


def bind_source(run, manifest, job, attempt, case, cfg, *, _manifest_sha256=None):
    values, unavailable = inspect_source(attempt,case,cfg,expected_seed=job['method_seed'],expected_roadmap=job['roadmap_id'])
    if unavailable:return None,unavailable
    binding=dict(mode=MODE,source_run=reference(run),source_run_manifest_sha256=_manifest_sha256 or sha(Path(run)/'run_manifest.json'),
        source_job_id=job['job_id'],source_attempt=reference(attempt),source_receipt_sha256=sha(attempt/'receipt.json'),
        case_id=case['problem_id'],case_sha256=digest(case),robot_sha256=case['robot_sha256'],
        collision_geometry_sha256=case['collision_geometry_sha256'],evaluation_config_sha256=digest(cfg),
        method_seed=job['method_seed'],replica_seed=job.get('replica_seed'),roadmap_id=job['roadmap_id'],
        files={name:sha(attempt/name) for name in FILES})
    binding.update(source_metadata(attempt))
    binding['initialization_digest']=digest(binding)
    return binding,None


def _check_binding(binding,case,cfg,seed,roadmap_id, cache):
    case=dict(case);case.pop('_objective',None)
    if not isinstance(binding,dict):raise ValueError('Shared initialization must be a bound object')
    content=dict(binding);claimed=content.pop('initialization_digest',None)
    if claimed!=digest(content) or binding.get('mode')!=MODE:
        raise ValueError('Shared initialization digest/mode mismatch')
    if (binding.get('case_sha256')!=digest(case) or binding.get('case_id')!=case['problem_id']
            or binding.get('robot_sha256')!=case['robot_sha256']
            or binding.get('collision_geometry_sha256')!=case['collision_geometry_sha256']
            or binding.get('evaluation_config_sha256')!=digest(cfg)
            or binding.get('method_seed')!=seed or binding.get('roadmap_id')!=roadmap_id):
        raise ValueError('Shared initialization is bound to another input/model/configuration/seed')
    run=path(binding['source_run']);attempt=path(binding['source_attempt'])
    # This cache is constructed within check_binding/check_bindings, never
    # supplied by a caller as a claim that evidence has already been checked.
    if run not in cache:
        manifest=object_json(run/'run_manifest.json')
        jobs={j['job_id']:j for j in manifest['jobs']}
        if len(jobs)!=len(manifest['jobs']):raise ValueError('Duplicate source job identity')
        cache[run]=(sha(run/'run_manifest.json'),jobs,{})
    manifest_hash,jobs,case_cache=cache[run]
    if manifest_hash!=binding['source_run_manifest_sha256'] or sha(attempt/'receipt.json')!=binding['source_receipt_sha256']:
        raise ValueError('Shared source manifest or receipt changed')
    source_job=jobs.get(binding['source_job_id'])
    if source_job is None:raise ValueError('Shared source job identity absent')
    if any(binding.get(k)!=source_job.get(k) for k in ('method_seed','replica_seed','roadmap_id')):
        raise ValueError('Shared source repetition identity differs from original job')
    source_attempt,_=terminal_attempt(run,source_job)
    if source_attempt is None or source_attempt.resolve()!=attempt:
        raise ValueError('Shared source is not the first terminal attempt')
    if object_json(attempt/'request.json')!=source_job or digest(resolve_case(source_job,case_cache))!=digest(case):
        raise ValueError('Shared source request/case differs from its manifest')
    if set(binding.get('files',{}))!=set(FILES):raise ValueError('Shared source file binding is incomplete')
    for name,h in binding['files'].items():
        if sha(attempt/name)!=h:raise ValueError('Shared source artifact changed: '+name)
    if any(binding.get(k)!=v for k,v in source_metadata(attempt).items()):
        raise ValueError('Shared source roadmap/timing declarations differ from sealed source evidence')
    values,missing=inspect_source(attempt,case,cfg,expected_seed=seed,expected_roadmap=roadmap_id)
    if missing:raise ValueError('Previously eligible source is now unavailable: '+missing)
    return values


def check_binding(binding,case,cfg,seed,roadmap_id):
    return _check_binding(binding,case,cfg,seed,roadmap_id,{})


def check_bindings(items):
    """One operation-local source manifest cache; every unique binding checked."""
    cache={};seen={}
    for binding,case,cfg,seed,roadmap_id in items:
        key=digest([binding,case,cfg,seed,roadmap_id])
        if key not in seen:
            seen[key]=_check_binding(binding,case,cfg,seed,roadmap_id,cache)


def load_bound(binding,case,cfg,seed,roadmap_id,out):
    values=check_binding(binding,case,cfg,seed,roadmap_id)
    from pydrake.all import HPolyhedron
    from .csdecomp_baseline import _check_corridor
    regions=[HPolyhedron(A,b) for A,b in values['arrays']]
    for region,(A,b) in zip(regions,values['arrays']):
        if not np.array_equal(region.A(),A) or not np.array_equal(region.b(),b):
            raise ValueError('Initial-region construction changed supplied coefficients')
    _check_corridor(regions,values['mapping'],values['path'],np.asarray([case['q_start'],case['q_goal']]))
    dest=Path(out)/'shared_initialization';dest.mkdir()
    source=path(binding['source_attempt'])
    for src,target in [('path.npy','path.npy'),('rounds/000/regions.npz','regions.npz'),('rounds/000/corridor.json','corridor.json')]:
        shutil.copyfile(source/src,dest/target)
    from ..common import write
    write(dest/'binding.json',binding)
    shutil.copyfile(dest/'path.npy',Path(out)/'path.npy')
    return values['path'],regions,values['mapping']


def prepare_index(source_run, cases, seed_roadmaps, cfg):
    """Freeze source-stage eligibility before either new backend is executed."""
    from ..batch import verify
    source_run=Path(source_run).resolve()
    checked=verify(source_run)
    if not checked.get('integrity_passed') or not checked.get('complete'):
        raise ValueError('Fixed initialization requires a complete intact source run')
    manifest=object_json(source_run/'run_manifest.json')
    manifest_sha256=sha(source_run/'run_manifest.json');case_cache={}
    source_jobs={}
    for job in manifest['jobs']:
        if (job.get('method'),job.get('objective'))!=('EI_ZO','minimum_distance'):continue
        key=(job['case_id'],job.get('replica_seed'))
        if key in source_jobs:raise ValueError('Duplicate source EI_ZO job for task/replica')
        source_jobs[key]=job
    bindings={};coverage=[]
    for seed,roadmap_id in seed_roadmaps:
        for case in cases:
            item=dict(problem_id=case['problem_id'],replica_seed=seed,roadmap_id=roadmap_id)
            if case.get('validation_status')!='valid_input':
                coverage.append(dict(item,eligible=False,reason='invalid_input'));continue
            job=source_jobs.get((case['problem_id'],seed))
            if job is None:raise ValueError('Source plan lacks required EI_ZO task/replica')
            pairings=[p for p in case['paired_seeds'] if p['drm_seed']==seed]
            if (len(pairings)!=1 or job.get('method_seed')!=pairings[0]['method_seed']
                    or job.get('roadmap_id')!=roadmap_id or digest(resolve_case(job,case_cache))!=digest(case)):
                raise ValueError('Source task/seed/model/roadmap differs from target cohort')
            attempt,_=terminal_attempt(source_run,job)
            if attempt is None:raise ValueError('Source plan has no terminal EI_ZO observation')
            if object_json(attempt/'request.json')!=job:
                raise ValueError('Source request differs from its planned job')
            binding,missing=bind_source(source_run,manifest,job,attempt,case,cfg,_manifest_sha256=manifest_sha256)
            if missing:
                coverage.append(dict(item,eligible=False,reason=missing,source_attempt=reference(attempt)));continue
            bindings[(case['problem_id'],seed)]=binding
            coverage.append(dict(item,eligible=True,source_attempt=reference(attempt),
                initialization_digest=binding['initialization_digest']))
    return bindings,dict(mode=MODE,source_run=reference(source_run),source_run_manifest_sha256=manifest_sha256,
        selection_basis='Original valid input plus validated source DRM path and complete original EI_ZO round-zero corridor, irrespective of final optimization status',
        registered_task_replicas=len(coverage),eligible_task_replicas=len(bindings),
        planned_paired_jobs=2*len(bindings),coverage=coverage)
