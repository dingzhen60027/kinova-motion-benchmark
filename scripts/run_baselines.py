#!/usr/bin/env python3
"""One serial, resumable end-to-end experiment over the frozen Kinova inputs."""
from __future__ import annotations

import argparse
import ast
from collections import Counter
from contextlib import contextmanager
import csv
import datetime
import fcntl
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import traceback

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from benchmarks.generic_motion_v1.common import BASE, cases, code_inventory, head, read, recorded_path, resolve_case, sha, write
from benchmarks.generic_motion_v1 import batch
from benchmarks.generic_motion_v1.roadmap import load_config
from benchmarks.generic_motion_v1.runners.methods import METHODS, catalog, objectives
from benchmarks.generic_motion_v1.runners.results import attempt_row

SEEDS = (0, 1, 2)
SPLITS = ('smoke', 'dev', 'test')
SMOKE_PROBLEM = 'smoke-bottleneck-000-p00'
PAIRS = tuple((m, o) for m in METHODS for o in objectives(m))
THREADS = {'OMP_NUM_THREADS': '4', 'OPENBLAS_NUM_THREADS': '1', 'MKL_NUM_THREADS': '1', 'CUDA_VISIBLE_DEVICES': '0'}
LGCS_SOURCE = 'benchmarks/generic_motion_v1/runners/paper_lgcs.py'


def now():
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def location(value):
    p = Path(value).expanduser()
    return (p if p.is_absolute() else ROOT / p).resolve()


def relative(value):
    p = Path(value).resolve()
    return str(p.relative_to(ROOT)) if p.is_relative_to(ROOT) else str(p)


@contextmanager
def locked(path):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('a+') as handle:
        try:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError('Another coordinator holds '+str(path)) from exc
        try:
            yield
        finally:
            fcntl.flock(handle, fcntl.LOCK_UN)


def gpu_idle(*, allow_current_process=False):
    result = subprocess.run(['nvidia-smi', '--query-compute-apps=pid,process_name,used_memory',
                             '--format=csv,noheader'], capture_output=True, text=True, timeout=15, check=True)
    # The desktop compositor is present on the recorded host. No other compute
    # workload is allowed to share the timed worker, even from another project.
    forbidden = [line for line in result.stdout.splitlines() if line.strip()
                 and '/usr/libexec/gnome-remote-desktop-daemon' not in line
                 and not (allow_current_process and line.split(',',1)[0].strip()==str(os.getpid()))]
    if forbidden:
        raise RuntimeError('GPU compute workload already present: '+repr(forbidden))
    return result.stdout.splitlines()


def environment():
    import csdecomp, pydrake, scsplanning, pybezier
    roots = [Path(m.__file__).resolve().parent for m in (csdecomp, pydrake, scsplanning, pybezier)]
    roots += [ROOT/'.local_setup/solver_probes'/name for name in
              ('ompl', 'pyscipopt', 'stomp-bridge-v1', 'trajopt_native_v2')]
    roots += [BASE/'native/build/drm_consistent_query_portable']
    files = {Path(sys.executable).resolve(), ROOT/'local/python.sh'}
    for root in roots:
        if not root.exists():
            raise FileNotFoundError(root)
        files.update(p.resolve() for p in root.rglob('*') if p.is_file() and
                     ('.so' in p.name or p.suffix == '.py' or p.name == 'build.json'))
    # Optional solvers live in repository-local wheel directories. Preflight
    # probes add these to sys.path; inventory must also work before any probe.
    distributions = list(importlib.metadata.distributions())
    distributions += list(importlib.metadata.distributions(path=[str(p) for p in roots]))
    versions = {}
    for d in distributions:
        if not d.metadata['Name']:continue
        name=d.metadata['Name'].lower()
        if name in versions and versions[name]!=d.version:raise ValueError('Conflicting dependency versions: '+name)
        versions[name]=d.version
    gpu = subprocess.check_output(['nvidia-smi', '--query-gpu=uuid,name,driver_version,memory.total',
                                   '--format=csv,noheader'], text=True).strip()
    return dict(python=sys.version, executable=str(Path(sys.executable).resolve()), prefix=sys.prefix,
                gpu=gpu, cpu=Path('/proc/cpuinfo').read_text().split('model name\t: ')[-1].splitlines()[0],
                threads={k:os.environ.get(k) for k in THREADS}, packages=versions,
                runtime_files={str(p):sha(p) for p in sorted(files)},
                runtime_stats={str(p):[p.stat().st_size,p.stat().st_mtime_ns] for p in sorted(files)})


def endpoint_audit():
    import numpy as np
    from benchmarks.generic_motion_v1.model import Model
    rows = []
    grouped = {}
    for split in SPLITS:
        for case in cases(BASE/f'manifests/{split}.jsonl'):
            if case['validation_status'] == 'valid_input':
                grouped.setdefault(case['scene_id'], []).append(case)
    for group in grouped.values():
        first=group[0]; model=Model(first['voxels_xyz'], first['voxel_radius_m'])
        if any(c['voxels_xyz']!=first['voxels_xyz'] or c['voxel_radius_m']!=first['voxel_radius_m'] for c in group):
            raise ValueError('Shared scene ID has inconsistent collision geometry')
        q=np.asarray([q for case in group for q in (case['q_start'],case['q_goal'])])
        cuda=model.cuda(q);clearance=model.clearance(q)
        for i,case in enumerate(group):
            passed=(model.robot_sha256==case['robot_sha256'] and
                    model.collision_geometry_sha256==case['collision_geometry_sha256'] and
                    np.all(q[2*i:2*i+2]>=model.lower) and np.all(q[2*i:2*i+2]<=model.upper) and
                    np.all(cuda[2*i:2*i+2]) and np.all(clearance[2*i:2*i+2]>model.contact_band_m))
            rows.append(dict(problem_id=case['problem_id'],scene_id=case['scene_id'],passed=bool(passed),
                             cuda_free=cuda[2*i:2*i+2].tolist(),clearance_m=clearance[2*i:2*i+2].tolist()))
    return dict(passed=all(r['passed'] for r in rows),legal_problems=len(rows),records=rows,
                scope='Actual CUDA and canonical Drake endpoint checks; no planning feasibility claim.')


def preflight(root, config):
    from benchmarks.generic_motion_v1.cli import require_environment
    from benchmarks.generic_motion_v1 import dataset
    from benchmarks.generic_motion_v1.health import check
    from benchmarks.generic_motion_v1.model import audit
    from benchmarks.generic_motion_v1.runners.csdecomp_baseline import dependency_audit
    require_environment()
    if any(os.environ.get(k)!=v for k,v in THREADS.items()):
        raise RuntimeError('Use the declared thread/GPU environment: '+repr(THREADS))
    cfg=load_config(config)
    if cfg['online_budget_seconds']!=60 or len(PAIRS)!=14:
        raise ValueError('This experiment requires 60 seconds and the existing 14 method-objective pairs')
    out=root/'preflight';out.mkdir(parents=True,exist_ok=True)
    gpu_idle()
    if not (out/'health.json').exists():
        check(out/'health.json')
    if not read(out/'health.json')['passed']:
        raise RuntimeError('Core health check failed')
    if not (out/'dataset.json').exists():
        write(out/'dataset.json',dataset.check())
    if read(out/'dataset.json')['result']!='passed':
        raise RuntimeError('Frozen dataset integrity check failed')
    with locked(BASE/'results/.gpu.lock'):
        # health.check performs its native probes in this disposable child;
        # its own idle CUDA context is expected until this process exits.
        gpu_idle(allow_current_process=True)
        if not (out/'model/audit.json').exists():
            audit(out/'model')
        if not read(out/'model/audit.json')['passed']:
            raise RuntimeError('Canonical model cross-check failed')
        if not (out/'endpoints.json').exists():
            write(out/'endpoints.json',endpoint_audit())
        if not read(out/'endpoints.json')['passed']:
            raise RuntimeError('Frozen endpoint collision/limit checks failed')
        dependencies=[]
        for method,objective in PAIRS:
            target=out/'dependencies'/f'{method}-{objective}';target.mkdir(parents=True,exist_ok=True)
            record=dependency_audit(target,method,objective,config=cfg)
            dependencies.append(dict(method=method,objective=objective,probe=record))
        write(out/'dependencies.json',dict(passed=True,records=dependencies))
    write(out/'environment.json',environment())
    write(out/'methods.json',catalog())
    robot=read(BASE/'configs/robot.json');maps=[]
    for seed in SEEDS:
        directory=BASE/f'roadmaps/rss2025_n6000_s{seed:03d}';m=read(directory/'metadata.json')
        if (m['status']!='ready' or m['actual_nodes']!=6000 or m['seed']!=seed or m['config']!=cfg['roadmap']
                or m['robot_sha256']!=robot['robot_sha256'] or m['collision_geometry_sha256']!=robot['collision_geometry_sha256']
                or sha(directory/'native.map')!=m['native_map_sha256']):
            raise ValueError('Frozen roadmap/model/configuration mismatch: '+str(directory))
        maps.append(dict(path=relative(directory),metadata=m,files={p.name:sha(p) for p in directory.iterdir() if p.is_file()}))
    write(out/'roadmaps.json',maps)
    return cfg


def reusable_time_run(path, cfg, current, current_environment):
    """Whole method-objective cohort eligibility, independent of success/failure."""
    m=read(path/'run_manifest.json')
    reasons=[]
    if m['source_hashes']!=current:reasons.append('runtime_source_inventory_differs')
    if m['timeout_seconds']!=60:reasons.append('budget_differs')
    if any(j['evaluation_config']!=cfg for j in m['jobs']):reasons.append('resolved_configuration_differs')
    if any(j.get('execution_mode','end_to_end')!='end_to_end' for j in m['jobs']):reasons.append('not_end_to_end')
    # A compatible solver binary alone cannot establish comparable timing.
    # Require the original sealed hardware/thread/runtime attestation as well.
    historical_environment=m.get('environment')
    if not historical_environment:
        reasons.append('missing_sealed_runtime_hardware_and_thread_attestation')
    elif any(historical_environment.get(k)!=current_environment[k] for k in
             ('python','gpu','cpu','threads','packages','runtime_files')):
        reasons.append('execution_environment_differs')
    record=dict(run=relative(path),manifest_sha256=sha(path/'run_manifest.json'),source_commit=m['source_code_commit'],
                registered_jobs=len(m['jobs']),reasons=reasons,eligible_jobs=0)
    if reasons:return record,{}
    checked=batch.verify(path,current_sources=True)
    record['verification']=checked
    if not checked['integrity_passed'] or not checked['complete']:
        record['reasons'].append('incomplete_or_invalid_evidence');return record,{}
    # Historical path profiles did not seal their SNOPT library fingerprint.
    # Reuse only the complete time cohort whose per-attempt native fingerprints
    # bind both the DRM collision engine and the actual IPOPT/Drake solver.
    eligible={};cache={};native_cache={}
    for job in m['jobs']:
        case=resolve_case(job,cache)
        if job['method']!='NTO_REFINED' or job['objective']!='minimum_time' or case['validation_status']!='valid_input':continue
        attempt=batch._first_terminal_attempt(sorted((path/'jobs'/job['job_id']/'attempts').glob('[0-9][0-9][0-9][0-9]')))
        try:
            receipt=read(attempt/'receipt.json')
            for name in ('paper_nto_dependencies.json','build_receipt.json'):
                if name not in receipt['files']:raise ValueError('unsealed native dependency evidence')
            solver=read(attempt/'paper_nto_dependencies.json')['linear_system']['native_binary']
            build=read(attempt/'build_receipt.json')
            for name,digest in ((solver['library'],solver['sha256']),
                                (build['native_library'],build['native_library_sha256']),
                                (build['binary'],build['binary_sha256'])):
                p=recorded_path(name)
                if str(p) not in native_cache:native_cache[str(p)]=sha(p)
                if native_cache[str(p)]!=digest:raise ValueError('native dependency changed')
            if not solver['loaded_matches_package']:raise ValueError('solver library did not match package')
            row=attempt_row(path,m,job,attempt,verify_files=True,case_cache=cache)
            if row['status']=='infrastructure_error':raise ValueError('historical infrastructure error')
            eligible[(case['problem_id'],job['replica_seed'],job['method'],job['objective'])]=dict(
                job=job,source_run=relative(path),source_job=relative(path/'jobs'/job['job_id']),
                source_attempt=relative(attempt),receipt_sha256=sha(attempt/'receipt.json'))
        except (OSError,KeyError,TypeError,ValueError) as exc:
            record['reasons'].append('entire_time_cohort_rejected: '+str(exc));return record,{}
    record.update(eligible_jobs=len(eligible),native_files_verified=native_cache,
                  excluded_path_reason='Historical SNOPT path attempts lack a sealed solver-library fingerprint')
    return record,eligible


def prepare(root, config):
    root.mkdir(parents=True,exist_ok=True)
    if (root/'experiment.json').exists():
        return verify_context(root)
    # CUDA dependency probes run in a child process and release their context
    # before the timed workers start. The coordinator never owns a GPU context.
    with (root/'preflight.log').open('a') as log:
        subprocess.run([sys.executable,str(Path(__file__).resolve()),'check','--run',str(root),'--config',str(config)],
                       stdout=log,stderr=subprocess.STDOUT,check=True)
    cfg=load_config(config);current=code_inventory()
    current_environment=read(root/'preflight/environment.json')
    audit=[];reuse={}
    paths=sorted((BASE/'results').glob('*/run_manifest.json'))
    for path in paths:
        if path.parent.name not in ('baseline_dev','baseline_test','nto_metis_dev','nto_metis_test') and not path.parent.name.startswith('rss2025_formal_'):continue
        record,entries=reusable_time_run(path.parent,cfg,current,current_environment)
        audit.append(record)
        for key,value in entries.items():
            if key in reuse:raise ValueError('Ambiguous historical reuse candidate')
            reuse[key]=value
    write(root/'reuse_audit.json',dict(policy='Exact runtime sources, resolved configuration, inputs, roadmap bytes, seeds and validator; full sealed receipts; per-attempt solver and DRM native hashes. Eligibility never uses success or objective value.',runs=audit))
    stages=[];matrix=[]
    for split in SPLITS:
        data=cases(BASE/f'manifests/{split}.jsonl');legal={c['problem_id'] for c in data if c['validation_status']=='valid_input'}
        for seed in SEEDS:
            name=f'{split}-seed-{seed}';target=root/name
            if target.exists():
                existing=read(target/'run_manifest.json')
                check=batch.verify(target,current_sources=True)
                if (not check['integrity_passed'] or existing['timeout_seconds']!=60
                        or any(j['evaluation_config']!=cfg for j in existing['jobs'])
                        or any(not p.is_symlink() for p in (target/'jobs').glob('*'))):
                    raise ValueError('Partial preparation contains changed or already executed jobs: '+str(target))
            else:
                batch.prepare(BASE/f'manifests/{split}.jsonl',[BASE/f'roadmaps/rss2025_n6000_s{seed:03d}'],target,
                              methods=list(METHODS),objectives=['minimum_distance','minimum_time'],timeout=60,
                              config=cfg,allow_test=True,snapshot_sources=False)
            # Complete the new manifest before any attempt exists. The original
            # input file is never edited or copied; failures remain registered
            # in that input and in the explicit exclusion count below.
            m=read(target/'run_manifest.json');m['jobs']=[j for j in m['jobs'] if j['case_id'] in legal]
            m.update(selection='all_valid_inputs',registered_inputs=len(data),legal_inputs=len(legal),
                     excluded_generation_failures=len(data)-len(legal))
            write(target/'run_manifest.json',m);write(target/'identity.json',dict(run_manifest_sha256=sha(target/'run_manifest.json')))
            expected={(p,seed,method,obj) for p in legal for method,obj in PAIRS}
            actual={(j['case_id'],j['replica_seed'],j['method'],j['objective']) for j in m['jobs']}
            if expected!=actual or len(actual)!=len(m['jobs']):raise ValueError('Incomplete or duplicate matrix')
            for job in m['jobs']:
                key=(job['case_id'],seed,job['method'],job['objective']);old=reuse.get(key)
                if old and old['job']!=job:raise ValueError('Reused request differs from new frozen job')
                if old:
                    destination=target/'jobs'/job['job_id'];destination.parent.mkdir(exist_ok=True)
                    if destination.is_symlink():
                        if destination.resolve()!=recorded_path(old['source_job']):raise ValueError('Changed reuse link')
                    else:
                        destination.symlink_to(os.path.relpath(recorded_path(old['source_job']),destination.parent),target_is_directory=True)
                matrix.append(dict(split=split,replica_seed=seed,problem_id=job['case_id'],scene_id=job['scene_id'],
                    method=job['method'],objective=job['objective'],method_seed=job['method_seed'],stage=name,job_id=job['job_id'],
                    source='reused' if old else 'new',source_attempt=old['source_attempt'] if old else '',
                    source_receipt_sha256=old['receipt_sha256'] if old else ''))
            stages.append(dict(name=name,split=split,seed=seed,jobs=len(m['jobs']),
                               manifest_sha256=sha(target/'run_manifest.json')))
    with (root/'matrix.csv').open('w',newline='') as handle:
        writer=csv.DictWriter(handle,fieldnames=list(matrix[0]));writer.writeheader();writer.writerows(matrix)
    selection=dict(comparisons=[dict(id='baselines_'+split,mode='end_to_end',runs=[relative(root/s['name']) for s in stages if s['split']==split]) for split in SPLITS],
                   figure_comparisons=['baselines_test'])
    write(root/'selection.json',selection)
    frozen={relative(p):sha(p) for p in [location(config),Path(__file__),ROOT/'local/python.sh',root/'matrix.csv',root/'reuse_audit.json',root/'selection.json']}
    frozen.update({relative(p):sha(p) for p in (root/'preflight').rglob('*') if p.is_file()})
    result=dict(created_utc=now(),source_commit=head(),source_hashes=current,config=cfg,config_path=relative(config),
                methods=list(METHODS),pairs=[list(p) for p in PAIRS],seeds=list(SEEDS),budget_seconds=60,mode='end_to_end',
                stages=stages,planned_legal_jobs=len(matrix),reused_jobs=sum(r['source']=='reused' for r in matrix),
                new_jobs=sum(r['source']=='new' for r in matrix),frozen_files=frozen,
                scope='No new algorithms, test tuning, fixed-frontend ablation or MuJoCo execution.',
                smoke_probe=dict(problem=SMOKE_PROBLEM,seed=0,combinations=14),
                statistics='Separate path/time tracks; all legal failures retained; matched problem and replica seed; 95% scene-cluster bootstrap, 2000 PCG64 draws, shared open_control as one cluster.')
    write(root/'experiment.json',result)
    write(root/'experiment_identity.json',dict(sha256=sha(root/'experiment.json')))
    return result


def experiment_context(root):
    """An explicit repair supplements, never rewrites, the original seal."""
    experiment=read(root/'experiment.json')
    if sha(root/'experiment.json')!=read(root/'experiment_identity.json')['sha256']:raise ValueError('Experiment manifest changed')
    if not (root/'repair.json').exists():return experiment
    repair=read(root/'repair.json')
    if sha(root/'repair.json')!=read(root/'repair_identity.json')['sha256']:raise ValueError('Repair declaration changed')
    if repair['original_experiment_sha256']!=sha(root/'experiment.json'):raise ValueError('Repair parent changed')
    for stage in experiment['stages']:
        if sha(root/stage['name']/'run_manifest.json')!=stage['manifest_sha256']:raise ValueError('Original matrix changed')
    frozen=dict(experiment['frozen_files'])
    frozen.update(repair['frozen_files'])
    return dict(experiment,source_commit=repair['source_commit'],source_hashes=repair['source_hashes'],
        stages=repair['stages'],frozen_files=frozen,reused_jobs=repair['retained_jobs'],new_jobs=repair['new_jobs'],
        selection_path='repair_selection.json',smoke_check_path='repair_smoke_check.json',repair=repair)


def verify_context(root, *, full_environment=True):
    experiment=experiment_context(root)
    if code_inventory()!=experiment['source_hashes']:raise ValueError('Runtime source changed; formal experiment stopped')
    for name,digest in experiment['frozen_files'].items():
        if sha(recorded_path(name))!=digest:raise ValueError('Frozen experiment input changed: '+name)
    env=read(root/'preflight/environment.json')
    if any(os.environ.get(k)!=v for k,v in env['threads'].items()):raise ValueError('Thread/GPU policy changed')
    for name,digest in env['runtime_files'].items():
        p=recorded_path(name)
        if full_environment:
            if sha(p)!=digest:raise ValueError('Runtime dependency changed: '+name)
        elif [p.stat().st_size,p.stat().st_mtime_ns]!=env['runtime_stats'][name]:
            raise ValueError('Runtime dependency metadata changed: '+name)
    for stage in experiment['stages']:
        if sha(root/stage['name']/'run_manifest.json')!=stage['manifest_sha256']:raise ValueError('Job matrix changed')
    return experiment


def lgcs_change_scope(old_sources, current_sources, old_text, current_text):
    """Only the LGCS-only optional route routine may differ for this repair."""
    changed={p for p in set(old_sources)|set(current_sources) if old_sources.get(p)!=current_sources.get(p)}
    if changed!={LGCS_SOURCE}:raise ValueError('Repair must change only the LGCS adapter runtime file')
    trees=[ast.parse(text) for text in (old_text,current_text)]
    for tree in trees:
        functions=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='second_route']
        if len(functions)!=1:raise ValueError('Missing unique LGCS second_route function')
        tree.body.remove(functions[0])
    if ast.dump(trees[0])!=ast.dump(trees[1]):raise ValueError('Change outside LGCS second_route')
    # csdecomp_baseline imports paper_graph only under method == 'LGCS'.
    # Its dependency probe has the same guard. Both callers are hash-identical.
    return dict(changed_files=sorted(changed),changed_function='second_route',
        affected_methods=['LGCS'],policy='Rerun the entire LGCS cohort, irrespective of previous outcome. Other methods cannot call the changed function; all their runtime files, inputs, configuration, validation, solver binaries and threads remain identical.')


def repair_lgcs(root, config):
    """One bounded cohort repair; uses ordinary batch manifests and receipts."""
    with locked(root/'.experiment.lock'), locked(BASE/'results/.gpu.lock'):
        gpu_idle()
        if (root/'repair.json').exists():return verify_context(root)
        original=experiment_context(root);current=code_inventory()
        if batch._git_source_errors(head(),current):raise ValueError('Commit the runtime repair before preparation')
        old_text=subprocess.check_output(['git','show',original['source_commit']+':'+LGCS_SOURCE],cwd=ROOT,text=True)
        scope=lgcs_change_scope(original['source_hashes'],current,old_text,(ROOT/LGCS_SOURCE).read_text())
        launcher=relative(Path(__file__))
        # Verify every immutable original asset. Only the launcher is updated;
        # its previous bytes must still match the original committed seal.
        for name,digest in original['frozen_files'].items():
            actual=(hashlib.sha256(subprocess.check_output(['git','show',original['source_commit']+':'+name],cwd=ROOT)).hexdigest()
                    if name==launcher else sha(recorded_path(name)))
            if actual!=digest:raise ValueError('Original frozen input changed: '+name)
        env=read(root/'preflight/environment.json')
        observed=environment()
        if any(observed[k]!=env[k] for k in ('python','gpu','cpu','threads','packages','runtime_files')):
            raise ValueError('Repair environment differs from original experiment')
        if load_config(config)!=original['config']:raise ValueError('Repair configuration changed')
        previous={};retained={};discarded=[];audit={};cache={}
        from benchmarks.generic_motion_v1.runners.failure_classification import KNOWN_DRM_NO_ROUTE
        for stage in original['stages']:
            directory=root/stage['name'];m=read(directory/'run_manifest.json')
            if sha(directory/'run_manifest.json')!=stage['manifest_sha256']:raise ValueError('Original stage changed')
            audit[stage['name']]=batch.verify(directory)
            if not audit[stage['name']]['integrity_passed']:raise ValueError('Original evidence failed verification')
            for job in m['jobs']:
                key=job['job_id']
                if key in previous:raise ValueError('Duplicate original job')
                previous[key]=(directory,m,job)
                attempt=batch._first_terminal_attempt(sorted((directory/'jobs'/key/'attempts').glob('[0-9][0-9][0-9][0-9]')))
                if attempt is None:continue
                row=attempt_row(directory,m,job,attempt,verify_files=True,case_cache=cache)
                if row['status'] in ('infrastructure_error','invalid_input'):
                    raw=read(attempt/'result.json')
                    if not (job['method']=='LGCS' and raw.get('failed_stage')=='alternate_drm_path'
                            and raw.get('exception')==repr(RuntimeError(KNOWN_DRM_NO_ROUTE))):
                        raise ValueError('Unrelated systemic failure: '+str(attempt))
                evidence=dict(job_id=key,source_run=relative(directory),source_attempt=relative(attempt),
                              receipt_sha256=sha(attempt/'receipt.json'),status=row['status'])
                if job['method']=='LGCS':discarded.append(evidence)
                else:retained[key]=evidence
        stages=[];matrix=[]
        for split in SPLITS:
            group=[entry for entry in previous.values() if entry[1]['split']==split]
            kept=[j for _,_,j in group if j['job_id'] in retained]
            for kind,jobs in [('retained',kept),('continued',[j for _,_,j in group if j['job_id'] not in retained])]:
                if not jobs:continue
                target=root/(kind+'-'+split)
                if not target.exists():
                    if kind=='continued':
                        batch.prepare(BASE/f'manifests/{split}.jsonl',
                            [BASE/f'roadmaps/rss2025_n6000_s{s:03d}' for s in SEEDS],target,
                            methods=list(METHODS),objectives=['minimum_distance','minimum_time'],timeout=60,
                            config=original['config'],allow_test=True,snapshot_sources=False)
                        m=read(target/'run_manifest.json');generated={j['job_id']:j for j in m['jobs']}
                        if any(generated.get(j['job_id'])!=j for j in jobs):raise ValueError('Repair changed a frozen request')
                    else:
                        target.mkdir();m=dict(group[0][1])
                        m['roadmap_files']={p:h for _,source,_ in group for p,h in source['roadmap_files'].items()}
                    m.update(jobs=jobs,selection='explicit_lgcs_repair_'+kind,
                             derived_from_manifests={relative(d/'run_manifest.json'):sha(d/'run_manifest.json') for d in {entry[0] for entry in group}})
                    write(target/'run_manifest.json',m)
                    write(target/'identity.json',dict(run_manifest_sha256=sha(target/'run_manifest.json')))
                m=read(target/'run_manifest.json')
                if m['jobs']!=jobs or m['source_hashes']!=(current if kind=='continued' else original['source_hashes']):
                    raise ValueError('Partial repair preparation changed')
                if kind=='retained':
                    for job in jobs:
                        link=target/'jobs'/job['job_id'];link.parent.mkdir(exist_ok=True)
                        source=previous[job['job_id']][0]/'jobs'/job['job_id']
                        if link.is_symlink():
                            if link.resolve()!=source.resolve():raise ValueError('Retained evidence link changed')
                        else:link.symlink_to(os.path.relpath(source,link.parent),target_is_directory=True)
                check=batch.verify(target,current_sources=kind=='continued')
                if not check['integrity_passed']:raise ValueError('Prepared repair failed: '+repr(check['errors']))
                stages.append(dict(name=target.name,split=split,jobs=len(jobs),read_only=kind=='retained',
                                   manifest_sha256=sha(target/'run_manifest.json')))
                for job in jobs:
                    old=retained.get(job['job_id'])
                    matrix.append(dict(split=split,replica_seed=job['replica_seed'],problem_id=job['case_id'],
                        method=job['method'],objective=job['objective'],stage=target.name,job_id=job['job_id'],
                        source='retained' if old else 'new',source_attempt=old['source_attempt'] if old else ''))
        if len(matrix)!=len(previous) or {r['job_id'] for r in matrix}!=set(previous):raise ValueError('Repair matrix incomplete')
        with (root/'repair_matrix.csv').open('w',newline='') as handle:
            writer=csv.DictWriter(handle,fieldnames=list(matrix[0]));writer.writeheader();writer.writerows(matrix)
        write(root/'repair_selection.json',dict(comparisons=[dict(id='baselines_'+split,mode='end_to_end',
            compatibility_note=scope['policy']+' See repair.json for byte hashes and the complete cohort selection.',
            runs=[relative(root/s['name']) for s in stages if s['split']==split]) for split in SPLITS],figure_comparisons=['baselines_test']))
        repair=dict(created_utc=now(),original_experiment_sha256=sha(root/'experiment.json'),source_commit=head(),
            source_hashes=current,scope=scope,original_verification=audit,stages=stages,
            retained_jobs=len(retained),new_jobs=len(previous)-len(retained),
            superseded_lgcs_attempts=discarded,retained_evidence=list(retained.values()),
            frozen_files={relative(p):sha(p) for p in (Path(__file__),root/'repair_matrix.csv',root/'repair_selection.json')})
        write(root/'repair.json',repair);write(root/'repair_identity.json',dict(sha256=sha(root/'repair.json')))
        return verify_context(root)


def finished(root, manifest, job, case_cache):
    attempt=batch._first_terminal_attempt(sorted((root/'jobs'/job['job_id']/'attempts').glob('[0-9][0-9][0-9][0-9]')))
    if attempt is None:return None
    row=attempt_row(root,manifest,job,attempt,verify_files=True,case_cache=case_cache)
    if row['status'] in ('infrastructure_error','invalid_input'):
        raise RuntimeError('Systemic failure stops this experiment: '+str(attempt)+': '+str(row['failure_reason']))
    return row


def execute_stage(root, stage, *, probe=False):
    directory=root/stage['name']
    if stage.get('read_only'):
        check=batch.verify(directory)
        if not check['integrity_passed'] or not check['complete']:raise ValueError('Retained cohort is incomplete or changed')
        return check
    # Reuse the existing interruption sealing and source/input verification.
    batch.resume(directory,max_jobs=0)
    m=read(directory/'run_manifest.json');cache={}
    completed=set()
    for job in m['jobs']:
        if finished(directory,m,job,cache) is not None:completed.add(job['job_id'])
    with locked(BASE/'results/.gpu.lock'):
        for job in m['jobs']:
            if probe and (job['case_id']!=SMOKE_PROBLEM or job['replica_seed']!=0):continue
            if job['job_id'] in completed:continue
            verify_context_fast(root)
            gpu_idle()
            write(root/'progress.json',dict(state='running',stage=stage['name'],job_id=job['job_id'],
                  method=job['method'],objective=job['objective'],problem_id=job['case_id'],
                  completed_in_stage=len(completed),stage_jobs=len(m['jobs']),updated_utc=now(),coordinator_pid=os.getpid()))
            batch.execute(job,directory,m['timeout_seconds'])
            if finished(directory,m,job,cache) is None:
                raise RuntimeError('Worker returned without sealed terminal evidence')
            completed.add(job['job_id'])
    check=batch.verify(directory)
    write(directory/'verification.json',check)
    if not check['integrity_passed']:raise ValueError('Stage integrity failure: '+repr(check['errors']))
    if not probe and not check['complete']:raise RuntimeError('Stage unexpectedly incomplete')
    return check


def verify_context_fast(root):
    """Cheap inter-query guards; batch.execute also hashes runtime code each time."""
    exp=experiment_context(root)
    for name in (relative(Path(__file__)),relative(location(exp['config_path'])),relative(ROOT/'local/python.sh')):
        if sha(recorded_path(name))!=exp['frozen_files'][name]:raise ValueError('Launcher or configuration changed')
    env=read(root/'preflight/environment.json')
    if any(os.environ.get(k)!=v for k,v in env['threads'].items()):raise ValueError('Thread/GPU policy changed')
    for name,stat in env['runtime_stats'].items():
        p=recorded_path(name)
        if [p.stat().st_size,p.stat().st_mtime_ns]!=stat:raise ValueError('Runtime dependency changed: '+name)


def status(root):
    exp=experiment_context(root);groups=[];total=0
    for stage in exp['stages']:
        directory=root/stage['name'];m=read(directory/'run_manifest.json');counts=Counter();by_method=Counter();complete=0
        for job in m['jobs']:
            attempt=batch._first_terminal_attempt(sorted((directory/'jobs'/job['job_id']/'attempts').glob('[0-9][0-9][0-9][0-9]')))
            if attempt is not None:
                receipt=read(attempt/'receipt.json')
                if sha(attempt/'result.json')!=receipt['files']['result.json']:raise ValueError('Result seal changed')
                counts[receipt['status']]+=1;by_method[job['method']]+=1;complete+=1
        total+=complete
        groups.append(dict(stage=stage['name'],completed=complete,remaining=len(m['jobs'])-complete,
                           raw_statuses=dict(counts),completed_by_method=dict(by_method)))
    progress=read(root/'progress.json') if (root/'progress.json').exists() else {'state':'prepared'}
    pid=progress.get('coordinator_pid')
    return dict(planned=exp['planned_legal_jobs'],completed=total,remaining=exp['planned_legal_jobs']-total,
                reused=exp['reused_jobs'],newly_completed=total-exp['reused_jobs'],progress=progress,stages=groups,
                superseded_lgcs_attempts=len(exp.get('repair',{}).get('superseded_lgcs_attempts',[])),
                coordinator_alive=bool(pid and Path(f'/proc/{pid}').exists()),
                note='Receipt/result status view; use verify for full artifact and matrix validation.')


def verify(root):
    exp=verify_context(root);checks={}
    expected=set()
    for split in SPLITS:
        for c in cases(BASE/f'manifests/{split}.jsonl'):
            if c['validation_status']=='valid_input':expected.update((c['problem_id'],s,m,o) for s in SEEDS for m,o in PAIRS)
    actual=set();duplicates=False
    for stage in exp['stages']:
        directory=root/stage['name'];checks[stage['name']]=batch.verify(directory,allow_unsealed=True)
        for j in read(directory/'run_manifest.json')['jobs']:
            key=(j['case_id'],j['replica_seed'],j['method'],j['objective']);duplicates |= key in actual;actual.add(key)
    retired_checks={}
    if 'repair' in exp:
        for s in read(root/'experiment.json')['stages']:
            retired_checks[s['name']]=batch.verify(root/s['name'])
    passed=not duplicates and expected==actual and all(c['integrity_passed'] for c in list(checks.values())+list(retired_checks.values()))
    result=dict(integrity_passed=passed,complete=passed and all(c['complete'] for c in checks.values()),
                planned=len(expected),missing=len(expected-actual),extra=len(actual-expected),duplicate=duplicates,
                stages=checks,preserved_original_stages=retired_checks)
    write(root/'verification.json',result)
    return result


def summarize(root):
    from benchmarks.generic_motion_v1.runners.results import export
    result=verify(root)
    if not result['integrity_passed']:raise ValueError('Cannot summarize corrupt experiment')
    exp=experiment_context(root)
    return export(output=root/'analysis',roots=[root/s['name'] for s in exp['stages']],
                  selection=root/exp.get('selection_path','selection.json'))


def smoke_rows(root, exp):
    rows=[];cache={}
    for stage in exp['stages']:
        if stage['split']!='smoke':continue
        directory=root/stage['name'];check=batch.verify(directory)
        if not check['integrity_passed']:raise ValueError('Smoke artifacts failed verification')
        manifest=read(directory/'run_manifest.json')
        rows.extend(finished(directory,manifest,j,cache) for j in manifest['jobs']
                    if j['case_id']==SMOKE_PROBLEM and j['replica_seed']==0)
    if len(rows)!=14 or any(row is None for row in rows):raise ValueError('Smoke interface matrix is incomplete')
    return rows


def run(root, config, *, probe=False):
    with locked(root/'.experiment.lock'):
        try:
            exp=prepare(root,config);verify_context(root)
            if probe:
                for stage in exp['stages']:
                    if stage['split']=='smoke' and stage.get('seed',0)==0:execute_stage(root,stage,probe=True)
                rows=smoke_rows(root,exp)
                write(root/exp.get('smoke_check_path','smoke_check.json'),dict(passed=True,scope='Real end-to-end calls for all 14 pairs, fixed smoke problem, seed 0; ordinary algorithm failures retained.',
                      rows=[{k:r[k] for k in ('method','objective','status','failure_reason','cold_e2e_seconds','attempt_path')} for r in rows]))
                write(root/'progress.json',dict(state='smoke_checked',updated_utc=now()))
                return status(root)
            for stage in exp['stages']:
                verify_context(root)
                execute_stage(root,stage)
            result=summarize(root)
            write(root/'progress.json',dict(state='complete',updated_utc=now(),summary=result))
            return status(root)
        except BaseException as exc:
            write(root/'progress.json',dict(state='stopped',updated_utc=now(),error_type=type(exc).__name__,error=str(exc)))
            with (root/'failure.log').open('a') as handle:handle.write(now()+'\n'+traceback.format_exc()+'\n')
            raise


def start(root, config, session):
    if not shutil.which('tmux'):raise RuntimeError('tmux is unavailable')
    prepare(root,config);exp=verify_context(root)
    smoke_check=root/(exp or {}).get('smoke_check_path','smoke_check.json')
    if not smoke_check.exists() or not read(smoke_check)['passed']:
        raise RuntimeError('Run smoke-check before detaching the complete experiment')
    smoke_rows(root,exp)
    if subprocess.run(['tmux','has-session','-t','='+session],capture_output=True).returncode==0:
        raise RuntimeError('tmux session already exists: '+session)
    argv=['env']+[f'{k}={v}' for k,v in THREADS.items()]+['bash',str(ROOT/'local/python.sh'),str(Path(__file__).resolve()),
                    'run','--run',str(root),'--config',str(config)]
    command=shlex.join(argv)+' >> '+shlex.quote(str(root/'console.log'))+' 2>&1'
    # Reject a live foreground coordinator too, not only a duplicate tmux name.
    with locked(root/'.experiment.lock'):
        pass
    subprocess.run(['tmux','new-session','-d','-s',session,'-c',str(ROOT),command],check=True)
    write(root/'launch.json',dict(session=session,command=argv,started_utc=now(),console=relative(root/'console.log')))
    return dict(started=True,tmux_session=session,console=relative(root/'console.log'))


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('command',choices=['check','prepare','repair-lgcs','smoke-check','start','run','resume','status','verify','summarize'])
    p.add_argument('--run',default='runs/end_to_end');p.add_argument('--config',default='configs/evaluation.json')
    p.add_argument('--session',default='kinova-baselines')
    args=p.parse_args(argv);root=location(args.run);config=location(args.config)
    # The launcher already pins BLAS; this entry also fixes OMP and GPU.
    for key,value in THREADS.items():os.environ[key]=value
    if args.command=='check':
        if (root/'experiment.json').exists():raise ValueError('Prepared preflight is immutable; use verify')
        preflight(root,config);result=dict(passed=True)
    elif args.command=='prepare':result=prepare(root,config)
    elif args.command=='repair-lgcs':
        exp=repair_lgcs(root,config)
        result=dict(prepared=True,retained_jobs=exp['reused_jobs'],new_jobs=exp['new_jobs'],
                    superseded_lgcs_attempts=len(exp['repair']['superseded_lgcs_attempts']))
    elif args.command=='smoke-check':result=run(root,config,probe=True)
    elif args.command in ('run','resume'):result=run(root,config)
    elif args.command=='start':result=start(root,config,args.session)
    elif args.command=='status':result=status(root)
    elif args.command=='verify':result=verify(root)
    else:result=summarize(root)
    print(json.dumps(result,indent=2,default=str))
    return 1 if result.get('integrity_passed') is False else 0


if __name__=='__main__':
    raise SystemExit(main())
