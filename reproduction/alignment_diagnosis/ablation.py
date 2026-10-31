"""Frozen same-input NTO matrix, process-isolated with all failures retained."""
import argparse
from collections import Counter
from copy import deepcopy
import datetime
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time
import traceback
import numpy as np
from reproduction.common import ROOT, atomic, read
from reproduction.paper_constructed.run import execute_child, identity


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def specs():
    variants = []
    for name, mult, margin in [('baseline',1,0.), ('margin_1mm',1,.001), ('margin_10mm',1,.01),
                                ('dense_4x',4,0.), ('dense_4x_margin_1mm',4,.001),
                                ('dense_4x_margin_10mm',4,.01)]:
        variants.append(dict(name='distance_'+name, problem='distance', nc_multiplier=mult, clearance=margin))
    for name, controls, margin in [('baseline',0,0.), ('margin_1mm',0,.001), ('knots16',16,0.),
                                   ('knots16_margin_1mm',16,.001), ('knots32_margin_1mm',32,.001)]:
        variants.append(dict(name='time_'+name, problem='time', minimum_controls=controls, clearance=margin))
    return variants


def inputs(run, case, kind):
    paths = [run/'manifest.json', run/'cases'/case/'input.json',
             run/'cases'/case/'seed_path.npz', run/'cases'/case/'case.json']
    for method in ['NTO_distance', 'NTO_time']:
        paths += [run/'cases'/case/method/'result.json', run/'cases'/case/method/'candidate_00.npz']
    if kind == 'fr3':
        paths.append(run/'scenes'/case.split('_')[0]/'scene.json')
    return paths


def worker(root, job):
    from reproduction.paper_constructed.model import ForestEnvironment, FR3
    from reproduction.paper_constructed.validation import validate
    from reproduction.pipeline import save_candidate
    from .transcription import solve
    manifest = read(root/'manifest.json')
    run = Path(manifest['source_run'])
    cfg = deepcopy(read(run/'manifest.json')['config'])
    dest = root/'jobs'/job['id']
    dest.mkdir(parents=True, exist_ok=True)
    started = time.perf_counter()
    record = dict(case=job['case'], variant=job['variant'], status='started')
    atomic(dest/'result.json', record)
    try:
        case = job['case']
        seed = int(case[1:3])
        path = np.load(run/'cases'/case/'seed_path.npz')['path']
        if manifest['environment'] == 'forest':
            env = ForestEnvironment(cfg, seed)
        else:
            env = FR3(cfg, np.array(read(run/'scenes'/f'e{seed:02d}'/'scene.json')['obstacles']))
        env.start, env.goal = path[0].copy(), path[-1].copy()
        env.cfg.update(start=env.start.tolist(), goal=env.goal.tolist())
        candidate = solve(path, env, job['variant'])
        save_candidate(dest, candidate)
        check = validate(candidate, env)
        np.savez_compressed(dest/'trajectory_samples.npz', **check.pop('_samples'))
        atomic(dest/'validation.json', check)
        record.update(status=('success_numerical' if candidate['solver_success'] and check['passed']
                              else 'validation_failed' if candidate['solver_success'] else 'optimization_failed'),
                      solver_success=candidate['solver_success'], validation_passed=check['passed'],
                      collision_free=check['collision_free'], cost=check['cost'],
                      native_cuda_collision_free=check['native_cuda_collision_free'],
                      max_collocation_residual=candidate['max_collocation_residual'],
                      control_points=candidate['control_points'],
                      solver_seconds=candidate['solver_seconds'],
                      warm_start_shape_max_error=candidate['warm_start_shape_max_error'])
        if job['variant']['name'].endswith('_baseline'):
            method = 'NTO_'+job['variant']['problem']
            old = read(run/'cases'/case/method/'result.json')
            record['baseline_parity'] = dict(status_same=old['status']==record['status'],
                cost_difference=None if old.get('cost') is None else check['cost']-old['cost'])
    except Exception as error:
        record.update(status='error', error=repr(error), traceback=traceback.format_exc())
    record['wall_seconds'] = time.perf_counter()-started
    atomic(dest/'result.json', record)
    print(json.dumps({k:record.get(k) for k in ['case','status','cost','solver_seconds']}, allow_nan=False), flush=True)


def summary(root):
    manifest = read(root/'manifest.json')
    records = []
    for job in manifest['jobs']:
        file = root/'jobs'/job['id']/'result.json'
        if file.exists():
            records.append(read(file))
    result = dict(planned=len(manifest['jobs']), recorded=len(records),
                  status_counts=dict(Counter(r['status'] for r in records)), variants={})
    for spec in manifest['variants']:
        rows = [r for r in records if r['variant']['name']==spec['name']]
        passed = [r for r in rows if r['status']=='success_numerical']
        result['variants'][spec['name']] = dict(recorded=len(rows), status_counts=dict(Counter(r['status'] for r in rows)),
            strictly_valid=len(passed), solver_success=sum(r.get('solver_success',False) for r in rows),
            valid_mean_cost=float(np.mean([r['cost'] for r in passed])) if passed else None)
    result['input_reverification_passed'] = all(sha(Path(name))==expected for name,expected in manifest['input_hashes'].items())
    result['source_reverification_passed'] = all(sha(ROOT/name)==expected for name,expected in manifest['source_hashes'].items())
    atomic(root/'summary.json', result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--environment', choices=['forest','fr3'])
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--resume', action='store_true')
    parser.add_argument('--worker', type=int)
    args = parser.parse_args()
    root = args.output.resolve()
    if args.worker is not None:
        worker(root, read(root/'manifest.json')['jobs'][args.worker])
        return
    if not args.resume:
        root.mkdir(parents=True, exist_ok=False)
        run = Path(read(ROOT/'reproduction/reports/paper_constructed/run_index.json')[args.environment]['run_dir'])
        old = read(run/'manifest.json')
        # Fixed input-only rule covers all ten scenes. Do not select by NTO outcomes.
        selected = [c for c in old['cases'] if c['roadmap_seed']==0 and c['roadmap_size']==min(old['config']['roadmap_sizes'][args.environment])]
        variants = specs()
        jobs = [dict(id=c['id']+'__'+v['name'], case=c['id'], variant=v) for c in selected for v in variants]
        files = inputs(run, selected[0]['id'], args.environment)[:1]
        for c in selected:
            files.extend(inputs(run,c['id'],args.environment)[1:])
        source = identity()
        for p in [Path(__file__), Path(__file__).with_name('transcription.py')]:
            source[str(p.relative_to(ROOT))] = sha(p)
        for name in source:
            target = root/'source_snapshot'/name
            target.parent.mkdir(parents=True,exist_ok=True)
            target.write_bytes((ROOT/name).read_bytes())
        atomic(root/'manifest.json', dict(environment=args.environment, source_run=run,
            created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
            kind='posthoc diagnostic, same-input comparisons, not held-out paper evaluation',
            selection_rule='all environment seeds 0..9; roadmap seed 0; smallest published roadmap size; no outcome reselection',
            variants=variants, jobs=jobs, per_job_wall_limit_seconds=100,
            unchanged='objectives, original input path/endpoints/model, solvers, derivative bounds and strict validation',
            source_hashes=source,input_hashes={str(p):sha(p) for p in files},
            git_head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()))
    manifest = read(root/'manifest.json')
    if not all(sha(ROOT/name)==value for name,value in manifest['source_hashes'].items()):
        raise RuntimeError('Implementation changed; refuse resume')
    for i,job in enumerate(manifest['jobs']):
        dest = root/'jobs'/job['id']
        if (dest/'execution.json').exists():
            continue
        command = [sys.executable,'-m','reproduction.alignment_diagnosis.ablation','--output',str(root),'--worker',str(i)]
        outcome = execute_child(command,dest,manifest['per_job_wall_limit_seconds'])
        atomic(dest/'execution.json',dict(command=command,**outcome))
        if outcome['timed_out'] or outcome['exit_code']!=0:
            previous = read(dest/'result.json') if (dest/'result.json').exists() else None
            atomic(dest/'result.json',dict(case=job['case'],variant=job['variant'],
                status='timeout' if outcome['timed_out'] else 'worker_failed', previous_record=previous,**outcome))
        record=read(dest/'result.json')
        print(f"{i+1}/{len(manifest['jobs'])} {job['id']} {record['status']}",flush=True)
        summary(root)
    print(json.dumps(summary(root)),flush=True)


if __name__=='__main__':
    main()
