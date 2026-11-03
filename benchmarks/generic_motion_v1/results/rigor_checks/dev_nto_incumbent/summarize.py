#!/usr/bin/env python3
"""CPU-only, receipt-verified paired snapshot of the incumbent DEV experiment.

The experiment is allowed to be in progress. No completed job is selected by
outcome. A snapshot must be written to a new path; input artifacts are read only.
"""
from __future__ import annotations
import argparse
from collections import Counter, defaultdict
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import statistics
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT))
from benchmarks.generic_motion_v1.runners.results import attempt_row  # noqa: E402

OLD = ROOT / 'benchmarks/generic_motion_v1/results/baseline_dev'
NEW = Path(__file__).resolve().parent / 'dev_seed0'

def read(path):
    return json.loads(path.read_text())

def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()

def counter(items):
    return dict(sorted(Counter(items).items()))

def number(x):
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)

def describe(values):
    values = [x for x in values if number(x)]
    return {'count': len(values), 'mean': statistics.mean(values) if values else None,
            'median': statistics.median(values) if values else None}

def config_check(old, new):
    a, b = deepcopy(old), deepcopy(new)
    av = a.get('nto_refined', {}).pop('early_accept_validated', False)
    bv = b.get('nto_refined', {}).pop('early_accept_validated', False)
    return av is False and bv is True and a == b

def snapshot_attempt(run, manifest, job):
    folder = run / 'jobs' / job['job_id'] / 'attempts'
    attempts = sorted(p for p in folder.glob('[0-9][0-9][0-9][0-9]') if p.is_dir())
    terminal = [p for p in attempts if (p / 'receipt.json').is_file()
                and not read(p / 'receipt.json').get('retry_pending')]
    info = {'attempts': [p.name for p in attempts],
            'terminal_attempts': [p.name for p in terminal],
            'selected_attempt': terminal[0].name if terminal else None}
    if not terminal:
        info['state'] = 'running_or_unsealed' if attempts else 'not_started'
        return info, None
    path = terminal[0]
    row = attempt_row(run, manifest, job, path, verify_files=True, case_cache={})
    if read(path / 'request.json') != job:
        raise ValueError(f'receipt-bound request differs from manifest: {path}')
    if row['receipt_integrity'] is not True:
        raise ValueError(f'receipt integrity failed: {path}')
    info['state'] = 'completed'
    info['receipt_sha256'] = row['receipt_sha256']
    info['attempt_path'] = row['attempt_path']
    return info, row

def metrics(rows):
    success = [x for x in rows if x['old_status'] == x['new_status'] == 'success_numerical']
    names = ('objective_value', 'cold_e2e_seconds', 'minimum_sampled_clearance_m')
    result = {}
    for name in names:
        pairs = [(x['old'][name], x['new'][name]) for x in success
                 if number(x['old'].get(name)) and number(x['new'].get(name))]
        result[name] = {'old': describe([a for a,b in pairs]),
                        'new': describe([b for a,b in pairs]),
                        'difference_new_minus_old': describe([b-a for a,b in pairs])}
    return result

def summarize(rows):
    done = [x for x in rows if x['new_state'] == 'completed' and x['old_state'] == 'completed']
    objectives = {x['objective'] for x in rows}
    return {'planned': len(rows), 'paired_completed': len(done),
            'new_states': counter(x['new_state'] for x in rows),
            'old_states': counter(x['old_state'] for x in rows),
            'old_statuses': counter(x['old_status'] for x in done),
            'new_statuses': counter(x['new_status'] for x in done),
            'status_transitions_old_to_new': counter(f"{x['old_status']} -> {x['new_status']}" for x in done),
            'common_success_metrics': metrics(done) if len(objectives) == 1 else None,
            'metrics_note': 'Metrics are separated by objective because distance and time have different units.' if len(objectives) != 1 else None}

def git_validator_diff(old, new):
    path = 'benchmarks/generic_motion_v1/runners/validate.py'
    commands = [['git','show',f'{commit}:{path}'] for commit in (old['source_code_commit'],new['source_code_commit'])]
    try:
        source = [subprocess.check_output(cmd,cwd=ROOT,stderr=subprocess.PIPE) for cmd in commands]
        diff = subprocess.check_output(['git','diff','--no-ext-diff',old['source_code_commit'],
                                         new['source_code_commit'],'--',path],cwd=ROOT,stderr=subprocess.PIPE).decode()
        return {'path':path, 'git_source_sha256':[hashlib.sha256(s).hexdigest() for s in source],
                'manifest_sha256':[old['source_hashes'].get(path),new['source_hashes'].get(path)],
                'git_sources_match_manifest':all(hashlib.sha256(s).hexdigest()==m['source_hashes'].get(path)
                                                 for s,m in zip(source,(old,new))),
                'diff':diff,
                'interpretation':'The changed branch splits out the already-rejected nonpositive duration; valid-candidate validation code is unchanged. Failure classification also depends on runner changes. Treat comparison as observational, not fully controlled.',
                'revalidation':'Saved candidate.npz + candidate.json with fixed inputs/config can be passed to current validate(); it calls both model.cuda and CPU clearance, so schedule serially after active GPU jobs. This snapshot does not claim revalidation.'}
    except (OSError,subprocess.CalledProcessError,UnicodeDecodeError) as e:
        return {'path':path,'error':str(e),'interpretation':'Source comparison unavailable; treat results as observational.'}

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True,type=Path,help='New JSON path; never overwrites')
    args=parser.parse_args()
    out=args.output.resolve()
    if out.exists():
        parser.error('Output already exists; choose a fresh filename')
    oldm,newm=read(OLD/'run_manifest.json'),read(NEW/'run_manifest.json')
    errors=[]
    key=lambda j:(j['case_id'],j['objective'],j['replica_seed'])
    oldjobs=[j for j in oldm['jobs'] if j['method']=='NTO_REFINED' and j['replica_seed']==0]
    newjobs=newm['jobs']
    lookup={key(j):j for j in oldjobs}
    if len(lookup)!=len(oldjobs) or len({key(j) for j in newjobs})!=len(newjobs):
        errors.append('Duplicate paired identity in run manifest')
    if set(lookup)!={key(j) for j in newjobs}:
        errors.append('Old and new declared job sets differ')
    if oldm['manifest_sha256']!=newm['manifest_sha256'] or oldm['split']!='dev' or newm['split']!='dev':
        errors.append('Input manifests or dev split differ')
    rows=[]
    for job in newjobs:
        k=key(job); oldjob=lookup.get(k)
        record={'problem_id':k[0], 'objective':k[1], 'replica_seed':k[2],
                'scene_id':job['scene_id'], 'category':job['family'],
                'cluster_id':'open_control_shared_base_world' if job['family']=='open_control' else job['scene_id']}
        if oldjob is None:
            errors.append(f'Missing historical job: {k}'); record.update(old_state='missing_job',new_state='unknown',old_status=None,new_status=None); rows.append(record);continue
        if any(oldjob.get(field)!=job.get(field) for field in set(oldjob)|set(job) if field!='evaluation_config'):
            errors.append(f'Non-configuration job identity differs: {k}')
        if not config_check(oldjob['evaluation_config'],job['evaluation_config']):
            errors.append(f'Configuration differs beyond early_accept_validated: {k}')
        for label, run, manifest, entry in (('old',OLD,oldm,oldjob),('new',NEW,newm,job)):
            try:
                info,row=snapshot_attempt(run,manifest,entry)
                record[label+'_attempt']=info;record[label+'_state']=info['state']
                record[label+'_status']=row['status'] if row else None
                if row:
                    record[label]={name:row.get(name) for name in ('input_sha256','robot_sha256','geometry_sha256','planner_seed','roadmap_id','frontend_path_sha256','objective_value','cold_e2e_seconds','minimum_sampled_clearance_m','validation_passed','status_basis','raw_status')}
            except (OSError,ValueError,KeyError,TypeError) as e:
                errors.append(f'{label} integrity failure {k}: {e}')
                record[label+'_state']='integrity_error';record[label+'_status']=None
        if record.get('old') and record.get('new'):
            for field in ('input_sha256','robot_sha256','geometry_sha256','planner_seed','roadmap_id'):
                if record['old'].get(field)!=record['new'].get(field):
                    errors.append(f'Paired {field} mismatch: {k}')
            a,b=record['old']['frontend_path_sha256'],record['new']['frontend_path_sha256']
            record['frontend_paths_identical']=(a==b) if a is not None and b is not None else None
        rows.append(record)
    by_objective={v:summarize([r for r in rows if r['objective']==v]) for v in sorted({r['objective'] for r in rows})}
    clusters=defaultdict(list)
    for r in rows:clusters[(r['cluster_id'],r['objective'])].append(r)
    by_cluster=[{'cluster_id':k[0],'objective':k[1],**summarize(v)} for k,v in sorted(clusters.items())]
    src={k:{'old':oldm['source_hashes'].get(k),'new':newm['source_hashes'].get(k)}
         for k in sorted(set(oldm['source_hashes'])|set(newm['source_hashes']))
         if oldm['source_hashes'].get(k)!=newm['source_hashes'].get(k)}
    report={'created_utc':datetime.now(timezone.utc).isoformat(),
            'scope':'DEV, seed 0, all declared NTO_REFINED jobs. First terminal attempt; no outcome selection. Partial snapshots are not final scores.',
            'old_run':str(OLD.relative_to(ROOT)), 'new_run':str(NEW.relative_to(ROOT)),
            'run_manifest_sha256':{'old':sha(OLD/'run_manifest.json'),'new':sha(NEW/'run_manifest.json')},
            'source_commits':{'old':oldm['source_code_commit'],'new':newm['source_code_commit']},
            'differing_source_hashes':src,'validator_source_comparison':git_validator_diff(oldm,newm),
            'configuration_check':'Same full job identity and configuration except early_accept_validated false -> true',
            'scene_grouping':'open_control layouts share one base-world cluster; other problems grouped by scene_id. No independent-scene uncertainty inferred from query counts.',
            'units':'objective_value is rad for minimum_distance and seconds for minimum_time; cold_e2e is seconds; minimum sampled clearance is metres',
            'coverage':summarize(rows),'by_objective':by_objective,'by_scene_cluster':by_cluster,
            'all_declared_jobs':rows,'errors':errors,'integrity_passed':not errors}
    out.parent.mkdir(parents=True,exist_ok=True)
    with out.open('x') as f:json.dump(report,f,indent=2,allow_nan=False);f.write('\n')
    print(json.dumps({'output':str(out),'integrity_passed':not errors,'errors':errors,'coverage':report['coverage']},indent=2))
    return bool(errors)

if __name__=='__main__':
    sys.exit(main())
