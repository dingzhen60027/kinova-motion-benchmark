#!/usr/bin/env python3
"""Create a geometry-selected DEV plan, or execute/resume its controls serially.

--write-plan reads manifests only and never runs a planner. --execute invokes
public evaluate.py prepare/resume; it must not run concurrently with enabled
GPU jobs. Every control uses the current source/configuration paired with the
already running enabled experiment; the only intentional setting difference is
nto_refined.early_accept_validated.
"""
from __future__ import annotations
import argparse
from collections import defaultdict
from copy import deepcopy
from datetime import datetime, timezone
import fcntl
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0,str(ROOT))
from benchmarks.generic_motion_v1.roadmap import load_config  # noqa: E402
from benchmarks.generic_motion_v1.runners.results import run_status  # noqa: E402

HERE=Path(__file__).resolve().parent
ENABLED=HERE/'dev_seed0'
PLAN=HERE/'control_plan.json'
CONTROL=HERE/'control'
LOGS=HERE/'control_logs'
CONFIG=ROOT/'configs/evaluation.json'
DATA=ROOT/'benchmarks/generic_motion_v1/manifests/dev.jsonl'
FAMILIES=('bottleneck','dense_clutter','route_choice','sparse_clutter')

def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for chunk in iter(lambda:f.read(1024*1024),b''):h.update(chunk)
    return h.hexdigest()

def read(path):
    return json.loads(path.read_text())

def key(job):
    return (job['case_id'],job['objective'],job['replica_seed'])

def base_config(job):
    value=deepcopy(job['evaluation_config'])
    if value['nto_refined'].pop('early_accept_validated',None) is not True:
        raise ValueError('Enabled job does not explicitly turn on early acceptance')
    return value

def assert_sources(manifest):
    bad=[]
    for name,expected in manifest['source_hashes'].items():
        p=Path(name);p=p if p.is_absolute() else ROOT/p
        if not p.is_file() or sha(p)!=expected:bad.append(name)
    if bad:raise ValueError('Current source differs from enabled manifest: '+', '.join(bad))

def enabled_lookup(manifest):
    jobs=manifest['jobs']
    if (manifest['split']!='dev' or manifest['formal_test_run'] is not False or len(jobs)!=120
        or any(j['method']!='NTO_REFINED' or j['replica_seed']!=0 for j in jobs)):
        raise ValueError('Expected the complete 120-job DEV enabled seed-0 declaration')
    result={key(j):j for j in jobs}
    if len(result)!=len(jobs):raise ValueError('Duplicate enabled job key')
    return result

def plan_create():
    manifest=read(ENABLED/'run_manifest.json');lookup=enabled_lookup(manifest)
    if sha(DATA)!=manifest['manifest_sha256']:raise ValueError('DEV input manifest hash mismatch')
    config=load_config(read(CONFIG))
    if config.get('nto_refined',{}).get('early_accept_validated',False) is not False:
        raise ValueError('Control config must disable early acceptance')
    assert_sources(manifest)
    scenes=defaultdict(lambda:defaultdict(list))
    for line in DATA.read_text().splitlines():
        case=json.loads(line)
        if case['family'] in FAMILIES:
            if case['validation_status']!='valid_input' or case['split']!='dev':
                raise ValueError('A geometry-selected family record is invalid; do not silently filter it')
            scenes[case['family']][case['scene_id']].append(case['problem_id'])
    selected=[]
    for family in FAMILIES:
        if len(scenes[family])<3:raise ValueError(f'Insufficient scenes for {family}')
        for scene in sorted(scenes[family])[:3]:
            problem=min(scenes[family][scene]);jobs=[]
            for objective in ('minimum_distance','minimum_time'):
                job=lookup.get((problem,objective,0))
                if job is None or job['scene_id']!=scene or job['family']!=family:
                    raise ValueError(f'No unique matching enabled job for {problem}, {objective}')
                if base_config(job)!=config:raise ValueError(f'Non-target config difference: {problem}, {objective}')
                jobs.append({'objective':objective,'enabled_job_id':job['job_id'],
                             'method_seed':job['method_seed'], 'roadmap_id':job['roadmap_id'],
                             'case_ref':job['case_ref']})
            selected.append({'family':family,'scene_id':scene,'problem_id':problem,'jobs':jobs})
    if len(selected)!=12 or len({s['scene_id'] for s in selected})!=12:
        raise ValueError('Expected 12 distinct physical obstacle scenes')
    # Counts capture progress, not outcomes, and never influence selection.
    done=running=unstarted=0
    for job in manifest['jobs']:
        folder=ENABLED/'jobs'/job['job_id']/'attempts'
        attempts=list(folder.glob('[0-9][0-9][0-9][0-9]'))
        if any((p/'receipt.json').is_file() and not read(p/'receipt.json').get('retry_pending') for p in attempts):done+=1
        elif attempts:running+=1
        else:unstarted+=1
    return {'schema_version':1,'created_utc':datetime.now(timezone.utc).isoformat(),
       'scope':'DEV control, all declared 12 problems x 2 objectives, replica seed 0; no test jobs',
       'selection_rule':'For each of four obstacle families, sort distinct scene_id lexicographically, take first three, and take lexicographically first problem_id per scene. No planner outcome or trajectory is used.',
       'families':list(FAMILIES),'open_control':'Excluded: shared base world is not an independent added-obstacle scene.',
       'selection_timing':'Selection was defined after the enabled full-DEV seed-0 experiment had already partially run; it is geometry-only but was not prospective to that run.',
       'enabled_progress_observed_at_plan':{'declared':len(manifest['jobs']),'completed':done,'running_or_unsealed':running,'not_started':unstarted},
       'enabled_manifest':'benchmarks/generic_motion_v1/results/rigor_checks/dev_nto_incumbent/dev_seed0/run_manifest.json',
       'enabled_manifest_sha256':sha(ENABLED/'run_manifest.json'),
       'dev_manifest_sha256':sha(DATA),'control_config':'configs/evaluation.json','control_config_sha256':sha(CONFIG),
       'enabled_source_commit':manifest['source_code_commit'],
       'source_condition':'All recorded enabled source SHA-256 hashes must match current files and each prepared control manifest.',
       'protocol':'60 s end-to-end; first terminal attempt; public CLI; same inputs, seeds, collision checker and numerical validator; disabled early_accept_validated.',
       'problem_count':len(selected),'planned_queries':2*len(selected),'problems':selected}

def invoke(command, label):
    LOGS.mkdir(parents=True,exist_ok=True)
    path=LOGS/(label+'-'+str(time.time_ns())+'.log')
    with path.open('x') as stream:
        stream.write('command: '+json.dumps(command,ensure_ascii=False)+'\n');stream.flush()
        result=subprocess.run(command,cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT,check=False)
        stream.write('\nreturncode: '+str(result.returncode)+'\n')
    print(f'{label}: exit {result.returncode}; log {path.relative_to(ROOT)}',flush=True)
    if result.returncode:raise RuntimeError(f'Public CLI failed; inspect {path}')

def control_verify(plan, chosen, enabled_manifest, control_manifest):
    if control_manifest['source_hashes']!=enabled_manifest['source_hashes']:
        raise ValueError('Control source hashes differ from enabled run')
    if control_manifest['manifest_sha256']!=enabled_manifest['manifest_sha256'] or control_manifest['timeout_seconds']!=enabled_manifest['timeout_seconds']:
        raise ValueError('Input identity or shared outer budget differs')
    if control_manifest['roadmap_files']!=enabled_manifest['roadmap_files']:
        raise ValueError('Control roadmap artifact hashes differ from enabled run')
    old=enabled_lookup(enabled_manifest)
    jobs=control_manifest['jobs']
    if len(jobs)!=2 or {j['objective'] for j in jobs}!={'minimum_distance','minimum_time'}:
        raise ValueError('Prepared control must contain exactly one job per track')
    for j in jobs:
        other=old[key(j)]
        if j['case_id']!=chosen['problem_id'] or j['evaluation_config']!=base_config(other):
            raise ValueError('Control job case or configuration mismatch')
        if any(j.get(k)!=other.get(k) for k in set(j)|set(other) if k!='evaluation_config'):
            raise ValueError('Control job identity differs from enabled job')

def execute():
    plan=read(PLAN);manifest=read(ENABLED/'run_manifest.json')
    if (sha(ENABLED/'run_manifest.json')!=plan['enabled_manifest_sha256'] or sha(DATA)!=plan['dev_manifest_sha256']
            or sha(CONFIG)!=plan['control_config_sha256']):
        raise ValueError('Plan-bound manifest, dataset or config changed')
    assert_sources(manifest)
    if plan['problems']!=plan_create()['problems']:
        raise ValueError('Plan no longer matches the declared geometry-only selection rule')
    # The active enabled public runner holds this file lock for its whole run.
    # Retain it throughout controls to prevent concurrent enabled resume calls.
    with (ENABLED/'.resume.lock').open('a+') as lock:
        try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:raise RuntimeError('Enabled GPU batch still holds its resume lock; try after it finishes')
        for chosen in plan['problems']:
            problem=chosen['problem_id'];run=CONTROL/problem
            if not run.exists():
                command=['bash','local/python.sh','scripts/evaluate.py','run','--split','dev','--problem',problem,
                         '--methods','NTO_REFINED','--track','both','--seeds','0','--config',str(CONFIG.relative_to(ROOT)),
                         '--output',str(run.relative_to(ROOT)),'--prepare-only']
                invoke(command,problem+'-prepare')
            if not (run/'run_manifest.json').is_file():
                raise ValueError(f'Existing control is unprepared; inspect {run}')
            control_verify(plan,chosen,manifest,read(run/'run_manifest.json'))
            status=run_status(run)
            if status['completed']!=status['planned']:
                invoke(['bash','local/python.sh','scripts/evaluate.py','resume','--run',str(run.relative_to(ROOT))],problem+'-resume')
                status=run_status(run)
            print(json.dumps({'problem':problem,'status':status},ensure_ascii=False),flush=True)
            if status['completed']!=2 or status['pending']!=0:
                raise RuntimeError(f'Control run incomplete: {problem}')

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    mode=parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--write-plan',action='store_true',help='Read manifests only; never run a planner')
    mode.add_argument('--execute',action='store_true',help='Prepare and resume controls, serial GPU use')
    args=parser.parse_args()
    if args.write_plan:
        plan=plan_create()
        with PLAN.open('x') as f:json.dump(plan,f,indent=2);f.write('\n')
        print(json.dumps({'plan':str(PLAN.relative_to(ROOT)),'problems':plan['problem_count'],
                          'queries':plan['planned_queries'],'enabled_progress':plan['enabled_progress_observed_at_plan']},indent=2))
    else:execute()

if __name__=='__main__':main()
