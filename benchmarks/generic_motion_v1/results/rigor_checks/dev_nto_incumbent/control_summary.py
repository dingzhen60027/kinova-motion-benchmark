#!/usr/bin/env python3
"""Receipt-verified current-source control vs enabled DEV snapshot, CPU-only.

Include all 24 planned pairs, even if execution has not started. Successful
objective/time/clearance comparisons are reported by track and scene cluster.
Always write to a fresh JSON filename; this script never resumes a planner.
"""
from __future__ import annotations
import argparse
from collections import defaultdict
from datetime import datetime,timezone
import json
import hashlib
from pathlib import Path
import subprocess
import sys

import control_driver as ctl
import summarize as sm


def historical_configuration(plan,enabled):
    """Bind original overrides and fully resolved job settings, never defaults today."""
    commit=enabled['source_code_commit']
    if plan['enabled_source_commit']!=commit:
        raise ValueError('Plan and enabled source commits differ')
    path=Path(plan['control_config'])
    if path.is_absolute() or '..' in path.parts:
        raise ValueError('Historical control configuration must be a repository path')
    payload=subprocess.check_output(['git','show',f'{commit}:{path.as_posix()}'],cwd=ctl.ROOT)
    if hashlib.sha256(payload).hexdigest()!=plan['control_config_sha256']:
        raise ValueError('Historical control configuration hash differs from plan')
    overrides=json.loads(payload)
    config=ctl.base_config(enabled['jobs'][0])
    def contains(actual,expected):
        return (isinstance(actual,dict) and all(k in actual and contains(actual[k],v) for k,v in expected.items())
                if isinstance(expected,dict) else actual==expected)
    if not isinstance(overrides,dict) or not contains(config,overrides):
        raise ValueError('Resolved control configuration differs from original overrides')
    return config


def verify_saved_plan(plan, enabled):
    """Replay geometry-only selection without requiring the old working tree.

    Running or resuming experiments still calls control_driver.assert_sources.
    This reader verifies historical source bindings with batch.verify instead.
    """
    scenes=defaultdict(lambda:defaultdict(list))
    for line in ctl.DATA.read_text().splitlines():
        case=json.loads(line)
        if case['family'] in ctl.FAMILIES:
            if case['split']!='dev' or case['validation_status']!='valid_input':
                raise ValueError('Geometry-selected family has an invalid input')
            scenes[case['family']][case['scene_id']].append(case['problem_id'])
    selected=[];lookup=ctl.enabled_lookup(enabled)
    config=historical_configuration(plan,enabled)
    for family in ctl.FAMILIES:
        if len(scenes[family])<3:raise ValueError(f'Insufficient scenes for {family}')
        for scene in sorted(scenes[family])[:3]:
            problem=min(scenes[family][scene]);jobs=[]
            for objective in ('minimum_distance','minimum_time'):
                job=lookup.get((problem,objective,0))
                if (job is None or job['scene_id']!=scene or job['family']!=family
                        or ctl.base_config(job)!=config):
                    raise ValueError('Selected enabled job or configuration differs')
                jobs.append({key:job[source] for key,source in (
                    ('objective','objective'),('enabled_job_id','job_id'),
                    ('method_seed','method_seed'),('roadmap_id','roadmap_id'),('case_ref','case_ref'))})
            selected.append(dict(family=family,scene_id=scene,problem_id=problem,jobs=jobs))
    if (plan['problems']!=selected or len(selected)!=12
            or len({p['scene_id'] for p in selected})!=12):
        raise ValueError('Plan does not match geometry-only selection')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True,help='Fresh output JSON path')
    args=parser.parse_args();out=args.output.resolve()
    if out.exists():parser.error('Output already exists; choose a fresh filename')
    plan=sm.read(ctl.PLAN);enabled=sm.read(ctl.ENABLED/'run_manifest.json')
    errors=[];rows=[];run_integrity=[]
    from benchmarks.generic_motion_v1.batch import verify
    def check_history(run):
        result=verify(run,allow_unsealed=True)
        run_integrity.append(dict(run=str(run.relative_to(ctl.ROOT)),**result))
        if result['integrity_passed'] is not True:
            raise ValueError('Historical run integrity failed: '+json.dumps(result['errors']))
    if (sm.sha(ctl.ENABLED/'run_manifest.json')!=plan['enabled_manifest_sha256']
            or sm.sha(ctl.DATA)!=plan['dev_manifest_sha256']):
        errors.append('Plan-bound manifest, inputs, or control configuration changed')
    try:
        check_history(ctl.ENABLED)
        verify_saved_plan(plan,enabled)
    except (OSError, ValueError, KeyError,subprocess.CalledProcessError) as e:
        errors.append(f'Provenance or selection verification failed: {e}')
    lookup=ctl.enabled_lookup(enabled)
    for problem in plan['problems']:
        run=ctl.CONTROL/problem['problem_id'];cm=None;controljobs={}
        if (run/'run_manifest.json').is_file():
            try:
                cm=sm.read(run/'run_manifest.json')
                check_history(run)
                ctl.control_verify(plan,problem,enabled,cm)
                controljobs={j['objective']:j for j in cm['jobs']}
            except (OSError, ValueError, KeyError, TypeError) as e:
                errors.append(f"Control manifest mismatch {problem['problem_id']}: {e}")
                cm=None
        elif run.exists():
            errors.append(f'Existing but unprepared control run: {run}')
        for meta in problem['jobs']:
            objective=meta['objective'];k=(problem['problem_id'],objective,0)
            record={'problem_id':k[0],'scene_id':problem['scene_id'],'cluster_id':problem['scene_id'],
                    'category':problem['family'],'objective':objective,'replica_seed':0}
            enabledjob=lookup[k]
            for label,runpath,manifest,job in (
                ('old',run,cm,controljobs.get(objective)),('new',ctl.ENABLED,enabled,enabledjob)):
                if manifest is None or job is None:
                    record[label+'_state']='not_started' if not runpath.exists() else 'unprepared'
                    record[label+'_status']=None
                    continue
                try:
                    info,row=sm.snapshot_attempt(runpath,manifest,job)
                    record[label+'_attempt']=info;record[label+'_state']=info['state']
                    record[label+'_status']=row['status'] if row else None
                    if row:
                        record[label]={name:row.get(name) for name in (
                            'input_sha256','robot_sha256','geometry_sha256','planner_seed','roadmap_id',
                            'frontend_path_sha256','objective_value','cold_e2e_seconds',
                            'minimum_sampled_clearance_m','validation_passed','status_basis','raw_status')}
                except (OSError,ValueError,KeyError,TypeError) as e:
                    errors.append(f'{label} receipt/integrity failure {k}: {e}')
                    record[label+'_state']='integrity_error';record[label+'_status']=None
            if record.get('old') and record.get('new'):
                for field in ('input_sha256','robot_sha256','geometry_sha256','planner_seed','roadmap_id'):
                    if record['old'].get(field)!=record['new'].get(field):errors.append(f'Pair {field} mismatch: {k}')
                a,b=record['old']['frontend_path_sha256'],record['new']['frontend_path_sha256']
                record['frontend_paths_identical']=(a==b) if a is not None and b is not None else None
            rows.append(record)
    if len(rows)!=24 or len({(r['problem_id'],r['objective'],r['replica_seed']) for r in rows})!=24:
        errors.append('Expected exactly 24 unique predeclared jobs')
    clusters=defaultdict(list)
    for r in rows:clusters[(r['cluster_id'],r['objective'])].append(r)
    compared=[r for r in rows if r.get('frontend_paths_identical') is not None]
    different=[{'problem_id':r['problem_id'],'objective':r['objective']} for r in compared if not r['frontend_paths_identical']]
    report={'created_utc':datetime.now(timezone.utc).isoformat(),
        'scope':'Geometry-selected DEV seed 0; 24 predeclared queries, selected after enabled batch began. No outcome selection.',
        'old_label':'control: early_accept_validated=false','new_label':'enabled: early_accept_validated=true',
        'plan':str(ctl.PLAN.relative_to(ctl.ROOT)),'plan_sha256':sm.sha(ctl.PLAN),
        'enabled_manifest_sha256':sm.sha(ctl.ENABLED/'run_manifest.json'),
        'source_identity':'Historical Git/source-snapshot provenance is checked by batch.verify; every prepared control manifest must retain exact equality of source and roadmap hashes to enabled manifest; each receipt and request is verified. Current working-tree source equality is required only by execution/resume, not this retrospective reader.',
        'historical_run_integrity':run_integrity,
        'selection_timing':plan['selection_timing'], 'scene_grouping':'Each chosen case comes from a distinct obstacle scene; do not pool two tracks as independent scenes.',
        'frontends':{'both_available':len(compared),'matched':len(compared)-len(different),'different':different,
                     'note':'Null when either frontend path is absent. Differences do not erase end-to-end results but prevent claiming an identical-initial-path pair.'},
        'complete':len(rows)==24 and all(r['old_state']==r['new_state']=='completed' for r in rows) and not errors,
        'coverage':sm.summarize(rows),
        'by_objective':{v:sm.summarize([r for r in rows if r['objective']==v]) for v in sorted({r['objective'] for r in rows})},
        'by_scene_cluster':[{'cluster_id':k[0],'objective':k[1],**sm.summarize(v)} for k,v in sorted(clusters.items())],
        'all_declared_jobs':rows,'errors':errors,'integrity_passed':not errors}
    out.parent.mkdir(parents=True,exist_ok=True)
    with out.open('x') as f:json.dump(report,f,indent=2,allow_nan=False);f.write('\n')
    print(json.dumps({'output':str(out),'integrity_passed':not errors,'complete':report['complete'],
                      'coverage':report['coverage'],'frontends':report['frontends'],'errors':errors},indent=2))
    return bool(errors)

if __name__=='__main__':sys.exit(main())
