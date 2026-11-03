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
from pathlib import Path
import sys

import control_driver as ctl
import summarize as sm


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True,help='Fresh output JSON path')
    args=parser.parse_args();out=args.output.resolve()
    if out.exists():parser.error('Output already exists; choose a fresh filename')
    plan=sm.read(ctl.PLAN);enabled=sm.read(ctl.ENABLED/'run_manifest.json')
    errors=[];rows=[]
    if (sm.sha(ctl.ENABLED/'run_manifest.json')!=plan['enabled_manifest_sha256']
            or sm.sha(ctl.DATA)!=plan['dev_manifest_sha256'] or sm.sha(ctl.CONFIG)!=plan['control_config_sha256']):
        errors.append('Plan-bound manifest, inputs, or control configuration changed')
    try:
        ctl.assert_sources(enabled)
        if plan['problems']!=ctl.plan_create()['problems']:
            errors.append('Plan does not match geometry-only selection')
    except (OSError, ValueError, KeyError) as e:
        errors.append(f'Provenance or selection verification failed: {e}')
    lookup=ctl.enabled_lookup(enabled)
    for problem in plan['problems']:
        run=ctl.CONTROL/problem['problem_id'];cm=None;controljobs={}
        if (run/'run_manifest.json').is_file():
            try:
                cm=sm.read(run/'run_manifest.json')
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
        'source_identity':'Every prepared control manifest is checked for exact equality of source and roadmap hashes to enabled manifest; each receipt and request is verified.',
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
