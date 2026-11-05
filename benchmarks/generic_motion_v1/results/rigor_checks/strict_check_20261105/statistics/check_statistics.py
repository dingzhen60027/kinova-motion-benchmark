#!/usr/bin/env python3
"""Read-only CPU check of published cohorts and saved acceptance evidence.

No planner, collision checker, or historical artifact is modified. Output must
be a fresh JSON path. The shared saved-output reader checks every selected
attempt; counts, means, medians, endpoint lower bounds and cohort selection are
also independently checked against the published CSVs.
"""
from __future__ import annotations
import argparse
from collections import Counter, defaultdict
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[6]
sys.path.insert(0, str(ROOT))
from benchmarks.generic_motion_v1.common import read, resolve_case, sha
from benchmarks.generic_motion_v1.runners import results

COHORTS = {
    'current_smoke_repaired', 'current_smoke_repaired_fixed_frontend',
    'current_dev_repaired', 'current_dev_repaired_fixed_frontend',
    'current_test_repaired', 'current_test_repaired_fixed_frontend',
    'current_test_repaired_initial_regions',
}
FIELDS = ('record_id', 'run', 'job_id', 'attempt_number', 'lifecycle', 'problem_id',
          'scene_id', 'cluster_id', 'split', 'category', 'method', 'objective',
          'planner_seed', 'replica_seed', 'input_valid', 'status', 'raw_status',
          'validation_passed', 'budget_seconds', 'cold_e2e_seconds',
          'objective_value', 'path_length_rad', 'trajectory_duration_s',
          'backend_seconds', 'frontend_validated', 'frontend_path_sha256',
          'initial_regions_sha256', 'config_sha256', 'validation_protocol_sha256',
          'input_sha256', 'robot_sha256', 'geometry_sha256', 'run_manifest_sha256',
          'receipt_sha256', 'receipt_integrity')

def table(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))

def cell(value):
    if value is None:
        return ''
    if isinstance(value, (dict, list, tuple)):
        return json.dumps(value, sort_keys=True, separators=(',', ':'))
    return str(value)

def close(a, b):
    return (a == b) if a == '' or b == '' else math.isclose(float(a), float(b), rel_tol=1e-10, abs_tol=1e-12)

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', required=True, type=Path)
    args=p.parse_args(); out=args.output.resolve()
    if out.exists(): p.error('Output exists; choose a fresh path')
    started=time.monotonic()
    paths={name:ROOT/'results'/name for name in ('records.csv','summary.csv','selection.json')}
    original_hash={name:sha(path) for name,path in paths.items()}
    raw=table(paths['records.csv']); summaries=table(paths['summary.csv'])
    specs={v['id']:v for v in read(paths['selection.json'])['comparisons']}
    errors=[]; warnings=[]; checks=Counter(); actual=[]; cache={}; manifests={}
    selected=[r for r in raw if COHORTS.intersection(json.loads(r['selected_comparisons']))]
    if len({r['record_id'] for r in raw}) != len(raw): errors.append('Duplicate record_id')
    for i,row in enumerate(selected):
        run=(ROOT/row['run']).resolve()
        if run not in manifests:
            manifest=read(run/'run_manifest.json')
            manifests[run]=(manifest,{j['job_id']:j for j in manifest['jobs']})
            if read(run/'identity.json')['run_manifest_sha256']!=sha(run/'run_manifest.json'):
                errors.append({'run_identity':str(run)})
        manifest,jobs=manifests[run];job=jobs[row['job_id']]
        attempt=ROOT/row['attempt_path']
        rebuilt=results.attempt_row(run,manifest,job,attempt,verify_files=True,case_cache=cache)
        different={k:[row[k],cell(rebuilt[k])] for k in FIELDS if row[k]!=cell(rebuilt[k])}
        if different: errors.append({'record':row['record_id'],'differences':different})
        terminals=[q for q in sorted(attempt.parent.glob('[0-9][0-9][0-9][0-9]'))
                   if (q/'receipt.json').is_file() and not read(q/'receipt.json').get('retry_pending')]
        if not terminals or terminals[0]!=attempt:
            errors.append({'not_first_terminal':str(attempt)})
        if rebuilt['receipt_integrity'] is not True: errors.append({'receipt':str(attempt)})
        case=resolve_case(job,cache)
        if rebuilt['status']==results.SUCCESS:
            checks['numerical_successes']+=1
            import numpy as np
            displacement=np.abs(np.asarray(case['q_goal'])-np.asarray(case['q_start']))
            if rebuilt['objective']=='minimum_distance':
                bound=float(np.linalg.norm(displacement))
            else:
                v=np.asarray(case['velocity_limits_rad_s']);a=np.asarray(case['acceleration_limits_rad_s2'])
                bound=float(np.max(np.where(displacement<=v*v/a,2*np.sqrt(displacement/a),displacement/v+v/a)))
            if rebuilt['objective_value']+1e-5 < bound:
                errors.append({'violates_endpoint_lower_bound':row['record_id'],'bound':bound,'cost':rebuilt['objective_value']})
            if rebuilt['raw_status']=='solver_failed': checks['validated_nonconverged']+=1
            if rebuilt['cold_e2e_seconds']>rebuilt['budget_seconds']: errors.append({'late_success':row['record_id']})
        elif row['objective_value'] or row['trajectory_duration_s'] or row['path_length_rad']:
            errors.append({'failure_has_objective':row['record_id']})
        if row['first_valid_seconds'] and not json.loads(row['first_valid_time_evidence']):
            errors.append({'unsubstantiated_first_time':row['record_id']})
        rebuilt['selected_comparisons']=json.loads(row['selected_comparisons']);actual.append(rebuilt)
        checks['verified_attempts']+=1
        if (i+1)%500==0: print(json.dumps({'verified':i+1,'total':len(selected),'seconds':time.monotonic()-started}),flush=True)
    cohort_reports=[]
    for name in sorted(COHORTS):
        group=[r for r in actual if name in r['selected_comparisons']]
        keys=[(r['problem_id'],r['replica_seed'],r['method'],r['objective']) for r in group]
        if len(keys)!=len(set(keys)): errors.append({'duplicate_observation':name})
        if name.endswith('fixed_frontend'):
            identity=defaultdict(set)
            for r in group:
                if r['frontend_validated'] is not True: errors.append({'unvalidated_frontend':name,'record':r['record_id']})
                identity[r['problem_id'],r['replica_seed'],r['objective']].add(r['frontend_path_sha256'])
            if any(len(v)!=1 or None in v for v in identity.values()): errors.append({'different_frontends':name})
        st=[r for r in summaries if r['comparison']==name and r['row_type']=='status']
        for summary in st:
            g=[r for r in group if r['split']==summary['split'] and r['objective']==summary['objective'] and r['method']==summary['method'] and (summary['category']=='all' or r['category']==summary['category'])]
            legal=[r for r in g if r['input_valid']]
            counts=Counter(r['status'] for r in legal)
            for status in set(results.STATUS.values())-{'invalid_input'}:
                if int(summary[status])!=counts[status]: errors.append({'summary_status':name,'method':summary['method'],'status':status})
            if int(summary['finished_legal_jobs'])!=len(legal): errors.append({'finished_count':name})
            estimate=sum(r['status']==results.SUCCESS for r in legal)/len(legal) if legal else None
            if not close(summary['success_rate'],cell(estimate)): errors.append({'rate':name})
            clusters=len({r['cluster_id'] for r in legal})
            if int(summary['scene_clusters'])!=clusters: errors.append({'cluster_count':name})
            if summary['category']=='open_control' and clusters==1 and (summary['success_ci_low'] or summary['success_ci_high']):
                errors.append({'single_world_ci':name})
        for summary in [r for r in summaries if r['comparison']==name and r['row_type']=='metric']:
            g=[r for r in group if r['input_valid'] and r['objective']==summary['objective'] and r['method']==summary['method'] and (summary['category']=='all' or r['category']==summary['category'])]
            scope=summary['scope']
            if scope in ('validated_success_only_descriptive','success_numerical_only'):g=[r for r in g if r['status']==results.SUCCESS]
            elif scope=='failed_legal_only':g=[r for r in g if r['status']!=results.SUCCESS]
            values=[r.get(summary['metric']) for r in g if results._finite(r.get(summary['metric']))]
            if int(summary['count'])!=len(values): errors.append({'metric_count':name,'method':summary['method'],'metric':summary['metric'],'scope':scope})
            for key,value in [('mean',statistics.mean(values) if values else None),('median',statistics.median(values) if values else None)]:
                if not close(summary[key],cell(value)): errors.append({'metric_'+key:name,'method':summary['method'],'metric':summary['metric'],'scope':scope})
        cohort_reports.append({'comparison':name,'selected':len(group),'legal':sum(r['input_valid'] for r in group),'statuses':dict(Counter(r['status'] for r in group)),'runtime_commits':sorted({r['source_commit'] for r in group})})
    legacy_probe=None
    example=next((r for r in actual if r['status']==results.SUCCESS and r['raw_status']=='solver_failed'),None)
    if example:
        from benchmarks.generic_motion_v1.runners.summarize import _row
        run=ROOT/example['run'];manifest,jobs=manifests[run]
        old=_row(run,jobs[example['job_id']],{})
        legacy_probe={'attempt_path':example['attempt_path'],'shared_status':example['status'],'shared_objective':example['objective_value'],'legacy_status':old['status'],'legacy_success':old['success'],'legacy_cost':old['cost'],'scope':'python -m benchmarks.generic_motion_v1 summarize --run still uses legacy reader; primary CSV unaffected'}
        if old['success'] is not True:
            warnings.append('Legacy per-run summary drops independently validated nonconverged candidates; public unified result reader accepts them correctly')
    if any(sha(paths[name])!=digest for name,digest in original_hash.items()): errors.append('Historical derived inputs changed during check')
    report={'passed':not errors,'scope':'CPU-only primary repaired cohorts; no new geometry validation or planning; saved arrays and artifact receipts rechecked',
            'commit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
            'source_hashes':{str(Path(__file__).resolve().relative_to(ROOT)):sha(Path(__file__))},
            'input_hashes':original_hash,'checks':dict(checks),'cohorts':cohort_reports,
            'legacy_interface_probe':legacy_probe,'warnings':warnings,'errors':errors,
            'elapsed_seconds':time.monotonic()-started}
    out.parent.mkdir(parents=True,exist_ok=True)
    with out.open('x') as f: json.dump(report,f,indent=2,allow_nan=False);f.write('\n')
    print(json.dumps(report,indent=2));return bool(errors)

if __name__=='__main__':sys.exit(main())
