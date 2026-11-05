#!/usr/bin/env python3
"""Recheck actual DEV evidence through study and extended metrics readers.

This is CPU-only postprocessing. The primary run is verified in full before a
predefined six-method view is passed to extended-metrics pairing checks. No
source mock, solver, geometry query, method outcome selection or raw edit.
"""
import argparse
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[6]
sys.path.insert(0,str(ROOT))
from benchmarks.generic_motion_v1.runners import study_metrics as study
from benchmarks.generic_motion_v1.runners import extended_metrics as extended

def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args();out=args.output.resolve()
    if out.exists():ap.error('Choose a fresh output path')
    started=time.monotonic();errors=[]
    records=ROOT/'results/records.csv';summary=ROOT/'results/summary.csv'
    hashes={'records':sha(records),'summary':sha(summary)}
    run=ROOT/'results/baseline_dev'
    rows,manifests=study.load_rows([run])
    with records.open() as f:
        primary=[r for r in csv.DictReader(f) if r['run'].endswith('/baseline_dev')]
    indexed={(r['problem_id'],r['method'],r['objective'],r['planner_seed']):r for r in primary}
    for row in rows:
        key=(row['case_id'],row['method'],row['objective'],str(row['method_seed']))
        raw=indexed.get(key)
        if raw is None or raw['status']!=row['status']:
            errors.append({'identity_or_status':key})
        elif ((raw['objective_value']=='' and row['cost'] is not None)
                or (raw['objective_value']!='' and float(raw['objective_value'])!=row['cost'])):
            errors.append({'cost':key})
    chosen=extended._annotate([r for r in rows if r['label'] in extended.LABELS and r['objective']=='minimum_distance'])
    differences=extended._check_pairings(chosen)
    if differences:errors.append({'extended_shared_frontend':differences})
    pairs=study.pair_metrics(chosen)
    if len(pairs)!=15 or any(p['paired_legal_jobs']!=180 for p in pairs):
        errors.append('Extended six-method paired coverage differs from 180 fixed DEV task/repetition pairs')
    groups={label:extended._group_metrics([r for r in chosen if r['label']==label]) for label in extended.LABELS}
    if sha(records)!=hashes['records'] or sha(summary)!=hashes['summary']:errors.append('Historical CSV changed')
    report=dict(passed=not errors,errors=errors,actual_dev_observations=len(rows),
                statuses=dict(Counter(r['status'] for r in rows)),
                primary_csv_status_and_cost_rows_checked=len(rows),
                extended_predeclared_method_view=list(extended.LABELS),
                extended_actual_observations=len(chosen),extended_pair_comparisons=len(pairs),
                extended_shared_frontend_differences=differences,
                extended_groups={k:{'requested_jobs':v['requested_jobs'],'valid_success':v['valid_success'],
                                    'status_counts':v['status_counts'],'validated_incumbent_nonconverged':v['validated_incumbent_nonconverged']} for k,v in groups.items()},
                run_verification=manifests,records_sha256=hashes['records'],summary_sha256=hashes['summary'],
                script_sha256=sha(Path(__file__)),elapsed_seconds=time.monotonic()-started,
                scope='Retrospective CPU evidence accounting, no new planning score or geometry certification')
    out.parent.mkdir(parents=True,exist_ok=True)
    with out.open('x') as f:json.dump(report,f,indent=2,allow_nan=False);f.write('\n')
    print(json.dumps(report,indent=2));return bool(errors)

if __name__=='__main__':raise SystemExit(main())
