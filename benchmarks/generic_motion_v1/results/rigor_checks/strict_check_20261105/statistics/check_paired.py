#!/usr/bin/env python3
"""Independently recompute main paired statistics from the published records."""
import argparse
from collections import Counter
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics

ROOT=Path(__file__).resolve().parents[6]
COHORTS={'current_smoke_repaired','current_smoke_repaired_fixed_frontend',
         'current_dev_repaired','current_dev_repaired_fixed_frontend',
         'current_test_repaired','current_test_repaired_fixed_frontend',
         'current_test_repaired_initial_regions'}

def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args();out=args.output.resolve()
    if out.exists():ap.error('Choose a fresh output path')
    records=ROOT/'results/records.csv';summary=ROOT/'results/summary.csv'
    with records.open() as f: rows=list(csv.DictReader(f))
    with summary.open() as f: summaries=list(csv.DictReader(f))
    groups={};errors=[];checked=Counter()
    for row in rows:
        if row['input_valid']!='True':continue
        for cohort in COHORTS.intersection(json.loads(row['selected_comparisons'])):
            for category in ['all',row['category']]:
                key=(cohort,row['split'],row['objective'],category,row['method'])
                group=groups.setdefault(key,{})
                pair=(row['problem_id'],row['replica_seed'])
                if pair in group:errors.append({'duplicate_pair':key,'problem':pair})
                group[pair]=row
    for row in summaries:
        if row['comparison'] not in COHORTS or row['row_type'] not in ('paired','paired_outcome'):continue
        key=tuple(row[k] for k in ('comparison','split','objective','category'))
        a=groups.get(key+(row['method'],),{});b=groups.get(key+(row['method_b'],),{})
        pairs=[(a[k],b[k]) for k in a.keys()&b.keys()]
        for left,right in pairs:
            if any(left[k]!=right[k] for k in ('input_sha256','planner_seed','cluster_id')):
                errors.append({'unpaired_identity':key})
        if row['row_type']=='paired_outcome':
            states={'both_success':(True,True),'a_only_success':(True,False),
                    'b_only_success':(False,True),'neither_success':(False,False)}
            state=states[row['metric']]
            vals=[int((a['status']=='success_numerical',b['status']=='success_numerical')==state) for a,b in pairs]
            if int(row['outcome_count'])!=sum(vals):errors.append({'outcome_count':key,'metric':row['metric']})
            clusters={a['cluster_id'] for a,b in pairs}
        else:
            if row['scope'].startswith('common_success_'):
                pairs=[(a,b) for a,b in pairs if a['status']==b['status']=='success_numerical']
            field=row['metric'];vals=[];clusters=set()
            for a,b in pairs:
                if field=='success':x,y=int(a['status']=='success_numerical'),int(b['status']=='success_numerical')
                else:
                    if a[field]=='' or b[field]=='':continue
                    x,y=float(a[field]),float(b[field])
                if not math.isfinite(x) or not math.isfinite(y):continue
                vals.append(x-y);clusters.add(a['cluster_id'])
            expected=statistics.median(vals) if vals else None
            if ((expected is None and row['median']!='') or (expected is not None and not math.isclose(float(row['median']),expected,rel_tol=1e-10,abs_tol=1e-12))):
                errors.append({'paired_median':key,'metric':field,'scope':row['scope']})
        if int(row['count'])!=len(vals):errors.append({'paired_count':key,'metric':row['metric']})
        expected=statistics.mean(vals) if vals else None
        if ((expected is None and row['mean']!='') or (expected is not None and not math.isclose(float(row['mean']),expected,rel_tol=1e-10,abs_tol=1e-12))):
            errors.append({'paired_mean':key,'metric':row['metric'],'scope':row['scope']})
        if int(row['scene_clusters'])!=len(clusters):errors.append({'paired_clusters':key})
        if len(clusters)<2 and (row['ci_low'] or row['ci_high']):errors.append({'spurious_ci':key})
        checked[row['row_type']]+=1
    report=dict(passed=not errors,checked=dict(checked),errors=errors,
                scope='All paired means, medians, counts, identity matching and scene counts in current repaired smoke/dev/test/fixed-initial-region cohorts; no new geometry queries',
                records_sha256=sha(records),summary_sha256=sha(summary),script_sha256=sha(Path(__file__)))
    out.parent.mkdir(parents=True,exist_ok=True)
    with out.open('x') as f:json.dump(report,f,indent=2,allow_nan=False);f.write('\n')
    print(json.dumps(report,indent=2));return bool(errors)

if __name__=='__main__':raise SystemExit(main())
