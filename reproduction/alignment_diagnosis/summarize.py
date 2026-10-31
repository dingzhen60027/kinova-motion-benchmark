"""Read-only same-case comparisons and exact baseline replay audit."""
import argparse
import csv
import hashlib
from pathlib import Path
import numpy as np
from reproduction.common import ROOT, atomic, read


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def csv_write(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', newline='') as file:
        if rows:
            writer=csv.DictWriter(file,fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',type=Path,action='append',required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    baseline,paired,summary,case_rows=[],[],[],[]
    inventory={}
    checks=[]
    def capture(path):
        inventory[str(path)]=sha(path)
        return read(path)
    for root in args.run:
        root=root.resolve()
        manifest=capture(root/'manifest.json')
        old=Path(manifest['source_run'])
        env=manifest['environment']
        records={}
        for job in manifest['jobs']:
            dest=root/'jobs'/job['id']
            result=capture(dest/'result.json')
            execution=capture(dest/'execution.json')
            method='NTO_'+job['variant']['problem']
            records[(job['case'],job['variant']['name'])]=result
            opt=capture(dest/'optimization_00.json') if (dest/'optimization_00.json').exists() else {}
            case_rows.append(dict(environment=env,case=job['case'],variant=job['variant']['name'],
                status=result['status'],solver_success=result.get('solver_success'),
                collision_free=result.get('collision_free'),strict_success=result['status']=='success_numerical',
                cost=result.get('cost'),controls=result.get('control_points'),
                nc=opt.get('collocation_points'),clearance=job['variant']['clearance'],
                max_collocation_residual=result.get('max_collocation_residual'),
                timed_out=execution['timed_out'],wall_seconds=result.get('wall_seconds')))
            if job['variant']['name'].endswith('_baseline'):
                oldmeta=capture(old/'cases'/job['case']/method/'optimization_00.json')
                oldresult=capture(old/'cases'/job['case']/method/'result.json')
                arraypaths=[old/'cases'/job['case']/method/'candidate_00.npz',dest/'candidate_00.npz']
                for p in arraypaths:inventory[str(p)]=sha(p)
                arrays=[np.load(p) for p in arraypaths]
                differences={key:float(np.max(np.abs(arrays[0][key]-arrays[1][key])))
                             if arrays[0][key].shape==arrays[1][key].shape else None
                             for key in ['controls','knots']}
                baseline.append(dict(environment=env,case=job['case'],method=method,
                    status_equal=result['status']==oldresult['status'],
                    solver_success_equal=opt.get('solver_success')==oldmeta['solver_success'],
                    array_shapes_equal=all(v is not None for v in differences.values()),
                    cost_difference=result.get('cost',0)-oldresult.get('cost',0),
                    controls_max_difference=differences['controls'],knots_max_difference=differences['knots']))
        for spec in manifest['variants']:
            rows=[(case,r) for (case,name),r in records.items() if name==spec['name']]
            ok=[r for _,r in rows if r['status']=='success_numerical']
            comparisons=[]
            for case,r in rows:
                if spec['problem']!='time' or r['status']!='success_numerical':continue
                base=records[(case,'time_baseline')]
                scsto=capture(old/'cases'/case/'SCSTO'/'result.json')
                for ref_name,ref in [('baseline_NTO_time',base),('saved_SCSTO',scsto)]:
                    if ref['status']!='success_numerical':continue
                    row=dict(environment=env,case=case,variant=spec['name'],reference=ref_name,
                             variant_seconds=r['cost'],reference_seconds=ref['cost'],
                             cost_ratio=r['cost']/ref['cost'])
                    paired.append(row)
                    if ref_name=='baseline_NTO_time':comparisons.append(row)
            summary.append(dict(environment=env,variant=spec['name'],scheduled=len(rows),
                solver_success=sum(bool(r.get('solver_success')) for _,r in rows),
                strict_success=len(ok),validation_failed=sum(r['status']=='validation_failed' for _,r in rows),
                optimization_failed=sum(r['status']=='optimization_failed' for _,r in rows),
                timeout=sum(r['status']=='timeout' for _,r in rows),
                other_failure=sum(r['status'] not in ['success_numerical','validation_failed','optimization_failed','timeout'] for _,r in rows),
                valid_subset_mean_cost=float(np.mean([r['cost'] for r in ok])) if ok else None,
                paired_with_valid_baseline_n=len(comparisons),
                paired_cost_ratio_mean=float(np.mean([r['cost_ratio'] for r in comparisons])) if comparisons else None))
        checks.append(dict(environment=env,planned_jobs=len(manifest['jobs']),recorded_jobs=len(records),
                           original_inputs_unchanged=all(sha(Path(p))==h for p,h in manifest['input_hashes'].items()),
                           run_sources_unchanged=all(sha(ROOT/p)==h for p,h in manifest['source_hashes'].items())))
    csv_write(args.output/'sensitivity_summary.csv',summary)
    csv_write(args.output/'case_metrics.csv',case_rows)
    csv_write(args.output/'paired_costs.csv',paired)
    atomic(args.output/'baseline_parity.json',baseline)
    atomic(args.output/'input_hashes.json',inventory)
    verification=dict(runs=checks,baseline_replays=len(baseline),
        baseline_status_mismatches=sum(not r['status_equal'] for r in baseline),
        baseline_solver_flag_mismatches=sum(not r['solver_success_equal'] for r in baseline),
        baseline_array_shape_mismatches=sum(not r['array_shapes_equal'] for r in baseline),
        baseline_max_control_difference=max(r['controls_max_difference'] or 0 for r in baseline),
        baseline_max_knot_difference=max(r['knots_max_difference'] or 0 for r in baseline),
        original_and_new_inputs_unchanged=all(sha(p)==h for p,h in inventory.items()),
        input_files=len(inventory),comparison='posthoc diagnostic selected by input IDs, not independent held-out evaluation')
    atomic(args.output/'verification.json',verification)
    print(verification)


if __name__=='__main__':main()
