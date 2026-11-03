#!/usr/bin/env python3
"""Retrospectively validate saved DEV NTO_REFINED intermediate candidates.

Selection: final normalized status is not success_numerical, a completed saved
round has minimum independent sampled clearance in (2 um, 0.5 mm), and added
constraints triggered another optimization round. This is a diagnostic, not a
new timed planner result. Source artifacts are checked against sealed receipts.
"""
from __future__ import annotations
import argparse, collections, csv, hashlib, json, subprocess, sys, time, traceback
from pathlib import Path
ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT))
from benchmarks.generic_motion_v1.model import Model
from benchmarks.generic_motion_v1.runners.validate import validate


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output', required=True, type=Path, help='New output directory')
    args = ap.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    records = ROOT / 'benchmarks/generic_motion_v1/results/records.csv'
    rows = [r for r in csv.DictReader(records.open(newline='')) if
            r['method'] == 'NTO_REFINED' and r['split'] == 'dev' and
            Path(r['run']).name == 'baseline_dev' and r['status'] != 'success_numerical']
    report = dict(scope=__doc__, git_head=subprocess.check_output(['git','rev-parse','HEAD'], cwd=ROOT, text=True).strip(),
                  source_records_sha256=sha(records), script_sha256=sha(__file__),
                  command=' '.join(sys.argv), candidates=[], errors=[],
                  notes=['Dev only; historical solver output and benchmark results are unchanged.',
                         'Validation and model reconstruction are retrospective, so these timings are not original online timing.',
                         'Passing is finite-resolution numerical validation, not a continuous collision certificate.'])
    cache = {}
    for row in rows:
        attempt = ROOT / row['attempt_path']
        receipt_path = attempt/'receipt.json'
        try:
            if sha(receipt_path) != row['receipt_sha256']:
                raise ValueError('receipt hash mismatch')
            receipt = json.loads(receipt_path.read_text())
            def load(name):
                p = attempt/name
                if name not in receipt['files'] or sha(p) != receipt['files'][name]:
                    raise ValueError(f'sealed file hash mismatch: {name}')
                return json.loads(p.read_text())
            if not (attempt/'nto_refinement.json').exists():
                continue
            refinement=load('nto_refinement.json')
            for rnd in refinement['rounds']:
                guard=rnd.get('geometry_guard',{})
                minimum=guard.get('minimum_clearance_m')
                if minimum is None or not 2e-6 < minimum < .0005 or not rnd.get('added_collocation_locations'):
                    continue
                entry=dict(problem_id=row['problem_id'], objective=row['objective'], seed=row['replica_seed'],
                           final_status=row['status'], attempt=row['attempt_path'], receipt_sha256=row['receipt_sha256'],
                           round=rnd['index'], previous_minimum_clearance_m=minimum, status='running')
                report['candidates'].append(entry)
                try:
                    case=load('input.json'); cfg=load('evaluation.json')
                    rel=f"refinement/round_{rnd['index']:02d}/candidate.json"
                    candidate=load(rel)
                    for key in ('robot_sha256','collision_geometry_sha256'):
                        if key not in case: raise ValueError(f'missing model hash {key}')
                    cache_key=(json.dumps(case['voxels_xyz']), case['voxel_radius_m'])
                    start=time.perf_counter()
                    if cache_key not in cache:
                        cache[cache_key]=Model(case['voxels_xyz'],case['voxel_radius_m'])
                    model=cache[cache_key]
                    entry['model_setup_or_cache_seconds']=time.perf_counter()-start
                    for key in ('robot_sha256','collision_geometry_sha256'):
                        if getattr(model,key) != case[key]: raise ValueError(f'model mismatch: {key}')
                    dest=output/f"{row['job_id']}-round-{rnd['index']:02d}"
                    start=time.perf_counter()
                    result=validate(candidate,model,case,cfg,out=dest)
                    entry['validation_seconds']=time.perf_counter()-start
                    entry.update(status='completed', candidate_sha256=sha(attempt/rel),
                                 solver_success=candidate['solver_success'], validation_status=result['status'],
                                 passed=result['passed'], minimum_clearance_m=result['minimum_sampled_clearance_m'],
                                 native_collisions=result['native_collisions'], independent_collisions=result['independent_collisions'],
                                 contact_band_samples=result['contact_band_samples'], derivative_limits_passed=result['derivative_limits_passed'],
                                 joint_limits_passed=result['joint_limits_passed'],endpoint_error_rad=result['endpoint_error_rad'],
                                 cost=result['cost'], cost_unit=result['cost_unit'], artifact_directory=str(dest.relative_to(output)))
                except Exception as exc:
                    entry.update(status='error',exception=repr(exc),traceback=traceback.format_exc())
                    report['errors'].append(f"{row['job_id']}/round_{rnd['index']:02d}: {exc!r}")
        except Exception as exc:
            report['errors'].append(f"{row['job_id']}: {exc!r}")
    report['outcomes']=dict(collections.Counter(x.get('validation_status',x['status']) for x in report['candidates']))
    report['status']='error' if report['errors'] else 'completed'
    (output/'summary.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:report[k] for k in ('status','outcomes','errors')},indent=2))
    return int(bool(report['errors']))

if __name__=='__main__':
    raise SystemExit(main())
