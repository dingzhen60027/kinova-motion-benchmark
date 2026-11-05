"""Freshly validate the lowest-clearance successful DEV output per method/track.

This is a saved-output correctness check, not a new score or planner call.
Selection is fixed from the existing primary DEV index before revalidation.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys
import time
import traceback

import numpy as np

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT))
from benchmarks.generic_motion_v1.common import read, write
from benchmarks.generic_motion_v1.model import Model
from benchmarks.generic_motion_v1.runners.validate import validate


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    out = parser.parse_args().output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    records = ROOT / 'results/records.csv'
    with records.open() as f:
        rows = [r for r in csv.DictReader(f) if Path(r['run']).name == 'baseline_dev'
                and r['status'] == 'success_numerical']
    groups = {}
    for row in rows:
        key = row['method'], row['objective']
        ordering = float(row['minimum_sampled_clearance_m']), row['record_id']
        if key not in groups or ordering < groups[key][0]:
            groups[key] = ordering, row
    if len(groups) != 14:
        raise ValueError('Expected all 14 supported method-track combinations')
    report = dict(scope='Fresh numerical validation of saved DEV outputs; no planner call, test optimization or historical mutation',
                  selection='Lowest saved sampled clearance among successful primary DEV outputs, per method and track; ties use record_id',
                  records_sha256=sha(records), optimizer_calls=0, checks=[],
                  source_sha256={p: sha(ROOT/p) for p in (
                      'benchmarks/generic_motion_v1/model.py', 'benchmarks/generic_motion_v1/runners/validate.py')})
    write(out/'summary.json', report)
    for (method, objective), (_, row) in sorted(groups.items()):
        attempt = ROOT / row['attempt_path']
        item = dict(method=method, objective=objective, problem_id=row['problem_id'],
                    source_attempt=row['attempt_path'], record_id=row['record_id'], status='running')
        report['checks'].append(item)
        start = time.perf_counter()
        try:
            if sha(attempt/'receipt.json') != row['receipt_sha256']:
                raise ValueError('Receipt differs from the primary index')
            receipt = read(attempt/'receipt.json')
            for name, expected in receipt['files'].items():
                path = (attempt/name).resolve()
                if not path.is_relative_to(attempt.resolve()) or sha(path) != expected:
                    raise ValueError('Changed or escaped sealed artifact: '+name)
            for name in ('candidate.json', 'candidate.npz', 'input.json', 'evaluation.json', 'validation.json'):
                if name not in receipt['files']:
                    raise ValueError('Unsealed required artifact: '+name)
            case, cfg = read(attempt/'input.json'), read(attempt/'evaluation.json')
            candidate = read(attempt/'candidate.json')
            with np.load(attempt/'candidate.npz', allow_pickle=False) as arrays:
                candidate.update({name: arrays[name].copy() for name in arrays.files})
            model = Model(case['voxels_xyz'], case['voxel_radius_m'])
            for key in ('robot_sha256', 'collision_geometry_sha256'):
                if getattr(model, key) != case[key]:
                    raise ValueError('Canonical model identity differs: '+key)
            checked = validate(candidate, model, case, cfg, out=out/f'{method}_{objective}')
            original = read(attempt/'validation.json')
            equal_cost = bool(np.isclose(checked['cost'], original['cost'], rtol=1e-10, atol=1e-9))
            equal_distance = bool(np.isclose(checked['minimum_sampled_clearance_m'],
                                             original['minimum_sampled_clearance_m'], rtol=1e-10, atol=1e-12))
            item.update(status='completed', passed=checked['passed'] and equal_cost and equal_distance,
                        validation_status=checked['status'], cost=checked['cost'], cost_unit=checked['cost_unit'],
                        cost_matches=equal_cost, clearance_matches=equal_distance,
                        minimum_sampled_clearance_m=checked['minimum_sampled_clearance_m'], samples=checked['samples'],
                        original_receipt_sha256=sha(attempt/'receipt.json'))
        except Exception as exc:
            item.update(status='error', passed=False, exception=repr(exc), traceback=traceback.format_exc())
        item['seconds'] = time.perf_counter()-start
        write(out/'summary.json', report)
        print(json.dumps({k:item[k] for k in ('method','objective','status','passed')}), flush=True)
    report.update(passed=all(r['passed'] for r in report['checks']),
                  records_unchanged=sha(records) == report['records_sha256'],
                  script_sha256=sha(__file__))
    report['passed'] = report['passed'] and report['records_unchanged']
    write(out/'summary.json', report)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
