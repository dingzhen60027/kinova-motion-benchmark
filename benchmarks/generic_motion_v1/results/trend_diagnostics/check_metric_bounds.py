"""Check reported successful costs against obstacle-free analytic lower bounds.

No planner or collision queries. Run after the selected result index is refreshed.
"""
import csv
import hashlib
import json
import math
from pathlib import Path

ROOT = next(p for p in Path(__file__).resolve().parents if (p/'scripts/evaluate.py').is_file())
records = ROOT/'results/records.csv'
tasks = {}
for split in ('smoke', 'dev', 'test'):
    for line in (ROOT/'benchmark/manifests'/f'{split}.jsonl').read_text().splitlines():
        case = json.loads(line)
        if case.get('validation_status') == 'valid_input':
            tasks[case['problem_id']] = case

selected = {'current_dev', 'current_test', 'current_dev_repaired_s000'}
counts = {'path_checks': 0, 'time_checks': 0}
errors, configs, groups = [], {}, {}
with records.open() as stream:
    for row in csv.DictReader(stream):
        comparisons = selected.intersection(json.loads(row['selected_comparisons']))
        if row['status'] != 'success_numerical' or not comparisons:
            continue
        for group in comparisons:
            groups[group] = groups.get(group, 0) + 1
        case = tasks[row['problem_id']]
        distances = [abs(a-b) for a,b in zip(case['q_start'], case['q_goal'])]
        bound = math.sqrt(sum(x*x for x in distances))
        value = float(row['path_length_rad'])
        counts['path_checks'] += 1
        if not math.isfinite(value) or value + 1e-4 < bound:
            errors.append(dict(record_id=row['record_id'], check='path_endpoint_chord', value=value, bound=bound))
        if row['objective'] == 'minimum_time':
            run = Path(row['run'])
            run = run if run.is_absolute() else ROOT/run
            if run not in configs:
                manifest = json.loads((run/'run_manifest.json').read_text())
                configs[run] = {j['job_id']: j['evaluation_config'] for j in manifest['jobs']}
            cfg = configs[run][row['job_id']]
            velocity, acceleration = cfg['velocity_limit_rad_s'], cfg['acceleration_limit_rad_s2']
            assert velocity > 0 and acceleration > 0
            bound = max(2*math.sqrt(d/acceleration) if d <= velocity*velocity/acceleration
                        else d/velocity + velocity/acceleration for d in distances)
            value = float(row['trajectory_duration_s'])
            counts['time_checks'] += 1
            if not math.isfinite(value) or value + 1e-4 < bound:
                errors.append(dict(record_id=row['record_id'], check='rest_to_rest_time', value=value, bound=bound))

report = dict(scope='Selected numerical successes only; analytic metric sanity check, no new collision or planner queries',
    counts=counts, groups=groups, passed=not errors, errors=errors,
    records_sha256=hashlib.sha256(records.read_bytes()).hexdigest(),
    path_bound='Euclidean fixed endpoint chord in bounded unwrapped joint coordinates',
    time_bound='Maximum joint rest-to-rest triangular/trapezoidal minimum using its saved motion limits',
    numerical_tolerance=1e-4,
    initial_probe_failure='Initial inline probe assumed run-level resolved_config.json and raised FileNotFoundError. This script reads each job evaluation_config from its manifest; no result records were modified.')
Path(__file__).with_name('metric_bounds.json').write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps(report, indent=2))
raise SystemExit(0 if report['passed'] else 1)
