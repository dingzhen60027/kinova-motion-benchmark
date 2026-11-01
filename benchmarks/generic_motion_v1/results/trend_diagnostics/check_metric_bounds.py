"""Check successful costs against analytic bounds; no planner/collision calls."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

ROOT = next(p for p in Path(__file__).resolve().parents if (p/'scripts/evaluate.py').is_file())
DEFAULT_COMPARISONS = ('current_dev', 'current_test', 'current_dev_repaired_s000')


def membership(row):
    value = row.get('selected_comparisons', [])
    return set(json.loads(value) if isinstance(value, str) else value)


def check_rows(rows, comparisons, tasks, configuration):
    """Pure arithmetic; configuration(row) returns that job's frozen limits."""
    selected = set(comparisons)
    counts = {'path_checks': 0, 'time_checks': 0}
    groups = {name: 0 for name in comparisons}
    observations = {name: 0 for name in comparisons}
    errors = []
    for row in rows:
        matches = selected.intersection(membership(row))
        if not matches:
            continue
        for group in matches:
            observations[group] += 1
        if row.get('status') != 'success_numerical':
            continue
        for group in matches:
            groups[group] += 1
        try:
            case = tasks[row['problem_id']]
            if case.get('validation_status') != 'valid_input':
                raise ValueError('successful output has invalid frozen input')
            if len(case['q_start']) != 7 or len(case['q_goal']) != 7:
                raise ValueError('fixed endpoints must have seven joints each')
            distances = [abs(a-b) for a, b in zip(case['q_start'], case['q_goal'])]
            if len(distances) != 7 or not all(math.isfinite(d) for d in distances):
                raise ValueError('fixed endpoints must be finite seven-joint vectors')
            bound = math.sqrt(sum(x*x for x in distances))
            value = float(row['path_length_rad'])
            counts['path_checks'] += 1
            if not math.isfinite(value) or value + 1e-4 < bound:
                errors.append(dict(record_id=row.get('record_id'), check='path_endpoint_chord', value=str(value), bound=bound))
            if row['objective'] == 'minimum_time':
                cfg = configuration(row)
                velocity, acceleration = cfg['velocity_limit_rad_s'], cfg['acceleration_limit_rad_s2']
                if not all(math.isfinite(x) and x > 0 for x in (velocity, acceleration)):
                    raise ValueError('motion limits must be finite positive values')
                bound = max(2*math.sqrt(d/acceleration) if d <= velocity*velocity/acceleration
                            else d/velocity + velocity/acceleration for d in distances)
                value = float(row['trajectory_duration_s'])
                counts['time_checks'] += 1
                if not math.isfinite(value) or value + 1e-4 < bound:
                    errors.append(dict(record_id=row.get('record_id'), check='rest_to_rest_time', value=str(value), bound=bound))
        except (KeyError, TypeError, ValueError, OSError) as exc:
            errors.append(dict(record_id=row.get('record_id'), check='bound_input', error=str(exc)))
    for name, count in observations.items():
        if not count:
            errors.append(dict(check='missing_comparison', comparison=name))
    return dict(scope='Selected numerical successes only; analytic metric sanity check, no new collision or planner queries',
        comparisons=list(comparisons), observations=observations, counts=counts, groups=groups,
        passed=not errors, errors=errors,
        path_bound='Euclidean fixed endpoint chord in bounded unwrapped joint coordinates',
        time_bound='Maximum joint rest-to-rest triangular/trapezoidal minimum using its saved motion limits',
        numerical_tolerance=1e-4)


def configuration_loader(root=ROOT):
    cache = {}
    def configuration(row):
        run = Path(row['run'])
        run = run if run.is_absolute() else root/run
        if run not in cache:
            manifest = json.loads((run/'run_manifest.json').read_text())
            cache[run] = {j['job_id']: j['evaluation_config'] for j in manifest['jobs']}
        return cache[run][row['job_id']]
    return configuration


def load_tasks(root=ROOT):
    tasks = {}
    for split in ('smoke', 'dev', 'test'):
        for line in (root/'benchmark/manifests'/f'{split}.jsonl').read_text().splitlines():
            case = json.loads(line)
            tasks[case['problem_id']] = case
    return tasks


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--comparisons', nargs='+', default=list(DEFAULT_COMPARISONS))
    parser.add_argument('--records', type=Path, default=ROOT/'results/records.csv')
    parser.add_argument('--output', type=Path, default=Path(__file__).with_name('metric_bounds.json'))
    args = parser.parse_args(argv)
    with args.records.open() as stream:
        rows = list(csv.DictReader(stream))
    report = check_rows(rows, args.comparisons, load_tasks(), configuration_loader())
    report['records_sha256'] = hashlib.sha256(args.records.read_bytes()).hexdigest()
    report['initial_probe_failure'] = 'Historical initial inline probe assumed run-level resolved_config.json and raised FileNotFoundError. Checks use each frozen job evaluation_config; raw results are unchanged.'
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps(report, indent=2, allow_nan=False))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
