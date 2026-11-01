"""Strict CPU-only acceptance of complete, explicitly selected baseline cohorts.

Run after the result index is refreshed. Pending cohorts exit 2 and write only a
.pending.json coverage report; an incomplete cohort never receives final scores.
No planning, collision queries, optimizer calls or artifact mutation occur here.
"""
from __future__ import annotations
import argparse
from collections import Counter, defaultdict
import csv
import datetime
import hashlib
import json
import math
from pathlib import Path
import sys

ROOT = next(p for p in Path(__file__).resolve().parents if (p/'scripts/evaluate.py').is_file())
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from benchmarks.generic_motion_v1 import batch
from benchmarks.generic_motion_v1.common import BASE, cases, read, recorded_path, resolve_case, sha
from benchmarks.generic_motion_v1.runners import results
from benchmarks.generic_motion_v1.runners.methods import METHODS, objectives
import check_metric_bounds as bounds

PAIRS = tuple((method, objective) for method in METHODS for objective in objectives(method))
SEEDS = {'smoke': (0,), 'dev': (0, 1, 2), 'test': (0,)}
# Existing approved inputs at e800d18608; repairs must not change the tasks.
PINNED_MANIFESTS = {
    'smoke': '92e19a293861fa1b93a2dba3919ae3bc3eec584f3845709dc99f2ce98e453720',
    'dev': '32de21ac2474846ae3cdc9867a3f41585cf80e7d0d4531e26285de216ef56000',
    'test': '9fcbe5ccae28d8681586b1ac6c9986f51f7cc7b5cdc19352c101910e3b6ed03b',
}
STATES = ('success_numerical', 'invalid_input', 'no_solution_within_budget', 'timeout',
          'solver_failure', 'validation_failed', 'infrastructure_error')


def _path(value):
    p = Path(value)
    return p.resolve() if p.is_absolute() else (ROOT/p).resolve()


def _fingerprint(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def job_key(job):
    return (job['case_id'], job['replica_seed'], job['method'], job['objective'])


def row_key(row):
    return (row['problem_id'], int(row['replica_seed']), row['method'], row['objective'])


def check_declared_matrix(manifests, dataset, *, expected_seeds=SEEDS, pairs=PAIRS):
    """Independently reconstruct the whole requested task/seed/method matrix."""
    errors, missing = [], []
    actual = Counter(job_key(job) for manifest in manifests for job in manifest['jobs'])
    splits = {manifest['split'] for manifest in manifests}
    expected = {(case['problem_id'], seed, method, objective)
                for split in splits for case in dataset[split]
                for seed in expected_seeds[split] for method, objective in pairs}
    for key, count in actual.items():
        if count != 1:
            errors.append({'check': 'duplicate_declared_job', 'key': key, 'count': count})
    for key in sorted(set(actual)-expected):
        errors.append({'check': 'unexpected_declared_job', 'key': key})
    missing = sorted(expected-set(actual))
    if missing:
        errors.append({'check': 'missing_declared_jobs', 'count': len(missing), 'examples': missing[:10]})
    return {'expected_jobs': len(expected), 'declared_jobs': sum(actual.values()),
            'unique_declared_jobs': len(actual), 'missing_jobs': len(missing), 'errors': errors}


def check_conditioned_cohort(run, manifest, dataset, primary_runs):
    """Regenerate stage-based eligibility, never choose by backend success."""
    from benchmarks.generic_motion_v1.runners.shared_initialization import prepare_index
    source = _path(manifest['shared_initialization_source'])
    errors = []
    if primary_runs and source not in primary_runs:
        errors.append({'check': 'conditioned_source_is_not_new_primary_run', 'source': str(source)})
    source_manifest = read(source/'run_manifest.json')
    roadmap_ids = defaultdict(set)
    for job in source_manifest['jobs']:
        if job['method'] == 'EI_ZO' and job['objective'] == 'minimum_distance':
            roadmap_ids[job['replica_seed']].add(job['roadmap_id'])
    seeds = SEEDS[manifest['split']]
    if any(len(roadmap_ids[seed]) != 1 for seed in seeds):
        raise ValueError('Conditioned source lacks an unambiguous roadmap for each required seed')
    jobs = manifest['jobs']
    if not jobs:
        raise ValueError('Conditioned run contains no predeclared backend jobs')
    _, regenerated = prepare_index(source, dataset[manifest['split']],
        [(seed, next(iter(roadmap_ids[seed]))) for seed in seeds], jobs[0]['evaluation_config'])
    declared = read(run/'initialization_coverage.json')
    if regenerated != declared:
        errors.append({'check': 'conditioned_eligibility_changed',
                       'regenerated_sha256': _fingerprint(regenerated), 'declared_sha256': _fingerprint(declared)})
    expected = {(x['problem_id'], x['replica_seed'], method, 'minimum_distance')
                for x in regenerated['coverage'] if x['eligible'] for method in ('EI_ZO', 'LGCS')}
    actual = Counter(job_key(job) for job in jobs)
    if set(actual) != expected or any(n != 1 for n in actual.values()):
        errors.append({'check': 'conditioned_paired_matrix', 'expected': len(expected), 'declared': sum(actual.values())})
    return {'source_run': str(source), 'registered_task_replicas': regenerated['registered_task_replicas'],
            'eligible_task_replicas': regenerated['eligible_task_replicas'], 'expected_paired_jobs': len(expected),
            'declared_jobs': sum(actual.values()), 'eligibility_reconstructed_before_backend_outcomes': True,
            'errors': errors}


def check_protocol(manifests, dataset, reference=None):
    errors = []
    canonical = {case['problem_id']: case for block in dataset.values() for case in block}
    model, validators, configurations, source_sets, budgets = set(), set(), set(), set(), set()
    sampling = set()
    validation_settings = set()
    cache = {}
    for manifest in manifests:
        budgets.add(manifest['timeout_seconds'])
        source_sets.add(_fingerprint(manifest['source_hashes']))
        validator = manifest['source_hashes'].get('benchmarks/generic_motion_v1/runners/validate.py')
        if validator is None:
            errors.append({'check': 'validator_source_missing'})
        validators.add(validator)
        for job in manifest['jobs']:
            case = resolve_case(job, cache)
            canonical_case = canonical.get(job['case_id'])
            if canonical_case is None or case != canonical_case:
                errors.append({'check': 'fixed_input_changed', 'job_id': job['job_id']})
            model.add((case['robot_sha256'], case['collision_geometry_sha256']))
            cfg = job['evaluation_config']
            configurations.add(_fingerprint(cfg))
            validation_settings.add(_fingerprint({k: cfg.get(k) for k in results.VALIDATION_KEYS}))
            sampling.add(cfg.get('ompl', {}).get('planning_seconds', 10.0))
            for field, setting in (('velocity_limits_rad_s', 'velocity_limit_rad_s'),
                                   ('acceleration_limits_rad_s2', 'acceleration_limit_rad_s2')):
                values = case.get(field)
                if values is not None and (len(values) != 7 or any(x != cfg.get(setting) for x in values)):
                    errors.append({'check': 'motion_limit_changed', 'job_id': job['job_id'], 'field': field})
            pairing = [p for p in case.get('paired_seeds', []) if p['drm_seed'] == job['replica_seed']]
            if len(pairing) != 1 or pairing[0]['method_seed'] != job['method_seed']:
                errors.append({'check': 'unpaired_algorithm_seed', 'job_id': job['job_id']})
    for name, values in [('robot_collision_model', model), ('validator_source', validators),
                         ('full_configuration', configurations), ('runtime_source', source_sets), ('outer_budget', budgets)]:
        if len(values) != 1:
            errors.append({'check': 'mixed_'+name, 'values': sorted(map(str, values))})
    if budgets != {60.0}:
        errors.append({'check': 'unexpected_outer_budget', 'values': sorted(budgets)})
    if reference is not None:
        expected_validator = reference['source_hashes'].get('benchmarks/generic_motion_v1/runners/validate.py')
        expected_settings = {_fingerprint({k: j['evaluation_config'].get(k) for k in results.VALIDATION_KEYS})
                             for j in reference['jobs']}
        if validators != {expected_validator} or validation_settings != expected_settings:
            errors.append({'check': 'validation_protocol_changed_from_reference',
                           'expected_validator': expected_validator, 'actual_validators': sorted(map(str, validators)),
                           'expected_settings': sorted(expected_settings), 'actual_settings': sorted(validation_settings)})
    return {'errors': errors, 'robot_collision_models': sorted(model), 'validator_sources': sorted(x for x in validators if x),
            'validation_settings_fingerprints': sorted(validation_settings),
            'configuration_fingerprints': sorted(configurations), 'runtime_fingerprints': sorted(source_sets),
            'outer_budget_seconds': sorted(budgets), 'ompl_native_planning_caps_seconds': sorted(sampling),
            'sampling_protocol_note': 'Historical OMPL runs used a 10-second native planning cap. The completed-repair profile may use 60 seconds as a ceiling, bounded by the shared remaining deadline minus validation/simplification reserves. This is an explicit protocol change, not solely a DRM/NTO implementation repair; RRTConnect still returns its first solution.'}


def csv_value(value):
    if value is None:
        return ''
    return json.dumps(value, separators=(',', ':'), sort_keys=True) if isinstance(value, (dict, list, tuple)) else str(value)


def check_record_index(fresh_rows, indexed_rows, selected_runs):
    """Every actual attempt must match its freshly read receipt-bound evidence."""
    errors = []
    relevant = [r for r in indexed_rows if _path(r['run']) in selected_runs]
    counts = Counter(r['record_id'] for r in relevant)
    fresh = {r['record_id']: r for r in fresh_rows}
    if len(fresh) != len(fresh_rows):
        errors.append({'check': 'duplicate_fresh_record_id'})
    if set(counts) != set(fresh) or any(n != 1 for n in counts.values()):
        errors.append({'check': 'index_attempt_coverage', 'fresh': len(fresh_rows), 'indexed': len(relevant),
                       'missing': len(set(fresh)-set(counts)), 'extra': len(set(counts)-set(fresh)),
                       'duplicates': sum(n-1 for n in counts.values() if n > 1)})
    for row in relevant:
        expected = fresh.get(row['record_id'])
        if expected is None:
            continue
        for field in results.RECORD_FIELDS:
            if field == 'selected_comparisons':
                continue
            if row.get(field, '') != csv_value(expected.get(field)):
                errors.append({'check': 'stale_or_changed_record', 'record_id': row['record_id'], 'field': field,
                               'expected': csv_value(expected.get(field)), 'actual': row.get(field)})
    return errors


def check_summary_counts(selected, planned, summary, comparison):
    errors = []
    expected = {}
    for split, objective in {(p['case']['split'], p['job']['objective']) for p in planned}:
        categories = {'all'} | {p['case']['family'] for p in planned if (p['case']['split'], p['job']['objective']) == (split, objective)}
        methods = {p['job']['method'] for p in planned if (p['case']['split'], p['job']['objective']) == (split, objective)}
        for category in categories:
            for method in methods:
                declaration = [p for p in planned if (p['case']['split'], p['job']['objective'], p['job']['method']) == (split, objective, method)
                               and (category == 'all' or p['case']['family'] == category)]
                chosen = [r for r in selected if (r['split'], r['objective'], r['method']) == (split, objective, method)
                          and (category == 'all' or r['category'] == category)]
                legal = [r for r in chosen if r['input_valid']]
                stat = Counter(r['status'] for r in legal)
                nlegal = sum(p['case']['validation_status'] == 'valid_input' for p in declaration)
                value = dict(planned_jobs=len(declaration), actual_attempts=sum(p['actual_attempts'] for p in declaration),
                             legal_jobs=nlegal, finished_legal_jobs=len(legal), pending_jobs=sum(p['row'] is None for p in declaration),
                             invalid_input_jobs=len(declaration)-nlegal)
                value.update({state: stat[state] for state in STATES if state != 'invalid_input'})
                expected[(split, objective, category, method)] = value
    actual = [r for r in summary if r['comparison'] == comparison and r['row_type'] == 'status']
    keys = Counter((r['split'], r['objective'], r['category'], r['method']) for r in actual)
    if set(keys) != set(expected) or any(n != 1 for n in keys.values()):
        errors.append({'check': 'summary_status_coverage', 'comparison': comparison, 'expected': len(expected), 'actual': len(actual)})
    for row in actual:
        key = (row['split'], row['objective'], row['category'], row['method'])
        for field, value in expected.get(key, {}).items():
            if row.get(field) != str(value):
                errors.append({'check': 'summary_count_mismatch', 'comparison': comparison, 'key': key,
                               'field': field, 'expected': value, 'actual': row.get(field)})
    return errors


def check_summary_metrics(selected, summary, comparison):
    """Recompute descriptive and paired quantities without bootstrap reuse."""
    import numpy as np
    errors, pools = [], {}
    def finite(value):
        return isinstance(value, (float, int)) and not isinstance(value, bool) and math.isfinite(value)
    def check(row, field, value):
        actual = row.get(field, '')
        good = actual == '' if value is None else bool(actual) and math.isfinite(float(actual)) and math.isclose(float(actual), value, rel_tol=1e-12, abs_tol=1e-12)
        if not good:
            errors.append({'check': 'summary_metric_mismatch', 'comparison': comparison,
                           'category': row.get('category'), 'method': row.get('method'), 'method_b': row.get('method_b'),
                           'objective': row.get('objective'), 'metric': row.get('metric'), 'scope': row.get('scope'),
                           'field': field, 'expected': value, 'actual': actual})
    for row in summary:
        if row['comparison'] != comparison or row['row_type'] not in ('metric', 'paired', 'paired_outcome'):
            continue
        key = (row['split'], row['objective'], row['category'])
        if key not in pools:
            pools[key] = [r for r in selected if r['input_valid'] and (r['split'], r['objective']) == key[:2]
                          and (key[2] == 'all' or r['category'] == key[2])]
        pool = pools[key]
        values = []
        if row['row_type'] == 'metric':
            group = [r for r in pool if r['method'] == row['method']]
            if row['scope'] == 'validated_success_only_descriptive':
                group = [r for r in group if r['status'] == 'success_numerical']
            values = [r[row['metric']] for r in group if finite(r.get(row['metric']))]
        else:
            left = {(r['problem_id'], r['replica_seed']): r for r in pool if r['method'] == row['method']}
            right = {(r['problem_id'], r['replica_seed']): r for r in pool if r['method'] == row['method_b']}
            pairs = [(left[k], right[k]) for k in left.keys() & right.keys()]
            if row['row_type'] == 'paired_outcome':
                expected = {'both_success': (True, True), 'a_only_success': (True, False),
                            'b_only_success': (False, True), 'neither_success': (False, False)}[row['metric']]
                n = sum((a['status'] == 'success_numerical', b['status'] == 'success_numerical') == expected for a, b in pairs)
                check(row, 'count', len(pairs)); check(row, 'outcome_count', n)
                check(row, 'mean', n/len(pairs) if pairs else None)
                continue
            for a, b in pairs:
                if row['scope'].startswith('common_success_') and not a['status'] == b['status'] == 'success_numerical':
                    continue
                x = int(a['status'] == 'success_numerical') if row['metric'] == 'success' else a.get(row['metric'])
                y = int(b['status'] == 'success_numerical') if row['metric'] == 'success' else b.get(row['metric'])
                if finite(x) and finite(y):
                    values.append(x-y)
        check(row, 'count', len(values))
        check(row, 'mean', sum(values)/len(values) if values else None)
        check(row, 'median', float(np.quantile(values, .5)) if values else None)
        check(row, 'p90', float(np.quantile(values, .9)) if values else None)
    return errors


def resolve_scope(selection, run_args=None, comparison_args=None):
    comparisons = selection.get('comparisons', [])
    if comparison_args:
        specs = [s for s in comparisons if s['id'] in comparison_args]
        missing = set(comparison_args)-{s['id'] for s in specs}
        if missing:
            raise ValueError('Unknown comparisons: '+', '.join(sorted(missing)))
    else:
        requested = {_path(x) for x in run_args or []}
        specs = [s for s in comparisons if s.get('mode', 'end_to_end') in ('end_to_end', 'fixed_initial_regions')
                 and {_path(x if isinstance(x, str) else x['path']) for x in s['runs']} <= requested]
    if not specs or len({s['id'] for s in specs}) != len(specs):
        raise ValueError('Supply unique, nonempty comparisons for the requested runs')
    if any(s.get('mode', 'end_to_end') not in ('end_to_end', 'fixed_initial_regions') for s in specs):
        raise ValueError('Use end_to_end or a predeclared fixed_initial_regions cohort; post-hoc conditional cohorts are separate')
    runs = {_path(x if isinstance(x, str) else x['path']) for s in specs for x in s['runs']}
    if run_args and runs != {_path(x) for x in run_args}:
        raise ValueError('Explicit runs and comparison run coverage differ')
    return sorted(runs), specs


def review(run_paths, specs, *, records_path, summary_path, reference_run=ROOT/'results/repaired_dev'):
    report = {'recorded_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
              'scope': 'CPU-only final cohort, receipt, input/protocol, accounting and metric acceptance; no new collision proof',
              'complete_primary_suite': False,
              'checker_sha256': sha(__file__), 'metric_checker_sha256': sha(bounds.__file__),
              'runs': [str(p) for p in run_paths], 'comparisons': [s['id'] for s in specs],
              'status': 'pending', 'passed': False, 'coverage': [], 'errors': []}
    manifests, verification = [], {}
    for run in run_paths:
        check = batch.verify(run, allow_unsealed=True)
        verification[results._relative(run)] = check
        report['coverage'].append({'run': str(run), **check})
        if (run/'run_manifest.json').is_file():
            manifests.append(read(run/'run_manifest.json'))
        if not check.get('integrity_passed'):
            report['errors'].append({'check': 'raw_integrity', 'run': str(run), 'errors': check.get('errors')})
    if len(manifests) != len(run_paths) or any(not x.get('complete') for x in verification.values()):
        return report
    # No final rates or cost summaries are computed before every run is sealed.
    dataset = {split: cases(BASE/'manifests'/f'{split}.jsonl') for split in SEEDS}
    report['input_manifest_sha256'] = {split: sha(BASE/'manifests'/f'{split}.jsonl') for split in SEEDS}
    if report['input_manifest_sha256'] != PINNED_MANIFESTS:
        report['errors'].append({'check': 'approved_input_manifest_changed', 'expected': PINNED_MANIFESTS})
    primary = [m for m in manifests if m.get('execution_mode', 'end_to_end') == 'end_to_end']
    matrix = check_declared_matrix(primary, dataset)
    report['matrix'] = matrix
    report['errors'].extend(matrix['errors'])
    if matrix['missing_jobs']:
        return report
    primary_runs = {run for run, manifest in zip(run_paths, manifests) if manifest.get('execution_mode', 'end_to_end') == 'end_to_end'}
    report['conditioned_cohorts'] = []
    for run, manifest in zip(run_paths, manifests):
        if manifest.get('execution_mode') == 'fixed_initial_regions':
            conditioned = check_conditioned_cohort(run, manifest, dataset, primary_runs)
            report['conditioned_cohorts'].append(conditioned)
            report['errors'].extend(conditioned['errors'])
    reference = read(reference_run/'run_manifest.json') if reference_run is not None else None
    protocol = check_protocol(manifests, dataset, reference)
    if reference_run is not None:
        protocol['reference_run_manifest_sha256'] = sha(reference_run/'run_manifest.json')
    report['protocol'] = protocol
    report['errors'].extend(protocol['errors'])
    report['status'] = 'failed'
    if report['errors']:
        return report
    fresh_rows, _ = results.collect(run_paths, verify_files=True)
    with records_path.open() as stream:
        indexed = list(csv.DictReader(stream))
    with summary_path.open() as stream:
        summary = list(csv.DictReader(stream))
    report['records_sha256'], report['summary_sha256'] = sha(records_path), sha(summary_path)
    report['errors'].extend(check_record_index(fresh_rows, indexed, set(run_paths)))
    completed_groups = {}
    for spec in specs:
        selected, planned = results._selected_rows(fresh_rows, spec, verification)
        if len(selected) != len(planned):
            report['errors'].append({'check': 'declared_terminal_mismatch', 'comparison': spec['id'], 'declared': len(planned), 'terminal': len(selected)})
        selected_ids = {r['record_id'] for r in selected}
        indexed_ids = [r['record_id'] for r in indexed if spec['id'] in bounds.membership(r)]
        if set(indexed_ids) != selected_ids or len(indexed_ids) != len(selected_ids):
            report['errors'].append({'check': 'comparison_index_mismatch', 'comparison': spec['id']})
        states = Counter(r['status'] for r in selected)
        unknown = set(states)-set(STATES)
        if unknown:
            report['errors'].append({'check': 'unknown_normalized_states', 'comparison': spec['id'], 'states': sorted(map(str, unknown))})
        infrastructure = [r for r in selected if r['status'] == 'infrastructure_error']
        if infrastructure:
            report['errors'].append({'check': 'unresolved_infrastructure_errors', 'comparison': spec['id'], 'count': len(infrastructure),
                                    'records': [{'record_id': r['record_id'], 'raw_status': r['raw_status'], 'reason': r['failure_reason']} for r in infrastructure]})
        for row in selected:
            if 'BoxBoxGradient' in str(row.get('failure_reason')) and row['status'] != 'infrastructure_error':
                report['errors'].append({'check': 'hidden_box_box_exception', 'record_id': row['record_id']})
        report['errors'].extend(check_summary_counts(selected, planned, summary, spec['id']))
        report['errors'].extend(check_summary_metrics(selected, summary, spec['id']))
        completed_groups[spec['id']] = {'registered_jobs': len(planned), 'terminal_jobs': len(selected),
            'legal_jobs': sum(r['input_valid'] for r in selected), 'normalized_statuses': dict(states)}
    report['metric_bounds'] = bounds.check_rows(indexed, report['comparisons'], bounds.load_tasks(), bounds.configuration_loader())
    if not report['metric_bounds']['passed']:
        report['errors'].append({'check': 'metric_bounds', 'errors': report['metric_bounds']['errors']})
    if (report['checker_sha256'] != sha(__file__) or report['metric_checker_sha256'] != sha(bounds.__file__)
            or report['records_sha256'] != sha(records_path) or report['summary_sha256'] != sha(summary_path)):
        report['errors'].append({'check': 'checker_or_derived_index_changed_during_review'})
    if not report['errors']:
        report.update(status='passed', passed=True, completed_groups=completed_groups,
                      complete_primary_suite={m['split'] for m in primary} == set(SEEDS))
    return report


def save_report(report, output):
    destination = output if report['passed'] else output.with_name(output.stem+'.'+report['status']+'.json')
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    return destination


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runs', nargs='+')
    parser.add_argument('--comparisons', nargs='+')
    parser.add_argument('--selection', type=Path, default=ROOT/'results/selection.json')
    parser.add_argument('--records', type=Path, default=ROOT/'results/records.csv')
    parser.add_argument('--summary', type=Path, default=ROOT/'results/summary.csv')
    parser.add_argument('--reference-run', type=Path, default=ROOT/'results/repaired_dev', help='Pinned previous run whose validator source and numeric validation/motion-limit settings must be unchanged')
    parser.add_argument('--output', type=Path, default=Path(__file__).with_name('completed_evaluation.json'))
    args = parser.parse_args(argv)
    if not args.runs and not args.comparisons:
        parser.error('Specify --runs and/or --comparisons explicitly')
    try:
        runs, specs = resolve_scope(read(args.selection), args.runs, args.comparisons)
        report = review(runs, specs, records_path=args.records, summary_path=args.summary, reference_run=args.reference_run)
    except (OSError, ValueError, KeyError, TypeError, RuntimeError) as exc:
        report = {'status': 'failed', 'passed': False, 'errors': [{'check': 'acceptance_exception', 'error': repr(exc)}]}
    destination = save_report(report, args.output)
    print(json.dumps({'status': report['status'], 'passed': report['passed'], 'report': str(destination),
                      'errors': report.get('errors'), 'coverage': report.get('coverage')}, indent=2))
    return 0 if report['passed'] else 2 if report['status'] == 'pending' else 1


if __name__ == '__main__':
    raise SystemExit(main())
