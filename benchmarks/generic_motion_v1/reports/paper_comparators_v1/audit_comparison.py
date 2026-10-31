"""Read-only audit of original paper comparators on the frozen dev benchmark.

No planner, CUDA or Drake imports. Checks sealed evidence and NTO transcription,
not a new collision oracle. Reused results remain byte-for-byte historical data.
"""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path
import re

import numpy as np

PAPER = ('LSCS', 'LGCS', 'NTO-distance', 'SCSTO', 'NTO-time')
REFINED = ('NTO-refined-distance', 'NTO-refined-time')
LABELS = PAPER + REFINED
SUCCESS = 'success_validated_numerical'
NEW_PREFIX = 'rss2025_paper_comparators_v1_'
REUSE_PREFIX = 'rss2025_reliable_v2_'
BLOCKS = ('smoke_s000', 'dev_s000', 'dev_s001', 'dev_s002')


def read(path):
    return json.loads(Path(path).read_text())


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def require(condition, message):
    if not condition:
        raise ValueError(message)


def finite_number(value, nonnegative=False):
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and np.isfinite(value) and (not nonnegative or value >= 0))


def key(row):
    return row['case_id'], row['roadmap_id'], row['method_seed']


def validate_pairing(rows):
    groups = defaultdict(dict)
    for row in rows:
        identity = key(row)
        require(row['label'] not in groups[identity], 'Duplicate method observation')
        groups[identity][row['label']] = row
        require(row['success'] is (row['status'] == SUCCESS), 'Success/status mismatch')
        require(finite_number(row['cold_e2e_seconds'], True), 'Invalid cold observation')
        require(not row['success'] or (row['input_legal'] is True and
                row['final_validation_passed'] is True and finite_number(row['cost'], True)),
                'Success without legal input, finite cost and passed numerical validation')
    for group in groups.values():
        require(set(group) == set(LABELS), 'Missing or unexpected method in fixed task pairing')
        reference = group['LSCS']
        for row in group.values():
            for field in ('split', 'family', 'cluster_id', 'input_legal', 'q_start', 'q_goal',
                          'input_sha256', 'robot_sha256', 'geometry_sha256', 'budget_seconds',
                          'roadmap_nodes', 'roadmap_seed', 'straight_line_collision',
                          'native_drm_success', 'drm_success', 'path_sha256'):
                require(row[field] == reference[field], 'Fixed input/frontend mismatch: ' + field)
    return groups


def validate_matrix(rows):
    groups = validate_pairing(rows)
    require(len(rows) == 1330 and len(groups) == 190, 'Expected 1330 slots / 190 fixed task-roadmap pairs')
    require({r['split'] for r in rows} == {'smoke', 'dev'}, 'Only smoke/dev are in this comparison')
    require(all(r['roadmap_nodes'] == 6000 and r['budget_seconds'] == 60 for r in rows),
            'Roadmap size or common external budget changed')
    for label in LABELS:
        chosen = [r for r in rows if r['label'] == label]
        dev = [r for r in chosen if r['split'] == 'dev']
        smoke = [r for r in chosen if r['split'] == 'smoke']
        require(len(dev) == 180 and all(r['input_legal'] for r in dev), 'Wrong legal dev denominator')
        require(len({r['case_id'] for r in dev}) == 60 and len({r['cluster_id'] for r in dev}) == 20,
                'Wrong dev scene or task count')
        require(Counter(r['roadmap_seed'] for r in dev) == {0: 60, 1: 60, 2: 60}, 'Wrong dev roadmap seeds')
        require(Counter(r['family'] for r in dev) == dict.fromkeys(
            ('sparse_clutter', 'dense_clutter', 'bottleneck', 'route_choice'), 45), 'Wrong dev family distribution')
        require(len(smoke) == 10 and sum(r['input_legal'] for r in smoke) == 8 and
                all(r['roadmap_seed'] == 0 for r in smoke), 'Wrong smoke denominator')
        require(sum(r['family'] == 'open_control' for r in smoke) == 2, 'Missing separate open control')
    return groups


def compare_sources(manifests):
    require(len(manifests) == 8, 'Expected exactly eight unique source runs')
    first = next(iter(manifests.values()))
    config = None
    for name, manifest in manifests.items():
        require(manifest['source_hashes'] == first['source_hashes'], 'Runtime source snapshot mismatch: ' + name)
        require(manifest['timeout_seconds'] == 60, 'External deadline mismatch')
        require(manifest['formal_test_run'] is False, 'Formal test run mixed into dev study')
        expected = (20 if name.endswith('smoke_s000') else 120) if name.startswith(NEW_PREFIX) else (
            50 if name.endswith('smoke_s000') else 300)
        require(len(manifest['jobs']) == expected, 'Block slot count mismatch: ' + name)
        for job in manifest['jobs']:
            if config is None:
                config = job['evaluation_config']
            require(job['evaluation_config'] == config, 'Evaluation config mismatch')
            require((job['method'] == 'NTO_PAPER') == name.startswith(NEW_PREFIX),
                    'New/reused method attribution mismatch')
    return hashlib.sha256(json.dumps(first['source_hashes'], sort_keys=True).encode()).hexdigest()


def compare_frozen_inputs(new, old, index, manifests, verified):
    for field in ('runtime_source_hashes', 'evaluation_config', 'config_file_sha256', 'data_files',
                  'roadmap_files', 'external_dependency_files', 'timeout_seconds', 'gpu_workers'):
        require(new[field] == old[field], 'New/reused execution freeze mismatch: ' + field)
    require(new['planned_registered_slots'] == 380 and index['slots'] == 380 and
            new['test_planner_calls'] == index['test_planner_calls'] == 0, 'Wrong frozen comparison matrix')
    expected_reused = {path: item['manifest_sha256'] for path, item in verified.items()
                       if Path(path).name.startswith(REUSE_PREFIX)}
    require({path: item['manifest_sha256'] for path, item in new['reused_evidence'].items()} == expected_reused,
            'Reused run hashes differ from execution freeze')
    require({block['run']: block['run_manifest_sha256'] for block in index['blocks']} ==
            {path: item['manifest_sha256'] for path, item in verified.items() if Path(path).name.startswith(NEW_PREFIX)},
            'New run hashes differ from execution index')
    by_input = defaultdict(list)
    for manifest in manifests.values():
        require(manifest['source_hashes'] == new['runtime_source_hashes'], 'Manifest runtime differs from execution freeze')
        for job in manifest['jobs']:
            by_input[(job['case_id'], job['roadmap_id'])].append(job)
    for jobs in by_input.values():
        require(len(jobs) == 7, 'Manifest task does not contain all seven fixed methods')
        for job in jobs[1:]:
            for field in ('case', 'method_seed', 'solver_seed', 'solver_seed_used', 'evaluation_config', 'roadmap_dir'):
                require(job[field] == jobs[0][field], 'Full manifest task data differs across methods: ' + field)


def check_nto_parameters(parameters, objective, path):
    timed = objective == 'minimum_time'
    profile = parameters['profile']
    expected = dict(name='rss2025_franka_nto_reconstruction_v1', clearance_m=0.0,
                    collocation_parameter_Nc=150, influence_distance_m=.01,
                    maximum_iterations=2000, maximum_optimizer_seconds=60.,
                    solver_feasibility_tolerance=1e-6, warm_start_resampled=False,
                    minimum_control_points=None, velocity_limit_rad_s=1., acceleration_limit_rad_s2=1.)
    for field, value in expected.items():
        require(profile.get(field) == value, 'Original NTO profile changed: ' + field)
    require(not {'maximum_native_solves', 'maximum_collocation_points', 'refinement_guard_m',
                 'snopt_total_iterations'}.intersection(profile), 'Refinement options leaked into original NTO')
    require(parameters['solver'] == ('IPOPT' if timed else 'SNOPT') and
            parameters['spline_order'] == (4 if timed else 2), 'Wrong original objective solver/spline order')
    require(parameters['collocation_parameter_Nc'] == 150 and parameters['collocation_points'] == 149,
            'Original NTO collision binding count changed')
    require(np.array_equal(parameters['collocation_locations'], np.arange(1, 150) / 150),
            'Original NTO collision locations changed')
    require(parameters['warm_start_resampled'] is False and parameters['refinement_warm_start'] is False and
            parameters['legacy_cfg_nto_used'] is False, 'Original initialization changed')
    require(parameters['warm_start_node_count'] == len(path) and
            parameters['warm_start_control_count'] == len(path) + (2 if timed else 0) and
            parameters['warm_start_endpoint_copies'] == (2 if timed else 0), 'Original control count changed')
    options = parameters['parameters']
    if timed:
        require(options['max_iter'] == 2000 and options['tol'] == 1e-6, 'IPOPT options changed')
        remaining = options['max_cpu_time']
    else:
        require(options['Major iterations limit'] == 2000 and options['Major feasibility tolerance'] == 1e-6
                and options['Major optimality tolerance'] == 1e-6, 'SNOPT options changed')
        require('Iterations limit' not in options and 'Minor iterations limit' not in options,
                'Original SNOPT defaults overridden')
        require(parameters['reported_iteration_limits']['defaults_overridden'] is False,
                'SNOPT strengthened profile substituted')
        remaining = options['Time limit']
    require(finite_number(remaining) and remaining <= 60, 'Original solver budget reset')


def audit_original_attempt(row):
    attempt = Path(row['attempt'])
    require(not (attempt / 'refinement').exists() and not (attempt / 'nto_refinement.json').exists(),
            'Original NTO attempt has refinement rounds')
    parameter_files = list(attempt.rglob('nto_parameters.json'))
    require(len(parameter_files) <= 1, 'Original NTO invoked more than one transcription')
    params = attempt / 'nto_parameters.json'
    if not params.exists():
        require(not row['success'], 'Original NTO success has no saved solver parameters')
        require(row['status'] in ('frontend_failed', 'invalid_endpoint', 'invalid_scene_geometry',
                                 'model_consistency_failed', 'timeout', 'infrastructure_error', 'unsupported'),
                'Completed optimizer lacks saved parameters')
        return dict(profile_saved=False, initial_saved=(attempt / 'nto_initial.npz').exists())
    path = np.load(attempt / 'path.npy', allow_pickle=False)
    check_nto_parameters(read(params), row['objective'], path)
    with np.load(attempt / 'nto_initial.npz', allow_pickle=False) as initial:
        require(np.array_equal(initial['original_path'], path), 'Original NTO seed differs from native DRM path')
        controls = np.vstack([path[0], path, path[-1]]) if row['objective'] == 'minimum_time' else path
        require(np.array_equal(initial['controls'], controls), 'Original NTO initial controls changed')
    candidate = attempt / 'candidate.json'
    if candidate.exists():
        saved = read(candidate)
        require(saved['profile'] == 'rss2025_franka_nto_reconstruction_v1' and
                saved['collocation_points'] == 149 and saved['warm_start_resampled'] is False,
                'Candidate is not the original NTO profile')
    log_limits = None
    log = attempt / 'snopt.log'
    if row['objective'] == 'minimum_distance' and log.exists():
        text = log.read_text(errors='replace')
        patterns = {'total': r'\bIteration limit\.+\s+(\d+)',
                    'major': r'\bMajor iterations limit\.+\s+(\d+)',
                    'minor': r'\bMinor iterations limit\.+\s+(\d+)'}
        matched = {name: re.search(pattern, text) for name, pattern in patterns.items()}
        if all(matched.values()):
            log_limits = {name: int(match.group(1)) for name, match in matched.items()}
            require(log_limits == dict(total=10000, major=2000, minor=500),
                    'Native SNOPT log reports different original iteration limits')
    return dict(profile_saved=True, initial_saved=True, native_snopt_log_limits=log_limits)


def collect_native_diagnostics(rows):
    native_outcomes = defaultdict(Counter)
    collision_records = []
    native_exceptions = []
    for row in rows:
        if row['split'] != 'dev':
            continue
        native_path = Path(row['attempt']) / 'nto_solver_result.json'
        if native_path.exists():
            record = read(native_path)
            detail = record.get('native_solver_details') or {}
            outcome = dict(terminal_status=row['status'], solver_status=record.get('solver_status'),
                           solver_success=record.get('solver_success'), info=detail.get('info'),
                           native_status=detail.get('status'))
            native_outcomes[row['label']][json.dumps(outcome, sort_keys=True)] += 1
            if record.get('solver_status') == 'exception' or record.get('error'):
                native_exceptions.append(dict(label=row['label'], case_id=row['case_id'],
                    roadmap_seed=row['roadmap_seed'], terminal_status=row['status'],
                    solver_status=record.get('solver_status'), error=record.get('error'),
                    raw_result=str(native_path)))
        if row['status'] == 'collision_failed':
            collision_records.append(dict(label=row['label'], case_id=row['case_id'],
                roadmap_seed=row['roadmap_seed'], minimum_sampled_clearance_m=row['minimum_sampled_clearance_m'],
                attempt=row['attempt']))
    return dict(original_nto_dev_native_outcomes={label: [dict(json.loads(item), count=count)
        for item, count in sorted(counts.items())] for label, counts in native_outcomes.items()},
        original_nto_dev_collision_failures=collision_records,
        original_nto_dev_native_exceptions=native_exceptions)


def audit(metrics_path):
    from benchmarks.generic_motion_v1.batch import verify
    metrics_path = Path(metrics_path).resolve()
    metrics = read(metrics_path)
    require(metrics['complete'] is True and metrics['comparable'] is True and
            metrics['pairing_differences'] == [], 'Metrics are incomplete or noncomparable')
    require(set(metrics['expected_method_labels']) == set(LABELS), 'Wrong declared comparator set')
    groups = validate_matrix(metrics['rows'])
    require(metrics['requested_jobs'] == len(metrics['rows']), 'Metrics slot declaration disagrees')
    expected_names = {prefix + block for prefix in (NEW_PREFIX, REUSE_PREFIX) for block in BLOCKS}
    manifests, verified = {}, {}
    for item in metrics['runs']:
        path = Path(item['run']).resolve()
        require(path.name in expected_names and path.name not in manifests, 'Unknown or duplicate source run')
        require(sha(path / 'run_manifest.json') == item['manifest_sha256'], 'Run manifest changed after metrics')
        manifest = read(path / 'run_manifest.json')
        require(manifest['source_code_commit'] == item['source_commit'], 'Run source commit mismatch')
        check = verify(path)
        require(check['integrity_passed'] and check['complete'], 'Source run no longer sealed/complete')
        manifests[path.name] = manifest
        verified[str(path)] = dict(manifest_sha256=item['manifest_sha256'], source_commit=item['source_commit'],
                                   planned_jobs=len(manifest['jobs']), integrity=check)
    require(set(manifests) == expected_names, 'Missing source run')
    fingerprint = compare_sources(manifests)
    directory = metrics_path.parent.parent
    freeze_path = directory / 'execution_freeze.json'
    old_freeze_path = directory.parent / 'baseline_repair_v2/execution_freeze.json'
    index_path = directory / 'run_index.json'
    compare_frozen_inputs(read(freeze_path), read(old_freeze_path), read(index_path), manifests, verified)
    original = [r for r in metrics['rows'] if r['method'] == 'NTO_PAPER']
    require(len(original) == 380, 'Expected 380 original NTO slots')
    profiles = [audit_original_attempt(row) for row in original]
    diagnostics = collect_native_diagnostics(original)
    # The published metrics have already independently checked each sealed
    # record; bind every row back to its actual run/job as an additional gate.
    for row in metrics['rows']:
        run = Path(row['run']).resolve()
        manifest = manifests[run.name]
        jobs = {j['job_id']: j for j in manifest['jobs']}
        require(row['job_id'] in jobs, 'Observation does not belong to its saved run')
        job = jobs[row['job_id']]
        require(row['method'] == job['method'] and row['objective'] == job['objective'] and
                row['input_sha256'] == job['case']['problem_sha256'], 'Metric row/job identity mismatch')
        attempt = Path(row['attempt']).resolve()
        require(attempt.parent.parent == run / 'jobs' / job['job_id'], 'Metric attempt path outside its job')
        require(read(attempt / 'result.json')['status'] == row['status'], 'Metric final status differs from raw result')
    return dict(schema_version=1, passed=True, errors=[], metrics_path=str(metrics_path),
                metrics_sha256=sha(metrics_path), registered_slots=1330, new_registered_slots=380,
                reused_registered_slots=950, legal_slots=sum(r['input_legal'] for r in metrics['rows']),
                invalid_slots=sum(not r['input_legal'] for r in metrics['rows']),
                new_legal_slots=sum(r['input_legal'] for r in original),
                freeze_sha256=sha(freeze_path), old_freeze_sha256=sha(old_freeze_path),
                run_index_sha256=sha(index_path),
                fixed_task_roadmap_pairs=len(groups), source_hash_fingerprint=fingerprint,
                source_runs=verified, original_nto_saved_profiles=sum(p['profile_saved'] for p in profiles),
                original_nto_interrupted_or_not_started_profiles=sum(not p['profile_saved'] for p in profiles),
                original_snopt_native_log_limits_checked=sum(p.get('native_snopt_log_limits') is not None for p in profiles),
                **diagnostics,
                status_counts=dict(Counter(r['status'] for r in metrics['rows'])),
                new_status_counts=dict(Counter(r['status'] for r in original)),
                no_test_planner_calls=True, numerical_scope='Reads sealed numerical collision validation; does not rerun the collision checker or claim continuous certification.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--metrics', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    try:
        result = audit(args.metrics)
    except Exception as error:
        result = dict(passed=False, errors=[repr(error)], metrics_path=str(args.metrics))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False) + '\n')
    print(json.dumps({k: result[k] for k in ('passed', 'errors')}, ensure_ascii=False))
    if not result['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
