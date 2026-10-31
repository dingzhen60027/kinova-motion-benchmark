"""Read sealed development artifacts only; no planner, Drake, or GPU imports."""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path

import numpy as np


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def span(values):
    return dict(count=len(values), minimum=min(values), maximum=max(values),
                median=float(np.median(values))) if values else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    manifest_path = args.run / 'run_manifest.json'
    manifest = json.loads(manifest_path.read_text())
    assert manifest['split'] == 'dev' and not manifest['formal_test_run']
    counts = defaultdict(Counter)
    nodes = defaultdict(Counter)
    budgets = defaultdict(list)
    clearances = defaultdict(list)
    constraint_excesses = defaultdict(list)
    solve_seconds = defaultdict(list)
    native_status = Counter()
    source_hashes = {}
    records = []
    for job in manifest['jobs']:
        if job['method'] != 'NTO_PAPER':
            continue
        attempts = list((args.run / 'jobs' / job['job_id'] / 'attempts').glob('*'))
        assert len(attempts) == 1, 'This completed run should have one sealed attempt per job'
        attempt = attempts[0]
        receipt = json.loads((attempt / 'receipt.json').read_text())

        def checked(name):
            path = attempt / name
            if not path.exists():
                return None
            digest = sha(path)
            assert receipt['files'][name] == digest, str(path)
            source_hashes[str(path)] = digest
            return path

        result = json.loads(checked('result.json').read_text())
        objective = job['objective']
        blocked = bool(job['case']['straight_line_collision'])
        state = result['status']
        counts[f'{objective}:straight_line_blocked={blocked}'][state] += 1
        row = dict(job_id=job['job_id'], case_id=job['case_id'],
                   scene_id=job['scene_id'], roadmap_id=job['roadmap_id'],
                   objective=objective, straight_line_collision=blocked,
                   accepted_frontend=result.get('frontend_success', False), status=state)
        path = checked('nto_parameters.json')
        if path:
            par = json.loads(path.read_text())
            nodes[objective][par['warm_start_node_count']] += 1
            budget = par['parameters'].get('Time limit', par['parameters'].get('max_cpu_time'))
            budgets[objective].append(budget)
            row.update(raw_drm_nodes=par['warm_start_node_count'],
                       controls=par['warm_start_control_count'], solver_budget_seconds=budget,
                       actual_collocation_points=par['collocation_points'])
        path = checked('validation.json')
        if path:
            validation = json.loads(path.read_text())
            row['minimum_sampled_clearance_m'] = validation['minimum_sampled_clearance_m']
            row['validation_passed'] = validation['passed']
            if state == 'collision_failed':
                clearances[objective].append(validation['minimum_sampled_clearance_m'])
        path = checked('nto_solver_result.json')
        if path:
            solver_result = json.loads(path.read_text())
            details = solver_result.get('native_solver_details', {})
            code = details.get('info', details.get('status'))
            row.update(native_status_code=code, native_solver_status=solver_result['solver_status'])
            native_status[f'{objective}:code={code}:final={state}'] += 1
            if 'solve_seconds' in solver_result:
                solve_seconds[f'{objective}:code={code}'].append(solver_result['solve_seconds'])
        path = checked('snopt.log')
        if path:
            row['snopt_log_bytes'] = path.stat().st_size
        path = checked('nto_solver_raw.npz')
        if path and objective == 'minimum_distance' and state == 'collision_failed':
            with np.load(path) as raw:
                assert raw['solver_F'].shape == (164,)
                value = float(np.max(raw['solver_F'][1:150]) - 1.0)
                constraint_excesses[objective].append(value)
                row['maximum_raw_collocation_constraint_excess'] = value
        records.append(row)
    report = dict(
        audit_type='Sealed dev artifacts only; no new solve or collision query',
        source_commit=manifest['source_code_commit'], run=str(args.run),
        run_manifest_sha256=sha(manifest_path), script_sha256=sha(Path(__file__)),
        nto_queries=len(records), distinct_cases=len({r['case_id'] for r in records}),
        distinct_scenes=len({r['scene_id'] for r in records}),
        stratified_status_counts={k: dict(v) for k, v in counts.items()},
        raw_drm_node_count_distribution={k: dict(v) for k, v in nodes.items()},
        optimizer_budget_seconds={k: span(v) for k, v in budgets.items()},
        collision_failed_clearance_m={k: span(v) for k, v in clearances.items()},
        collision_failed_raw_constraint_excess={k: span(v) for k, v in constraint_excesses.items()},
        native_code_and_final_state=dict(native_status),
        native_solve_seconds_by_code={k: span(v) for k, v in solve_seconds.items()},
        checked_source_files_sha256=source_hashes, records=records)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + '\n')
    print(json.dumps(dict(nto_queries=len(records), checked_files=len(source_hashes),
                          output=str(args.output), stratified_status_counts=report['stratified_status_counts']),
                     indent=2))


if __name__ == '__main__':
    main()
