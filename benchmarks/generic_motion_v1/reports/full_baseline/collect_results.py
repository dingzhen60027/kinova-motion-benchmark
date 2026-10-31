"""Recompute complete five-method receipts and pairing; never run a planner."""
from collections import Counter, defaultdict
import json
from pathlib import Path
import numpy as np
from ...batch import verify
from ...common import read, write, sha
from ...runners.summarize import summarize, _stat

BASE = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
SUCCESS = 'success_validated_numerical'


def collect(name):
    run = BASE / 'results' / name
    integrity = verify(run, current_sources=True)
    if not integrity['integrity_passed'] or not integrity['complete']:
        raise ValueError(f'Unfinished/unverified run: {integrity}')
    summary = summarize(run)
    manifest = read(run / 'run_manifest.json')
    records, paired = [], defaultdict(list)
    for job in manifest['jobs']:
        attempt = sorted((run / 'jobs' / job['job_id'] / 'attempts').glob('[0-9][0-9][0-9][0-9]'))[-1]
        result = read(attempt / 'result.json')
        validation = read(attempt / 'validation.json') if (attempt / 'validation.json').exists() else None
        candidate = read(attempt / 'candidate.json') if (attempt / 'candidate.json').exists() else None
        process = read(attempt / 'process.json')
        input_record = read(attempt / 'input.json') if (attempt / 'input.json').exists() else None
        if input_record is not None:
            input_record.pop('_objective', None)
            if input_record != job['case']:
                raise ValueError('Saved task differs from frozen manifest: ' + job['job_id'])
        row = dict(job_id=job['job_id'], method=job['method'], objective=job['objective'],
            case_id=job['case_id'], scene_id=job['scene_id'], family=job['family'],
            roadmap_id=job['roadmap_id'], method_seed=job['method_seed'],
            attempt=str(attempt.relative_to(BASE)), status=result['status'],
            native_frontend=bool(result.get('frontend_native_success')),
            accepted_frontend=bool(result.get('frontend_success')),
            source_code_commit=result.get('source_code_commit'),
            failure_stage=result.get('failed_stage', result.get('final_stage')),
            error_category=result.get('error_category'), exception=result.get('exception'),
            straight_line_collision=job['case']['straight_line_collision'],
            result_sha256=sha(attempt / 'result.json'),
            path_sha256=sha(attempt / 'path.npy') if (attempt / 'path.npy').exists() else None,
            timing=result.get('timing', {}), process_wall_seconds=process.get('cold_e2e_seconds', process.get('wall_seconds')),
            complete_component_timing=bool(candidate is not None and not result.get('failed_stage')
                and result['status'] != 'timeout'
                and 'trajectory_optimization_seconds' in result.get('timing', {})),
            metrics=result.get('metrics', {}), solver_success=candidate.get('solver_success') if candidate else None,
            solver_status=candidate.get('solver_status') if candidate else None,
            globally_optimal_in_saved_pool=candidate.get('globally_optimal') if candidate else None,
            validation_status=validation.get('status') if validation else None,
            validation_samples=validation.get('samples') if validation else None,
            model_disagreements=validation.get('model_disagreements') if validation else None,
            final_or_last_validated_candidate_cost=validation.get('cost') if validation else None,
            cost_unit=validation.get('cost_unit') if validation else None,
            minimum_sampled_clearance_m=validation.get('minimum_sampled_clearance_m') if validation else None,
            validation_scope='For a failed later round, root validation may describe the last saved validated candidate; successful rows must match final candidate',
            alternative_route=result.get('alternate_route'))
        if (attempt / 'nto_solver_result.json').exists():
            row['nto_raw_solver'] = read(attempt / 'nto_solver_result.json')
        if (attempt / 'optimizer_failure_metadata.json').exists():
            row['optimizer_failure'] = read(attempt / 'optimizer_failure_metadata.json')
        records.append(row)
        paired[(job['case_id'], job['roadmap_id'], job['method_seed'])].append(row)
    pairing = []
    for key, rows in sorted(paired.items()):
        if len(rows) != 5:
            raise ValueError(f'Expected all five algorithms for {key}')
        hashes = sorted({r['path_sha256'] for r in rows if r['path_sha256']})
        statuses = {str((r['native_frontend'], r['accepted_frontend'])) for r in rows}
        pairing.append(dict(case_id=key[0], roadmap_id=key[1], method_seed=key[2],
            jobs=len(rows), distinct_saved_path_hashes=hashes, same_saved_path=len(hashes) <= 1,
            same_frontend_outcome=len(statuses) <= 1, frontend_outcomes=sorted(statuses)))
    groups = []
    for method, objective in sorted({(r['method'], r['objective']) for r in records}):
        rows = [r for r in records if (r['method'], r['objective']) == (method, objective)]
        successes = [r for r in rows if r['status'] == SUCCESS]
        groups.append(dict(method=method, objective=objective, count=len(rows),
            status_counts=dict(Counter(r['status'] for r in rows)),
            native_frontend=sum(r['native_frontend'] for r in rows),
            accepted_frontend=sum(r['accepted_frontend'] for r in rows),
            solver_success_candidates=sum(r['solver_success'] is True for r in rows),
            validated_successes=len(successes),
            blocked_straight_line_successes=sum(r['straight_line_collision'] is True for r in successes),
            recovery_successes=sum(r['metrics'].get('recovery_attempts', 0) > 0 for r in successes),
            successful_validation_samples=sum(r['validation_samples'] or 0 for r in successes),
            successful_model_disagreements=sum(r['model_disagreements'] or 0 for r in successes),
            successful_cost=_stat([r['final_or_last_validated_candidate_cost'] for r in successes]),
            TO_CS_seconds=_stat([sum(r['timing'].get(k, 0) for k in
                ('eizo_inflation_seconds', 'alternate_drm_path_seconds',
                 'trajectory_optimization_seconds', 'collision_recovery_seconds'))
                for r in rows if r['complete_component_timing']]),
            TO_CS_scope='Only attempts returning a candidate with recorded optimization timing, no exception stage and no timeout. Shared initial seed validation and separately timed candidate validation are excluded; LGCS alternate-route validation and map IO are included in its alternate-path stage. This is not pure solver time. Partial interrupted stages and initialized zero accumulators are excluded, not zero-filled. Main comparison uses external full wall time.'))
    analysis = dict(run=name, integrity=integrity, source_commit=manifest['source_code_commit'],
        groups=groups, paired_inputs=pairing,
        paired_path_mismatches=sum(not p['same_saved_path'] for p in pairing),
        paired_frontend_mismatches=sum(not p['same_frontend_outcome'] for p in pairing),
        failure_stage_counts=[dict(status=k[0], stage=k[1], count=v)
            for k, v in sorted(Counter((r['status'], str(r['failure_stage'])) for r in records
                                      if r['status'] != SUCCESS).items())],
        records=records, summary_sha256=sha(run / 'summary/summary.json'))
    write(OUT / f'{name}_analysis.json', analysis)
    return analysis


def main():
    analyses = [collect(name) for name in ('rss2025_full_smoke_v1', 'rss2025_full_dev_v1')]
    protocol = read(OUT / 'preregistered_protocol.json')
    frozen = {name: sha(BASE.parents[1] / name) == expected
              for name, expected in protocol['artifacts'].items()}
    if not all(frozen.values()):
        raise ValueError('A frozen input/profile changed')
    write(OUT / 'completion.json', dict(complete=True,
        total_jobs=sum(a['integrity']['completed_jobs'] for a in analyses),
        input_hashes_unchanged=frozen, test_executed=False,
        runtime_source_commits=sorted({a['source_commit'] for a in analyses}),
        paired_path_mismatches=sum(a['paired_path_mismatches'] for a in analyses),
        paired_frontend_mismatches=sum(a['paired_frontend_mismatches'] for a in analyses),
        reproduction='Five method families on Kinova adaptation; original FR3 and hardware remain unreproduced'))


if __name__ == '__main__':
    main()
