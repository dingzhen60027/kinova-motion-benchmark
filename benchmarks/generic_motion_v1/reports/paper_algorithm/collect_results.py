"""Collect sealed RSS2025/Kinova evidence; never start a planner or touch inputs.

Run after both batches finish:
  bash local/python.sh -m benchmarks.generic_motion_v1.reports.paper_algorithm.collect_results
"""
from collections import Counter
import hashlib
import json
import math
from pathlib import Path

from ...batch import verify
from ...common import read, write, sha
from ...runners.summarize import summarize

BASE = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent


def collect(name):
    run = BASE / 'results' / name
    check = verify(run)
    if not check['complete']:
        raise ValueError(f'Cannot report an unfinished or corrupted experiment: {check}')
    summary = summarize(run)
    manifest = read(run / 'run_manifest.json')
    records = []
    failure_stages = Counter()
    handoff_corrections = []
    recovery_events = Counter()
    reuse_jobs = []
    native_parameter_checks = 0
    for job in manifest['jobs']:
        attempt = sorted((run / 'jobs' / job['job_id'] / 'attempts').glob('[0-9][0-9][0-9][0-9]'))[-1]
        result = read(attempt / 'result.json')
        if (attempt/'eizo_parameters.json').exists():
            actual = read(attempt/'eizo_parameters.json')['native_effective_options']
            assert actual['seed'] == job['method_seed']
            for key, expected in job['evaluation_config']['eizo'].items():
                assert math.isclose(actual[key],expected,rel_tol=1e-6,abs_tol=1e-9), (job['job_id'], key)
            native_parameter_checks += 1
        if result['status'] != 'success_validated_numerical':
            phase = read(attempt/'phase.json').get('phase') if (attempt/'phase.json').exists() else None
            failure_stages[(result['status'], result.get('failed_stage') or result.get('final_stage') or phase or 'unknown')] += 1
        row = dict(job_id=job['job_id'], case_id=job['case_id'], scene_id=job['scene_id'],
            family=job['family'], objective=job['objective'], method_seed=job['method_seed'],
            roadmap_id=job['roadmap_id'], straight_line_collision=job['case']['straight_line_collision'],
            status=result['status'], native_frontend_success=result.get('frontend_native_success', False),
            error_category=result.get('error_category'),
            accepted_frontend_success=result['frontend_success'],
            seed_validation_status=result.get('frontend_validation_status'),
            attempt=str(attempt.relative_to(BASE)), result_sha256=sha(attempt/'result.json'),
            recovery_round_collision_counts=[])
        for directory in sorted((attempt / 'rounds').glob('[0-9][0-9][0-9]')):
            if (directory/'validation.json').exists():
                v = read(directory/'validation.json')
                row['recovery_round_collision_counts'].append(dict(
                    round=directory.name,native=v['native_collisions'],independent=v['independent_collisions'],
                    minimum_clearance_m=v['minimum_sampled_clearance_m'],status=v['status']))
        first_corridor = attempt / 'rounds/000/corridor.json'
        if first_corridor.exists():
            corridor = read(first_corridor)
            row.update(original_drm_edges=corridor['original_edge_count'],
                       initial_active_regions=corridor['active_region_count'])
            if corridor['active_region_count'] < corridor['original_edge_count']:
                reuse_jobs.append(job['job_id'])
        for receipt in attempt.glob('coverage/recovery_*/native_handoff/*/handoff.json'):
            item = read(receipt)
            if item['status'] == 'passed':
                handoff_corrections.append(item['maximum_total_normalized_offset_change_from_input'])
        for receipt in attempt.glob('coverage/recovery_*/recovery_events.json'):
            for event in read(receipt)['events']:
                recovery_events[event['action']] += 1
        validation_path = attempt/'validation.json'
        if validation_path.exists():
            v = read(validation_path)
            row.update(cost=v['cost'],cost_unit=v['cost_unit'],
                minimum_sampled_clearance_m=v['minimum_sampled_clearance_m'],
                final_validation_samples=v['samples'],model_disagreements=v['model_disagreements'])
        row['metrics']=result.get('metrics', {})
        records.append(row)
    result = dict(run=name,integrity=check,source_code_commit=manifest['source_code_commit'],
        run_manifest_sha256=sha(run/'run_manifest.json'),
        summary_sha256=sha(run/'summary/summary.json'),status_counts=summary['status_counts'],
        objective_groups=[g for g in summary['groups'] if g['family']=='all'],
        family_groups=[g for g in summary['groups'] if g['family']!='all'],
        failure_stages=[dict(status=s,stage=t,count=n) for (s,t),n in sorted(failure_stages.items())],
        seed_gate_rejections=sum(r['native_frontend_success'] and not r['accepted_frontend_success'] for r in records),
        consecutive_region_reuse_jobs=reuse_jobs,recovery_event_counts=dict(recovery_events),
        native_handoff_checks=len(handoff_corrections),
        effective_native_parameter_checks=native_parameter_checks,
        maximum_native_handoff_total_offset_change_rad=max(handoff_corrections,default=None),
        recovery_successes=[r['job_id'] for r in records if r['status']=='success_validated_numerical'
                            and r['metrics'].get('recovery_attempts',0)>0],
        records=records)
    write(OUT/f'{name}_analysis.json',result)
    return result


def main():
    collected = [collect(name) for name in ('rss2025_smoke_v1','rss2025_dev_v1')]
    original = read(OUT/'preregistered_protocol.json')
    unchanged = {split:sha(BASE/f'manifests/{split}.jsonl')==expected
                 for split,expected in original['input_manifests_sha256'].items()}
    final_config = read(OUT/'pre_run_amendment.json')['final_config_sha256']
    status = dict(complete=all(x['integrity']['complete'] for x in collected),
        manifest_hashes_unchanged=unchanged,
        config_sha256=sha(BASE/'configs/rss2025_eizo.json'),
        final_preregistered_config_unchanged=sha(BASE/'configs/rss2025_eizo.json')==final_config,
        formal_test_run=False,
        total_jobs=sum(x['integrity']['planned_jobs'] for x in collected),
        runs=[dict(run=x['run'],status_counts=x['status_counts'],
                   source_code_commit=x['source_code_commit']) for x in collected],
        collector_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        limitations=['Kinova adaptation, not original FR3 experiments',
            'Finite numerical collision validation, not continuous safety proof',
            'Native EI-ZO Python API does not expose statistical-test acceptance',
            'LGCS and paper performance ratios are not reproduced'])
    assert all(unchanged.values()) and status['final_preregistered_config_unchanged']
    write(OUT/'completion.json',status)
    print(json.dumps(status,indent=2))


if __name__ == '__main__':
    main()
