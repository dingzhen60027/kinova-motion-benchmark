"""CPU-only independent calculations and in-memory admission probes.

Reads sealed development-derived metrics; never invokes a planner or changes
experimental evidence. Synthetic probes are explicitly not experimental data.
"""
import copy
import hashlib
import json
from collections import Counter, defaultdict
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch

import numpy as np

from benchmarks.generic_motion_v1.runners import study_metrics as study

BASE = Path(__file__).resolve().parents[2]
REPORT = BASE / 'reports/baseline_study'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    after_path = REPORT / 'dev_metrics_stage_qualified/metrics.json'
    after = json.loads(after_path.read_text())
    rows = after['rows']
    primary = {g['label']: g for g in after['groups'] if g['scope'] == 'obstacle_main'}
    lgcs_unsupported = [r for r in rows if r['label'] == 'LGCS' and r['status'] == 'unsupported']
    stages = {}
    for key in sorted(study.BACKEND_STAGE_KEYS):
        if key in primary['LGCS']['stage_seconds']:
            stages[key] = dict(
                raw_counter_aggregation_before_fix=study._stat([r['timing'].get(key) for r in rows
                    if r['label'] == 'LGCS' and r['input_legal']]),
                after=primary['LGCS']['stage_seconds'][key]['all_legal'])
    independent = {}
    for label in sorted(primary):
        subset = [r for r in rows if r['label'] == label]
        legal = [r for r in subset if r['input_legal']]
        expected_success = sum(r['success'] for r in legal) / len(legal)
        measured = primary[label]['valid_success']['estimate']
        assert expected_success == measured
        independent[label] = dict(jobs=len(subset), legal=len(legal),
            physical_scenes=len({r['cluster_id'] for r in legal}),
            success=sum(r['success'] for r in legal), status_counts=dict(Counter(r['status'] for r in subset)),
            complete_backend_timing_jobs=primary[label]['complete_backend_timing_jobs'])
    # Verify paired cost and latency independently on exactly the joint success set.
    by_label = defaultdict(dict)
    for r in rows:
        by_label[r['label']][(r['case_id'], r['roadmap_id'], r['method_seed'])] = r
    paired = []
    for value in after['paired_comparisons']:
        if value['scope'] != 'obstacle_main':
            continue
        a, b = (by_label[value[key]] for key in ('method_a', 'method_b'))
        joint = [(a[key], b[key]) for key in a if a[key]['success'] and b[key]['success']]
        cost = float(np.mean([x['cost'] - y['cost'] for x, y in joint])) if joint else None
        latency = float(np.mean([x['cold_e2e_seconds'] - y['cold_e2e_seconds'] for x, y in joint])) if joint else None
        assert np.isclose(cost, value['common_success_cost_difference']['estimate'])
        assert np.isclose(latency, value['common_success_cold_latency_difference']['estimate'])
        assert value['common_success_cost_difference']['denominator'] == len(joint)
        assert value['common_success_cold_latency_difference']['denominator'] == len(joint)
        paired.append(dict(method_a=value['method_a'], method_b=value['method_b'],
            joint_success_jobs=len(joint), cost_difference=cost, cold_latency_difference=latency))
    # Minimal counterexample at the statistics boundary: a whole method becomes
    # invalid before pair_metrics constructs its label list. All other fixed task
    # metadata remain the same, so shared frontend checking cannot detect this.
    blocks = defaultdict(list)
    for r in rows:
        blocks[(r['case_id'], r['roadmap_id'], r['method_seed'])].append(r)
    source = next(block for block in blocks.values() if len(block) == 5 and all(
        r['native_drm_success'] is False and r['drm_success'] is False
        and r['path_sha256'] is None for r in block))
    synthetic = copy.deepcopy(source)
    changed = next(r for r in synthetic if r['label'] == 'LGCS')
    changed.update(input_legal=False, input_sha256='SYNTHETIC-OTHER-INPUT', status='invalid_endpoint')
    with TemporaryDirectory(prefix='metrics-review-cpu-') as temp:
        try:
            with patch.object(study, 'load_rows', return_value=(synthetic, [])):
                study.analyze([], Path(temp))
        except ValueError as error:
            boundary_probe = dict(synthetic_not_experiment=True,
                bypassed_real_load_and_verifier=True, rejected=True, reason=str(error),
                comparison_file_written=(Path(temp)/'metrics.json').exists())
        else:
            raise AssertionError('The synthetic mixed-input pair must be rejected')
    extremes = {}
    for success in (False, True):
        extreme = [dict(cluster_id=str(i), success=success) for i in range(5)]
        extremes[str(success)] = study._cluster_ratio(extreme, lambda r: int(r['success']), lambda r: 1)
    evidence = dict(cpu_only=True, real_observations=len(rows),
        current_metrics_sha256=sha(after_path),
        pre_pair_guard_probe_sha256=sha(REPORT/'metrics_independent_review.pre_pair_guard.json'),
        before_fix_scope='Direct recomputation of the former raw-counter aggregation from unchanged result receipts, not a claim of a historical metrics-file snapshot.',
        audited_sources={str(path.relative_to(BASE)): sha(path) for path in
            (BASE/'runners/study_metrics.py', BASE/'runners/summarize.py', BASE/'BASELINE_PROTOCOL.md')},
        status_and_denominator_recalculation=independent,
        paired_common_success_recalculation=paired,
        lgcs_unsupported_count=len(lgcs_unsupported), lgcs_stage_comparison=stages,
        raw_timing_preserved=all(row['timing'] == json.loads(
            (Path(row['attempt'])/'result.json').read_text()).get('timing', {}) for row in rows),
        actual_frontend_mismatch_count=len(after['pairing_differences']),
        synthetic_legality_pooling_probe=boundary_probe,
        synthetic_extreme_scene_bootstrap=extremes)
    target = REPORT / 'metrics_independent_review.json'
    target.write_text(json.dumps(evidence, indent=2, allow_nan=False) + '\n')
    print(json.dumps(dict(output=str(target), observations=len(rows), lgcs_unsupported=len(lgcs_unsupported),
                         raw_timing_preserved=evidence['raw_timing_preserved'],
                         synthetic_probe=boundary_probe), indent=2))


if __name__ == '__main__':
    main()
