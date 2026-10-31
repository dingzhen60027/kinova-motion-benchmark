"""Unconditional/DRM-conditional reporting with scene-cluster resampling."""
from __future__ import annotations

from collections import Counter, defaultdict
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
from ..batch import verify as _verify_run

SUCCESS = 'success_validated_numerical'
PENDING = {'pending', 'running', 'not_started', 'retry_pending'}
HERE = Path(__file__).resolve().parents[1]


def _read(path, default=None):
    path = Path(path)
    return json.loads(path.read_text()) if path.exists() else default


def _number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and np.isfinite(value)


def _stat(values):
    a = np.asarray([x for x in values if _number(x)], float)
    if not len(a):
        return {'count': 0, 'mean': None, 'p50': None, 'p90': None, 'p95': None, 'min': None, 'max': None}
    return dict(count=len(a), mean=float(a.mean()), p50=float(np.quantile(a, .5)),
                p90=float(np.quantile(a, .9)), p95=float(np.quantile(a, .95)),
                min=float(a.min()), max=float(a.max()))


def _cluster_ratio(rows, numerator, denominator, seed=45827, replicates=2000):
    groups = defaultdict(lambda: [0., 0.])
    for row in rows:
        groups[row['cluster_id']][0] += numerator(row)
        groups[row['cluster_id']][1] += denominator(row)
    # Clusters with no observations in this estimator do not contribute units.
    values = np.asarray([v for _, v in sorted(groups.items()) if v[1] > 0], float).reshape(-1, 2)
    n = len(values)
    total_n = float(values[:, 0].sum()) if n else 0.
    total_d = float(values[:, 1].sum()) if n else 0.
    estimate = total_n / total_d if total_d else None
    result = dict(estimate=estimate, numerator=total_n, denominator=total_d,
                  independent_scene_clusters=n, ci95=None, bootstrap_seed=seed,
                  bootstrap_replicates=replicates, method='scene-level cluster bootstrap; resample scene clusters with all paired jobs intact')
    if n < 2:
        result['limitation'] = 'Fewer than two independent scene clusters; no confidence interval reported.'
        return result
    rng = np.random.Generator(np.random.PCG64(seed))
    draws = rng.integers(n, size=(replicates, n))
    totals = values[draws].sum(axis=1)
    ratios = totals[:, 0] / totals[:, 1]
    result['ci95'] = np.quantile(ratios, [.025, .975]).tolist()
    return result


def _legal(case):
    status = case.get('validation_status')
    if isinstance(status, dict):
        return status.get('valid', status.get('passed')) is True
    if case.get('valid_input') is False or case.get('input_valid') is False:
        return False
    if status is None:
        return False
    return status in {'valid_input', 'valid', 'passed', 'success_validated_numerical', 'audited_valid'}


def _latest_attempt(run_dir, job_id):
    directory = run_dir / 'jobs' / job_id / 'attempts'
    attempts = sorted(p for p in directory.glob('[0-9][0-9][0-9][0-9]') if p.is_dir()) if directory.exists() else []
    return attempts[-1] if attempts else None


def _select(*values):
    return next((x for x in values if x is not None), None)


def _finite_select(*values):
    return next((x for x in values if _number(x)), None)


def _row(run_dir, job):
    case = job.get('case', {})
    attempt = _latest_attempt(run_dir, job['job_id'])
    receipt = _read(attempt / 'receipt.json') if attempt else None
    sealed = receipt is not None
    retry_pending = bool(receipt.get('retry_pending')) if sealed else False
    result = _read(attempt / 'result.json', {}) if attempt else {}
    process = _read(attempt / 'process.json', {}) if attempt else {}
    # The standalone file is the validation evidence checked by verify().
    # An embedded duplicate must not override its objective or cost.
    validation = _read(attempt / 'validation.json') if attempt else None
    if validation is None:
        validation = result.get('validation', {}) or {}
    timing = result.get('timing', result.get('timings', {})) or {}
    metrics = result.get('metrics', {}) or {}
    last_attempt_status = _select(result.get('status'), process.get('status'))
    # A worker can have written its result while the coordinator is still
    # recording wall time or handling an external stop. Only a receipt seals
    # that outcome; an interrupted attempt explicitly awaiting retry is not a
    # completed job either.
    status = ('pending' if attempt is None else 'running' if not sealed else
              'retry_pending' if retry_pending else _select(last_attempt_status, 'pending'))
    drm = _select(result.get('drm_success'), result.get('frontend_success'), result.get('frontend', {}).get('drm_success'), result.get('frontend', {}).get('success'))
    roadmap_directory = Path(job['roadmap_dir']).resolve() if job.get('roadmap_dir') else None
    roadmap_metadata_path = roadmap_directory / 'metadata.json' if roadmap_directory else None
    roadmap_metadata = _read(roadmap_metadata_path, {}) if roadmap_metadata_path else {}
    roadmap_id = roadmap_metadata.get('roadmap_id', job.get('roadmap_id'))
    family = _select(job.get('family'), case.get('family'), 'unknown')
    scene_id = _select(job.get('scene_id'), case.get('scene_id'), 'unknown')
    # open_control has no additional geometry. Multiple IDs/seeds for that
    # fixed physical world must not masquerade as independent environments.
    cluster = ('open_control_shared_base_world' if family == 'open_control' else
               _select(case.get('scene_cluster_id'), case.get('cluster_id'), scene_id))
    cold = _select(process.get('cold_e2e_seconds'), process.get('wall_seconds'),
                   process.get('elapsed_seconds'), timing.get('cold_e2e_seconds'))
    warm = _select(timing.get('warm_query_seconds'), result.get('warm_query_seconds'),
                   timing.get('online_seconds'), timing.get('query_seconds'))
    gpu = _select(process.get('sampled_gpu_peak_mib'), process.get('gpu_peak_mib'),
                  result.get('sampled_gpu_peak_mib'), result.get('gpu_peak_mib'))
    return dict(job_id=job['job_id'], case_id=_select(job.get('case_id'), case.get('problem_id')),
                scene_id=scene_id, cluster_id=cluster, family=family,
                method=job.get('method', result.get('method')), objective=job.get('objective', result.get('objective')),
                roadmap_id=roadmap_id, roadmap_id_in_manifest=job.get('roadmap_id'),
                roadmap_metadata_path=str(roadmap_metadata_path) if roadmap_metadata_path else None,
                method_seed=job.get('method_seed'),
                input_legal=bool(_legal(case)), status=status, success=status == SUCCESS,
                last_attempt_status=last_attempt_status, attempt_sealed=sealed,
                retry_pending=retry_pending,
                drm_success=bool(drm) if drm is not None else None,
                native_drm_success=bool(result.get('frontend_native_success', drm)) if
                    result.get('frontend_native_success', drm) is not None else None,
                frontend_validation_status=result.get('frontend_validation_status'),
                cold_e2e_seconds=cold, warm_query_seconds=warm, sampled_gpu_peak_mib=gpu,
                cost=_select(validation.get('cost'), metrics.get('cost'), result.get('cost')) if status == SUCCESS else None,
                cost_unit=_select(validation.get('cost_unit'), metrics.get('cost_unit'), result.get('cost_unit')),
                minimum_sampled_clearance_m=_finite_select(validation.get('minimum_sampled_clearance_m'), metrics.get('minimum_sampled_clearance_m')),
                normalized_path_length=_finite_select(validation.get('normalized_path_length'), metrics.get('normalized_path_length')),
                region_count=_finite_select(result.get('region_count'), result.get('regions_count'), metrics.get('regions'), validation.get('regions')),
                facets=_finite_select(metrics.get('facets'), validation.get('facets'), result.get('facets')),
                repair_count=_finite_select(result.get('repair_count'), result.get('recovery_count'), metrics.get('recovery_attempts'), validation.get('recovery_attempts')),
                attempt=str(attempt.relative_to(run_dir)) if attempt else None,
                timing=timing)


def _group(rows, method, objective, family):
    legal = [r for r in rows if r['input_legal']]
    finished = [r for r in rows if r['status'] not in PENDING]
    successes = [r for r in finished if r['success']]
    failures = [r for r in finished if not r['success']]
    known_drm = [r for r in legal if r['drm_success'] is True]
    native_drm = [r for r in legal if r.get('native_drm_success', r['drm_success']) is True]
    fields = ('cold_e2e_seconds', 'warm_query_seconds', 'sampled_gpu_peak_mib')
    metric_fields = dict(regions='region_count', facets='facets', recovery_attempts='repair_count',
                         normalized_path_length='normalized_path_length',
                         minimum_sampled_clearance_m='minimum_sampled_clearance_m')
    stages = sorted({key for r in finished for key, value in r['timing'].items() if _number(value)})
    return dict(method=method, objective=objective, family=family, planned_jobs=len(rows),
                legal_input_jobs=len(legal), invalid_input_jobs=len(rows) - len(legal),
                pending_jobs=sum(r['status'] in PENDING for r in rows),
                status_counts=dict(sorted(Counter(r['status'] for r in rows).items())),
                unconditional_success=_cluster_ratio(legal, lambda r: float(r['success']), lambda r: 1.),
                conditional_on_drm_success=_cluster_ratio(known_drm, lambda r: float(r['success']), lambda r: 1.),
                conditional_on_native_drm_success=_cluster_ratio(native_drm, lambda r: float(r['success']), lambda r: 1.),
                conditional_denominator_note='drm_success is the accepted frontend (including the seed-path geometry gate when present); native_drm_success is the native planner return before that gate. Legacy receipts without a seed gate use the same denominator.',
                drm_status_unknown_jobs=sum(r['drm_success'] is None for r in legal),
                timing_and_memory={field: {label: _stat([r[field] for r in sample])
                                           for label, sample in [('all_finished', finished), ('success', successes), ('failure', failures)]}
                                   for field in fields},
                metrics={name: {label: _stat([r[field] for r in sample])
                                 for label, sample in [('all_finished', finished), ('success', successes), ('failure', failures)]}
                         for name, field in metric_fields.items()},
                metrics_note='Finite recorded values only, stratified by final status. Missing or unexecuted stages are not zero-filled; each statistic includes its observed count. Failure metrics describe saved candidates, not valid solutions.',
                stage_timing={key: {label: _stat([r['timing'].get(key) for r in sample])
                                   for label, sample in [('all_finished', finished), ('success', successes), ('failure', failures)]}
                              for key in stages},
                success_cost=_stat([r['cost'] for r in successes]),
                success_cost_note='Descriptive only. Compare methods using paired common-success costs below; unequal success subsets are not rankings.')


def _paired(rows):
    by_objective = defaultdict(list)
    for row in rows:
        if row['input_legal']:
            by_objective[row['objective']].append(row)
    output = []
    for objective, pool in sorted(by_objective.items()):
        methods = sorted({r['method'] for r in pool})
        for method in methods:
            keys = [(r['case_id'], r['roadmap_id'], r['method_seed']) for r in pool if r['method'] == method]
            if len(keys) != len(set(keys)):
                raise ValueError('Duplicate case/roadmap/seed pairing; cannot count duplicated jobs as independent runs.')
        by_method = {method: {(r['case_id'], r['roadmap_id'], r['method_seed']): r
                              for r in pool if r['method'] == method} for method in methods}
        for i, a in enumerate(methods):
            for b in methods[i + 1:]:
                common = sorted(set(by_method[a]) & set(by_method[b]), key=str)
                pairs = []
                for key in common:
                    ra, rb = by_method[a][key], by_method[b][key]
                    if ra['cluster_id'] != rb['cluster_id']:
                        raise ValueError('Paired methods disagree on scene-cluster identity.')
                    pairs.append(dict(cluster_id=ra['cluster_id'], a=ra, b=rb))
                costs = [p for p in pairs if p['a']['success'] and p['b']['success'] and
                         _number(p['a']['cost']) and _number(p['b']['cost']) and p['a']['cost_unit'] == p['b']['cost_unit']]
                times = [p for p in pairs if p['a']['status'] not in PENDING and p['b']['status'] not in PENDING
                         and _number(p['a']['cold_e2e_seconds']) and _number(p['b']['cold_e2e_seconds'])]
                output.append(dict(objective=objective, method_a=a, method_b=b,
                                   paired_legal_jobs=len(pairs),
                                   paired_finished_jobs=sum(p['a']['status'] not in PENDING and p['b']['status'] not in PENDING for p in pairs),
                                   success_difference_a_minus_b=_cluster_ratio(pairs, lambda p: float(p['a']['success']) - float(p['b']['success']), lambda p: 1.),
                                   common_success_cost_difference_a_minus_b=_cluster_ratio(costs, lambda p: p['a']['cost'] - p['b']['cost'], lambda p: 1.),
                                   observed_cold_time_difference_a_minus_b=_cluster_ratio(times, lambda p: p['a']['cold_e2e_seconds'] - p['b']['cold_e2e_seconds'], lambda p: 1.),
                                   note='Paired by fixed case, roadmap and method seed within one objective. Time includes failures when measured; pending time is not imputed. Cost comparison conditions on both methods succeeding.'))
    return output


def summarize(run_dir, output=None):
    run_dir = Path(run_dir).resolve()
    integrity = _verify_run(run_dir, allow_unsealed=True)
    if not integrity['integrity_passed']:
        raise ValueError('Refusing to summarize unverified evidence: ' + json.dumps(integrity))
    manifest = _read(run_dir / 'run_manifest.json')
    if not isinstance(manifest, dict) or not isinstance(manifest.get('jobs'), list):
        raise ValueError('run_manifest.json must contain the complete planned jobs list.')
    ids = [job['job_id'] for job in manifest['jobs']]
    if len(ids) != len(set(ids)):
        raise ValueError('Duplicate job_id in frozen run manifest.')
    rows = [_row(run_dir, job) for job in manifest['jobs']]
    groups = []
    for method, objective in sorted({(r['method'], r['objective']) for r in rows}):
        pool = [r for r in rows if r['method'] == method and r['objective'] == objective]
        groups.append(_group(pool, method, objective, 'all'))
        for family in sorted({r['family'] for r in pool}):
            groups.append(_group([r for r in pool if r['family'] == family], method, objective, family))
    used_roadmaps = {r['roadmap_id'] for r in rows}
    offline = []
    # Never substitute a different local map which happens to reuse its label.
    metadata_paths = {Path(r['roadmap_metadata_path']) for r in rows if r.get('roadmap_metadata_path')}
    seen_roadmaps = set()
    for path in sorted(metadata_paths):
        metadata = _read(path, {})
        expected = manifest.get('roadmap_files', {}).get(str(path.resolve()))
        if expected and (not path.exists() or hashlib.sha256(path.read_bytes()).hexdigest() != expected):
            raise ValueError(f'Frozen roadmap metadata changed: {path}')
        if metadata.get('roadmap_id') in used_roadmaps and metadata['roadmap_id'] not in seen_roadmaps:
            seen_roadmaps.add(metadata['roadmap_id'])
            offline.append(dict(roadmap_id=metadata['roadmap_id'], metadata_path=str(path),
                                metadata_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                                offline_seconds=metadata.get('offline_seconds'),
                                timing=metadata.get('timing'), actual_nodes=metadata.get('actual_nodes'),
                                native_map_bytes=metadata.get('native_map_bytes')))
    result = dict(schema_version=1, run_dir=str(run_dir), planned_jobs=len(rows),
                  integrity=integrity,
                  complete=all(r['status'] not in PENDING for r in rows),
                  finished_jobs=sum(r['status'] not in PENDING for r in rows),
                  status_counts=dict(sorted(Counter(r['status'] for r in rows).items())),
                  run_manifest_sha256=hashlib.sha256((run_dir / 'run_manifest.json').read_bytes()).hexdigest(),
                  main_denominator='All numerically legal planned jobs, including failures, timeouts and pending jobs. Invalid inputs are separately counted.',
                  uncertainty='Scene-level cluster bootstrap. All open_control IDs share one physical-world cluster. Seed repetitions are not independent environments.',
                  numerical_validation='Finite collision sampling, not a continuous safety certificate.',
                  timing_definition='cold_e2e is external process wall time including initialization and validation; warm_query is the recorded already-initialized online query. Offline roadmaps are listed once separately; missing durations remain null.',
                  gpu_memory_definition='Sampled process memory, not an exact allocator peak. Polling may miss short peaks.',
                  offline_roadmaps=offline,
                  unresolved_offline_roadmap_ids=sorted(str(x) for x in used_roadmaps - {r['roadmap_id'] for r in offline}),
                  groups=groups, paired_comparisons=_paired(rows))
    out = Path(output) if output else run_dir / 'summary'
    out.mkdir(parents=True, exist_ok=True)
    (out / 'summary.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    (out / 'paired_comparisons.json').write_text(json.dumps(result['paired_comparisons'], indent=2, allow_nan=False) + '\n')
    fields = [key for key in rows[0] if key != 'timing'] if rows else ['job_id', 'status']
    with (out / 'summary.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator='\n')
        writer.writeheader()
        writer.writerows({key: row[key] for key in fields} for row in rows)
    return result
