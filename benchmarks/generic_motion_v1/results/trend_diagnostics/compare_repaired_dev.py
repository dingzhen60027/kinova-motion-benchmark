#!/usr/bin/env python3
"""CPU-only paired repair comparison. Incomplete runs produce no final scores."""
from __future__ import annotations

import argparse
from collections import Counter
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys

ROOT = next(p for p in Path(__file__).resolve().parents if (p/'scripts/evaluate.py').is_file())
sys.path.insert(0, str(ROOT))
from benchmarks.generic_motion_v1.common import read, recorded_path, sha, write
from benchmarks.generic_motion_v1.batch import verify
from benchmarks.generic_motion_v1.runners import results
from benchmarks.generic_motion_v1.runners.summarize import _cluster_ratio

SUCCESS = results.SUCCESS
INDEPENDENT = {'RRTCONNECT', 'RRTSTAR'}
PAIR_KEYS = ('problem_id', 'method', 'objective', 'replica_seed')
EQUAL_FIELDS = ('input_sha256', 'robot_sha256', 'geometry_sha256',
                'validation_protocol_sha256', 'planner_seed', 'budget_seconds', 'cluster_id')


def key(row):
    return tuple(row[name] for name in PAIR_KEYS)


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def old_selected_index(records, comparison, replica):
    with Path(records).open(newline='') as stream:
        return [row for row in csv.DictReader(stream)
                if comparison in json.loads(row['selected_comparisons'])
                and (replica is None or int(row['replica_seed']) == replica)]


def flatten(value, prefix=''):
    if not isinstance(value, dict):
        return {prefix: value}
    return {path: item for k, v in value.items()
            for path, item in flatten(v, prefix+'.'+k if prefix else k).items()}


def configuration_audit(run, records, comparison, replica):
    """Exact declared differences; never apply today's defaults to old jobs."""
    run = Path(run).resolve()
    new_manifest = read(run/'run_manifest.json')
    new_jobs = {(j['case_id'], j['method'], j['objective'], j['replica_seed']): j
                for j in new_manifest['jobs']}
    manifests, groups = {}, {}
    for saved in old_selected_index(records, comparison, replica):
        old_run = recorded_path(saved['run'])
        if old_run not in manifests:
            m = read(old_run/'run_manifest.json')
            manifests[old_run] = (m, {j['job_id']: j for j in m['jobs']})
        old_manifest, jobs = manifests[old_run]
        old_job = jobs[saved['job_id']]
        pair_key = (saved['problem_id'], saved['method'], saved['objective'], int(saved['replica_seed']))
        new_job = new_jobs[pair_key]
        method, objective = saved['method'], saved['objective']
        before, after = flatten(old_job['evaluation_config']), flatten(new_job['evaluation_config'])
        diffs = []
        for field in sorted(before.keys() | after.keys()):
            if field in before and field in after and before[field] == after[field]:
                continue
            scope = 'potential_runtime_change_requires_review'
            if field.startswith('drm_query.'):
                scope = 'inactive_for_independent_OMPL' if method in INDEPENDENT else 'active_DRM_backend_repair'
            elif field.startswith('nto_refined.'):
                scope = 'active_time_spline_basis_change' if (method, objective) == ('NTO_REFINED', 'minimum_time') else 'inactive_for_this_method_and_track'
            elif field.startswith('extended_baselines.'):
                scope = 'inactive_other_method_configuration' if field.split('.')[1] != method.lower() else scope
            elif field.startswith('ompl.'):
                scope = ('inactive_other_method_configuration' if method not in INDEPENDENT else
                         'active_OMPL_native_planning_cap_change' if field == 'ompl.planning_seconds' else
                         'explicit_observer_setting; verify_historical_native_evidence' if field in
                         ('ompl.record_validated_progress', 'ompl.progress_poll_seconds') else
                         'active_OMPL_search_or_validation_reserve_change')
            diffs.append(dict(field=field, old_present=field in before, old=before.get(field),
                new_present=field in after, new=after.get(field), scope=scope))
        group_key = (method, objective, saved['run'], json.dumps(diffs, sort_keys=True))
        if group_key not in groups:
            source_files = ['model.py', 'runners/validate.py', 'runners/csdecomp_baseline.py',
                'runners/native_drm_query.py', 'runners/ompl_baselines.py', 'runners/chomp_stomp.py',
                'runners/trajopt_baseline.py', 'runners/paper_nto.py', 'runners/paper_nto_refined.py',
                'runners/native_drm_corrections.py', 'runners/nto_collision_constraint.py']
            source_hashes = {name: dict(old=old_manifest['source_hashes'].get('benchmarks/generic_motion_v1/'+name),
                new=new_manifest['source_hashes'].get('benchmarks/generic_motion_v1/'+name)) for name in source_files}
            groups[group_key] = dict(method=method, objective=objective, paired_jobs=0,
                old_run=saved['run'], old_manifest_sha256=sha(old_run/'run_manifest.json'),
                old_source_commit=old_manifest['source_code_commit'],
                declared_differences=diffs, recorded_source_hashes=source_hashes,
                unchanged_critical_settings={field: before[field] for field in (
                    'eizo.max_iterations', 'lgcs.solver', 'online_budget_seconds', 'validation_step_rad',
                    'velocity_limit_rad_s', 'acceleration_limit_rad_s2')
                    if field in before and before.get(field) == after.get(field)})
            if method in INDEPENDENT:
                source_name = 'benchmarks/generic_motion_v1/runners/ompl_baselines.py'
                source = old_run/'source_snapshot'/source_name
                source_verified = source.is_file() and sha(source) == old_manifest['source_hashes'].get(source_name)
                groups[group_key]['historical_ompl_evidence'] = dict(native_records=0,
                    same_recorded_search_settings=0, positive_matching_native_seed=0,
                    without_progress_artifact=0, source_verified=source_verified,
                    source=str(source.relative_to(ROOT)) if source_verified else None,
                    source_sha256=sha(source) if source_verified else None,
                    source_contains_progress_observer_switch=('record_validated_progress' in source.read_text()) if source_verified else None)
        groups[group_key]['paired_jobs'] += 1
        if method in INDEPENDENT:
            native_path = recorded_path(saved['attempt_path'])/'ompl_result.json'
            if native_path.is_file():
                native = read(native_path)
                evidence = groups[group_key]['historical_ompl_evidence']
                evidence['native_records'] += 1
                recorded_settings = native.get('settings', {})
                evidence['same_recorded_search_settings'] += bool(recorded_settings) and all(
                    new_job['evaluation_config']['ompl'].get(k) == v for k, v in recorded_settings.items())
                evidence['positive_matching_native_seed'] += (old_job['method_seed'] > 0
                    and native.get('native_seed') == old_job['method_seed'])
                evidence['without_progress_artifact'] += not (native_path.parent/'validated_progress/records.jsonl').exists()
    output = dict(new_run=results._relative(run), new_manifest_sha256=sha(run/'run_manifest.json'),
        new_source_commit=new_manifest['source_code_commit'], old_comparison=comparison,
        replica_seed=replica, paired_jobs=sum(g['paired_jobs'] for g in groups.values()),
        methods=list(groups.values()),
        critical_protocol_checks=dict(eizo_max_iterations_30_unchanged=all(g['unchanged_critical_settings'].get('eizo.max_iterations') == 30 for g in groups.values()),
            lgcs_solver_scip_unchanged=all(g['unchanged_critical_settings'].get('lgcs.solver') == 'scip' for g in groups.values() if g['method'] == 'LGCS')),
        interpretation='Exact per-method declared configuration comparison. Check critical_protocol_checks before claiming a cap/solver change. Active differences are individually classified; other-method configuration blocks are inactive for the method being compared. Source hashes are retained separately, so matching configuration alone is not a claim of identical implementations. Historical OMPL source/native-parameter evidence is recorded for the now-explicit disabled observer settings.',
        statistics_produced=False)
    if output['paired_jobs'] != len(new_jobs):
        raise ValueError('Configuration audit does not cover the complete paired cohort')
    return output


def ratio(rows, field):
    usable = [row for row in rows if finite(row.get(field))]
    return _cluster_ratio(usable, lambda row: row[field], lambda row: 1)


def pair_rows(old, new):
    for field in EQUAL_FIELDS:
        if old[field] != new[field]:
            raise ValueError(f'Unmatched {field} for {key(old)}')
    if key(old) != key(new) or old['input_valid'] is not True or new['input_valid'] is not True:
        raise ValueError('Pairing requires the same legal fixed task/method/objective/replica')
    row = {name: new[name] for name in PAIR_KEYS}
    row.update(cluster_id=new['cluster_id'], category=new['category'],
        input_sha256=new['input_sha256'], planner_seed=new['planner_seed'],
        budget_seconds=new['budget_seconds'], roadmap_id_old=old['roadmap_id'], roadmap_id_new=new['roadmap_id'])
    for prefix, observation in (('old', old), ('new', new)):
        row[prefix+'_success'] = int(observation['status'] == SUCCESS)
        for field in ('status', 'raw_status', 'status_basis', 'failure_stage', 'failure_reason',
                'cold_e2e_seconds', 'objective_value', 'objective_unit', 'path_length_rad',
                'trajectory_duration_s', 'frontend_validated', 'solver_reported_success',
                'source_commit', 'config_sha256', 'run_manifest_sha256', 'receipt_sha256', 'attempt_path'):
            row[prefix+'_'+field] = observation.get(field)
        raw = read(recorded_path(observation['result_path'])) if observation.get('result_path') else {}
        row[prefix+'_native_frontend_success'] = raw.get('frontend_native_success')
        validation = read(recorded_path(observation['validation_path'])) if observation.get('validation_path') else {}
        row[prefix+'_joint_range_scaled_length'] = (validation.get('normalized_path_length')
            if observation['status'] == SUCCESS else None)
    row['success_delta_new_minus_old'] = row['new_success']-row['old_success']
    row['both_success'] = bool(row['old_success'] and row['new_success'])
    row['new_only_success'] = bool(row['new_success'] and not row['old_success'])
    row['old_only_success'] = bool(row['old_success'] and not row['new_success'])
    row['neither_success'] = not (row['new_success'] or row['old_success'])
    row['cost_delta_new_minus_old'] = None
    row['cost_relative_delta_new_minus_old'] = None
    if row['both_success']:
        a, b = row['old_objective_value'], row['new_objective_value']
        if not finite(a) or not finite(b) or row['old_objective_unit'] != row['new_objective_unit']:
            raise ValueError('Common success must have finite same-unit objective values')
        row['cost_delta_new_minus_old'] = b-a
        row['cost_relative_delta_new_minus_old'] = (b-a)/a if a > 0 else None
    a, b = row['old_cold_e2e_seconds'], row['new_cold_e2e_seconds']
    row['cold_seconds_delta_new_minus_old'] = b-a if finite(a) and finite(b) else None
    return row


def exact_pairs(old, new):
    """Retain invalid registrations, but never put them in legal score denominators."""
    by_old, by_new = {key(r): r for r in old}, {key(r): r for r in new}
    if len(by_old) != len(old) or len(by_new) != len(new) or by_old.keys() != by_new.keys():
        raise ValueError('Old and repaired cohorts are not an exact one-to-one matched set')
    pairs, invalid = [], []
    for pair_key in sorted(by_new):
        before, after = by_old[pair_key], by_new[pair_key]
        for field in EQUAL_FIELDS:
            if before[field] != after[field]:
                raise ValueError(f'Unmatched {field} for {pair_key}')
        if before['input_valid'] is not after['input_valid']:
            raise ValueError('Before/after fixed-input validity changed')
        if after['input_valid'] is True:
            pairs.append(pair_rows(before, after))
        elif after['input_valid'] is False:
            invalid.append(dict(zip(PAIR_KEYS, pair_key), old_status=before['status'], new_status=after['status']))
        else:
            raise ValueError('Fixed-input validity is not Boolean')
    return pairs, invalid


def within_new_comparisons(summary_rows, comparison):
    """Read the primary aggregator's common-success CI, preserving its cohort."""
    output = []
    for objective, left, right in [('minimum_distance', 'RRTCONNECT', 'EI_ZO'),
                                   ('minimum_time', 'NTO_REFINED', 'EI_ZO')]:
        rows = [r for r in summary_rows if r['comparison'] == comparison and r['category'] == 'all'
                and r['objective'] == objective and r['row_type'] == 'paired' and r['metric'] == 'objective_value'
                and r['scope'] == 'common_success_a_minus_b' and {r['method'], r['method_b']} == {left, right}]
        if len(rows) != 1:
            raise ValueError('Missing or duplicate primary common-success summary for '+comparison+' '+objective)
        row = rows[0]
        sign = 1 if row['method'] == left else -1
        number = lambda field: float(row[field]) if row.get(field) not in (None, '') else None
        mean, lower, upper = number('mean'), number('ci_low'), number('ci_high')
        if sign < 0:
            mean = -mean if mean is not None else None
            lower, upper = (-upper if upper is not None else None, -lower if lower is not None else None)
        output.append(dict(comparison=comparison, objective=objective, method=left, reference_method=right,
            direction=left+' minus '+right, common_success_queries=int(row['count']),
            scene_clusters=int(row['scene_clusters']), paired_mean=mean,
            scene_cluster_ci95=[lower, upper] if lower is not None and upper is not None else None,
            unit=row['unit'], source='Existing primary summary.csv common-success paired row; no alternative success subset'))
    return output


def method_summary(pairs, category='all'):
    subset = [r for r in pairs if category == 'all' or r['category'] == category]
    if not subset:
        raise ValueError('Empty summary group')
    method, objective = subset[0]['method'], subset[0]['objective']
    summary = dict(method=method, objective=objective, category=category, paired_legal_jobs=len(subset),
        independent_scene_clusters=len({r['cluster_id'] for r in subset}),
        common_success=sum(r['both_success'] for r in subset),
        new_only_success=sum(r['new_only_success'] for r in subset),
        old_only_success=sum(r['old_only_success'] for r in subset),
        neither_success=sum(r['neither_success'] for r in subset),
        success_delta_new_minus_old=ratio(subset, 'success_delta_new_minus_old'),
        common_success_cost_delta_new_minus_old=ratio(subset, 'cost_delta_new_minus_old'),
        common_success_cost_relative_delta_new_minus_old=ratio(subset, 'cost_relative_delta_new_minus_old'),
        all_finished_cold_seconds_delta_new_minus_old=ratio(subset, 'cold_seconds_delta_new_minus_old'))
    for side in ('old', 'new'):
        success = [r for r in subset if r[side+'_success']]
        frontend = [r for r in subset if r[side+'_frontend_validated'] is True]
        summary[side] = dict(status_counts=dict(Counter(r[side+'_status'] for r in subset)),
            failure_stages=dict(Counter(str(r[side+'_failure_stage']) for r in subset if not r[side+'_success'])),
            successes=len(success), success_rate=ratio(subset, side+'_success'),
            frontend_applicable=method not in INDEPENDENT,
            frontend_legal_input_denominator=len(subset) if method not in INDEPENDENT else None,
            native_frontend_successes=sum(r[side+'_native_frontend_success'] is True for r in subset) if method not in INDEPENDENT else None,
            validated_frontend_successes=len(frontend) if method not in INDEPENDENT else None,
            frontend_not_reached_or_without_passing_evidence=len(subset)-len(frontend) if method not in INDEPENDENT else None,
            success_given_validated_frontend=ratio(frontend, side+'_success') if method not in INDEPENDENT else None)
    return summary


def _csv(path, rows):
    fields = list(dict.fromkeys(k for row in rows for k in row))
    with path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: json.dumps(v, separators=(',', ':')) if isinstance(v, (dict, list)) else v
                             for k, v in row.items()})


def compare(run, records, old_comparison, replica, prefix, *, new_comparison=None, summary_path=None):
    run, records, prefix = map(lambda p: Path(p).resolve(), (run, records, prefix))
    outputs = [prefix.with_suffix('.json'), prefix.with_name(prefix.name+'_pairs.csv'),
               prefix.with_name(prefix.name+'_summary.csv')]
    if any(path.exists() for path in outputs):
        raise FileExistsError('Refusing to overwrite finalized comparison evidence')
    prefix.parent.mkdir(parents=True, exist_ok=True)
    if not (run/'run_manifest.json').is_file():
        write(prefix.with_name(prefix.name+'_pending.json'),
              dict(final_statistics_produced=False, reason='new_run_not_prepared', run=results._relative(run),
                   observed_at_utc=datetime.now(timezone.utc).isoformat()))
        print('Pending: the declared new run has not been prepared; no scores produced.')
        return 2
    # Status uses the same first-terminal acceptance rule as the unified index.
    # No partial-cohort success estimate is published by this script.
    coverage = results.run_status(run)
    if coverage['pending']:
        pending = dict(final_statistics_produced=False, reason='unfinished_predeclared_cohort',
            observed_at_utc=datetime.now(timezone.utc).isoformat(),
            run=results._relative(run), coverage=coverage,
            next_command='Run this same command after all declared jobs have terminal receipts.')
        write(prefix.with_name(prefix.name+'_pending.json'), pending)
        print(json.dumps({k: coverage[k] for k in ('planned', 'completed', 'pending')}, indent=2))
        return 2
    integrity = verify(run)
    if integrity.get('integrity_passed') is not True or integrity.get('complete') is not True:
        raise ValueError('New run is not complete and verified: '+json.dumps(integrity))
    manifest = read(run/'run_manifest.json')
    if replica is not None and any(job.get('replica_seed') != replica for job in manifest['jobs']):
        raise ValueError('This paired report requires a single explicitly matched replica seed')
    if manifest.get('execution_mode', 'end_to_end') != 'end_to_end':
        raise ValueError('Before/after repair comparison requires a full-flow run')
    new_rows, _ = results.collect([run], verify_files=True)
    new, planned = results._selected_rows(new_rows,
        dict(id='repaired_dev_paired_source', mode='end_to_end', runs=[str(run)]),
        {results._relative(run): integrity})
    if len(new) != len(planned) or any(row['lifecycle'] != 'terminal' for row in new):
        raise ValueError('Not all new declared jobs have selected terminal receipts')
    old_index = old_selected_index(records, old_comparison, replica)
    if not old_index:
        raise ValueError('No old comparison rows with the requested replica')
    old, manifests, caches = [], {}, {}
    for saved in old_index:
        old_run = recorded_path(saved['run'])
        if old_run not in manifests:
            checked = verify(old_run)
            if checked.get('integrity_passed') is not True or checked.get('complete') is not True:
                raise ValueError('Old selected run is not complete and verified: '+str(old_run))
            m = read(old_run/'run_manifest.json')
            m['_index_manifest_sha256'] = sha(old_run/'run_manifest.json')
            manifests[old_run] = (m, {j['job_id']: j for j in m['jobs']})
            caches[old_run] = {}
        m, jobs = manifests[old_run]
        job = jobs[saved['job_id']]
        attempts = sorted((old_run/'jobs'/job['job_id']/'attempts').glob('[0-9][0-9][0-9][0-9]'))
        terminal = [p for p in attempts if (p/'receipt.json').is_file() and not read(p/'receipt.json').get('retry_pending')]
        if not terminal or terminal[0] != recorded_path(saved['attempt_path']):
            raise ValueError('Old index does not select its first terminal attempt')
        fresh = results.attempt_row(old_run, m, job, terminal[0], verify_files=True, case_cache=caches[old_run])
        if fresh['receipt_integrity'] is not True or fresh['record_id'] != saved['record_id'] or fresh['status'] != saved['status']:
            raise ValueError('Old index disagrees with freshly verified original evidence')
        old.append(fresh)
    pairs, invalid_pairs = exact_pairs(old, new)
    within_new = None
    if new_comparison is not None:
        indexed = old_selected_index(records, new_comparison, replica)
        fresh_by_id = {r['record_id']: r for r in new}
        if {r['record_id'] for r in indexed} != set(fresh_by_id) or len(indexed) != len(new):
            raise ValueError('New comparison index is not the complete newly verified run')
        for saved in indexed:
            fresh = fresh_by_id[saved['record_id']]
            for field in ('status', 'raw_status', 'receipt_sha256', 'run_manifest_sha256'):
                if saved[field] != fresh[field]:
                    raise ValueError('New index disagrees with raw receipt evidence: '+field)
        if summary_path is None:
            raise ValueError('New comparison requires the primary summary.csv')
        with Path(summary_path).open(newline='') as stream:
            within_new = within_new_comparisons(list(csv.DictReader(stream)), new_comparison)
    summaries = []
    for method, objective in sorted({(r['method'], r['objective']) for r in pairs}):
        block = [r for r in pairs if (r['method'], r['objective']) == (method, objective)]
        for category in ['all'] + sorted({r['category'] for r in block}):
            summaries.append(method_summary(block, category))
    config_audit = configuration_audit(run, records, old_comparison, replica)
    ompl_changes = [dict(method=g['method'], old_run=g['old_run'], old=d['old'], new=d['new'])
                    for g in config_audit['methods'] for d in g['declared_differences']
                    if d['scope'] == 'active_OMPL_native_planning_cap_change']
    report = dict(complete=True, final_statistics_produced=True, paired_rows=len(pairs),
        registered_paired_rows=len(old), invalid_input_paired_rows=len(invalid_pairs),
        invalid_input_status_counts={side: dict(Counter(r[side+'_status'] for r in invalid_pairs)) for side in ('old', 'new')},
        old_comparison=old_comparison, replica_seed=replica,
        replica_seeds=sorted({r['replica_seed'] for r in new}), new_comparison=new_comparison,
        new_run=results._relative(run), new_run_manifest_sha256=sha(run/'run_manifest.json'),
        new_source_commit=manifest['source_code_commit'], old_records_sha256=sha(records),
        historical_selection_unchanged=True,
        selection_policy='Exact same task/method/objective/replica and planner seed; first terminal only; no best-of.',
        statistical_policy='95% scene-cluster bootstrap, PCG64 seed45827,2000 replicates. All paired observations from a scene stay together; fewer than2 clusters has no CI.',
        cost_policy='Costs only on common numerical successes; differences are new minus old. Cold time also includes completed failures. Unstarted/pending jobs are never failures.',
        frontend_policy='Full-flow success denominator is every completed legal query. Local optimizer conditional denominator is independently validated DRM frontends; independent OMPL is not assigned a DRM denominator.',
        metric_definitions=dict(path_length_rad='sum_i ||q[i+1]-q[i]||_2, bounded unwrapped7D coordinates',
            joint_range_scaled_length='saved normalized_path_length: sum_i ||(q[i+1]-q[i])/(upper-lower)||_2; not length divided by endpoint chord',
            endpoint_detour_ratio='path_length_rad / ||q_goal-q_start||_2 is a different metric and is not computed or relabelled here'),
        configuration_audit=config_audit,
        ompl_native_cap_changes=ompl_changes,
        interpretation='Paired before/after comparison of the declared repaired pipeline. The configuration audit records every changed field. '
            +('OMPL native planning cap changed as recorded in ompl_native_cap_changes; in the completed-repair protocol this is10 to60 seconds, still bounded by the common remaining deadline minus reserves. Its effects cannot be attributed solely to DRM/NTO implementation fixes. ' if ompl_changes else 'No active OMPL native planning-cap change appears in this matched cohort. ')
            +'Each pipeline retains its actual source/limits; this is not a single-factor causal ablation. The selected historical cohort already used EI-ZO max_iterations=30 and LGCS SCIP. NTO_PAPER versus NTO_REFINED also differs in clearance/refinement/solver settings.',
        within_new_common_success=within_new,
        primary_summary_sha256=sha(summary_path) if summary_path is not None else None,
        summary=summaries)
    _csv(outputs[1], pairs); _csv(outputs[2], summaries); write(outputs[0], report)
    print(json.dumps(dict(paired_rows=len(pairs), summary_groups=len(summaries), complete=True), indent=2))
    return 0


def self_test():
    pairs=[]
    for i, (old, new, before, after) in enumerate([(1,1,10.,8.),(0,1,None,2.),(1,0,4.,None),(0,0,None,None)]):
        r=dict(method='CHOMP',objective='minimum_distance',category='clutter',cluster_id=str(i//2),
            old_success=old,new_success=new,old_status=SUCCESS if old else 'timeout',new_status=SUCCESS if new else 'timeout',
            old_failure_stage=None,new_failure_stage=None,old_frontend_validated=bool(old),new_frontend_validated=bool(new),
            old_native_frontend_success=bool(old),new_native_frontend_success=bool(new),
            both_success=bool(old and new),new_only_success=bool(new and not old),old_only_success=bool(old and not new),
            neither_success=not(old or new),success_delta_new_minus_old=new-old,
            cost_delta_new_minus_old=after-before if old and new else None,
            cost_relative_delta_new_minus_old=(after-before)/before if old and new else None,
            cold_seconds_delta_new_minus_old=1.)
        pairs.append(r)
    summary=method_summary(pairs)
    assert summary['paired_legal_jobs']==4 and summary['common_success']==1
    assert summary['old']['successes']==summary['new']['successes']==2
    assert summary['old']['frontend_legal_input_denominator']==4
    assert summary['old']['success_given_validated_frontend']['denominator']==2
    assert summary['success_delta_new_minus_old']['estimate']==0
    assert summary['success_delta_new_minus_old']['independent_scene_clusters']==2
    assert summary['common_success_cost_delta_new_minus_old']['estimate']==-2
    assert summary['common_success_cost_delta_new_minus_old']['ci95'] is None
    assert summary['common_success_cost_relative_delta_new_minus_old']['estimate']==-.2
    assert flatten({'a': {'b': 3}, 'list': [1, 2]}) == {'a.b': 3, 'list': [1, 2]}
    print('PASS: matched outcome/denominator, common-success-only cost, difference sign, scene clusters, one-cluster CI refusal')
    return 0


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',type=Path,default=ROOT/'results/repaired_dev')
    parser.add_argument('--records',type=Path,default=ROOT/'results/records.csv')
    parser.add_argument('--baseline-comparison',default='current_dev')
    repetitions = parser.add_mutually_exclusive_group()
    repetitions.add_argument('--replica-seed',type=int,default=0)
    repetitions.add_argument('--all-replicas', action='store_true', help='Match every predeclared replica in the complete new run (e.g. dev0,1,2)')
    parser.add_argument('--new-comparison', help='Completed end-to-end comparison ID in records.csv/summary.csv')
    parser.add_argument('--summary', type=Path, default=ROOT/'results/summary.csv')
    parser.add_argument('--output-prefix',type=Path,default=ROOT/'results/trend_diagnostics/repaired_dev_comparison')
    parser.add_argument('--self-test',action='store_true')
    parser.add_argument('--configuration-only',action='store_true')
    args=parser.parse_args()
    replica = None if args.all_replicas else args.replica_seed
    if args.self_test:
        return self_test()
    if args.configuration_only:
        report = configuration_audit(args.run,args.records,args.baseline_comparison,replica)
        path = args.output_prefix.with_name(args.output_prefix.name+'_configuration.json')
        write(path, report)
        print(json.dumps(dict(paired_jobs=report['paired_jobs'], method_objective_groups=len(report['methods']),
                              statistics_produced=False, output=str(path)), indent=2))
        return 0
    return compare(args.run,args.records,args.baseline_comparison,replica,args.output_prefix,
                   new_comparison=args.new_comparison, summary_path=args.summary if args.new_comparison else None)


if __name__=='__main__':
    raise SystemExit(main())
