"""Comparable paper-baseline metrics computed only from sealed real attempts.

This module neither executes planners nor accepts a solver status as a valid
trajectory. Input-invalid tasks stay in accounting but never dilute latency
statistics for legal tasks. The no-obstacle control is reported separately.
"""
from collections import Counter, defaultdict
import argparse
import csv
import hashlib
import itertools
import json
from pathlib import Path

import numpy as np

from ..batch import verify
from ..common import read, write, sha, resolve_case
from .summarize import _row, _stat, _cluster_ratio, SUCCESS

LABELS = {('EI_ZO', 'minimum_distance'): 'LSCS', ('LGCS', 'minimum_distance'): 'LGCS',
          ('NTO_PAPER', 'minimum_distance'): 'NTO-distance',
          ('EI_ZO', 'minimum_time'): 'SCSTO', ('NTO_PAPER', 'minimum_time'): 'NTO-time',
          ('NTO', 'minimum_distance'): 'NTO-IPOPT-adapted-distance',
          ('NTO', 'minimum_time'): 'NTO-IPOPT-adapted-time',
          ('NTO_REFINED', 'minimum_distance'): 'NTO-refined-distance',
          ('NTO_REFINED', 'minimum_time'): 'NTO-refined-time',
          ('RRTCONNECT','minimum_distance'):'RRTConnect', ('RRTSTAR','minimum_distance'):'RRT*',
          ('CHOMP','minimum_distance'):'CHOMP', ('STOMP','minimum_distance'):'STOMP',
          ('TRAJOPT','minimum_distance'):'TrajOpt'}
LATENCY_THRESHOLDS = (0.5, 1., 2., 5., 10., 20., 30., 60.)
PAPER_LABELS = ('LSCS', 'LGCS', 'NTO-distance', 'SCSTO', 'NTO-time')
BACKEND_STAGE_KEYS = frozenset(('eizo_inflation_seconds', 'alternate_drm_path_seconds',
    'trajectory_optimization_seconds', 'validation_seconds', 'collision_recovery_seconds'))


def completed_stage_times(timing, result, candidate_present, validation_present):
    """Select measured stages without treating initialized counters as work.

    Backend counters accumulate over several calls and can remain zero, or be
    partial positive values, when a later call raises. Require the entire backend
    to return a candidate and final validation normally. This deliberately drops
    even completed early backend calls on an incomplete attempt; raw timing is
    retained separately. A normal solver failure is still a completed attempt.
    Frontend timers are assigned after their individual calls return.
    """
    reasons = []
    if result.get('failed_stage'):
        reasons.append('failed_stage')
    if result.get('exception'):
        reasons.append('exception')
    if result.get('status') in ('timeout', 'unsupported', 'infrastructure_error', 'interrupted'):
        reasons.append('incomplete_terminal_status')
    if not candidate_present:
        reasons.append('no_candidate')
    if not validation_present:
        reasons.append('no_final_validation')
    if not all(key in timing for key in ('trajectory_optimization_seconds', 'validation_seconds')):
        reasons.append('missing_backend_completion_timers')
    complete = not reasons
    excluded = sorted(BACKEND_STAGE_KEYS.intersection(timing)) if not complete else []
    selected = {key: value for key, value in timing.items() if key not in excluded}
    return selected, complete, excluded, reasons


def objective_reference(case, objective):
    """Obstacle-free endpoint relaxation, never a claimed optimal solution."""
    if case.get('validation_status') != 'valid_input':
        return None
    delta = np.abs(np.asarray(case['q_goal']) - np.asarray(case['q_start']))
    if objective == 'minimum_distance':
        return float(np.linalg.norm(delta))
    velocity = np.asarray(case['velocity_limits_rad_s'])
    acceleration = np.asarray(case['acceleration_limits_rad_s2'])
    if (delta.shape != (7,) or velocity.shape != (7,) or acceleration.shape != (7,)
            or not np.isfinite(np.r_[delta, velocity, acceleration]).all()
            or np.any(velocity <= 0) or np.any(acceleration <= 0)):
        raise ValueError('Invalid fixed motion-limit data')
    transition = velocity ** 2 / acceleration
    times = np.where(delta <= transition, 2 * np.sqrt(delta / acceleration),
                     delta / velocity + velocity / acceleration)
    return float(times.max())


def load_rows(runs):
    rows, identities, manifests = [], set(), []
    protocol_fingerprints = set()
    algorithm_fingerprints = set()
    budgets = set()
    models = set()
    for run in map(Path, runs):
        run = run.resolve()
        check = verify(run)
        if not check['integrity_passed'] or not check['complete']:
            raise ValueError(f'Refusing incomplete/unverified evidence: {run}')
        manifest = read(run / 'run_manifest.json')
        # Include orchestration, configuration loading and classification, not
        # just solver code. Different commits with identical source snapshots
        # are fine; changed runtime semantics must be analyzed separately.
        algorithm_sources = manifest['source_hashes']
        algorithm_fingerprints.add(hashlib.sha256(json.dumps(algorithm_sources,sort_keys=True).encode()).hexdigest())
        if len(algorithm_fingerprints)>1:
            raise ValueError('Different algorithm/model/validator source protocols cannot be pooled')
        budgets.add(manifest['timeout_seconds'])
        if len(budgets)>1:
            raise ValueError('Different wall-clock budgets cannot be pooled')
        manifests.append(dict(run=str(run), manifest_sha256=sha(run/'run_manifest.json'),
                              source_commit=manifest['source_code_commit'], integrity=check))
        case_cache = {}
        for job in manifest['jobs']:
            protocol_fingerprints.add(hashlib.sha256(json.dumps(job['evaluation_config'],sort_keys=True).encode()).hexdigest())
            if len(protocol_fingerprints)>1:
                raise ValueError('Different evaluation configurations cannot be pooled')
            row = _row(run, job, case_cache)
            key = (row['case_id'], row['roadmap_id'], row['method_seed'], row['method'], row['objective'])
            if key in identities:
                raise ValueError('Duplicate study observation; do not recount reused 6000-node runs')
            identities.add(key)
            case = resolve_case(job, case_cache)
            models.add((case['robot_sha256'],case['collision_geometry_sha256']))
            if len(models)>1:
                raise ValueError('Different canonical robot/geometry identities cannot be pooled')
            attempt = run / row['attempt']
            result = read(attempt / 'result.json')
            metadata = read(Path(job['roadmap_dir'])/'metadata.json') if job.get('roadmap_dir') else dict(
                requested_nodes=None, actual_nodes=None, seed=job.get('replica_seed'))
            row.update(run=str(run), attempt=str(attempt), split=case['split'],
                label=LABELS[(row['method'], row['objective'])],
                roadmap_nodes=metadata['requested_nodes'],
                actual_roadmap_nodes=metadata['actual_nodes'], roadmap_seed=metadata['seed'],
                budget_seconds=manifest['timeout_seconds'],
                straight_line_collision=case.get('straight_line_collision'),
                distance_bin=case.get('endpoint_sampling', {}).get('distance_bin', 'unknown'),
                start_goal_distance_rad=case.get('start_goal_joint_distance'),
                normalized_endpoint_distance=case.get('normalized_start_goal_joint_distance'),
                q_start=case.get('q_start'), q_goal=case.get('q_goal'),
                robot_sha256=case['robot_sha256'], geometry_sha256=case['collision_geometry_sha256'],
                input_sha256=case['problem_sha256'],
                objective_reference=objective_reference(case, row['objective']),
                dependency_unsupported=row['status']=='unsupported',
                infrastructure_failed=row['status']=='infrastructure_error',
                failure_stage=result.get('failed_stage', result.get('final_stage')),
                exception=result.get('exception'),
                path_sha256=sha(attempt/'path.npy') if (attempt/'path.npy').exists() else None,
                final_validation_passed=None, solver_reported_success=None,
                seed_cost_rad=None, seed_cost_validated=False, seed_min_clearance_m=None,
                frontend_component_seconds=None)
            if (attempt/'candidate.json').exists():
                candidate = read(attempt/'candidate.json')
                row['solver_reported_success'] = candidate.get('solver_success')
            if (attempt/'validation.json').exists():
                row['final_validation_passed'] = read(attempt/'validation.json')['passed']
            (row['completed_stage_seconds'], row['backend_timing_complete'],
             row['stage_timing_excluded_keys'], row['stage_timing_exclusion_reasons']) = completed_stage_times(
                row['timing'], result, (attempt/'candidate.json').exists(),
                (attempt/'validation.json').exists())
            seed_path = attempt/'frontend_validation/validation.json'
            if seed_path.exists():
                seed = read(seed_path)
                row['seed_cost_validated'] = seed['passed'] is True
                if seed['passed']:
                    path = np.load(attempt/'path.npy', allow_pickle=False)
                    length = float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())
                    if not np.isclose(length, seed['cost'], rtol=1e-10, atol=1e-9):
                        raise ValueError('Saved seed cost disagrees with its actual path')
                    row['seed_cost_rad'] = length
                    row['seed_min_clearance_m'] = seed['minimum_sampled_clearance_m']
                stage_keys = ('model_setup_seconds', 'gpu_initialization_warmup_seconds',
                    'roadmap_load_seconds', 'online_collision_map_seconds',
                    'drm_path_seconds', 'drm_path_validation_seconds')
                if all(k in row['timing'] for k in stage_keys):
                    row['frontend_component_seconds'] = sum(row['timing'][k] for k in stage_keys)
            reference = row['objective_reference']
            row['cost_over_endpoint_reference'] = (row['cost']/reference
                if row['success'] and reference and reference > 0 else None)
            row['distance_improvement_over_seed_fraction'] = (
                (row['seed_cost_rad']-row['cost'])/row['seed_cost_rad']
                if row['success'] and row['objective']=='minimum_distance'
                and row['seed_cost_rad'] and row['seed_cost_rad'] > 0 else None)
            rows.append(row)
    return rows, manifests


def group_metrics(rows):
    legal = [r for r in rows if r['input_legal']]
    success = [r for r in legal if r['success']]
    failed = [r for r in legal if not r['success']]
    native = [r for r in legal if r['native_drm_success'] is True]
    seed = [r for r in legal if r['drm_success'] is True]
    resolved = [r for r in legal if not r['dependency_unsupported'] and not r['infrastructure_failed']]
    fields = ('cold_e2e_seconds', 'warm_query_seconds', 'sampled_gpu_peak_mib',
              'minimum_sampled_clearance_m', 'region_count', 'facets', 'repair_count',
              'normalized_path_length', 'cost', 'cost_over_endpoint_reference',
              'distance_improvement_over_seed_fraction')
    return dict(requested_jobs=len(rows), legal_jobs=len(legal), invalid_jobs=len(rows)-len(legal),
        legal_scene_clusters=len({r['cluster_id'] for r in legal}),
        status_counts=dict(Counter(r['status'] for r in rows)),
        legal_status_counts=dict(Counter(r['status'] for r in legal)),
        valid_success=_cluster_ratio(legal, lambda r: int(r['success']), lambda r: 1),
        conditional_native_drm=_cluster_ratio(native, lambda r: int(r['success']), lambda r: 1),
        conditional_validated_drm=_cluster_ratio(seed, lambda r: int(r['success']), lambda r: 1),
        dependency_or_infrastructure_resolved_rate=_cluster_ratio(legal,
            lambda r: int(not r['dependency_unsupported'] and not r['infrastructure_failed']), lambda r: 1),
        conditional_resolved_success=_cluster_ratio(resolved, lambda r: int(r['success']), lambda r: 1),
        conditional_resolved_validated_drm=_cluster_ratio([r for r in resolved if r['drm_success'] is True],
            lambda r: int(r['success']), lambda r: 1),
        resolved_scope='Only absence of an observed dependency/infrastructure failure. Not proof of general solver support; selected by observed status, not a population algorithm ranking.',
        statistics={field: {name: _stat([r[field] for r in pool]) for name, pool in
                    [('all_legal', legal), ('validated_success', success), ('failed_legal', failed)]}
                    for field in fields},
        stage_seconds={key: {name: _stat([r['completed_stage_seconds'].get(key) for r in pool]) for name, pool in
                    [('all_legal', legal), ('validated_success', success), ('failed_legal', failed)]}
                    for key in sorted({key for r in legal for key in r['timing']})},
        complete_backend_timing_jobs=sum(r['backend_timing_complete'] for r in legal),
        incomplete_backend_timing_jobs=sum(not r['backend_timing_complete'] for r in legal),
        stage_timing_scope='Frontend timers are included when assigned after their calls return. Backend inflation, alternate-path, optimization, validation and recovery counters are included only if the whole backend returned a candidate and final validation without exception, failed stage or incomplete terminal status, with both optimization and validation timers. Normal returned solver failures remain eligible. All counters from an incomplete backend, including completed early work and partial positive values, are excluded from stage statistics; raw timing remains in each observation. Genuine zero recovery on a complete backend is retained. These selected stage means are not unconditional pipeline latencies.',
        observed_output_attainment=[dict(threshold_seconds=t,
            **_cluster_ratio(legal, lambda r: int(r['success'] and r['cold_e2e_seconds']<=t), lambda r: 1))
            for t in LATENCY_THRESHOLDS],
        attainment_scope='Final validated output latency observed in this fixed-budget run. Not new runs with shorter optimizer budgets and not the first feasible iterate.',
        missing_values='Unavailable or incompletely returned backend stages are excluded, never zero-filled; each statistic reports count. Raw initialized zeros remain only as evidence in timing. Cold latency includes legal failures. Invalid input latency is excluded.')


def assert_same_fixed_input(rows, key_fields=('case_id', 'roadmap_id', 'method_seed')):
    """Check all requested inputs before legality or reporting strata filter them."""
    groups = defaultdict(list)
    for row in rows:
        groups[tuple(row[key] for key in key_fields)].append(row)
    for group in groups.values():
        for field in ('input_legal', 'input_sha256', 'robot_sha256', 'geometry_sha256',
                      'cluster_id', 'budget_seconds', 'family', 'distance_bin',
                      'straight_line_collision'):
            if any(row[field] != group[0][field] for row in group[1:]):
                raise ValueError(f'Fixed-task input mismatch on {field}; cannot compare different inputs')


def pair_metrics(rows, key_fields=('case_id', 'roadmap_id', 'method_seed')):
    """Only compare the same objective/model/endpoints; keep selection explicit."""
    assert_same_fixed_input(rows, key_fields)
    output = []
    for objective in sorted({r['objective'] for r in rows}):
        pool = [r for r in rows if r['objective']==objective and r['input_legal']]
        labels = sorted({r['label'] for r in pool})
        maps = {label: {tuple(r[k] for k in key_fields): r for r in pool if r['label']==label}
                for label in labels}
        if len({tuple(sorted(mapping)) for mapping in maps.values()}) > 1:
            raise ValueError('Methods have different fixed-task pairing keys')
        for a,b in itertools.combinations(labels, 2):
            pairs = []
            for key in sorted(maps[a].keys() & maps[b].keys()):
                left,right = maps[a][key],maps[b][key]
                for field in ('cluster_id','input_sha256','robot_sha256','geometry_sha256','budget_seconds'):
                    if left[field] != right[field]:
                        raise ValueError(f'Paired comparison differs on {field}')
                pairs.append(dict(cluster_id=left['cluster_id'], a=left,b=right))
            joint = [p for p in pairs if p['a']['success'] and p['b']['success']]
            supported = [p for p in pairs if not any(p[x]['dependency_unsupported'] or
                p[x]['infrastructure_failed'] for x in ('a','b'))]
            output.append(dict(method_a=a, method_b=b, objective=objective, paired_legal_jobs=len(pairs),
                valid_success_difference=_cluster_ratio(pairs, lambda p: int(p['a']['success'])-int(p['b']['success']),lambda p:1),
                common_success_cost_difference=_cluster_ratio(joint,lambda p:p['a']['cost']-p['b']['cost'],lambda p:1),
                common_success_cold_latency_difference=_cluster_ratio(joint,lambda p:p['a']['cold_e2e_seconds']-p['b']['cold_e2e_seconds'],lambda p:1),
                observed_cold_latency_difference=_cluster_ratio(pairs,lambda p:p['a']['cold_e2e_seconds']-p['b']['cold_e2e_seconds'],lambda p:1),
                resolved_pair_success_difference=_cluster_ratio(supported,lambda p:int(p['a']['success'])-int(p['b']['success']),lambda p:1),
                selection_note='Cost conditions on joint success. Resolved-pair analysis excludes observed dependency/infrastructure failures from BOTH methods on matching tasks, is secondary and selection-biased; all legal pairs remain in the primary result.'))
    return output


def analyze(runs, output, expected_labels=PAPER_LABELS, problem_ids=None):
    rows, manifests = load_rows(runs)
    if problem_ids is not None:
        if (not isinstance(problem_ids, list) or not problem_ids
                or any(not isinstance(value, str) for value in problem_ids)
                or len(set(problem_ids)) != len(problem_ids)):
            raise ValueError('The prespecified reporting subset needs a nonempty unique ID list')
        if set(problem_ids) - {row['case_id'] for row in rows}:
            raise ValueError('The prespecified reporting subset contains unavailable cases')
        rows = [row for row in rows if row['case_id'] in set(problem_ids)]
    assert_same_fixed_input(rows)
    groups, paired = [], []
    for split,nodes in sorted({(r['split'],r['roadmap_nodes']) for r in rows}):
        pool = [r for r in rows if (r['split'],r['roadmap_nodes'])==(split,nodes)]
        if {r['label'] for r in pool} != set(expected_labels):
            raise ValueError('Missing or unexpected methods in declared study')
        observed = defaultdict(set)
        for row in pool:
            observed[(row['case_id'],row['roadmap_id'],row['method_seed'])].add(row['label'])
        if any(labels != set(expected_labels) for labels in observed.values()):
            raise ValueError('Incomplete method pairing within a split/roadmap-size comparison')
        strata = [('obstacle_main',[r for r in pool if r['family']!='open_control']),
                  ('open_control',[r for r in pool if r['family']=='open_control'])]
        strata += [('family:'+f,[r for r in pool if r['family']==f])
                   for f in sorted({r['family'] for r in pool if r['family']!='open_control'})]
        strata += [('straight_line:'+str(value),[r for r in pool if r['family']!='open_control'
                     and r['straight_line_collision'] is value]) for value in (True,False)]
        strata += [('distance_bin:'+b,[r for r in pool if r['family']!='open_control' and r['distance_bin']==b])
                   for b in ('short','medium','long')]
        for scope,sample in strata:
            for label in sorted({r['label'] for r in sample}):
                selected = [r for r in sample if r['label']==label]
                groups.append(dict(split=split,roadmap_nodes=nodes,scope=scope,label=label,
                                   objective=selected[0]['objective'],**group_metrics(selected)))
            if sample:
                paired += [dict(split=split,roadmap_nodes=nodes,scope=scope,**p) for p in pair_metrics(sample)]
    drm = []
    # The reference is measured from the same, prespecified LSCS call, not the
    # fastest of five repetitions and not a manufactured standalone DRM run.
    for row in rows:
        if row['label']=='LSCS':
            drm.append({key:row[key] for key in ('case_id','split','cluster_id','family','roadmap_nodes','actual_roadmap_nodes',
                'roadmap_seed','method_seed','input_legal','native_drm_success','drm_success',
                'seed_cost_rad','seed_min_clearance_m','frontend_component_seconds','attempt')})
    pairing = defaultdict(list)
    for row in rows:
        pairing[(row['case_id'],row['roadmap_id'],row['method_seed'])].append(row)
    differences=[]
    for key,group in pairing.items():
        hashes={r['path_sha256'] for r in group if r['path_sha256']}
        fronts={(r['native_drm_success'],r['drm_success']) for r in group}
        if len(hashes)>1 or len(fronts)>1:
            differences.append(dict(key=list(key),path_hashes=sorted(hashes),frontend_outcomes=sorted(map(str,fronts))))
    result = dict(schema_version=1, complete=not differences, comparable=not differences,
        incomparability_reason='Shared frontend path or outcome mismatch; diagnostic output only.' if differences else None,
        runs=manifests, requested_jobs=len(rows),
        completeness_scope='All supplied runs and declared method combinations. Full preregistered case/size matrix must additionally be checked against its study protocol.',
        expected_method_labels=list(expected_labels),
        reporting_problem_ids=problem_ids,
        reporting_selection_scope='All original runs are verified first. An optional prespecified reporting subset reuses sealed observations without another planner call.',
        groups=groups, paired_comparisons=paired, drm_references=drm, pairing_differences=differences,
        drm_reference_scope='Validated native PWL seed from prespecified LSCS run; component time excludes interpreter/probe overhead and final optimizer. Not a standalone cold-runtime benchmark, not a time-trajectory optimizer.',
        objective_reference_scope='Distance: endpoint chord length. Time: independent-axis rest-to-rest acceleration/velocity relaxation, ignoring obstacles. Ratio is not an empirical optimum or a tight optimality gap.',
        statistics_scope='Scene cluster bootstrap with all endpoints/roadmap seeds together, 2000 PCG64 replicates, seed45827; open-control shared world is one cluster and is separate from obstacle_main.',
        roadmap_size_scope='Groups use the preregistered requested node budget. Actual retained nodes are separately recorded; the native builder may remove isolated nodes.',
        rows=rows)
    output=Path(output)
    output.mkdir(parents=True,exist_ok=True)
    write(output/'metrics.json',result)
    scalar_fields=[k for k,v in rows[0].items() if not isinstance(v,(dict,list))]
    with (output/'observations.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=scalar_fields,extrasaction='ignore')
        writer.writeheader();writer.writerows(rows)
    if differences:
        raise ValueError('Shared frontend mismatch recorded; inspect before comparing algorithms')
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runs',type=Path,nargs='+',required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--expected-labels',nargs='+',default=list(PAPER_LABELS))
    parser.add_argument('--problem-ids-json',type=Path,
                        help='Prespecified JSON list for reporting only; all source runs are still verified')
    args=parser.parse_args()
    result=analyze(args.runs,args.output,args.expected_labels,
                   read(args.problem_ids_json) if args.problem_ids_json else None)
    print(f"Verified {result['requested_jobs']} actual observations; wrote {args.output}")


if __name__=='__main__':
    main()
