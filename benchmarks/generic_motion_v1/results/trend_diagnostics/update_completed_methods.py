"""Refresh the method catalog only after complete repaired cohorts pass acceptance.

This CPU-only derived-data command never runs a planner or changes raw results.
An incomplete suite writes a pending report and leaves methods.json untouched.
"""
from __future__ import annotations

import argparse
from collections import Counter
import copy
import csv
import datetime
import json
import os
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import check_completed_evaluation as audit

ROOT = audit.ROOT
DEFAULT_COMPARISONS = ('current_smoke_repaired', 'current_dev_repaired', 'current_test_repaired')
FAILURES = ('no_solution_within_budget', 'timeout', 'solver_failure', 'validation_failed', 'infrastructure_error')


def preflight(runs):
    """Cheap pending gate; complete receipts still undergo the full verifier."""
    coverage = []
    for run in runs:
        if not (run/'run_manifest.json').is_file():
            coverage.append(dict(run=str(run), state='not_prepared'))
            continue
        manifest = audit.read(run/'run_manifest.json')
        terminal = 0
        for job in manifest['jobs']:
            attempts = sorted((run/'jobs'/job['job_id']/'attempts').glob('*'))
            receipt = attempts[-1]/'receipt.json' if attempts else None
            if receipt is not None and receipt.is_file() and not audit.read(receipt).get('retry_pending'):
                terminal += 1
        coverage.append(dict(run=str(run), split=manifest['split'],
                             declared=len(manifest['jobs']), terminal=terminal,
                             state='ready_for_verification' if terminal == len(manifest['jobs']) else 'pending'))
    return coverage


def repaired_protocol(manifests):
    """Derive descriptions from the actual frozen profile, never today's defaults."""
    configurations = {audit._fingerprint(job['evaluation_config']): job['evaluation_config']
                      for manifest in manifests for job in manifest['jobs']}
    if len(configurations) != 1:
        raise ValueError('Method catalog requires one verified frozen evaluation profile')
    cfg = next(iter(configurations.values()))
    expected = {'max_number_planning_attempts': 1000, 'max_iterations_steering_to_node': 1000,
                'max_nodes_to_expand': 100000, 'online_edge_step_size': 0.01}
    if any(cfg.get('drm', {}).get(key) != value for key, value in expected.items()):
        raise ValueError('Frozen DRM settings do not match the completed-repair profile')
    ompl = cfg.get('ompl', {})
    if any(ompl.get(key) != value for key, value in {
            'planning_seconds': 60., 'validation_reserve_seconds': 5., 'simplification_seconds': .5}.items()):
        raise ValueError('Frozen OMPL settings do not match the completed-repair profile')
    query = cfg.get('drm_query', {})
    if query.get('backend') != 'safe_native' or Path(query.get('build_dir', '')).name != 'drm_consistent_query_portable':
        raise ValueError('Frozen run does not use the reviewed portable consistent DRM query')
    if any(manifest['timeout_seconds'] != 60 for manifest in manifests):
        raise ValueError('Frozen run has a different outer budget')
    return cfg


def coverage_cell(group, summary_row, spec):
    counts = Counter(row['status'] for row in group)
    legal = [row for row in group if row['input_valid'] == 'True']
    numbers = {key: int(summary_row[key]) for key in (
        'planned_jobs', 'actual_attempts', 'legal_jobs', 'finished_legal_jobs',
        'invalid_input_jobs', 'pending_jobs', 'success_numerical', *FAILURES)}
    if (numbers['pending_jobs'] or len(group) != numbers['planned_jobs']
            or len(legal) != numbers['finished_legal_jobs']
            or counts['invalid_input'] != numbers['invalid_input_jobs']
            or any(counts[state] != numbers[state] for state in ('success_numerical', *FAILURES))):
        raise ValueError('Method coverage does not match selected completed records and summary')
    return dict(planned_jobs=numbers['planned_jobs'], finished_jobs=len(group),
        legal_jobs=numbers['legal_jobs'], finished_legal_jobs=len(legal),
        invalid_input_jobs=numbers['invalid_input_jobs'], success_numerical=counts['success_numerical'],
        failure_counts={state: counts[state] for state in FAILURES},
        run_paths=sorted({row['run'] for row in group}), comparison_id=spec['id'],
        selection_policy='First terminal attempt per declared job; no best-of.',
        mode=spec.get('mode', 'end_to_end'), split=summary_row['split'],
        planner_replica_seeds=sorted({int(row['replica_seed']) for row in group}),
        actual_attempts=numbers['actual_attempts'], pending_jobs=0,
        evidence_scope='Complete repaired cohort; raw integrity and full matrix accepted; counts cross-checked against records.csv and summary.csv.')


def build_catalog(original, records, summary, specs, manifests, acceptance, evidence):
    """Pure transform: preserve implementation identities, replace current coverage."""
    if not acceptance.get('passed') or not acceptance.get('complete_primary_suite'):
        raise ValueError('All three repaired primary cohorts must pass acceptance before catalog refresh')
    cfg = repaired_protocol(manifests)
    methods = {item['method']: item for item in original['methods']}
    declared_pairs = {(name, objective) for name, method in methods.items() for objective in method['objectives']}
    if len(methods) != len(original['methods']) or declared_pairs != set(audit.PAIRS):
        raise ValueError('Method identities do not match the verified 10-method/14-combination catalog')
    catalog = copy.deepcopy(original)
    catalog_methods = {item['method']: item for item in catalog['methods']}
    for method in catalog_methods.values():
        method['actual_coverage'] = {}
        # Earlier partial repair remains available in Git and its original run;
        # it must not appear beside new scores as if using the same protocol.
        method.pop('repaired_dev_seed0', None)
    primary_cells = set()
    for spec in specs:
        mode = spec.get('mode', 'end_to_end')
        block = [row for row in records if spec['id'] in audit.bounds.membership(row)]
        status_rows = [row for row in summary if row['comparison'] == spec['id']
                       and row['row_type'] == 'status' and row['category'] == 'all']
        used = set()
        for row in status_rows:
            key = (row['method'], row['objective'], row['split'])
            if key in used or row['mode'] != mode:
                raise ValueError('Duplicate or mixed-mode method summary')
            used.add(key)
            group = [item for item in block if (item['method'], item['objective'], item['split']) == key]
            cell = coverage_cell(group, row, spec)
            coverage = catalog_methods[row['method']]['actual_coverage'].setdefault(mode, {})
            if mode == 'end_to_end':
                if key in primary_cells:
                    raise ValueError('Multiple current primary cohorts for one method/track/split')
                primary_cells.add(key)
                coverage.setdefault(row['objective'], {})[row['split']] = cell
            elif mode == 'fixed_initial_regions':
                coverage.setdefault(row['objective'], {})[row['split']] = cell
            else:
                raise ValueError('Post-hoc conditional subsets are not current full method coverage')
    expected_cells = {(method, objective, split) for method, objective in audit.PAIRS for split in audit.SEEDS}
    if primary_cells != expected_cells:
        raise ValueError('Missing or extra method/track/split coverage cells')

    drm = ('Native portable consistent DRM query: at most 1000 planning attempts, 1000 endpoint-connection '
           'trials per endpoint per attempt, 100000 graph expansions and 0.01-rad steering; all query '
           'collision-map reconciliation, search and repair consume the shared 60-second outer deadline.')
    ompl = ('Native search ceiling 60 s, further bounded by the remaining outer 60-second deadline minus '
            '5 s final-validation reserve and 0.5 s simplification reserve; setup consumes the same outer '
            'budget; vertex reduction at most 0.5 s.')
    for method in catalog_methods.values():
        if method.get('roadmap_required'):
            # Strip only our own previous prefix so rerunning is idempotent.
            previous = method['stopping'].split(' Backend: ', 1)[-1]
            method['stopping'] = drm + ' Backend: ' + previous
            method['drm_query_protocol'] = dict(cfg['drm'], **cfg['drm_query'], outer_budget_seconds=60)
        if method['method'] in ('RRTCONNECT', 'RRTSTAR'):
            method['stopping'] = ompl + (' RRTConnect stops at its first exact solution.'
                if method['method'] == 'RRTCONNECT' else
                ' RRTstar retains its native tree and improves until this remaining budget; main profile disables the validated-progress observer.')
        if method['method'] in ('NTO', 'NTO_PAPER', 'NTO_REFINED'):
            method['collision_backend_repair'] = ('Native-first solve uses direct Drake collision callbacks without an ordinary '
                'Python guard. Only the exact reproduced BoxBoxGradient near-contact rounding assertion permits at most '
                'one guarded recovery solve on the same mathematical program, with the same original query deadline '
                'and remaining budget; no fresh query or budget reset. Recovery retains all canonical pairs and the '
                'native aggregate, with finite-difference gradient convergence and AutoDiff chain checks. Unknown '
                'errors and non-contact disagreements still propagate; no collision constraint is dropped.')
            method['collision_evaluation_accounting'] = ('native_evaluations counts only calls through the guarded Python '
                'callback. Direct native solver callback evaluations are not instrumented and their count is unknown; '
                'a saved zero means zero guarded callback calls, not zero collision work or zero native evaluations.')
        if method['method'] in ('NTO', 'NTO_PAPER', 'NTO_REFINED'):
            method['historical_backend_evidence'] = original.get('known_backend_issue', {}).get(
                'historical_evidence', original.get('known_backend_issue', {}).get(
                    'evidence', 'results/nto_backend_issue.json'))
            method['verification'] = ('Current coverage is generated only after complete-cohort receipt, '
                'fixed-input, model, protocol, state-accounting and metric acceptance. Solver termination '
                'and independent finite-sampling validation remain separate; no continuous certificate is claimed. '
                'Earlier backend failures remain in historical_backend_evidence; current repair evidence is '
                'results/trend_diagnostics/nto_collision_independent_review.json.')
        if method['method'] == 'NTO_PAPER':
            method['distinction'] = ('Published-profile Drake KTO reconstruction, not unpublished author code. '
                'Nc=150 maps to 149 interior k/150 bindings from archived author TeX; the published wording '
                'alone does not resolve that transcription detail. Historical BoxBoxGradient failures are '
                'retained; the current exact-assertion recovery is described in collision_backend_repair.')
            method['correctness'] = ('Real SNOPT path and IPOPT time solves with canonical collision constraints; '
                'the native-first callback and bounded exact-assertion recovery preserve all collision pairs. '
                'All candidates require the common independent numerical validator.')
            method['initialization'] = ('Path uses the original DRM vertices. Time duplicates the first and '
                'last control once each; the published-profile variant inserts no additional knots or controls.')
        if method['method'] == 'NTO':
            minimum_controls = cfg['nto']['minimum_control_points']
            method['distinction'] = ('Local historical Drake KTO/IPOPT adaptation; it uses IPOPT for both tracks '
                'and does not equal the paper SNOPT path profile. Historical backend exceptions remain recorded; '
                'the current exact-assertion recovery is described in collision_backend_repair.')
            method['correctness'] = ('Real Drake KTO/IPOPT transcription with canonical collision constraints, '
                'native-first callbacks and bounded exact-assertion recovery. It remains a separately labeled '
                'adaptation and every output undergoes common independent numerical validation.')
            method['initialization'] = (f'Path retains the original DRM controls. Time duplicates both endpoints '
                f'once and bisects longest knot spans with shape-preserving InsertKnots until at least '
                f'{minimum_controls} controls; there is no arc-length resampling.')
        if method['method'] == 'NTO_REFINED':
            minimum_controls = cfg.get('nto_refined', {}).get('minimum_time_control_points', 16)
            method['distinction'] = ('Declared local numerical adaptation of the published NTO profile. '
                'The post-solve geometry guard adds collocation constraints; the separate native-first '
                'collision backend handles only the reviewed exact BoxBoxGradient assertion. Neither '
                'mechanism drops canonical collision pairs or relaxes final validation.')
            method['correctness'] = ('Real SNOPT/IPOPT solves with positive optimization clearance, '
                'collision-driven constraint refinement and bounded exact-assertion recovery. The fixed '
                'robot model, endpoints, motion limits and final validator are unchanged.')
            method['initialization'] = (f'Path retains the original DRM controls. Time duplicates both endpoints '
                f'once and uses shape-preserving knot insertion to at least {minimum_controls} controls. '
                'Each later refinement round starts from the preceding native candidate on the same basis.')
            method['current_time_parameterization'] = (f'At least {minimum_controls} time-spline controls through '
                'shape-preserving knot insertion; the initial curve, motion limits and endpoint conditions '
                'are unchanged. Current outcomes are reported only in actual_coverage.')
        if method['method'] == 'NTO_REFINED':
            method['stopping'] = drm + (' Backend: At most 4 refinement rounds and 600 collocation positions. '
                'At query level, only the exact known BoxBoxGradient assertion may abort a native Solve and add '
                'at most 1 guarded recovery Solve, so there are at most 5 actual solver invocations. Later refinement '
                'rounds retain the guarded evaluator after recovery. All construction, solves, recovery, checks and '
                'validation share the original 60 s deadline; SNOPT total/major/minor limits 100000/2000/500.')
    sources = manifests[0]['source_hashes']
    prefix = 'benchmarks/generic_motion_v1/'
    catalog['source_sha256'] = {}
    for key in original.get('source_sha256', {}):
        if prefix+key not in sources:
            raise ValueError('Method source absent from frozen source inventory: '+key)
        catalog['source_sha256'][key] = sources[prefix+key]
    for key in ('runners/native_drm_corrections.py', 'runners/nto_collision_constraint.py'):
        if prefix+key not in sources:
            raise ValueError('Reviewed repair source absent from frozen source inventory: '+key)
        catalog['source_sha256'][key] = sources[prefix+key]
    catalog['source_compatibility'] = ('Current coverage uses only the explicitly accepted repaired cohorts and their frozen '
        'source/configuration. Historical OMPL native search was capped at 10 s; current cap is 60 s within '
        'the shared outer budget. Changes in results cannot be attributed solely to DRM/NTO bug repair.')
    catalog['known_backend_issue'] = dict(
        status='Known BoxBoxGradient near-contact assertion strictly repaired with native-first solve and at most one guarded recovery under the same original deadline; historical failed observations remain unchanged.',
        historical_evidence=original.get('known_backend_issue', {}).get('historical_evidence',
            original.get('known_backend_issue', {}).get('evidence', 'results/nto_backend_issue.json')),
        exact_q_captured=True, repair_source_sha256=sources[prefix+'runners/nto_collision_constraint.py'],
        independent_review='results/trend_diagnostics/nto_collision_independent_review.json',
        remaining_policy='Unknown backend errors remain infrastructure_error and reject completed-suite acceptance.')
    catalog.update(actual_coverage_status='All three repaired end-to-end smoke/dev/test cohorts complete and verified. Only explicitly accepted extra fixed-initial cohorts appear in current coverage.',
        actual_coverage_evidence=evidence, latest_review_at_utc=evidence['updated_at_utc'],
        latest_review_scope='CPU derivation after strict complete-suite receipt, matrix, protocol, normalized-state, accounting and metric acceptance; no additional planning.',
        repair_review_scope='Current coverage supersedes earlier partial repair and historical scores; prior attempts remain in records.csv and original run directories.',
        head_at_repair_review=sorted({manifest['source_code_commit'] for manifest in manifests}),
        historical_evidence_scope='Original tests, changes, limitations and review history are preserved as historical identity evidence. Current coverage and stopping conditions come only from the accepted repaired runs.')
    return catalog


def update(args):
    selection = audit.read(args.selection)
    runs, specs = audit.resolve_scope(selection, comparison_args=args.comparisons)
    coverage = preflight(runs)
    if any(row['state'] != 'ready_for_verification' for row in coverage):
        return dict(status='pending', passed=False, catalog_written=False, coverage=coverage)
    acceptance = audit.review(runs, specs, records_path=args.records, summary_path=args.summary, reference_run=args.reference_run)
    if not acceptance.get('passed') or not acceptance.get('complete_primary_suite'):
        return dict(status=acceptance['status'] if not acceptance['passed'] else 'failed', passed=False,
                    catalog_written=False, acceptance=acceptance,
                    errors=[] if not acceptance['passed'] else ['Missing one of smoke/dev/test primary cohorts'])
    original_sha = audit.sha(args.output)
    original = audit.read(args.output)
    with args.records.open() as stream:
        records = list(csv.DictReader(stream))
    with args.summary.open() as stream:
        summary = list(csv.DictReader(stream))
    evidence = dict(updated_at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        records=str(args.records), records_sha256=acceptance['records_sha256'],
        summary=str(args.summary), summary_sha256=acceptance['summary_sha256'],
        selection=str(args.selection), selection_sha256=audit.sha(args.selection),
        accepted_comparisons=[spec['id'] for spec in specs],
        previous_catalog_sha256=original_sha, completed_groups=acceptance['completed_groups'],
        acceptance_checker_sha256=acceptance['checker_sha256'],
        counts_denominator='Legal finished queries; paired seeds are repeated queries, not independent scene layouts.')
    catalog = build_catalog(original, records, summary, specs,
        [audit.read(run/'run_manifest.json') for run in runs], acceptance, evidence)
    if (audit.sha(args.output) != original_sha or audit.sha(args.records) != acceptance['records_sha256']
            or audit.sha(args.summary) != acceptance['summary_sha256'] or audit.read(args.selection) != selection):
        raise ValueError('Evidence or catalog changed during method refresh; refusing to write')
    temporary = args.output.with_suffix('.json.tmp')
    temporary.write_text(json.dumps(catalog, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    os.replace(temporary, args.output)
    return dict(status='passed', passed=True, catalog_written=True, catalog=str(args.output),
                catalog_sha256=audit.sha(args.output), evidence=evidence)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--comparisons', nargs='+', default=list(DEFAULT_COMPARISONS))
    for key, default in [('selection', ROOT/'results/selection.json'), ('records', ROOT/'results/records.csv'),
                         ('summary', ROOT/'results/summary.csv'), ('output', ROOT/'results/methods.json'),
                         ('reference-run', ROOT/'results/repaired_dev'), ('report', HERE/'methods_update.json')]:
        parser.add_argument('--'+key, type=Path, default=default)
    args = parser.parse_args(argv)
    try:
        report = update(args)
    except (OSError, ValueError, KeyError, TypeError, RuntimeError) as exc:
        report = dict(status='failed', passed=False, catalog_written=False, error=repr(exc))
    destination = audit.save_report(report, args.report)
    print(json.dumps(dict(status=report['status'], catalog_written=report['catalog_written'], report=str(destination))))
    return 0 if report['passed'] else 2 if report['status'] == 'pending' else 1


if __name__ == '__main__':
    raise SystemExit(main())
