"""Render original-paper comparator evidence after complete planning and audit.

No planner is called. The seven-label table is checked, but only the five paper
comparators supply the primary gallery. Failed optimizers show a marked saved
DRM seed, never their failed candidate or a new fallback solution.
"""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from contextlib import redirect_stderr, redirect_stdout
import html
import json
from pathlib import Path
import shlex
import sys
import traceback

from ..baseline_study.make_formal_replays import (
    check_selected, label_replay, read, require, seed_projection, sha, write,
)
from ..baseline_repair_v2.make_development_replays import source_snapshot, unique

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
LABELS = ('LSCS', 'LGCS', 'NTO-distance', 'SCSTO', 'NTO-time')
ALL_LABELS = LABELS + ('NTO-refined-distance', 'NTO-refined-time')
METHODS = dict(zip(ALL_LABELS, (
    ('EI_ZO', 'minimum_distance'), ('LGCS', 'minimum_distance'),
    ('NTO_PAPER', 'minimum_distance'), ('EI_ZO', 'minimum_time'),
    ('NTO_PAPER', 'minimum_time'), ('NTO_REFINED', 'minimum_distance'),
    ('NTO_REFINED', 'minimum_time'))))
BLOCKS = {'smoke_s000': 20, 'dev_s000': 120, 'dev_s001': 120, 'dev_s002': 120}
SUCCESS = 'success_validated_numerical'
RULE = ('Use complete, independently audited development evidence only. Select the '
    'lexicographically first case/roadmap-seed/roadmap-id/method-seed group where '
    'all five original paper comparator labels succeeded, preferring input '
    'straight_line_collision=True. If no common group exists, use that ordering '
    'within each original label and disclose that the inputs differ. Never replace '
    'original NTO by NTO_REFINED. If the selected original NTO-time view is not '
    'blocked, add the first blocked validated NTO-time view when one exists. '
    'Add the first failed original-comparator dev attempt with a saved validated '
    'DRM seed under the same blocked-first lexical ordering; show that seed only. '
    'No cost, latency, failure-type or appearance ranking. This selected gallery '
    'is an illustration, not a performance sample or a success-rate estimate.')


def completion_barrier(here, planning_finished, audit_path):
    """Do not read metric rows until full completion and a bound audit pass."""
    require(planning_finished is True, 'Explicit --planning-finished is required')
    index, done, frozen = (read(here / name) for name in
        ('run_index.json', 'completed_blocks.json', 'execution_freeze.json'))
    require(index['slots'] == 380 and index['test_planner_calls'] == 0,
            'Wrong declared new study size or formal-test scope')
    require(done.get('complete') is True and done['completed_jobs'] == done['total_jobs'] == 380,
            'All 380 new slots must be complete before replay')
    require(frozen['planned_registered_slots'] == 380 and frozen['test_planner_calls'] == 0,
            'Wrong frozen study scope')
    blocks = unique(index['blocks'], lambda b: b['name'], 'new block')
    completed = unique(done['blocks'], lambda b: b['name'], 'completed block')
    require(blocks.keys() == completed.keys() == BLOCKS.keys(), 'Missing or duplicate completion blocks')
    runs = {}
    for name, block in blocks.items():
        require(block['planned_jobs'] == BLOCKS[name] and block['split'] == name.split('_')[0],
                'New block size or split changed')
        require({k: v for k, v in completed[name].items() if k != 'verification'} == block,
                'Completion does not match indexed block')
        check = completed[name]['verification']
        require(check.get('complete') is True and check.get('integrity_passed') is True and
                check['completed_jobs'] == check['planned_jobs'] == BLOCKS[name] and
                check.get('errors') == [] and check.get('unsealed_attempts') == [],
                'New block is not completely verified')
        runs[str(Path(block['run']).resolve())] = block['run_manifest_sha256']
    reused = frozen['reused_evidence']
    require(len(reused) == 4 and sum(r['jobs'] for r in reused.values()) == 950,
            'Expected exactly four previously sealed runs / 950 slots')
    for path, evidence in reused.items():
        check = evidence['verification']
        require(check.get('complete') is True and check.get('integrity_passed') is True and
                check.get('errors') == [] and check.get('unsealed_attempts') == [],
                'Reused evidence was not verified before the new freeze')
        path = str(Path(path).resolve())
        require(path not in runs, 'Reused and new runs overlap')
        runs[path] = evidence['manifest_sha256']
    require(len(runs) == 8, 'Expected eight distinct source runs')
    for path, digest in runs.items():
        require(sha(Path(path) / 'run_manifest.json') == digest, 'Source run manifest changed')
    audit = read(audit_path)
    require(audit.get('passed') is True and audit.get('errors') == [] and
            (audit['registered_slots'], audit['new_registered_slots'], audit['reused_registered_slots'],
             audit['legal_slots'], audit['invalid_slots'], audit['new_legal_slots']) == (1330, 380, 950, 1316, 14, 376) and
            audit['no_test_planner_calls'] is True, 'Complete error-free comparator audit required')
    require(audit['freeze_sha256'] == sha(here / 'execution_freeze.json') and
            audit['run_index_sha256'] == sha(here / 'run_index.json'), 'Audit is not bound to the current study freeze/index')
    require(set(audit['source_runs']) == set(runs), 'Audit refers to a different source-run set')
    for path, digest in runs.items():
        item = audit['source_runs'][path]
        require(item['manifest_sha256'] == digest and item['integrity'].get('complete') is True and
                item['integrity'].get('integrity_passed') is True, 'Audit source binding changed')
    for name, digest in frozen['runtime_source_hashes'].items():
        require(sha(ROOT / name) == digest, 'Frozen runtime source changed: ' + name)
    evidence_paths = [here / n for n in
        ('run_index.json', 'completed_blocks.json', 'execution_freeze.json')] + [Path(audit_path)]
    return dict(run_hashes=runs, audit=audit,
                evidence={p.name: dict(path=str(p.resolve()), sha256=sha(p)) for p in evidence_paths})


def check_metrics(metrics, metrics_path, barrier):
    require(sha(metrics_path) == barrier['audit']['metrics_sha256'], 'Merged metrics changed after audit')
    require(metrics.get('complete') is True and metrics.get('comparable') is True and
            metrics['requested_jobs'] == len(metrics['rows']) == 1330,
            'Only the complete comparable 1330-slot matrix is allowed')
    require(not metrics.get('reporting_problem_ids') and not metrics.get('pairing_differences'),
            'Filtered or unmatched metrics cannot supply the gallery')
    require(set(metrics['expected_method_labels']) == set(ALL_LABELS), 'Wrong metric method labels')
    runs = unique(metrics['runs'], lambda r: str(Path(r['run']).resolve()), 'metric run')
    require(runs.keys() == barrier['run_hashes'].keys(), 'Wrong merged run set')
    for run, info in runs.items():
        require(info['manifest_sha256'] == barrier['run_hashes'][run] and
                info['source_commit'] == barrier['audit']['source_runs'][run]['source_commit'],
                'Metric source binding changed')
    rows = metrics['rows']
    unique(rows, lambda r: (r['case_id'], r['roadmap_id'], r['method_seed'], r['label']), 'metric slot')
    require(Counter((r['split'], r['label']) for r in rows) == Counter({
        (split, label): n for split, n in (('smoke', 10), ('dev', 180)) for label in ALL_LABELS}),
        'Wrong split/method coverage')
    require(sum(r['input_legal'] is True for r in rows) == 1316, 'Wrong legal denominator')
    groups = defaultdict(list)
    for row in rows:
        require((row['method'], row['objective']) == METHODS[row['label']], 'Wrong method/objective label')
        require(row['success'] is (row['status'] == SUCCESS), 'Success/status disagreement')
        require(row['roadmap_nodes'] == 6000, 'Wrong roadmap size')
        if row['split'] == 'dev':
            require(row['input_legal'] is True and row['family'] != 'open_control', 'Invalid dev row')
        run, attempt = Path(row['run']).resolve(), Path(row['attempt']).resolve()
        require(str(run) in runs and attempt.is_relative_to(run / 'jobs') and
                attempt.parent.name == 'attempts', 'Attempt outside sealed run')
        groups[(row['case_id'], row['roadmap_id'], row['method_seed'])].append(row)
    for group in groups.values():
        require(len(group) == 7 and {r['label'] for r in group} == set(ALL_LABELS), 'Incomplete seven-label query')
        for field in ('q_start', 'q_goal', 'input_sha256', 'robot_sha256', 'geometry_sha256',
                      'family', 'straight_line_collision', 'input_legal', 'roadmap_seed', 'split'):
            require(all(r[field] == group[0][field] for r in group), 'Paired input changed: ' + field)


def order(row):
    return (row['straight_line_collision'] is not True, row['case_id'], row['roadmap_seed'],
            row['roadmap_id'], row['method_seed'], row['label'], row['job_id'])


def select_illustrations(rows):
    rows = [r for r in rows if r['split'] == 'dev' and r['label'] in LABELS]
    unique(rows, lambda r: (r['case_id'], r['roadmap_id'], r['method_seed'], r['label']), 'selection slot')
    success = [r for r in rows if r['input_legal'] is True and r['success'] is True and r['status'] == SUCCESS]
    groups = defaultdict(list)
    for row in success:
        groups[(row['case_id'], row['roadmap_id'], row['method_seed'])].append(row)
    common = [g for g in groups.values() if len(g) == 5 and {r['label'] for r in g} == set(LABELS)]
    selected = min(common, key=lambda g: order(next(r for r in g if r['label'] == 'LSCS'))) if common else []
    if not common:
        selected = [min(pool, key=order) for label in LABELS if (pool := [r for r in success if r['label'] == label])]
    choices = [dict(name=r['label'], role='validated_final_trajectory', row=r)
               for r in sorted(selected, key=lambda r: LABELS.index(r['label']))]
    missing = [dict(label=label, reason='No validated original-comparator dev trajectory')
               for label in LABELS if label not in {r['label'] for r in selected}]
    time_view = next((r for r in selected if r['label'] == 'NTO-time'), None)
    optional = []
    if time_view is None or time_view['straight_line_collision'] is not True:
        pool = [r for r in success if r['label'] == 'NTO-time' and r['straight_line_collision'] is True]
        if pool:
            choices.append(dict(name='NTO-time_blocked', role='validated_final_trajectory', row=min(pool, key=order)))
        else:
            optional.append(dict(label='NTO-time_blocked', reason='No blocked original NTO-time success; no substitute'))
    failed = [r for r in rows if r['input_legal'] is True and r['success'] is False and r['status'] != SUCCESS
              and r['drm_success'] is True and r['seed_cost_validated'] is True]
    if failed:
        choices.append(dict(name='failure_seed_only', role='verified_frontend_seed_only', row=min(failed, key=order)))
    else:
        missing.append(dict(label='failure_seed_only', reason='No legal failed original comparator with a validated saved seed'))
    return choices, dict(rule=RULE, shared_case_for_five_methods=bool(common),
        common_query_straight_line_collision=selected[0]['straight_line_collision'] if common else None,
        missing=missing, unavailable_optional=optional)


def gallery(record, output):
    selection = record['selection']
    common = ('The five primary methods share one fixed query. Its straight-line collision label is ' +
        str(selection['common_query_straight_line_collision']) + '.' if selection['shared_case_for_five_methods'] else
        'No common successful query exists for all five methods; the primary views use individually selected inputs.')
    cards = []
    for item in record['illustrations']:
        note = ('SEED ONLY — failed attempt; not an optimized final solution' if
                item['role'] == 'verified_frontend_seed_only' else 'Saved validated final trajectory')
        content = '<h2>' + html.escape(item['name']) + '</h2><p>' + html.escape(note) + '</p><p>' + html.escape(
            f"{item['case_id']} · roadmap {item['roadmap_seed']} · {item['source_status']}") + '</p>'
        content += '<p>Input straight-line collision: ' + str(item['straight_line_collision']) + '</p>'
        if item.get('status') == 'created':
            content += '<a href="' + item['name'] + '/replay.html">Open interactive replay</a><br>'
            content += '<img src="' + item['name'] + '/scene.png" alt="Saved canonical collision-model scene">'
        else:
            content += '<p>Replay failed; the error and original source are retained in index.json.</p>'
        cards.append('<article>' + content + '</article>')
    unavailable = selection['missing'] + selection['unavailable_optional']
    notes = ''.join('<li>' + html.escape(x['label'] + ': ' + x['reason']) + '</li>' for x in unavailable)
    page = ('<!doctype html><html lang="en"><meta charset="utf-8"><title>Original paper comparator replays</title>'
        '<style>body{font-family:system-ui;max-width:1050px;margin:30px auto;padding:20px}article{border-top:1px solid #ccd;padding:20px 0}img{max-width:720px;width:100%}p{line-height:1.5}</style>'
        '<h1>Kinova benchmark: original paper comparators</h1><p>' + html.escape(common) + '</p><p>' +
        html.escape(RULE) + '</p><p>Gallery complete: ' + str(record['complete']) +
        '. Finite numerical collision checks are not a continuous safety proof.</p>' +
        ('<ul>' + notes + '</ul>' if notes else '') + ''.join(cards))
    (output / 'index.html').write_text(page)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--planning-finished', action='store_true')
    parser.add_argument('--metrics', type=Path, default=HERE / 'comparison_metrics/metrics.json')
    parser.add_argument('--audit', type=Path, default=HERE / 'comparison_audit.json')
    parser.add_argument('--output', type=Path, default=HERE / 'replays')
    args = parser.parse_args(argv)
    barrier = completion_barrier(HERE, args.planning_finished, args.audit)
    metrics = read(args.metrics)
    check_metrics(metrics, args.metrics, barrier)
    choices, selection = select_illustrations(metrics['rows'])
    output = args.output.resolve()
    require(output.is_relative_to(HERE) and output != HERE and not output.exists(),
            'Output must be a new directory under paper_comparators_v1')
    output.mkdir(parents=True, exist_ok=False)
    helpers = [ROOT / 'benchmarks/generic_motion_v1/reports' / part for part in
        ('baseline_study/make_formal_replays.py', 'baseline_repair_v2/make_development_replays.py')]
    record = dict(complete=False, selection=selection, planning_complete_slots=380,
        combined_evidence_slots=1330, evidence=barrier['evidence'],
        metrics=dict(path=str(args.metrics.resolve()), sha256=sha(args.metrics)),
        command=shlex.join(sys.orig_argv), script_sha256=sha(__file__),
        reused_helpers_sha256={str(p): sha(p) for p in helpers}, illustrations=[], failures=[])
    write(output / 'index.json', record)
    from ...batch import verify
    from ...runners.replay import replay
    verified = {}
    for choice in choices:
        row, name = choice['row'], choice['name']
        item = dict(name=name, role=choice['role'], case_id=row['case_id'], job_id=row['job_id'],
            label=row['label'], source_status=row['status'], source_attempt=row['attempt'],
            roadmap_seed=row['roadmap_seed'], roadmap_id=row['roadmap_id'], method_seed=row['method_seed'],
            straight_line_collision=row['straight_line_collision'])
        record['illustrations'].append(item)
        attempt, before = Path(row['attempt']), None
        try:
            with (output / (name + '.log')).open('x') as log, redirect_stdout(log), redirect_stderr(log):
                if row['run'] not in verified:
                    check = verify(row['run'])
                    require(check.get('complete') is True and check.get('integrity_passed') is True,
                            'Selected source run is no longer sealed and complete')
                    verified[row['run']] = check
                receipt = check_selected(row)
                before = source_snapshot(attempt)
                item['source_receipt_sha256'] = sha(attempt / 'receipt.json')
                source = attempt
                if choice['role'] == 'verified_frontend_seed_only':
                    source = output / 'seed_source_projection'
                    item['projection'] = seed_projection(row, source, receipt)
                else:
                    require(read(source / 'validation.json').get('passed') is True and
                            'trajectory_samples.npz' in receipt['files'], 'Final validated samples are not sealed')
                target = output / name
                result = replay(source, output=target)
                data = label_replay(target, row, choice['role'], item['source_receipt_sha256'])
                require(before == source_snapshot(attempt), 'Original source changed during replay')
                item.update(status='created', output=str(target), replay=result, source_unchanged=True,
                    sample_source_sha256=data['sample_source_sha256'],
                    files={str(p.relative_to(target)): sha(p) for p in sorted(target.rglob('*')) if p.is_file()})
        except Exception:
            error = traceback.format_exc()
            item.update(status='failed', error=error)
            record['failures'].append(dict(name=name, error=error))
            (output / (name + '.failure.log')).write_text(error)
        finally:
            if before is not None:
                item['source_unchanged'] = before == source_snapshot(attempt)
                if not item['source_unchanged']:
                    item['status'] = 'failed'
                    record['failures'].append(dict(name=name, error='Original source changed during replay'))
            write(output / 'index.json', record)
    record['source_run_verification'] = verified
    record['complete'] = not record['failures'] and not selection['missing']
    gallery(record, output)
    record['gallery_sha256'] = sha(output / 'index.html')
    write(output / 'index.json', record)
    print(json.dumps(dict(complete=record['complete'], output=str(output),
                         illustrations=len(record['illustrations']), failures=record['failures']), indent=2))
    if not record['complete']:
        raise SystemExit('Replay gallery incomplete; missing views and real errors are retained')


if __name__ == '__main__':
    main()
