"""Render saved development evidence only after the complete 950-slot audit gate.

Selection never ranks costs, timings or appearance. No planner is called. Original
sealed attempts are read-only; even failed-attempt seed views use report copies.
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

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
SOURCE = 'e6b7e372e091ce5b01417073e0880e7c3068910c'
LABELS = ('LSCS', 'LGCS', 'NTO-refined-distance', 'SCSTO', 'NTO-refined-time')
METHODS = dict(zip(LABELS, (('EI_ZO', 'minimum_distance'), ('LGCS', 'minimum_distance'),
    ('NTO_REFINED', 'minimum_distance'), ('EI_ZO', 'minimum_time'),
    ('NTO_REFINED', 'minimum_time'))))
SUCCESS = 'success_validated_numerical'
BLOCKS = {'smoke_s000': 50, 'dev_s000': 300, 'dev_s001': 300, 'dev_s002': 300}
RULE = ('Use development rows only, after all 950 smoke/dev slots and three independent audits pass. '
    'Prefer the lexicographically first case/roadmap-seed/roadmap-id/method-seed group in which all '
    'five labels have validated final trajectories, prioritizing straight_line_collision=True. '
    'If none exists, apply that same ordering separately to each label. Add one legal failed '
    'attempt with a saved validated frontend seed under the same blocked-first lexical ordering. '
    'No cost, latency, failure-type or appearance ranking. A failure view displays only its '
    'saved frontend seed. Selected illustrations are not a performance sample or success estimate.')


def unique(items, key, what):
    result = {}
    for item in items:
        k = key(item)
        require(k not in result, 'Duplicate ' + what + ': ' + str(k))
        result[k] = item
    return result


def completion_barrier(here, planning_finished):
    """No metrics file is read until this function has completely passed."""
    require(planning_finished is True, 'Explicit --planning-finished is required')
    index, done, frozen = (read(here / name) for name in
        ('run_index.json', 'completed_blocks.json', 'execution_freeze.json'))
    require(index['slots'] == 950 and index['test_planner_calls'] == 0,
            'Only the declared 950-slot smoke/dev study is permitted')
    require(done.get('complete') is True and done['completed_jobs'] == done['total_jobs'] == 950,
            'All 950 slots must be sealed before reading metrics')
    require(frozen['source_commit'] == SOURCE and frozen['planned_registered_slots'] == 950,
            'Wrong frozen runtime or study size')
    blocks = unique(index['blocks'], lambda b: b['name'], 'declared block')
    completed = unique(done['blocks'], lambda b: b['name'], 'completed block')
    require(blocks.keys() == completed.keys() == BLOCKS.keys(), 'Wrong or missing completed blocks')
    run_hashes = {}
    for name, block in blocks.items():
        require(block['planned_jobs'] == BLOCKS[name] and block['split'] == name.split('_')[0],
                'Block size/split differs from declaration')
        require({k: v for k, v in completed[name].items() if k != 'verification'} == block,
                'Completion entry differs from indexed block')
        check = completed[name]['verification']
        require(check.get('complete') is True and check.get('integrity_passed') is True and
                check['completed_jobs'] == check['planned_jobs'] == BLOCKS[name] and
                check.get('errors') == [] and check.get('unsealed_attempts') == [],
                'Block has unverified or unsealed attempts')
        run = Path(block['run']).resolve()
        require(str(run) not in run_hashes, 'Duplicate run directory')
        require(sha(run / 'run_manifest.json') == block['run_manifest_sha256'], 'Run manifest changed')
        run_hashes[str(run)] = block['run_manifest_sha256']
    audits = {name: read(here / (name + '_audit.json')) for name in ('integrity', 'lgcs', 'nto')}
    require(all(a.get('passed') is True and a.get('errors') == [] for a in audits.values()),
            'Three passing error-free independent audits are required')
    index_sha, freeze_sha = sha(here / 'run_index.json'), sha(here / 'execution_freeze.json')
    integrity, lgcs, nto = (audits[name] for name in ('integrity', 'lgcs', 'nto'))
    require(integrity.get('complete') is True and integrity['source_code_commit'] == SOURCE and
            (integrity['registered_slots'], integrity['legal_slots'], integrity['invalid_slots']) == (950, 940, 10) and
            integrity['run_manifest_sha256'] == run_hashes and
            integrity['evidence']['run_index']['sha256'] == index_sha and
            integrity['evidence']['freeze']['sha256'] == freeze_sha,
            'Integrity audit is not bound to this complete study')
    require(lgcs.get('complete') is True and lgcs['index_sha256'] == index_sha and
            lgcs['freeze_sha256'] == freeze_sha and lgcs['new_source_commit'] == SOURCE and
            lgcs['expected_lgcs_jobs'] == 190 and lgcs['paired_dev']['n'] == lgcs['paired_dev']['legal_n'] == 180,
            'LGCS audit is not bound to this complete study')
    require(nto['index_sha256'] == index_sha and
            (nto['planned_new_slots'], nto['new_nto_slots'], nto['paired_dev_nto_slots']) == (950, 380, 360),
            'NTO audit is not bound to this complete study')
    for name, digest in frozen['runtime_source_hashes'].items():
        require(sha(ROOT / name) == digest, 'Frozen runtime source changed: ' + name)
    evidence = {name: dict(path=str((here / name).resolve()), sha256=sha(here / name)) for name in
        ('run_index.json', 'completed_blocks.json', 'execution_freeze.json',
         'integrity_audit.json', 'lgcs_audit.json', 'nto_audit.json')}
    return dict(index=index, run_hashes=run_hashes, audits=audits, evidence=evidence)


def check_metrics(metrics, barrier):
    require(metrics.get('complete') is True and metrics.get('comparable') is True and
            metrics['requested_jobs'] == len(metrics['rows']) == 950,
            'Metrics must contain all 950 complete comparable slots')
    require(not metrics.get('reporting_problem_ids') and not metrics.get('pairing_differences'),
            'Filtered or unpaired metrics cannot supply illustrations')
    runs = unique(metrics['runs'], lambda r: str(Path(r['run']).resolve()), 'metric run')
    require(runs.keys() == barrier['run_hashes'].keys(), 'Metrics refer to another run set')
    for run, row in runs.items():
        require(row['manifest_sha256'] == barrier['run_hashes'][run] and row['source_commit'] == SOURCE and
                row['integrity'].get('complete') is True and row['integrity'].get('integrity_passed') is True,
                'Metrics contain unsealed or mismatched run evidence')
    rows = metrics['rows']
    unique(rows, lambda r: (r['case_id'], r['roadmap_id'], r['method_seed'], r['label']), 'metric slot')
    require(Counter((r['split'], r['label']) for r in rows) ==
            Counter({(split, label): n for split, n in (('smoke', 10), ('dev', 180)) for label in LABELS}),
            'Wrong split or method coverage')
    require(sum(r['input_legal'] is True for r in rows) == 940, 'Wrong legal-input denominator')
    groups = defaultdict(list)
    for row in rows:
        require((row['method'], row['objective']) == METHODS[row['label']], 'Method/objective label mismatch')
        require(row['success'] is (row['status'] == SUCCESS), 'Final status and success flag disagree')
        require(row['roadmap_nodes'] == 6000, 'Wrong roadmap size')
        if row['split'] == 'dev':
            require(row['input_legal'] is True and row['family'] != 'open_control', 'Invalid dev input')
        run, attempt = Path(row['run']).resolve(), Path(row['attempt']).resolve()
        require(str(run) in runs and attempt.is_relative_to(run / 'jobs') and
                attempt.parent.name == 'attempts', 'Metric attempt lies outside its sealed run')
        groups[(row['case_id'], row['roadmap_id'], row['method_seed'])].append(row)
    for group in groups.values():
        require(len(group) == 5 and {r['label'] for r in group} == set(LABELS), 'Incomplete five-method group')
        for field in ('input_sha256', 'q_start', 'q_goal', 'robot_sha256', 'geometry_sha256',
                      'family', 'straight_line_collision', 'input_legal', 'roadmap_seed', 'split'):
            require(all(row[field] == group[0][field] for row in group), 'Paired input changed: ' + field)
    audited_counts = {(g['split'], g['method']): g for g in barrier['audits']['integrity']['groups'] if g['family'] == 'all'}
    for split in ('smoke', 'dev'):
        for label in LABELS:
            pool = [r for r in rows if (r['split'], r['label']) == (split, label)]
            audited = audited_counts[(split, label)]
            require(audited['registered'] == len(pool) and audited['success'] == sum(r['success'] for r in pool) and
                    audited['status_counts'] == dict(Counter(r['status'] for r in pool)),
                    'Metric outcomes disagree with independent raw-record counts')


def order(row):
    return (row['straight_line_collision'] is not True, row['case_id'], row['roadmap_seed'],
            row['roadmap_id'], row['method_seed'], row['label'], row['job_id'])


def select_illustrations(rows):
    """Pure deterministic selector; production caller must pass the audit gate first."""
    rows = [r for r in rows if r['split'] == 'dev']
    unique(rows, lambda r: (r['case_id'], r['roadmap_id'], r['method_seed'], r['label']), 'selection slot')
    successful = [r for r in rows if r['input_legal'] is True and r['success'] is True and r['status'] == SUCCESS]
    groups = defaultdict(list)
    for row in successful:
        groups[(row['case_id'], row['roadmap_id'], row['method_seed'])].append(row)
    common = [group for group in groups.values() if len(group) == 5 and {r['label'] for r in group} == set(LABELS)]
    selected = min(common, key=lambda group: order(next(r for r in group if r['label'] == 'LSCS'))) if common else []
    if not common:
        selected = [min(pool, key=order) for label in LABELS
                    if (pool := [r for r in successful if r['label'] == label])]
    choices = [dict(name=r['label'], role='validated_final_trajectory', row=r)
               for r in sorted(selected, key=lambda r: LABELS.index(r['label']))]
    missing = [dict(label=label, reason='No validated successful final trajectory in dev')
               for label in LABELS if label not in {r['label'] for r in selected}]
    failures = [r for r in rows if r['input_legal'] is True and r['success'] is False and r['status'] != SUCCESS
                and r['drm_success'] is True and r['seed_cost_validated'] is True]
    if failures:
        choices.append(dict(name='failure_seed_only', role='verified_frontend_seed_only', row=min(failures, key=order)))
    else:
        missing.append(dict(label='failure_seed_only', reason='No legal failed dev attempt with a validated saved frontend seed'))
    return choices, dict(rule=RULE, shared_case_for_five_methods=bool(common), missing=missing)


def source_snapshot(attempt):
    """Include unlisted and added files as well as the receipt to catch any write."""
    return {str(p.relative_to(attempt)): sha(p) for p in sorted(attempt.rglob('*')) if p.is_file()}


def gallery(record, output):
    cards = []
    for item in record['illustrations']:
        note = ('SEED ONLY — failed attempt; not an optimized final solution' if
                item['role'] == 'verified_frontend_seed_only' else 'Saved validated final trajectory')
        detail = html.escape(f"{item['case_id']} · roadmap {item['roadmap_seed']} · {item['source_status']}")
        content = '<h2>' + html.escape(item['name']) + '</h2><p>' + html.escape(note) + '</p><p>' + detail + '</p>'
        content += '<p>Input straight-line collision: ' + str(item['straight_line_collision']) + '</p>'
        if item.get('status') == 'created':
            content += '<a href="' + item['name'] + '/replay.html">Open interactive replay</a><br>'
            content += '<img src="' + item['name'] + '/scene.png" alt="Saved collision-model scene">'
        else:
            content += '<p>Replay generation failed; see index.json and the retained failure log.</p>'
        cards.append('<article>' + content + '</article>')
    missing = ''.join('<li>' + html.escape(v['label'] + ': ' + v['reason']) + '</li>' for v in record['selection']['missing'])
    page = ('<!doctype html><html lang="en"><meta charset="utf-8"><title>Development baseline saved replays</title>'
        '<style>body{font-family:system-ui;max-width:1050px;margin:30px auto;padding:20px}article{border-top:1px solid #ccd;padding:20px 0}img{max-width:720px;width:100%}p{line-height:1.5}</style>'
        '<h1>Kinova development baseline: saved evidence</h1><p>' + html.escape(RULE) + '</p>'
        '<p>Gallery complete: ' + str(record['complete']) + '. Finite numerical collision checks are not a continuous safety proof.</p>'
        + ('<h2>Unavailable views</h2><ul>' + missing + '</ul>' if missing else '') + ''.join(cards))
    (output / 'index.html').write_text(page)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--planning-finished', action='store_true')
    parser.add_argument('--metrics', type=Path, default=HERE / 'metrics/metrics.json')
    parser.add_argument('--output', type=Path, default=HERE / 'development_replays_v1')
    args = parser.parse_args(argv)
    barrier = completion_barrier(HERE, args.planning_finished)
    metrics = read(args.metrics)  # Deliberately after the complete independent-audit gate.
    check_metrics(metrics, barrier)
    choices, selection = select_illustrations(metrics['rows'])
    output = args.output.resolve()
    require(output.is_relative_to(HERE) and output != HERE, 'Output must be a new directory under baseline_repair_v2')
    require(not output.exists(), 'Refuse to overwrite any previous replay output')
    output.mkdir(parents=True, exist_ok=False)
    helper = ROOT / 'benchmarks/generic_motion_v1/reports/baseline_study/make_formal_replays.py'
    record = dict(complete=False, selection=selection, planning_complete_slots=950, source_commit=SOURCE,
        evidence=barrier['evidence'], metrics=dict(path=str(args.metrics.resolve()), sha256=sha(args.metrics)),
        command=shlex.join(sys.orig_argv), script_sha256=sha(__file__), reused_helpers_sha256=sha(helper),
        illustrations=[], failures=[])
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
                            'Selected run no longer has complete intact receipts')
                    verified[row['run']] = check
                receipt = check_selected(row)
                before = source_snapshot(attempt)
                item['source_receipt_sha256'] = sha(attempt / 'receipt.json')
                source = attempt
                if choice['role'] == 'verified_frontend_seed_only':
                    source = output / 'seed_source_projection'
                    item['projection'] = seed_projection(row, source, receipt)
                else:
                    require(read(source / 'validation.json').get('passed') is True, 'Final validation is missing')
                    require('trajectory_samples.npz' in receipt['files'], 'Final samples are not sealed')
                target = output / name
                result = replay(source, output=target)
                data = label_replay(target, row, choice['role'], item['source_receipt_sha256'])
                require(before == source_snapshot(attempt), 'Original sealed attempt changed during rendering')
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
                    record['failures'].append(dict(name=name, error='Original attempt changed during replay'))
            write(output / 'index.json', record)
    record['source_run_verification'] = verified
    record['complete'] = not record['failures'] and not selection['missing']
    gallery(record, output)
    record['gallery_sha256'] = sha(output / 'index.html')
    write(output / 'index.json', record)
    print(json.dumps(dict(complete=record['complete'], output=str(output),
        illustrations=len(record['illustrations']), missing=selection['missing'], failures=record['failures']), indent=2))
    if not record['complete']:
        raise SystemExit('Replay gallery incomplete; missing views and actual rendering failures are retained')


if __name__ == '__main__':
    main()
