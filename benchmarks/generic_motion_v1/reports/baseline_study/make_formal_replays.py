"""Post-completion illustrations from sealed formal trajectories, never planning.

Requires both --planning-finished and all 24 verified completion entries before
reading a metric row. Selection is deterministic and affects illustrations only.
No frozen runtime source, input, attempt or experimental result is modified.
"""
from __future__ import annotations

import argparse
from collections import defaultdict
from contextlib import redirect_stderr, redirect_stdout
import html
import json
from pathlib import Path
import shlex
import shutil
import sys
import traceback

from .audit_formal_study import completion_barrier, read, require, sha, write

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
LABELS = ('LSCS', 'LGCS', 'NTO-distance', 'SCSTO', 'NTO-time')
SUCCESS = 'success_validated_numerical'
SELECTION_RULE = (
    'Use complete formal main results only. Prefer the lexicographically first '
    'fixed case/roadmap/method-seed pair where all five labels succeeded, first '
    'preferring input straight_line_collision=True and then a non-open-control '
    'case. If no common-success pair exists, apply the same ordering separately '
    'within each label. Do not rank by cost, latency or visual appearance. '
    'Add explicit EI_ZO distance/time blocked-success views; reuse a selected '
    'blocked view when available, otherwise choose the first blocked success '
    'under the same ordering. Record absence honestly. Select one legal failed '
    'attempt with an independently validated saved DRM seed, preferring blocked '
    'obstacle cases, then collision_failed, solver_failed, unsupported, timeout, '
    'infrastructure_error, then lexicographic task order. Display its seed only. '
    'This selected gallery is not a performance sample or a success-rate estimate.'
)


def task_order(row):
    return (row['case_id'], row['roadmap_seed'], row['roadmap_id'],
            row['method_seed'], row['label'], row['job_id'])


def illustration_order(row):
    return (row['straight_line_collision'] is not True,
            row['family'] == 'open_control', *task_order(row))


def select_illustrations(rows):
    """Pure selection; called only after the completion barrier in main()."""
    success = [r for r in rows if r['input_legal'] is True and r['success'] is True
               and r['status'] == SUCCESS]
    pairs = defaultdict(list)
    for row in success:
        pairs[(row['case_id'], row['roadmap_id'], row['method_seed'])].append(row)
    common = [group for group in pairs.values()
              if len(group) == len(LABELS) and {r['label'] for r in group} == set(LABELS)]
    selected = (min(common, key=lambda group: illustration_order(
        next(r for r in group if r['label'] == 'LSCS'))) if common else [])
    if not selected:
        for label in LABELS:
            pool = [r for r in success if r['label'] == label]
            if pool:
                selected.append(min(pool, key=illustration_order))
    choices = [dict(name=row['label'], role='validated_final_trajectory', row=row)
               for row in sorted(selected, key=lambda r: LABELS.index(r['label']))]
    missing = [dict(label=label, reason='No strictly validated successful final trajectory')
               for label in LABELS if label not in {r['label'] for r in selected}]
    for label, name in (('LSCS', 'EI_ZO_distance_blocked'), ('SCSTO', 'EI_ZO_time_blocked')):
        already = next((r for r in selected if r['label'] == label
                        and r['straight_line_collision'] is True), None)
        pool = [r for r in success if r['label'] == label and r['straight_line_collision'] is True]
        if already:
            choices.append(dict(name=name, role='validated_final_trajectory', row=already, reuse_of=label))
        elif pool:
            choices.append(dict(name=name, role='validated_final_trajectory',
                                row=min(pool, key=illustration_order)))
        else:
            missing.append(dict(label=name, reason='No blocked successful final trajectory; no unblocked substitute'))
    failures = [r for r in rows if r['input_legal'] is True and not r['success']
                and r['status'] != SUCCESS and r['drm_success'] is True
                and r['seed_cost_validated'] is True]
    if failures:
        priority = {name: i for i, name in enumerate(('collision_failed', 'solver_failed',
            'unsupported', 'timeout', 'infrastructure_error'))}
        row = min(failures, key=lambda r: (r['straight_line_collision'] is not True,
            r['family'] == 'open_control', priority.get(r['status'], 99), *task_order(r)))
        choices.append(dict(name='failure_seed_only', role='verified_frontend_seed_only', row=row))
    else:
        missing.append(dict(label='failure_seed_only', reason='No legal failed attempt with a verified saved DRM seed'))
    return choices, dict(rule=SELECTION_RULE, shared_case_for_five_methods=bool(common), missing=missing)


def check_main_metrics(metrics, index, protocol):
    expected = {Path(v['run']).resolve(): v for v in index['blocks'] if v['phase'] == 'main'}
    require(len(expected) == 15, 'Expected exactly 15 main blocks')
    require(metrics.get('complete') is True and metrics.get('comparable') is True,
            'Only a complete comparable main report may supply illustrations')
    require(metrics.get('requested_jobs') == len(metrics['rows']) == 6300,
            'Main metrics must include all 6300 slots, not a partial or selected subset')
    require(not metrics.get('reporting_problem_ids') and not metrics.get('pairing_differences'),
            'Selection requires the full main matrix and a shared frontend')
    recorded = {Path(item['run']).resolve(): item for item in metrics['runs']}
    require(len(recorded) == len(metrics['runs']) == len(expected) and recorded.keys() == expected.keys(),
            'Main report covers different or duplicate runs')
    for run, item in recorded.items():
        require(item['manifest_sha256'] == expected[run]['run_manifest_sha256']
                == sha(run/'run_manifest.json'), 'Main run manifest binding changed')
        require(item['source_commit'] == protocol['source_code_commit'], 'Main report runtime source differs')
        check = item['integrity']
        require(check.get('complete') is True and check.get('integrity_passed') is True,
                'Main report includes unverified evidence')
    require({r['label'] for r in metrics['rows']} == set(LABELS), 'Main report method labels differ')
    require(all(r['roadmap_nodes'] == 6000 and r['split'] == 'test' for r in metrics['rows']),
            'Illustrations must come from the formal main block, not dev or scale runs')
    for row in metrics['rows']:
        run, attempt = Path(row['run']).resolve(), Path(row['attempt']).resolve()
        require(run in expected and attempt.is_relative_to(run/'jobs')
                and attempt.parent.name == 'attempts', 'Metric attempt is outside its sealed main run')
    return expected


def check_selected(row):
    """Bind each selected final/seed source to its sealed receipt, never fallback."""
    attempt = Path(row['attempt'])
    receipt, result = read(attempt/'receipt.json'), read(attempt/'result.json')
    require(not receipt.get('retry_pending'), 'Cannot illustrate a pending retry')
    require(receipt['status'] == result['status'] == row['status'], 'Selected status changed')
    require(result['method'] == row['method'] and result['objective'] == row['objective'],
            'Selected method/objective changed')
    for name, digest in receipt['files'].items():
        path = (attempt/name).resolve()
        require(path.is_relative_to(attempt.resolve()) and sha(path) == digest,
                'Selected receipt artifact changed: ' + name)
    request = read(attempt/'request.json')
    require(all(request[key] == row[key] for key in
                ('job_id', 'case_id', 'roadmap_id', 'method_seed', 'method', 'objective')),
            'Selected metric identity differs from its sealed request')
    case = request['case']
    require(case['problem_id'] == row['case_id'] and case['validation_status'] == 'valid_input'
            and case['family'] == row['family']
            and case['straight_line_collision'] is row['straight_line_collision'],
            'Selected legal/blocked/family metadata differs from the sealed input')
    return receipt


def seed_projection(row, destination, receipt):
    """A report-only, byte-identical projection of verified saved frontend data."""
    attempt = Path(row['attempt'])
    validation = read(attempt/'frontend_validation/validation.json')
    require(validation.get('passed') is True, 'Failure illustration needs a verified frontend seed')
    mapping = {'input.json': 'input.json', 'result.json': 'result.json', 'process.json': 'process.json',
               'trajectory_samples.npz': 'frontend_validation/trajectory_samples.npz',
               'validation.json': 'frontend_validation/validation.json'}
    destination.mkdir(parents=True, exist_ok=False)
    copied = {}
    for target, original in mapping.items():
        require(original in receipt['files'], 'Projection file was not sealed: ' + original)
        source = attempt/original
        shutil.copyfile(source, destination/target)
        digest = sha(source)
        require(digest == receipt['files'][original] == sha(destination/target), 'Projection copy differs')
        copied[target] = dict(source=str(source), sha256=digest)
    provenance = dict(role='verified_frontend_seed_only', source_attempt=str(attempt),
        source_status=row['status'], source_receipt_sha256=sha(attempt/'receipt.json'),
        files=copied, note='Report-only copies. The failed optimizer produced no successful final result. '
        'Displayed points are the saved independently validated DRM seed, not a final solution or retry.')
    write(destination/'projection.json', provenance)
    return provenance


def label_replay(output, row, role, source_receipt_sha):
    # Only derived report files change. Reuse the existing renderer and its HTML;
    # make sample provenance visible instead of relying on a gallery caption.
    from ...runners import replay as renderer
    data = read(output/'replay.json')
    data.update(display_role=role, original_attempt=row['attempt'],
                original_status=row['status'], source_receipt_sha256=source_receipt_sha)
    if role == 'verified_frontend_seed_only':
        note = ('SEED ONLY — failed ' + row['label'] + ' attempt (' + row['status'] + '). '
            'This is the independently validated saved DRM seed. It is not a successful final '
            'trajectory, an optimizer candidate, a retry, or a minimum-time solution.')
        data.update(sample_source_note=note,
            original_sample_source=str(Path(row['attempt'])/'frontend_validation/trajectory_samples.npz'),
            evidence_integrity=dict(verified=True, scope='Byte-identical report projection of sealed '
                'frontend seed files; original selected run verified before rendering', run=row['run']))
    else:
        require(data.get('sample_source') == 'trajectory_samples.npz',
                'Successful replay must use the saved final samples, not a recursive fallback')
        note = ('FINAL VALIDATED TRAJECTORY — ' + row['label'] + '. Saved numerical samples; '
                'finite sampling is not a continuous collision-free certificate.')
    data['input_straight_line_collision'] = row['straight_line_collision']
    data['illustration_note'] = note
    write(output/'replay.json', data)
    serialized = json.dumps(data, separators=(',', ':'), allow_nan=False).replace('<', r'\u003c')
    page = renderer._HTML.replace('__DATA__', serialized).replace('__EDGES__', json.dumps(renderer.BOX_EDGES))
    marker = '<h1>Kinova 7-DoF · saved experiment replay</h1>'
    page = page.replace(marker, marker + '<p class="card status">' + html.escape(note) + '</p>')
    (output/'replay.html').write_text(page)
    return data


def gallery(index, destination):
    cards = []
    for item in index['illustrations']:
        if item.get('status') != 'created':
            continue
        target = item.get('reuse_of', item['name'])
        note = 'SEED ONLY — failed attempt; not an optimized final solution' if item['role'] == 'verified_frontend_seed_only' else 'Saved strictly validated final trajectory'
        cards.append('<article><h2>'+html.escape(item['name'])+'</h2><p>'+html.escape(note)+
            '</p><p>'+html.escape(item['case_id']+' · '+item['source_status'])+
            ' · input straight-line collision: '+str(item['straight_line_collision'])+'</p>'+
            '<a href="'+html.escape(target)+'/replay.html">Open interactive replay</a><br>'+
            '<img src="'+html.escape(target)+'/scene.png" alt="Saved collision-model scene"></article>')
    content = '<!doctype html><html lang="en"><meta charset="utf-8"><title>Formal Kinova saved replays</title><style>body{font-family:system-ui;max-width:1050px;margin:30px auto;padding:20px}article{border-top:1px solid #ccd;padding:20px 0}img{max-width:720px;width:100%}p{line-height:1.5}</style><h1>Formal Kinova benchmark: saved results</h1><p>'+html.escape(SELECTION_RULE)+'</p>'+''.join(cards)
    (destination/'index.html').write_text(content)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--planning-finished', action='store_true')
    parser.add_argument('--main-metrics', type=Path, required=True)
    parser.add_argument('--output', type=Path, default=HERE/'formal_replays_v1')
    args = parser.parse_args()
    index, _ = completion_barrier(HERE/'formal_run_index.json', HERE/'formal_completed_blocks.json',
                                 args.planning_finished)
    protocol_path = HERE/'formal_protocol_v1.json'
    protocol = read(protocol_path)
    require(index['protocol_sha256'] == sha(protocol_path), 'Frozen protocol binding differs')
    for name, digest in protocol['runtime_source_hashes'].items():
        require(sha(ROOT/name) == digest, 'Frozen runtime source changed: ' + name)
    metrics = read(args.main_metrics)
    check_main_metrics(metrics, index, protocol)
    choices, selection = select_illustrations(metrics['rows'])
    output = args.output.resolve()
    require(output.is_relative_to(HERE.resolve()) and output != HERE.resolve(),
            'Replay output must be a new directory under reports/baseline_study')
    require(not output.exists(), 'Refuse to overwrite a previous replay attempt; choose another output directory')
    output.mkdir(parents=True)
    record = dict(complete=False, selection=selection, planning_complete_blocks=24, planning_complete_slots=9900,
        main_metrics=dict(path=str(args.main_metrics.resolve()), sha256=sha(args.main_metrics)),
        protocol_sha256=sha(protocol_path), command=shlex.join(sys.orig_argv),
        script_sha256=sha(__file__), illustrations=[], failures=[])
    write(output/'index.json', record)
    from ...batch import verify
    from ...runners.replay import replay
    verified_runs = {}
    for choice in choices:
        row, name = choice['row'], choice['name']
        item = dict(name=name, role=choice['role'], case_id=row['case_id'], job_id=row['job_id'],
            label=row['label'], source_status=row['status'], source_attempt=row['attempt'],
            roadmap_seed=row['roadmap_seed'], straight_line_collision=row['straight_line_collision'])
        record['illustrations'].append(item)
        try:
            if choice.get('reuse_of'):
                original = next(v for v in record['illustrations'] if v['name'] == choice['reuse_of'])
                require(original.get('status') == 'created', 'Cannot reuse a failed replay')
                item.update(status='created', reuse_of=choice['reuse_of'], output=original['output'])
                continue
            with (output/(name+'.log')).open('w') as log, redirect_stdout(log), redirect_stderr(log):
                if row['run'] not in verified_runs:
                    check = verify(row['run'])
                    require(check.get('integrity_passed') is True and check.get('complete') is True,
                            'Selected run no longer has complete intact receipts')
                    verified_runs[row['run']] = check
                receipt = check_selected(row)
                source = Path(row['attempt'])
                if choice['role'] == 'verified_frontend_seed_only':
                    source = output/'seed_source_projection'
                    item['projection'] = seed_projection(row, source, receipt)
                else:
                    require(read(source/'validation.json').get('passed') is True,
                            'Successful illustration lacks final validation')
                    require('trajectory_samples.npz' in receipt['files'], 'Final samples are not sealed')
                target = output/name
                result = replay(source, output=target)
                data = label_replay(target, row, choice['role'], sha(Path(row['attempt'])/'receipt.json'))
                item.update(status='created', output=str(target), replay=result,
                    sample_source_sha256=data['sample_source_sha256'],
                    files={file.name: sha(file) for file in sorted(target.iterdir()) if file.is_file()})
                print(json.dumps(item, indent=2))
        except Exception:
            error = traceback.format_exc()
            item.update(status='failed', error=error)
            record['failures'].append(dict(name=name, error=error))
            (output/(name+'.failure.log')).write_text(error)
        finally:
            write(output/'index.json', record)
    record['source_run_verification'] = verified_runs
    record['complete'] = not record['failures'] and not selection['missing']
    write(output/'index.json', record)
    gallery(record, output)
    print(json.dumps(dict(complete=record['complete'], output=str(output),
        illustrations=len(record['illustrations']), missing=selection['missing'], failures=record['failures']), indent=2))
    if not record['complete']:
        raise SystemExit('Replay gallery incomplete; actual missing results/render errors retained in index.json')


if __name__ == '__main__':
    main()
