"""Post-planning integrity/independent audit, then separately seal metrics evidence.

Run only AFTER all 24 planning blocks finish. This module never launches a
planner, optimizer, GPU query, or runtime numerical validator. The explicit
--planning-finished switch and complete coordinator ledger are both required.
Sequence: audit -> study_metrics (main and scaling) -> seal-bundle -> renderer.
No existing evidence is overwritten; a failed audit stays a failed audit.
"""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
from pathlib import Path
import re
import subprocess
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[4]
LABELS = {('EI_ZO', 'minimum_distance'): 'LSCS', ('LGCS', 'minimum_distance'): 'LGCS',
          ('NTO_PAPER', 'minimum_distance'): 'NTO-distance',
          ('EI_ZO', 'minimum_time'): 'SCSTO', ('NTO_PAPER', 'minimum_time'): 'NTO-time'}
REVIEWED_METHODS = ['LSCS', 'LGCS', 'NTO-distance', 'SCSTO', 'NTO-time']
TOTALS = dict(blocks=24, main_blocks=15, size_blocks=9, jobs=9900, legal=9300,
              invalid=600, main_jobs=6300, size_jobs=3600, main_cases=420,
              legal_cases=395, subset_cases=80, subset_legal=75)
AUDITORS = {
    'scs': 'benchmarks.generic_motion_v1.reports.baseline_study.formal_scs_audit',
    'nto': 'benchmarks.generic_motion_v1.reports.full_baseline.independent_nto_audit',
    'lgcs': 'benchmarks.generic_motion_v1.reports.full_baseline.independent_lgcs_audit',
}


def read(path):
    return json.loads(Path(path).read_text(), parse_constant=lambda v: (_ for _ in ()).throw(ValueError(v)))


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def write(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_suffix(path.suffix + '.tmp')
    temp.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + '\n')
    temp.replace(path)


def require(value, message):
    if not value:
        raise ValueError(message)


def stamp():
    return datetime.now(timezone.utc).isoformat()


def ref(path):
    path = Path(path).resolve()
    return dict(path=str(path), sha256=sha(path))


def check_ref(value, description):
    require(isinstance(value, dict) and isinstance(value.get('path'), str), f'{description}: missing file reference')
    path = Path(value['path'])
    require(path.is_absolute() and path.is_file(), f'{description}: missing absolute file path')
    require(re.fullmatch(r'[0-9a-f]{64}', value.get('sha256', '')) is not None,
            f'{description}: invalid SHA256')
    require(sha(path) == value['sha256'], f'{description}: file SHA256 changed')
    return path


def check_digest(value, length, description):
    require(isinstance(value, str) and re.fullmatch('[0-9a-f]{' + str(length) + '}', value) is not None,
            f'{description}: full lowercase hexadecimal digest required')


def completion_barrier(index_path, completed_path, planning_finished):
    """Reads small coordinator metadata only, before any run manifest/receipt."""
    require(planning_finished is True, 'Explicit --planning-finished confirmation required')
    index, done = read(index_path), read(completed_path)
    require(index.get('matrix_prepared') is True, 'Matrix was not fully prepared')
    expected = {v['block']: v for v in index['blocks']}
    require(len(expected) == len(index['blocks']) == TOTALS['blocks'], 'Missing or duplicate indexed blocks')
    require(done.get('complete') is True, 'Planning is still running or its completion ledger is absent/incomplete')
    observed = {v['block']: v for v in done['completed']}
    require(len(observed) == len(done['completed']) and observed.keys() == expected.keys(), 'Completion block coverage differs')
    require(done.get('completed_jobs') == done.get('total_jobs') == TOTALS['jobs'], 'Completion slot count differs')
    for name, item in expected.items():
        completed = observed[name]
        require(all(completed.get(k) == item.get(k) for k in ('phase', 'run', 'jobs', 'run_manifest_sha256')),
                f'{name}: completion entry differs from frozen index')
        check = completed.get('verification', {})
        require(check.get('integrity_passed') is True and check.get('complete') is True
                and check.get('completed_jobs') == check.get('planned_jobs') == item['jobs']
                and not check.get('errors') and not check.get('unsealed_attempts'),
                f'{name}: coordinator has not verified all terminal receipts')
    return index, done


def declared_matrix(protocol):
    """Re-derive the prespecified universe from inputs, never solver outcomes."""
    manifest_path = check_ref(protocol['manifest'], 'input manifest')
    cases = [json.loads(line) for line in manifest_path.read_text().splitlines() if line.strip()]
    by_id = {c['problem_id']: c for c in cases}
    require(len(by_id) == len(cases) == TOTALS['main_cases'], 'Input ID count/uniqueness differs')
    require(sum(c['validation_status'] == 'valid_input' for c in cases) == TOTALS['legal_cases'], 'Legal input count differs')
    subset = [c['problem_id'] for c in cases if c['family'] != 'open_control'
              and int(c['problem_id'].rsplit('-p', 1)[1]) == int(c['scene_id'].rsplit('-', 1)[1]) % 5]
    require(len(subset) == len(set(subset)) == TOTALS['subset_cases'], 'Prespecified 80-scene subset differs')
    require(sum(by_id[c]['validation_status'] == 'valid_input' for c in subset) == TOTALS['subset_legal'], 'Subset legality differs')
    require(read(check_ref(protocol['size_selection']['problem_ids_file'], 'size subset')) == subset,
            'Stored size subset differs from the input-only selection rule')
    main = {(c, 6000, s, m, o) for c in by_id for s in range(3) for m, o in LABELS}
    size = {(c, n, s, m, o) for c in subset for n in (3000, 9000, 12000)
            for s in range(3) for m, o in LABELS}
    require(not main & size, 'Main/scaling keys overlap')
    require(len(main) == TOTALS['main_jobs'] and len(size) == TOTALS['size_jobs'], 'Declared matrix size differs')
    return by_id, subset, {'main': main, 'size_effect': size}


def verify_run(run):
    # Artifact verifier only; no runtime trajectory validator or planner imports.
    from ...batch import verify
    return verify(run, current_sources=False)


def inspect_block(item, block, protocol, protocol_sha, source, by_id, seen):
    require(Path(item['run']).is_absolute(), 'Run path must be absolute')
    run = Path(item['run']).resolve()
    manifest_path = run / 'run_manifest.json'
    require(sha(manifest_path) == item['run_manifest_sha256'], f'{item["block"]}: run manifest changed')
    manifest = read(manifest_path)
    require(manifest.get('source_code_commit') == source, 'Unexpected runtime source commit')
    require(manifest.get('source_hashes') == protocol['runtime_source_hashes'], 'Runtime source inventory differs from preregistration')
    require(manifest.get('manifest_sha256') == protocol['manifest']['sha256'], 'Input manifest binding differs')
    require(manifest.get('formal_test_run') is True and manifest.get('split') == 'test', 'Run is not an approved formal test block')
    binding = manifest.get('study_protocol') or {}
    require(binding.get('sha256') == protocol_sha and binding.get('block') == item['block'], 'Wrong study protocol or block binding')
    require(manifest.get('timeout_seconds') == block['timeout_seconds'] == 60., 'Online budget differs')
    require(len(block['roadmap_dirs']) == len(block['roadmap_seeds']) == len(block['requested_node_budgets']) == 1,
            'Each formal block must use exactly one roadmap')
    require(item['phase'] == block['phase'], 'Block phase differs')
    require(set(map(tuple, block['method_pairs'])) == set(LABELS), 'Declared five method combinations differ')
    jobs = manifest['jobs']
    require(len(jobs) == len({j['job_id'] for j in jobs}) == item['jobs'] == block['planned_jobs'], 'Job count or uniqueness differs')
    require(set(j['case_id'] for j in jobs) == set(block['problem_ids']), 'Frozen block case selection differs')
    roadmap = Path(block['roadmap_dirs'][0]).resolve()
    metadata = read(roadmap / 'metadata.json')
    nodes, seed = block['requested_node_budgets'][0], block['roadmap_seeds'][0]
    require(metadata['requested_nodes'] == nodes and metadata['seed'] == seed and metadata['status'] == 'ready',
            'Roadmap requested-node budget/seed/readiness differs')
    require(manifest['roadmap_files'] == block['roadmap_files'], 'Roadmap artifact inventory differs')
    verification = verify_run(run)
    require(verification.get('integrity_passed') is True and verification.get('complete') is True
            and verification.get('planned_jobs') == verification.get('completed_jobs') == len(jobs)
            and not verification.get('errors') and not verification.get('unsealed_attempts'),
            f'{item["block"]}: receipt verification failed: {verification}')
    methods = {label: dict(registered=0, legal=0, invalid=0, statuses=Counter(), legal_statuses=Counter(), invalid_statuses=Counter())
               for label in REVIEWED_METHODS}
    job_statuses = {}
    phase_keys = set()
    for job in jobs:
        method = (job['method'], job['objective'])
        require(method in LABELS, 'Unexpected method/objective')
        require(job['case_id'] in by_id and job['case'] == by_id[job['case_id']], 'Case differs from frozen input record')
        require(job['evaluation_config'] == block['evaluation_config'], 'Per-job evaluation configuration differs')
        require(Path(job['roadmap_dir']).resolve() == roadmap and job['roadmap_id'] == metadata['roadmap_id'], 'Per-job roadmap differs')
        pairing = [p for p in job['case']['paired_seeds'] if p['drm_seed'] == seed]
        require(len(pairing) == 1 and job['method_seed'] == pairing[0]['method_seed'], 'Method seed is not the prespecified pairing')
        key = (job['case_id'], nodes, seed, *method)
        require(key not in seen, 'Duplicate global job key: ' + str(key))
        seen.add(key); phase_keys.add(key)
        attempts = sorted((run / 'jobs' / job['job_id'] / 'attempts').glob('[0-9][0-9][0-9][0-9]'))
        latest = attempts[-1]
        receipt, result = read(latest / 'receipt.json'), read(latest / 'result.json')
        require(receipt.get('retry_pending') is not True and receipt['status'] == result['status'], 'Latest attempt is not terminal')
        legal = job['case']['validation_status'] == 'valid_input'
        if not legal:
            require(result['status'] == 'invalid_endpoint' and result.get('success') is not True,
                    'Invalid input was upgraded to a planning outcome')
            require(not (latest / 'path.npy').exists() and not (latest / 'candidate.npz').exists()
                    and not (latest / 'regions.npz').exists(), 'Invalid input reached planning artifacts')
        row = methods[LABELS[method]]
        row['registered'] += 1; row['legal' if legal else 'invalid'] += 1
        row['statuses'][result['status']] += 1
        row['legal_statuses' if legal else 'invalid_statuses'][result['status']] += 1
        job_statuses[job['job_id']] = dict(label=LABELS[method], status=result['status'],
            objective=job['objective'], method=job['method'], case_id=job['case_id'],
            method_seed=job['method_seed'], roadmap_nodes=nodes, roadmap_seed=seed, input_legal=legal)
    expected = {(c, nodes, seed, m, o) for c in block['problem_ids'] for m, o in LABELS}
    require(phase_keys == expected, 'Block method/case matrix is incomplete')
    return dict(block=item['block'], phase=item['phase'], run=str(run), jobs=len(jobs),
                run_manifest_sha256=item['run_manifest_sha256'], requested_nodes=nodes,
                actual_nodes=metadata['actual_nodes'], roadmap_seed=seed, verification=verification,
                methods=methods, legal_jobs=sum(v['legal'] for v in methods.values()),
                invalid_jobs=sum(v['invalid'] for v in methods.values()), independent={},
                job_evidence=job_statuses), phase_keys, job_statuses


def invoke_auditor(kind, run, out, source):
    out.mkdir(parents=True, exist_ok=False)
    report_path = out / 'audit.json'
    module = AUDITORS[kind]
    script = Path(importlib.util.find_spec(module).origin)
    argv = [sys.executable, '-m', module, '--run', str(run), '--output', str(out if kind == 'lgcs' else report_path)]
    if kind == 'scs':
        argv += ['--expected-source', source]
    record = dict(argv=argv, cwd=str(ROOT), started_utc=stamp(), audit_script=ref(script))
    write(out / 'invocation.json', record)
    start = time.monotonic()
    with (out / 'stdout.log').open('w') as stdout, (out / 'stderr.log').open('w') as stderr:
        process = subprocess.run(argv, cwd=ROOT, stdout=stdout, stderr=stderr, check=False)
    record.update(returncode=process.returncode, finished_utc=stamp(), elapsed_seconds=time.monotonic()-start,
                  stdout=ref(out / 'stdout.log'), stderr=ref(out / 'stderr.log'))
    write(out / 'invocation.json', record)
    return record, report_path


def inspect_independent(kind, report, item, jobs, source):
    require(report.get('passed') is True, f'{kind}: independent audit failed')
    require(Path(report.get('run', '')).resolve() == Path(item['run']).resolve(), f'{kind}: wrong run')
    require(report.get('run_manifest_sha256') == item['run_manifest_sha256'], f'{kind}: wrong run manifest SHA')
    source_field = 'source_commit' if kind == 'lgcs' else 'frozen_planning_commit'
    require(report.get(source_field) == source, f'{kind}: wrong planning source')
    labels = {'scs': {'LSCS', 'SCSTO'}, 'nto': {'NTO-distance', 'NTO-time'}, 'lgcs': {'LGCS'}}[kind]
    expected = {jid: row for jid, row in jobs.items() if row['label'] in labels}
    records = report.get('records', [])
    latest = records if kind == 'nto' else [r for r in records if r.get('latest') is True]
    require(len(latest) == len(expected) and len({r['job_id'] for r in latest}) == len(latest)
            and {r['job_id'] for r in latest} == set(expected), f'{kind}: incomplete latest-job coverage')
    for row in latest:
        require(row['status'] == expected[row['job_id']]['status'], f'{kind}: failure/success classification differs')
    if kind == 'scs':
        require(report.get('expected_source_commit') == source and report.get('complete') is True
                and report.get('planned_jobs') == report.get('completed_latest_jobs') == item['jobs']
                and report.get('receipts_verified', 0) >= item['jobs']
                and report.get('scs_latest_jobs') == len(expected), 'scs: incomplete receipts or expected source binding')
    elif kind == 'nto':
        require(report.get('planned_jobs') == report.get('receipts_verified') == item['jobs']
                and report.get('nto_jobs') == len(expected), 'nto: incomplete receipts')
    else:
        require(report.get('planned_lgcs_jobs') == len(expected), 'lgcs: incomplete planned coverage')
    return dict(reviewed_methods=sorted(labels), latest_jobs=len(latest),
                latest_status_counts=dict(Counter(r['status'] for r in latest)))


def execute_audit(index_path, completed_path, protocol_path, output, source, protocol_sha, planning_finished=False):
    output = Path(output).resolve()
    require(not output.exists(), 'Refusing to overwrite any prior audit directory')
    # No output under a sealed run or its jobs; use the report directory.
    require('results' not in output.parts, 'Audit output must be in reports, never sealed results')
    output.mkdir(parents=True)
    report = dict(schema_version=1, kind='formal_baseline_integrity_and_independent_audit',
                  started_utc=stamp(), complete=False, passed=False, integrity_passed=False,
                  independent_passed=False, all_outcomes_reviewed=False, reviewed_methods=[],
                  covered_job_slots=0, source_code_commit=source, protocol_sha256=protocol_sha,
                  run_manifest_sha256={}, blocks=[], errors=[], audit_driver=ref(__file__),
                  limitations=['Independent auditors check saved arrays and collision labels; no new collision queries.',
                      'Finite numerical collision samples are not a mathematical continuous collision proof.',
                      'Audit pass means evidence and algebra passed, not that every planner job succeeded.',
                      'Kinova adaptation and declared NTO/LGCS reconstructions; not original FR3/hardware reproduction.'])
    target = output / 'integrity_audit.json'
    write(target, report)
    try:
        check_digest(source, 40, 'Expected planning source')
        check_digest(protocol_sha, 64, 'Expected protocol SHA')
        index, _ = completion_barrier(index_path, completed_path, planning_finished)
        report.update(run_index=ref(index_path), run_index_sha256=sha(index_path), completion_ledger=ref(completed_path))
        require(sha(protocol_path) == protocol_sha == index['protocol_sha256'], 'Protocol SHA binding differs')
        protocol = read(protocol_path)
        require(protocol.get('state') == 'approved' and protocol.get('authorization', {}).get('text', '').strip(), 'Missing approved authorization')
        require(protocol.get('source_code_commit') == source and protocol.get('runtime_source_hashes'), 'Protocol runtime source differs')
        require(set(protocol['blocks']) == {i['block'] for i in index['blocks']}, 'Protocol/index block sets differ')
        require((index['registered_job_slots'], index['legal_planner_jobs'], index['input_invalid_slots'])
                == (TOTALS['jobs'], TOTALS['legal'], TOTALS['invalid']), 'Indexed slot accounting differs')
        input_path = check_ref(protocol['input_audit'], 'independent input audit')
        input_report = read(input_path)
        require(input_report.get('passed') is True and not input_report.get('errors')
                and input_report.get('manifest_sha256') == protocol['manifest']['sha256'],
                'Input audit did not pass for this manifest')
        report['input_audit'] = ref(input_path)
        report['protocol'] = ref(protocol_path)
        report['metrics_contract'] = ref(check_ref(protocol['metrics_contract'], 'metrics contract'))
        by_id, subset, expected = declared_matrix(protocol)
        report['prespecified_size_problem_ids'] = subset
        report['manifest_sha256'] = protocol['manifest']['sha256']
        seen, phase_keys, job_tables = set(), {'main': set(), 'size_effect': set()}, {}
        # Verify EVERY block before starting even the first independent auditor.
        for item in index['blocks']:
            block, keys, jobs = inspect_block(item, protocol['blocks'][item['block']], protocol, protocol_sha, source, by_id, seen)
            report['blocks'].append(block)
            report['run_manifest_sha256'][block['run']] = block['run_manifest_sha256']
            phase_keys[block['phase']].update(keys)
            job_tables[block['block']] = jobs
            report['covered_job_slots'] += block['jobs']
            write(target, report)
        require(phase_keys == expected and len(seen) == TOTALS['jobs'], 'Global 9900-key matrix has missing/extra/duplicate observations')
        require(sum(b['legal_jobs'] for b in report['blocks']) == TOTALS['legal']
                and sum(b['invalid_jobs'] for b in report['blocks']) == TOTALS['invalid'], 'Legal/invalid global accounting differs')
        require(Counter(b['phase'] for b in report['blocks']) == {'main': TOTALS['main_blocks'], 'size_effect': TOTALS['size_blocks']}, 'Phase block count differs')
        report.update(integrity_passed=True, legal_planner_jobs=TOTALS['legal'], input_invalid_slots=TOTALS['invalid'])
        write(target, report)
        for block in report['blocks']:
            for kind in ('scs', 'nto', 'lgcs'):
                invocation, path = invoke_auditor(kind, block['run'], output / 'blocks' / block['block'] / kind, source)
                entry = dict(passed=False, invocation=invocation)
                block['independent'][kind] = entry
                if path.is_file():
                    entry['report'] = ref(path)
                write(target, report)
                require(invocation['returncode'] == 0 and path.is_file(), f'{block["block"]}/{kind}: subprocess audit failed; see saved logs')
                independent = read(path)
                check_ref(invocation['audit_script'], f'{block["block"]}/{kind} auditor source')
                script_field = 'audit_source_sha256' if kind == 'lgcs' else 'audit_script_sha256'
                require(independent.get(script_field) == invocation['audit_script']['sha256'],
                        f'{block["block"]}/{kind}: auditor source changed or report provenance differs')
                entry.update(inspect_independent(kind, independent, block, job_tables[block['block']], source), passed=True)
                write(target, report)
        # Detect mid-audit changes of all top-level protocol/run identities.
        require(sha(index_path) == report['run_index_sha256'] and sha(protocol_path) == protocol_sha, 'Metadata changed during audit')
        for run, expected_sha in report['run_manifest_sha256'].items():
            require(sha(Path(run) / 'run_manifest.json') == expected_sha, 'Run manifest changed during audit')
        report.update(complete=True, passed=True, independent_passed=True, all_outcomes_reviewed=True,
                      reviewed_methods=REVIEWED_METHODS, finished_utc=stamp())
    except Exception as error:
        report['errors'].append(dict(error=repr(error), traceback=traceback.format_exc()))
        report['finished_utc'] = stamp()
        write(target, report)
        raise
    write(target, report)
    return report


def inspect_metrics(metrics, expected_runs, expected_jobs, subset, description):
    require(metrics.get('complete') is True and metrics.get('comparable') is True and not metrics.get('pairing_differences'),
            f'{description}: incomplete or incomparable metrics')
    require(set(metrics.get('expected_method_labels', [])) == set(REVIEWED_METHODS), f'{description}: methods differ')
    require(metrics.get('requested_jobs') == expected_jobs == len(metrics.get('rows', [])), f'{description}: observation count differs')
    require(metrics.get('reporting_problem_ids') == subset, f'{description}: reporting selection differs')
    runs = metrics.get('runs', [])
    bindings = {str(Path(v['run']).resolve()): v['manifest_sha256'] for v in runs}
    require(len(bindings) == len(runs) and bindings == expected_runs, f'{description}: source run coverage differs')
    keys = [(r['case_id'], r['roadmap_nodes'], r['roadmap_seed'], r['method'], r['objective']) for r in metrics['rows']]
    require(len(keys) == len(set(keys)), f'{description}: duplicate rows')
    for row in metrics['rows']:
        require(row.get('label') == LABELS.get((row['method'], row['objective'])), f'{description}: incorrect method label')
    return set(keys)


def seal_bundle(integrity_path, main_path, scaling_path, output):
    """Bind completed independent audits and metrics; never re-run any planning."""
    output = Path(output).resolve()
    require(not output.exists(), 'Refusing to replace a published audit bundle')
    integrity = read(integrity_path)
    for field in ('complete', 'passed', 'integrity_passed', 'independent_passed', 'all_outcomes_reviewed'):
        require(integrity.get(field) is True, f'Integrity report is not publishable: {field}')
    require(not integrity.get('errors') and integrity.get('covered_job_slots') == TOTALS['jobs']
            and integrity.get('legal_planner_jobs') == TOTALS['legal']
            and integrity.get('input_invalid_slots') == TOTALS['invalid'], 'Integrity coverage differs')
    require(set(integrity.get('reviewed_methods', [])) == set(REVIEWED_METHODS), 'Unreviewed method')
    protocol_path = check_ref(integrity['protocol'], 'protocol')
    protocol = read(protocol_path)
    require(sha(protocol_path) == integrity['protocol_sha256'], 'Integrity protocol SHA differs')
    check_ref(integrity['run_index'], 'run index')
    require(integrity['run_index']['sha256'] == integrity['run_index_sha256'], 'Index SHA differs')
    check_ref(integrity['metrics_contract'], 'metrics contract')
    input_path = check_ref(integrity['input_audit'], 'input audit')
    require(read(input_path).get('passed') is True and not read(input_path).get('errors')
            and integrity['input_audit'] == protocol['input_audit'], 'Input audit no longer passed or its binding differs')
    blocks = integrity['blocks']
    require(len(blocks) == TOTALS['blocks'] and len({b['block'] for b in blocks}) == len(blocks), 'Audit block coverage differs')
    require({b['block'] for b in blocks} == set(protocol['blocks']), 'Audit/protocol blocks differ')
    all_runs = {b['run']: b['run_manifest_sha256'] for b in blocks}
    require(len(all_runs) == len(blocks) and all_runs == integrity['run_manifest_sha256'], 'Audit run binding differs')
    main_runs = {b['run']: b['run_manifest_sha256'] for b in blocks if b['phase'] == 'main'}
    require(len(main_runs) == TOTALS['main_blocks'], 'Main run count differs')
    for block in blocks:
        require(sha(Path(block['run']) / 'run_manifest.json') == block['run_manifest_sha256'], 'Run changed since audit')
        require(set(block['independent']) == set(AUDITORS), 'Missing independent auditor')
        for kind, evidence in block['independent'].items():
            require(evidence.get('passed') is True and read(check_ref(evidence['report'], f'{block["block"]}/{kind}')).get('passed') is True,
                    'Independent audit did not pass or changed')
    _, subset, expected = declared_matrix(protocol)
    main, scaling = read(main_path), read(scaling_path)
    main_keys = inspect_metrics(main, main_runs, TOTALS['main_jobs'], None, 'main')
    scaling_expected = {key for key in expected['main'] if key[0] in set(subset)} | expected['size_effect']
    scaling_keys = inspect_metrics(scaling, all_runs, len(scaling_expected), subset, 'scaling')
    require(main_keys == expected['main'] and scaling_keys == scaling_expected, 'Metrics do not cover the exact preregistered matrices')
    # Bind reported classifications to independently reviewed terminal statuses.
    status_lookup, job_lookup = {}, {}
    for block in blocks:
        for jid, job in block['job_evidence'].items():
            job_lookup[(block['run'], jid)] = job
        for kind, evidence in block['independent'].items():
            audited = read(evidence['report']['path'])
            records = audited['records'] if kind == 'nto' else [r for r in audited['records'] if r.get('latest') is True]
            for row in records:
                status_lookup[(block['run'], row['job_id'])] = row['status']
    for metrics in (main, scaling):
        for record in metrics['runs']:
            require(record.get('source_commit') == integrity['source_code_commit'], 'Metric runtime commit differs')
        for row in metrics['rows']:
            key = (str(Path(row['run']).resolve()), row['job_id'])
            require(status_lookup.get(key) == row['status'],
                    'Metric classification differs from independent evidence')
            require(key in job_lookup and all(row.get(field) == value for field, value in job_lookup[key].items()),
                    'Metric row identity, seed or input legality differs from the sealed job')
            require(row.get('success') == (row['status'] == 'success_validated_numerical'),
                    'Metric success flag differs from its terminal status')
    bundle = dict(schema_version=1, kind='formal_baseline_audit_bundle', complete=True, passed=True, errors=[],
                  created_utc=stamp(), protocol_sha256=integrity['protocol_sha256'],
                  source_code_commit=integrity['source_code_commit'], run_index_sha256=integrity['run_index_sha256'],
                  metrics_sha256={'main': sha(main_path), 'scaling': sha(scaling_path)},
                  metrics={'main': ref(main_path), 'scaling': ref(scaling_path)}, run_manifest_sha256=all_runs,
                  evidence={'input': ref(input_path), 'integrity': ref(integrity_path), 'independent': ref(integrity_path)},
                  covered_job_slots=TOTALS['jobs'], legal_planner_jobs=TOTALS['legal'], input_invalid_slots=TOTALS['invalid'],
                  main_observations=TOTALS['main_jobs'], scaling_observations=len(scaling_expected),
                  scaling_reuses_main_observations=len(scaling_expected & main_keys),
                  reviewed_methods=REVIEWED_METHODS, seal_script=ref(__file__))
    write(output, bundle)
    return bundle


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    audit = sub.add_parser('audit')
    for name in ('index', 'completed', 'protocol', 'output'):
        audit.add_argument('--' + name, type=Path, required=True)
    audit.add_argument('--expected-source', required=True)
    audit.add_argument('--expected-protocol-sha', required=True)
    audit.add_argument('--planning-finished', action='store_true')
    seal = sub.add_parser('seal-bundle')
    for name in ('integrity', 'main-metrics', 'scaling-metrics', 'output'):
        seal.add_argument('--' + name, type=Path, required=True)
    args = parser.parse_args(argv)
    if args.command == 'audit':
        report = execute_audit(args.index, args.completed, args.protocol, args.output,
                               args.expected_source, args.expected_protocol_sha, args.planning_finished)
    else:
        report = seal_bundle(args.integrity, args.main_metrics, args.scaling_metrics, args.output)
    print(json.dumps({k: report[k] for k in ('complete', 'passed', 'covered_job_slots')}, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
