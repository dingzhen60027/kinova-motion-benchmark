"""Small synthetic artifacts only; never opens any actual study run."""
from pathlib import Path
import json
import pytest

from benchmarks.generic_motion_v1.reports.baseline_study import audit_formal_study as audit
from benchmarks.generic_motion_v1.reports.baseline_study import formal_scs_audit as scs

SOURCE = '1' * 40


def save(path, value):
    audit.write(path, value)
    return path


@pytest.fixture
def matrix(tmp_path, monkeypatch):
    totals = dict(blocks=2, main_blocks=1, size_blocks=1, jobs=20, legal=10,
                  invalid=10, main_jobs=10, size_jobs=10, main_cases=2,
                  legal_cases=1, subset_cases=2, subset_legal=1)
    monkeypatch.setattr(audit, 'TOTALS', totals)
    cases = {f'test-sparse_clutter-000-p0{i}': dict(problem_id=f'test-sparse_clutter-000-p0{i}',
             scene_id='test-sparse_clutter-000', family='sparse_clutter', split='test',
             validation_status='valid_input' if i == 0 else 'invalid_endpoint',
             paired_seeds=[dict(drm_seed=0, method_seed=17)], q_start=[0]*7, q_goal=[1]*7)
             for i in range(2)}
    data = tmp_path / 'cases.jsonl'
    data.write_text(''.join(json.dumps(c) + '\n' for c in cases.values()))
    input_audit = save(tmp_path / 'input_audit.json', dict(passed=True, manifest_sha256=audit.sha(data)))
    contract = tmp_path / 'contract.md'; contract.write_text('Frozen test fixture metrics contract.\n')
    subset = save(tmp_path / 'subset.json', list(cases))
    protocol = dict(state='approved', authorization=dict(text='Synthetic fixture, never an actual study authorization'),
        source_code_commit=SOURCE, runtime_source_hashes={'runtime.py': 'a'*64},
        manifest=audit.ref(data), input_audit=audit.ref(input_audit), metrics_contract=audit.ref(contract),
        size_selection=dict(problem_ids_file=audit.ref(subset)), blocks={})
    items, keys = [], {'main': set(), 'size_effect': set()}
    manifests = {}
    for phase, nodes in [('main', 6000), ('size_effect', 3000)]:
        name = phase + '_fixture'
        run = tmp_path / name; run.mkdir()
        roadmap = tmp_path / ('map' + str(nodes)); roadmap.mkdir()
        metadata = save(roadmap / 'metadata.json', dict(requested_nodes=nodes, actual_nodes=nodes,
                        seed=0, status='ready', roadmap_id=roadmap.name))
        files = {str(metadata): audit.sha(metadata)}
        block = dict(phase=phase, roadmap_dirs=[str(roadmap)], roadmap_seeds=[0], requested_node_budgets=[nodes],
                     method_pairs=list(map(list, audit.LABELS)), problem_ids=list(cases), timeout_seconds=60.,
                     evaluation_config={'fixed': True}, roadmap_files=files, planned_jobs=10)
        protocol['blocks'][name] = block
        jobs = []
        for cid, case in cases.items():
            for method, objective in audit.LABELS:
                jid = f'{cid}-{method}-{objective}'
                job = dict(job_id=jid, case_id=cid, case=case, method=method, objective=objective,
                           evaluation_config=block['evaluation_config'], roadmap_dir=str(roadmap),
                           roadmap_id=roadmap.name, method_seed=17)
                jobs.append(job)
                attempt = run / 'jobs' / jid / 'attempts' / '0001'
                status = 'solver_failed' if case['validation_status'] == 'valid_input' else 'invalid_endpoint'
                save(attempt / 'receipt.json', dict(status=status, retry_pending=False))
                save(attempt / 'result.json', dict(status=status, success=False))
                keys[phase].add((cid, nodes, 0, method, objective))
        manifests[name] = dict(source_code_commit=SOURCE, source_hashes=protocol['runtime_source_hashes'],
            manifest_sha256=audit.sha(data), formal_test_run=True, split='test', timeout_seconds=60.,
            roadmap_files=files, jobs=jobs)
        items.append(dict(block=name, phase=phase, run=str(run), jobs=10))
    pp = save(tmp_path / 'protocol.json', protocol)
    for item in items:
        manifest = manifests[item['block']]
        manifest['study_protocol'] = dict(sha256=audit.sha(pp), block=item['block'])
        item['run_manifest_sha256'] = audit.sha(save(Path(item['run']) / 'run_manifest.json', manifest))
    index = save(tmp_path / 'index.json', dict(matrix_prepared=True, blocks=items,
             protocol_sha256=audit.sha(pp), registered_job_slots=20, legal_planner_jobs=10, input_invalid_slots=10))
    verified = dict(integrity_passed=True, complete=True, completed_jobs=10, planned_jobs=10, errors=[], unsealed_attempts=[])
    complete = save(tmp_path / 'complete.json', dict(complete=True, completed_jobs=20, total_jobs=20,
                    completed=[dict(i, verification=verified) for i in items]))
    calls = []
    monkeypatch.setattr(audit, 'declared_matrix', lambda p: (cases, list(cases), keys))
    monkeypatch.setattr(audit, 'verify_run', lambda run: dict(verified))

    def invoke(kind, run, out, source):
        calls.append((kind, str(run), source))
        manifest = audit.read(Path(run) / 'run_manifest.json')
        records = []
        labels = {'scs': {'LSCS', 'SCSTO'}, 'nto': {'NTO-distance', 'NTO-time'}, 'lgcs': {'LGCS'}}[kind]
        for job in manifest['jobs']:
            if audit.LABELS[(job['method'], job['objective'])] in labels:
                result = audit.read(Path(run) / 'jobs' / job['job_id'] / 'attempts/0001/result.json')
                records.append(dict(job_id=job['job_id'], latest=True, status=result['status']))
        report = dict(passed=True, run=str(run), run_manifest_sha256=audit.sha(Path(run) / 'run_manifest.json'),
            frozen_planning_commit=source, source_commit=source, expected_source_commit=source,
            records=records, planned_jobs=10, completed_latest_jobs=10, receipts_verified=10,
            complete=True, scs_latest_jobs=len(records), nto_jobs=len(records), planned_lgcs_jobs=len(records),
            audit_source_sha256=audit.sha(__file__), audit_script_sha256=audit.sha(__file__))
        return {'returncode': 0, 'audit_script': audit.ref(__file__)}, save(out / 'audit.json', report)

    monkeypatch.setattr(audit, 'invoke_auditor', invoke)
    return dict(root=tmp_path, source=SOURCE, protocol=pp, index=index, completed=complete,
                output=tmp_path / 'audited', calls=calls, items=items, cases=cases, keys=keys, invoke=invoke)


def execute(f, **overrides):
    kwargs = dict(index_path=f['index'], completed_path=f['completed'], protocol_path=f['protocol'],
                  output=f['output'], source=f['source'], protocol_sha=audit.sha(f['protocol']), planning_finished=True)
    kwargs.update(overrides)
    return audit.execute_audit(**kwargs)


def modify(path, fn):
    value = audit.read(path); fn(value); save(path, value)


def test_success_preserves_failures_and_invalid_denominator(matrix):
    result = execute(matrix)
    assert result['passed'] and result['complete'] and result['all_outcomes_reviewed']
    assert result['covered_job_slots'] == 20 and result['legal_planner_jobs'] == 10
    assert result['input_invalid_slots'] == 10 and len(matrix['calls']) == 6
    assert all(call[2] == SOURCE for call in matrix['calls'])
    for block in result['blocks']:
        for method in block['methods'].values():
            assert method['legal_statuses'] == {'solver_failed': 1}
            assert method['invalid_statuses'] == {'invalid_endpoint': 1}
    with pytest.raises(ValueError, match='overwrite'):
        execute(matrix)


@pytest.mark.parametrize('mutation', ['flag', 'incomplete', 'missing_block', 'unverified', 'wrong_total'])
def test_completion_barrier_prevents_any_run_read(matrix, monkeypatch, mutation):
    if mutation == 'incomplete':
        modify(matrix['completed'], lambda d: d.update(complete=False))
    elif mutation == 'missing_block':
        modify(matrix['completed'], lambda d: d['completed'].pop())
    elif mutation == 'unverified':
        modify(matrix['completed'], lambda d: d['completed'][0]['verification'].update(complete=False))
    elif mutation == 'wrong_total':
        modify(matrix['completed'], lambda d: d.update(completed_jobs=19))
    monkeypatch.setattr(audit, 'inspect_block', lambda *a: pytest.fail('Large run read during active planning'))
    with pytest.raises(ValueError):
        execute(matrix, planning_finished=mutation != 'flag')
    failed = audit.read(matrix['output'] / 'integrity_audit.json')
    assert not failed['passed'] and failed['errors'] and not matrix['calls']


@pytest.mark.parametrize('mutation', ['source', 'protocol_sha', 'manifest', 'index_missing_block'])
def test_metadata_binding_fail_closed(matrix, mutation):
    overrides = {}
    if mutation == 'source': overrides['source'] = '2'*40
    if mutation == 'protocol_sha': overrides['protocol_sha'] = '0'*64
    if mutation == 'manifest':
        p = Path(matrix['items'][0]['run']) / 'run_manifest.json'
        modify(p, lambda d: d.update(timeout_seconds=59.))
    if mutation == 'index_missing_block':
        modify(matrix['index'], lambda d: d['blocks'].pop())
    with pytest.raises(ValueError): execute(matrix, **overrides)
    assert not matrix['calls']
    assert audit.read(matrix['output'] / 'integrity_audit.json')['passed'] is False


def resave_run_bindings(f, idx=0):
    item = f['items'][idx]
    digest = audit.sha(Path(item['run']) / 'run_manifest.json')
    item['run_manifest_sha256'] = digest
    modify(f['index'], lambda d: d['blocks'][idx].update(run_manifest_sha256=digest))
    modify(f['completed'], lambda d: d['completed'][idx].update(run_manifest_sha256=digest))


@pytest.mark.parametrize('mutation', ['block', 'source_inventory', 'missing_method', 'duplicate_job', 'case', 'config', 'budget', 'method_seed'])
def test_exact_block_job_protocol_checks(matrix, mutation):
    path = Path(matrix['items'][0]['run']) / 'run_manifest.json'
    def change(d):
        if mutation == 'block': d['study_protocol']['block'] = 'other'
        if mutation == 'source_inventory': d['source_hashes']['runtime.py'] = 'b'*64
        if mutation == 'missing_method': d['jobs'].pop()
        if mutation == 'duplicate_job': d['jobs'][1] = d['jobs'][0]
        if mutation == 'case': d['jobs'][0]['case']['q_goal'][0] = 2
        if mutation == 'config': d['jobs'][0]['evaluation_config']['fixed'] = False
        if mutation == 'budget': d['timeout_seconds'] = 61.
        if mutation == 'method_seed': d['jobs'][0]['method_seed'] = 22
    modify(path, change); resave_run_bindings(matrix)
    with pytest.raises(ValueError): execute(matrix)
    assert not matrix['calls']


def test_invalid_cannot_be_promoted(matrix):
    run = Path(matrix['items'][0]['run'])
    manifest = audit.read(run / 'run_manifest.json')
    job = next(j for j in manifest['jobs'] if j['case']['validation_status'] != 'valid_input')
    attempt = run / 'jobs' / job['job_id'] / 'attempts/0001'
    save(attempt / 'result.json', dict(status='solver_failed', success=False))
    save(attempt / 'receipt.json', dict(status='solver_failed', retry_pending=False))
    with pytest.raises(ValueError, match='Invalid input'): execute(matrix)
    assert not matrix['calls']


def test_invalid_cannot_have_planning_artifacts(matrix):
    run = Path(matrix['items'][0]['run'])
    job = next(j for j in audit.read(run / 'run_manifest.json')['jobs'] if j['case']['validation_status'] != 'valid_input')
    (run / 'jobs' / job['job_id'] / 'attempts/0001/path.npy').write_bytes(b'not actually numpy')
    with pytest.raises(ValueError, match='planning artifacts'): execute(matrix)


def test_verify_failure_prevents_independent_audits(matrix, monkeypatch):
    monkeypatch.setattr(audit, 'verify_run', lambda r: dict(integrity_passed=False, complete=False, errors=['bad receipt']))
    with pytest.raises(ValueError, match='receipt verification'): execute(matrix)
    assert not matrix['calls']


@pytest.mark.parametrize('mutation', ['process', 'passed', 'manifest', 'source', 'missing_record', 'status', 'auditor_source'])
def test_independent_failure_preserved_and_blocks_publication(matrix, monkeypatch, mutation):
    invoke = matrix['invoke']
    def changed(*args):
        invocation, path = invoke(*args)
        if mutation == 'process': invocation['returncode'] = 1
        elif mutation == 'passed': modify(path, lambda d: d.update(passed=False))
        elif mutation == 'manifest': modify(path, lambda d: d.update(run_manifest_sha256='0'*64))
        elif mutation == 'source': modify(path, lambda d: d.update(frozen_planning_commit='2'*40))
        elif mutation == 'missing_record': modify(path, lambda d: d['records'].pop())
        elif mutation == 'status': modify(path, lambda d: d['records'][0].update(status='success_validated_numerical'))
        elif mutation == 'auditor_source': modify(path, lambda d: d.update(audit_script_sha256='0'*64))
        return invocation, path
    monkeypatch.setattr(audit, 'invoke_auditor', changed)
    with pytest.raises(ValueError): execute(matrix)
    failed = audit.read(matrix['output'] / 'integrity_audit.json')
    assert failed['integrity_passed'] and not failed['independent_passed'] and not failed['passed']
    assert failed['errors'] and len(matrix['calls']) == 1
    assert failed['blocks'][0]['independent']['scs']['report']['sha256']


def make_metrics(f, report):
    rows, runs = [], []
    for item in f['items']:
        run = Path(item['run']); manifest = audit.read(run / 'run_manifest.json')
        runs.append(dict(run=str(run), manifest_sha256=item['run_manifest_sha256'], source_commit=SOURCE))
        nodes = 6000 if item['phase'] == 'main' else 3000
        for job in manifest['jobs']:
            result = audit.read(run / 'jobs' / job['job_id'] / 'attempts/0001/result.json')
            rows.append(dict(run=str(run), job_id=job['job_id'], case_id=job['case_id'],
                roadmap_nodes=nodes, roadmap_seed=0, method=job['method'], objective=job['objective'],
                label=audit.LABELS[(job['method'],job['objective'])], status=result['status'],
                method_seed=job['method_seed'], input_legal=job['case']['validation_status']=='valid_input', success=False))
    main = dict(complete=True, comparable=True, pairing_differences=[], expected_method_labels=audit.REVIEWED_METHODS,
                requested_jobs=10, reporting_problem_ids=None, runs=runs[:1], rows=rows[:10])
    scaling = dict(main, requested_jobs=20, reporting_problem_ids=list(f['cases']), runs=runs, rows=rows)
    return save(f['root']/'main_metrics.json', main), save(f['root']/'scaling_metrics.json', scaling)


def test_bundle_binds_metrics_input_and_all_independent_reports(matrix):
    report = execute(matrix)
    main, scaling = make_metrics(matrix, report)
    out = matrix['root']/'bundle.json'
    bundle = audit.seal_bundle(matrix['output']/'integrity_audit.json', main, scaling, out)
    assert bundle['passed'] and bundle['covered_job_slots'] == 20
    assert bundle['metrics_sha256'] == {'main': audit.sha(main), 'scaling': audit.sha(scaling)}
    assert bundle['scaling_reuses_main_observations'] == 10
    assert bundle['evidence']['integrity'] == bundle['evidence']['independent']
    with pytest.raises(ValueError, match='replace'):
        audit.seal_bundle(matrix['output']/'integrity_audit.json', main, scaling, out)


@pytest.mark.parametrize('mutation', ['audit_changed', 'metrics_incomplete', 'wrong_runs', 'duplicate_rows', 'changed_status', 'wrong_subset', 'wrong_job_identity', 'wrong_input_legal', 'wrong_runtime'])
def test_bundle_rejects_changed_or_incomplete_evidence(matrix, mutation):
    report = execute(matrix); main, scaling = make_metrics(matrix, report)
    if mutation == 'audit_changed':
        path = Path(report['blocks'][0]['independent']['scs']['report']['path'])
        modify(path, lambda d: d.update(passed=False))
    elif mutation == 'metrics_incomplete': modify(main, lambda d: d.update(complete=False))
    elif mutation == 'wrong_runs': modify(main, lambda d: d['runs'][0].update(manifest_sha256='0'*64))
    elif mutation == 'duplicate_rows': modify(main, lambda d: d['rows'].__setitem__(1, d['rows'][0]))
    elif mutation == 'changed_status': modify(main, lambda d: d['rows'][0].update(status='success_validated_numerical'))
    elif mutation == 'wrong_subset': modify(scaling, lambda d: d.update(reporting_problem_ids=None))
    elif mutation == 'wrong_job_identity': modify(main, lambda d: d['rows'][0].update(method_seed=22))
    elif mutation == 'wrong_input_legal': modify(main, lambda d: d['rows'][0].update(input_legal=False))
    elif mutation == 'wrong_runtime': modify(main, lambda d: d['runs'][0].update(source_commit='2'*40))
    out = matrix['root']/'bundle.json'
    with pytest.raises(ValueError): audit.seal_bundle(matrix['output']/'integrity_audit.json', main, scaling, out)
    assert not out.exists()


def test_scs_explicit_source_required_and_mismatch_rejected(tmp_path):
    with pytest.raises(ValueError, match='40-character'):
        scs.audit(tmp_path, 'c8adbd8c')
    data = tmp_path/'empty.jsonl'; data.write_text('')
    manifest = dict(source_code_commit=SOURCE, jobs=[], manifest_path=str(data), manifest_sha256=audit.sha(data),
                    source_hashes={}, roadmap_files={})
    save(tmp_path/'run_manifest.json', manifest)
    save(tmp_path/'identity.json', dict(run_manifest_sha256=audit.sha(tmp_path/'run_manifest.json')))
    save(tmp_path/'source_snapshot/benchmarks/generic_motion_v1/configs/robot.json',
         dict(lower=[-1]*7, upper=[1]*7, contact_band_m=1e-5))
    result = scs.audit(tmp_path, '2'*40)
    assert not result['passed']
    assert any('Unexpected planning source commit' in v['message'] for v in result['issues'])
    assert result['expected_source_commit'] == '2'*40


def test_duplicate_global_observation_rejected(matrix):
    item = matrix['items'][0]
    protocol = audit.read(matrix['protocol'])
    already_seen = {next(iter(matrix['keys']['main']))}
    with pytest.raises(ValueError, match='Duplicate global job key'):
        audit.inspect_block(item, protocol['blocks'][item['block']], protocol,
            audit.sha(matrix['protocol']), SOURCE, matrix['cases'], already_seen)


def test_subprocess_scs_requires_explicit_expected_source(tmp_path, monkeypatch):
    calls = []
    def fake_run(argv, **kwargs):
        calls.append(argv)
        assert kwargs['check'] is False
        assert argv[-2:] == ['--expected-source', SOURCE]
        class Completed:
            returncode = 0
        return Completed()
    monkeypatch.setattr(audit.subprocess, 'run', fake_run)
    record, path = audit.invoke_auditor('scs', tmp_path/'synthetic_run', tmp_path/'synthetic_audit', SOURCE)
    assert len(calls) == 1 and record['returncode'] == 0
    assert record['audit_script']['sha256'] == audit.sha(scs.__file__)
    assert path.name == 'audit.json'
