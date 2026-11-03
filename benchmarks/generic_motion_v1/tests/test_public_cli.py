"""Execution-policy, provenance and portable public-entry regressions."""
import fcntl
import hashlib
import json
from pathlib import Path
import subprocess
import sys

import pytest

from benchmarks.generic_motion_v1 import batch
from benchmarks.generic_motion_v1.common import ROOT, read, write


def test_public_methods_from_other_directory(tmp_path):
    command = [sys.executable, str(ROOT / 'scripts/evaluate.py'), 'methods']
    result = subprocess.run(command, cwd=tmp_path, capture_output=True, text=True, check=True)
    methods = {r['method']: r for r in json.loads(result.stdout)}
    assert len(methods) == 10
    assert sum(len(row['objectives']) for row in methods.values()) == 14
    assert methods['NTO']['display_name'] == 'NTO legacy adaptation'
    assert methods['NTO']['objectives'] == ['minimum_distance', 'minimum_time']
    assert 'adaptation' in methods['CHOMP']['display_name']
    assert 'adaptation' in methods['TRAJOPT']['display_name']


def test_public_show_resolves_original_case_from_other_directory(tmp_path):
    command = [sys.executable, str(ROOT / 'scripts/benchmark.py'), 'show', 'dev-bottleneck-000-p00']
    result = subprocess.run(command, cwd=tmp_path, capture_output=True, text=True, check=True)
    record = json.loads(result.stdout)
    assert record['problem_id'] == 'dev-bottleneck-000-p00'
    assert 'voxels_xyz' not in record  # index refers to immutable source arrays


def _prepare_test(tmp_path, monkeypatch, allow_test=False, snapshot_sources=True):
    monkeypatch.setattr(batch, 'code_inventory', lambda: {})
    path = tmp_path / 'test.jsonl'
    path.write_text(json.dumps(dict(problem_id='case', scene_id='scene', family='open_control',
        split='test', paired_seeds=[dict(drm_seed=0, method_seed=121, solver_seed=323)]))+'\n')
    return batch.prepare(path, None, tmp_path/'run', methods=['RRTCONNECT'],
        objectives=['minimum_distance'], seed_indices=[0], allow_test=allow_test,
        snapshot_sources=snapshot_sources)


def test_explicit_current_test_execution_needs_no_approval_copies(tmp_path, monkeypatch):
    run = _prepare_test(tmp_path, monkeypatch, allow_test=True, snapshot_sources=False)
    manifest = read(run/'run_manifest.json')
    assert manifest['formal_test_run'] and manifest['execution_policy']=='fixed_public_inputs'
    assert manifest['source_recording']=='git' and not (run/'source_snapshot').exists()
    assert batch.verify(run)['integrity_passed']
    # Integrity is independent of the old interactive approval documents.
    manifest['formal_test_run'] = False
    write(run/'run_manifest.json',manifest)
    write(run/'identity.json',{'run_manifest_sha256':batch.sha(run/'run_manifest.json')})
    assert not batch.verify(run)['integrity_passed']


def test_old_test_guard_remains_for_legacy_api(tmp_path, monkeypatch):
    with pytest.raises(ValueError,match='locked'):
        _prepare_test(tmp_path, monkeypatch)


def test_git_provenance_reads_actual_commit_bytes():
    commit = subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
    name = 'LICENSE'
    payload = subprocess.check_output(['git','show',f'{commit}:{name}'],cwd=ROOT)
    assert not batch._git_source_errors(commit,{name:hashlib.sha256(payload).hexdigest()})
    assert batch._git_source_errors(commit,{name:'0'*64}) == [name]


def test_separate_runs_share_project_gpu_lock(tmp_path, monkeypatch):
    monkeypatch.setattr(batch,'BASE',tmp_path)
    lock = tmp_path/'results/.gpu.lock';lock.parent.mkdir()
    run = tmp_path/'run';run.mkdir()
    monkeypatch.setattr(batch,'_resume_locked',lambda *args:pytest.fail('GPU lock must prevent execution'))
    with lock.open('a+') as stream:
        fcntl.flock(stream,fcntl.LOCK_EX|fcntl.LOCK_NB)
        with pytest.raises(RuntimeError,match='owns the GPU'):
            batch.resume(run)


def test_status_distinguishes_output_error_and_pending_interruption(tmp_path, capsys):
    from benchmarks.generic_motion_v1.cli import evaluate_main
    run = tmp_path / 'run'
    jobs = [dict(job_id='output', case={'validation_status':'valid_input'}),
            dict(job_id='input', case={'validation_status':'generation_failed'}),
            dict(job_id='interrupted', case={'validation_status':'valid_input'})]
    write(run/'run_manifest.json', {'jobs':jobs})
    for job in jobs:
        receipt = {'status':'invalid_endpoint'}
        if job['job_id']=='interrupted':
            receipt = {'status':'infrastructure_error', 'retry_pending':True}
        attempt=run/'jobs'/job['job_id']/'attempts/0001'
        write(attempt/'result.json', {'status':receipt['status']})
        receipt['files']={'result.json':batch.sha(attempt/'result.json')}
        write(attempt/'receipt.json', receipt)
    evaluate_main(['status','--run',str(run)])
    status=json.loads(capsys.readouterr().out)
    assert status['completed']==2 and status['pending']==1
    assert status['interrupted_jobs_awaiting_resume']==1
    assert status['statuses']=={'validation_failed':1, 'invalid_input':1}


@pytest.mark.parametrize('split,seeds', [('smoke', [0]), ('dev', [0, 1, 2]), ('test', [0])])
def test_public_legacy_nto_dispatches_both_tracks_on_current_protocol(tmp_path, monkeypatch, split, seeds):
    from benchmarks.generic_motion_v1 import cli, roadmap
    calls = []
    config = {'online_budget_seconds': 60, 'nto': {'solver': 'IPOPT'}}
    monkeypatch.setattr(cli, 'require_environment', lambda: None)
    monkeypatch.setattr(cli, 'ROOT', tmp_path)
    monkeypatch.setattr(cli, 'BASE', tmp_path / 'benchmark')
    monkeypatch.setattr(roadmap, 'load_config', lambda path: config)
    def capture_prepare(manifest, roadmaps, output, **kwargs):
        calls.append((manifest, roadmaps, output, kwargs))
        return output
    monkeypatch.setattr(batch, 'prepare', capture_prepare)
    monkeypatch.setattr(batch, 'resume', lambda *a, **kw: pytest.fail('prepare-only must not plan'))
    monkeypatch.chdir(tmp_path.parent)
    cli.evaluate_main(['batch', '--split', split, '--methods', 'NTO', '--track', 'both',
                      '--seeds', *map(str, seeds), '--config', 'configs/evaluation.json',
                      '--output', 'results/legacy', '--prepare-only'])
    assert len(calls) == 1
    manifest, roadmaps, output, options = calls[0]
    assert manifest == tmp_path / 'benchmark' / 'manifests' / f'{split}.jsonl'
    assert roadmaps == [tmp_path / 'benchmark' / 'roadmaps' / f'rss2025_n6000_s{seed:03d}'
                        for seed in seeds]
    assert output == tmp_path / 'results/legacy'
    assert options['methods'] == ['NTO']
    assert options['objectives'] == ['minimum_distance', 'minimum_time']
    assert options['timeout'] == 60 and options['config'] is config
    assert options['mode'] == 'end_to_end' and options['initial_source_run'] is None
    assert options['allow_test'] and not options['snapshot_sources']
    assert options['problem'] is None and options['seed_indices'] is None
    assert not output.exists()


def test_default_public_batch_includes_all_ten_existing_methods(tmp_path, monkeypatch):
    from benchmarks.generic_motion_v1 import cli, roadmap
    from benchmarks.generic_motion_v1.runners.methods import METHODS, objectives
    calls = []
    monkeypatch.setattr(cli, 'require_environment', lambda: None)
    monkeypatch.setattr(roadmap, 'load_config', lambda path: {'online_budget_seconds': 60})
    def capture_prepare(manifest, roadmaps, output, **kwargs):
        calls.append(kwargs)
        return output
    monkeypatch.setattr(batch, 'prepare', capture_prepare)
    monkeypatch.setattr(batch, 'resume', lambda *a, **kw: pytest.fail('prepare-only must not plan'))
    cli.evaluate_main(['batch', '--split', 'smoke', '--output', str(tmp_path / 'all'), '--prepare-only'])
    assert calls[0]['methods'] == list(METHODS)
    assert len(calls[0]['methods']) == 10
    assert sum(len(objectives(method)) for method in calls[0]['methods']) == 14


@pytest.mark.parametrize('command,split,expected', [
    ('batch', 'smoke', [0]), ('batch', 'dev', [0, 1, 2]),
    ('batch', 'test', [0, 1, 2]), ('run', 'dev', [0]), ('run', 'test', [0]),
])
def test_default_repetition_coverage_is_explicit(tmp_path, monkeypatch, capsys, command, split, expected):
    from benchmarks.generic_motion_v1 import cli, roadmap
    calls = []
    monkeypatch.setattr(cli, 'require_environment', lambda: None)
    monkeypatch.setattr(roadmap, 'load_config', lambda path: {'online_budget_seconds': 60})
    def capture_prepare(manifest, roadmaps, output, **kwargs):
        calls.append(kwargs)
        return output
    monkeypatch.setattr(batch, 'prepare', capture_prepare)
    monkeypatch.setattr(batch, 'resume', lambda *a, **kw: pytest.fail('prepare-only must not plan'))
    args = [command, '--split', split, '--methods', 'RRTCONNECT', '--track', 'path',
            '--output', str(tmp_path / 'new'), '--prepare-only']
    if command == 'run':
        args += ['--problem', 'fixed-problem']
    cli.evaluate_main(args)
    assert calls[0]['seed_indices'] == expected
    assert json.loads(capsys.readouterr().out)['requested_seed_indices'] == expected
