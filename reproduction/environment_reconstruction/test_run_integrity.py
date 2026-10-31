"""CPU-only regressions for frozen inputs, output integrity and failed-job resume.

No model, CUDA, IK, solver, child process, or real run directory is opened.
Interrupted jobs must retain their evidence without automatic retry.
"""
from pathlib import Path
import sys
import types

import pytest

from reproduction.common import atomic, read
from reproduction.environment_reconstruction import run
import reproduction.run as base_run


@pytest.fixture
def frozen(tmp_path, monkeypatch):
    root = tmp_path / 'run'
    atomic(root / 'protocol.json', {'name': 'fixed-protocol'})
    atomic(root / 'scenes/scene.json', {'seed': 17, 'centers': [[.2, -.3]]})
    source = {'code.py': 'unchanged-source'}
    runtime = {'numpy': 'fixed-version'}
    monkeypatch.setattr(run, 'sources', lambda: source.copy())
    monkeypatch.setattr(base_run, 'runtime_versions', lambda: runtime.copy())
    case = dict(id='fixed_case', kind='forest', scene='scenes/scene.json', roadmap='map', size=200)
    manifest = dict(source_hashes=source, runtime_versions=runtime,
                    protocol_sha256=run.sha(root/'protocol.json'),
                    scene_hashes=run.receipt(root/'scenes'), cases=[case],
                    profile={'child_timeout_seconds': 1}, scope='test-only')
    atomic(root/'manifest.json', manifest)
    # Replace only the process-launch boundary; exercise the real resume logic.
    stub = types.ModuleType('reproduction.paper_constructed.run')
    stub.execute_child = lambda *a, **kw: pytest.fail('resume unexpectedly launched a child')
    monkeypatch.setitem(sys.modules, 'reproduction.paper_constructed.run', stub)
    return root, case, stub


def complete_job(folder, *, failed=False):
    folder.mkdir(parents=True, exist_ok=True)
    (folder/'native.map').write_bytes(b'original frozen map')
    (folder/'stderr.log').write_text('retained diagnostic\n' if failed else '')
    atomic(folder/'execution.json', dict(exit_code=1 if failed else 0, timed_out=False,
                                         output_hashes=run.job_receipt(folder)))


@pytest.mark.parametrize('mutation', ['modify', 'delete', 'add'])
def test_changed_scene_set_cannot_resume(frozen, mutation):
    root, _, _ = frozen
    assert run.check_run(root)['cases']
    path = root/'scenes/scene.json'
    if mutation == 'modify':
        atomic(path, {'seed': 17, 'centers': [[.3, -.3]]})
    elif mutation == 'delete':
        path.unlink()
    else:
        atomic(root/'scenes/injected.json', {'new_scene': True})
    with pytest.raises(ValueError, match='Frozen input scenes changed'):
        run.check_run(root)


def test_modified_protocol_cannot_resume(frozen):
    root, _, _ = frozen
    atomic(root/'protocol.json', {'name': 'different-protocol'})
    with pytest.raises(ValueError, match='Frozen protocol changed'):
        run.check_run(root)


def test_changed_code_or_runtime_cannot_resume(frozen, monkeypatch):
    root, _, _ = frozen
    monkeypatch.setattr(run, 'sources', lambda: {'code.py': 'new-source'})
    with pytest.raises(ValueError, match='Computational sources changed'):
        run.check_run(root)
    monkeypatch.setattr(run, 'sources', lambda: {'code.py': 'unchanged-source'})
    monkeypatch.setattr(base_run, 'runtime_versions', lambda: {'numpy': 'different-version'})
    with pytest.raises(ValueError, match='Numerical environment changed'):
        run.check_run(root)


@pytest.mark.parametrize('mutation', ['modify', 'delete', 'add'])
def test_completed_map_tamper_is_detected_before_case_launch(frozen, mutation):
    root, _, _ = frozen
    folder=root/'roadmaps/map'
    complete_job(folder)
    if mutation == 'modify':
        (folder/'native.map').write_bytes(b'replaced map')
    elif mutation == 'delete':
        (folder/'native.map').unlink()
    else:
        (folder/'unrecorded.map').write_bytes(b'new map')
    with pytest.raises(ValueError, match='Completed job outputs changed'):
        run.smoke(root)
    assert not (root/'cases').exists()


def test_completed_failed_case_is_preserved_without_retry(frozen):
    root, case, _ = frozen
    complete_job(root/'roadmaps/map')
    folder=root/'cases'/case['id']
    atomic(folder/'case.json', dict(**case, status='validation_failed',
                                   validation={'minimum_clearance': -.01}))
    (folder/'stderr.log').write_text('original collision evidence\n')
    atomic(folder/'execution.json', dict(exit_code=0, timed_out=False,
                                         output_hashes=run.job_receipt(folder)))
    original=run.receipt(folder)
    run.smoke(root)
    run.smoke(root)
    assert run.receipt(folder)==original
    summary=read(root/'summary.json')
    assert summary['requested']==1
    assert summary['by_environment']['forest']=={'validation_failed': 1}


def test_failed_map_retains_failure_and_all_scheduled_cases(frozen):
    root, case, _ = frozen
    folder=root/'roadmaps/map'
    complete_job(folder, failed=True)
    original=run.receipt(folder)
    run.smoke(root)
    case_original=run.receipt(root/'cases'/case['id'])
    run.smoke(root)
    assert run.receipt(folder)==original
    assert run.receipt(root/'cases'/case['id'])==case_original
    assert read(root/'cases'/case['id']/'case.json')['status']=='roadmap_failed'
    assert read(root/'summary.json')['by_environment']['forest']=={'roadmap_failed': 1}


def test_failed_case_tamper_is_not_silently_trusted(frozen):
    root, case, _ = frozen
    complete_job(root/'roadmaps/map')
    folder=root/'cases'/case['id']
    atomic(folder/'case.json', dict(**case, status='timeout'))
    atomic(folder/'execution.json', dict(exit_code=-15, timed_out=True,
                                         output_hashes=run.job_receipt(folder)))
    atomic(folder/'case.json', dict(**case, status='validated'))
    with pytest.raises(ValueError, match='Completed job outputs changed'):
        run.smoke(root)


def test_unreceipted_interruption_does_not_destroy_previous_evidence(frozen):
    root, case, stub = frozen
    complete_job(root/'roadmaps/map')
    folder=root/'cases'/case['id']
    atomic(folder/'case.json', dict(**case, status='running', partial_evidence='first attempt'))
    (folder/'stderr.log').write_text('first interrupted attempt diagnostic\n')
    original=(folder/'stderr.log').read_bytes()
    original_case=(folder/'case.json').read_bytes()
    run.smoke(root)
    assert (folder/'stderr.log').read_bytes()==original
    assert (folder/'case.json').read_bytes()==original_case
    assert run.verify_job(folder)['status']=='interrupted_preserved'
    assert run.case_record(root, case)['status']=='interrupted_preserved'
    first_receipt=run.receipt(folder)
    run.smoke(root)
    assert run.receipt(folder)==first_receipt


def test_unreceipted_map_is_preserved_and_dependents_are_retained(frozen):
    root, case, _ = frozen
    folder=root/'roadmaps/map'
    folder.mkdir(parents=True)
    (folder/'native.map').write_bytes(b'incomplete map retained for diagnosis')
    (folder/'stderr.log').write_text('interrupted map build diagnostic\n')
    original_map=(folder/'native.map').read_bytes()
    original_log=(folder/'stderr.log').read_bytes()
    run.smoke(root)
    assert (folder/'native.map').read_bytes()==original_map
    assert (folder/'stderr.log').read_bytes()==original_log
    execution=run.verify_job(folder)
    assert execution.get('status')=='interrupted_preserved' or execution.get('skipped')=='interrupted_preserved'
    assert read(root/'cases'/case['id']/'case.json')['status']=='roadmap_failed'
    original=run.receipt(root/'cases'/case['id'])
    run.smoke(root)
    assert run.receipt(root/'cases'/case['id'])==original


def test_direct_case_cannot_replace_completed_failure(frozen, monkeypatch):
    root, case, _ = frozen
    complete_job(root/'roadmaps/map')
    folder=root/'cases'/case['id']
    atomic(folder/'case.json', dict(**case, status='validation_failed'))
    (folder/'stderr.log').write_text('must retain this failed case\n')
    original=run.receipt(folder)
    # Prevent optional native/model imports; the guard must fire before use.
    def forbidden(*args, **kwargs):
        pytest.fail('completed case must be rejected before constructing model or planning')
    for module_name, members in [
        ('reproduction.paper_constructed.data', ['planner','online_pose_ik','endpoint_audit']),
        ('reproduction.paper_constructed.model', ['FR3']),
        ('reproduction.paper_constructed.validation', ['polygon_samples']),
        ('reproduction.environment_reconstruction.model', ['FixedForest']),
    ]:
        stub=types.ModuleType(module_name)
        for name in members: setattr(stub,name,forbidden)
        monkeypatch.setitem(sys.modules,module_name,stub)
    with pytest.raises((ValueError, FileExistsError)):
        run.run_case(root, case)
    assert run.receipt(folder)==original


def test_direct_case_cannot_consume_a_replaced_map(frozen, monkeypatch):
    root, case, _ = frozen
    folder=root/'roadmaps/map'
    complete_job(folder)
    (folder/'native.map').write_bytes(b'different geometry roadmap')
    def forbidden(*args, **kwargs):
        pytest.fail('map identity must be verified before constructing model or planning')
    for module_name, members in [
        ('reproduction.paper_constructed.data', ['planner','online_pose_ik','endpoint_audit']),
        ('reproduction.paper_constructed.model', ['FR3']),
        ('reproduction.paper_constructed.validation', ['polygon_samples']),
        ('reproduction.environment_reconstruction.model', ['FixedForest']),
    ]:
        stub=types.ModuleType(module_name)
        for name in members: setattr(stub,name,forbidden)
        monkeypatch.setitem(sys.modules,module_name,stub)
    with pytest.raises(ValueError, match='Completed job outputs changed'):
        run.run_case(root, case)
    assert not (root/'cases'/case['id']/'case.json').exists()
