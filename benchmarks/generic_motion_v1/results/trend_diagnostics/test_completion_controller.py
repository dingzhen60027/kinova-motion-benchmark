"""CPU-only small fixtures for comparison recovery; no benchmark runs are invoked."""
import importlib.util
import json
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('completion_controller', HERE.parent/'logs/complete_repair_evaluation.py')
controller = importlib.util.module_from_spec(spec)
spec.loader.exec_module(controller)


@pytest.fixture
def setup(tmp_path, monkeypatch):
    monkeypatch.setattr(controller, 'ROOT', tmp_path)
    logs = tmp_path/'results/logs'
    logs.mkdir(parents=True)
    monkeypatch.setattr(controller, 'logs', logs)
    monkeypatch.setattr(controller, 'events', logs/'completion_execution.jsonl')
    for name in ('results/records.csv', 'results/summary.csv',
                 'results/trend_diagnostics/compare_repaired_dev.py',
                 'benchmarks/generic_motion_v1/runners/results.py',
                 'benchmarks/generic_motion_v1/runners/summarize.py',
                 'benchmarks/generic_motion_v1/batch.py',
                 'benchmarks/generic_motion_v1/common.py',
                 'benchmarks/generic_motion_v1/outcomes.py'):
        path = tmp_path/name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text('fixed fixture bytes: '+name+'\n')
    for run in ('new', 'old'):
        path = tmp_path/'results'/run/'run_manifest.json'
        path.parent.mkdir(parents=True)
        path.write_text(json.dumps({'source_code_commit': run+'-commit'}))
    args = ['results/trend_diagnostics/compare_repaired_dev.py', '--run', 'results/new',
            '--baseline-comparison', 'current_test', '--new-comparison', 'current_test_repaired',
            '--output-prefix', 'results/trend_diagnostics/completed_test_comparison', '--replica-seed', '0']
    calls = []
    def emit(command, log_name, *, returncode=0):
        controller.record('command_started', argv=[controller.sys.executable, *command], log=str(logs/log_name))
        controller.record('command_finished', returncode=returncode, log=str(logs/log_name))
    def finalized(command):
        parsed = controller._comparison_spec(command)
        outputs = controller._comparison_outputs(parsed)
        report = dict(complete=True, final_statistics_produced=True, new_run='results/new',
            old_comparison='current_test', new_comparison='current_test_repaired', replica_seed=0,
            new_source_commit='new-commit',
            new_run_manifest_sha256=controller.sha(tmp_path/'results/new/run_manifest.json'),
            old_records_sha256=controller.sha(tmp_path/'results/records.csv'),
            primary_summary_sha256=controller.sha(tmp_path/'results/summary.csv'),
            configuration_audit={'methods': [{'old_run': 'results/old',
                'old_manifest_sha256': controller.sha(tmp_path/'results/old/run_manifest.json')}]})
        outputs[0].parent.mkdir(parents=True, exist_ok=True)
        outputs[0].write_text(json.dumps(report, indent=2)+'\n')
        outputs[1].write_text('pair,value\na,3\n')
        outputs[2].write_text('metric,value\nmean,3\n')
        return outputs
    def fake_command(command, log_name):
        calls.append(list(command))
        finalized(command)
        emit(command, log_name)
    monkeypatch.setattr(controller, 'command', fake_command)
    def forbidden(*a, **kw):
        pytest.fail('Small controller fixture must not launch any real subprocess or planner')
    monkeypatch.setattr(controller.subprocess, 'run', forbidden)
    return dict(root=tmp_path, args=args, calls=calls, finalized=finalized, emit=emit,
                log='completed_test_comparison.log', checkpoint=logs/'completed_test_comparison_checkpoint.json')


def test_first_execution_unchanged_then_verified_restart_skips(setup):
    s = setup
    controller.comparison_command(s['args'], s['log'])
    assert s['calls'] == [s['args']]
    before = controller._comparison_snapshot(s['args'])
    saved = controller.read(s['checkpoint'])
    assert saved['origin'] == 'fresh_command' and saved['snapshot'] == before
    controller.comparison_command(s['args'], s['log'])
    assert s['calls'] == [s['args']]
    assert controller._comparison_snapshot(s['args']) == before


def test_legacy_loaded_controller_event_requires_deterministic_three_file_reproduction(setup):
    s = setup
    original = s['finalized'](s['args'])
    s['emit'](s['args'], s['log'])
    before = [p.read_bytes() for p in original]
    controller.comparison_command(s['args'], s['log'])
    assert len(s['calls']) == 1 and s['calls'][0] != s['args']
    assert [p.read_bytes() for p in original] == before
    assert controller.read(s['checkpoint'])['origin'] == 'legacy_success_rederived'
    assert not list(controller.logs.glob('comparison_recovery_*/comparison.json'))
    controller.comparison_command(s['args'], s['log'])
    assert len(s['calls']) == 1


@pytest.mark.parametrize('event_kind', ['absent', 'failed', 'different_command', 'orphan_finished'])
def test_existing_outputs_without_matching_success_never_skipped_or_overwritten(setup, event_kind):
    s = setup
    outputs = s['finalized'](s['args'])
    before = [p.read_bytes() for p in outputs]
    if event_kind == 'failed':
        s['emit'](s['args'], s['log'], returncode=1)
    elif event_kind == 'different_command':
        s['emit'](s['args'][:-1]+['2'], s['log'])
    elif event_kind == 'orphan_finished':
        controller.record('command_finished', returncode=0, log=str(controller.logs/s['log']))
    with pytest.raises(RuntimeError, match='no matching successful command'):
        controller.comparison_command(s['args'], s['log'])
    assert [p.read_bytes() for p in outputs] == before
    assert not s['calls'] and not s['checkpoint'].exists()


@pytest.mark.parametrize('changed', ['input', 'source', 'output', 'event'])
def test_checkpoint_rejects_changed_inputs_outputs_sources_or_success_log(setup, changed):
    s = setup
    controller.comparison_command(s['args'], s['log'])
    checkpoint_before = s['checkpoint'].read_bytes()
    if changed == 'input':
        (s['root']/'results/records.csv').write_text('changed input')
    elif changed == 'source':
        (s['root']/'results/trend_diagnostics/compare_repaired_dev.py').write_text('changed code')
    elif changed == 'output':
        controller._comparison_outputs(controller._comparison_spec(s['args']))[1].write_text('changed CSV')
    elif changed == 'event':
        controller.events.write_text('')
    with pytest.raises(RuntimeError):
        controller.comparison_command(s['args'], s['log'])
    assert len(s['calls']) == 1
    assert s['checkpoint'].read_bytes() == checkpoint_before


def test_legacy_tampered_csv_rejected_and_recovery_failure_evidence_retained(setup):
    s = setup
    outputs = s['finalized'](s['args'])
    outputs[1].write_text('tampered finalized CSV')
    s['emit'](s['args'], s['log'])
    with pytest.raises(RuntimeError, match='does not match a fresh deterministic derivation'):
        controller.comparison_command(s['args'], s['log'])
    assert outputs[1].read_text() == 'tampered finalized CSV'
    assert list(controller.logs.glob('comparison_recovery_*/comparison_pairs.csv'))
    assert not s['checkpoint'].exists()


def test_partial_final_output_is_preserved_and_cannot_be_retried_over_it(setup):
    s = setup
    outputs = s['finalized'](s['args'])
    outputs[2].unlink()
    s['emit'](s['args'], s['log'])
    original = outputs[0].read_bytes()
    with pytest.raises(RuntimeError, match='Incomplete finalized'):
        controller.comparison_command(s['args'], s['log'])
    assert outputs[0].read_bytes() == original
    assert not s['calls'] and not s['checkpoint'].exists()


def test_legacy_final_json_must_match_requested_comparison(setup):
    s = setup
    outputs = s['finalized'](s['args'])
    report = controller.read(outputs[0])
    report['new_comparison'] = 'a_different_comparison'
    outputs[0].write_text(json.dumps(report))
    s['emit'](s['args'], s['log'])
    with pytest.raises(RuntimeError, match='identity does not match'):
        controller.comparison_command(s['args'], s['log'])
    assert not s['calls']


def test_checkpoint_install_is_atomic_and_never_overwrites_existing_evidence(setup, monkeypatch):
    path = setup['checkpoint']
    def fail_link(source, target):
        assert json.loads(Path(source).read_text()) == {'status': 'fully_written'}
        assert not Path(target).exists()
        raise OSError('simulated interruption before atomic install')
    real_link = controller.os.link
    monkeypatch.setattr(controller.os, 'link', fail_link)
    with pytest.raises(OSError, match='simulated interruption'):
        controller._save_comparison_checkpoint(path, {'status': 'fully_written'})
    assert not path.exists()
    assert list(path.parent.glob(path.name+'.pending_*'))  # failed staging evidence retained
    monkeypatch.setattr(controller.os, 'link', real_link)
    controller._save_comparison_checkpoint(path, {'status': 'fully_written'})
    before = path.read_bytes()
    with pytest.raises(FileExistsError):
        controller._save_comparison_checkpoint(path, {'status': 'replacement_forbidden'})
    assert path.read_bytes() == before
