from pathlib import Path
import argparse
import collections
import datetime
import hashlib
import json
import os
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT))
from benchmarks.generic_motion_v1 import batch
from benchmarks.generic_motion_v1.common import read, write, sha
from benchmarks.generic_motion_v1.runners import results

logs = ROOT/'results/logs'
events = logs/'completion_execution.jsonl'

def record(event, **fields):
    row = dict(time_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(), event=event, **fields)
    with events.open('a') as stream:
        stream.write(json.dumps(row, allow_nan=False)+'\n')
    print(json.dumps(row, allow_nan=False), flush=True)

def command(args, log_name):
    argv = [sys.executable, *args]
    record('command_started', argv=argv, log=str(logs/log_name))
    with (logs/log_name).open('a') as stream:
        outcome = subprocess.run(argv, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT)
    record('command_finished', returncode=outcome.returncode, log=str(logs/log_name))
    if outcome.returncode:
        raise RuntimeError('Command failed; original logs retained: '+log_name)


def _resolved(value):
    path = Path(value)
    return (path if path.is_absolute() else ROOT/path).resolve()


def _successful_commands(args, log_name):
    """Old controllers logged argv at start and only log/returncode at finish."""
    expected = [sys.executable, *args]
    active, successful = {}, []
    if not events.is_file():
        return successful
    for number, line in enumerate(events.read_text().splitlines()):
        if not line.strip():
            continue
        event = json.loads(line)
        if event.get('event') == 'command_started':
            active[event['log']] = (number, event)
        elif event.get('event') == 'command_finished':
            started = active.pop(event.get('log'), None)
            if (started and event.get('returncode') == 0 and event.get('log') == str(logs/log_name)
                    and started[1].get('argv') == expected):
                payload = json.dumps([started[1], event], sort_keys=True, separators=(',', ':')).encode()
                successful.append(dict(started_line=started[0], finished_line=number,
                                       event_pair_sha256=hashlib.sha256(payload).hexdigest()))
    return successful


def _comparison_spec(args):
    if args[0] != 'results/trend_diagnostics/compare_repaired_dev.py':
        raise ValueError('Checkpoint support is restricted to the paired comparison command')
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--run', required=True)
    parser.add_argument('--records', default='results/records.csv')
    parser.add_argument('--summary', default='results/summary.csv')
    parser.add_argument('--baseline-comparison', required=True)
    parser.add_argument('--new-comparison', required=True)
    parser.add_argument('--output-prefix', required=True)
    replicas = parser.add_mutually_exclusive_group()
    replicas.add_argument('--replica-seed', type=int, default=0)
    replicas.add_argument('--all-replicas', action='store_true')
    return parser.parse_args(args[1:])


def _comparison_outputs(spec):
    prefix = _resolved(spec.output_prefix)
    return [prefix.with_suffix('.json'), prefix.with_name(prefix.name+'_pairs.csv'),
            prefix.with_name(prefix.name+'_summary.csv')]


def _comparison_snapshot(args):
    """Bind outputs to the exact current inputs named in the successful report."""
    spec = _comparison_spec(args)
    outputs = _comparison_outputs(spec)
    if not all(path.is_file() for path in outputs):
        raise RuntimeError('Incomplete finalized comparison files; original evidence retained')
    report = read(outputs[0])
    run = _resolved(spec.run)
    replica = None if spec.all_replicas else spec.replica_seed
    if (report.get('complete') is not True or report.get('final_statistics_produced') is not True
            or _resolved(report['new_run']) != run
            or report.get('old_comparison') != spec.baseline_comparison
            or report.get('new_comparison') != spec.new_comparison
            or report.get('replica_seed') != replica):
        raise RuntimeError('Final comparison identity does not match the requested command')
    bindings = [(run/'run_manifest.json', report['new_run_manifest_sha256']),
                (_resolved(spec.records), report['old_records_sha256']),
                (_resolved(spec.summary), report['primary_summary_sha256'])]
    bindings.extend((_resolved(group['old_run'])/'run_manifest.json', group['old_manifest_sha256'])
                    for group in report['configuration_audit']['methods'])
    inputs = {}
    for path, expected in bindings:
        digest = sha(path)
        if digest != expected or (str(path) in inputs and inputs[str(path)] != digest):
            raise RuntimeError('Comparison input changed; refusing reuse: '+str(path))
        inputs[str(path)] = digest
    if read(run/'run_manifest.json')['source_code_commit'] != report['new_source_commit']:
        raise RuntimeError('Final comparison source commit does not match its run')
    # Revisions of the derived-statistics implementation also invalidate reuse.
    for relative in ('results/trend_diagnostics/compare_repaired_dev.py',
                     'benchmarks/generic_motion_v1/runners/results.py',
                     'benchmarks/generic_motion_v1/runners/summarize.py',
                     'benchmarks/generic_motion_v1/batch.py',
                     'benchmarks/generic_motion_v1/common.py',
                     'benchmarks/generic_motion_v1/outcomes.py'):
        path = _resolved(relative)
        inputs[str(path)] = sha(path)
    return dict(argv=[sys.executable, *args], inputs_sha256=inputs,
                outputs_sha256={str(path): sha(path) for path in outputs})


def _legacy_comparison_recheck(args, expected):
    """A legacy event has no output hashes: reproduce separately before binding."""
    directory = Path(tempfile.mkdtemp(prefix='comparison_recovery_', dir=logs))
    replay = list(args)
    replay[replay.index('--output-prefix')+1] = str(directory/'comparison')
    recovery_log = directory.name+'.log'
    try:
        command(replay, recovery_log)  # pure CPU comparison, no planning
        regenerated = _comparison_snapshot(replay)
        if regenerated['inputs_sha256'] != expected['inputs_sha256']:
            raise RuntimeError('Comparison inputs changed during legacy recovery')
        if list(regenerated['outputs_sha256'].values()) != list(expected['outputs_sha256'].values()):
            raise RuntimeError('Legacy comparison does not match a fresh deterministic derivation')
    except Exception:
        record('comparison_recovery_rejected', evidence_directory=str(directory), log=str(logs/recovery_log))
        raise  # Keep all diagnostic files and original outputs on failure.
    record('comparison_recovery_verified', evidence_directory=str(directory),
           matching_outputs_sha256=list(expected['outputs_sha256'].values()), log=str(logs/recovery_log))
    # Only proven byte-identical, newly regenerated duplicates are removed.
    for path in _comparison_outputs(_comparison_spec(replay)):
        path.unlink()
    directory.rmdir()


def _save_comparison_checkpoint(checkpoint, payload):
    """Install a fully written checkpoint atomically, without replacing evidence."""
    with tempfile.NamedTemporaryFile(mode='w', prefix=checkpoint.name+'.pending_',
                                     dir=checkpoint.parent, delete=False) as stream:
        staging = Path(stream.name)
        json.dump(payload, stream, indent=2)
        stream.flush()
        os.fsync(stream.fileno())
    os.link(staging, checkpoint)  # Atomic create; an existing checkpoint is never replaced.
    staging.unlink()


def comparison_command(args, log_name):
    """Resume only an event-backed and hash-verified successful comparison."""
    spec = _comparison_spec(args)
    outputs = _comparison_outputs(spec)
    checkpoint = logs/(Path(log_name).stem+'_checkpoint.json')
    exists = any(path.exists() for path in outputs)
    successful = _successful_commands(args, log_name)
    if exists or checkpoint.exists():
        if not successful:
            raise RuntimeError('Comparison files have no matching successful command event; evidence retained')
        snapshot = _comparison_snapshot(args)
        if checkpoint.is_file():
            saved = read(checkpoint)
            if (saved.get('snapshot') != snapshot or saved.get('successful_command') not in successful
                    or saved.get('status') != 'verified_success'):
                raise RuntimeError('Comparison checkpoint or input/output hashes differ; evidence retained')
        else:
            _legacy_comparison_recheck(args, snapshot)
            # Recheck originals after regeneration, before recording acceptance.
            if _comparison_snapshot(args) != snapshot:
                raise RuntimeError('Original comparison changed during recovery; evidence retained')
            _save_comparison_checkpoint(checkpoint, dict(status='verified_success', snapshot=snapshot,
                successful_command=successful[-1], origin='legacy_success_rederived'))
        record('comparison_skipped_verified', argv=snapshot['argv'], checkpoint=str(checkpoint),
               outputs_sha256=snapshot['outputs_sha256'])
        return
    command(args, log_name)
    snapshot = _comparison_snapshot(args)
    successful = _successful_commands(args, log_name)
    if not successful:
        raise RuntimeError('Successful comparison is missing its command event; outputs retained')
    _save_comparison_checkpoint(checkpoint, dict(status='verified_success', snapshot=snapshot,
        successful_command=successful[-1], origin='fresh_command'))
    record('comparison_checkpoint_written', checkpoint=str(checkpoint), outputs_sha256=snapshot['outputs_sha256'])

def complete_run(name):
    run = (ROOT/'results'/name).resolve()
    command(['scripts/evaluate.py','resume','--run',f'results/{name}'], name+'_execution.log')
    checked = batch.verify(run)
    rows, inventory = results.collect([run])
    terminal = [r for r in rows if r['lifecycle']=='terminal']
    states = collections.Counter(r['status'] for r in terminal)
    report = dict(verification=checked, actual_inventory=inventory, terminal_statuses=dict(states))
    write(logs/(name+'_acceptance.json'), report)
    record('run_checked', run=name, verification=checked, terminal_statuses=dict(states))
    if not checked['complete'] or not checked['integrity_passed'] or states.get('infrastructure_error',0):
        raise RuntimeError('Run needs diagnosis; later stages have not started: '+name)

def main():
    for name in ['repaired_native_smoke','repaired_dev_all_seeds','repaired_test']:
        complete_run(name)

    fixed = 'repaired_fixed_initial_regions'
    if not (ROOT/'results'/fixed/'run_manifest.json').exists():
        command(['scripts/evaluate.py','batch','--split','test','--methods','EI_ZO','LGCS','--track','path','--seeds','0','--mode','fixed_initial_regions','--initial-source-run','results/repaired_test','--output',f'results/{fixed}','--prepare-only'], fixed+'_prepare.log')
    complete_run(fixed)
    selection_path = ROOT/'results/selection.json'
    selection = read(selection_path)
    identity = 'current_test_repaired_initial_regions'
    if identity not in {c['id'] for c in selection['comparisons']}:
        selection['comparisons'].append(dict(id=identity,mode='fixed_initial_regions',runs=[dict(path=f'results/{fixed}',methods=['EI_ZO','LGCS'],objectives=['minimum_distance'])],scope='Same receipt-bound initial EI-ZO path, regions and corridor from the complete repaired_test source stage; eligibility never uses final optimizer success. Every eligible pair receives both backends with the unchanged 60-second cold budget including source revalidation, optimization, graph expansion, collision feedback, repair and final validation. Source-stage cost is reported separately.'))
        write(selection_path,selection)
    command(['scripts/evaluate.py','summarize','--selection','results/selection.json','--output','results'], 'completed_summary_export.log')
    command(['results/trend_diagnostics/check_completed_evaluation.py','--comparisons','current_smoke_repaired','current_dev_repaired','current_test_repaired',identity], 'completed_evaluation_acceptance.log')
    for split,name in [('dev','repaired_dev_all_seeds'),('test','repaired_test')]:
        arguments=['results/trend_diagnostics/compare_repaired_dev.py','--run',f'results/{name}','--baseline-comparison',f'current_{split}','--new-comparison',f'current_{split}_repaired','--output-prefix',f'results/trend_diagnostics/completed_{split}_comparison']
        arguments += ['--all-replicas'] if split=='dev' else ['--replica-seed','0']
        comparison_command(arguments, f'completed_{split}_comparison.log')
    command(['results/trend_diagnostics/update_completed_methods.py','--comparisons','current_smoke_repaired','current_dev_repaired','current_test_repaired',identity], 'completed_methods_update.log')
    command(['results/trend_diagnostics/check_metric_bounds.py','--comparisons','current_smoke_repaired','current_dev_repaired','current_test_repaired','--output','results/trend_diagnostics/completed_metric_bounds.json'], 'completed_metric_bounds.log')
    command(['scripts/benchmark.py','index'],'completed_benchmark_index.log')
    command(['scripts/benchmark.py','check'],'completed_benchmark_check.log')
    record('all_stages_complete')


if __name__ == '__main__':
    main()
