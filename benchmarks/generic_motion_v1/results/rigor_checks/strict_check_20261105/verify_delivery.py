"""Read-only delivery check; retain historical indexes and first-attempt outcomes."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[5]
BASE = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
from benchmarks.generic_motion_v1.batch import verify
from benchmarks.generic_motion_v1.runners.results import collect


def read(path):
    return json.loads(Path(path).read_text())


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def check(output):
    if output.exists():
        raise FileExistsError(output)
    checks = {}
    evidence = {}
    for name in ('dataset/review.json', 'statistics/verification.json',
                 'statistics/integration_verification.json', 'core/summary.json',
                 'revalidated_outputs/summary.json'):
        value = read(BASE / name)
        checks[name] = value['passed'] is True
        evidence[name] = sha(BASE / name)
        for relative, expected in value.get('evidence_sha256', {}).items():
            target = (BASE / name).parent / relative
            checks[name + ':' + relative] = target.is_file() and sha(target) == expected
    for name in ('core/regression_final.json', 'core/regression_lifecycle.json'):
        value = read(BASE / name)
        log = BASE / name.replace('.json', '.log')
        checks[name] = value['returncode'] == 0 and sha(log) == value['log_sha256']
        evidence[name] = sha(BASE / name)
    resume = read(BASE / 'core/resume_idempotence.json')
    checks['completed_run_resume_changes_no_attempt'] = (resume['returncode'] == 0
                                                       and resume['attempt_files_unchanged'])
    evidence['core/resume_idempotence.json'] = sha(BASE / 'core/resume_idempotence.json')
    native = read(BASE / 'native_gpu_recheck/comparison.json')
    checks['native_recheck_same_frontend_and_initial_spline'] = (native['frontend_path_arrays_identical']
                                                              and native['initial_spline_arrays_identical'])
    evidence['native_gpu_recheck/comparison.json'] = sha(BASE / 'native_gpu_recheck/comparison.json')
    protected = read(BASE / 'dataset/run/summary.json')['protected_files_before']
    checks['frozen_inputs_unchanged'] = all(sha(ROOT / name) == expected for name, expected in protected.items())
    originals = {'records.csv': '2cb418255861d5daa850b2756425b08f5beaf76aa9fe99d21686c2ebbf31662c',
                 'summary.csv': '20d2b4cb081b5fbf22020f2e574c3c1eb8d15455726fb1f3d47c47a81efb60b1'}
    checks['historical_primary_indexes_unchanged'] = all(sha(ROOT / 'results' / name) == expected
                                                      for name, expected in originals.items())
    runs = []
    for name, count in (('online_smoke', 14), ('native_gpu_recheck', 1), ('lifecycle_smoke', 2)):
        run = BASE / name
        integrity = verify(run)
        rows, inventory = collect([run])
        checks[name] = (integrity['integrity_passed'] and integrity['complete']
                        and integrity['planned_jobs'] == count and len(rows) == count)
        runs.append(dict(run=name, integrity=integrity, inventory=inventory,
                         status_counts=dict(Counter(row['status'] for row in rows)),
                         outcomes=[{key: row.get(key) for key in
                             ('method', 'objective', 'problem_id', 'status', 'objective_value',
                              'objective_unit', 'cold_e2e_seconds', 'attempt_path')}
                                   for row in rows]))
        evidence[name + '/run_manifest.json'] = sha(run / 'run_manifest.json')
    report = dict(
        scope='Correctness diagnostics; no new formal-test optimization, score replacement or best-of selection',
        source_commit=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        checks=checks, checked_scopes_passed=all(checks.values()),
        protected_input_files=len(protected), runs=runs, evidence_sha256=evidence,
        fixes=[
            'Per-run summaries use receipt-verified numerical acceptance rather than solver convergence.',
            'Summaries, verification and resume preserve the first terminal attempt; later interruptions cannot rerun a finished job.',
            'Model audit writes new evidence and never replaces the canonical robot inventory.',
            'Historical control summaries verify their original source/config bytes without requiring current execution sources.'],
        unresolved=[
            'Historical libdrake SIGSEGV root cause remains unknown. Fresh reruns cannot erase that original infrastructure error.',
            'Collision validation is finite numerical sampling, not a mathematical continuous certificate.',
            'Formal test has historical exposure and only seed 0; dev seeds are 0, 1, 2.'],
        conclusion='Use the recorded validated scopes; this audit does not establish universal absence of bugs.',
        script_sha256=sha(__file__))
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    print(json.dumps(dict(checked_scopes_passed=report['checked_scopes_passed'],
                         checks=checks, outcomes=[dict(run=r['run'], status_counts=r['status_counts']) for r in runs])))
    return 0 if report['checked_scopes_passed'] else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    raise SystemExit(check(parser.parse_args().output.resolve()))
