"""Paired DEV-only cap diagnostic using the public serial runner.

Select the lexicographically first seed-0 legacy iteration failure per track
before running either cap. This is failure analysis, not a population score.
No test query is optimized; original runs/configs/results remain unchanged.
"""
import argparse
import copy
import csv
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[4]
BASE = ROOT / 'benchmarks/generic_motion_v1'
sys.path.insert(0, str(ROOT))
from benchmarks.generic_motion_v1.common import read, write
from benchmarks.generic_motion_v1.runners.results import run_status


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def sealed(attempt, name):
    receipt = read(attempt / 'receipt.json')
    path = attempt / name
    if sha(path) != receipt['files'][name]:
        raise ValueError(f'Changed sealed evidence: {path}')
    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    records_path = BASE / 'results/records.csv'
    with records_path.open() as stream:
        rows = [r for r in csv.DictReader(stream) if
                r['run'] == 'benchmarks/generic_motion_v1/results/baseline_dev'
                and r['method'] == 'NTO' and r['status'] == 'solver_failure'
                and r['replica_seed'] == '0']
    manifest_path = BASE / 'results/baseline_dev/run_manifest.json'
    manifest = read(manifest_path)
    jobs = {j['job_id']: j for j in manifest['jobs']}
    selected = []
    for objective in ('minimum_distance', 'minimum_time'):
        for row in sorted(rows, key=lambda r: r['problem_id']):
            if row['objective'] != objective:
                continue
            attempt = ROOT / row['attempt_path']
            result = read(sealed(attempt, 'nto_solver_result.json'))
            if result['solver_status'] == 'SolutionResult.kIterationLimit':
                selected.append(dict(problem_id=row['problem_id'], objective=objective,
                    original_attempt=row['attempt_path'], original_job=jobs[row['job_id']],
                    original_receipt_sha256=sha(attempt / 'receipt.json')))
                break
        else:
            raise ValueError(f'No eligible DEV failure for {objective}')
    report = dict(scope='DEV failure mechanism only; not an unbiased score or timing comparison',
        selection='First lexicographic seed-0 legacy NTO iteration-limit failure per track; selection fixed before new calls',
        caps=[500, 2000], selected=selected, calls=[],
        records_sha256=sha(records_path), original_manifest_sha256=sha(manifest_path),
        script_sha256=sha(__file__),
        source_commit=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        budget_seconds=60, test_optimizer_calls=0, status='running')
    write(out / 'summary.json', report)
    for case in selected:
        track = 'path' if case['objective'] == 'minimum_distance' else 'time'
        for cap in report['caps']:
            label = f'{track}_cap{cap}'
            config = copy.deepcopy(case['original_job']['evaluation_config'])
            config['nto']['maximum_iterations'] = cap
            config_path = out / f'{label}.json'
            write(config_path, config)
            run = out / label
            command = ['bash', str(ROOT / 'local/python.sh'), 'scripts/evaluate.py', 'run',
                '--split', 'dev', '--problem', case['problem_id'], '--methods', 'NTO',
                '--track', track, '--seeds', '0', '--config', str(config_path), '--output', str(run)]
            entry = dict(problem_id=case['problem_id'], objective=case['objective'],
                cap=cap, config_sha256=sha(config_path), command=command, run=str(run), status='running')
            report['calls'].append(entry)
            write(out / 'summary.json', report)
            print(json.dumps(dict(starting=label, problem_id=case['problem_id'])), flush=True)
            with (out / f'{label}.stdout.log').open('w') as stdout, (out / f'{label}.stderr.log').open('w') as stderr:
                completed = subprocess.run(command, cwd=ROOT, stdout=stdout, stderr=stderr)
            entry.update(returncode=completed.returncode, status='completed' if completed.returncode == 0 else 'command_failed')
            if (run / 'run_manifest.json').is_file():
                entry['run_status'] = run_status(run)
            write(out / 'summary.json', report)
            print(json.dumps(entry), flush=True)
    report['original_records_unchanged'] = sha(records_path) == report['records_sha256']
    report['status'] = 'completed' if all(c['returncode'] == 0 for c in report['calls']) else 'completed_with_errors'
    write(out / 'summary.json', report)
    if not report['original_records_unchanged']:
        raise ValueError('Original published records changed')


if __name__ == '__main__':
    main()
