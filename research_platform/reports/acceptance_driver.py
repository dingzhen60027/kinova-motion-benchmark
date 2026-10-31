"""Small, real acceptance workflow; isolated runs and a complete command ledger.

Run: bash local/python.sh -m research_platform.reports.acceptance_driver
Use a new --label for each execution.
This is acceptance evidence, not a paper benchmark or a statistical evaluation.
"""
import argparse
import json
from pathlib import Path
import subprocess
import time

from research_platform.storage import ROOT, atomic, inventory, read, read_result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--label', default='platform_final_v3')
    args = parser.parse_args()
    if not args.label.replace('_', '').replace('-', '').isalnum():
        raise ValueError('Use an alphanumeric output label')
    evidence = ROOT / 'research_platform/reports/acceptance_v3' / args.label
    evidence.mkdir(parents=True, exist_ok=False)
    ledger, checks = [], []

    def command(*argv, expected=0):
        cmd = ['bash', 'local/research.sh', *map(str, argv)]
        number = len(ledger) + 1
        started = time.monotonic()
        with (evidence / f'{number:02d}.stdout.log').open('x') as out, (evidence / f'{number:02d}.stderr.log').open('x') as err:
            p = subprocess.run(cmd, cwd=ROOT, stdout=out, stderr=err)
        ledger.append(dict(argv=cmd, exit_code=p.returncode, expected_exit_code=expected,
                           wall_seconds=time.monotonic()-started))
        atomic(evidence / 'commands.json', ledger)
        print(number, ' '.join(cmd), '->', p.returncode, flush=True)
        if p.returncode != expected:
            raise RuntimeError(f'Unexpected command exit: {ledger[-1]}')
        return json.loads((evidence / f'{number:02d}.stdout.log').read_text()) if argv[0] == 'verify' else None

    paths = {name: ROOT / 'runs' / f'{args.label}_{name}' for name in ['forest', 'fr3', 'batch']}
    try:
        for name, config in [('forest', 'forest_smoke'), ('fr3', 'fr3_smoke')]:
            command('run', '--config', f'configs/research/{config}.json', '--output', paths[name])
            verified = command('verify', '--run', paths[name])
            checks.append(dict(check=f'{name}_complete', passed=verified['complete'], all_valid=verified['all_valid']))
        batch = paths['batch']
        command('run', '--config', 'configs/research/forest_batch.json', '--output', batch, '--prepare-only')
        before = command('verify', '--run', batch, expected=2)
        assert before['missing_jobs'] and not before['complete']
        command('resume', '--run', batch, '--case', 'e000_r000_n00200')
        partial = command('verify', '--run', batch, expected=2)
        summary = read(batch / 'reports/summary.json')
        frozen = inventory(batch / 'jobs')
        assert partial['missing_jobs'] and not partial['complete']
        atomic(evidence / 'partial_verify.json', partial)
        atomic(evidence / 'partial_summary.json', summary)
        command('resume', '--run', batch)
        complete = command('verify', '--run', batch)
        assert complete['complete']
        after = inventory(batch / 'jobs')
        assert all(after[k] == value for k, value in frozen.items())
        checks.append(dict(check='single_case_to_batch_preserves_existing_bytes', passed=True,
                           preserved_files=len(frozen), initial_missing=len(before['missing_jobs']),
                           after_single_case_missing=len(partial['missing_jobs']),
                           completed_jobs=complete['planned_jobs']))
        command('resume', '--run', batch)
        assert inventory(batch / 'jobs') == after
        checks.append(dict(check='default_resume_is_noop_for_completed_successes_and_failures', passed=True,
                           unchanged_files=len(after)))
        forest = paths['forest']
        job = 'method-e000_r000_n00200-NTO_distance'
        result = read_result(forest, job)
        if result['status'] != 'success':
            old = inventory(forest / 'jobs')
            command('resume', '--run', forest, '--retry-job', job)
            new = inventory(forest / 'jobs')
            assert all(new[k] == value for k, value in old.items())
            assert read_result(forest, job)['attempt'] == '0002'
            checks.append(dict(check='explicit_retry_preserves_every_prior_attempt', passed=True,
                               preserved_files=len(old), old_status=result['status'],
                               new_status=read_result(forest, job)['status']))
        else:
            checks.append(dict(check='explicit_retry', skipped='NTO_distance unexpectedly succeeded; never force numerical failure'))
        for name, case in [('forest', 'e000_r000_n00200'), ('fr3', 'e000_r000_n03000'), ('batch', 'e002_r000_n00200')]:
            command('verify', '--run', paths[name])
            command('summarize', '--run', paths[name])
            command('replay', '--run', paths[name], '--case', case)
        atomic(evidence / 'checks.json', dict(passed=True, runs={k:str(v) for k,v in paths.items()}, checks=checks))
    except BaseException as exc:
        atomic(evidence / 'checks.json', dict(passed=False, error=repr(exc), checks=checks))
        raise


if __name__ == '__main__':
    main()
