"""Run the revised paper experiments serially and preserve command outcomes."""
import argparse
import datetime
from pathlib import Path
import subprocess
import sys
import time
from reproduction.common import ROOT, atomic


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--environments', nargs='+', choices=['forest', 'fr3'], default=['forest', 'fr3'])
    parser.add_argument('--pilot', action='store_true')
    args = parser.parse_args()
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    label = 'pilot' if args.pilot else 'full'
    ledger = ROOT / 'reproduction/reports/paper_constructed' / f'{stamp}_{label}_commands.json'
    rows = []
    for environment in args.environments:
        cmd = ['bash', 'local/python.sh', '-m', 'reproduction.paper_constructed.run', 'run', '--environment', environment]
        if args.pilot:
            cmd += ['--environments', '0', '--roadmap-seeds', '0', '--sizes', '200' if environment == 'forest' else '3000']
        log = ROOT / 'reproduction/logs' / f'{stamp}_constructed_{label}_{environment}.log'
        row = dict(environment=environment, command=cmd, status='running', started_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(), log=str(log.relative_to(ROOT)))
        rows.append(row); atomic(ledger, rows)
        start = time.perf_counter()
        try:
            with log.open('w') as stream:
                process = subprocess.run(cmd, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT)
            row.update(exit_code=process.returncode, status='completed' if process.returncode == 0 else 'failed')
        except KeyboardInterrupt:
            row.update(status='interrupted', exit_code=-2)
            raise
        finally:
            row.update(elapsed_seconds=time.perf_counter()-start, completed_utc=datetime.datetime.now(datetime.timezone.utc).isoformat())
            for line in log.read_text().splitlines():
                if line.startswith('RUN_DIR='):
                    row['run_dir'] = line.split('=',1)[1]
            atomic(ledger, rows)
            print(row, flush=True)
        if row['exit_code']:
            return row['exit_code']
    print('LEDGER=' + str(ledger), flush=True)
    return 0

if __name__ == '__main__':
    raise SystemExit(main())
