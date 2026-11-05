"""Run real serial checks in the existing Conda environment; retain every log."""
import datetime as dt
import hashlib
import json
from pathlib import Path
import subprocess
import time

ROOT = Path(__file__).resolve().parents[5]
OUT = Path(__file__).resolve().parent / 'core'


def main():
    OUT.mkdir(parents=True, exist_ok=False)
    prefix = ['bash', 'local/python.sh']
    health = ('import json; from pathlib import Path; '
              'from benchmarks.generic_motion_v1.health import check; '
              f'r=check(Path({str(OUT / "health.json")!r})); '
              'print(json.dumps(r, default=str)); '
              'raise SystemExit(0 if r["passed"] else 1)')
    commands = [
        ('pip_check', prefix + ['-m', 'pip', 'check']),
        ('dataset_index', prefix + ['scripts/benchmark.py', 'check']),
        ('regression', prefix + ['-m', 'pytest', '-q']),
        ('upstream', prefix + ['-m', 'pytest', '-q', '--import-mode=prepend',
                              'csdecomp/tests/csdecomp_test.py', 'csdecomp/tests/drake_dep_test.py']),
        ('health', prefix + ['-c', health]),
    ]
    report = dict(started_utc=dt.datetime.now(dt.timezone.utc).isoformat(),
                  source_commit=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
                  scope='Correctness checks only; serial GPU use; no performance comparison or score replacement',
                  commands=[])
    for name, command in commands:
        print(json.dumps(dict(starting=name)), flush=True)
        start = time.perf_counter()
        log = OUT / (name + '.log')
        with log.open('x') as stream:
            result = subprocess.run(command, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT)
        row = dict(name=name, command=command, returncode=result.returncode,
                   seconds=time.perf_counter()-start, log=log.name,
                   log_sha256=hashlib.sha256(log.read_bytes()).hexdigest())
        report['commands'].append(row)
        (OUT / 'summary.json').write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps(row), flush=True)
    report['passed'] = all(c['returncode'] == 0 for c in report['commands'])
    (OUT / 'summary.json').write_text(json.dumps(report, indent=2)+'\n')
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
