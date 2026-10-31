"""Real fixed development regressions after changing only SNOPT log routing."""
import json
from pathlib import Path
import re
import subprocess
import sys

from ...common import BASE, ROOT, read, sha, write
from ...batch import verify

OUT=Path(__file__).resolve().parent
CASES=('dev-bottleneck-000-p00','dev-sparse_clutter-002-p02','dev-route_choice-002-p00')


def main():
    results=[]
    for name in CASES:
        target=BASE/'results'/f'nto_logging_check_v1_{name}'
        argv=[sys.executable,'-m','benchmarks.generic_motion_v1','run',
              '--manifest',str(BASE/'manifests/dev.jsonl'),
              '--roadmaps',str(BASE/'roadmaps/rss2025_n6000_s000'),
              '--config',str(BASE/'configs/rss2025_eizo.json'),'--methods','NTO_PAPER',
              '--objectives','minimum_distance','minimum_time','--problem',name,
              '--timeout','60','--output',str(target)]
        if target.exists():
            raise FileExistsError('Do not replace a development check')
        with (OUT/f'nto_logging_{name}.log').open('w') as stream:
            process=subprocess.run(argv,cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT)
        integrity=verify(target,current_sources=True)
        if process.returncode or not integrity['integrity_passed'] or not integrity['complete']:
            raise RuntimeError(f'Development check did not finish cleanly: {name}')
        manifest=read(target/'run_manifest.json')
        record=dict(command=argv,returncode=process.returncode,run=str(target),integrity=integrity,jobs=[])
        for job in manifest['jobs']:
            attempt=target/'jobs'/job['job_id']/'attempts/0001'
            result=read(attempt/'result.json')
            evidence=dict(job_id=job['job_id'],status=result['status'],attempt=str(attempt))
            path=attempt/'snopt.log'
            if path.exists():
                log=path.read_text()
                evidence.update(log_bytes=path.stat().st_size,log_sha256=sha(path),
                    has_iterations='No. of iterations' in log,
                    has_major_iterations='No. of major iterations' in log,
                    has_termination='SNOPTA INFO' in log,
                    reported_limits={label:(re.search(re.escape(label)+r'\.+\s+(\d+)',log).group(1)
                        if re.search(re.escape(label)+r'\.+\s+(\d+)',log) else None)
                        for label in ('Iteration limit','Major iterations limit','Minor iterations limit')})
                # Backend exceptions or external timeout may leave an incomplete
                # native log; preserve that failure rather than forging a footer.
                if result['status']=='success_validated_numerical' and not all(evidence[k] for k in
                    ('has_iterations','has_major_iterations','has_termination')):
                    raise RuntimeError('Successful native solve lacks complete logging')
            record['jobs'].append(evidence)
        results.append(record)
        write(OUT/'nto_logging_actual_dev_checks.json',dict(results=results,formal_test=False,
            purpose='Validate logging interface on fixed prior development cases; not parameter tuning or new baseline scores.'))
        print(name,[x['status'] for x in record['jobs']],flush=True)


if __name__=='__main__':
    main()
