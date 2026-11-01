"""Run a declared failed-dev cohort with native graph queries and a real cap."""
import argparse
import collections
import json
from pathlib import Path
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT))
from benchmarks.generic_motion_v1.roadmap import write


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--build',type=Path,required=True)
    p.add_argument('--attempts',type=int,default=1000)
    p.add_argument('--endpoint-trials',type=int,default=50)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    cohort=[r for r in json.loads(a.source.read_text()) if r['status']!='success_numerical']
    write(a.output/'cohort.json',dict(source=str(a.source),scope='All failures of the declared development frontend cohort; this conditional diagnostic is not a full benchmark score',
          problem_ids=[r['problem_id'] for r in cohort],attempts=a.attempts,endpoint_trials=a.endpoint_trials,wall_cap_seconds=60))
    rows=[]
    for previous in cohort:
        pid=previous['problem_id'];folder=a.output/pid;folder.mkdir(exist_ok=True)
        if not (folder/'record.json').exists():
            command=['flock',str(ROOT/'results/.gpu.lock'),'timeout','--signal=TERM','--kill-after=5','60',
                     'bash','local/python.sh','results/trend_diagnostics/drm_query_regression.py',
                     '--build',str(a.build),'--output',str(a.output),'--native-defaults',
                     '--attempts',str(a.attempts),'--endpoint-trials',str(a.endpoint_trials),
                     '--problem',pid,'--external-gpu-lock']
            write(folder/'command.json',dict(argv=command,cwd=str(ROOT),timing='Timeout starts after GPU lock acquisition; includes process startup, model, query, validation and diagnostic graph checks'))
            with (folder/'process.log').open('w') as log:
                process=subprocess.run(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
            write(folder/'process.json',dict(returncode=process.returncode))
            if not (folder/'record.json').exists():
                write(folder/'record.json',dict(problem_id=pid,scene_id=previous['scene_id'],category=previous['category'],
                    status='diagnostic_timeout' if process.returncode in (124,137) else 'infrastructure_error',
                    returncode=process.returncode,query_result_available=False))
        row=json.loads((folder/'record.json').read_text());rows.append(row)
        print(json.dumps({k:row[k] for k in ('problem_id','status')}),flush=True)
        write(a.output/'records.json',rows)
    write(a.output/'summary.json',dict(scope='Conditional failure diagnosis; never pooled as best-of with the source cohort',
          requested=len(cohort),rows=len(rows),status_counts=dict(collections.Counter(r['status'] for r in rows)),
          native_attempt_cap=a.attempts,endpoint_trial_cap=a.endpoint_trials,wall_cap_seconds=60,
          heuristic_free_nodes_with_unblocked_free_neighbor=sum(len(r.get('diagnostic_heuristic_nodes_with_free_unblocked_neighbor',[])) for r in rows)))


if __name__=='__main__':main()
