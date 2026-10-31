"""Record actual postprocessing commands in a separate append-only ledger."""
import argparse
from datetime import datetime,timezone
import json
from pathlib import Path
import subprocess
import sys
import time
from ...common import ROOT

OUT=Path(__file__).resolve().parent

def main():
    p=argparse.ArgumentParser();p.add_argument('stage',choices=['audit','replays','report']);args=p.parse_args()
    module={'audit':'audit_extended','replays':'make_replays','report':'build_report'}[args.stage]
    argv=[sys.executable,'-m','benchmarks.generic_motion_v1.reports.extended_baselines_v1.'+module]
    ledger=OUT/'postprocess_commands.jsonl'
    previous=[json.loads(line) for line in ledger.read_text().splitlines()] if ledger.exists() else []
    attempt=1+sum(x.get('event')=='start' and x.get('stage')==args.stage for x in previous)
    log=OUT/f'postprocess_{args.stage}_{attempt:02d}.log'
    record=dict(stage=args.stage,attempt=attempt,argv=argv,cwd=str(ROOT),log=str(log),started_utc=datetime.now(timezone.utc).isoformat())
    with ledger.open('a') as stream:stream.write(json.dumps(dict(record,event='start'))+'\n')
    start=time.perf_counter()
    with log.open('x') as stream:
        result=subprocess.run(argv,cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT)
    with ledger.open('a') as stream:stream.write(json.dumps(dict(record,event='finish',returncode=result.returncode,seconds=time.perf_counter()-start,finished_utc=datetime.now(timezone.utc).isoformat()))+'\n')
    print(json.dumps(dict(record,returncode=result.returncode)))
    if result.returncode:raise SystemExit(result.returncode)

if __name__=='__main__':main()
