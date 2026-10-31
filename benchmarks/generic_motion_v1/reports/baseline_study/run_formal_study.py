"""Prepare/resume every block of the frozen study, with one GPU worker.

Records actual subprocess commands and failures. Never replaces completed
attempts, changes a protocol or chooses tasks from observed solver outcomes.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import subprocess
import sys
import time

from ...common import BASE, ROOT, code_inventory, read, sha, write
from ...batch import verify

OUT = Path(__file__).resolve().parent


def stamp():
    return datetime.now(timezone.utc).isoformat()


def check_dependencies(protocol):
    if not protocol.get('external_dependency_files'):
        raise ValueError('Missing installed dependency fingerprint')
    changed=[path for path,digest in protocol['external_dependency_files'].items()
             if not Path(path).is_file() or sha(path)!=digest]
    if changed:
        write(OUT/'formal_dependency_change.json',dict(changed=changed,observed_utc=stamp()))
        raise ValueError('Installed numerical dependencies changed during the study')


def command(argv, block, action):
    record=dict(argv=argv,cwd=str(ROOT),block=block,action=action,started_utc=stamp())
    ledger=OUT/'formal_commands.jsonl'
    with ledger.open('a') as stream:
        stream.write(json.dumps(dict(record,event='start'))+'\n')
    started=time.perf_counter()
    progress=dict(block=block,action=action,started_utc=record['started_utc'],
                  session_completed_jobs=0,last_job=None)
    log_path=OUT/f'formal_{block}_{action}.log'
    with log_path.open('a') as log:
        log.write('\n'+json.dumps(record)+'\n');log.flush()
        process=subprocess.Popen(argv,cwd=ROOT,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,
                                 text=True,bufsize=1)
        for line in process.stdout:
            log.write(line);log.flush()
            try:
                value=json.loads(line)
            except (ValueError,TypeError):
                continue
            if isinstance(value,dict) and 'job_id' in value:
                progress.update(session_completed_jobs=progress['session_completed_jobs']+1,
                                last_job=value,updated_utc=stamp())
                write(OUT/'formal_progress.json',progress)
        code=process.wait()
    with ledger.open('a') as stream:
        stream.write(json.dumps(dict(record,event='finish',returncode=code,
            finished_utc=stamp(),process_seconds=time.perf_counter()-started))+'\n')
    if code:
        write(OUT/'formal_coordinator_failure.json',dict(record,returncode=code,log=str(log_path)))
        raise RuntimeError(f'{block}/{action} failed: {log_path}')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepare-only',action='store_true')
    args=parser.parse_args()
    protocol_path=OUT/'formal_protocol_v1.json'
    protocol=read(protocol_path)
    if code_inventory()!=protocol['runtime_source_hashes']:
        raise ValueError('Runtime differs from preregistered sources')
    if sha(protocol['metrics_contract']['path'])!=protocol['metrics_contract']['sha256']:
        raise ValueError('Metrics contract changed')
    check_dependencies(protocol)
    index=[]
    for name,block in protocol['blocks'].items():
        run=BASE/'results'/f'rss2025_formal_v1_{name}'
        if not run.exists():
            argv=[sys.executable,'-m','benchmarks.generic_motion_v1','batch',
                '--manifest',protocol['manifest']['path'],'--roadmaps',*block['roadmap_dirs'],
                '--output',str(run),'--config',str(BASE/'configs/rss2025_eizo.json'),
                '--methods','EI_ZO','LGCS','NTO_PAPER',
                '--objectives','minimum_distance','minimum_time','--timeout','60',
                '--study-protocol',str(protocol_path),'--study-block',name,'--prepare-only']
            command(argv,name,'prepare')
        check=verify(run,current_sources=True,allow_unsealed=True)
        if not check['integrity_passed']:
            raise ValueError(f'Cannot resume altered/unverified block {name}: {check}')
        manifest=read(run/'run_manifest.json')
        if (manifest['study_protocol']['sha256']!=sha(protocol_path)
                or manifest['study_protocol']['block']!=name
                or len(manifest['jobs'])!=block['planned_jobs']):
            raise ValueError('Prepared run differs from this exact preregistration')
        index.append(dict(block=name,phase=block['phase'],run=str(run),jobs=block['planned_jobs'],
                          run_manifest_sha256=sha(run/'run_manifest.json')))
    assert sum(v['jobs'] for v in index)==9900
    write(OUT/'formal_run_index.json',dict(protocol_sha256=sha(protocol_path),blocks=index,
        registered_job_slots=9900,legal_planner_jobs=9300,input_invalid_slots=600,
        matrix_prepared=True))
    if args.prepare_only:
        print('Prepared 24 immutable blocks, 9900 slots; no planner executed.',flush=True)
        return
    completed=[]
    for item in index:
        check_dependencies(protocol)
        command([sys.executable,'-m','benchmarks.generic_motion_v1','resume','--run',item['run']],
                item['block'],'resume')
        check_dependencies(protocol)
        check=verify(item['run'],current_sources=True)
        if not check['integrity_passed'] or not check['complete']:
            raise ValueError(f"Block incomplete: {item['block']}")
        completed.append(dict(item,verification=check))
        write(OUT/'formal_completed_blocks.json',dict(completed=completed,
            completed_jobs=sum(v['jobs'] for v in completed),total_jobs=9900,
            complete=len(completed)==len(index),updated_utc=stamp()))
        print(f"{item['block']}: complete ({sum(v['jobs'] for v in completed)}/9900 sealed slots)",flush=True)


if __name__=='__main__':
    main()
