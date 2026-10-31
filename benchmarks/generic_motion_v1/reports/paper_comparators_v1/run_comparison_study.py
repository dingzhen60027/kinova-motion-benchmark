"""Freeze and execute all declared smoke/dev slots with one GPU worker.

Runs original published-profile NTO, with no refinement. No test planner is invoked. Every subprocess and incomplete action is retained.
Installed Conda dependencies, optional solver files and the CPU sidecar are
checked before and after each immutable run block.
"""
import argparse
from datetime import datetime, timezone
from importlib import metadata
from pathlib import Path
import platform
import subprocess
import sys
import time
import json

from ...common import BASE, ROOT, read, write, sha, cases, code_inventory, head
from ...batch import verify
from ...roadmap import load_config
from ...study_protocol import _dataset_files, _roadmap_files

OUT = Path(__file__).resolve().parent
CONFIG = BASE / 'configs/rss2025_reliable.json'
METHODS = ['NTO_PAPER']
LABELS = ['NTO-distance', 'NTO-time']


def stamp():
    return datetime.now(timezone.utc).isoformat()


def dependency_files(config):
    from ...runners.paper_lgcs_scip import dependency_receipt
    files = dependency_receipt()
    for name in ('csdecomp', 'drake', 'scsplanning', 'pybezier', 'clarabel', 'gurobipy', 'numpy', 'scipy'):
        distribution = metadata.distribution(name)
        for item in distribution.files or []:
            path = Path(distribution.locate_file(item)).resolve()
            if path.is_file() and (path.suffix == '.py' or '.so' in path.name):
                files[str(path)] = sha(path)
    build = ROOT / config['drm_query']['build_dir']
    for path in build.iterdir():
        if path.is_file():
            files[str(path.resolve())] = sha(path)
    return files


def freeze():
    target = OUT / 'execution_freeze.json'
    if target.exists():
        raise FileExistsError('Keep the original execution freeze; use it to resume')
    config = load_config(CONFIG)
    declaration = read(OUT / 'development_protocol.json')
    reused = {}
    for name in declaration['comparison']['reused_runs']:
        run = ROOT / name
        check = verify(run)
        previous = read(run / 'run_manifest.json')
        if (not check['integrity_passed'] or not check['complete']
                or previous['source_hashes'] != code_inventory()
                or previous['timeout_seconds'] != 60.
                or any(job['evaluation_config'] != config for job in previous['jobs'])):
            raise ValueError(f'Reused evidence is incomplete or uses a different runtime: {run}')
        reused[str(run)] = dict(manifest_sha256=sha(run/'run_manifest.json'),
                               jobs=len(previous['jobs']), verification=check)
    files = {}
    blocks = []
    for split, seeds in [('smoke', [0]), ('dev', [0, 1, 2])]:
        manifest = BASE / 'manifests' / (split + '.jsonl')
        data = cases(manifest)
        assert len(data) == (10 if split == 'smoke' else 60)
        assert all(c['split'] == split for c in data)
        files.update(_dataset_files(manifest, data))
        for seed in seeds:
            blocks.append(dict(name=f'{split}_s{seed:03d}', split=split,
                manifest=str(manifest), roadmap=str(BASE/f'roadmaps/rss2025_n6000_s{seed:03d}'),
                run=str(BASE/f'results/rss2025_paper_comparators_v1_{split}_s{seed:03d}'),
                planned_jobs=len(data)*2, registered_inputs=len(data),
                legal_inputs=sum(c['validation_status']=='valid_input' for c in data)))
    assert sum(b['planned_jobs'] for b in blocks) == 380
    frozen = dict(frozen_utc=stamp(), source_commit=head(), runtime_source_hashes=code_inventory(),
        declaration_sha256=sha(OUT/'development_protocol.json'),
        execution_script_sha256=sha(__file__), evaluation_config=config,
        config_file_sha256=sha(CONFIG), data_files=files,
        roadmap_files=_roadmap_files(sorted({Path(b['roadmap']) for b in blocks})),
        external_dependency_files=dependency_files(config),
        methods=METHODS, expected_labels=LABELS, timeout_seconds=60., gpu_workers=1,
        blocks=blocks, planned_registered_slots=380, test_planner_calls=0,
        environment=dict(python=sys.version, executable=sys.executable, platform=platform.platform(),
            packages={name: metadata.version(name) for name in
                ('csdecomp','drake','scsplanning','pybezier','clarabel','gurobipy','numpy','scipy')},
            gpu=subprocess.check_output(['nvidia-smi','--query-gpu=name,driver_version,memory.total',
                                         '--format=csv,noheader'],text=True).strip()),
        declaration=declaration, reused_evidence=reused)
    write(target,frozen)
    print(f'Frozen {len(blocks)} blocks / 380 slots; no planner called',flush=True)


def check_frozen(frozen):
    if code_inventory() != frozen['runtime_source_hashes']:
        raise ValueError('Runtime changed after freeze; do not mix implementations')
    if sha(__file__) != frozen['execution_script_sha256']:
        raise ValueError('Execution script changed after freeze')
    if sha(OUT/'development_protocol.json') != frozen['declaration_sha256']:
        raise ValueError('Declared study changed after freeze')
    if sha(CONFIG) != frozen['config_file_sha256']:
        raise ValueError('Evaluation configuration changed after freeze')
    for run, evidence in frozen.get('reused_evidence', {}).items():
        if sha(Path(run)/'run_manifest.json') != evidence['manifest_sha256']:
            raise ValueError('Reused run identity changed')
    for key in ('data_files', 'roadmap_files', 'external_dependency_files'):
        changed = [path for path, digest in frozen[key].items()
                   if not Path(path).is_file() or sha(path) != digest]
        if changed:
            write(OUT/'changed_dependencies.json',dict(kind=key,changed=changed,utc=stamp()))
            raise ValueError(f'Frozen {key} changed')


def check_existing_run(manifest, block, frozen):
    """Check the entire requested matrix, not just its number of entries."""
    if (manifest['source_hashes']!=frozen['runtime_source_hashes'] or
            len(manifest['jobs'])!=block['planned_jobs'] or manifest['split']!=block['split'] or
            manifest['timeout_seconds']!=frozen['timeout_seconds'] or
            Path(manifest['manifest_path']).resolve()!=Path(block['manifest']).resolve() or
            manifest['manifest_sha256']!=sha(block['manifest']) or
            any(j['evaluation_config']!=frozen['evaluation_config'] for j in manifest['jobs'])):
        raise ValueError('Existing run differs from the execution freeze')
    data=cases(block['manifest'])
    roadmap=read(Path(block['roadmap'])/'metadata.json')
    seed=int(roadmap['seed'])
    expected={}
    for case in data:
        pairing=next(p for p in case['paired_seeds'] if p['drm_seed']==seed)
        for method, objective in frozen['declaration']['methods']:
            key=(case['problem_id'],method,objective)
            expected[key]=(case['scene_id'],case['family'],str(Path(block['roadmap']).resolve()),
                roadmap['roadmap_id'],pairing['method_seed'],pairing['solver_seed'],case)
    observed={}
    for job in manifest['jobs']:
        key=(job['case_id'],job['method'],job['objective'])
        if key in observed:
            raise ValueError('Repeated job in an existing run')
        observed[key]=(job['scene_id'],job['family'],str(Path(job['roadmap_dir']).resolve()),
            job['roadmap_id'],job['method_seed'],job['solver_seed'],job['case'])
    if observed!=expected:
        raise ValueError('Existing run changes a task, method, endpoint, roadmap or seed')


def command(argv, name, action):
    record = dict(argv=argv,cwd=str(ROOT),block=name,action=action,started_utc=stamp())
    ledger = OUT/'commands.jsonl'
    with ledger.open('a') as stream:
        stream.write(json.dumps(dict(record,event='start'))+'\n')
    started=time.perf_counter()
    completed=0
    with (OUT/f'{name}_{action}.log').open('a') as log:
        log.write(json.dumps(record)+'\n');log.flush()
        process=subprocess.Popen(argv,cwd=ROOT,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,bufsize=1)
        for line in process.stdout:
            log.write(line);log.flush()
            try:
                value=json.loads(line)
            except ValueError:
                continue
            if isinstance(value,dict) and 'job_id' in value:
                completed+=1
                write(OUT/'progress.json',dict(block=name,session_completed=completed,
                    last_job=value,updated_utc=stamp()))
        code=process.wait()
    with ledger.open('a') as stream:
        stream.write(json.dumps(dict(record,event='finish',returncode=code,
            finished_utc=stamp(),seconds=time.perf_counter()-started))+'\n')
    if code:
        raise RuntimeError(f'{name}/{action} failed with {code}; see retained log')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--freeze-only',action='store_true')
    parser.add_argument('--prepare-only',action='store_true')
    args=parser.parse_args()
    if args.freeze_only:
        freeze();return
    frozen=read(OUT/'execution_freeze.json')
    check_frozen(frozen)
    index=[]
    for block in frozen['blocks']:
        run=Path(block['run'])
        if not run.exists():
            command([sys.executable,'-m','benchmarks.generic_motion_v1','batch',
                '--manifest',block['manifest'],'--roadmaps',block['roadmap'],'--output',str(run),
                '--methods',*METHODS,'--config',str(CONFIG),'--timeout','60','--prepare-only'],
                block['name'],'prepare')
        manifest=read(run/'run_manifest.json')
        check_existing_run(manifest,block,frozen)
        index.append(dict(block,run_manifest_sha256=sha(run/'run_manifest.json')))
    write(OUT/'run_index.json',dict(blocks=index,slots=380,test_planner_calls=0))
    if args.prepare_only:
        return
    completed=[]
    for block in index:
        check_frozen(frozen)
        command([sys.executable,'-m','benchmarks.generic_motion_v1','resume','--run',block['run']],
                block['name'],'resume')
        check_frozen(frozen)
        check=verify(block['run'])
        if not check['integrity_passed'] or not check['complete']:
            raise RuntimeError(f'Incomplete or corrupt block: {block["name"]}')
        completed.append(dict(block,verification=check))
        write(OUT/'completed_blocks.json',dict(blocks=completed,
            completed_jobs=sum(b['planned_jobs'] for b in completed),total_jobs=380,
            complete=len(completed)==len(index),updated_utc=stamp()))
        print(f'{block["name"]} sealed ({sum(b["planned_jobs"] for b in completed)}/380)',flush=True)
    command([sys.executable,'-m','benchmarks.generic_motion_v1.runners.study_metrics',
        '--runs',*[b['run'] for b in index],'--output',str(OUT/'metrics'),
        '--expected-labels',*LABELS],'all','metrics')


if __name__=='__main__':
    main()
