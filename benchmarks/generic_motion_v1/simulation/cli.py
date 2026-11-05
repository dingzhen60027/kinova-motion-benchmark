"""One optional simulator CLI; planning scores and source attempts stay immutable."""
from __future__ import annotations
import argparse
import csv
import json
from pathlib import Path
import traceback

from ..common import ROOT, read, write, sha, recorded_path
from .execution import run, configuration, source_hashes


def path(value):
    value = Path(value)
    return value.resolve() if value.is_absolute() else (ROOT/value).resolve()


def summarize(directory):
    directory = Path(directory)
    manifest = read(directory/'batch_manifest.json')
    records = []
    for job in manifest['jobs']:
        output = directory/'jobs'/job['job_id']/'result.json'
        if output.exists():
            verify(output.parent)
        row = read(output) if output.exists() else dict(status='pending')
        records.append({k:row.get(k,job.get(k,'')) for k in (
            'job_id','problem_id','scene_id','family','method','status','original_planning_status',
            'reference_duration_s','physical_duration_s','wall_seconds','tracking_rmse_rad',
            'maximum_tracking_error_rad','nominal_endpoint_error_rad','settled_endpoint_error_rad',
            'contact_steps','saturation_steps','velocity_excess_rad_s','acceleration_excess_rad_s2')})
        records[-1]['artifact_path'] = str(output.parent.relative_to(ROOT)) if output.parent.is_relative_to(ROOT) else str(output.parent)
    with (directory/'records.csv').open('w',newline='') as handle:
        writer = csv.DictWriter(handle,fieldnames=list(records[0]) if records else ['status'])
        writer.writeheader(); writer.writerows(records)
    counts = {s:sum(r['status']==s for r in records) for s in sorted({r['status'] for r in records})}
    result = dict(planned=len(records),completed=len(records)-counts.get('pending',0),statuses=counts,
                  scope='MuJoCo execution of saved references; not new planning scores or hardware validation')
    write(directory/'summary.json',result)
    return result


def batch(source_run, output, methods, config=None, resume=False):
    output = Path(output)
    if not resume:
        output.mkdir(parents=True, exist_ok=False)
        try:
            from ..batch import _first_terminal_attempt
            original = read(Path(source_run)/'run_manifest.json')
            jobs = []
            for job in original['jobs']:
                if job['method'] not in methods or job['objective']!='minimum_time':
                    continue
                attempts = sorted((Path(source_run)/'jobs'/job['job_id']/'attempts').glob('*'))
                attempt = _first_terminal_attempt(attempts)
                if attempt is None:
                    raise ValueError('Source run has an uncompleted job: '+job['job_id'])
                result = read(attempt/'result.json')
                jobs.append(dict(job_id=job['job_id'],attempt=str(attempt.resolve()),
                                 receipt_sha256=sha(attempt/'receipt.json'),problem_id=result.get('problem_id'),
                                 scene_id=result.get('scene_id'),family=result.get('family'),method=job['method']))
            if not jobs:
                raise ValueError('No matching completed time-track jobs')
            if len({j['job_id'] for j in jobs})!=len(jobs):
                raise ValueError('Duplicate source jobs')
            write(output/'batch_manifest.json',dict(source_run=str(Path(source_run).resolve()),
                  source_manifest_sha256=sha(Path(source_run)/'run_manifest.json'),config=configuration(config),
                  source_hashes=source_hashes(),jobs=jobs))
        except Exception:
            write(output/'preparation_failure.json',dict(status='infrastructure_error',error=traceback.format_exc()))
            (output/'failure.log').write_text(traceback.format_exc())
            raise
    manifest = read(output/'batch_manifest.json')
    pending = [j for j in manifest['jobs'] if not (output/'jobs'/j['job_id']/'result.json').exists()]
    if pending and manifest['source_hashes'] != source_hashes():
        raise ValueError('Simulator source changed; create a new execution cohort')
    if sha(recorded_path(manifest['source_run'])/'run_manifest.json') != manifest['source_manifest_sha256']:
        raise ValueError('Original planning manifest changed')
    for job in manifest['jobs']:
        destination = output/'jobs'/job['job_id']
        if (destination/'result.json').exists():
            verify(destination)
            continue
        if destination.exists():
            raise ValueError('Unsealed simulation directory retained; inspect before continuing: '+str(destination))
        attempt = recorded_path(job['attempt'])
        if sha(attempt/'receipt.json')!=job['receipt_sha256']:
            raise ValueError('Original planning receipt changed')
        result = run(attempt,destination,manifest['config'])
        print(json.dumps(dict(problem=job['problem_id'],method=job['method'],status=result['status'])),flush=True)
    return summarize(output)


def verify(directory):
    directory = Path(directory).resolve()
    if (directory/'batch_manifest.json').is_file():
        manifest=read(directory/'batch_manifest.json')
        completed=0
        for job in manifest['jobs']:
            child=directory/'jobs'/job['job_id']
            if (child/'result.json').is_file():
                verify(child); completed+=1
        return dict(integrity_passed=True,complete=completed==len(manifest['jobs']),
                    completed=completed,planned=len(manifest['jobs']))
    result = read(Path(directory)/'result.json')
    if sha(directory/'result.json')!=read(directory/'receipt.json')['result_sha256']:
        raise ValueError('Simulation result changed')
    for name,digest in result['artifacts'].items():
        file = (Path(directory)/name).resolve()
        if not file.is_relative_to(Path(directory).resolve()) or not file.is_file() or sha(file)!=digest:
            raise ValueError('Simulation evidence changed: '+name)
    model_file=directory/'model/model_metadata.json'
    if model_file.exists():
        metadata=read(model_file)
        for field in ('canonical_source_files_sha256','visual_mesh_files_sha256'):
            for name,digest in metadata[field].items():
                if sha(ROOT/name)!=digest:raise ValueError('Robot asset changed: '+name)
    return dict(integrity_passed=True,status=result['status'],checked_files=len(result['artifacts']))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command',required=True)
    s = sub.add_parser('run')
    s.add_argument('--attempt',required=True); s.add_argument('--output',required=True)
    s.add_argument('--config'); s.add_argument('--render',action='store_true')
    s.add_argument('--disable-control',action='store_true',help='Explicit negative diagnostic, not the normal controller')
    s = sub.add_parser('batch')
    s.add_argument('--source-run',required=True); s.add_argument('--output',required=True)
    s.add_argument('--methods',nargs='+',required=True); s.add_argument('--config')
    for name in ('resume','summarize','verify'):
        s=sub.add_parser(name); s.add_argument('--run',required=True)
    for name in ('render','view'):
        s=sub.add_parser(name); s.add_argument('--run',required=True)
        if name=='render':s.add_argument('--output',required=True)
        s.add_argument('--collision-debug',action='store_true')
    args = parser.parse_args(argv)
    try:
        if args.command=='run':
            result = run(path(args.attempt),path(args.output),path(args.config) if args.config else None,
                         args.render,args.disable_control)
        elif args.command=='batch':
            result = batch(path(args.source_run),path(args.output),args.methods,path(args.config) if args.config else None)
        elif args.command=='resume':
            result = batch(None,path(args.run),None,resume=True)
        elif args.command=='summarize':result = summarize(path(args.run))
        elif args.command=='verify':result = verify(path(args.run))
        else:
            directory=path(args.run)
            verify(directory)
            from .render import render_rollout,play_rollout
            if args.command=='render':
                result=render_rollout(directory/'model/model.xml',directory/'rollout.npz',path(args.output),
                                      camera='overview',collision_debug=args.collision_debug)
            else:
                result=play_rollout(directory/'model/model.xml',directory/'rollout.npz',
                                    camera='overview',collision_debug=args.collision_debug)
        # Per-step arrays and verbose pair maps remain in their evidence files.
        print(json.dumps({k:v for k,v in (result or {}).items() if k not in ('artifacts','simulation_source_hashes','render')},indent=2))
        return 1 if result and result.get('status')=='infrastructure_error' else 0
    except Exception:
        traceback.print_exc()
        return 2
