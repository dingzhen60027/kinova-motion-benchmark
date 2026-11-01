"""Read-only low-cost monitor of saved NTO backend events; no planner, solver or GPU calls."""
from __future__ import annotations
import argparse,collections,json,hashlib,time
from datetime import datetime,timezone
from pathlib import Path
ROOT=Path(__file__).resolve().parents[4]
NTO={'NTO','NTO_PAPER','NTO_REFINED'}

def read(path):
 try:return json.loads(path.read_text())
 except (FileNotFoundError,json.JSONDecodeError):return None

def review(run):
 manifest=read(run/'run_manifest.json');jobs=[j for j in manifest['jobs'] if j['method'] in NTO];rows=[];blockers=[]
 for job in jobs:
  attempts=sorted((run/'jobs'/job['job_id']/'attempts').glob('*'))
  if not attempts:continue
  for attempt in attempts:
   process=read(attempt/'process.json');result=read(attempt/'result.json') or {};terminal=process is not None
   item=dict(job_id=job['job_id'],method=job['method'],objective=job['objective'],attempt=str(attempt.relative_to(ROOT)),terminal=terminal,status=result.get('status'),process=process,timing=result.get('timing',{}),backend_rounds=[])
   if result.get('exception'):item['exception']=result['exception']
   validation=read(attempt/'validation.json')
   if validation:item['validation']={k:validation.get(k) for k in ('passed','status','samples','independent_collisions','native_collisions','minimum_sampled_clearance_m','endpoint_error_rad','velocity_violation','acceleration_violation')}
   paths=sorted(attempt.glob('refinement/round_*/nto_collision_backend.json'))
   if not paths and (attempt/'nto_collision_backend.json').exists():paths=[attempt/'nto_collision_backend.json']
   for p in paths:
    b=read(p)
    if b is None:continue
    events=b.get('recovery_events',[])
    record=dict(path=str(p.relative_to(ROOT)),native_evaluations=b.get('native_evaluations'),retained_collision_pairs=b.get('collision_pairs'),dropped_collision_pairs=b.get('collision_pairs_dropped'),recovery_count=len(events),recovery_seconds=sum(e.get('seconds',0.) for e in events),recovery_failures=[e.get('recovery_failure') for e in events if e.get('recovery_failure')],recovered_pair_distances_m=[v.get('signed_distance_m') for e in events for v in e.get('recovered_pairs',[])])
    item['backend_rounds'].append(record)
    if record['recovery_failures']:blockers.append(dict(job_id=job['job_id'],reason='recovery_rejection',events=record['recovery_failures']))
    if b.get('model_changed') or b.get('collision_pairs_dropped',0):blockers.append(dict(job_id=job['job_id'],reason='model_or_pair_contract_violation'))
   if terminal:
    for name in ['stderr.log','nto_failure.json','optimizer_failure.json']:
     p=attempt/name
     if not p.exists():continue
     text=p.read_text(errors='replace')
     if any(token in text for token in ['BoxBoxGradient','Collision constraint expects','Nonfinite gradient in BoxBoxGradient','BoxBoxGradient recovery rejected','Collision constraint AutoDiff']):
      blockers.append(dict(job_id=job['job_id'],reason='backend_error_in_terminal_log',path=str(p.relative_to(ROOT)),tail=text[-3000:]))
   rows.append(item)
 completed={r['job_id'] for r in rows if r['terminal']}
 return dict(created_utc=datetime.now(timezone.utc).isoformat(),scope=__doc__,run=str(run.relative_to(ROOT)),source_commit=manifest['source_code_commit'],manifest_sha256=manifest['manifest_sha256'],gpu_calls=0,solver_calls=0,planned_nto_jobs=len(jobs),completed_nto_jobs=len(completed),all_nto_jobs_terminal=len(completed)==len(jobs),terminal_status_counts=dict(collections.Counter(r['status'] for r in rows if r['terminal'])),total_recovery_events=sum(b['recovery_count'] for r in rows for b in r['backend_rounds']),total_recovery_seconds=sum(b['recovery_seconds'] for r in rows for b in r['backend_rounds']),blockers=blockers,rows=rows,notes=['Refined top-level copy is not double-counted; each native round is examined once.','Native evaluation count may remain initialization0 if external watchdog killed a solve before final save; this is incomplete telemetry, not zero work.','Numerical collision/solver failures and true timeouts are preserved, not classified as backend implementation bugs.','No final aggregate success claim while jobs are incomplete.'])

def main():
 parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--run',type=Path,required=True);parser.add_argument('--output',type=Path,required=True);parser.add_argument('--watch',action='store_true');args=parser.parse_args()
 run=args.run.resolve();last=None
 while True:
  result=review(run);args.output.write_text(json.dumps(result,indent=2)+'\n')
  status=(result['completed_nto_jobs'],len(result['blockers']),result['total_recovery_events'])
  if status!=last:print(json.dumps({k:result[k] for k in ['created_utc','completed_nto_jobs','planned_nto_jobs','terminal_status_counts','total_recovery_events','total_recovery_seconds','blockers']}),flush=True);last=status
  if not args.watch or result['all_nto_jobs_terminal'] or result['blockers']:break
  time.sleep(30)
if __name__=='__main__':main()
