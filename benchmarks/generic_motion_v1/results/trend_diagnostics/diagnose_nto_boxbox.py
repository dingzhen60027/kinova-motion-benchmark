"""CPU-only replay of immutable paths; external watchdog preserves failed evidence."""
from pathlib import Path
import argparse,hashlib,json,subprocess,sys,time,traceback
import numpy as np
ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT))
from benchmarks.generic_motion_v1.roadmap import write
BASE=ROOT/'results/trend_diagnostics/nto_boxbox_repair'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
 p=argparse.ArgumentParser();p.add_argument('--case',choices=['dev','test','smoke_bottleneck'],default='dev');p.add_argument('--backend',choices=['native','robust','native_first'],default='robust');p.add_argument('--worker',action='store_true');p.add_argument('--pipeline',action='store_true');p.add_argument('--output',type=Path);a=p.parse_args()
 if a.case=='dev':
  old=ROOT/'results/repaired_dev/jobs/dev-route_choice-002-p00-NTO_PAPER-minimum_distance-48b608ce/attempts/0001'
 elif a.case=='smoke_bottleneck':
  old=next((ROOT/'results/smoke/jobs').glob('smoke-bottleneck-000-p00-NTO_PAPER-minimum_time-*'))/'attempts/0001'
 else:
  choices=list((ROOT/'benchmarks/generic_motion_v1/results').glob('*/jobs/test-route_choice-014-p01-NTO_PAPER-minimum_distance*/attempts/0001/input.json'))
  if not choices:raise FileNotFoundError('Saved test regression fixture missing')
  old=sorted(choices)[0].parent
 out=a.output or BASE/(a.case+'_'+a.backend)
 if not a.worker:
  out.mkdir(parents=True,exist_ok=False)
  cmd=[sys.executable,str(Path(__file__).resolve()),'--worker','--case',a.case,'--backend',a.backend,'--output',str(out)]
  if a.pipeline:cmd.append('--pipeline')
  lock=None
  if a.pipeline:
   import fcntl
   lock=(ROOT/'results/.gpu.lock').open('a');fcntl.flock(lock.fileno(),fcntl.LOCK_EX)
  start=time.perf_counter()
  with (out/'worker.log').open('w') as log:
   try:proc=subprocess.run(cmd,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,timeout=62);status={'returncode':proc.returncode,'timeout':False}
   except subprocess.TimeoutExpired:status={'returncode':None,'timeout':True}
  if lock is not None:lock.close()
  status.update(command=cmd,wall_seconds=time.perf_counter()-start);write(out/'process.json',status);print(json.dumps(status,indent=2));return
 from benchmarks.generic_motion_v1.model import Model
 from benchmarks.generic_motion_v1.runners.paper_nto import paper_nto
 from benchmarks.generic_motion_v1.runners.validate import sample_candidate
 import benchmarks.generic_motion_v1.runners.nto_collision_constraint as module
 if not a.pipeline:
  Model.cuda=lambda *args,**kwargs: (_ for _ in ()).throw(AssertionError('GPU forbidden in CPU diagnostic'))
 if a.backend=='native':
  from pydrake.all import MinimumDistanceLowerBoundConstraint
  class Original:
   def __init__(self,native,clearance,influence,out=None):
    self.constraint=MinimumDistanceLowerBoundConstraint(native.plant,clearance,native.plant_context,influence_distance_offset=influence)
    self.original=self.constraint
   def save(self):return {}
  module.RobustMinimumDistanceConstraint=Original
  class NativeOnlySolve:
   def __init__(self,traj,backend,locations,recovery_state=None):
    self.prog=traj.get_mutable_prog()
    for s in locations:traj.AddPathPositionConstraint(backend.original,float(s))
   def solve(self,solver,deadline,*,timeout_option,log_path=None,solver_options=None):
    remaining=deadline-time.perf_counter()
    if remaining<=0:raise TimeoutError('Native-only diagnostic exhausted shared deadline')
    self.prog.SetSolverOption(solver.solver_id(),timeout_option,remaining)
    return solver.Solve(self.prog,None,solver_options)
  module.NativeFirstCollisionSolve=NativeOnlySolve
 elif a.backend=='robust':
  NativeFirst=module.NativeFirstCollisionSolve
  class GuardedOnlySolve(NativeFirst):
   def __init__(self,traj,backend,locations,recovery_state=None):
    state=dict(guarded=True,restarts=0,solver_invocations=0) if recovery_state is None else recovery_state
    state['guarded']=True
    super().__init__(traj,backend,locations,state)
  module.NativeFirstCollisionSolve=GuardedOnlySolve
 case=json.loads((old/'input.json').read_text());cfg=json.loads((old/'evaluation.json').read_text());path=np.load(old/'path.npy');prior=json.loads((old/'result.json').read_text())
 evidence=dict(scope='CPU diagnostic on exact saved input and path, not a fresh end-to-end score; final common GPU validator not run',case=a.case,backend=a.backend,source_attempt=str(old.relative_to(ROOT)),input_sha256=sha(old/'input.json'),path_sha256=sha(old/'path.npy'),source_sha256={str(p.relative_to(ROOT)):sha(p) for p in [Path(__file__).resolve(),ROOT/'benchmarks/generic_motion_v1/runners/nto_collision_constraint.py',ROOT/'benchmarks/generic_motion_v1/runners/paper_nto.py']})
 write(out/'diagnostic.json',evidence);tic=time.perf_counter()
 try:
  if a.pipeline:
   from benchmarks.generic_motion_v1.runners.csdecomp_baseline import run_case
   from benchmarks.generic_motion_v1.common import code_inventory
   request=json.loads((old/'request.json').read_text());before=code_inventory();write(out/'runtime_sources_before.json',before)
   result=run_case(case,request['roadmap_dir'],out,objective=case['_objective'],seed=request['method_seed'],method='NTO_PAPER',config=cfg)
   after=code_inventory();write(out/'runtime_sources_after.json',after)
   evidence.update(scope='Actual end-to-end NTO_PAPER query with common CUDA and CPU validator, serialized by results/.gpu.lock',status=result['status'],source_unchanged=before==after,metrics=result.get('metrics'),timing=result.get('timing'),exception=result.get('exception'),seconds=time.perf_counter()-tic);write(out/'diagnostic.json',evidence);print(json.dumps(evidence,indent=2));return
  model=Model(case['voxels_xyz'],case['voxel_radius_m']);reserve=sum(prior['timing'].get(k,0.) for k in ['drm_path_seconds','drm_path_validation_seconds','online_collision_map_seconds'])
  candidate=paper_nto(path,model,case,cfg,out,tic+60-reserve);write(out/'candidate.json',candidate)
  np.savez_compressed(out/'candidate.npz',controls=candidate['controls'],knots=candidate['knots'],duration=candidate['duration'])
  evidence.update(outcome='candidate_returned',solver_success=candidate['solver_success'],solver_status=candidate['solver_status'],duration=candidate['duration'])
  sample=sample_candidate(candidate,cfg);clearance=model.clearance(sample['q']);np.savez_compressed(out/'cpu_validation_samples.npz',q=sample['q'],clearance=clearance)
  evidence.update(cpu_minimum_clearance_m=float(clearance.min()),cpu_colliding_samples=int(np.sum(clearance<=model.contact_band_m)),sample_count=len(clearance),endpoint_error_rad=float(np.max(np.abs(sample['q'][[0,-1]]-path[[0,-1]]))))
  evidence['path_length_rad' if case['_objective']=='minimum_distance' else 'trajectory_duration_s']=sample['cost']
 except (Exception,SystemExit) as exc:evidence.update(outcome='exception',exception=repr(exc),traceback=traceback.format_exc())
 evidence['seconds']=time.perf_counter()-tic;write(out/'diagnostic.json',evidence);print(json.dumps(evidence,indent=2))
if __name__=='__main__':main()
