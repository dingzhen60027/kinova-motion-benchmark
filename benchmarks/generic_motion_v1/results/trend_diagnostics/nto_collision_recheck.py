import csv,json,sys,time,hashlib
from pathlib import Path
import numpy as np
ROOT=Path('/home/eric/wjg/wd/csdecomp')
sys.path.insert(0,str(ROOT))
from pydrake.all import BsplineBasis,BsplineTrajectory,MinimumDistanceLowerBoundConstraint
from benchmarks.generic_motion_v1.model import Model
from benchmarks.generic_motion_v1.runners.validate import sample_candidate
rows=list(csv.DictReader(open(ROOT/'results/records.csv')))
output={'scope':'New CPU-only checking of saved paired dev curves; no GPU or planning run; no data or settings changed','source_head':'856939a029b001b58ace555343226fe6a9a9dabc','cases':[],'counts':[]}
for comparison in ['current_dev','current_test']:
 for method in ['NTO_PAPER','NTO_REFINED','NTO','EI_ZO']:
  for obj in ['minimum_distance','minimum_time']:
   rr=[r for r in rows if comparison in json.loads(r['selected_comparisons']) and r['method']==method and r['objective']==obj and r['input_valid']=='True']
   from collections import Counter
   output['counts'].append(dict(comparison=comparison,method=method,objective=obj,legal=len(rr),statuses=dict(Counter(r['status'] for r in rr)),failure_stages=dict(Counter(r['failure_stage'] for r in rr if r['status']!='success_numerical'))))
for method in ['NTO_PAPER','NTO_REFINED']:
 row=next(r for r in rows if 'current_dev' in json.loads(r['selected_comparisons']) and r['problem_id']=='dev-sparse_clutter-002-p01' and r['replica_seed']=='0' and r['objective']=='minimum_distance' and r['method']==method)
 out=ROOT/row['attempt_path']; case=json.loads((out/'input.json').read_text()); params=json.loads((out/'nto_parameters.json').read_text())
 candidate=json.loads((out/'candidate.json').read_text())
 with np.load(out/'candidate.npz') as a:
  candidate.update({k:a[k] for k in a.files})
 model=Model(case['voxels_xyz']); native=model.nto_plant()
 curve=BsplineTrajectory(BsplineBasis(int(candidate['spline_order']),candidate['knots'].tolist()),[q[:,None] for q in candidate['controls']])
 loc=np.asarray(params['collocation_locations']); q=curve.vector_values(loc*float(candidate['duration'])).T
 tic=time.perf_counter(); collocation_clearance=model.clearance(q)
 constraint=MinimumDistanceLowerBoundConstraint(native.plant,params['profile']['clearance_m'],native.plant_context,influence_distance_offset=params['profile']['influence_distance_m'])
 residual=np.asarray([constraint.Eval(p)[0] for p in q])
 samples=sample_candidate(candidate); clearance=model.clearance(samples['q'])
 saved=json.loads((out/'validation.json').read_text())
 evidence=dict(method=method,problem_id=case['problem_id'],replica_seed=0,attempt_path=row['attempt_path'],input_sha256=hashlib.sha256((out/'input.json').read_bytes()).hexdigest(),initial_path_sha256=hashlib.sha256((out/'path.npy').read_bytes()).hexdigest(),saved_status=row['status'],solver=json.loads((out/'nto_solver_result.json').read_text()),collocation_points=len(loc),collocation_minimum_clearance_m=float(collocation_clearance.min()),collocation_maximum_aggregated_constraint=float(residual.max()),collocation_constraint_upper_bound=float(constraint.upper_bound()[0]),dense_samples=len(clearance),new_dense_minimum_clearance_m=float(clearance.min()),saved_dense_minimum_clearance_m=saved['minimum_sampled_clearance_m'],new_dense_collision_samples=int(np.sum(clearance<=0)),saved_dense_collision_samples=saved['independent_collisions'],saved_validated_path_length_rad=saved['path_length_rad'],new_path_length_rad=float(samples['cost']),elapsed_cpu_check_seconds=time.perf_counter()-tic)
 output['cases'].append(evidence)
print(json.dumps(output,indent=2))
Path('/tmp/nto_independent_recheck_20261101.json').write_text(json.dumps(output,indent=2)+'\n')
