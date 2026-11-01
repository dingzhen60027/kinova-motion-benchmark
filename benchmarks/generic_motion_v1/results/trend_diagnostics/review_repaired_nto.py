"""Read-only SciPy/NumPy review; no robot construction, CUDA or solver calls."""
import json, hashlib, collections, datetime
from pathlib import Path
import numpy as np
from scipy.interpolate import BSpline
ROOT=Path(__file__).resolve().parents[4]
RUN=ROOT/'results/repaired_dev'; OUT=ROOT/'results/trend_diagnostics/nto_repaired_review.json'
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def read(p): return json.loads(p.read_text())
def arrays(p):
 with np.load(p,allow_pickle=False) as a:return {k:a[k] for k in a.files}
def curve(p):
 a=arrays(p);duration=a.get('duration');duration=read(p.with_suffix('.json'))['duration'] if duration is None else duration;return BSpline(a['knots'],np.squeeze(a['controls']),3),float(duration)
def difference(a,b):return float(np.max(np.abs(np.asarray(a)-np.asarray(b))))
manifest=read(RUN/'run_manifest.json');cases={r['problem_id']:r for r in [json.loads(s) for s in (ROOT/'benchmarks/generic_motion_v1/manifests/dev.jsonl').read_text().splitlines()]}
jobs=[j for j in manifest['jobs'] if j['method']=='NTO_REFINED' and j['objective']=='minimum_time']
output=dict(scope='Read-only artifact review of completed NTO_REFINED time attempts; incomplete coverage is not a final benchmark result; NumPy/SciPy only, no CUDA/Drake/solver calls',reviewed_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),source_commit=manifest['source_code_commit'],planned_jobs=len(jobs),run_manifest_sha256=sha(RUN/'run_manifest.json'),checks=[],errors=[],pending_jobs=[])
for job in jobs:
 ps=sorted((RUN/'jobs'/job['job_id']/'attempts').glob('*/receipt.json'))
 if not ps:output['pending_jobs'].append(job['job_id']);continue
 for receipt_path in ps:
  p=receipt_path.parent; receipt=read(receipt_path); result=read(p/'result.json'); inp=read(p/'input.json'); cfg=read(p/'evaluation.json'); errors=[]
  for rel,h in receipt['files'].items():
   if not (p/rel).is_file() or sha(p/rel)!=h:errors.append('receipt mismatch:'+rel)
  frozen=cases[job['case_id']]
  fields=['q_start','q_goal','robot_sha256','collision_geometry_sha256','obstacle_geometry_sha256','voxel_geometry_sha256','scene_sha256','problem_sha256','voxel_radius_m','voxels_xyz','velocity_limits_rad_s','acceleration_limits_rad_s2','endpoint_velocity_rad_s']
  for field in fields:
   if inp.get(field)!=frozen.get(field):errors.append('input differs from frozen manifest:'+field)
  for key,value in [('velocity_limit_rad_s',1.0),('acceleration_limit_rad_s2',1.0),('validation_step_rad',.005),('endpoint_tolerance_rad',1e-5),('online_budget_seconds',60.0)]:
   if cfg[key]!=value:errors.append('changed evaluation '+key)
  if cfg['nto_refined']['minimum_time_control_points']!=16:errors.append('resolved minimum control count not16')
  initial_path=p/'nto_initial.npz' if (p/'nto_initial.npz').is_file() else p/'refinement/round_00/nto_initial.npz'
  check=dict(job_id=job['job_id'],attempt=str(p.relative_to(ROOT)),status=result['status'],receipt_files_checked=len(receipt['files']),fixed_input_and_limits_match=True,optimizer_constructed=initial_path.is_file(),rounds=[])
  if initial_path.is_file():
   initial=arrays(initial_path); original=initial['original_path']; duration=float(initial['duration']); controls=initial['controls']; knots=initial['knots']; expected=max(16,len(original)+2)
   if len(controls)!=expected:errors.append('wrong initial controls')
   if not np.array_equal(original,np.load(p/'path.npy',allow_pickle=False)):errors.append('saved original seed differs from path.npy')
   if difference(original[[0,-1]],[inp['q_start'],inp['q_goal']])>cfg['endpoint_tolerance_rad']:errors.append('seed endpoint mismatch')
   raw_controls=np.vstack([original[0],original,original[-1]])
   raw_knots=np.r_[np.zeros(3),np.linspace(0,duration,len(raw_controls)-2),np.full(3,duration)]
   base=BSpline(raw_knots,raw_controls,3); refined=BSpline(knots,controls,3); t=np.linspace(0,duration,1001)
   derivatives=[difference(base.derivative(i)(t),refined.derivative(i)(t)) for i in range(3)]
   if max(derivatives)>1e-10:errors.append('knot insertion changed seed curve')
   check.update(original_drm_nodes=len(original),initial_controls=len(controls),initial_curve_derivative_max_errors=derivatives,original_path_sha256=sha(p/'path.npy'))
   prior=None
   for round_path in sorted((p/'refinement').glob('round_*')):
    rp=read(round_path/'nto_parameters.json') if (round_path/'nto_parameters.json').exists() else None
    ri=arrays(round_path/'nto_initial.npz') if (round_path/'nto_initial.npz').exists() else None
    rr=dict(round=round_path.name,parameters_present=rp is not None,initial_present=ri is not None,candidate_present=(round_path/'candidate.npz').is_file())
    if rp is not None:
     if rp['minimum_time_control_points']!=16 or rp['warm_start_control_count']!=expected:errors.append('round control policy mismatch '+round_path.name)
     if rp['collision_pairs']!=137+15*len(inp['voxels_xyz']):errors.append('collision pair count mismatch')
     rr.update(control_count=rp['warm_start_control_count'],collision_pairs=rp['collision_pairs'],refinement_warm_start=rp['refinement_warm_start'])
    if prior is not None and ri is not None:
     error=max(difference(ri[k],prior[k]) for k in ['controls','knots','duration'])
     rr['previous_candidate_reinitialization_max_error']=error
     if error>1e-12:errors.append('previous round warmstart mismatch '+round_path.name)
    prior=arrays(round_path/'candidate.npz') if rr['candidate_present'] else None
    check['rounds'].append(rr)
  if (p/'candidate.npz').is_file():
   tr,T=curve(p/'candidate.npz'); t=np.linspace(0,T,10001); val=read(p/'validation.json') if (p/'validation.json').exists() else None
   v=tr.derivative();a=tr.derivative(2)
   endpoint_error=difference(tr([0,T]),[inp['q_start'],inp['q_goal']]);vmax=float(np.max(np.abs(v(t))));amax=float(np.max(np.abs(a(t))));vend=float(np.max(np.abs(v([0,T]))))
   check.update(candidate_duration_s=T,candidate_endpoint_error=endpoint_error,candidate_dense_velocity_max=vmax,candidate_dense_acceleration_max=amax,candidate_endpoint_velocity=vend,final_validation_passed=None if val is None else val['passed'])
   if val and val['passed']:
    if endpoint_error>cfg['endpoint_tolerance_rad'] or vmax>1+cfg['feasibility_tolerance'] or amax>1+cfg['feasibility_tolerance'] or vend>cfg['feasibility_tolerance']:errors.append('saved successful candidate violates independent derivative/endpoint reconstruction')
  check['errors']=errors;check['fixed_input_and_limits_match']=not any(e.startswith(('input differs','changed evaluation')) for e in errors);output['checks'].append(check)
  output['errors'].extend({'job_id':job['job_id'],'error':e} for e in errors)
output['completed_jobs']=len({x['job_id'] for x in output['checks']});output['complete']=output['completed_jobs']==len(jobs);output['status_counts_so_far']=dict(collections.Counter(c['status'] for c in output['checks']));output['optimizer_constructed_count']=sum(c['optimizer_constructed'] for c in output['checks']);output['round_count_distribution']=dict(collections.Counter(len(c['rounds']) for c in output['checks'] if c['optimizer_constructed']))
fixture=ROOT/'results/trend_diagnostics/nto_parameterization_control';record=read(fixture/'result.json');rechecked=[]
for run in record['runs']:
 n=run['control_count'];tr,T=curve(fixture/f'candidate_{n}.npz'); t=np.linspace(0,T,10001);v=tr.derivative();a=tr.derivative(2); vc=np.abs(v.c[:len(v.t)-v.k-1]);ac=np.abs(a.c[:len(a.t)-a.k-1]);metrics={'duration_s':T,'max_velocity_dense':float(np.abs(v(t)).max()),'max_acceleration_dense':float(np.abs(a(t)).max()),'max_velocity_control_hull':float(vc.max()),'max_acceleration_control_hull':float(ac.max()),'endpoint_velocity':float(np.abs(v([0,T])).max())};d={k:abs(metrics[k]-run[k]) for k in metrics}
 rechecked.append(dict(controls=n,arrays_sha256=sha(fixture/f'candidate_{n}.npz'),recomputed=metrics,max_recorded_metric_error=max(d.values()),endpoint_displacement_rad=(tr(T)-tr(0)).tolist()))
 if max(d.values())>1e-12:output['errors'].append({'fixture':n,'error':'fixture metrics disagree with arrays'})
output['analytical_fixture']=dict(source_result_sha256=sha(fixture/'result.json'),scope=record['scope'],continuous_rest_to_rest_lower_bound_s=record['continuous_rest_to_rest_lower_bound_s'],runs=rechecked)
output['source_file_hash_checks']={}
for rel in ['benchmarks/generic_motion_v1/model.py','benchmarks/generic_motion_v1/runners/paper_nto.py','benchmarks/generic_motion_v1/runners/paper_nto_refined.py','benchmarks/generic_motion_v1/runners/validate.py']:
 observed=sha(ROOT/rel);expected=manifest['source_hashes'][rel];output['source_file_hash_checks'][rel]={'actual':observed,'frozen':expected,'matches':observed==expected}
 if observed!=expected:output['errors'].append({'source_file':rel,'error':'runtime source differs from frozen manifest'})
output['integrity_passed']=not output['errors'];OUT.write_text(json.dumps(output,indent=2)+'\n');print(json.dumps({k:output[k] for k in ['completed_jobs','planned_jobs','complete','status_counts_so_far','optimizer_constructed_count','round_count_distribution','integrity_passed','errors']},indent=2))
