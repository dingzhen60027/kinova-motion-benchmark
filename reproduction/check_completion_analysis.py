"""CPU-only checks of analysis semantics; fixtures are not benchmark results."""
from copy import deepcopy
from pathlib import Path
import tempfile
import numpy as np
from reproduction.common import ROOT,atomic,read,digest
from reproduction.paper_completion.analyze import summarize,source_integrity_issues
from reproduction.paper_completion.run import runtime_versions
from reproduction.audit_paper_completion import check_online_inputs
OUT=ROOT/'reproduction/reports/paper_completion/diagnostics/analysis_checks'
PILOT=ROOT/'reproduction/runs/20261025T092044.332382Z_paper_completion_fr3'
FOREST=ROOT/'reproduction/runs/20261025T092109.664874Z_paper_completion_forest'
checks=[]
m=read(PILOT/'manifest.json')
assert source_integrity_issues(PILOT,m)==[]
checks.append('actual pilot source snapshot, manifest source digest and configuration digest agree')
bad=deepcopy(m);bad['config']['validation_step']*=2
assert 'manifest configuration does not match configuration hash' in source_integrity_issues(PILOT,bad)
checks.append('mutated in-memory config is detected')
bad=deepcopy(m);first=next(iter(bad['source_files']));bad['source_files'][first]='0'*64
assert 'manifest source-file map does not match implementation hash' in source_integrity_issues(PILOT,bad)
checks.append('mutated in-memory source manifest is detected')
case=PILOT/'cases/e00_r00_n03000';scene=read(PILOT/'scenes/e00/scene.json')
inputs=read(case/'input.json');ik=read(case/'online_pose_ik.json');issues=[]
a,b=check_online_inputs('pilot',scene,inputs,ik,issues)
assert not issues,issues
assert b is not None and not np.array_equal(b,inputs['goal'])
checks.append('actual pilot keeps fixed pose and recovers nonzero raw-double to canonical-goal difference')
bad=deepcopy(ik);bad['target_pose'][0][3]+=.1;issues=[]
check_online_inputs('tampered',scene,inputs,bad,issues)
assert any('changed the frozen target pose' in x for x in issues)
checks.append('mutated in-memory IK target is detected')
actual=summarize(FOREST,OUT/'actual_forest_summary')
assert not actual['source_integrity_issues'] and not actual['artifact_issues']
assert actual['terminal_cases']==actual['requested']==400
checks.append('completed real 400-case Forest summary reconciles source and artifacts')
# Explicit synthetic bookkeeping fixture: it exercises the analyst, not a solver.
# A failed solve may still have a CUDA-free trajectory, and an interrupted method
# is an attempted method without a completed timing.
with tempfile.TemporaryDirectory(prefix='csdecomp_analysis_fixture_') as tmp:
 root=Path(tmp); cfg=deepcopy(m['config']);cfg['method_order']=['LSCS','LGCS','SCSTO','NTO_distance','NTO_time']
 entries=[dict(id=f'e00_r{i:02d}_n00200',environment_seed=0,roadmap_seed=i,roadmap_size=200) for i in range(2)]
 manifest=dict(environment='forest',status='completed',config=cfg,config_hash=digest(cfg),source_files={},implementation_hash=digest({}),runtime_versions=runtime_versions(),cases=entries,fidelity='synthetic_analysis_test_only')
 atomic(root/'manifest.json',manifest)
 for index,entry in enumerate(entries):
  dest=root/'cases'/entry['id'];methods={}
  if index==0:
   r=dict(status='optimization_failed',solver_success=False,validation_passed=False,collision_free=True,native_cuda_collision_free=True,independent_collision_free=True,total_seconds=.125,cost=1.0,model_disagreements=0)
   methods['NTO_distance']=r;atomic(dest/'NTO_distance/result.json',r)
  c=dict(**entry,status='completed' if index==0 else 'timeout',implementation_hash=digest({}),roadmap=dict(success=True),drm_validated_success=True,methods=methods)
  if index==1:c['failure_stage']='NTO_time'
  atomic(dest/'case.json',c)
 summary=summarize(root,OUT/'synthetic_bookkeeping_fixture',historical=True)
 nd=next(x for x in summary['methods'] if x['method']=='NTO_distance')
 nt=next(x for x in summary['methods'] if x['method']=='NTO_time')
 assert nd['native_cuda_cfr_given_drm']==.5 and nd['solver_success_and_native_cuda_free_rate_given_drm']==0
 assert nd['solver_failed_but_native_cuda_free']==1
 assert nt['attempted']==1 and nt['attempts_with_completed_timing']==0 and nt['interrupted_attempts_without_timing']==1
 checks.append('synthetic bookkeeping: failed solver plus CUDA-free counts in CFR, not solver conjunction')
 checks.append('synthetic bookkeeping: interrupted method counts as an attempt without fabricated elapsed time')
 atomic(OUT/'synthetic_fixture_definition.json',dict(scope='ANALYSIS UNIT FIXTURE, NOT EXPERIMENTAL RESULTS',manifest=manifest))
atomic(OUT/'verification.json',dict(passed=True,checks=checks,real_forest_method_counts=[dict(method=r['method'],native_cuda_free=r['native_cuda_free_trajectories'],solver_and_cuda_free=r['native_cuda_validated_successes'],recovery_triggered=r['recovery_triggered']) for r in actual['methods']],raw_goal_roundtrip_error=float(np.linalg.norm(b-inputs['goal']))))
print('PASS',len(checks),'CPU-only analysis checks')
