"""Real OMPL timing regression with CPU-only collision callbacks, not benchmark data."""
import hashlib,json,sys,time
from pathlib import Path
from types import SimpleNamespace
import numpy as np
ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT))
from benchmarks.generic_motion_v1.runners.ompl_baselines import solve,OmplNoExactSolution
from benchmarks.generic_motion_v1.roadmap import load_config,write
out=ROOT/'results/trend_diagnostics/ompl_budget_cpu'
if out.exists():raise FileExistsError('Preserve existing budget timing evidence')
out.mkdir()
config=load_config(ROOT/'configs/evaluation.json')
assert config['ompl']['planning_seconds']==60
model=SimpleNamespace(lower=np.full(7,-1.),upper=np.ones(7),robot_sha256='cpu_fixture',collision_geometry_sha256='cpu_fixture',
  cuda=lambda q: np.abs(q[:,0])>.05,clearance=lambda q:np.abs(q[:,0])-.05)
start=np.zeros(7);goal=start.copy();start[0]=-.5;goal[0]=.5
case=dict(q_start=start,q_goal=goal,_objective='minimum_distance')
began=time.perf_counter();wall_budget=18.
try:
 solve(model,case,config,out,began+wall_budget,173,'RRTSTAR')
 raise AssertionError('Impossible CPU barrier returned exact solution')
except OmplNoExactSolution:pass
wall=time.perf_counter()-began
r=json.loads((out/'ompl_result.json').read_text())
review=dict(scope='Actual OMPL native RRTstar with impossible full-space barrier CPU fixture; no benchmark tasks and no GPU',
 configured_planning_seconds=config['ompl']['planning_seconds'],outer_fixture_budget_seconds=wall_budget,
 validation_reserve_seconds=config['ompl']['validation_reserve_seconds'],simplification_reserve_seconds=config['ompl']['simplification_seconds'],
 native_solve_seconds=r['native_solve_seconds'],wall_seconds=wall,failed_as_expected=not r['has_exact_solution'],
 exceeded_old_10_second_cap=r['native_solve_seconds']>11,retained_validation_reserve=wall<wall_budget-config['ompl']['validation_reserve_seconds'],
 gpu_calls=0,source_sha256=hashlib.sha256((ROOT/'benchmarks/generic_motion_v1/runners/ompl_baselines.py').read_bytes()).hexdigest(),
 note='The 18s fixture deadline avoids a 60s CPU-only stress run; actual 60s planning override is used and tested above the former 10s cap.')
write(out/'review.json',review)
print(json.dumps(review,indent=2))
assert review['failed_as_expected'] and review['exceeded_old_10_second_cap'] and review['retained_validation_reserve']
