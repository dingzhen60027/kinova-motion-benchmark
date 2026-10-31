"""Two-second integration probes on the first frozen smoke task; not score rows."""
import json
from pathlib import Path
import sys
import time
from benchmarks.generic_motion_v1.model import Model
from benchmarks.generic_motion_v1.roadmap import HERE, load_config, write
from benchmarks.generic_motion_v1.runners.ompl_baselines import solve, OmplNoExactSolution
from benchmarks.generic_motion_v1.runners.validate import validate
method=sys.argv[1]
case=json.loads((HERE/'manifests/smoke.jsonl').read_text().splitlines()[0])
model=Model(case['voxels_xyz'],case['voxel_radius_m'])
assert model.robot_sha256==case['robot_sha256']
assert model.collision_geometry_sha256==case['collision_geometry_sha256']
out=HERE/'reports/extended_baselines_v1'/('ompl_canonical_'+method.lower())
out.mkdir(exist_ok=False)
write(out/'case.json',case)
config=load_config()
config['ompl']=dict(planning_seconds=2.)
started=time.perf_counter()
summary=dict(method=method,problem_id=case['problem_id'],scope='integration probe, not registered performance score',planning_seconds=2.)
try:
 candidate=solve(model,case,config,out,started+30,73,method)
 write(out/'candidate.json',candidate)
 report=validate(candidate,model,case,load_config(),out=out)
 summary.update(status=report['status'],cost=report['cost'],minimum_clearance_m=report['minimum_sampled_clearance_m'])
except OmplNoExactSolution as exc:
 summary.update(status='planner_timeout',native_status=exc.solver_status)
finally:
 summary['total_seconds']=time.perf_counter()-started
 write(out/'probe_summary.json',summary)
 print(summary)
