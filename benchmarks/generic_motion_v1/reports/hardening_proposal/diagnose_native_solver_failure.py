"""Read sealed failures; CPU-only optimizer checks, never edit original artifacts."""
from pathlib import Path
import hashlib
import json
import time
import numpy as np
from pydrake.all import HPolyhedron
from scsplanning.polygonal import get_points
from scsplanning import biconvex
from benchmarks.generic_motion_v1.roadmap import write

BASE=Path(__file__).resolve().parents[2]
REPORT=BASE/'reports/native_solver_failure_diagnosis.json'
selected=[]
for run in ('smoke_v2_clean','dev_eizo_v1'):
    for p in sorted((BASE/'results'/run).glob('jobs/*/attempts/0001/result.json')):
        r=json.loads(p.read_text())
        if r.get('status')=='solver_failed' and r.get('objective')=='minimum_distance' and (p.parent/'receipt.json').exists():
            selected.append(p.parent);break
records=[]
for index,directory in enumerate(selected):
    files=[directory/name for name in ('result.json','input.json','path.npy','regions.npz','receipt.json')]
    before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    case=json.loads((directory/'input.json').read_text())
    path=np.load(directory/'path.npy');a=np.load(directory/'regions.npz')
    start=np.asarray(case['q_start']);goal=np.asarray(case['q_goal'])
    originals=[HPolyhedron(a[f'A_{i:04d}'],a[f'b_{i:04d}']) for i in range(len(path)-1)]
    seed_residuals=[];shared_residuals=[];corrected=[];increments=[]
    for i,(r,left,right) in enumerate(zip(originals,path[:-1],path[1:])):
        seed_residuals.append([float(np.max(r.A()@left-r.b())),float(np.max(r.A()@right-r.b()))])
        if i:
            shared_residuals.append(max(float(np.max(originals[i-1].A()@left-originals[i-1].b())),float(np.max(r.A()@left-r.b()))))
        anchor_left=start if i==0 else left
        anchor_right=goal if i==len(originals)-1 else right
        b=np.maximum(r.b(),np.maximum(r.A()@anchor_left,r.A()@anchor_right))
        increments.append(float(np.max(b-r.b())))
        corrected.append(HPolyhedron(r.A(),b))
    record=dict(case_id=case['problem_id'],attempt=str(directory),before_file_hashes=before,
                region_count=len(originals),native_path_endpoint_max_abs_error=[float(np.max(np.abs(start-path[0]))),float(np.max(np.abs(goal-path[-1])))],
                path_seed_membership_residuals=seed_residuals,adjacent_region_shared_witness_residuals=shared_residuals,
                manifest_endpoint_residuals=[float(np.max(originals[0].A()@start-originals[0].b())),float(np.max(originals[-1].A()@goal-originals[-1].b()))],
                maximum_b_correction_per_region=increments,solves={})
    for name,regions,endpoints in [('original_manifest_endpoints',originals,[start,goal]),
                                  ('original_float32_endpoints',originals,[path[0],path[-1]]),
                                  ('corrected_original_manifest_endpoints',corrected,[start,goal])]:
        tic=time.perf_counter()
        try:
            points=get_points(*endpoints,regions)
            record['solves'][name]=dict(success=True,cost=float(np.linalg.norm(np.diff(points,axis=0),axis=1).sum()),
                                       maximum_region_residual=max(float(np.max(r.A()@points[i:i+2].T-r.b()[:,None])) for i,r in enumerate(regions)),
                                       endpoint_error=float(np.max(np.abs(points[[0,-1]]-endpoints))),seconds=time.perf_counter()-tic)
        except Exception as exc:
            record['solves'][name]=dict(success=False,error=repr(exc),seconds=time.perf_counter()-tic)
    for name,regions in [('original_manifest_time',originals),('corrected_manifest_time',corrected)]:
        tic=time.perf_counter()
        try:
            curve=biconvex(start,goal,regions,HPolyhedron.MakeBox(-np.ones(7),np.ones(7)),HPolyhedron.MakeBox(-np.ones(7),np.ones(7)),6,.01)
            controls=np.asarray([c.points for c in curve.curves],float)
            durations=np.asarray([c.duration for c in curve.curves],float)
            np.savez_compressed(BASE/'reports/hardening_proposal'/f'diagnostic_{index}_{name}.npz',controls=controls,durations=durations)
            record['solves'][name]=dict(success=True,duration_s=float(curve.duration),seconds=time.perf_counter()-tic)
        except Exception as exc:
            record['solves'][name]=dict(success=False,error=repr(exc),seconds=time.perf_counter()-tic)
    after={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    record['original_artifacts_unchanged']=before==after
    records.append(record)
write(REPORT,dict(check='CPU-only isolated diagnosis; no CUDA, no benchmark score replacement',records=records))
print(json.dumps([dict(case=r['case_id'],residuals=r['manifest_endpoint_residuals'],corrections=r['maximum_b_correction_per_region'],solves=r['solves']) for r in records],indent=2))
