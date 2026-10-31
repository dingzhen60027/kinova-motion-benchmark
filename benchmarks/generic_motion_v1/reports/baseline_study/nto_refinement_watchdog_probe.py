"""Hard CPU watchdog regression on the already selected dev route-choice path."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time
import traceback

import numpy as np

from benchmarks.generic_motion_v1.model import Model
from benchmarks.generic_motion_v1.roadmap import HERE, ROOT, write
from benchmarks.generic_motion_v1.runners.paper_nto_refined import paper_nto_refined


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--worker',action='store_true')
    parser.add_argument('--deadline',type=float)
    args=parser.parse_args()
    old=HERE/'results/rss2025_full_dev_v1/jobs/dev-route_choice-002-p00-NTO_PAPER-minimum_distance-48b608ce/attempts/0001'
    out=args.output.resolve()
    case=json.loads((old/'input.json').read_text())
    cfg=json.loads((old/'request.json').read_text())['evaluation_config']
    if args.worker:
        try:
            model=Model(case['voxels_xyz'],case['voxel_radius_m'])
            candidate=paper_nto_refined(np.load(old/'path.npy'),model,case,cfg,out,args.deadline)
            write(out/'returned_candidate.json',candidate)
        except Exception as exc:
            write(out/'worker_exception.json',dict(exception=repr(exc),traceback=traceback.format_exc()))
        return
    out.mkdir(parents=True,exist_ok=False)
    timing=json.loads((old/'result.json').read_text())['timing']
    reserved=sum(timing[k] for k in ('online_collision_map_seconds','drm_path_seconds','drm_path_validation_seconds'))
    budget=60-reserved
    started=time.perf_counter()
    argv=[sys.executable,'-m','benchmarks.generic_motion_v1.reports.baseline_study.nto_refinement_watchdog_probe',
          '--output',str(out),'--worker','--deadline',str(started+budget)]
    record=dict(command=argv,original_attempt=str(old),reserved_frontend_seconds=reserved,
                worker_budget_seconds=budget,full_online_budget_seconds=60,
                GPU_used=False,test_used=False)
    write(out/'watchdog_request.json',record)
    with (out/'worker.log').open('w') as stream:
        try:
            process=subprocess.run(argv,cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT,timeout=budget)
            record.update(returncode=process.returncode,external_timeout=False)
        except subprocess.TimeoutExpired:
            record.update(returncode=None,external_timeout=True)
    record.update(actual_worker_wall_seconds=time.perf_counter()-started,
                  candidate_returned=(out/'returned_candidate.json').exists(),
                  retained_round_directories=[str(p.relative_to(out)) for p in sorted((out/'refinement').glob('round_*'))])
    write(out/'watchdog_result.json',record)
    print(json.dumps(record,indent=2))


if __name__=='__main__':main()
