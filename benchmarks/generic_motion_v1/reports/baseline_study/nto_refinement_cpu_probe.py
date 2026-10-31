"""Predeclared fixed-dev CPU diagnostics; not full pipeline benchmark scores."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time
import traceback

import numpy as np

from benchmarks.generic_motion_v1.model import Model
from benchmarks.generic_motion_v1.roadmap import HERE, ROOT, write
from benchmarks.generic_motion_v1.runners.paper_nto_refined import PROFILE, paper_nto_refined
from benchmarks.generic_motion_v1.runners.validate import sample_candidate

JOBS = (
    "dev-sparse_clutter-002-p02-NTO_PAPER-minimum_distance-462e42b6",
    "dev-dense_clutter-002-p00-NTO_PAPER-minimum_distance-62127136",
    "dev-route_choice-002-p00-NTO_PAPER-minimum_distance-48b608ce",
)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    plan = dict(profile=PROFILE, jobs=JOBS,
                selection="Previously diagnosed fixed dev iteration-limit, collision-failure and backend-exception cases; selected before refined solves. Not a random performance sample.",
                comparison="Reuses exact saved accepted DRM path and input; no GPU or frontend rerun, so these are CPU diagnostic outcomes, not full benchmark success labels.",
                test_data_used=False, source_sha256={str(p.relative_to(ROOT)):sha(p) for p in (
                    Path(__file__).resolve(), HERE/'runners/paper_nto.py', HERE/'runners/paper_nto_refined.py')})
    write(out/'predeclared_plan.json',plan)
    results=[]
    for job in JOBS:
        old=HERE/'results/rss2025_full_dev_v1/jobs'/job/'attempts/0001'
        target=out/job
        target.mkdir()
        case=json.loads((old/'input.json').read_text())
        cfg=json.loads((old/'request.json').read_text())["evaluation_config"]
        path=np.load(old/'path.npy')
        prior=json.loads((old/'result.json').read_text())
        assert case['split']=='dev' and case['straight_line_collision']
        evidence=dict(job=job, old_attempt=str(old), prior_status=prior['status'],
                      input_sha256=sha(old/'input.json'), path_sha256=sha(old/'path.npy'),
                      observed_frontend_seconds=prior['timing']['drm_path_seconds']+prior['timing']['drm_path_validation_seconds']+prior['timing']['online_collision_map_seconds'])
        write(target/'source.json',evidence)
        tic=time.perf_counter()
        try:
            model=Model(case['voxels_xyz'],case['voxel_radius_m'])
            # Reserve the historically observed online frontend duration rather
            # than giving this isolated optimizer a fresh full 60 seconds.
            deadline=tic+60-evidence['observed_frontend_seconds']
            candidate=paper_nto_refined(path,model,case,cfg,target,deadline)
            write(target/'returned_candidate.json',candidate)
            samples=sample_candidate(candidate,cfg)
            clearance=model.clearance(samples['q'])
            np.savez_compressed(target/'cpu_final_samples.npz',q=samples['q'],clearance=clearance)
            endpoint_error=float(np.max(np.abs(samples['q'][[0,-1]]-path[[0,-1]])))
            limits=bool(np.all(samples['q']>=model.lower-cfg['feasibility_tolerance']) and np.all(samples['q']<=model.upper+cfg['feasibility_tolerance']))
            evidence.update(solver_success=candidate['solver_success'],
                            solver_status=candidate['solver_status'],
                            cpu_geometry_free=bool(np.all(clearance>model.contact_band_m)),
                            minimum_clearance_m=float(clearance.min()),
                            samples=len(clearance),endpoint_error_rad=endpoint_error,
                            joint_limits_passed=limits,cost=samples['cost'],
                            initial_path_length=float(np.linalg.norm(np.diff(path,axis=0),axis=1).sum()),
                            native_solves=candidate['refinement_native_solves'],
                            outcome='returned_candidate')
        except Exception as exc:
            evidence.update(outcome='exception',exception=repr(exc),traceback=traceback.format_exc())
        evidence['cpu_diagnostic_seconds']=time.perf_counter()-tic
        write(target/'diagnostic.json',evidence)
        results.append(evidence)
        write(out/'results.json',dict(plan=plan,results=results,final_common_gpu_validation_not_run=True))
        print(job,evidence.get('solver_success'),evidence.get('cpu_geometry_free'),evidence.get('exception'),flush=True)
    # Isolated assertion reproduction remains separate from the refined solver.
    script=Path(__file__).with_name('nto_backend_last_q_probe')/'reproduce_isolated_pair.py'
    with (out/'boxbox_minimal_reproduction.log').open('w') as stream:
        process=subprocess.run([sys.executable,'-m','benchmarks.generic_motion_v1.reports.baseline_study.nto_backend_last_q_probe.reproduce_isolated_pair'],cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT,timeout=15)
    log=(out/'boxbox_minimal_reproduction.log').read_text()
    write(out/'boxbox_minimal_reproduction.json',dict(returncode=process.returncode,
          reproduced='BoxBoxGradient' in log and 'failed' in log,
          script=str(script),script_sha256=sha(script),
          q_sha256=sha(script.with_name('last_evaluated_q.npy')),
          original_model_and_filters=True,backend_patched=False))


if __name__=='__main__':
    main()
