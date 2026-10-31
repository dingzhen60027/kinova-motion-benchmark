"""CPU-only contract re-audit; never substitute historical collision evidence.

Run from repository root with:
  bash local/python.sh -m benchmarks.generic_motion_v1.reports.qualification.baseline_reaudit
"""
from collections import Counter
from pathlib import Path
import json
import time
import numpy as np
from scipy.interpolate import BSpline, BPoly

from ...roadmap import HERE, sha, write
from ...runners.validate import sample_candidate, validate


class SavedEvidence:
    def __init__(self, robot, arrays):
        self.lower=np.asarray(robot['lower']);self.upper=np.asarray(robot['upper'])
        self.contact_band_m=robot['contact_band_m'];self.arrays=arrays
    def check(self,q):
        if not np.array_equal(q,self.arrays['q']):
            raise ValueError('Current q differs from saved q; cannot reuse collision evidence')
    def cuda(self,q):
        self.check(q);return self.arrays['native']
    def clearance(self,q):
        self.check(q);return self.arrays['clearance']


class Region:
    def __init__(self,A,b):self.a=A;self.bound=b
    def A(self):return self.a
    def b(self):return self.bound


def scipy_oracle(candidate,samples):
    """Different implementation: SciPy de Boor/Bernstein polynomial engine."""
    kind=candidate['kind']
    if kind=='polygon':
        return {'implementation':'PWL; no timed derivatives', 'timed':False}
    if kind=='bspline':
        curve=BSpline(candidate['knots'],candidate['controls'],candidate['spline_order']-1)
        difference=float(np.max(np.abs(curve(samples['time'])-samples['q'])))
        if candidate['objective']=='minimum_distance':
            return {'implementation':'SciPy BSpline de Boor','timed':False,'position_max_error_rad':difference}
        grid=np.linspace(samples['time'][0],samples['time'][-1],5001)
        velocity=np.max(np.abs(curve.derivative(1)(grid)),axis=0)
        acceleration=np.max(np.abs(curve.derivative(2)(grid)),axis=0)
        endvel=float(np.abs(curve.derivative(1)(grid[[0,-1]])).max())
        continuity=0. # Checked independently by required knot multiplicity.
    else:
        all_q=[];velocity=[];acceleration=[];ends=[];dends=[]
        offset=0.
        for i,(controls,duration) in enumerate(zip(candidate['controls'],candidate['durations'])):
            curve=BPoly(controls[:,None,:],[0.,duration])
            local=samples['time'][samples['segment']==i]-offset
            all_q.append(curve(local))
            grid=np.linspace(0,duration,5001)
            velocity.append(np.max(np.abs(curve.derivative(1)(grid)),axis=0))
            acceleration.append(np.max(np.abs(curve.derivative(2)(grid)),axis=0))
            ends.append(curve([0.,duration]));dends.append(curve.derivative(1)([0.,duration]))
            offset+=duration
        difference=float(np.max(np.abs(np.vstack(all_q)-samples['q'])))
        velocity=np.max(velocity,axis=0);acceleration=np.max(acceleration,axis=0)
        endvel=float(np.abs([dends[0][0],dends[-1][-1]]).max())
        continuity=max([0.]+[float(np.abs(ends[i][-1]-ends[i+1][0]).max()) for i in range(len(ends)-1)]
            +[float(np.abs(dends[i][-1]-dends[i+1][0]).max()) for i in range(len(ends)-1)])
    derivatives=samples['derivatives']
    if np.any(velocity>derivatives['velocity_per_joint']+1e-10) or np.any(acceleration>derivatives['acceleration_per_joint']+1e-10):
        raise ValueError('Independent SciPy derivative samples exceed claimed control-hull bounds')
    return dict(implementation='SciPy BSpline or BPoly derivatives; 5001 points per polynomial segment',
        timed=True,position_max_error_rad=difference,dense_velocity_per_joint=velocity,
        dense_acceleration_per_joint=acceleration,endpoint_velocity=endvel,continuity_residual=continuity)


def main():
    start=time.perf_counter();rows=[];runs={}
    robot=json.loads((HERE/'configs/robot.json').read_text())
    for run_name in ('smoke_v3_fixed','dev_eizo_v2','dev_paired_v2'):
        run=HERE/'results'/run_name
        manifest=json.loads((run/'run_manifest.json').read_text())
        run_rows=[]
        for job in manifest['jobs']:
            attempts=sorted((run/'jobs'/job['job_id']/'attempts').iterdir())
            attempt=attempts[-1]
            result=json.loads((attempt/'result.json').read_text())
            if result['status']!='success_validated_numerical':continue
            row=dict(run=run_name,job_id=job['job_id'],method=job['method'],objective=job['objective'],
                     attempt=str(attempt.relative_to(HERE)),status='failed_contract_reaudit')
            try:
                receipt=json.loads((attempt/'receipt.json').read_text())
                required=['candidate.json','candidate.npz','trajectory_samples.npz','validation.json','result.json','input.json']
                if (attempt/'regions.npz').is_file():required.append('regions.npz')
                for name in required:
                    if receipt.get('files',{}).get(name)!=sha(attempt/name):
                        raise ValueError('Missing or mismatched sealed evidence: '+name)
                row['evidence_sha256']={name:receipt['files'][name] for name in required}
                row['receipt_sha256']=sha(attempt/'receipt.json')
                candidate=json.loads((attempt/'candidate.json').read_text())
                with np.load(attempt/'candidate.npz',allow_pickle=False) as archive:
                    candidate.update({key:archive[key] for key in archive.files})
                if candidate.get('solver_success') is not True:
                    raise ValueError('Saved success does not have successful solver evidence')
                with np.load(attempt/'trajectory_samples.npz',allow_pickle=False) as archive:
                    saved={key:archive[key] for key in archive.files}
                samples=sample_candidate(candidate,job['evaluation_config'])
                if not np.array_equal(samples['q'],saved['q']) or not np.array_equal(samples['time'],saved['time']):
                    raise ValueError('Trajectory sampling changed; fresh collision computation required')
                if not np.array_equal(saved['independent'],saved['clearance']>0):
                    raise ValueError('Saved independent classifications disagree with saved clearances')
                regions=[]
                if (attempt/'regions.npz').is_file():
                    with np.load(attempt/'regions.npz',allow_pickle=False) as archive:
                        regions=[Region(archive[key],archive[key.replace('A_','b_')])
                            for key in sorted(archive.files) if key.startswith('A_')]
                case=dict(job['case'],_objective=job['objective'])
                new=validate(candidate,SavedEvidence(robot,saved),case,job['evaluation_config'],regions=regions)
                oracle=scipy_oracle(candidate,samples)
                if not new['passed']:raise ValueError('New validation status: '+new['status'])
                if oracle.get('position_max_error_rad',0)>1e-10:raise ValueError('Independent position evaluation disagrees')
                if oracle.get('endpoint_velocity',0)>job['evaluation_config']['feasibility_tolerance']:
                    raise ValueError('Independent endpoint velocity check failed')
                if oracle.get('continuity_residual',0)>job['evaluation_config']['feasibility_tolerance']:
                    raise ValueError('Independent segment continuity check failed')
                old=json.loads((attempt/'validation.json').read_text())
                if not np.isclose(new['cost'],old['cost'],rtol=1e-12,atol=1e-12) or new['cost_unit']!=old['cost_unit']:
                    raise ValueError('Objective value or unit changed')
                row.update(status='passed_contract_reaudit',samples_identical=True,samples=new['samples'],
                    endpoint_error_rad=new['endpoint_error_rad'],cost=new['cost'],cost_unit=new['cost_unit'],
                    derivative_bounds=new['derivative_bounds'],scipy_oracle=oracle,
                    minimum_historical_sampled_clearance_m=new['minimum_sampled_clearance_m'],
                    maximum_sample_step_rad=new['actual_maximum_sample_joint_step_rad'])
            except Exception as exc:row['error']=repr(exc)
            rows.append(row);run_rows.append(row)
        runs[run_name]=dict(planned_jobs=len(manifest['jobs']),successful_candidates=len(run_rows),
            statuses=dict(Counter(row['status'] for row in run_rows)),run_manifest_sha256=sha(run/'run_manifest.json'))
    record=dict(scope='CPU contract re-audit of every sealed historical successful candidate; unchanged original files. '
        'Collision arrays are reused only at bit-identical q samples. No fresh collision checking or GPU planning was performed. '
        'SciPy independent position/derivative checks do not certify continuous collision freedom.',
        source_sha256={str(path.relative_to(HERE)):sha(path) for path in
            [Path(__file__),HERE/'runners/validate.py',HERE/'roadmap.py']},
        runs=runs,statuses=dict(Counter(row['status'] for row in rows)),candidates=len(rows),
        elapsed_seconds=time.perf_counter()-start,rows=rows)
    out=HERE/'reports/qualification/baseline_saved_candidate_reaudit.json';write(out,record)
    lines=['# Historical successful-candidate contract re-audit','',record['scope'],'',
        '| Run | Successful candidates | Passed | Rejected |','|---|---:|---:|---:|']
    for name,summary in runs.items():
        counts=summary['statuses'];lines.append(f"| {name} | {summary['successful_candidates']} | {counts.get('passed_contract_reaudit',0)} | {counts.get('failed_contract_reaudit',0)} |")
    errors=[row for row in rows if row['status']!='passed_contract_reaudit']
    lines+=['',f"Elapsed CPU audit wall time: {record['elapsed_seconds']:.3f} s.",
        '',f"Full per-attempt hashes and numerical checks: [{out.name}]({out.name})."]
    if errors:lines+=['','Rejected candidates:']+[f"- {row['job_id']}: {row['error']}" for row in errors]
    out.with_suffix('.md').write_text('\n'.join(lines)+'\n')
    print(json.dumps({key:record[key] for key in ('runs','statuses','candidates','elapsed_seconds')},indent=2))
    return int(bool(errors))


if __name__=='__main__':raise SystemExit(main())
