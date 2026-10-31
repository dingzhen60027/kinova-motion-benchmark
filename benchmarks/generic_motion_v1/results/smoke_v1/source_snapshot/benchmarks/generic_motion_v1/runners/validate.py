"""Common finite-resolution numerical validation, independent of optimization.

Collision samples never establish a mathematical continuous-safety certificate.
Bezier derivative-control hull bounds are conservative for their full segment.
"""
from __future__ import annotations
import numpy as np
from scipy.special import comb
from ..roadmap import load_config, write


def _bezier(controls, parameter):
    controls=np.asarray(controls,float)
    n=len(controls)-1
    s=np.asarray(parameter,float)[:,None]
    k=np.arange(n+1)
    basis=comb(n,k)*s**k*(1-s)**(n-k)
    return basis@controls


def sample_candidate(candidate, config=None):
    cfg=load_config(config)
    step=float(cfg['validation_step_rad'])
    cap=int(cfg['validation_maximum_samples'])
    if step<=0 or cap<2:
        raise ValueError('Invalid numerical validation resolution')
    kind=candidate['kind']
    derivatives={}
    if kind=='polygon':
        p=np.asarray(candidate['points'],float)
        if p.ndim!=2 or p.shape[1]!=7 or len(p)<2 or not np.isfinite(p).all():
            raise ValueError('A PWL path must have at least two finite 7D points')
        counts=np.maximum(2,np.ceil(np.linalg.norm(np.diff(p,axis=0),axis=1)/step).astype(int)+1)
        if int(counts.sum())>cap:
            raise ValueError('Numerical sampling cap exceeded')
        q=np.vstack([np.linspace(a,b,int(n)) for a,b,n in zip(p[:-1],p[1:],counts)])
        segment=np.concatenate([np.full(int(n),i) for i,n in enumerate(counts)])
        time=np.concatenate([np.linspace(i,i+1,int(n))/(len(p)-1) for i,n in enumerate(counts)])
        cost=float(np.linalg.norm(np.diff(p,axis=0),axis=1).sum())
        return dict(q=q,time=time,segment=segment,duration=None,cost=cost,derivatives=derivatives,
                    position_controls=p,time_axis='normalized_path_parameter')
    if kind=='bezier':
        controls=np.asarray(candidate['controls'],float)
        durations=np.asarray(candidate['durations'],float)
        if (controls.ndim!=3 or controls.shape[2]!=7 or controls.shape[1]<3 or
            len(controls)==0 or durations.shape!=(len(controls),) or not np.isfinite(controls).all() or
            not np.isfinite(durations).all() or np.any(durations<=0)):
            raise ValueError('Malformed Bezier controls or durations')
        degree=controls.shape[1]-1
        velocity=degree*np.diff(controls,axis=1)/durations[:,None,None]
        acceleration=(degree-1)*np.diff(velocity,axis=1)/durations[:,None,None]
        speed=np.max(np.linalg.norm(velocity,axis=2),axis=1)
        counts=np.maximum(3,np.ceil(durations*speed/step).astype(int)+1)
        counts=np.maximum(counts,int(np.ceil(cfg['validation_minimum_samples']/len(controls))))
        if int(counts.sum())>cap:
            raise ValueError('Numerical sampling cap exceeded')
        offsets=np.r_[0,np.cumsum(durations)]
        params=[np.linspace(0,1,int(n)) for n in counts]
        q=np.vstack([_bezier(c,s) for c,s in zip(controls,params)])
        time=np.concatenate([t+s*d for t,s,d in zip(offsets[:-1],params,durations)])
        segment=np.concatenate([np.full(int(n),i) for i,n in enumerate(counts)])
        continuity=max([0.]+[float(np.max(np.abs(controls[i,-1]-controls[i+1,0]))) for i in range(len(controls)-1)]
                      +[float(np.max(np.abs(velocity[i,-1]-velocity[i+1,0]))) for i in range(len(controls)-1)])
        endpoint_velocity=float(np.max(np.abs([velocity[0,0],velocity[-1,-1]])))
        derivatives=dict(velocity_per_joint=np.max(np.abs(velocity),axis=(0,1)),
                         acceleration_per_joint=np.max(np.abs(acceleration),axis=(0,1)),
                         endpoint_velocity=endpoint_velocity,continuity_residual=continuity,
                         method='Bezier derivative control hull bounds over every segment')
        return dict(q=q,time=time,segment=segment,duration=float(durations.sum()),cost=float(durations.sum()),
                    derivatives=derivatives,position_controls=controls.reshape(-1,7),time_axis='seconds')
    if kind=='bspline':
        from pydrake.all import BsplineBasis,BsplineTrajectory
        controls=np.asarray(candidate['controls'],float)
        knots=np.asarray(candidate['knots'],float)
        order=int(candidate['spline_order'])
        if (controls.ndim!=2 or controls.shape[1]!=7 or order<2 or len(controls)<order or
            len(knots)!=len(controls)+order or np.any(np.diff(knots)<0) or
            not np.isfinite(controls).all() or not np.isfinite(knots).all()):
            raise ValueError('Malformed B-spline')
        traj=BsplineTrajectory(BsplineBasis(order,knots.tolist()),[p[:,None] for p in controls])
        duration=traj.end_time()-traj.start_time()
        if duration<=0:
            raise ValueError('Nonpositive B-spline duration')
        dv=traj.MakeDerivative(1)
        velocity=np.asarray([p.ravel() for p in dv.control_points()])
        speed=float(np.linalg.norm(velocity,axis=1).max())
        count=max(cfg['validation_minimum_samples'],int(np.ceil(duration*speed/step))+1)
        if count+len(knots)>cap:
            raise ValueError('Numerical sampling cap exceeded')
        time=np.unique(np.r_[np.linspace(traj.start_time(),traj.end_time(),count),knots[(knots>=traj.start_time())&(knots<=traj.end_time())]])
        q=traj.vector_values(time).T
        timed=candidate.get('objective')=='minimum_time'
        if timed:
            da=traj.MakeDerivative(2)
            acceleration=np.asarray([p.ravel() for p in da.control_points()])
            derivatives=dict(velocity_per_joint=np.max(np.abs(velocity),axis=0),
                             acceleration_per_joint=np.max(np.abs(acceleration),axis=0),
                             endpoint_velocity=float(np.max(np.abs(dv.vector_values([traj.start_time(),traj.end_time()])))),
                             continuity_residual=0.,method='B-spline derivative control hull bounds')
        cost=float(duration) if timed else float(np.linalg.norm(np.diff(controls if order==2 else q,axis=0),axis=1).sum())
        return dict(q=q,time=time,segment=np.zeros(len(q),int),duration=float(duration) if timed else None,
                    cost=cost,derivatives=derivatives,position_controls=controls,
                    time_axis='seconds' if timed else 'spline_parameter_not_physical_time')
    raise ValueError(f'Unsupported candidate kind {kind}')


def validate(candidate,model,case,config=None,regions=None,out=None):
    cfg=load_config(config)
    samples=sample_candidate(candidate,cfg)
    q=samples['q']; tol=cfg['feasibility_tolerance']
    native=model.cuda(q)
    clearance=model.clearance(q)
    independent=clearance>0
    band=float(getattr(model,'contact_band_m',2e-6))
    ambiguous=np.abs(clearance)<=band
    endpoints=np.asarray([case['q_start'],case['q_goal']],float)
    endpoint_error=float(np.max(np.linalg.norm(q[[0,-1]]-endpoints,axis=1)))
    limits=bool(np.all(samples['position_controls']>=model.lower-tol) and np.all(samples['position_controls']<=model.upper+tol))
    deriv=samples['derivatives']
    derivative_pass=(not deriv or
        (np.max(deriv['velocity_per_joint'])<=cfg['velocity_limit_rad_s']+tol and
         np.max(deriv['acceleration_per_joint'])<=cfg['acceleration_limit_rad_s2']+tol and
         deriv['endpoint_velocity']<=tol and deriv['continuity_residual']<=tol))
    region_residual=0.
    if regions:
        if candidate['kind']=='polygon':
            p=np.asarray(candidate['points'])
            if len(regions)!=len(p)-1:
                raise ValueError('PWL-region correspondence mismatch')
            region_residual=max(float(np.max(r.A()@p[i:i+2].T-r.b()[:,None])) for i,r in enumerate(regions))
        elif candidate['kind']=='bezier':
            if len(regions)!=len(candidate['controls']):
                raise ValueError('Bezier-region correspondence mismatch')
            region_residual=max(float(np.max(r.A()@c.T-r.b()[:,None])) for r,c in zip(regions,candidate['controls']))
    collisions=(~native)|(~independent)
    if endpoint_error>cfg['endpoint_tolerance_rad']:
        status='invalid_endpoint'
    elif not limits or not derivative_pass or region_residual>tol:
        status='limit_failed'
    elif np.any(collisions & ~ambiguous):
        status='collision_failed'
    elif np.any(ambiguous):
        status='validation_undetermined'
    else:
        status='success_validated_numerical'
    report=dict(status=status,passed=status=='success_validated_numerical',
        check='Finite collision sampling; not a mathematical continuous collision certificate',
        collision_free=bool(not np.any(collisions)),endpoint_error_rad=endpoint_error,
        joint_limits_passed=limits,derivative_limits_passed=bool(derivative_pass),derivative_bounds=deriv,
        region_residual=region_residual,samples=len(q),sample_step_bound_rad=cfg['validation_step_rad'],
        actual_maximum_sample_joint_step_rad=float(np.linalg.norm(np.diff(q,axis=0),axis=1).max()),
        native_collisions=int((~native).sum()),independent_collisions=int((~independent).sum()),
        model_disagreements=int((native!=independent).sum()),contact_band_samples=int(ambiguous.sum()),
        minimum_sampled_clearance_m=float(clearance.min()),cost=samples['cost'],duration_s=samples['duration'],
        path_length_rad=float(np.linalg.norm(np.diff(q,axis=0),axis=1).sum()),
        normalized_path_length=float(np.linalg.norm(np.diff(q,axis=0)/(model.upper-model.lower),axis=1).sum()),
        cost_unit='s' if candidate.get('objective')=='minimum_time' else 'rad',
        collision_points=q[collisions],collision_regions=samples['segment'][collisions],
        _samples=dict(q=q,time=samples['time'],segment=samples['segment'],native=native,independent=independent,clearance=clearance),
        time_axis=samples['time_axis'])
    if out is not None:
        from pathlib import Path
        out=Path(out);out.mkdir(parents=True,exist_ok=True)
        write(out/'validation.json',{k:v for k,v in report.items() if k!='_samples'})
        np.savez_compressed(out/'trajectory_samples.npz',**report['_samples'])
    return report
