"""Common finite-resolution numerical validation, independent of optimization.

Collision samples never establish a mathematical continuous-safety certificate.
Bezier derivative-control hull bounds are conservative for their full segment.
"""
from __future__ import annotations
import numpy as np
from scipy.special import comb
from ..roadmap import load_config, write


def _numeric_array(value, name):
    def contains_bool(item):
        if isinstance(item,(bool,np.bool_)):
            return True
        if isinstance(item,(list,tuple)):
            return any(contains_bool(child) for child in item)
        return isinstance(item,np.ndarray) and item.dtype.kind=='b'
    array=np.asarray(value)
    if array.dtype.kind not in 'fiu' or contains_bool(value):
        raise ValueError(f'{name} must contain numeric values, not coerced strings or booleans')
    return array.astype(float)


def _sample_counts(bounds, step, cap, minimum):
    bounds = np.asarray(bounds, float)
    counts = np.maximum(minimum, np.ceil(bounds / step) + 1)
    if not np.isfinite(counts).all() or np.any(counts > cap) or counts.sum() > cap:
        raise ValueError('Numerical sampling cap exceeded or nonfinite derivative bound')
    return counts.astype(np.int64)


def _objective(candidate):
    objective = candidate.get('objective')
    if objective not in ('minimum_distance', 'minimum_time'):
        raise ValueError('Candidate must declare a supported objective')
    kind = candidate.get('kind')
    if ((objective == 'minimum_time' and kind not in ('bezier', 'bspline')) or
            (objective == 'minimum_distance' and kind not in ('polygon', 'bspline'))):
        raise ValueError('Candidate representation cannot implement the declared objective')
    return objective


def _region_matrices(region):
    A=_numeric_array(region.A(),'Region A')
    b=_numeric_array(region.b(),'Region b')
    if (A.ndim!=2 or A.shape[1]!=7 or len(A)==0 or b.shape!=(len(A),)
            or not np.isfinite(A).all() or not np.isfinite(b).all()):
        raise ValueError('Region constraints must be finite A[m,7] and b[m]')
    return A,b


def _bezier(controls, parameter):
    controls=np.asarray(controls,float)
    n=len(controls)-1
    s=np.asarray(parameter,float)[:,None]
    k=np.arange(n+1)
    basis=comb(n,k)*s**k*(1-s)**(n-k)
    return basis@controls


def sample_candidate(candidate, config=None):
    cfg=load_config(config)
    objective = _objective(candidate)
    step=float(cfg['validation_step_rad'])
    cap=int(cfg['validation_maximum_samples'])
    if step<=0 or cap<2:
        raise ValueError('Invalid numerical validation resolution')
    kind=candidate['kind']
    derivatives={}
    if kind=='polygon':
        p=_numeric_array(candidate['points'],'PWL points')
        if p.ndim!=2 or p.shape[1]!=7 or len(p)<2 or not np.isfinite(p).all():
            raise ValueError('A PWL path must have at least two finite 7D points')
        counts=_sample_counts(np.linalg.norm(np.diff(p,axis=0),axis=1),step,cap,2)
        q=np.vstack([np.linspace(a,b,int(n)) for a,b,n in zip(p[:-1],p[1:],counts)])
        segment=np.concatenate([np.full(int(n),i) for i,n in enumerate(counts)])
        time=np.concatenate([np.linspace(i,i+1,int(n))/(len(p)-1) for i,n in enumerate(counts)])
        cost=float(np.linalg.norm(np.diff(p,axis=0),axis=1).sum())
        return dict(q=q,time=time,segment=segment,duration=None,cost=cost,derivatives=derivatives,
                    position_controls=p,time_axis='normalized_path_parameter')
    if kind=='bezier':
        controls=_numeric_array(candidate['controls'],'Bezier controls')
        durations=_numeric_array(candidate['durations'],'Bezier durations')
        if (controls.ndim!=3 or controls.shape[2]!=7 or controls.shape[1]<3 or
            len(controls)==0 or durations.shape!=(len(controls),) or not np.isfinite(controls).all() or
            not np.isfinite(durations).all() or np.any(durations<=0)):
            raise ValueError('Malformed Bezier controls or durations')
        degree=controls.shape[1]-1
        velocity=degree*np.diff(controls,axis=1)/durations[:,None,None]
        acceleration=(degree-1)*np.diff(velocity,axis=1)/durations[:,None,None]
        speed=np.max(np.linalg.norm(velocity,axis=2),axis=1)
        counts=_sample_counts(durations*speed,step,cap,
            max(3,int(np.ceil(cfg['validation_minimum_samples']/len(controls)))))
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
        controls=_numeric_array(candidate['controls'],'B-spline controls')
        knots=_numeric_array(candidate['knots'],'B-spline knots')
        order=candidate['spline_order']
        if isinstance(order,bool) or not isinstance(order,(int,np.integer)):
            raise ValueError('B-spline order must be an integer')
        if (controls.ndim!=2 or controls.shape[1]!=7 or order<2 or len(controls)<order or
            knots.ndim!=1 or len(knots)!=len(controls)+order or np.any(np.diff(knots)<0) or
            not np.isfinite(controls).all() or not np.isfinite(knots).all()):
            raise ValueError('Malformed B-spline')
        timed=objective=='minimum_time'
        if (not timed and order != 2) or (timed and order < 3):
            raise ValueError('Distance requires PWL order two; time requires a C1 spline')
        if knots[order-1] >= knots[-order]:
            raise ValueError('Nonpositive B-spline duration')
        if (np.any(knots[:order] != knots[order-1])
                or np.any(knots[-order:] != knots[-order])):
            raise ValueError('Only clamped B-splines with positive active duration are supported')
        interior = knots[order:-order]
        _, multiplicities = np.unique(interior, return_counts=True)
        if (np.any(interior <= knots[order-1]) or np.any(interior >= knots[-order])
                or np.any(multiplicities > (order-2 if timed else 1))):
            raise ValueError('Interior knot multiplicity violates required spline continuity')
        traj=BsplineTrajectory(BsplineBasis(order,knots.tolist()),[p[:,None] for p in controls])
        duration=traj.end_time()-traj.start_time()
        if duration<=0:
            raise ValueError('Nonpositive B-spline duration')
        dv=traj.MakeDerivative(1)
        velocity=np.asarray([p.ravel() for p in dv.control_points()])
        speed=float(np.linalg.norm(velocity,axis=1).max())
        count=int(_sample_counts([duration*speed],step,cap,cfg['validation_minimum_samples'])[0])
        if count+len(knots)>cap:
            raise ValueError('Numerical sampling cap exceeded')
        time=np.unique(np.r_[np.linspace(traj.start_time(),traj.end_time(),count),knots[(knots>=traj.start_time())&(knots<=traj.end_time())]])
        q=traj.vector_values(time).T
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
    objective=_objective(candidate)
    if case.get('_objective',case.get('objective',objective)) != objective:
        raise ValueError('Candidate objective differs from the fixed requested objective')
    endpoints=_numeric_array([case['q_start'],case['q_goal']],'Fixed endpoints')
    if endpoints.shape != (2,7) or not np.isfinite(endpoints).all():
        raise ValueError('Fixed endpoints must be finite seven-dimensional vectors')
    lower,upper=np.asarray(model.lower,float),np.asarray(model.upper,float)
    if lower.shape!=(7,) or upper.shape!=(7,) or not np.isfinite([lower,upper]).all() or np.any(lower>=upper):
        raise ValueError('Model must expose finite ordered seven-joint limits')
    if case.get('validation_status','valid_input')!='valid_input':
        raise ValueError('An invalid frozen input cannot be validated as a solution')
    # A standalone validator must honor the manifest, not silently assume its
    # motion limits from the implementation defaults.
    motion_fields=('velocity_limits_rad_s','acceleration_limits_rad_s2','endpoint_velocity_rad_s')
    for key,expected in zip(motion_fields,[np.full(7,cfg['velocity_limit_rad_s']),
            np.full(7,cfg['acceleration_limit_rad_s2']),np.zeros(7)]):
        values=_numeric_array(case.get(key),key)
        if values.shape!=(7,) or not np.isfinite(values).all() or not np.array_equal(values,expected):
            raise ValueError('Validator motion limits differ from frozen manifest')
    samples=sample_candidate(candidate,cfg)
    q=samples['q']; tol=cfg['feasibility_tolerance']
    if (q.ndim!=2 or q.shape[1]!=7 or len(q)<2 or not np.isfinite(q).all()
            or not np.isfinite(samples['time']).all() or not np.isfinite(samples['cost'])
            or samples['cost']<0):
        raise ValueError('Candidate produced nonfinite or malformed numerical samples')
    if objective=='minimum_time' and (samples['duration'] is None or
            not np.isfinite(samples['duration']) or samples['duration']<=0):
        raise ValueError('Time objective requires a finite positive physical duration')
    native=np.asarray(model.cuda(q))
    clearance=np.asarray(model.clearance(q),float)
    if native.shape!=(len(q),) or native.dtype!=np.bool_ or clearance.shape!=(len(q),) or not np.isfinite(clearance).all():
        raise ValueError('Collision checker must return one boolean and finite clearance per sample')
    independent=clearance>0
    band=float(getattr(model,'contact_band_m',2e-6))
    if not np.isfinite(band) or band<0:
        raise ValueError('Invalid collision contact band')
    ambiguous=np.abs(clearance)<=band
    endpoint_error=float(np.max(np.linalg.norm(q[[0,-1]]-endpoints,axis=1)))
    limits=bool(np.all(samples['position_controls']>=model.lower-tol) and np.all(samples['position_controls']<=model.upper+tol))
    deriv=samples['derivatives']
    if deriv and not np.isfinite(np.r_[deriv['velocity_per_joint'],deriv['acceleration_per_joint'],
            deriv['endpoint_velocity'],deriv['continuity_residual']]).all():
        raise ValueError('Nonfinite derivative bounds cannot establish feasibility')
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
            region_residual=max(float(np.max(A@p[i:i+2].T-b[:,None]))
                for i,(A,b) in enumerate(map(_region_matrices,regions)))
        elif candidate['kind']=='bezier':
            if len(regions)!=len(candidate['controls']):
                raise ValueError('Bezier-region correspondence mismatch')
            region_residual=max(float(np.max(A@c.T-b[:,None]))
                for (A,b),c in zip(map(_region_matrices,regions),candidate['controls']))
        else:
            raise ValueError('Region correspondence is not implemented for this candidate representation')
        if not np.isfinite(region_residual):
            raise ValueError('Nonfinite region residual cannot establish feasibility')
    collisions=(~native)|(~independent)
    actual_step=float(np.linalg.norm(np.diff(q,axis=0),axis=1).max())
    if endpoint_error>cfg['endpoint_tolerance_rad']:
        status='invalid_endpoint'
    elif not limits or not derivative_pass or region_residual>tol:
        status='limit_failed'
    elif np.any(collisions & ~ambiguous):
        status='collision_failed'
    elif np.any(ambiguous):
        status='validation_undetermined'
    elif actual_step>cfg['validation_step_rad']*(1+1e-9)+1e-12:
        status='validation_undetermined'
    else:
        status='success_validated_numerical'
    report=dict(status=status,passed=status=='success_validated_numerical',
        check='Finite collision sampling; not a mathematical continuous collision certificate',
        collision_free=bool(not np.any(collisions)),endpoint_error_rad=endpoint_error,
        joint_limits_passed=limits,derivative_limits_passed=bool(derivative_pass),derivative_bounds=deriv,
        region_residual=region_residual,samples=len(q),sample_step_bound_rad=cfg['validation_step_rad'],
        actual_maximum_sample_joint_step_rad=actual_step,
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
