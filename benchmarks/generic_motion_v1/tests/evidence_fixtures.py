"""Small numeric artifacts for CPU evidence-accounting tests, not real runs."""
import numpy as np
from ..roadmap import clean, load_config
from ..common import write, read, BASE
from ..runners.validate import sample_candidate


def save_numeric_output(attempt, case, config, *, kind="polygon", objective="minimum_distance",
                        solver_success=True, length=None, duration=4., passed=True):
    """Construct self-consistent arrays without invoking a collision checker.

    Collision masks are explicit test doubles, used only by accounting tests.
    """
    points=np.asarray([case["q_start"],case["q_goal"]],float)
    if length is not None:
        middle=points[0].copy();middle[0]+=length/2
        points=np.vstack([points[0],middle,points[-1]])
    candidate=dict(kind=kind,objective=objective,solver_success=solver_success)
    if kind=="polygon":
        arrays=dict(points=points)
    elif kind=="bspline":
        candidate["spline_order"]=2
        arrays=dict(controls=points,knots=np.r_[0.,np.linspace(0.,1.,len(points)),1.])
    elif kind=="bezier":
        arrays=dict(controls=np.asarray([[points[0],points[0],points[-1],points[-1]]]),durations=np.array([duration]))
    else:
        raise ValueError(kind)
    cfg=load_config(config)
    sampled=sample_candidate(dict(candidate,**arrays),cfg)
    q,t,segment=sampled["q"],sampled["time"],sampled["segment"]
    np.savez_compressed(attempt/"candidate.npz",**arrays)
    write(attempt/"candidate.json",candidate)
    np.savez_compressed(attempt/"trajectory_samples.npz",q=q,time=t,segment=segment,
                       native=np.ones(len(q),bool),independent=np.ones(len(q),bool),clearance=np.full(len(q),.01))
    robot=read(BASE/"configs/robot.json")
    lower,upper=np.asarray(robot['lower']),np.asarray(robot['upper'])
    report=dict(passed=passed,status="success_validated_numerical" if passed else "collision_failed",
        collision_free=passed,joint_limits_passed=True,derivative_limits_passed=True,
        native_collisions=0,independent_collisions=0,model_disagreements=0,contact_band_samples=0,
        endpoint_error_rad=float(np.max(np.linalg.norm(q[[0,-1]]-np.asarray([case["q_start"],case["q_goal"]]),axis=1))),
        sample_step_bound_rad=cfg["validation_step_rad"],minimum_sampled_clearance_m=.01,
        actual_maximum_sample_joint_step_rad=float(np.linalg.norm(np.diff(q,axis=0),axis=1).max()),
        cost=sampled["cost"],cost_unit="rad" if objective=="minimum_distance" else "s",
        path_length_rad=float(np.linalg.norm(np.diff(q,axis=0),axis=1).sum()),
        normalized_path_length=float(np.linalg.norm(np.diff(q,axis=0)/(upper-lower),axis=1).sum()),
        duration_s=sampled["duration"],derivative_bounds=sampled["derivatives"],
        time_axis=sampled["time_axis"],samples=len(q),region_residual=0.)
    write(attempt/"validation.json",clean(report))
