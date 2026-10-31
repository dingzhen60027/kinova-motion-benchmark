"""Adversarial admission tests with analytic and independent spline oracles."""
import json
import numpy as np
import pytest
from scipy.interpolate import BSpline

from benchmarks.generic_motion_v1.roadmap import build_roadmap,load_config
from benchmarks.generic_motion_v1.runners.validate import sample_candidate,validate
from benchmarks.generic_motion_v1.runners.csdecomp_baseline import (_checked_native_path,_method_seed,
    _input_endpoints,dependency_audit,cover_region_anchors,run_case)


class EmptyWorld:
    lower=np.full(7,-10.)
    upper=np.full(7,10.)
    contact_band_m=2e-6
    def cuda(self,q):return np.ones(len(q),bool)
    def clearance(self,q):return np.full(len(q),.1)


def task():
    return dict(q_start=[0.]*7,q_goal=[.1,0,0,0,0,0,0],
        velocity_limits_rad_s=[1.]*7,acceleration_limits_rad_s2=[1.]*7,endpoint_velocity_rad_s=[0.]*7)


def polygon():
    t=task()
    return dict(kind='polygon',points=[t['q_start'],t['q_goal']],objective='minimum_distance')


@pytest.mark.parametrize('goal',[[float('nan')]*7,[float('inf')]*7,[True]*7,['0.1']*7,[0.]*6,None])
def test_non_numeric_or_nonfinite_fixed_endpoint_never_succeeds(goal):
    case=task();case['q_goal']=goal
    with pytest.raises((ValueError,TypeError)):
        validate(polygon(),EmptyWorld(),case)


def test_mixed_boolean_coordinates_are_not_silently_cast_to_joint_angles():
    case=task();case['q_goal'][3]=True
    with pytest.raises(ValueError):validate(polygon(),EmptyWorld(),case)
    case['validation_status']='valid_input'
    assert _input_endpoints(case) is None


@pytest.mark.parametrize('shape,nan',[(None,True),((1,1),False),((0,7),False)])
def test_region_constraint_nan_or_broadcast_cannot_be_a_success(shape,nan):
    class Region:
        def A(self):return np.full(shape or (1,7),float('nan') if nan else 0.)
        def b(self):return np.ones(1)
    with pytest.raises(ValueError):validate(polygon(),EmptyWorld(),task(),regions=[Region()])


@pytest.mark.parametrize('objective',['minimum_time','typo',None])
def test_polygon_cannot_masquerade_as_a_time_solution(objective):
    candidate=polygon();candidate['objective']=objective
    with pytest.raises(ValueError,match='objective'):
        validate(candidate,EmptyWorld(),task())


def test_request_objective_cannot_be_overridden_by_candidate():
    case=task();case['_objective']='minimum_time'
    with pytest.raises(ValueError,match='objective'):
        validate(polygon(),EmptyWorld(),case)


@pytest.mark.parametrize('key',['velocity_limits_rad_s','acceleration_limits_rad_s2','endpoint_velocity_rad_s'])
def test_validator_requires_frozen_motion_contract(key):
    case=task();del case[key]
    with pytest.raises(ValueError):validate(polygon(),EmptyWorld(),case)


@pytest.mark.parametrize('key,value',[
    ('endpoint_tolerance_rad',float('inf')),('feasibility_tolerance',float('nan')),
    ('online_budget_seconds',0),('optimizer_budget_seconds',61),('max_recovery_attempts',-1),
    ('validation_step_rad',float('inf')),('validation_maximum_samples',2),
    ('validation_minimum_samples',False),('validation_minimum_samples',400.0),
    ('drm',{'try_shortcutting':'false'}),('nto',{'solver':'pretend'}),
    ('roadmap',{'map_extent':[1.,0.,1.]}),('eizo',{'tau':1.}),('drm',None)])
def test_malformed_config_rejected_before_any_planning(key,value):
    with pytest.raises((ValueError,TypeError)):
        load_config({key:value})


@pytest.mark.parametrize('kind',['scalar','column','float_bool','infinite_clearance','nan_clearance'])
def test_collision_batch_cannot_broadcast_or_certify_infinite_distance(kind):
    class Broken(EmptyWorld):
        def cuda(self,q):
            if kind=='scalar':return np.bool_(True)
            if kind=='column':return np.ones((len(q),1),bool)
            if kind=='float_bool':return np.ones(len(q))
            return super().cuda(q)
        def clearance(self,q):
            if kind=='infinite_clearance':return np.full(len(q),float('inf'))
            if kind=='nan_clearance':return np.full(len(q),float('nan'))
            return super().clearance(q)
    with pytest.raises(ValueError,match='checker'):
        validate(polygon(),Broken(),task())


@pytest.mark.parametrize('order,knots,n,objective',[
    (4,[0]*4+[.5]*3+[1]*4,7,'minimum_time'),
    (4,[0]*4+[.5]*4+[1]*4,8,'minimum_time'),
    (2,[0,0,.5,.5,1,1],4,'minimum_distance'),
    (2,[0,1,2,3,4],3,'minimum_distance'),
    (4,[0]*4+[1]*4,4,'minimum_distance'),
    (2,[0,0,1,1],2,'minimum_time'),
    (2.5,[0,0,1,1],2,'minimum_distance')])
def test_discontinuous_or_wrong_objective_splines_fail_before_native_construction(order,knots,n,objective):
    with pytest.raises(ValueError):
        sample_candidate(dict(kind='bspline',spline_order=order,knots=knots,
            controls=np.zeros((n,7)),objective=objective))


def test_drake_samples_agree_with_independent_scipy_and_analytic_motion():
    # Cubic q(t)=0.1*(3*s^2-2*s^3), s=t/3: zero endpoint velocity.
    controls=np.zeros((4,7));controls[2:,0]=.1
    knots=np.array([0.]*4+[3.]*4)
    candidate=dict(kind='bspline',controls=controls,knots=knots,spline_order=4,objective='minimum_time')
    samples=sample_candidate(candidate)
    oracle=BSpline(knots,controls,3)
    np.testing.assert_allclose(samples['q'],oracle(samples['time']),atol=1e-14)
    s=samples['time']/3
    np.testing.assert_allclose(samples['q'][:,0],.1*(3*s*s-2*s*s*s),atol=1e-14)
    dense=np.linspace(0,3,10001)
    assert np.max(np.abs(oracle.derivative(1)(dense)))<=np.max(samples['derivatives']['velocity_per_joint'])+1e-14
    assert np.max(np.abs(oracle.derivative(2)(dense)))<=np.max(samples['derivatives']['acceleration_per_joint'])+1e-14
    assert validate(candidate,EmptyWorld(),task())['passed']


@pytest.mark.parametrize('seed',[-1,2**32,True,0.5,'1'])
def test_invalid_seed_cannot_be_coerced_to_another_experiment(seed):
    with pytest.raises(ValueError):_method_seed(seed)


@pytest.mark.parametrize('path',[np.zeros(14),np.zeros((7,2)),np.full((2,7),float('nan')),np.zeros((1,7))])
def test_drm_shape_cannot_be_reshaped_into_a_fake_path(path):
    with pytest.raises(ValueError):_checked_native_path(path,np.array([task()['q_start'],task()['q_goal']]),1e-5)


def test_invalid_map_request_retains_failure_without_touching_model(tmp_path):
    out=tmp_path/'map'
    with pytest.raises(ValueError):build_roadmap(out,nodes=2,seed=-1)
    receipt=json.loads((out/'metadata.json').read_text())
    assert receipt['status']=='failed' and receipt['failed_stage']=='input_or_model_setup'
    assert (out/'exception.txt').is_file()
    with pytest.raises(FileExistsError):build_roadmap(out,nodes=2,seed=0)


def test_failure_in_offline_stage_is_retained_without_overwriting(tmp_path,monkeypatch):
    from benchmarks.generic_motion_v1 import roadmap
    def broken(*args,**kwargs):raise RuntimeError('Independent injected native build failure')
    monkeypatch.setattr(roadmap,'_build_roadmap',broken)
    with pytest.raises(RuntimeError):build_roadmap(tmp_path/'map',nodes=2,seed=0)
    assert json.loads((tmp_path/'map'/'metadata.json').read_text())['status']=='failed'


def test_unlabelled_previous_map_files_are_not_overwritten(tmp_path):
    (tmp_path/'native.map').write_bytes(b'previous partial native evidence')
    with pytest.raises(FileExistsError):build_roadmap(tmp_path,nodes=2,seed=0)
    assert (tmp_path/'native.map').read_bytes()==b'previous partial native evidence'


def test_region_coverage_overflow_cannot_bypass_the_correction_cap():
    class Region:
        def A(self):return np.full((1,7),1e308)
        def b(self):return np.ones(1)
    with pytest.raises(ValueError,match='overflowed'):
        cover_region_anchors(Region(),np.ones((2,7)),1e-6)


def test_nto_actual_cpu_solver_probe_does_not_require_scsplanning(tmp_path):
    record=dependency_audit(tmp_path,'NTO')
    assert record['ipopt']['probe_success']
    assert 'scsplanning' not in record['modules']
    assert record['ipopt']['probe_solution']==pytest.approx(1.,abs=1e-6)


def test_unavailable_solver_is_explicitly_unsupported_and_saves_probe(tmp_path,monkeypatch):
    import pydrake.all as drake
    class Unavailable:
        def available(self):return False
        def enabled(self):return False
        def Solve(self,*args):raise AssertionError('Unavailable solver must never run')
    monkeypatch.setattr(drake,'IpoptSolver',Unavailable)
    with pytest.raises(ImportError,match='unavailable'):
        dependency_audit(tmp_path,'NTO')
    assert json.loads((tmp_path/'dependencies.json').read_text())['ipopt']['available'] is False


def test_nan_budget_saves_input_failure_before_model_or_monitor(tmp_path,monkeypatch):
    from benchmarks.generic_motion_v1.runners import csdecomp_baseline as runner
    def forbidden():raise AssertionError('Input contract must run before monitor or GPU')
    monkeypatch.setattr(runner,'MemoryMonitor',forbidden)
    result=run_case({},tmp_path/'missingmap',tmp_path/'attempt',config={'online_budget_seconds':float('nan')})
    assert result['status']=='infrastructure_error'
    assert result['error_category']=='input_protocol_mismatch'
    assert (tmp_path/'attempt'/'result.json').is_file()
