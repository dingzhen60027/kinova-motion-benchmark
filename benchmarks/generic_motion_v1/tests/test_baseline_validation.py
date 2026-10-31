"""Small numerical contracts, including failures hidden by coarse sampling."""
import numpy as np
import pytest
from benchmarks.generic_motion_v1.runners.validate import sample_candidate,validate
from benchmarks.generic_motion_v1.runners.csdecomp_baseline import dependency_audit
from benchmarks.generic_motion_v1.roadmap import load_config

class Model:
    lower=np.full(7,-10.)
    upper=np.full(7,10.)
    contact_band_m=2e-6
    def cuda(self,q):return self.clearance(q)>0
    def clearance(self,q):return np.full(len(q),.1)


def case(points):return dict(q_start=np.array(points)[0],q_goal=np.array(points)[-1],
    velocity_limits_rad_s=[1.]*7,acceleration_limits_rad_s2=[1.]*7,endpoint_velocity_rad_s=[0.]*7)


def test_polygon_joint_step_and_exact_cost():
    p=np.array([[0]*7,[1,0,0,0,0,0,0],[1,2,0,0,0,0,0]],float)
    c=dict(kind='polygon',points=p,objective='minimum_distance')
    s=sample_candidate(c)
    assert s['cost']==3.
    assert np.max(np.linalg.norm(np.diff(s['q'],axis=0),axis=1))<=.005+1e-12
    r=validate(c,Model(),case(p))
    assert r['status']=='success_validated_numerical'
    assert r['cost']==3.


def test_endpoints_and_collisions_rejected():
    p=np.array([[0]*7,[1,0,0,0,0,0,0]],float)
    c=dict(kind='polygon',points=p,objective='minimum_distance')
    class Thin(Model):
        def clearance(self,q):return np.abs(q[:,0]-.5)-.002
    assert validate(c,Thin(),case(p))['status']=='collision_failed'
    wrong=case(p);wrong['q_goal']=np.ones(7)
    assert validate(c,Model(),wrong)['status']=='invalid_endpoint'


def test_contact_band_not_silent_success():
    class Touching(Model):
        def clearance(self,q):return np.full(len(q),1e-7)
    p=np.array([[0]*7,[1,0,0,0,0,0,0]],float)
    assert validate(dict(kind='polygon',points=p,objective='minimum_distance'),Touching(),case(p))['status']=='validation_undetermined'


def test_bezier_endpoint_rest_and_derivative_scaling():
    controls=np.zeros((1,4,7));controls[0,2:,0]=1
    c=dict(kind='bezier',controls=controls,durations=np.array([3.]),objective='minimum_time')
    r=validate(c,Model(),case(controls[0,[0,-1]]))
    assert r['passed']
    assert r['derivative_bounds']['endpoint_velocity']==0
    assert r['cost']==3
    c['durations']=np.array([.3])
    assert validate(c,Model(),case(controls[0,[0,-1]]))['status']=='limit_failed'


def test_bezier_continuity_checked():
    controls=np.zeros((2,4,7));controls[1,:,0]=.1
    c=dict(kind='bezier',controls=controls,durations=np.ones(2),objective='minimum_time')
    assert validate(c,Model(),case([controls[0,0],controls[1,-1]]))['status']=='limit_failed'


def test_sampling_cap_rejects_without_allocation():
    p=np.array([[0]*7,[100000,0,0,0,0,0,0]],float)
    with pytest.raises(ValueError,match='cap'):
        sample_candidate(dict(kind='polygon',points=p,objective='minimum_distance'))
    with pytest.raises(ValueError,match='Malformed'):
        sample_candidate(dict(kind='bezier',controls=np.zeros((1,4,7)),durations=np.array([0.]),objective='minimum_time'))


def test_official_author_lscs_and_scsto_actual_open_solver(tmp_path):
    from pydrake.all import HPolyhedron
    from scsplanning.polygonal import get_points
    from scsplanning import biconvex
    a=np.zeros(7);b=np.array([1.,0,0,0,0,0,0])
    regions=[HPolyhedron.MakeBox(np.full(7,-2.),np.full(7,2.))]
    points=get_points(a,b,regions)
    np.testing.assert_allclose(points[[0,-1]],[a,b],atol=1e-7)
    curve=biconvex(a,b,regions,HPolyhedron.MakeBox(-np.ones(7),np.ones(7)),HPolyhedron.MakeBox(-np.ones(7),np.ones(7)),6,.01)
    c=dict(kind='bezier',controls=np.array([p.points for p in curve.curves],float),durations=np.array([p.duration for p in curve.curves]),objective='minimum_time')
    assert validate(c,Model(),case([a,b]))['passed']
    audit=dependency_audit(tmp_path)
    assert audit['clarabel']['probe_success']
    assert audit['scsplanning_solver']=='Clarabel'


def test_order_two_bspline_has_exact_pwl_cost():
    from pydrake.all import BsplineBasis,KnotVectorType
    p=np.array([[0]*7,[1,0,0,0,0,0,0],[1,1,0,0,0,0,0]],float)
    c=dict(kind='bspline',controls=p,knots=np.array(BsplineBasis(2,3,KnotVectorType.kClampedUniform,0.,1.).knots()),spline_order=2,objective='minimum_distance')
    r=validate(c,Model(),case(p))
    assert r['passed'] and r['cost']==2.


def test_config_typos_rejected():
    with pytest.raises(ValueError,match='Unknown'):
        load_config({'eizo':{'particle_count':100}})


def test_generation_failure_preserved_without_model_or_gpu(tmp_path):
    from benchmarks.generic_motion_v1.runners.csdecomp_baseline import run_case
    case=dict(problem_id='invalid-case',scene_id='scene',family='dense_clutter',
              validation_status='invalid_endpoint',q_start=None,q_goal=None)
    result=run_case(case,tmp_path/'nonexistent-map',tmp_path/'failure')
    assert result['status']=='invalid_endpoint'
    assert not result['frontend_success']
    assert 'model_setup_seconds' not in result['timing']
