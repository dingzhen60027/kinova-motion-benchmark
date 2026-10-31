"""Independent corridor connectivity and original-edge repair regressions (CPU)."""
import json
import numpy as np
import pytest
from pydrake.all import HPolyhedron

from benchmarks.generic_motion_v1.runners.csdecomp_baseline import (
    build_active_corridor,recover_active_corridor,_check_corridor,_corridor_anchors,
    _native_option_values,_validate_frontend_path)
from benchmarks.generic_motion_v1.runners.validate import sample_candidate


def points(values):
    array=np.zeros((len(values),7));array[:,0]=values;return array


def box(low,high,ylow=-1.,yhigh=1.):
    lo=np.full(7,-2.);hi=np.full(7,2.)
    lo[:2]=[low,ylow];hi[:2]=[high,yhigh]
    return HPolyhedron.MakeBox(lo,hi)


def test_consecutive_covered_edges_produce_one_real_author_optimization_segment(tmp_path):
    path=points([0,.25,.5,.75,1.]);calls=[]
    def inflate(a,b):calls.append((a.copy(),b.copy()));return box(-.1,1.1)
    regions,mapping=build_active_corridor(path,path[[0,-1]],inflate,1e-6,tmp_path)
    assert len(calls)==len(regions)==1
    assert mapping==[dict(original_seed_edge_index=0,covered_edge_indices=[0,1,2,3])]
    saved=json.loads((tmp_path/'corridor.json').read_text())
    assert saved['active_region_count']==1 and saved['complete']
    assert [event['action'] for event in saved['events'][:-1]]==['native_inflate_original_edge']+['reuse_last_region']*3
    from scsplanning.polygonal import get_points
    from scsplanning import biconvex
    optimized=get_points(path[0],path[-1],regions)
    assert optimized.shape==(2,7)
    curve=biconvex(path[0],path[-1],regions,HPolyhedron.MakeBox(-np.ones(7),np.ones(7)),
                   HPolyhedron.MakeBox(-np.ones(7),np.ones(7)),6,.01)
    assert len(curve.curves)==1
    candidate=dict(kind='bezier',objective='minimum_time',controls=np.asarray([c.points for c in curve.curves],float),
                   durations=np.asarray([c.duration for c in curve.curves],float))
    samples=sample_candidate(candidate)
    np.testing.assert_allclose(samples['q'][[0,-1]],path[[0,-1]],atol=1e-7)
    assert np.unique(samples['segment']).tolist()==[0]


def test_return_to_old_region_does_not_destroy_corridor_order(tmp_path):
    path=np.zeros((5,7));path[:,:2]=[[0,0],[1,0],[1,1],[1,0],[0,0]]
    horizontal=box(-.1,1.1,-.1,.1);vertical=box(.9,1.1,-.1,1.1)
    supplied=iter([horizontal,vertical,horizontal]);calls=[]
    def inflate(a,b):calls.append((a.copy(),b.copy()));return next(supplied)
    regions,mapping=build_active_corridor(path,path[[0,-1]],inflate,1e-6,tmp_path)
    assert len(calls)==3
    assert [entry['covered_edge_indices'] for entry in mapping]==[[0],[1,2],[3]]
    assert [entry['original_seed_edge_index'] for entry in mapping]==[0,1,3]
    assert [x['original_waypoint_index'] for x in _check_corridor(regions,mapping,path,path[[0,-1]])]==[1,3]


def test_cut_shared_waypoint_reinflates_only_affected_original_span(tmp_path):
    path=points([0,.25,.5,.75,1.]);endpoints=path[[0,-1]]
    mapping=[dict(original_seed_edge_index=0,covered_edge_indices=[0,1]),
             dict(original_seed_edge_index=2,covered_edge_indices=[2,3])]
    edited=[box(-.1,.3),box(.4,1.1)] # first seed survives; shared q=.5 was cut
    calls=[]
    def inflate(a,b):
        calls.append((a.copy(),b.copy()))
        return box(a[0]-.01,b[0]+.01)
    regions,updated,events=recover_active_corridor(edited,mapping,path,endpoints,inflate,1e-6,tmp_path)
    assert len(calls)==2 and len(regions)==3
    for i,call in enumerate(calls):np.testing.assert_array_equal(call,path[i:i+2])
    assert [entry['covered_edge_indices'] for entry in updated]==[[0],[1],[2,3]]
    assert events[0]['action']=='reinflate_original_span'
    np.testing.assert_array_equal(regions[-1].A(),edited[-1].A())
    np.testing.assert_array_equal(regions[-1].b(),edited[-1].b())
    saved=json.loads((tmp_path/'accepted/corridor.json').read_text())
    assert saved['complete'] and saved['active_region_count']==3
    with np.load(tmp_path/'regions.npz') as raw:
        np.testing.assert_array_equal(raw['b_0000'],edited[0].b())
    assert json.loads((tmp_path/'anchor_checks/0000/coverage.json').read_text())['status']=='rejected'
    _check_corridor(regions,updated,path,endpoints)


def test_endpoint_quantization_correction_is_bounded_and_keeps_fixed_anchors(tmp_path):
    path=points([0,.25,.5]);endpoints=path[[0,-1]].copy();endpoints[-1,0]+=3e-7
    regions,mapping=build_active_corridor(path,endpoints,lambda a,b:box(-.1,.5),1e-6,tmp_path)
    assert len(regions)==1
    anchors=_corridor_anchors(path,mapping[0],endpoints)
    np.testing.assert_array_equal(anchors[-1],endpoints[-1])
    assert regions[-1].PointInSet(endpoints[-1],1e-12)


def test_failed_native_reinflation_remains_failed_with_original_cut_evidence(tmp_path):
    path=points([0,.25,.5]);mapping=[dict(original_seed_edge_index=0,covered_edge_indices=[0,1])]
    def broken(a,b):raise RuntimeError('native reinflation failed')
    with pytest.raises(RuntimeError,match='native reinflation failed'):
        recover_active_corridor([box(-.1,.3)],mapping,path,path[[0,-1]],broken,1e-6,tmp_path)
    assert (tmp_path/'regions.npz').is_file()
    assert (tmp_path/'anchor_checks/0000/before.npz').is_file()
    assert not (tmp_path/'accepted').exists()


def test_runtime_option_record_includes_unchanged_installed_native_defaults():
    import csdecomp
    options=csdecomp.EizoOptions();values=_native_option_values(options)
    assert values['max_iterations']==options.max_iterations
    assert values['track_iteration_information']==options.track_iteration_information
    assert values['seed']==options.seed


def test_mapping_cannot_skip_an_original_path_edge():
    path=points([0,.25,.5]);bad=[dict(original_seed_edge_index=0,covered_edge_indices=[0])]
    with pytest.raises(ValueError,match='partition'):
        _check_corridor([box(-.1,1.)],bad,path,path[[0,-1]])


def test_frontend_coarse_edge_cannot_seed_inflation_without_dense_geometry_check(tmp_path):
    path=points([0,1.])
    class ThinObstacle:
        lower=np.full(7,-2.);upper=np.full(7,2.);contact_band_m=2e-6
        def clearance(self,q):return np.abs(q[:,0]-.05)-.008
        def cuda(self,q):return self.clearance(q)>0
    model=ThinObstacle()
    # Coarse 0.1 samples miss an obstacle between 0 and 0.1.
    assert model.cuda(points(np.linspace(0,1,11))).all()
    case=dict(q_start=path[0],q_goal=path[-1],_objective='minimum_time',validation_status='valid_input',
              velocity_limits_rad_s=[1.]*7,acceleration_limits_rad_s2=[1.]*7,endpoint_velocity_rad_s=[0.]*7)
    report=_validate_frontend_path(path,model,case,None,tmp_path)
    assert report['status']=='collision_failed'
    assert case['_objective']=='minimum_time'
    assert (tmp_path/'trajectory_samples.npz').is_file()
    assert json.loads((tmp_path/'validation.json').read_text())['cost_unit']=='rad'
