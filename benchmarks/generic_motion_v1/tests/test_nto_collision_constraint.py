"""CPU geometry and real solver regressions for the narrow Drake workaround."""
import json
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from benchmarks.generic_motion_v1.model import Model
from benchmarks.generic_motion_v1.roadmap import HERE
from benchmarks.generic_motion_v1.runners.nto_collision_constraint import (
    RobustMinimumDistanceConstraint, box_box_signed_distance, is_known_boxbox_error,
)


def box_query(size_a,size_b,xa,xb):
    from pydrake.all import (SceneGraph, GeometryFrame, GeometryInstance, Box,
                            RigidTransform, ProximityProperties, FramePoseVector)
    graph=SceneGraph();source=graph.RegisterSource("test")
    frames=[graph.RegisterFrame(source,GeometryFrame(n)) for n in ("A","B")]
    ids=[]
    for frame,size,name in zip(frames,(size_a,size_b),("a","b")):
        identity=graph.RegisterGeometry(source,frame,GeometryInstance(RigidTransform(),Box(*size),name))
        graph.AssignRole(source,identity,ProximityProperties());ids.append(identity)
    context=graph.CreateDefaultContext();poses=FramePoseVector()
    for frame,x in zip(frames,(xa,xb)):poses.set_value(frame,RigidTransform(x))
    graph.get_source_pose_port(source).FixValue(context,poses)
    query=graph.get_query_output_port().Eval(context)
    return graph,context,query,ids


def test_complete_box_distance_matches_native_random_separation_and_penetration():
    from scipy.spatial.transform import Rotation
    rng=np.random.default_rng(20261101)
    both_sides=set(); edge_edge_cases=0
    for i in range(100):
        size_a=rng.uniform(.05,1.,3);size_b=rng.uniform(.05,1.,3)
        xa=np.eye(4);xb=np.eye(4)
        xa[:3,:3]=Rotation.random(random_state=rng).as_matrix()
        xb[:3,:3]=Rotation.random(random_state=rng).as_matrix()
        xb[:3,3]=rng.normal(size=3)*(.12 if i%2 else 1.2)
        graph,context,query,ids=box_query(size_a,size_b,xa,xb)
        pair=query.ComputeSignedDistancePairClosestPoints(*ids)
        expected=pair.distance
        if (expected>0 and np.isclose(np.abs(pair.p_ACa),size_a/2,atol=1e-8).sum()==2
                and np.isclose(np.abs(pair.p_BCb),size_b/2,atol=1e-8).sum()==2):
            edge_edge_cases+=1
        actual=box_box_signed_distance(size_a,xa,size_b,xb)
        both_sides.add(expected>=0)
        assert actual==pytest.approx(expected,abs=2e-8)
        assert box_box_signed_distance(size_b,xb,size_a,xa)==pytest.approx(actual,abs=1e-12)
    assert both_sides=={False,True}
    assert edge_edge_cases>0


def test_diagonal_separation_is_euclidean_not_sat_lower_bound():
    xa=np.eye(4);xb=np.eye(4);xb[:3,3]=[2.,2.,2.]
    assert box_box_signed_distance(np.ones(3),xa,np.ones(3),xb)==pytest.approx(np.sqrt(3))


def test_upstream_barely_penetrating_fixture_returns_near_zero_without_geometry_change():
    # Exact transforms in official Drake issue21675, comment dated2026-10-27.
    xa=np.eye(4);xb=np.eye(4)
    xa[:3,:3]=[[1.0000000000000044,2.2886701630728595e-19,0.],
               [1.0581860424029641e-20,1.0000000000000044,0.],[0.,0.,1.]]
    xa[:3,3]=[3.5000000000000107,2.2500000000000027,.15]
    xb[:3,:3]=[[1.,0.,0.],[0.,-.16393107388774433,.9864717953464309],
               [0.,-.9864717953464309,-.16393107388774433]]
    xb[:3,3]=[3.2750000000000203,2.6252702917956396,.16073032646376256]
    distance=box_box_signed_distance([.05,.55,.3],xa,[.4,.3,.2],xb)
    assert abs(distance)<1e-12
    shifted=xb.copy();shifted[0,3]-=.001
    assert box_box_signed_distance([.05,.55,.3],xa,[.4,.3,.2],shifted)==pytest.approx(.001,abs=1e-12)


@pytest.fixture(scope="module")
def failing_model():
    attempt=HERE/'results/repaired_dev/jobs/dev-route_choice-002-p00-NTO_PAPER-minimum_distance-48b608ce/attempts/0001'
    case=json.loads((attempt/'input.json').read_text())
    model=Model(case['voxels_xyz'],case['voxel_radius_m'])
    q=np.load(HERE/'reports/baseline_study/nto_backend_last_q_probe/last_evaluated_q.npy')
    return model,model.nto_plant(),q


def test_known_real_kinova_assertion_retains_every_pair_and_recovers_autodiff(failing_model,tmp_path):
    from pydrake.autodiffutils import InitializeAutoDiff,ExtractGradient,ExtractValue
    model,native,q=failing_model
    wrapper=RobustMinimumDistanceConstraint(native,0.,.01,tmp_path)
    with pytest.raises((RuntimeError,SystemExit),match='BoxBoxGradient'):
        wrapper.original.Eval(q)
    direct=wrapper.constraint.Eval(q)
    ad=wrapper.constraint.Eval(InitializeAutoDiff(q))
    assert np.isfinite(ExtractGradient(ad)).all()
    assert direct[0]==pytest.approx(float(ExtractValue(ad)[0,0]),abs=1e-12)
    assert abs(direct[0]-1)<1e-8  # Contact constraint is retained, never free.
    assert len(wrapper.pairs)==native.collision_pair_count
    evidence=wrapper.save()
    assert evidence['collision_pairs_dropped']==0
    assert evidence['recovery_events'][-1]['gradient_step_consistency_max_error']<1e-3
    assert set(evidence['recovery_events'][-1]['recovered_pairs'][0]['pair'])=={'robotiq::left_inner_finger_collision','table::Box'}
    # Arbitrary AutoDiff seed chain rule (not just identity seeds).
    seed=np.arange(21,dtype=float).reshape(7,3)/20
    mapped=wrapper.constraint.Eval(InitializeAutoDiff(q,seed))
    np.testing.assert_allclose(ExtractGradient(mapped),ExtractGradient(ad)@seed,atol=1e-12,rtol=0)
    direct_column=wrapper.evaluate(InitializeAutoDiff(q,seed).reshape(7,1))
    np.testing.assert_allclose(ExtractGradient(direct_column),ExtractGradient(ad)@seed,atol=1e-12,rtol=0)
    np.testing.assert_array_equal(wrapper.evaluate(q[:,None]),direct)


def test_ordinary_active_values_and_derivatives_are_bitwise_native(failing_model):
    from pydrake.autodiffutils import InitializeAutoDiff,ExtractGradient,ExtractValue
    model,native,q=failing_model
    wrapper=RobustMinimumDistanceConstraint(native,0.,.01)
    rng=np.random.default_rng(51)
    for i in range(100):
        x=q+rng.normal(0,.0002,7) if i<50 else rng.uniform(model.lower,model.upper)
        ad=InitializeAutoDiff(x)
        actual=wrapper.constraint.Eval(ad);expected=wrapper.original.Eval(ad)
        np.testing.assert_array_equal(ExtractValue(actual),ExtractValue(expected))
        np.testing.assert_array_equal(ExtractGradient(actual),ExtractGradient(expected))
        # Independent pair aggregation is the same native mathematical function.
        fallback,_=wrapper._fallback_scalar(x)
        assert fallback==pytest.approx(expected[0].value(),abs=1e-11)
    assert not wrapper.events


@pytest.mark.parametrize('error',[RuntimeError('different geometry failure'),SystemExit('user stop')])
def test_unrelated_errors_are_never_swallowed(error):
    wrapper=object.__new__(RobustMinimumDistanceConstraint);wrapper.native_evaluations=0;wrapper.num_positions=7
    def fail(q):raise error
    wrapper.original=SimpleNamespace(Eval=fail)
    with pytest.raises(type(error),match=str(error)):
        wrapper.evaluate(np.zeros(7))


@pytest.mark.parametrize('q',[np.zeros((1,7)),np.zeros((7,1,1)),np.zeros(6),np.full(7,np.nan),np.full(7,np.inf)])
def test_malformed_configuration_rejected_before_native_call(q):
    wrapper=object.__new__(RobustMinimumDistanceConstraint);wrapper.num_positions=7
    wrapper.original=SimpleNamespace(Eval=lambda q:pytest.fail('Native call on malformed input'))
    with pytest.raises(ValueError):wrapper.evaluate(q)


def test_recovery_rejects_noncontact_distance_instead_of_dropping_pair(failing_model,monkeypatch,tmp_path):
    import benchmarks.generic_motion_v1.runners.nto_collision_constraint as module
    model,native,q=failing_model
    wrapper=RobustMinimumDistanceConstraint(native,0.,.01,tmp_path)
    monkeypatch.setattr(module,'box_box_signed_distance',lambda *args:.01)
    with pytest.raises(RuntimeError,match='non-contact disagreement'):
        wrapper.constraint.Eval(q)
    record=json.loads((tmp_path/'nto_collision_backend.json').read_text())
    assert 'non-contact disagreement' in record['recovery_events'][-1]['recovery_failure']


@pytest.mark.parametrize('objective',['minimum_distance','minimum_time'])
def test_legacy_nto_uses_same_backend_with_real_ipopt(objective,tmp_path):
    from benchmarks.generic_motion_v1.roadmap import load_config
    from benchmarks.generic_motion_v1.runners.csdecomp_baseline import _nto
    from benchmarks.generic_motion_v1.runners.validate import sample_candidate
    model=Model()
    start=np.asarray(json.loads((HERE/'manifests/dev.jsonl').read_text().splitlines()[0])['q_start'])
    goal=start.copy();goal[0]+=.02
    case=dict(q_start=start.tolist(),q_goal=goal.tolist(),_objective=objective)
    candidate=_nto(np.array([start,goal]),model,case,load_config(),tmp_path,time.perf_counter()+20)
    assert candidate['solver_success']
    sample=sample_candidate(candidate)
    assert np.max(np.abs(sample['q'][[0,-1]]-[start,goal]))<1e-6
    assert model.clearance(sample['q']).min()>.01
    record=json.loads((tmp_path/'nto_collision_backend.json').read_text())
    assert record['native_evaluations']>0
    assert record['collision_pairs']==137 and record['collision_pairs_dropped']==0
