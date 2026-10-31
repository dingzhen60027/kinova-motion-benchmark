"""Real optimizer regression for the float32 region / float64 endpoint boundary."""
import json
from pathlib import Path
import numpy as np
import pytest
from pydrake.all import HPolyhedron
from scsplanning.polygonal import get_points
from scsplanning import biconvex
from benchmarks.generic_motion_v1.runners.csdecomp_baseline import cover_region_anchors,_region_anchors


def test_bound_is_normal_scale_invariant_and_records_originals(tmp_path):
    A=np.vstack([np.eye(7),-np.eye(7)])
    b=np.ones(14)
    anchor=np.zeros(7);anchor[0]=1.+3e-8
    A[0]*=1e4;b[0]*=1e4
    region=HPolyhedron(A,b)
    answer=cover_region_anchors(region,[anchor],1e-6,tmp_path)
    np.testing.assert_array_equal(answer.A(),A)
    assert np.max(answer.A()@anchor-answer.b())<=1e-12
    before=np.load(tmp_path/'before.npz');after=np.load(tmp_path/'after.npz')
    np.testing.assert_array_equal(before['A'],after['A'])
    np.testing.assert_array_equal(before['anchors'],after['anchors'])
    report=json.loads((tmp_path/'coverage.json').read_text())
    assert report['status']=='accepted'
    assert report['maximum_normalized_correction']==pytest.approx(3e-8)
    assert report['corrected_planes']==1


def test_large_region_error_fails_without_fabricated_after(tmp_path):
    region=HPolyhedron.MakeBox(-np.ones(7),np.ones(7))
    anchor=np.zeros(7);anchor[0]=1.00002
    with pytest.raises(ValueError,match='exceeding fixed numerical tolerance'):
        cover_region_anchors(region,[anchor],1e-6,tmp_path)
    assert (tmp_path/'before.npz').exists()
    assert not (tmp_path/'after.npz').exists()
    assert json.loads((tmp_path/'coverage.json').read_text())['status']=='rejected'


def test_endpoint_regression_with_real_author_solvers(tmp_path):
    # A realistic fixed endpoint differs from its native float32 projection.
    # The halfspace is valid for the seed but excludes the original endpoint.
    start=np.array([1.10000007,0,0,0,0,0,0])
    goal=np.zeros(7)
    native=start.astype(np.float32).astype(float)
    assert native[0]<start[0]
    upper=np.ones(7);upper[0]=native[0]
    original=HPolyhedron.MakeBox(-np.ones(7),upper)
    # Native seed feasibility is not equivalent to the fixed input equality.
    assert np.max(original.A()@native-original.b())<=0
    assert np.max(original.A()@start-original.b())>0
    with pytest.raises(ValueError,match='Infeasible'):
        get_points(start,goal,[original])
    fixed=cover_region_anchors(original,[native,goal,start,goal],1e-6,tmp_path)
    points=get_points(start,goal,[fixed])
    np.testing.assert_allclose(points[[0,-1]],[start,goal],atol=1e-8)
    limits=HPolyhedron.MakeBox(-np.ones(7),np.ones(7))
    curve=biconvex(start,goal,[fixed],limits,limits,6,.01)
    np.testing.assert_allclose(np.asarray(curve.curves[0].points[0],float),start,atol=1e-7)
    np.testing.assert_allclose(np.asarray(curve.curves[-1].points[-1],float),goal,atol=1e-7)
    assert curve.duration>0


def test_all_regions_keep_native_seed_and_only_global_endpoints_added():
    path=np.arange(28,dtype=float).reshape(4,7)
    fixed=np.array([path[0]+1e-8,path[-1]-1e-8])
    first=_region_anchors(path,0,fixed)
    middle=_region_anchors(path,1,fixed)
    last=_region_anchors(path,2,fixed)
    np.testing.assert_array_equal(first[:2],path[:2])
    np.testing.assert_array_equal(first[2],fixed[0])
    np.testing.assert_array_equal(middle,path[1:3])
    np.testing.assert_array_equal(last[:2],path[2:])
    np.testing.assert_array_equal(last[2],fixed[1])


def test_reused_region_and_recovery_can_restore_tiny_endpoint_roundoff(tmp_path):
    region=HPolyhedron.MakeBox(-np.ones(7),np.ones(7))
    native=np.zeros(7);native[0]=1.
    fixed=native.copy();fixed[0]+=2e-8
    recovered=cover_region_anchors(region,[native,fixed],1e-6,tmp_path/'recovery')
    # A later region reuse must cover its own additional anchor, too.
    new_anchor=native.copy();new_anchor[1]=1.+4e-8
    reused=cover_region_anchors(recovered,[native,fixed,new_anchor],1e-6,tmp_path/'reuse')
    assert np.max(reused.A()@np.array([native,fixed,new_anchor]).T-reused.b()[:,None])<=1e-12
    np.testing.assert_array_equal(region.A(),reused.A())
