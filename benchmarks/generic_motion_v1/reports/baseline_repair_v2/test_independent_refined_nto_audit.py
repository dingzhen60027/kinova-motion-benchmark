"""Temporary CPU arrays only: no planner, solver, Drake or real run reads."""
from copy import deepcopy
import json
from pathlib import Path

import numpy as np
import pytest

from benchmarks.generic_motion_v1.reports.baseline_repair_v2 import independent_refined_nto_audit as module


def write(path, value):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(module.clean(value)))


@pytest.fixture
def valid_attempt(tmp_path):
    p=np.zeros((2,7));p[1,0]=.02
    knots=np.array([0.,0.,1.,1.])
    t=np.linspace(0,1,5);q=t[:,None]*p[1]
    clear=np.full(len(t),.002)
    target=tmp_path/'refinement/round_00';target.mkdir(parents=True)
    np.save(tmp_path/'path.npy',p)
    cfg=dict(optimizer_budget_seconds=60.,validation_step_rad=.005,
             endpoint_tolerance_rad=1e-5,feasibility_tolerance=1e-5)
    case=dict(q_start=p[0].tolist(),q_goal=p[1].tolist(),voxels_xyz=[])
    job=dict(job_id='fixture',evaluation_config=cfg,objective='minimum_distance',case=case)
    result=dict(status=module.SUCCESS,frontend_success=True)
    profile=dict(name=module.PROFILE,clearance_m=.001,refinement_guard_m=.0005,
                 maximum_native_solves=4,maximum_collocation_points=600,
                 maximum_iterations=2000,snopt_total_iterations=100000,
                 snopt_minor_iterations=500,influence_distance_m=.01,
                 solver_feasibility_tolerance=1e-6,warm_start_resampled=False,
                 maximum_optimizer_seconds=60.)
    locations=np.arange(1,150)/150
    parameters=dict(profile=profile,parameters={"Iterations limit":100000,
                    "Major iterations limit":2000,"Minor iterations limit":500,
                    "Major feasibility tolerance":1e-6,"Major optimality tolerance":1e-6,
                    "Time limit":28.},solver='SNOPT',spline_order=2,
                    collision_pairs=137,legacy_cfg_nto_used=False,
                    collocation_locations=locations,warm_start_node_count=2,
                    warm_start_control_count=2,warm_start_endpoint_copies=0,
                    warm_start_resampled=False)
    write(target/'nto_parameters.json',parameters)
    initial=dict(original_path=p,controls=p,knots=knots,duration=np.array(1.))
    np.savez(target/'nto_initial.npz',**initial);np.savez(tmp_path/'nto_initial.npz',**initial)
    native=dict(solver_success=True,solver_status='SolutionResult.kSolutionFound',solver_objective=.02)
    write(target/'nto_solver_result.json',native)
    np.savez(target/'nto_solver_raw.npz',controls=p,normalized_knots=knots,duration=np.array(1.),decision_variables=p.ravel())
    candidate=dict(kind='bspline',objective='minimum_distance',profile=module.PROFILE,
                   spline_order=2,duration=1.,solver_success=True,
                   solver_status=native['solver_status'],refinement_native_solves=1)
    for folder in (target,tmp_path):
        write(folder/'candidate.json',candidate)
        np.savez(folder/'candidate.npz',controls=p,knots=knots,duration=np.array(1.))
    np.savez(target/'refinement_geometry.npz',q=q,parameter=t,clearance=clear)
    guard=dict(passed=True,minimum_clearance_m=.002,below_guard_samples=0,seconds=.2)
    write(target/'refinement_geometry.json',guard)
    row=dict(index=0,directory='refinement/round_00',remaining_seconds=29.,
             collocation_locations=locations,collocation_points=149,status='completed',
             added_collocation_locations=[],proposed_collocation_locations=[],
             solve_and_construction_seconds=1.)
    record=dict(profile=profile,final_validation_required=True,geometry_model_changed=False,
                controls_inserted=False,initial_remaining_seconds=30.,rounds=[row],
                stopping_reason='geometry_guard_passed',elapsed_seconds=2.)
    write(tmp_path/'nto_refinement.json',record)
    validation=dict(passed=True,status=module.SUCCESS,cost=.02,endpoint_error_rad=0.,
                    model_disagreements=0,derivative_bounds={})
    write(tmp_path/'validation.json',validation)
    np.savez(tmp_path/'trajectory_samples.npz',q=q,time=t,clearance=clear,
             native=np.ones(len(t),bool),independent=np.ones(len(t),bool))
    return tmp_path,job,result


def run_fixture(fixture):
    audit=module.Audit()
    report=module.check_refined_attempt(audit,*fixture)
    return audit,report


def test_independent_valid_curve_and_policy(valid_attempt):
    audit,report=run_fixture(valid_attempt)
    assert audit.issues==[]
    assert report['reconstructed_final']
    assert audit.curves==1
    assert audit.maximum_curve_error==0


@pytest.mark.parametrize('corrupt_bound',[False,True])
def test_cubic_time_reconstruction_and_derivative_bounds(valid_attempt,corrupt_bound):
    attempt,job,result=valid_attempt
    target=attempt/'refinement/round_00'
    job['objective']='minimum_time'
    job['evaluation_config'].update(velocity_limit_rad_s=1.,acceleration_limit_rad_s2=1.)
    p=np.load(attempt/'path.npy')
    controls=np.vstack([p[0],p,p[-1]])
    duration=.4
    knots=np.r_[np.zeros(4),np.full(4,duration)]
    initial=dict(original_path=p,controls=controls,knots=np.r_[np.zeros(4),np.full(4,2.02)],duration=np.array(2.02))
    for folder in (target,attempt):np.savez(folder/'nto_initial.npz',**initial)
    params=module.read(target/'nto_parameters.json')
    params.update(parameters=dict(max_iter=2000,tol=1e-6,max_cpu_time=28.),solver='IPOPT',
                  spline_order=4,warm_start_control_count=4,warm_start_endpoint_copies=2)
    write(target/'nto_parameters.json',params)
    native=module.read(target/'nto_solver_result.json');native['solver_objective']=duration
    write(target/'nto_solver_result.json',native)
    np.savez(target/'nto_solver_raw.npz',controls=controls,normalized_knots=knots/duration,
             duration=np.array(duration),decision_variables=controls.ravel())
    candidate=module.read(attempt/'candidate.json')
    candidate.update(objective='minimum_time',spline_order=4,duration=duration)
    for folder in (target,attempt):
        write(folder/'candidate.json',candidate)
        np.savez(folder/'candidate.npz',controls=controls,knots=knots,duration=np.array(duration))
    s=np.linspace(0,1,9);q=(3*s*s-2*s*s*s)[:,None]*p[1]
    clear=np.full(len(s),.002)
    np.savez(target/'refinement_geometry.npz',q=q,parameter=s,clearance=clear)
    np.savez(attempt/'trajectory_samples.npz',q=q,time=s*duration,clearance=clear,
             native=np.ones(len(s),bool),independent=np.ones(len(s),bool))
    validation=module.read(attempt/'validation.json')
    # Cubic Bezier derivative control hull: v={0, 3D/T, 0}; a={6D/T²,-6D/T²}.
    v=np.zeros(7);v[0]=.15
    a=np.zeros(7);a[0]=.75
    if corrupt_bound:v[0]=.01
    validation.update(cost=duration,derivative_bounds=dict(velocity_per_joint=v,
                      acceleration_per_joint=a,endpoint_velocity=0.))
    write(attempt/'validation.json',validation)
    audit,report=run_fixture(valid_attempt)
    assert bool(audit.issues)==corrupt_bound,audit.issues
    assert report['reconstructed_final']


@pytest.mark.parametrize('corruption',['native_failure','initial_path','initial_endpoint',
                                     'inherited_final_controls','budget_reset','too_many_points'])
def test_forged_success_or_modified_input_is_detected(valid_attempt,corruption):
    attempt,job,result=valid_attempt
    target=attempt/'refinement/round_00'
    if corruption=='native_failure':
        raw=module.read(target/'nto_solver_result.json');raw['solver_success']=False
        write(target/'nto_solver_result.json',raw)
    elif corruption in ('initial_path','initial_endpoint'):
        raw=module.arrays(target/'nto_initial.npz')
        raw['original_path' if corruption=='initial_path' else 'controls'][0,0]+=.1
        np.savez(target/'nto_initial.npz',**raw)
    elif corruption=='inherited_final_controls':
        raw=module.arrays(attempt/'candidate.npz');raw['controls'][1,0]+=.1
        np.savez(attempt/'candidate.npz',**raw)
    elif corruption=='budget_reset':
        rec=module.read(attempt/'nto_refinement.json');rec['rounds'][0]['remaining_seconds']=31.
        write(attempt/'nto_refinement.json',rec)
    else:
        rec=module.read(attempt/'nto_refinement.json')
        rec['rounds'][0].update(collocation_points=601,collocation_locations=np.linspace(.001,.999,601))
        write(attempt/'nto_refinement.json',rec)
    audit,_=run_fixture(valid_attempt)
    assert audit.issues,corruption


def test_interrupted_last_round_cannot_inherit_first_success(valid_attempt):
    attempt,job,result=valid_attempt
    rec=module.read(attempt/'nto_refinement.json')
    row=deepcopy(rec['rounds'][0]);row.update(index=1,directory='refinement/round_01',
                                           remaining_seconds=20.,status='running')
    rec['rounds'].append(row)
    write(attempt/'nto_refinement.json',rec)
    (attempt/'refinement/round_01').mkdir()
    audit,_=run_fixture(valid_attempt)
    assert any('inherited' in x['message'] or 'missing parameters' in x['message'] for x in audit.issues)


@pytest.mark.parametrize('watchdog', [True, False])
def test_killed_first_native_call_need_not_publish_top_initial_copy(valid_attempt,watchdog):
    attempt,job,result=valid_attempt
    target=attempt/'refinement/round_00'
    result['status']='timeout'
    rec=module.read(attempt/'nto_refinement.json')
    rec['rounds'][0]['status']='running'
    rec['stopping_reason']='running'
    write(attempt/'nto_refinement.json',rec)
    write(attempt/'process.json',dict(timed_out=watchdog))
    for path in (attempt/'nto_initial.npz',attempt/'candidate.json',attempt/'candidate.npz',
                 target/'nto_solver_result.json',target/'nto_solver_raw.npz',
                 target/'candidate.json',target/'candidate.npz',
                 target/'refinement_geometry.json',target/'refinement_geometry.npz'):
        path.unlink()
    audit,report=run_fixture(valid_attempt)
    assert bool(audit.issues) is (not watchdog),audit.issues
    assert report['reconstructed_final'] is False
    if watchdog:assert report['rounds'][0]['top_initial_copy_absent_after_watchdog'] is True


def test_more_than_four_solve_rounds_is_detected(valid_attempt):
    attempt,job,result=valid_attempt
    rec=module.read(attempt/'nto_refinement.json')
    for i in range(1,5):
        row=deepcopy(rec['rounds'][0]);row.update(index=i,directory=f'refinement/round_{i:02d}',
                                               remaining_seconds=29.-i,status='running')
        rec['rounds'].append(row);(attempt/row['directory']).mkdir()
    write(attempt/'nto_refinement.json',rec)
    audit,_=run_fixture(valid_attempt)
    assert any('solve/directory count' in x['message'] for x in audit.issues)


@pytest.mark.parametrize('corruption',['duplicate','changed_endpoint','changed_path'])
def test_methods_cannot_change_task_or_path(corruption):
    rows=[dict(method=m,objective=o,input_digest='same-task',path_sha256='same-path',
               frontend_accepted=True) for m,o in sorted(module.METHOD_PAIRS)]
    if corruption=='duplicate':rows[0]=deepcopy(rows[1])
    elif corruption=='changed_endpoint':rows[0]['input_digest']='different-endpoint'
    else:rows[0]['path_sha256']='different-path'
    audit=module.Audit()
    module.check_shared_groups(audit,{('dev','fixture','roadmap'):rows})
    assert audit.issues


def test_receipt_rejects_modified_raw_file(tmp_path):
    write(tmp_path/'result.json',dict(status='solver_failed'))
    (tmp_path/'raw.txt').write_text('actual failure')
    files={p.name:module.sha(p) for p in tmp_path.iterdir()}
    write(tmp_path/'receipt.json',dict(status='solver_failed',files=files))
    audit=module.Audit();audit.receipt(tmp_path)
    assert not audit.issues
    (tmp_path/'raw.txt').write_text('forged success')
    audit=module.Audit();audit.receipt(tmp_path)
    assert any('SHA' in x['message'] for x in audit.issues)


def test_interval_rule_uses_worst_sample_and_total_cap():
    got=module.expected_additions(np.array([.5]),np.array([.1,.2,.6,.7]),
                                 np.array([.0004,-.01,-.02,-.01]))
    np.testing.assert_array_equal(got,[.2,.6])
    loc=np.arange(1,600)/600
    t=(np.r_[0.,loc]+np.r_[loc,1.])/2
    assert len(module.expected_additions(loc,t,-np.arange(len(t))))==1
