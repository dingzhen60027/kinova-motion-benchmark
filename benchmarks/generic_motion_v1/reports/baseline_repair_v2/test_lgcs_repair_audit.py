"""Tiny algebra/provenance fixtures only; no optimizer or GPU is invoked."""
import copy
import json
from pathlib import Path

import numpy as np
import pytest

from benchmarks.generic_motion_v1.reports.baseline_repair_v2 import audit_lgcs_repair as audit


def write(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value,indent=2)+'\n')


def fixture(directory,status='optimal',dual=.5):
    directory.mkdir(exist_ok=True)
    d=7;start=np.zeros(d);goal=np.zeros(d);goal[0]=.5
    regions=[(np.vstack([np.eye(d),-np.eye(d)]),np.ones(2*d))]
    edges=np.array([[1,0],[0,2]],int)
    np.savez(directory/'graph.npz',edges=edges,interior_edges=np.empty((0,2),int),start=start,goal=goal)
    np.savez(directory/'solver_solution.npz',edges=edges,interior_edges=np.empty((0,2),int),
             flow=np.ones(2),transition=np.empty((0,d)),length=np.array([.5]),delta=goal[None])
    names={'phi_1_0':1.,'phi_0_2':1.,'length_0':.5,**{f'delta_0_{k}':goal[k] for k in range(d)}}
    cip=['VARIABLES']
    cip += [f'  [{"binary" if k.startswith("phi_") else "continuous"}] <{k}>: obj=0;' for k in names]
    cip += ['CONSTRAINTS']+[f' [linear] <row_{i}>: 0 <= 0;' for i in range(39)]
    cip += [' [nonlinear] <soc_0>: (<delta_0_0>*<delta_0_0>)^0.5-<length_0> <= 0;']
    (directory/'model.cip').write_text('\n'.join(cip))
    (directory/'solution.sol').write_text('objective value: 0.5\n'+'\n'.join(f'{k} {v} (obj:0)' for k,v in names.items()))
    residual=dict(finite=True,flow_integrality=0.,flow_bounds=0.,length_nonnegative=0.,soc_norm=0.,
        flow_balance=0.,degree=0.,vector_flow=0.,homogeneous_membership=0.,tolerance=1e-7,passed=True)
    gap=audit.scip_gap(.5,dual)
    m=dict(solver='SCIP',backend='scip',pyscipopt_version='6.3.0',solver_version=[10,0,2],
        graph_vertices=1,graph_edges=2,solver_variables=10,solver_constraints=39,
        solver_soc_constraints=1,solver_nonlinear_constraints=1,solver_quadratic_constraints=0,
        solver_status=status,solver_has_incumbent=True,solver_success=True,
        globally_optimal=status=='optimal',solver_objective=.5,solver_primal_bound=.5,
        solver_dual_bound=dual,gap=gap,solver_parameters={'limits/gap':1e-6},
        algebraic_validation=residual,perspective_membership_rows=28,scip_original_solution_feasible=True,
        segment_regions=[0],selected_regions=[0],discarded_disjoint_cycle_edges=[],
        decoded_membership_violation=0.,decoded_objective_excess=0.,recovered_path_length=.5)
    descriptions={'optimal':'problem is solved [optimal solution found]',
                  'gaplimit':'solving was interrupted [gap limit reached]',
                  'timelimit':'solving was interrupted [time limit reached]'}
    (directory/'scip.log').write_text(f"SCIP Status : {descriptions[status]}\nPrimal Bound : +5.0e-1 (1 solutions)\nDual Bound : {dual:.16g}\nGap : {100*gap:.2f} %\nSolving Time (sec) : 1.23\n")
    np.savez(directory/'candidate.npz',points=np.array([start,goal]))
    write(directory/'lgcs_solver.json',m)
    return regions,np.array([start,goal]),m


def test_valid_raw_fixture_and_status(tmp_path):
    regions,endpoints,m=fixture(tmp_path)
    r=audit.scip_round(tmp_path,regions,endpoints)
    assert r['raw']['decoded'] and r['raw']['cost']==.5 and r['raw']['algebra']['passed']


@pytest.mark.parametrize('status,dual',[('timelimit',.4),('gaplimit',.4999998)])
def test_incumbent_acceptance_does_not_require_optimality(tmp_path,status,dual):
    regions,endpoints,m=fixture(tmp_path,status,dual)
    r=audit.scip_round(tmp_path,regions,endpoints)
    assert r['solver_success'] and not r['globally_optimal']


@pytest.mark.parametrize('field,value',[
    ('globally_optimal',True),('solver_dual_bound',.6),('gap',0.),('solver_status','optimal'),
    ('segment_regions',[1]),('solver_objective',.4),('solver_variables',11),
    ('backend','gurobi'),('pyscipopt_version','6.2.0'),('scip_original_solution_feasible',False)])
def test_mutated_metadata_rejected(tmp_path,field,value):
    regions,endpoints,m=fixture(tmp_path,'timelimit',.4)
    m[field]=value;write(tmp_path/'lgcs_solver.json',m)
    with pytest.raises((ValueError,KeyError,IndexError)):
        audit.scip_round(tmp_path,regions,endpoints)


@pytest.mark.parametrize('field,value',[
    ('flow',np.array([.5,.5])),('delta',np.zeros((1,7))),('length',np.array([.499])),
    ('edges',np.array([[0,1],[0,2]]))])
def test_mutated_raw_solution_rejected(tmp_path,field,value):
    regions,endpoints,m=fixture(tmp_path)
    raw=audit.arrays(tmp_path/'solver_solution.npz');raw[field]=value
    np.savez(tmp_path/'solver_solution.npz',**raw)
    with pytest.raises(ValueError):audit.scip_round(tmp_path,regions,endpoints)


def test_squared_soc_model_rejected(tmp_path):
    regions,endpoints,m=fixture(tmp_path)
    p=tmp_path/'model.cip';p.write_text(p.read_text().replace('^0.5','^2'))
    with pytest.raises(ValueError,match='unsquared'):audit.scip_round(tmp_path,regions,endpoints)


def test_solution_text_and_candidate_rechecked(tmp_path):
    regions,endpoints,m=fixture(tmp_path)
    np.savez(tmp_path/'candidate.npz',points=np.zeros((2,7)))
    with pytest.raises(ValueError,match='decoded raw'):audit.scip_round(tmp_path,regions,endpoints)
    np.savez(tmp_path/'candidate.npz',points=endpoints)
    p=tmp_path/'solution.sol';p.write_text(p.read_text().replace('phi_1_0 1.0','phi_1_0 0.5'))
    with pytest.raises(ValueError,match='raw mismatch'):audit.scip_round(tmp_path,regions,endpoints)


def test_gap_definition_and_textual_percent_are_not_interchangeable(tmp_path):
    assert audit.scip_gap(10.,8.)==.25
    assert audit.scip_gap(0.,0.)==0.
    assert audit.scip_gap(1.,0.) is None
    regions,endpoints,m=fixture(tmp_path,'gaplimit',.4999998)
    assert audit.scip_log(tmp_path/'scip.log')['rounded_gap_percent']==0.
    assert audit.scip_round(tmp_path,regions,endpoints)['gap']>0.
    (tmp_path/'scip.log').write_text('SCIP Status : problem is solved [infeasible]\nPrimal Bound : infinite\nDual Bound : infinite\nGap : infinite\nSolving Time (sec) : 1.\n')
    assert audit.scip_log(tmp_path/'scip.log')['rounded_gap_percent'] is None


def test_no_incumbent_cannot_claim_candidate(tmp_path):
    regions,endpoints,m=fixture(tmp_path)
    m.update(solver_status='infeasible',solver_has_incumbent=False,solver_success=False,
             globally_optimal=False,solver_objective=None,solver_primal_bound=1e20,solver_dual_bound=1e20,gap=0.)
    write(tmp_path/'lgcs_solver.json',m)
    (tmp_path/'scip.log').write_text('SCIP Status : problem is solved [infeasible]\nPrimal Bound : +1e20\nDual Bound : +1e20\nGap : 0.00 %\nSolving Time (sec) : 0.01\n')
    with pytest.raises(ValueError,match='No-incumbent'):
        audit.scip_round(tmp_path,regions,endpoints)
    np.savez(tmp_path/'candidate.npz')
    assert not audit.scip_round(tmp_path,regions,endpoints)['has_incumbent']


def test_missing_footer_is_partial_not_completion(tmp_path):
    regions,endpoints,m=fixture(tmp_path)
    (tmp_path/'scip.log').write_text('presolving:\n')
    assert audit.scip_log(tmp_path/'scip.log')=={'complete_footer':False}
    with pytest.raises(ValueError,match='footer'):audit.scip_round(tmp_path,regions,endpoints)


def rows():
    base=dict(latest=True,split='dev',case_id='dev-one',map_sha256='map',method_seed=5,solver_seed=7,
        case_sha='case',legal=True,family='bottleneck',cold_seconds=2.,source_commit='old',backend='gurobi',
        successful=False,status='unsupported',cost=None)
    new=dict(base,source_commit='new',backend='scip',successful=True,status=audit.SUCCESS,cost=3.,cold_seconds=6.)
    return [new],[base]


def test_pairs_include_failures_but_cost_only_common_success():
    new,previous=rows();p=audit.paired_comparison(new,previous)[0]
    assert p['cost_difference_new_minus_old'] is None
    assert p['cold_difference_new_minus_old']==4.
    previous[0].update(successful=True,status=audit.SUCCESS,cost=4.)
    p=audit.paired_comparison(new,previous)[0]
    assert p['cost_difference_new_minus_old']==-1.


@pytest.mark.parametrize('field,value',[('case_sha','changed'),('map_sha256','changed'),('method_seed',9),('legal',False)])
def test_changed_query_or_map_cannot_be_paired(field,value):
    new,previous=rows();new[0][field]=value
    with pytest.raises(ValueError):audit.paired_comparison(new,previous)


def test_runtime_mixing_rejected():
    new,previous=rows();new.append(dict(new[0],case_id='dev-two',source_commit='mixed'))
    previous.append(dict(previous[0],case_id='dev-two'))
    with pytest.raises(ValueError,match='different runtimes'):audit.paired_comparison(new,previous)


def test_incomplete_run_rejected_before_hashing_results(tmp_path):
    manifest=dict(jobs=[{'job_id':'job'}],source_hashes={})
    write(tmp_path/'run_manifest.json',manifest)
    write(tmp_path/'identity.json',{'run_manifest_sha256':audit.digest(tmp_path/'run_manifest.json')})
    with pytest.raises(ValueError,match='incomplete'):audit.run_preflight(tmp_path)


def test_sealed_receipt_detects_changed_unlisted_and_unsafe_files(tmp_path):
    write(tmp_path/'result.json',{'status':'infrastructure_error'})
    write(tmp_path/'receipt.json',dict(status='infrastructure_error',retry_pending=True,
        files={'result.json':audit.digest(tmp_path/'result.json')}))
    audit.receipt_check(tmp_path,{}, {},{})
    (tmp_path/'unlisted').write_text('extra')
    with pytest.raises(ValueError,match='inventory'):audit.receipt_check(tmp_path,{}, {},{})
    (tmp_path/'unlisted').unlink();write(tmp_path/'result.json',{'status':'success'})
    with pytest.raises(ValueError,match='SHA mismatch'):audit.receipt_check(tmp_path,{}, {},{})
    write(tmp_path/'receipt.json',dict(files={'../outside':'x','result.json':'x'}))
    with pytest.raises(ValueError,match='Unsafe'):audit.receipt_check(tmp_path,{}, {},{})
