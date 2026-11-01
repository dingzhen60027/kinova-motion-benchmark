"""Integration contract regressions with explicit CPU-only model/planner mocks."""
import json
from pathlib import Path
import numpy as np
import pytest
from benchmarks.generic_motion_v1 import batch, model as model_module
from benchmarks.generic_motion_v1.common import read, write
from benchmarks.generic_motion_v1.roadmap import HERE, load_config
from benchmarks.generic_motion_v1.runners import csdecomp_baseline as runner
from benchmarks.generic_motion_v1.runners import ompl_baselines as ompl

@pytest.fixture
def manifest(tmp_path, monkeypatch):
    monkeypatch.setattr(batch, 'code_inventory', lambda: {})
    case = dict(problem_id='fixture-p0', split='smoke', scene_id='fixture-s0', family='sparse_clutter',
                paired_seeds=[dict(drm_seed=i, method_seed=81237+i, solver_seed=9129+i) for i in range(2)])
    path = tmp_path/'cases.jsonl'; path.write_text(json.dumps(case)+'\n')
    return path

def test_independent_only_prepare_needs_no_roadmap_and_retains_seeds(manifest, tmp_path):
    root = batch.prepare(manifest, None, tmp_path/'independent', methods=['RRTCONNECT','RRTSTAR'],
                         objectives=['minimum_distance','minimum_time'], seed_indices=[0,1])
    plan = read(root/'run_manifest.json')
    assert len(plan['jobs']) == 4
    assert plan['roadmap_files'] == {}
    assert {(j['replica_seed'],j['method_seed']) for j in plan['jobs']} == {(0,81237),(1,81238)}
    assert all(j['roadmap_dir'] is None and j['roadmap_used'] is False for j in plan['jobs'])
    assert len(plan['omitted_unsupported_combinations']) == 2
    assert batch.verify(root)['integrity_passed']

def test_implicit_independent_seed_index_is_zero(manifest, tmp_path):
    root = batch.prepare(manifest, [], tmp_path/'independent', methods=['RRTCONNECT'], objectives=['minimum_distance'])
    assert read(root/'run_manifest.json')['jobs'][0]['replica_seed'] == 0

@pytest.mark.parametrize('method',['EI_ZO','CHOMP','STOMP','TRAJOPT'])
def test_drm_methods_never_silently_run_without_map(manifest, tmp_path, method):
    with pytest.raises(ValueError, match='require --roadmaps'):
        batch.prepare(manifest, None, tmp_path/method, methods=[method], objectives=['minimum_distance'])

@pytest.mark.parametrize('indices',[[-1],[True],[.5],[0,0],[]])
def test_invalid_independent_seed_indices_rejected(manifest, tmp_path, indices):
    with pytest.raises(ValueError):
        batch.prepare(manifest, None, tmp_path/'bad', methods=['RRTSTAR'], objectives=['minimum_distance'],seed_indices=indices)

def test_undeclared_seed_pairing_rejected(manifest, tmp_path):
    with pytest.raises(ValueError, match='seed'):
        batch.prepare(manifest, None, tmp_path/'bad',methods=['RRTSTAR'],objectives=['minimum_distance'],seed_indices=[9])

def test_sampling_configuration_validation_fails_before_preparing_jobs(manifest, tmp_path):
    with pytest.raises(ValueError, match='motion step'):
        batch.prepare(manifest,None,tmp_path/'bad', methods=['RRTSTAR'],objectives=['minimum_distance'],
                      config={'ompl':{'motion_step_rad':.5}})
    assert not (tmp_path/'bad').exists()

@pytest.fixture
def mock_execution(monkeypatch):
    case = json.loads((HERE/'manifests/smoke.jsonl').read_text().splitlines()[0])
    class MockModel:
        def __init__(self,*args):
            self.robot_sha256=case['robot_sha256']; self.collision_geometry_sha256=case['collision_geometry_sha256']
            self.lower=np.full(7,-100.); self.upper=np.full(7,100.); self.contact_band_m=2e-6
        def cuda(self,q): return np.ones(len(q),bool)
        def clearance(self,q): return np.full(len(q),.1)
    class MockMemory:
        def __enter__(self): return self
        def __exit__(self,*args): pass
        def report(self): return {'fixture_only':True}
    monkeypatch.setattr(model_module,'Model',MockModel)
    monkeypatch.setattr(runner,'MemoryMonitor',MockMemory)
    monkeypatch.setattr(runner,'dependency_audit',lambda *a,**k: {'fixture_only':True})
    def solve(model,case,cfg,out,deadline,seed,method):
        return dict(kind='polygon',points=np.asarray([case['q_start'],case['q_goal']]),
                    objective='minimum_distance',solver_success=True,fixture_only=True)
    monkeypatch.setattr(ompl,'solve',solve)
    def validate(candidate,model,case,cfg,out):
        result=dict(status='success_validated_numerical',passed=True,cost=1.,cost_unit='rad',path_length_rad=1.,
                    normalized_path_length=.1,minimum_sampled_clearance_m=.1,duration_s=None,fixture_only=True)
        write(out/'validation.json',result)
        return result
    monkeypatch.setattr(runner,'validate',validate)
    return case

@pytest.mark.parametrize('method',['RRTCONNECT','RRTSTAR'])
def test_independent_runner_does_not_touch_nonexistent_roadmap(mock_execution,tmp_path,method):
    result=runner.run_case(mock_execution,tmp_path/'does-not-exist',tmp_path/'attempt',method=method,seed=17)
    assert result['status']=='success_validated_numerical'
    assert result['roadmap_used'] is False
    assert result['frontend_success'] is None and result['frontend_native_success'] is None
    assert 'roadmap_load_seconds' not in result['timing']
    assert result['timing']['sampling_planning_seconds'] >= 0
    assert result['timing']['validation_seconds'] >= 0
    assert not (tmp_path/'attempt/path.npy').exists()

@pytest.mark.parametrize('error,status',[(ompl.OmplNoExactSolution('Approximate solution'),'planner_timeout'),
    (ompl.OmplNoExactSolution('Timeout'),'planner_timeout'),(ompl.OmplNoExactSolution('Invalid start'),'infrastructure_error'),
    (TimeoutError('No online time remains after reserving common validation'),'timeout'),
    (TimeoutError('deadline'),'infrastructure_error'),(RuntimeError('CUDA device failure'),'infrastructure_error')])
def test_native_sampling_failures_keep_terminal_class_and_raw_exception(mock_execution,tmp_path,monkeypatch,error,status):
    def failed(*a,**k): raise error
    monkeypatch.setattr(ompl,'solve',failed)
    result=runner.run_case(mock_execution,None,tmp_path/'attempt',method='RRTSTAR',seed=17)
    assert result['status']==status
    assert result['failed_stage']=='sampling_planning'
    assert result['frontend_success'] is None
    assert (tmp_path/'attempt/exception.txt').exists()
    assert not (tmp_path/'attempt/candidate.json').exists()

def test_invalid_endpoint_is_saved_without_invoking_model_or_planner(mock_execution,tmp_path,monkeypatch):
    case=dict(mock_execution,q_start=None,q_goal=None,validation_status='invalid_endpoint')
    monkeypatch.setattr(model_module,'Model',lambda *a,**k: pytest.fail('Invalid input must not instantiate model'))
    monkeypatch.setattr(ompl,'solve',lambda *a,**k: pytest.fail('Invalid input must not be replanned'))
    result=runner.run_case(case,None,tmp_path/'attempt',method='RRTCONNECT')
    assert result['status']=='invalid_endpoint' and result['roadmap_used'] is False
    assert result['frontend_success'] is None and result['frontend_native_success'] is None

def test_invalid_ompl_config_records_independent_frontend_identity(mock_execution,tmp_path):
    result=runner.run_case(mock_execution,None,tmp_path/'attempt',method='RRTCONNECT',config={'ompl':{'range_rad':-1}})
    assert result['status']=='infrastructure_error'
    assert result['roadmap_used'] is False
    assert result['frontend_success'] is None and result['frontend_native_success'] is None
