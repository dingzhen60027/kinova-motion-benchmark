"""Native linear-system routing is explicit without changing planning inputs."""
import hashlib
import json
from pathlib import Path

import numpy as np
import pytest

from benchmarks.generic_motion_v1.roadmap import load_config
from benchmarks.generic_motion_v1.runners.ipopt_options import DEFAULTS, evidence, settings


def test_default_is_materialized_and_returned_as_independent_data():
    assert load_config()['ipopt'] == settings() == DEFAULTS
    result = settings()
    result['spral_scaling'] = 'none'
    assert settings()['spral_scaling'] == 'mc64'


@pytest.mark.parametrize('value', [None, [], 'metis', True, {'other': 'none'},
    {'linear_solver': 'mumps'}, {'spral_order': 'METIS'}, {'spral_scaling': 'dynamic'},
    {'spral_order': 'matching'}, {'spral_scaling': 'matching'},
    {'spral_order': 'matching', 'spral_scaling': 'mc64'}, {'spral_scaling': True}])
def test_bad_or_overridden_ordering_cannot_silently_reach_native_code(value):
    with pytest.raises(ValueError):
        load_config({'ipopt': value})
    with pytest.raises(ValueError):
        settings({'ipopt': value})


def test_historical_matching_is_an_explicit_paired_configuration():
    value = dict(spral_order='matching', spral_scaling='matching')
    assert load_config({'ipopt': value})['ipopt'] == dict(DEFAULTS, **value)


def test_binary_evidence_identifies_the_actual_loaded_drake_library():
    value = evidence()
    binary = value['native_binary']
    assert binary['sha256'] == hashlib.sha256(Path(binary['library']).read_bytes()).hexdigest()
    assert binary['loaded_library_paths'] == [binary['library']]
    assert binary['loaded_matches_package'] is True
    assert value['avoids_matching_order']
    assert value['native_library_patched'] is False
    assert value['constraints_changed'] is False
    assert not evidence({'ipopt': dict(spral_order='matching', spral_scaling='matching')})['avoids_matching_order']


@pytest.mark.parametrize('paths', [[], ['/tmp/foreign/libdrake.so'],
                                  ['/tmp/one/libdrake.so', '/tmp/two/libdrake.so']])
def test_missing_or_foreign_loaded_library_is_rejected(monkeypatch, paths):
    monkeypatch.setattr('benchmarks.generic_motion_v1.runners.ipopt_options._loaded_library_paths', lambda: paths)
    with pytest.raises(RuntimeError, match='Loaded libdrake differs'):
        evidence()


def test_native_probe_failure_still_saves_solver_configuration(tmp_path, monkeypatch):
    from pydrake.solvers import IpoptSolver
    from benchmarks.generic_motion_v1.runners import paper_nto
    class Fails:
        def available(self): return True
        def enabled(self): return True
        def solver_id(self): return IpoptSolver().solver_id()
        def Solve(self, program): raise RuntimeError('diagnostic failure')
    monkeypatch.setattr(paper_nto, '_solver', lambda objective: Fails())
    with pytest.raises(RuntimeError, match='diagnostic failure'):
        paper_nto.dependency_probe(tmp_path, 'minimum_time')
    value = json.loads((tmp_path/'paper_nto_dependencies.json').read_text())
    assert value['linear_system']['options'] == DEFAULTS
    assert value['linear_system']['native_binary']['loaded_matches_package'] is True


@pytest.mark.parametrize('scaling', ['mc64', 'none'])
def test_actual_ipopt_solves_and_echoes_effective_linear_solver_options(tmp_path, scaling):
    from pydrake.all import IpoptSolver, MathematicalProgram
    program = MathematicalProgram()
    x = program.NewContinuousVariables(2)
    program.AddBoundingBoxConstraint(np.zeros(2), np.ones(2)*3, x)
    program.AddLinearEqualityConstraint(np.ones((1, 2)), [3.], x)
    program.AddQuadraticErrorCost(np.eye(2), np.array([1., 2.]), x)
    solver = IpoptSolver()
    options = settings({'ipopt': {'spral_scaling': scaling}})
    log = tmp_path/'ipopt.log'
    for key, value in dict(options, print_level=0, file_print_level=5,
                           print_user_options='yes', output_file=str(log)).items():
        program.SetSolverOption(solver.solver_id(), key, value)
    result = solver.Solve(program)
    assert result.is_success(), result.get_solution_result()
    np.testing.assert_allclose(result.GetSolution(x), [1., 2.], atol=1e-6, rtol=0)
    text = log.read_text()
    assert 'linear_solver = spral' in text
    assert 'spral_order = metis' in text
    assert 'spral_scaling = '+scaling in text


@pytest.mark.parametrize('method,objective', [('NTO', 'minimum_distance'), ('NTO', 'minimum_time'),
    ('NTO_PAPER', 'minimum_time'), ('NTO_REFINED', 'minimum_time')])
def test_native_dependency_probes_use_the_shared_policy(tmp_path, method, objective):
    from benchmarks.generic_motion_v1.runners.csdecomp_baseline import dependency_audit
    result = dependency_audit(tmp_path, method, objective, config=load_config())
    assert result['linear_system']['options'] == DEFAULTS
    assert result['linear_system']['native_binary']['sha256'] == evidence()['native_binary']['sha256']
