"""Independent CPU adversarial tests; no GPU collision query or solver."""
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from pydrake.all import Box
from pydrake.autodiffutils import InitializeAutoDiff, ExtractGradient, ExtractValue
from benchmarks.generic_motion_v1.model import Model
from benchmarks.generic_motion_v1.roadmap import HERE
from benchmarks.generic_motion_v1.runners import nto_collision_constraint as module

KNOWN = "Failure at geometry/proximity/distance_to_shape_touching.cc:122 in BoxBoxGradient(): condition '(X_WA * p_ACa - X_WB * p_BCb).norm() < kEps' failed."
EVIDENCE = {}


@pytest.fixture(scope='module')
def native():
    # A CUDA call would invalidate this review's CPU-only scope.
    original_cuda = Model.cuda
    Model.cuda = lambda *a, **k: (_ for _ in ()).throw(AssertionError('GPU forbidden during independent review'))
    try:
        path = HERE/'results/repaired_dev/jobs/dev-route_choice-002-p00-NTO_PAPER-minimum_distance-48b608ce/attempts/0001/input.json'
        case = json.loads(path.read_text())
        model = Model(case['voxels_xyz'], case['voxel_radius_m'])
        native = model.nto_plant()
        q = np.load(HERE/'reports/baseline_study/nto_backend_last_q_probe/last_evaluated_q.npy')
        yield native, q
    finally:
        Model.cuda = original_cuda
        Path(__file__).with_name('nto_collision_independent_values.json').write_text(json.dumps(EVIDENCE, indent=2)+'\n')


def proxy_native(native, q, intercept):
    native.plant.SetPositions(native.plant_context, q)
    query = native.scene_graph.get_query_output_port().Eval(native.scene_context)
    class Query:
        def ComputeSignedDistancePairClosestPoints(self, a, b):
            intercept(a, b)
            return query.ComputeSignedDistancePairClosestPoints(a, b)
        def GetPoseInWorld(self, identity):
            return query.GetPoseInWorld(identity)
    port = SimpleNamespace(Eval=lambda context: Query())
    return SimpleNamespace(**{**vars(native), 'scene_graph': SimpleNamespace(get_query_output_port=lambda: port)})


def test_fallback_queries_every_canonical_pair_exactly_once(native):
    source, q = native
    wrapper = module.RobustMinimumDistanceConstraint(source, 0., .01)
    seen = []
    wrapper.native = proxy_native(source, q, lambda a, b: seen.append((a, b)))
    values, recovered = wrapper._distances(q)
    assert len(seen) == len(set(seen)) == len(wrapper.pairs) == source.collision_pair_count
    assert set(seen) == set(source.inspector.GetCollisionCandidates())
    assert len(values) == len(seen) and np.isfinite(values).all() and recovered
    assert all(abs(x['signed_distance_m']) <= 1e-9 for x in recovered)
    EVIDENCE['canonical_pairs_queried_once'] = len(seen)
    EVIDENCE['recovered_pair_names'] = [x['pair'] for x in recovered]
    EVIDENCE['maximum_recovered_absolute_distance_m'] = max(abs(x['signed_distance_m']) for x in recovered)


def test_recovery_uses_native_aggregation_including_far_pairs(native):
    from pydrake.all import MinimumValueLowerBoundConstraint
    source, q = native
    wrapper = module.RobustMinimumDistanceConstraint(source, .001, .02)
    rng = np.random.Generator(np.random.PCG64(984721))
    differences = []
    for step in np.r_[np.linspace(-.001, -.00001, 15), np.linspace(.00001, .001, 15)]:
        x = q.copy(); x[1] += step
        values, recovered = wrapper._distances(x)
        assert not recovered
        actual, _ = wrapper._fallback_scalar(x)
        expected = wrapper.original.Eval(x)[0]
        differences.append(float(abs(actual-expected)))
        assert abs(actual-expected) < 1e-10
    EVIDENCE['native_aggregation_comparisons'] = len(differences)
    EVIDENCE['maximum_aggregation_absolute_error'] = max(differences)


def test_recovered_gradient_matches_independent_directions_and_ad_chain(native):
    source, q = native
    wrapper = module.RobustMinimumDistanceConstraint(source, 0., .01)
    grad = ExtractGradient(wrapper.evaluate(InitializeAutoDiff(q)))[0]
    rng = np.random.Generator(np.random.PCG64(8715))
    differences = []
    for _ in range(5):
        direction = rng.normal(size=7); direction /= np.linalg.norm(direction)
        h = 4e-7
        direct = (wrapper._fallback_scalar(q+h*direction)[0]-wrapper._fallback_scalar(q-h*direction)[0])/(2*h)
        error = abs(direct-grad@direction)
        differences.append(float(error))
        assert error < 1e-3*max(1., abs(direct))
    seed = rng.normal(size=(7, 4))
    seeded = ExtractGradient(wrapper.evaluate(InitializeAutoDiff(q, seed)))[0]
    np.testing.assert_allclose(seeded, grad@seed, atol=1e-10, rtol=0)
    np.testing.assert_array_equal(source.plant.GetPositions(source.plant_context), q)
    EVIDENCE['independent_directional_derivative_checks'] = len(differences)
    EVIDENCE['maximum_directional_derivative_absolute_error'] = max(differences)
    EVIDENCE['nonidentity_autodiff_chain_max_error'] = float(np.max(np.abs(seeded-grad@seed)))


@pytest.mark.parametrize('exception', [SystemExit('user requested stop'), SystemExit('BoxBoxGradient(): different condition failed'),
                                       RuntimeError('different geometry failure'), RuntimeError('BoxBoxGradient witness assertion without expected expression failed')])
def test_unrelated_exceptions_escape_without_fallback(exception):
    wrapper = object.__new__(module.RobustMinimumDistanceConstraint)
    wrapper.num_positions = 7
    wrapper.native_evaluations = 0
    wrapper.original = SimpleNamespace(Eval=lambda q: (_ for _ in ()).throw(exception))
    wrapper._fallback_scalar = lambda q: pytest.fail('Unknown exception must not enter recovery')
    with pytest.raises(type(exception), match=str(exception).split(':')[0]):
        wrapper.evaluate(np.zeros(7))


def test_known_systemexit_recovers_constraint_without_marking_contact_free(native):
    source, q = native
    wrapper = module.RobustMinimumDistanceConstraint(source, 0., .01)
    wrapper.original = SimpleNamespace(Eval=lambda q: (_ for _ in ()).throw(SystemExit(KNOWN)))
    value = wrapper.evaluate(q)[0]
    assert value >= 1.-1e-8
    assert wrapper.events[-1]['recovered_pairs']
    EVIDENCE['known_systemexit_constraint_value'] = float(value)


def test_boxbox_recovery_never_accepts_nonbox_geometry(native):
    source, q = native
    wrapper = module.RobustMinimumDistanceConstraint(source, 0., .01)
    target = next(pair for pair in wrapper.pairs if not all(isinstance(source.inspector.GetShape(g), Box) for g in pair))
    def intercept(a, b):
        if (a, b) == target:
            raise RuntimeError(KNOWN)
    wrapper.native = proxy_native(source, q, intercept)
    with pytest.raises(RuntimeError, match='non-box geometry'):
        wrapper._distances(q)


@pytest.mark.parametrize('distance', [.01, -.01, float('nan')])
def test_nonnear_contact_or_nonfinite_reconstruction_is_rejected(native, monkeypatch, distance):
    source, q = native
    wrapper = module.RobustMinimumDistanceConstraint(source, 0., .01)
    monkeypatch.setattr(module, 'box_box_signed_distance', lambda *args: distance)
    with pytest.raises((RuntimeError, ValueError), match='non-contact|Nonfinite'):
        wrapper.evaluate(q)
