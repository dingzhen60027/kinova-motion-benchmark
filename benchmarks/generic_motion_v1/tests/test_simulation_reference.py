"""CPU-only coefficient/physical-clock tests, including real saved trajectories."""
import hashlib
import json
from pathlib import Path

import numpy as np
import pytest

from benchmarks.generic_motion_v1.simulation.reference import _from_candidate, load_reference


ROOT = Path(__file__).resolve().parents[3]


def candidate():
    points = np.zeros((1, 4, 7))
    points[0, 2:, 0] = 1.
    return dict(kind="bezier", objective="minimum_time", controls=points, durations=np.array([3.]))


def test_cubic_bezier_exact_physical_derivatives_and_hold():
    ref = _from_candidate(candidate(), {})
    assert ref.duration == 3.
    q, v, a = ref.at(1.5)
    np.testing.assert_allclose(q, [.5, 0, 0, 0, 0, 0, 0], atol=1e-14)
    np.testing.assert_allclose(v, [.5, 0, 0, 0, 0, 0, 0], atol=1e-14)
    np.testing.assert_allclose(a, 0., atol=1e-14)
    assert ref.at(3.)[2][0] == pytest.approx(-2/3)
    for t, expected in [(-1., 0.), (3.00001, 1.)]:
        q, v, a = ref.at(t)
        assert q[0] == expected
        np.testing.assert_array_equal(v, np.zeros(7))
        np.testing.assert_array_equal(a, np.zeros(7))


def test_bspline_physical_knots_nonzero_origin_not_scaled_twice():
    c = candidate()
    c.update(kind="bspline", controls=c["controls"][0], spline_order=4,
             knots=np.array([2.] * 4 + [5.] * 4), duration=3.)
    c.pop("durations")
    ref = _from_candidate(c, {})
    for t in np.linspace(0., 3., 21):
        for actual, expected in zip(ref.at(t), _from_candidate(candidate(), {}).at(t)):
            np.testing.assert_allclose(actual, expected, atol=3e-15)
    assert ref.metadata["original_knot_start_seconds"] == 2.


@pytest.mark.parametrize("change", [
    {"objective": "minimum_distance"},
    {"kind": "polygon"},
    {"durations": np.array([0.])},
    {"durations": np.array([-1.])},
    {"durations": np.array([np.inf])},
    {"durations": np.array([True])},
    {"durations": [True]},
    {"controls": np.zeros((1, 4, 6))},
    {"duration": 5.},
    {"duration": True},
    {"duration": "3"},
])
def test_invalid_candidate_rejected(change):
    c = candidate()
    c.update(change)
    with pytest.raises(ValueError):
        _from_candidate(c, {})


def test_mixed_numeric_boolean_cannot_coerce_to_float():
    c = candidate()
    c["controls"] = c["controls"].tolist()
    c["controls"][0][0][0] = True
    with pytest.raises(ValueError, match="booleans"):
        _from_candidate(c, {})


@pytest.mark.parametrize("bad_time", [True, "1", np.nan, np.inf, np.array([1.]), 1j])
def test_malformed_time_rejected(bad_time):
    with pytest.raises(ValueError):
        _from_candidate(candidate(), {}).at(bad_time)


def test_segment_position_and_velocity_discontinuities_rejected():
    c = candidate()
    c["controls"] = np.concatenate([c["controls"], c["controls"]])
    c["durations"] = np.array([3., 3.])
    with pytest.raises(ValueError, match="C1"):
        _from_candidate(c, {})
    c["controls"][1, :, 0] += 1.
    assert _from_candidate(c, {}).duration == 6.
    c["controls"][1, 1, 0] = 1.1
    with pytest.raises(ValueError, match="C1"):
        _from_candidate(c, {})


@pytest.mark.parametrize("change", [
    {"spline_order": True}, {"spline_order": 2}, {"spline_order": 3.5},
    {"knots": [0.] * 8}, {"knots": [0., 0., 0., 0., 2., 1., 3., 3.]},
    {"knots": [0., 0., 0., .1, 3., 3., 3., 3.]},
    {"knots": [0.] * 4 + [np.nan] * 4},
])
def test_malformed_spline_rejected(change):
    c = candidate()
    c.update(kind="bspline", controls=c["controls"][0], spline_order=4,
             knots=np.array([0.] * 4 + [3.] * 4), duration=3.)
    c.pop("durations")
    c.update(change)
    with pytest.raises(ValueError):
        _from_candidate(c, {})


def test_excess_interior_knot_multiplicity_rejected():
    c = dict(kind="bspline", objective="minimum_time", controls=np.zeros((7, 7)),
             knots=np.array([0.] * 4 + [1.] * 3 + [2.] * 4), spline_order=4, duration=2.)
    with pytest.raises(ValueError, match="C1"):
        _from_candidate(c, {})


def test_json_npz_conflict_rejected(tmp_path):
    (tmp_path / "candidate.json").write_text(json.dumps(dict(kind="bezier", objective="minimum_time", durations=[3.])))
    np.savez(tmp_path / "candidate.npz", controls=candidate()["controls"], durations=[3.])
    with pytest.raises(ValueError, match="both JSON and NPZ"):
        load_reference(tmp_path)


REAL_ATTEMPTS = [
    "benchmarks/generic_motion_v1/results/nto_metis_smoke/jobs/smoke-bottleneck-000-p00-NTO_REFINED-minimum_time-633cc65e/attempts/0001",
    "benchmarks/generic_motion_v1/results/baseline_smoke/jobs/smoke-open_control-000-p01-EI_ZO-minimum_time-61913820/attempts/0001",
]


@pytest.mark.parametrize("relative", REAL_ATTEMPTS)
def test_actual_saved_trajectory_matches_validator_samples_and_derivatives(relative):
    directory = ROOT / relative
    ref = load_reference(directory)
    assert json.loads((directory / "validation.json").read_text())["passed"]
    with np.load(directory / "trajectory_samples.npz", allow_pickle=False) as samples:
        times, positions = samples["time"], samples["q"]
    for time, expected in zip(times[::max(1, len(times)//37)], positions[::max(1, len(times)//37)]):
        np.testing.assert_allclose(ref.at(float(time))[0], expected, rtol=0, atol=3e-12)
    # Interior finite differences verify physical seconds and both analytic derivatives.
    h = 1e-5
    for time in [.137 * ref.duration, .371 * ref.duration, .719 * ref.duration]:
        q, v, a = ref.at(time)
        minus, plus = ref.at(time-h), ref.at(time+h)
        np.testing.assert_allclose((plus[0]-minus[0])/(2*h), v, atol=2e-8, rtol=2e-7)
        np.testing.assert_allclose((plus[1]-minus[1])/(2*h), a, atol=2e-8, rtol=2e-7)
    for name, digest in ref.metadata["source_files_sha256"].items():
        assert hashlib.sha256((directory / name).read_bytes()).hexdigest() == digest
    assert ref.metadata["evaluation"].startswith("original polynomial")
