"""Independent CPU-only native handoff audit; no model, planner or GPU use."""
from pathlib import Path
import hashlib
import json
import subprocess
import sys
import time
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT))
import csdecomp as csd
from pydrake.all import HPolyhedron
from benchmarks.generic_motion_v1.runners.native_handoff import NativeHandoffError, native_region_with_anchors

OUT = ROOT / 'results/trend_diagnostics/native_handoff_independent'
OUT.mkdir(exist_ok=True)
checks = []
started = time.monotonic()


def forbidden(*args, **kwargs):
    raise AssertionError('This audit must not execute GPU or a planner')


def receipt(name):
    return json.loads((OUT/name/'handoff.json').read_text())


def region(A, b):
    return SimpleNamespace(A=lambda: A, b=lambda: b)


def step(value, count):
    value = np.float32(value)
    for _ in range(count):
        value = np.nextafter(value, np.float32(np.inf))
    return value


def case(name, initial, threshold, allowed, original=None, expected_error=False, scales=None):
    initial = np.atleast_1d(initial).astype(np.float32)
    threshold = np.atleast_1d(threshold).astype(np.float32)
    if scales is None:
        scales = np.ones(len(initial))
    A = np.zeros((len(initial), 7)); A[:, 0] = scales
    # All rows share an anchor coordinate, with corresponding scalar normals.
    q = np.zeros((2, 7)); q[:, 0] = float(initial[0]) / scales[0]
    original = initial.astype(float) if original is None else np.asarray(original, float)
    seen = []

    class Oracle:
        def __init__(self, supplied_A, supplied_b):
            np.testing.assert_array_equal(supplied_A, A.astype(np.float32))
            assert supplied_A.shape == A.shape
            assert np.isfinite(supplied_b).all(), 'No infinite/vacuous offsets permitted'
            self._A, self._b = supplied_A.copy(), supplied_b.copy()
            seen.append(self._b.copy())
        def PointInSet(self, supplied_q, tol):
            assert tol == 0.
            np.testing.assert_array_equal(supplied_q, q.astype(np.float32)[0])
            return bool(np.all(self._b >= threshold))
        def A(self):
            return self._A
        def b(self):
            return self._b

    A0, b0, q0 = A.copy(), original.copy(), q.copy()
    with patch.object(csd, 'HPolyhedron', Oracle):
        if expected_error:
            try:
                native_region_with_anchors(region(A, original), q, allowed, OUT/name)
            except NativeHandoffError:
                pass
            else:
                raise AssertionError(f'{name} accepted excessive displacement')
        else:
            fixed = native_region_with_anchors(region(A, original), q, allowed, OUT/name)
            np.testing.assert_array_equal(fixed.b(), threshold)
            r = receipt(name)
            assert r['maximum_normalized_correction'] <= allowed
            assert r['maximum_total_normalized_offset_change_from_input'] <= allowed
            for search in r['bounded_native_offset_search']:
                assert search['native_predicate_calls'] <= 34
                assert np.float32(search['preceding_offset']) < threshold[search['row']]
    np.testing.assert_array_equal(A, A0)
    np.testing.assert_array_equal(original, b0)
    np.testing.assert_array_equal(q, q0)
    r = receipt(name)
    checks.append(dict(name=name, passed=True, status=r['status'], constructor_calls=len(seen),
        corrected_offsets=None if expected_error else fixed.b().astype(float).tolist(),
        normalized_change=r.get('maximum_normalized_correction'),
        total_normalized_change=r.get('maximum_total_normalized_offset_change_from_input')))


with patch.object(csd, 'CudaEdgeInflator', forbidden), patch.object(csd, 'EditRegionsCuda', forbidden), patch.object(csd, 'CheckCollisionFreeCuda', forbidden), patch.object(csd, 'CheckCollisionFreeVoxelsCuda', forbidden):
    tiny = np.nextafter(np.float32(0), np.float32(1))
    maximum = np.finfo(np.float32).max
    cases = [
        ('positive_100_ulp', np.float32(1), step(1, 100), 2e-5),
        ('negative_100_ulp', np.float32(-1), step(-1, 100), 1e-5),
        ('negative_to_positive', np.float32(-1e-7), np.float32(1e-7), 3e-7),
        ('negative_to_signed_zero', np.float32(-128*tiny), np.float32(-0.), float(256*tiny)),
        ('negative_zero_to_subnormal', np.float32(-0.), np.float32(128*tiny), float(256*tiny)),
        ('positive_zero_to_subnormal', np.float32(0.), np.float32(128*tiny), float(256*tiny)),
        ('positive_subnormal', np.float32(32*tiny), np.float32(128*tiny), float(256*tiny)),
        ('negative_subnormal', np.float32(-128*tiny), np.float32(-32*tiny), float(256*tiny)),
    ]
    start = maximum
    for _ in range(64):
        start = np.nextafter(start, np.float32(-np.inf))
    cases.extend([
        ('largest_finite_positive', start, maximum, (float(maximum)-float(start))*1.001),
        ('largest_finite_negative', -maximum, step(-maximum, 64), (float(step(-maximum,64))+float(maximum))*1.001),
    ])
    for name, initial, threshold, allowed in cases:
        case(name, initial, threshold, allowed)
    # Multiple failing rows must all be minimally repaired, with full matrix A
    # supplied on every actual native-predicate call.
    case('multiple_rows', [1., 1., 1.], [step(1,35),step(1,81),step(1,149)], 2e-5)
    case('multirow_partial_failure_preserves_attempted_arrays', [1., 1.], [step(1,35), step(1,500)], 2e-5, expected_error=True)
    partial = receipt('multirow_partial_failure_preserves_attempted_arrays')
    assert partial['bounded_native_offset_search'][0]['status'] == 'minimal_passing_offset_found'
    assert partial['bounded_native_offset_search'][1]['status'] == 'no_passing_offset_within_bound'
    with np.load(OUT/'multirow_partial_failure_preserves_attempted_arrays/corrected.npz') as partial_arrays:
        assert partial_arrays['b'][0] == step(1,35)
        assert partial_arrays['b'][1] == step(1,16)
    ulp = float(step(1,1)) - 1.
    threshold = float(step(1,50))
    for delta, name in [(-.49, 'total_bound_stricter'), (.49, 'additional_bound_stricter')]:
        orig = 1. + delta*ulp
        raw = threshold - 1.
        total = threshold - orig
        case(name, [1.], [threshold], min(raw,total)+.20*ulp, original=[orig], expected_error=True)
    # Errors are not translated into a success, nor mislabelled as a bounded
    # numerical handoff failure when the native predicate itself malfunctions.
    for exception_type in (RuntimeError, SystemExit, KeyboardInterrupt):
        def raiser(*args, **kwargs):
            raise exception_type('independent injected native failure')
        name = f'propagate_{exception_type.__name__}'
        with patch.object(csd, 'HPolyhedron', raiser):
            try:
                native_region_with_anchors(region(np.eye(7), np.ones(7)), np.zeros((2,7)), 1e-6, OUT/name)
            except BaseException as exc:
                assert type(exc) is exception_type
            else:
                raise AssertionError('Native exception was swallowed')
        checks.append(dict(name=name, passed=True, exception_type=exception_type.__name__))
    class InconsistentOracle:
        def __init__(self, A, b):
            self.b = b
        def PointInSet(self, q, tol):
            return bool(np.count_nonzero(self.b == maximum) >= 1)
    with patch.object(csd, 'HPolyhedron', InconsistentOracle):
        try:
            native_region_with_anchors(region(np.eye(7),np.ones(7)),np.zeros((2,7)),1e-6,OUT/'inconsistent_predicate')
        except ValueError as exc:
            assert not isinstance(exc, NativeHandoffError)
            assert 'Cannot localize' in str(exc)
        else:
            raise AssertionError('Inconsistent whole-matrix predicate accepted')
    checks.append(dict(name='whole_matrix_row_isolation_inconsistency',passed=True))
    # Real source fixture: reproduce old rejection, repair, then demonstrate
    # every lower representable candidate fails with the complete original A.
    source = ROOT/'results/repaired_dev_all_seeds/jobs/dev-dense_clutter-002-p01-LGCS-minimum_distance-83e6f638/attempts/0001/recovery/000/native_handoff/0005'
    fixture_sha = {p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in source.iterdir() if p.is_file()}
    with np.load(source/'original.npz') as d:
        A,b,q = [d[k] for k in ('A','b','anchors')]
    fixed = native_region_with_anchors(HPolyhedron(A,b),q,1e-6,OUT/'real_52x7')
    r = receipt('real_52x7')
    assert all(fixed.PointInSet(x,0.) for x in q)
    searches = r['bounded_native_offset_search']; assert len(searches)==1
    row = searches[0]['row']; assert row==14
    first = b.astype(np.float32)[row]
    offsets = []
    candidate = first
    while candidate <= fixed.b()[row]:
        trial_b = fixed.b().copy(); trial_b[row] = candidate
        passed = all(csd.HPolyhedron(fixed.A(),trial_b).PointInSet(x,0.) for x in q)
        offsets.append(dict(offset=float(candidate), passes=passed))
        candidate = np.nextafter(candidate,np.float32(np.inf))
    assert len(offsets)==24 and all(not x['passes'] for x in offsets[:-1]) and offsets[-1]['passes']
    checks.append(dict(name='real_52x7_23_ulp_minimum',passed=True,row=row,A_shape=list(A.shape),anchors_shape=list(q.shape),
        maximum_total_change=r['maximum_total_normalized_offset_change_from_input'],offsets=offsets))
    for idx in range(5):
        normal = source.parent/f'{idx:04d}'
        with np.load(normal/'original.npz') as d:
            fixed = native_region_with_anchors(HPolyhedron(d['A'],d['b']),d['anchors'],1e-6,OUT/f'normal_{idx}')
        with np.load(normal/'corrected.npz') as d:
            assert fixed.A().tobytes()==d['A'].tobytes()
            assert fixed.b().tobytes()==d['b'].tobytes()
        assert receipt(f'normal_{idx}')==json.loads((normal/'handoff.json').read_text())
    checks.append(dict(name='five_previously_normal_outputs_and_receipts_unchanged',passed=True,count=5))
    assert fixture_sha == {p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in source.iterdir() if p.is_file()}

runtime = ROOT/'benchmarks/generic_motion_v1/runners/native_handoff.py'
summary = dict(passed=True,check_count=len(checks),checks=checks,cpu_seconds=time.monotonic()-started,
    runtime_sha256=hashlib.sha256(runtime.read_bytes()).hexdigest(),fixture_sha256=fixture_sha,
    native_predicate_source='csdecomp/src/geometry/hpolyhedron.cpp:265; ((_A * point).array() <= _b.array() + tol).all()',
    interpretation='Ordered float32 b is monotone for this exact fixed full-A native CPU predicate; no geometry or anchor changes beyond required cast and bounded b correction.',
    gpu_calls=0,planner_calls=0,solver_calls=0,limitations=['CPU numeric handoff only; no continuous collision certification or new planning success implied.'])
(OUT/'review.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(summary,indent=2))
