"""Bounded float32 region handoff, checked by the native CPU predicate.

This module never calls CUDA. It preserves the supplied anchors and changes
only the halfspace offsets after the conversion required by CSDecomp's API.
It does not establish collision freedom of an enlarged region.
"""
from pathlib import Path

import numpy as np

from ..common import write
from .validate import _numeric_array


def native_region_with_anchors(region, anchors, maximum_normalized_correction, out=None):
    """Return native HPolyhedron containing every anchor after float32 casting.

    Corrections are measured in configuration-space distance, using each
    quantized normal's norm. Both the additional outward correction and the
    complete offset change from the supplied region are bounded by the given
    tolerance. Native containment is checked with zero tolerance, which is
    stricter than EditRegionsCuda's default 1e-9 predicate.

    ``out`` stores original, quantized and final arrays plus handoff.json.
    Large or unresolved errors raise ValueError; endpoints are never moved.
    """
    import csdecomp as csd

    directory = None if out is None else Path(out)
    if directory is not None:
        directory.mkdir(parents=True, exist_ok=True)
    record = dict(status="checking", native_predicate_tolerance=0.,
        fixed_anchors_changed=False, correction_semantics="Outward b changes only after required float32 quantization; normalized by quantized row norms.",
        note="Numerical handoff only; final independent collision validation remains required.")
    final_arrays = None
    try:
        A = _numeric_array(region.A(), "Region normals")
        b = _numeric_array(region.b(), "Region offsets")
        anchors = _numeric_array(anchors, "Region anchors")
        if (A.ndim != 2 or A.shape[1] != 7 or len(A) == 0 or b.shape != (len(A),)
                or anchors.ndim != 2 or anchors.shape[1] != 7 or not len(anchors)
                or not np.isfinite(A).all() or not np.isfinite(b).all() or not np.isfinite(anchors).all()):
            raise ValueError("Region handoff requires finite A[M,7], b[M], anchors[N,7]")
        if (isinstance(maximum_normalized_correction, (bool, np.bool_))
                or not isinstance(maximum_normalized_correction, (int, float, np.integer, np.floating))
                or not np.isfinite(maximum_normalized_correction) or maximum_normalized_correction < 0):
            raise ValueError("Region handoff tolerance must be a finite nonnegative scalar")
        allowed = float(maximum_normalized_correction)
        record["allowed_normalized_correction"] = allowed
        if directory is not None:
            np.savez_compressed(directory / "original.npz", A=A, b=b, anchors=anchors)
        maximum = np.finfo(np.float32).max
        if any(np.any(np.abs(x) > maximum) for x in (A, b, anchors)):
            raise ValueError("Region or anchors cannot be represented as float32")
        A32, initial_b, q32 = [np.asarray(x, dtype=np.float32) for x in (A, b, anchors)]
        norms = np.linalg.norm(A32.astype(float), axis=1)
        if (not np.isfinite(norms).all() or
                np.any((norms == 0) & np.any(A != 0, axis=1))):
            raise ValueError("Region normals are unrepresentable after float32 quantization")
        # Avoid invoking a native dot product which could overflow to infinity.
        absolute_dot_bound = np.abs(A32.astype(float)) @ np.abs(q32.astype(float)).T
        if not np.isfinite(absolute_dot_bound).all() or np.any(absolute_dot_bound > maximum):
            raise ValueError("Native region containment arithmetic can overflow")
        double_dot = A32.astype(float) @ q32.astype(float).T
        b32 = initial_b.copy()
        if directory is not None:
            np.savez_compressed(directory / "quantized.npz", A=A32, b=b32, anchors=q32)
        record.update(normal_quantization_max_abs_error=float(np.max(np.abs(A32.astype(float) - A))),
            anchor_quantization_max_abs_error=float(np.max(np.abs(q32.astype(float) - anchors))),
            maximum_quantized_anchor_residual_in_float64=float(np.max(double_dot - b32[:, None])))

        def check_bound():
            nonlocal final_arrays
            if not np.isfinite(b32).all():
                raise ValueError("Region offset correction overflowed float32")
            raw = b32.astype(float) - initial_b.astype(float)
            total = np.abs(b32.astype(float) - b)
            if np.any((norms == 0) & ((raw > 0) | (total > 0))):
                raise ValueError("Cannot normalize a correction to a zero-normal constraint")
            normalized = np.divide(raw, norms, out=np.zeros_like(raw), where=norms > 0)
            normalized_total = np.divide(total, norms, out=np.zeros_like(total), where=norms > 0)
            largest = float(np.max(normalized, initial=0.))
            total_largest = float(np.max(normalized_total, initial=0.))
            record.update(maximum_normalized_correction=largest,
                maximum_total_normalized_offset_change_from_input=total_largest,
                corrected_planes=int(np.count_nonzero(raw)))
            final_arrays = dict(A=A32, b=b32.copy(), anchors=q32, raw_corrections=raw,
                                normalized_corrections=normalized, normalized_total_offset_changes=normalized_total)
            if largest > allowed or total_largest > allowed:
                raise ValueError(f"Native handoff correction {max(largest, total_largest):.9g} exceeds fixed tolerance {allowed:.9g}")

        def contained(native):
            return [bool(native.PointInSet(q, 0.)) for q in q32]

        initial_native = csd.HPolyhedron(A32, b32)
        record["native_anchor_containment_before"] = contained(initial_native)
        # Round a double-precision estimate of each native dot product upward.
        # A later native check is essential: NumPy and Eigen can use different
        # accumulation orders, including fused multiply/add instructions.
        target = np.maximum(b32.astype(float), np.max(double_dot, axis=1))
        b32 = target.astype(np.float32)
        rounded_down = b32.astype(float) < target
        b32[rounded_down] = np.nextafter(b32[rounded_down], np.float32(np.inf))
        for iteration in range(17):
            check_bound()
            native = csd.HPolyhedron(A32, b32)
            passed = contained(native)
            record["native_anchor_containment_after"] = passed
            record["native_refinement_rounds"] = iteration
            if all(passed):
                record["status"] = "passed"
                if directory is not None:
                    np.savez_compressed(directory / "corrected.npz", **final_arrays)
                    write(directory / "handoff.json", record)
                return native
            if iteration == 16:
                raise ValueError("Native anchor containment remains unresolved after bounded float32 refinement")
            # Isolate each predicate while preserving A's complete matrix shape:
            # changing the number of rows can change Eigen's float32 reduction.
            # Other finite offsets are made vacuous only in this CPU diagnostic;
            # they are never returned as region geometry.
            failing = []
            for row in range(len(b32)):
                probe_b = np.full_like(b32, maximum)
                probe_b[row] = b32[row]
                probe = csd.HPolyhedron(A32, probe_b)
                if not all(contained(probe)):
                    failing.append(row)
            if not failing:
                raise ValueError("Cannot localize native containment failure without changing the predicate")
            b32[failing] = np.nextafter(b32[failing], np.float32(np.inf))
    except Exception as exc:
        record.update(status="rejected", error=repr(exc))
        if directory is not None:
            if final_arrays is not None:
                np.savez_compressed(directory / "corrected.npz", **final_arrays)
            write(directory / "handoff.json", record)
        raise
