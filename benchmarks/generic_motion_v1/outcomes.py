"""Read-only acceptance of saved, independently validated planning outputs.

Solver convergence is separate from trajectory feasibility. Every attempt is
assessed independently; selecting the first terminal attempt is the result
aggregator's responsibility, while dataset witnesses establish only existence.
No missing historical evidence is inferred and no raw artifact is rewritten.
"""
from pathlib import Path
import math

from .common import read, resolve_case, sha

RAW_SUCCESSES = {"success_validated_numerical", "success_numerical"}
CANDIDATE_STATUSES = RAW_SUCCESSES | {"solver_failed"}
NONCONVERGED_STATUS_BASIS = "saved_candidate_independently_validated_within_budget_despite_solver_nonconvergence"
REQUIRED_FILES = {"candidate.json", "candidate.npz", "validation.json", "trajectory_samples.npz",
                  "process.json", "result.json", "input.json", "request.json"}


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _reject(reason, status="infrastructure_error"):
    return dict(accepted=False, status=status, status_basis="saved_output_evidence_rejected", reason=reason)


def _array_consistency(attempt, candidate_metadata, validation, case, config):
    """Bind saved coefficients, numerical samples and summary without geometry calls.

    Uses the common sampler and existing collision masks/clearances. It does
    not repeat CUDA/Drake collision queries or certify continuous collision
    freedom. The candidate must reproduce the exact sample schedule recorded by
    this protocol; agreement uses only a tight floating-point roundoff tolerance.
    """
    import numpy as np
    from .common import BASE
    from .runners.validate import sample_candidate
    from .roadmap import load_config

    def scalar_close(actual, expected):
        return (_finite(actual) and _finite(float(expected))
                and bool(np.isclose(actual, expected, rtol=1e-10, atol=1e-12)))

    def numeric_array(value, name, *, integer=False):
        array = np.asarray(value)
        if (array.dtype.kind not in ('iu' if integer else 'fiu')
                or not np.isfinite(array).all()):
            raise ValueError(name + ' must be finite numeric array')
        return array

    try:
        cfg = load_config(config)
        candidate = dict(candidate_metadata)
        kinds = {'polygon': {'points'}, 'bspline': {'controls', 'knots'},
                 'bezier': {'controls', 'durations'}}
        kind = candidate.get('kind')
        if kind not in kinds:
            return 'Unsupported saved candidate representation'
        with np.load(attempt / 'candidate.npz', allow_pickle=False) as archive:
            if not kinds[kind] <= set(archive.files):
                return 'Candidate array archive lacks required coefficients'
            if set(archive.files) & candidate.keys():
                return 'Candidate metadata and array archive have ambiguous duplicate keys'
            for key in archive.files:
                candidate[key] = numeric_array(archive[key], 'candidate.' + key).copy()
        reconstructed = sample_candidate(candidate, cfg)
        q, t, segment = (reconstructed[key] for key in ('q', 'time', 'segment'))
        n = len(q)
        with np.load(attempt / 'trajectory_samples.npz', allow_pickle=False) as archive:
            required = {'q', 'time', 'segment', 'native', 'independent', 'clearance'}
            if not required <= set(archive.files):
                return 'Saved validation samples lack required numerical evidence'
            saved = {key: archive[key].copy() for key in required}
        for key, rebuilt in (('q', q), ('time', t), ('segment', segment)):
            value = numeric_array(saved[key], 'samples.' + key, integer=key == 'segment')
            if (value.shape != rebuilt.shape or not np.allclose(
                    value, rebuilt, atol=1e-12, rtol=1e-10)):
                return 'Saved samples do not match actual candidate: ' + key
        if (n < 2 or q.shape != (n, 7) or t.shape != (n,)
                or np.any(np.diff(t) < 0) or segment.shape != (n,)):
            return 'Malformed saved validation sample schedule'
        if type(validation.get('samples')) is not int or validation['samples'] != n:
            return 'Validation sample count differs from saved arrays'
        for key in ('native', 'independent'):
            if saved[key].dtype.kind != 'b' or saved[key].shape != (n,):
                return 'Collision mask must be one boolean per saved sample'
        clearance = numeric_array(saved['clearance'], 'samples.clearance')
        if clearance.shape != (n,) or not np.array_equal(saved['independent'], clearance > 0):
            return 'Independent collision mask contradicts signed clearance'
        # These are immutable canonical protocol limits, not a new geometry
        # model or a collision-query dependency. Bind them to the input hashes.
        robot = read(BASE / 'configs/robot.json')
        for key in ('robot_sha256', 'collision_geometry_sha256'):
            if case.get(key) != robot.get(key):
                return 'Saved input does not bind the canonical numerical model limits'
        lower = numeric_array(robot['lower'], 'canonical lower limits')
        upper = numeric_array(robot['upper'], 'canonical upper limits')
        if lower.shape != (7,) or upper.shape != (7,) or np.any(lower >= upper):
            return 'Canonical joint limits are malformed'
        controls = reconstructed['position_controls']
        tol = cfg['feasibility_tolerance']
        region_residual = validation.get('region_residual')
        if not _finite(region_residual) or region_residual > tol:
            return 'Recorded region residual contradicts the common validation pass'
        if np.any(controls < lower-tol) or np.any(controls > upper+tol):
            return 'Actual candidate violates canonical joint limits'
        band = 2e-6
        counts = dict(native_collisions=int((~saved['native']).sum()),
            independent_collisions=int((~saved['independent']).sum()),
            model_disagreements=int((saved['native'] != saved['independent']).sum()),
            contact_band_samples=int((np.abs(clearance) <= band).sum()))
        if any(type(validation.get(key)) is not int or validation[key] != value
               for key, value in counts.items()):
            return 'Validation collision counts contradict saved arrays'
        if any(counts.values()) or not saved['native'].all() or np.any(clearance <= band):
            return 'Saved collision evidence does not establish a numerical pass'
        endpoints = numeric_array([case['q_start'], case['q_goal']], 'fixed endpoints')
        if endpoints.shape != (2, 7):
            return 'Malformed fixed endpoint contract'
        endpoint = float(np.linalg.norm(q[[0, -1]]-endpoints, axis=1).max())
        step = float(np.linalg.norm(np.diff(q, axis=0), axis=1).max())
        if (endpoint > cfg['endpoint_tolerance_rad'] or
                step > cfg['validation_step_rad']*(1+1e-9)+1e-12):
            return 'Actual candidate endpoint or sampling resolution fails protocol'
        if not scalar_close(validation.get('sample_step_bound_rad'), cfg['validation_step_rad']):
            return 'Recorded sampling bound differs from original numerical protocol'
        scalar_fields = dict(endpoint_error_rad=endpoint,
            actual_maximum_sample_joint_step_rad=step,
            minimum_sampled_clearance_m=float(clearance.min()),
            cost=reconstructed['cost'],
            path_length_rad=float(np.linalg.norm(np.diff(q, axis=0), axis=1).sum()),
            normalized_path_length=float(np.linalg.norm(np.diff(q, axis=0)/(upper-lower), axis=1).sum()))
        if any(not scalar_close(validation.get(key), value) for key, value in scalar_fields.items()):
            return 'Validation objective/endpoint/clearance summary differs from actual arrays'
        objective = candidate['objective']
        if validation.get('time_axis') != reconstructed['time_axis']:
            return 'Recorded time axis differs from actual trajectory representation'
        duration = reconstructed['duration']
        if objective == 'minimum_time':
            if (duration is None or duration <= 0 or
                    not scalar_close(validation.get('duration_s'), duration)):
                return 'Actual trajectory duration contradicts time objective summary'
            if 'duration' in candidate and not scalar_close(candidate['duration'], duration):
                return 'Candidate duration metadata contradicts its spline knots'
            derivatives = reconstructed['derivatives']
            motion = [('velocity_limits_rad_s', cfg['velocity_limit_rad_s']),
                      ('acceleration_limits_rad_s2', cfg['acceleration_limit_rad_s2']),
                      ('endpoint_velocity_rad_s', 0.)]
            for key, expected in motion:
                value = numeric_array(case[key], key)
                if value.shape != (7,) or not np.array_equal(value, np.full(7, expected)):
                    return 'Frozen input motion limits differ from common protocol'
            for key, bound in [('velocity_per_joint', cfg['velocity_limit_rad_s']),
                               ('acceleration_per_joint', cfg['acceleration_limit_rad_s2'])]:
                value = numeric_array(derivatives[key], key)
                recorded = numeric_array(validation.get('derivative_bounds', {}).get(key), 'recorded '+key)
                if (value.shape != (7,) or recorded.shape != (7,) or np.any(value > bound+tol)
                        or not np.allclose(value, recorded, rtol=1e-10, atol=1e-12)):
                    return 'Recomputed trajectory derivative bounds do not pass'
            for key in ('endpoint_velocity', 'continuity_residual'):
                value = derivatives[key]
                if (not _finite(value) or value > tol or not scalar_close(
                        validation.get('derivative_bounds', {}).get(key), value)):
                    return 'Recomputed trajectory boundary/continuity condition does not pass'
        elif duration is not None or validation.get('duration_s') is not None:
            return 'Path-only candidate incorrectly reports physical trajectory duration'
        return None
    except Exception as exc:
        return 'Malformed or inconsistent candidate/sample arrays: ' + type(exc).__name__ + ': ' + str(exc)


def assess_saved_output(attempt, *, case=None, manifest=None, job=None, evidence_cache=None):
    """Apply the same evidence requirement to reported successes/nonconvergence.

    Checks the original external query budget (60 s in the current protocol).
    This is an accounting check of the saved independent numerical validation,
    not a new collision check, continuous certificate or optimality claim.
    """
    try:
        attempt = Path(attempt).resolve()
        if (len(attempt.name) != 4 or not attempt.name.isdigit()
                or attempt.parent.name != "attempts" or attempt.parents[2].name != "jobs"):
            return _reject("Output is not a manifest-backed attempt")
        receipt, result = read(attempt / "receipt.json"), read(attempt / "result.json")
        raw = result.get("status")
        request_mode = read(attempt/'request.json')
        if isinstance(request_mode,dict) and request_mode.get('execution_mode')=='fixed_initial_regions':
            binding=request_mode.get('shared_initialization',{})
            if (result.get('execution_mode')!='fixed_initial_regions'
                    or result.get('initialization_digest')!=binding.get('initialization_digest')):
                return _reject('Candidate ignored its fixed initial-region execution mode')
            for name,source_name in [('path.npy','path.npy'),('shared_initialization/regions.npz','rounds/000/regions.npz'),('shared_initialization/corridor.json','rounds/000/corridor.json')]:
                if sha(attempt/name)!=binding.get('files',{}).get(source_name):
                    return _reject('Candidate consumed a different shared initialization: '+name)
        if raw not in CANDIDATE_STATUSES:
            return _reject("Raw status is not an eligible completed candidate")
        if receipt.get("status") != raw or receipt.get("retry_pending", False) is not False:
            return _reject("Receipt is not terminal or does not match the result")
        files = receipt.get("files")
        if not isinstance(files, dict) or not REQUIRED_FILES <= files.keys():
            return _reject("Receipt lacks required formal output/process/input bindings")
        for name, digest in files.items():
            path = (attempt / name).resolve()
            if (name == "receipt.json" or not path.is_relative_to(attempt)
                    or not path.is_file() or sha(path) != digest):
                return _reject("Receipt artifact mismatch: " + name)
        actual = {str(p.relative_to(attempt)) for p in attempt.rglob("*")
                  if p.is_file() and p.name != "receipt.json" and "replay" not in p.relative_to(attempt).parts}
        if actual - files.keys():
            return _reject("Attempt contains unsealed artifacts")
        # Cache only inside one caller operation. A fresh export/witness scan
        # must reread manifests; no global cache can mask later input changes.
        evidence_cache = {} if evidence_cache is None else evidence_cache
        runs = evidence_cache.setdefault("outcome_run_manifests", {})
        run = str(attempt.parents[3])
        if run not in runs:
            saved_manifest = read(attempt.parents[3] / "run_manifest.json")
            jobs = saved_manifest.get("jobs", [])
            by_id = {item["job_id"]: item for item in jobs}
            if len(by_id) != len(jobs):
                return _reject("Original run manifest contains duplicate jobs")
            runs[run] = dict(manifest=saved_manifest, jobs=by_id, supplied=None)
        cached = runs[run]
        saved_manifest = cached["manifest"]
        if manifest is not None and cached["supplied"] is not manifest:
            if any(saved_manifest.get(k) != v for k, v in manifest.items()
                   if not k.startswith("_index_")):
                return _reject("Supplied manifest differs from original run manifest")
            cached["supplied"] = manifest
        original_job = cached["jobs"].get(attempt.parents[1].name)
        if original_job is None or (job is not None and original_job != job):
            return _reject("Attempt does not match exactly one original job")
        job = original_job
        original = resolve_case(job, evidence_cache)
        if case is not None and original != case:
            return _reject("Supplied input differs from original job input")
        if original.get("validation_status") != "valid_input":
            return _reject("Original input is not legal", "invalid_input")
        if read(attempt / "request.json") != job:
            return _reject("Saved request differs from original job")
        inp = read(attempt / "input.json")
        if any(inp.get(k) != v for k, v in original.items()):
            return _reject("Saved input differs from original case")
        objective = job.get("objective")
        if objective not in {"minimum_distance", "minimum_time"} or inp.get("_objective", objective) != objective:
            return _reject("Saved objective differs from original objective")
        expected = {"problem_id": original.get("problem_id"), "scene_id": original.get("scene_id"),
                    "family": original.get("family"), "method": job.get("method"),
                    "objective": objective, "method_seed": job.get("method_seed")}
        if any(value is None or result.get(key) != value for key, value in expected.items()):
            return _reject("Result identity differs from original task/method/seed")
        if result.get("exception") or result.get("error") or result.get("failed_stage"):
            return _reject("Result contains an exception or incomplete stage")
        candidate = read(attempt / "candidate.json")
        if (not candidate.get("kind") or candidate.get("objective") != objective
                or (raw == "solver_failed" and candidate.get("solver_success") is not False)):
            return _reject("Saved candidate identity or solver flag is inconsistent")
        process = read(attempt / "process.json")
        budget, elapsed = saved_manifest.get("timeout_seconds"), process.get("cold_e2e_seconds")
        if process.get("timed_out") is True:
            return _reject("External process timed out", "timeout")
        if (type(process.get("exit_code")) is not int or process["exit_code"] != 0
                or process.get("timed_out") is not False or process.get("exception")):
            return _reject("External process did not complete normally")
        if (not _finite(budget) or budget <= 0 or not _finite(elapsed) or elapsed < 0
                or process.get("timeout_seconds") != budget or result.get("budget_seconds") != budget):
            return _reject("Original deadline or external elapsed-time evidence is missing/inconsistent")
        if elapsed > budget:
            return _reject("Output completed after the original external query deadline", "timeout")
        validation = read(attempt / "validation.json")
        if (validation.get("passed") is not True or validation.get("status") not in RAW_SUCCESSES
                or any(validation.get(key) is not True for key in
                       ("collision_free", "joint_limits_passed", "derivative_limits_passed"))
                or any(validation.get(key) != 0 for key in
                       ("native_collisions", "independent_collisions", "model_disagreements", "contact_band_samples"))):
            return _reject("Independent numerical validation did not pass all required checks", "validation_failed")
        config = job.get("evaluation_config", {})
        endpoint, step = validation.get("endpoint_error_rad"), validation.get("sample_step_bound_rad")
        clearance, cost = validation.get("minimum_sampled_clearance_m"), validation.get("cost")
        endpoint_limit, step_limit = config.get("endpoint_tolerance_rad"), config.get("validation_step_rad")
        if (not _finite(endpoint_limit) or not _finite(step_limit)
                or not _finite(endpoint) or not 0 <= endpoint <= min(endpoint_limit, 1e-5)
                or not _finite(step) or not 0 < step <= min(step_limit, .005)
                or not _finite(clearance) or clearance <= 2e-6
                or not _finite(cost) or cost < 0
                or validation.get("cost_unit") != {"minimum_distance": "rad", "minimum_time": "s"}[objective]):
            return _reject("Independent validation limits, clearance or objective are invalid", "validation_failed")
        array_error = _array_consistency(attempt, candidate, validation, original, config)
        if array_error:
            return _reject(array_error, "validation_failed")
        return dict(accepted=True, status="success_numerical", reason=None,
                    status_basis=NONCONVERGED_STATUS_BASIS if raw == "solver_failed"
                    else "saved_candidate_independently_validated_within_budget")
    except (OSError, ValueError, KeyError, TypeError, IndexError, AttributeError) as exc:
        return _reject("Missing or malformed saved-output evidence: " + type(exc).__name__)
