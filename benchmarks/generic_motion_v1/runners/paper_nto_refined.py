"""Declared numerical refinement of the paper NTO transcription.

This is a separate development baseline, not the published author profile.
It keeps the model, objectives, solvers, ordered DRM path and final validator.
A fixed clearance, explicit iteration limits and constraint generation improve
numerical reliability. Shape-preserving knot insertion gives the time optimizer
enough freedom after DRM simplification. CPU checks are not a certificate.
"""
from __future__ import annotations

from copy import deepcopy
from pathlib import Path
import shutil
import time
import traceback

import numpy as np

from ..roadmap import write
from .paper_nto import PROFILE as PAPER_PROFILE, paper_nto, dependency_probe
from .validate import sample_candidate


PROFILE = deepcopy(PAPER_PROFILE)
PROFILE.update(
    name="rss2025_nto_numerical_refinement_v2",
    provenance="declared_NTO_refinement_on_frozen_Kinova_not_exact_author_configuration",
    clearance_m=0.001,
    snopt_total_iterations=100000,
    snopt_minor_iterations=500,
    refinement_guard_m=0.0005,
    maximum_native_solves=4,
    maximum_collocation_points=600,
    refinement_geometry_batch=32,
    minimum_time_control_points=16,
)
PROFILE["sources"].update(
    clearance="Author archived appendix describes epsilon as the minimum admissible distance but gives no value. The fixed 1 mm value is a declared local development policy.",
    refinement="Local constraint generation: after each native solve, add the minimum-clearance sampled parameter in each current interval with clearance below 0.5 mm. At most 4 solves and 600 total points. No case-specific policy.",
    time_parameterization="Declared adapter choice, matching the existing legacy NTO basis policy: bisect longest knot spans to at least 16 time-spline controls, preserving the entire initial curve. Distance controls are unchanged. All methods retain identical derivative limits and endpoint conditions.",
    iteration_limits="Local policy: SNOPT total 100000 / major 2000 / minor 500; IPOPT max_iter 2000. All construction, solves and refinement checks share the external online deadline.",
)


def refinement_locations(locations, parameters, clearance):
    """One worst interior sample per violated interval, under one global cap."""
    locations = np.asarray(locations, float)
    parameters = np.asarray(parameters, float)
    clearance = np.asarray(clearance, float)
    if parameters.shape != clearance.shape or not np.isfinite(clearance).all():
        raise ValueError("Refinement requires matching finite sampled clearances")
    bounds = np.r_[0.0, locations, 1.0]
    proposals = []
    for left, right in zip(bounds[:-1], bounds[1:]):
        idx = np.flatnonzero((parameters > left + 1e-12)
                             & (parameters < right - 1e-12)
                             & (clearance < PROFILE["refinement_guard_m"]))
        if len(idx):
            worst = idx[np.argmin(clearance[idx])]
            proposals.append((float(clearance[worst]), float(parameters[worst])))
    # If the global cap binds, prioritize smallest clearance, then parameter.
    proposals.sort()
    proposals = proposals[:max(0, PROFILE["maximum_collocation_points"] - len(locations))]
    return np.unique([point for _, point in proposals])


def _geometry_guard(candidate, model, cfg, out, deadline):
    started = time.perf_counter()
    samples = sample_candidate(candidate, cfg)
    q = samples["q"]
    parameters = np.asarray(samples["time"], float) / float(candidate["duration"])
    clearance = np.full(len(q), np.nan)
    summary = dict(purpose="CPU constraint refinement only; common final validation still required",
                   samples=len(q), sampling_step_rad=cfg["validation_step_rad"],
                   guard_m=PROFILE["refinement_guard_m"], passed=False)
    try:
        for first in range(0, len(q), PROFILE["refinement_geometry_batch"]):
            if time.perf_counter() >= deadline:
                raise TimeoutError("NTO refinement geometry check exhausted shared online budget")
            last = first + PROFILE["refinement_geometry_batch"]
            clearance[first:last] = model.clearance(q[first:last])
        if not np.isfinite(clearance).all():
            raise ValueError("Nonfinite independent clearance in NTO refinement")
        summary.update(minimum_clearance_m=float(clearance.min()),
                       below_guard_samples=int(np.sum(clearance < PROFILE["refinement_guard_m"])),
                       collision_samples=int(np.sum(clearance <= 0)),
                       cost=samples["cost"],
                       passed=bool(np.all(clearance >= PROFILE["refinement_guard_m"])))
    except Exception as exc:
        summary.update(exception=repr(exc), traceback=traceback.format_exc())
        raise
    finally:
        summary["seconds"] = time.perf_counter() - started
        np.savez_compressed(out / "refinement_geometry.npz", q=q,
                            parameter=parameters, clearance=clearance)
        write(out / "refinement_geometry.json", summary)
    return parameters, clearance, summary


def _publish_native_artifacts(source, out, first):
    # Keep each solve immutable under refinement/round_NN. Top-level native
    # result/raw/log is the final returned solve for existing inspection tools.
    names = ["nto_parameters.json", "nto_solver_result.json", "nto_solver_raw.npz",
             "snopt.log", "ipopt.log"]
    if first:
        names.append("nto_initial.npz")
    for name in names:
        path = source / name
        if path.exists():
            shutil.copyfile(path, out / name)


def paper_nto_refined(path, model, case, cfg, out, deadline):
    """Run one fixed policy within the caller's shared wall-clock deadline.

    The return has the common candidate schema. No failed solve is relabelled
    successful, no seed fallback is returned as an optimized trajectory, and no
    final validator tolerance is relaxed. Native exceptions remain exceptions.
    """
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    started = time.perf_counter()
    profile = deepcopy(PROFILE)
    settings = cfg.get("nto_refined", {})
    if (not isinstance(settings, dict)
            or set(settings) - {"minimum_time_control_points"}):
        raise ValueError("Unsupported NTO_REFINED settings")
    minimum_controls = settings.get("minimum_time_control_points",
                                    profile["minimum_time_control_points"])
    if (isinstance(minimum_controls, (bool, np.bool_))
            or not isinstance(minimum_controls, (int, np.integer)) or minimum_controls < 4):
        raise ValueError("NTO_REFINED minimum_time_control_points must be an integer >= 4")
    profile["minimum_time_control_points"] = int(minimum_controls)
    # Do not reset an already partly used query budget at a refinement round.
    deadline = min(float(deadline), started + float(cfg["optimizer_budget_seconds"]),
                   started + PROFILE["maximum_optimizer_seconds"])
    if not np.isfinite(deadline):
        raise ValueError("NTO refinement requires a finite deadline")
    locations = np.arange(1, PROFILE["collocation_parameter_Nc"]) / PROFILE["collocation_parameter_Nc"]
    manifest = dict(profile=profile, rounds=[], final_validation_required=True,
                    initial_remaining_seconds=deadline-started,
                    source_profile=PAPER_PROFILE["name"],
                    geometry_model_changed=False,
                    controls_inserted=bool(case.get("_objective") == "minimum_time"
                                           and len(path)+2 < minimum_controls),
                    minimum_time_control_points=int(minimum_controls),
                    initial_curve_preserved=True,
                    stopping_reason="running")
    write(out / "nto_refinement.json", manifest)
    candidate = None
    try:
        for index in range(PROFILE["maximum_native_solves"]):
            if time.perf_counter() >= deadline:
                raise TimeoutError("NTO refinement exhausted shared online budget")
            target = out / "refinement" / f"round_{index:02d}"
            target.mkdir(parents=True, exist_ok=False)
            row = dict(index=index, directory=str(target.relative_to(out)),
                       collocation_points=len(locations), collocation_locations=locations,
                       remaining_seconds=deadline-time.perf_counter(), status="running")
            manifest["rounds"].append(row)
            write(out / "nto_refinement.json", manifest)
            tic = time.perf_counter()
            try:
                candidate = paper_nto(path, model, case, cfg, target, deadline,
                                     _profile=profile, _locations=locations,
                                     _initial_candidate=candidate)
            finally:
                _publish_native_artifacts(target, out, first=index == 0)
            row.update(solve_and_construction_seconds=time.perf_counter()-tic,
                       solver_success=candidate["solver_success"],
                       solver_status=candidate["solver_status"])
            write(target / "candidate.json", candidate)
            np.savez_compressed(target / "candidate.npz", controls=candidate["controls"],
                                knots=candidate["knots"], duration=candidate["duration"])
            parameters, clearance, guard = _geometry_guard(candidate, model, cfg, target, deadline)
            row["geometry_guard"] = guard
            added = refinement_locations(locations, parameters, clearance)
            row["proposed_collocation_locations"] = added
            row["added_collocation_locations"] = []
            row["status"] = "completed"
            if guard["passed"]:
                manifest["stopping_reason"] = "geometry_guard_passed"
                break
            if not len(added):
                manifest["stopping_reason"] = "no_new_interior_refinement_point"
                break
            if index + 1 == PROFILE["maximum_native_solves"]:
                manifest["stopping_reason"] = "native_solve_limit"
                break
            row["added_collocation_locations"] = added
            locations = np.unique(np.r_[locations, added])
            write(out / "nto_refinement.json", manifest)
        if time.perf_counter() >= deadline:
            raise TimeoutError("NTO refinement exhausted shared online budget after geometry check")
        candidate.update(refinement_profile=PROFILE["name"],
                         refinement_native_solves=len(manifest["rounds"]),
                         refinement_stopping_reason=manifest["stopping_reason"],
                         refinement_final_geometry_guard_passed=guard["passed"],
                         refinement_seconds=time.perf_counter()-started)
        return candidate
    except Exception as exc:
        manifest.update(stopping_reason="exception", exception=repr(exc),
                        traceback=traceback.format_exc())
        if manifest["rounds"] and manifest["rounds"][-1]["status"] == "running":
            manifest["rounds"][-1]["status"] = "exception"
        raise
    finally:
        manifest["elapsed_seconds"] = time.perf_counter()-started
        write(out / "nto_refinement.json", manifest)
