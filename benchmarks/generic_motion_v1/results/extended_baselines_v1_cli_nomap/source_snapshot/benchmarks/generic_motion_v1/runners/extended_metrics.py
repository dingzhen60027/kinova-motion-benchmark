"""Sealed-evidence metrics for native sampling and trajectory baselines.

The independent OMPL searches share fixed task inputs and method seeds, but do
not use DRM. Conditional DRM statistics and shared-path checks apply only to
the four explicitly DRM-initialized pipelines.
"""
from __future__ import annotations

from collections import Counter, defaultdict
import argparse
import csv
import json
from pathlib import Path

import numpy as np

from ..common import read, write
from . import study_metrics as study


LABELS = ("LSCS", "RRTConnect", "RRT*", "CHOMP", "STOMP", "TrajOpt")
INDEPENDENT = frozenset(("RRTConnect", "RRT*"))
SHARED_DRM = frozenset(("LSCS", "CHOMP", "STOMP", "TrajOpt"))
METHOD_SCOPE = {
    "LSCS": dict(native_objective="Euclidean piecewise-linear joint-space path length",
                 initialization="validated native DRM path; EI-ZO corridor"),
    "RRTConnect": dict(native_objective="Find a feasible path, then native bounded vertex reduction",
                       initialization="Independent bidirectional OMPL search from frozen endpoints"),
    "RRT*": dict(native_objective="OMPL PathLengthOptimizationObjective; finite-budget rewiring and vertex reduction",
                 initialization="Independent OMPL search from frozen start and goal"),
    "CHOMP": dict(native_objective="CHOMP trajectory smoothness and canonical-geometry C-space arc-integral potential",
                  initialization="same validated native DRM path as LSCS"),
    "STOMP": dict(native_objective="STOMP trajectory smoothness and collision cost",
                  initialization="same validated native DRM path as LSCS"),
    "TrajOpt": dict(native_objective="TrajOpt sequential convex trajectory objective with collision penalties",
                    initialization="same validated native DRM path as LSCS"),
}
CONDITIONAL_DRM_FIELDS = ("conditional_native_drm", "conditional_validated_drm",
                          "conditional_resolved_validated_drm")


def independent_completed_stage_times(timing, result, candidate_present, validation_present):
    """Sampling has its own backend completion contract; no optimizer is run."""
    reasons = []
    if result.get("failed_stage"):
        reasons.append("failed_stage")
    if result.get("exception"):
        reasons.append("exception")
    if result.get("status") in ("timeout", "planner_timeout", "planner_failed", "unsupported",
                                 "infrastructure_error", "interrupted"):
        reasons.append("incomplete_terminal_status")
    if not candidate_present:
        reasons.append("no_candidate")
    if not validation_present:
        reasons.append("no_final_validation")
    backend_keys = frozenset(("sampling_planning_seconds", "validation_seconds"))
    if not backend_keys <= set(timing):
        reasons.append("missing_backend_completion_timers")
    complete = not reasons
    excluded = sorted(backend_keys.intersection(timing)) if not complete else []
    return ({key: value for key, value in timing.items() if key not in excluded},
            complete, excluded, reasons)


def _annotate(rows):
    """Keep native objectives distinct from the common evaluated PWL distance."""
    for row in rows:
        label = row["label"]
        if label not in LABELS or row["objective"] != "minimum_distance":
            raise ValueError("Extended study accepts only the six declared distance methods")
        result = read(Path(row["attempt"]) / "result.json")
        candidate_path = Path(row["attempt"]) / "candidate.json"
        candidate = read(candidate_path) if candidate_path.exists() else {}
        converged = candidate.get("solver_converged")
        if converged is not None and not isinstance(converged, bool):
            raise ValueError("Candidate solver_converged must be an explicit boolean or null")
        termination_source = next((key for key in ("termination", "solver_status", "native_status")
                                   if candidate.get(key) is not None), None)
        independent = label in INDEPENDENT
        if independent:
            if result.get("roadmap_used") is not False:
                raise ValueError("Independent OMPL attempt must explicitly record roadmap_used=False")
            if any(result.get(key) is not None for key in ("frontend_success", "frontend_native_success", "drm_success")):
                raise ValueError("Independent OMPL cannot claim a DRM frontend outcome")
            if any(row[key] is not None for key in ("native_drm_success", "drm_success", "path_sha256", "seed_cost_rad")):
                raise ValueError("Independent OMPL cannot carry a DRM path, outcome or seed cost")
            (row["completed_stage_seconds"], row["backend_timing_complete"],
             row["stage_timing_excluded_keys"], row["stage_timing_exclusion_reasons"]) = independent_completed_stage_times(
                row["timing"], result, candidate_path.exists(),
                (Path(row["attempt"]) / "validation.json").exists())
        elif result.get("roadmap_used") is False:
            raise ValueError("A declared DRM-initialized method cannot opt out of its common frontend")
        row.update(
            roadmap_used=not independent,
            roadmap_role="paired repetition identifier only; no graph loaded" if independent else "planning frontend",
            initialization=METHOD_SCOPE[label]["initialization"],
            native_objective=candidate.get("native_objective", result.get("native_objective", METHOD_SCOPE[label]["native_objective"])),
            native_objective_source="candidate" if "native_objective" in candidate else "result" if "native_objective" in result else "declared method definition",
            evaluation_objective="sum of Euclidean joint-space PWL segment lengths in radians",
            output_representation=candidate.get("kind"),
            candidate_present=candidate_path.exists(),
            solver_converged=converged,
            candidate_solver_status=candidate.get("solver_status"),
            candidate_native_status=candidate.get("native_status"),
            candidate_termination=candidate.get("termination"),
            reported_termination=candidate.get(termination_source) if termination_source else
                                 "unknown" if candidate_path.exists() else "no_candidate",
            termination_source=termination_source,
            drm_conditional_metrics_applicable=not independent,
        )
    return rows


def _check_pairings(rows):
    if not rows:
        raise ValueError("No observations supplied")
    study.assert_same_fixed_input(rows)
    pairing = defaultdict(list)
    for row in rows:
        pairing[(row["case_id"], row["roadmap_id"], row["method_seed"])].append(row)
    differences = []
    for key, group in pairing.items():
        if len(group) != len(LABELS) or {row["label"] for row in group} != set(LABELS):
            raise ValueError("Every fixed input and repetition must contain all six methods exactly once")
        # The hash check above binds source evidence; explicit endpoint equality
        # also makes accidental mixing immediately visible in this analyzer.
        for field in ("q_start", "q_goal", "split", "roadmap_nodes", "roadmap_seed"):
            if any(row[field] != group[0][field] for row in group[1:]):
                raise ValueError(f"Fixed paired input differs on {field}")
        shared = [row for row in group if row["label"] in SHARED_DRM]
        hashes = {row["path_sha256"] for row in shared}
        fronts = {(row["native_drm_success"], row["drm_success"]) for row in shared}
        missing_validated_path = any(row["input_legal"] and row["drm_success"] is True
                                     and row["path_sha256"] is None for row in shared)
        if len(hashes) > 1 or len(fronts) > 1 or missing_validated_path:
            differences.append(dict(key=list(key), methods=sorted(SHARED_DRM),
                path_hashes=sorted(map(str, hashes)), frontend_outcomes=sorted(map(str, fronts)),
                missing_validated_path=missing_validated_path))
    return differences


def _group_metrics(rows):
    result = study.group_metrics(rows)
    legal = [row for row in rows if row["input_legal"]]
    successful = [row for row in legal if row["success"]]
    result.update(
        termination_counts=dict(Counter(str(row["reported_termination"]) for row in legal)),
        converged_successes=sum(row["solver_converged"] is True for row in successful),
        validated_incumbent_nonconverged=sum(row["solver_converged"] is False for row in successful),
        validated_unknown_convergence=sum(row["solver_converged"] is None for row in successful),
        unknown_convergence=sum(row["solver_converged"] is None for row in legal),
        unknown_convergence_candidates=sum(row["candidate_present"] and row["solver_converged"] is None
                                           for row in legal),
        no_candidate_jobs=sum(not row["candidate_present"] for row in legal),
        convergence_scope=(
            "Native convergence is the candidate's explicit solver_converged boolean only; "
            "missing values stay unknown and are never inferred from solver_success or successful "
            "collision validation. Successful outputs are partitioned into explicitly converged, "
            "explicitly nonconverged validated incumbent, and unknown convergence. Unknown counts "
            "over all legal jobs include jobs without a candidate; the candidate-only count is also "
            "reported. Termination counts use candidate termination, then solver_status, then "
            "native_status, without replacing the common evaluated path-length cost."),
    )
    independent = rows[0]["label"] in INDEPENDENT
    if independent:
        for key in CONDITIONAL_DRM_FIELDS:
            result[key] = dict(applicable=False, estimate=None, numerator=None, denominator=None,
                               ci95=None, reason="Independent OMPL search does not use DRM")
        result["stage_timing_scope"] = (
            "Model/GPU setup timers are included when assigned after their calls return. "
            "Sampling and validation timers require a returned exact candidate, final validation, "
            "both completion timers, and no exception, failed stage or incomplete terminal status. "
            "An independent sampler has no trajectory-optimization stage. Incomplete sampling or "
            "validation measurements remain in raw timing and unconditional cold latency, but are "
            "excluded from completed-stage means. Normally returned collision-invalid candidates "
            "remain eligible for stage timing.")
    else:
        for key in CONDITIONAL_DRM_FIELDS:
            result[key]["applicable"] = True
    result.update(
        roadmap_used=not independent,
        native_objective=rows[0]["native_objective"],
        evaluation_objective=rows[0]["evaluation_objective"],
        initialization=rows[0]["initialization"],
        output_scope="All valid-input attempts form the success and latency denominator; quality conditions on common numerical validation",
    )
    return result


def analyze(runs, output):
    """Verify sources/configs/receipts before producing any comparative result."""
    rows, manifests = study.load_rows(runs)
    rows = _annotate(rows)
    differences = _check_pairings(rows)
    groups, paired = [], []
    for split, nodes in sorted({(row["split"], row["roadmap_nodes"]) for row in rows}):
        pool = [row for row in rows if (row["split"], row["roadmap_nodes"]) == (split, nodes)]
        strata = [("all", pool),
                  ("obstacle_main", [row for row in pool if row["family"] != "open_control"]),
                  ("open_control", [row for row in pool if row["family"] == "open_control"])]
        strata += [("family:" + family, [row for row in pool if row["family"] == family])
                   for family in sorted({row["family"] for row in pool if row["family"] != "open_control"})]
        strata += [("straight_line:" + str(value), [row for row in pool if row["family"] != "open_control"
                                                    and row["straight_line_collision"] is value])
                   for value in (True, False)]
        strata += [("distance_bin:" + value, [row for row in pool if row["family"] != "open_control"
                                               and row["distance_bin"] == value])
                   for value in ("short", "medium", "long")]
        for scope, sample in strata:
            if not sample:
                continue
            for label in LABELS:
                selected = [row for row in sample if row["label"] == label]
                groups.append(dict(split=split, roadmap_nodes=nodes, scope=scope,
                                   label=label, objective="minimum_distance", **_group_metrics(selected)))
            paired.extend(dict(split=split, roadmap_nodes=nodes, scope=scope, **pair)
                          for pair in study.pair_metrics(sample))
    drm = [{key: row[key] for key in ("case_id", "split", "cluster_id", "family", "roadmap_nodes",
            "actual_roadmap_nodes", "roadmap_seed", "method_seed", "input_legal", "native_drm_success",
            "drm_success", "seed_cost_rad", "seed_min_clearance_m", "frontend_component_seconds", "attempt")}
           for row in rows if row["label"] == "LSCS"]
    result = dict(
        schema_version=1, study="extended_native_baselines_v1",
        complete=not differences, comparable=not differences,
        incomparability_reason="Shared DRM path or outcome mismatch; diagnostic results only" if differences else None,
        requested_jobs=len(rows), expected_method_labels=list(LABELS), runs=manifests,
        groups=groups, paired_comparisons=paired, pairing_differences=differences,
        drm_references=drm, rows=rows,
        pairing_scope="All six methods use identical frozen endpoints, geometry, input hashes, wall-clock budget and repetition seed. Roadmap ID is a pairing identifier for OMPL only; those algorithms never load or query that roadmap.",
        frontend_scope="Only LSCS, CHOMP, STOMP and TrajOpt share a validated DRM seed path. Shared frontend equality checks and DRM-conditional rates exclude RRTConnect and RRT* entirely.",
        objective_scope="Common reported cost is validated Euclidean PWL path length. Native optimizer objectives differ and remain explicit per observation; CHOMP/STOMP/TrajOpt smoothness and collision penalties are not relabelled as an exact path-length objective.",
        statistics_scope="Scene-cluster bootstrap; all endpoint and repeated-seed observations in each scene stay together. 2000 PCG64 replicates, seed 45827. Open-control rows share one physical-world cluster and also have a separate stratum.",
        quality_scope="Paired costs condition on joint numerical-validation success; all legal attempts remain in primary success and cold-latency comparisons. Finite collision samples do not prove continuous collision freedom.",
        budget_scope="Cold wall time includes interpreter, dependency checks, model setup, planning and final validation. Offline DRM build is a separate cost; independent OMPL has no offline roadmap cost. Missing stage values are not zero-filled.",
        completeness_scope="Completeness of supplied sealed runs and all six method pairings. The study protocol separately binds the full case/seed matrix.",
    )
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    write(output / "metrics.json", result)
    scalar_fields = [key for key in rows[0] if all(not isinstance(row.get(key), (dict, list)) for row in rows)]
    with (output / "observations.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=scalar_fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
    if differences:
        raise ValueError("Shared DRM mismatch saved; comparative ranking is not accepted")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runs", type=Path, nargs="+", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = analyze(args.runs, args.output)
    print(json.dumps(dict(requested_jobs=result["requested_jobs"], complete=result["complete"],
                          comparable=result["comparable"], output=str(args.output))))


if __name__ == "__main__":
    main()
