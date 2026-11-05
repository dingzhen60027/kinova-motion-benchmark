#!/usr/bin/env python3
"""Read-only completion gate and derived comparison for the METIS/MC64 runs.

Run with ``bash local/python.sh results/nto_metis/complete.py``. Repeated
``--run PATH`` replaces the three default run paths; ``--methods`` declares the
complete new method cohort. No planner or collision query runs in this script.
Until every declared job is terminal, only coverage.json is written. Use
``--coverage-only`` for monitoring even after the last job completes.
"""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from contextlib import contextmanager
from datetime import datetime, timezone
import csv
import itertools
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import uuid

ROOT = Path(__file__).resolve().parents[4]
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

from benchmarks.generic_motion_v1 import batch
from benchmarks.generic_motion_v1.common import BASE, cases, read, recorded_path, resolve_case, sha, write
from benchmarks.generic_motion_v1.runners import results
from benchmarks.generic_motion_v1.runners.methods import INDEPENDENT, METHODS, objectives

SOURCE = "f4446e54ec"
NEW_METHODS = frozenset({"NTO_REFINED", "NTO_PAPER", "NTO"})
SPLIT_SEEDS = {"smoke": (0,), "dev": (0, 1, 2), "test": (0,)}
IPOPT = dict(linear_solver="spral", spral_order="metis", spral_scaling="mc64")
NATIVE_SHA = "6e06e85a0385cf0ac62483c76a4d959f3465883c8bab5739e84280d4005be80f"
VALIDATOR_PAIR = (
    "12086b43b1735ab8455b260235eb0c92b336cb2b2cbaef6c286d1d2f5f376b43",
    "98cb9244d0d73831082894275d0b15c7732c9ac1c27755feeff8e3b31883521c")
VALIDATOR_NOTE = ("The exact reviewed source pair separates the existing nonpositive B-spline active-duration "
                  "rejection from the clamping rejection to give a specific error message. "
                  "The accepted candidate set and numerical sampling/motion-limit protocol are unchanged.")
PAIR_FIELDS = (
    "problem_id", "scene_id", "cluster_id", "split", "category", "method", "objective", "replica_seed",
    "input_valid", "old_record_id", "new_record_id", "old_status", "new_status", "outcome",
    "old_objective_value", "new_objective_value", "common_success_cost_new_minus_old",
    "old_cold_e2e_seconds", "new_cold_e2e_seconds", "cold_seconds_new_minus_old",
    "old_source_commit", "new_source_commit", "old_attempt_path", "new_attempt_path", "objective_unit",
)


def relative(path):
    return str(Path(path).resolve().relative_to(ROOT))


def job_key(job):
    return job["case_id"], job["replica_seed"], job["method"], job["objective"]


def expected_keys(data, methods, seeds):
    return {(case["problem_id"], seed, method, objective)
            for case, seed, method in itertools.product(data, seeds, methods)
            for objective in objectives(method)}


def first_terminal(run, job):
    """Coverage only: receipt bytes are verified later by the common exporter."""
    attempts = sorted((run / "jobs" / job["job_id"] / "attempts").glob("[0-9][0-9][0-9][0-9]"))
    selected = None
    unsealed = []
    for attempt in attempts:
        receipt = attempt / "receipt.json"
        if not receipt.is_file():
            unsealed.append(relative(attempt))
            continue
        document = read(receipt)
        if not isinstance(document, dict) or not isinstance(document.get("status"), str):
            raise ValueError("Malformed receipt: " + str(receipt))
        if not document.get("retry_pending") and selected is None:
            selected = attempt
    return selected, unsealed, len(attempts)


def check_matrix(manifest, data, methods, seeds):
    jobs = manifest["jobs"]
    keys = [job_key(job) for job in jobs]
    expected = expected_keys(data, methods, seeds)
    if len(keys) != len(set(keys)) or set(keys) != expected:
        raise ValueError("Declared jobs differ from the complete fixed input/method/objective/seed matrix: "
                         + json.dumps(dict(missing=sorted(expected - set(keys))[:10],
                                           extra=sorted(set(keys) - expected)[:10],
                                           repeated=len(keys)-len(set(keys)))))
    if manifest.get("timeout_seconds") != 60.0:
        raise ValueError("Expected the common 60-second outer budget")
    if manifest.get("execution_mode", "end_to_end") != "end_to_end":
        raise ValueError("This cohort must use complete end-to-end queries")
    canonical = {row["problem_id"]: row for row in data}
    inventory = read(BASE / "configs/robot.json")
    cache = {}
    for job in jobs:
        case = resolve_case(job, cache)
        if case != canonical[job["case_id"]]:
            raise ValueError("A job differs from the canonical frozen input")
        for key in ("robot_sha256", "collision_geometry_sha256"):
            if case.get(key) != inventory[key]:
                raise ValueError("A job uses a different canonical robot/collision model")
        pairing = [p for p in case["paired_seeds"] if p["drm_seed"] == job["replica_seed"]]
        if len(pairing) != 1 or any(job.get(key) != pairing[0][key]
                                    for key in ("method_seed", "solver_seed")):
            raise ValueError("A job changed the frozen planner/solver seed")
        cfg = job["evaluation_config"]
        if cfg.get("online_budget_seconds") != 60.0:
            raise ValueError("Inner configuration differs from the outer 60-second budget")
        if cfg.get("nto_refined", {}).get("early_accept_validated", False) is not False:
            raise ValueError("Early validated acceptance must remain disabled")
        if job.get("execution_mode", "end_to_end") != "end_to_end":
            raise ValueError("A conditioned job cannot enter the complete-flow cohort")
    return dict(registered_problems=len(data), legal_problems=sum(c["validation_status"] == "valid_input" for c in data),
                methods=sorted(methods), seeds=list(seeds), planned_jobs=len(jobs))


def inspect_runs(paths, methods, source_commit):
    if not methods or not set(methods) <= NEW_METHODS or len(methods) != len(set(methods)):
        raise ValueError("Declare unique NTO methods with --methods")
    if len(paths) != len(set(paths)):
        raise ValueError("Duplicate run path")
    expected_source = subprocess.check_output(["git", "rev-parse", source_commit], cwd=ROOT, text=True).strip()
    anchor = read(paths[0]/"run_manifest.json")
    if anchor.get("source_code_commit") != expected_source:
        raise ValueError("First new run must bind the declared source freeze")
    frozen_sources = anchor["source_hashes"]
    datasets = {split: cases(BASE / "manifests" / (split + ".jsonl")) for split in SPLIT_SEEDS}
    old = {split: BASE / "results" / ("baseline_" + split) for split in SPLIT_SEEDS}
    old_manifests = {split: read(path / "run_manifest.json") for split, path in old.items()}
    old_jobs = {}
    for split, manifest in old_manifests.items():
        check_matrix(manifest, datasets[split], METHODS, SPLIT_SEEDS[split])
        old_jobs[split] = {job_key(job): job for job in manifest["jobs"]}
    manifests, progress, entries = {}, [], []
    all_keys = defaultdict(set)
    for run in paths:
        manifest = read(run / "run_manifest.json")
        split = manifest["split"]
        if split not in datasets:
            raise ValueError("Unknown split")
        declared = sorted({job["method"] for job in manifest["jobs"]})
        if not declared or not set(declared) <= set(methods):
            raise ValueError("Run includes an undeclared method or no jobs")
        checked = check_matrix(manifest, datasets[split], declared, SPLIT_SEEDS[split])
        if (read(run / "identity.json").get("run_manifest_sha256") != sha(run / "run_manifest.json")
                or manifest.get("manifest_sha256") != sha(BASE / "manifests" / (split + ".jsonl"))):
            raise ValueError("Run/input identity changed")
        validator_pair = (old_manifests[split]["source_hashes"].get("benchmarks/generic_motion_v1/runners/validate.py"),
                          manifest["source_hashes"].get("benchmarks/generic_motion_v1/runners/validate.py"))
        if validator_pair[0] != validator_pair[1] and validator_pair != VALIDATOR_PAIR:
            raise ValueError("Independent validator source differs from the baseline")
        if manifest["source_hashes"] != frozen_sources:
            raise ValueError("New runs differ from the declared frozen runtime source files")
        states, actual_attempts, pending, unsealed = Counter(), 0, 0, []
        for job in manifest["jobs"]:
            key = job_key(job)
            if key in all_keys[split]:
                raise ValueError("New runs duplicate a task/method/objective/seed")
            all_keys[split].add(key)
            original = old_jobs[split][key]
            new_cfg = dict(job["evaluation_config"])
            if new_cfg.pop("ipopt", None) != IPOPT or new_cfg != original["evaluation_config"]:
                raise ValueError("Configuration changed beyond explicit METIS/MC64 options")
            for field in ("method_seed", "solver_seed", "roadmap_id", "roadmap_dir", "roadmap_used"):
                if job.get(field) != original.get(field):
                    raise ValueError("New and original jobs differ on " + field)
            selected, running, attempts = first_terminal(run, job)
            actual_attempts += attempts
            unsealed.extend(running)
            pending += selected is None
            if selected:
                states[read(selected / "receipt.json")["status"]] += 1
            entries.append(dict(run=run, manifest=manifest, job=job, attempt=selected))
        progress.append(dict(run=relative(run), split=split, **checked, pending=pending,
            completed=checked["planned_jobs"]-pending, actual_attempts=actual_attempts,
            unsealed_attempts=unsealed, raw_receipt_statuses=dict(states),
            run_manifest_sha256=sha(run / "run_manifest.json")))
        manifests[run] = manifest
    for split, data in datasets.items():
        if all_keys[split] != expected_keys(data, methods, SPLIT_SEEDS[split]):
            raise ValueError("All three complete splits and every requested method must be declared: " + split)
    return dict(datasets=datasets, old=old, old_manifests=old_manifests,
                manifests=manifests, progress=progress, entries=entries, source=expected_source)


def check_solver_options(entries):
    checked = []
    for entry in entries:
        attempt = entry["attempt"]
        if attempt is None:
            raise ValueError("Solver evidence requested before completion")
        parameters = list(attempt.rglob("nto_parameters.json"))
        checked_here = 0
        for file in parameters:
            document = read(file)
            options = document.get("parameters", {})
            uses_ipopt = document.get("solver") == "IPOPT" or options.get("linear_solver") == "spral"
            if not uses_ipopt:
                continue  # Published-profile distance uses SNOPT, not IPOPT.
            if any(options.get(k) != v for k, v in IPOPT.items()):
                raise ValueError("Actual IPOPT options differ: " + str(file))
            linear = document.get("linear_system", {})
            binary = linear.get("native_binary", {})
            if (linear.get("options") != IPOPT or linear.get("avoids_matching_order") is not True
                    or linear.get("constraints_changed") is not False
                    or linear.get("native_library_patched") is not False
                    or binary.get("loaded_matches_package") is not True
                    or binary.get("sha256") != NATIVE_SHA or binary.get("drake_version") != "1.51.1"):
                raise ValueError("IPOPT native backend evidence differs: " + str(file))
            checked.append(dict(path=relative(file), sha256=sha(file)))
            checked_here += 1
        if ((attempt/"candidate.npz").is_file()
                and (entry["job"]["method"] == "NTO" or entry["job"]["objective"] == "minimum_time")
                and checked_here == 0):
            raise ValueError("Returned IPOPT candidate lacks actual option evidence")
        # Every returned or interrupted main solve that produced an IPOPT log
        # must have its actual option record. Frontend/construction failures are
        # retained and do not falsely assert that a solver was invoked.
        for log in attempt.rglob("ipopt.log"):
            if not (log.parent / "nto_parameters.json").is_file():
                raise ValueError("IPOPT log lacks bound option record: " + str(log))
        for file in attempt.rglob("nto_refinement.json"):
            if read(file).get("early_accept_validated", False) is not False:
                raise ValueError("Actual refinement enabled early acceptance")
    return dict(checked_parameter_files=len(checked), files=checked,
                scope="Every recorded main/refinement IPOPT invocation; failures before solver construction remain outcomes")


def frontend_differences(entries, old_paths, old_manifests, methods):
    groups = defaultdict(list)
    combined = list(entries)
    for split in ("dev", "test"):
        for job in old_manifests[split]["jobs"]:
            if job["method"] in methods or job["method"] in INDEPENDENT:
                continue
            attempt, unsealed, _ = first_terminal(old_paths[split], job)
            if attempt is None or unsealed:
                raise ValueError("Historical control is incomplete")
            combined.append(dict(run=old_paths[split], job=job, attempt=attempt))
    for entry in combined:
        job, attempt = entry["job"], entry["attempt"]
        split = job["case_id"].split("-", 1)[0]
        if split not in ("dev", "test"):
            continue
        frontend = attempt / "frontend_validation/validation.json"
        passed = frontend.is_file() and read(frontend).get("passed") is True
        path = attempt / "path.npy"
        groups[(split, job["case_id"], job["replica_seed"], job["objective"])].append(dict(
            method=job["method"], passed=passed, path_sha256=sha(path) if path.is_file() else None,
            attempt_path=relative(attempt)))
    differences = []
    for key, rows in sorted(groups.items()):
        eligible = [row for row in rows if row["passed"]]
        if eligible and (len(eligible) != len(rows) or eligible[0]["path_sha256"] is None
                         or len({row["path_sha256"] for row in eligible}) != 1):
            differences.append(dict(split=key[0], problem_id=key[1], replica_seed=key[2],
                                    objective=key[3], observations=rows))
    return differences


def make_selection(paths, context, methods, differences):
    controls = [method for method in METHODS if method not in methods]
    comparisons, figures = [], []
    for split in ("dev", "test"):
        new = [{"path": relative(path), "methods": sorted({job["method"] for job in context["manifests"][path]["jobs"]})}
               for path in paths if context["manifests"][path]["split"] == split]
        note = ("New NTO runtime source freeze " + context["source"] + "; equivalent commits with identical bound runtime files retain their actual Git identities. Unchanged controls retain their original source/configuration. "
                "Resolved configuration differs only by explicit IPOPT linear_solver=spral, spral_order=metis and spral_scaling=mc64; "
                "early_accept_validated stays disabled. Frozen inputs, robot/collision geometry, numerical validation protocol, motion limits, "
                "60-second cold deadline, paired seeds and DRM identities are checked before export. "
                "The exact reviewed validator source pair changes only the specific nonpositive B-spline-duration rejection message; its accepted candidate set is unchanged. "
                "Source changes additionally contain read-only model audit and evidence/accounting repairs; all rows use the common current saved-output acceptance. "
                "Old NTO profiles not rerun are explicitly historical configurations. No outcome-based substitution or best-of retry.")
        identifier = "current_" + split + "_nto_metis"
        comparisons.append(dict(id=identifier, mode="end_to_end", compatibility_note=note,
            runs=[dict(path=relative(context["old"][split]), methods=controls)] + new))
        figures.append(identifier)
        if not any(row["split"] == split for row in differences):
            comparisons.append(dict(id=identifier+"_fixed_frontend", mode="fixed_frontend", compatibility_note=note,
                runs=[dict(path=relative(context["old"][split]), methods=[m for m in controls if m not in INDEPENDENT])] + new))
    return dict(schema_version=1, attempt_policy="first_terminal", comparisons=comparisons, figure_comparisons=figures)


@contextmanager
def require_complete_verification(proofs):
    """Strengthen the exporter's own verification, without skipping any check."""
    original = batch.verify
    def checked(run, *args, **kwargs):
        kwargs["allow_unsealed"] = False
        proof = original(run, *args, **kwargs)
        proofs[relative(run)] = proof
        if not proof["integrity_passed"] or not proof["complete"]:
            raise ValueError("Refusing incomplete/unverified final evidence: " + str(run))
        return proof
    batch.verify = checked
    try:
        yield
    finally:
        batch.verify = original


def csv_rows(path):
    with Path(path).open(newline="") as stream:
        return list(csv.DictReader(stream))


def numeric(value):
    return float(value) if value not in (None, "") else None


def paired_before_after(rows, paths, old_paths, methods):
    first = {}
    for row in rows:
        if row["lifecycle"] != "terminal":
            continue
        key = (row["run"], row["problem_id"], row["replica_seed"], row["method"], row["objective"])
        if key not in first or int(row["attempt_number"]) < int(first[key]["attempt_number"]):
            first[key] = row
    new_paths = {relative(path) for path in paths}
    pairs = []
    for key, new in sorted(first.items()):
        if key[0] not in new_paths or new["method"] not in methods:
            continue
        old = first[(relative(old_paths[new["split"]]), *key[1:])]
        if any(new[field] != old[field] for field in ("input_sha256", "robot_sha256", "geometry_sha256",
                "validation_protocol_sha256", "budget_seconds", "planner_seed", "roadmap_id", "cluster_id", "input_valid")):
            raise ValueError("Before/after evidence differs on a fixed comparison input")
        a, b = old["status"] == results.SUCCESS, new["status"] == results.SUCCESS
        valid = new["input_valid"] == "True"
        cost_old, cost_new = numeric(old["objective_value"]), numeric(new["objective_value"])
        cold_old, cold_new = numeric(old["cold_e2e_seconds"]), numeric(new["cold_e2e_seconds"])
        row = {field: new[field] for field in PAIR_FIELDS[:9]}
        row.update(old_record_id=old["record_id"], new_record_id=new["record_id"],
            old_status=old["status"], new_status=new["status"],
            outcome=("invalid_input" if not valid else "both_success" if a and b else
                     "new_only_success" if b else "old_only_success" if a else "neither_success"),
            old_objective_value=cost_old, new_objective_value=cost_new,
            common_success_cost_new_minus_old=cost_new-cost_old if a and b else None,
            old_cold_e2e_seconds=cold_old, new_cold_e2e_seconds=cold_new,
            cold_seconds_new_minus_old=cold_new-cold_old if valid and cold_new is not None and cold_old is not None else None,
            old_source_commit=old["source_commit"], new_source_commit=new["source_commit"],
            old_attempt_path=old["attempt_path"], new_attempt_path=new["attempt_path"],
            objective_unit="rad" if new["objective"] == "minimum_distance" else "s")
        pairs.append(row)
    expected = sum(len(read(path / "run_manifest.json")["jobs"]) for path in paths)
    if len(pairs) != expected:
        raise ValueError("Before/after table is missing declared jobs")
    summaries = []
    for split, method, objective in sorted({(r["split"], r["method"], r["objective"]) for r in pairs}):
        block = [r for r in pairs if (r["split"],r["method"],r["objective"]) == (split,method,objective)]
        for category in ["all"] + sorted({r["category"] for r in block}):
            legal = [r for r in block if r["outcome"] != "invalid_input" and (category == "all" or r["category"] == category)]
            base = dict(split=split, method=method, objective=objective, category=category, legal_pairs=len(legal),
                        **{name: sum(r["outcome"] == name for r in legal) for name in
                           ("both_success", "old_only_success", "new_only_success", "neither_success")})
            for field in ("common_success_cost_new_minus_old", "cold_seconds_new_minus_old", "success_difference"):
                sample = [dict(cluster_id=r["cluster_id"], value=(int(r["new_status"] == results.SUCCESS)-int(r["old_status"] == results.SUCCESS)
                          if field == "success_difference" else r[field])) for r in legal]
                unit = "fraction" if field == "success_difference" else "s" if field == "cold_seconds_new_minus_old" else "rad" if objective == "minimum_distance" else "s"
                scope = "all_legal_matched_outcomes" if field == "success_difference" else "all_legal_matched_cold_latencies" if field == "cold_seconds_new_minus_old" else "common_numerically_successful_only"
                summaries.append(dict(base, metric=field, unit=unit, scope=scope, **results._stats(sample, "value")))
    return pairs, summaries


def check_export_paths(staged, output):
    """Check references in their final locations before atomic publication."""
    if staged.parent != output.parent:
        raise ValueError("Staging and published output must be sibling directories")
    inventory = read(staged/"inventory.json")
    if (inventory.get("summary_selection") != relative(output/"selection.json")
            or inventory.get("summary_selection_sha256") != sha(staged/"selection.json")):
        raise ValueError("Inventory selection reference/hash would break after publication")
    for item in inventory.get("runs", []):
        file = recorded_path(item["run"])/"run_manifest.json"
        if not file.is_file() or sha(file) != item["manifest_sha256"]:
            raise ValueError("Inventory source-run reference changed")
    count = 0
    for folder in ("logs", "trajectories"):
        for link in (staged/folder).iterdir() if (staged/folder).is_dir() else []:
            if not link.is_symlink() or not link.resolve().is_dir():
                raise ValueError("Export link is not a valid source-run directory")
            final_target = output/folder/os.readlink(link)
            if final_target.resolve() != link.resolve():
                raise ValueError("Export link would change target after publication")
            count += 1
    return dict(selection_reference_valid=True, inventory_run_references=len(inventory.get("runs", [])),
                relocated_directory_links=count, staging_and_output_are_siblings=True)


def publish(staged, output):
    """Publish the entire verified tree; failures cannot expose a partial CSV."""
    if staged.parent != output.parent:
        raise ValueError("Publication must stay on the same directory level/filesystem")
    previous = None
    if output.exists():
        prior = list(output.iterdir())
        if any(not file.is_file() or file.is_symlink() or file.name not in {"coverage.json", "failed_check.json"}
               for file in prior):
            raise ValueError("Final output already contains artifacts; choose a new --output")
        # A prior diagnostic is unique failure evidence. Its exact bytes travel
        # with the final export; the pending coverage is a rebuildable view.
        failure = output/"failed_check.json"
        if failure.is_file():
            shutil.copyfile(failure, staged/"failed_check.json")
        previous = output.with_name("."+output.name+".previous-"+uuid.uuid4().hex[:10])
        output.rename(previous)
    try:
        staged.rename(output)
    except Exception:
        if previous is not None:
            previous.rename(output)
        raise
    if previous is not None:
        # Only this script's pending view/retained diagnostic can be present.
        for file in previous.iterdir():
            file.unlink()
        previous.rmdir()


def run(paths, methods, output, source_commit=SOURCE, coverage_only=False):
    output = Path(output).resolve()
    if not output.is_relative_to(HERE) or output == HERE:
        raise ValueError("Derived output must be a child of results/nto_metis")
    protected = {relative(file): sha(file) for file in (BASE/"results/records.csv", BASE/"results/summary.csv") if file.is_file()}
    context = inspect_runs(paths, methods, source_commit)
    coverage = dict(schema_version=1, checked_utc=datetime.now(timezone.utc).isoformat(), methods=sorted(methods), source_commit=context["source"], runs=context["progress"],
        completed=sum(row["completed"] for row in context["progress"]), pending=sum(row["pending"] for row in context["progress"]),
        coverage_only=True, final_score=False,
        status_scope="Unverified receipt lifecycle only; no success rates or costs until every declared job is complete")
    if coverage_only or coverage["pending"] or any(row["unsealed_attempts"] for row in context["progress"]):
        if (output/"summary.csv").exists():
            raise ValueError("Incomplete cohort conflicts with previously exported summary; use a new output path")
        write(output / "coverage.json", coverage)
        return coverage
    solver = check_solver_options(context["entries"])
    differences = frontend_differences(context["entries"], context["old"], context["old_manifests"], methods)
    selection = make_selection(paths, context, methods, differences)
    # Stage at the same directory depth as the published output so its relative
    # trajectory/log links stay correct after publication. A failed check never
    # leaves summary.csv in the requested final output.
    staged = output.with_name("."+output.name+".checking-"+uuid.uuid4().hex[:10])
    write(staged/"selection.json", selection)
    write(staged/"frontend_differences.json", dict(differences=differences,
        policy="No mismatch is filtered out; a split with any mismatch has no fixed_frontend comparison"))
    proofs = {}
    # Export collects arrays/receipts once and verifies each selected run once
    # across all comparison modes. No separate repeated 9k-record audit loop.
    with require_complete_verification(proofs):
        exported = results.export(staged, roots=paths + list(context["old"].values()),
                                  selection=staged/"selection.json", verify_files=True)
    # Smoke is preserved in records but has no formal comparative score. Its
    # full provenance is checked explicitly because it is not in selection.
    for path in paths + [context["old"]["smoke"]]:
        if relative(path) not in proofs:
            proof = batch.verify(path)
            proofs[relative(path)] = proof
            if not proof["integrity_passed"] or not proof["complete"]:
                raise ValueError("Smoke provenance is incomplete/invalid")
    pairs, paired_summary = paired_before_after(csv_rows(staged/"records.csv"), paths, context["old"], methods)
    results._csv(staged/"paired_before_after.csv", pairs, PAIR_FIELDS)
    fields = ("split", "method", "objective", "category", "metric", "unit", "scope", "legal_pairs", "both_success", "old_only_success",
              "new_only_success", "neither_success", "count", "scene_clusters", "mean", "median", "p90", "p95", "ci_low", "ci_high")
    results._csv(staged/"paired_before_after_summary.csv", paired_summary, fields)
    if any(sha(ROOT/name) != digest for name,digest in protected.items()):
        raise ValueError("Historical primary tables changed during read-only derivation")
    if any(sha(recorded_path(row["run"])/"run_manifest.json") != row["run_manifest_sha256"]
           for row in context["progress"]):
        raise ValueError("A new run manifest changed during derivation")
    # Receipt collection also checks the numerical candidate arrays. Inspect
    # complete exported coverage independently of plot/summary filtering.
    rows = csv_rows(staged/"records.csv")
    new_paths = {relative(path) for path in paths}
    terminal = [r for r in rows if r["run"] in new_paths and r["lifecycle"] == "terminal"]
    if any(r["receipt_integrity"] != "True" for r in terminal):
        raise ValueError("New evidence contains a failed receipt check")
    inventory = read(staged/"inventory.json")
    inventory["summary_selection"] = relative(output/"selection.json")
    write(staged/"inventory.json", inventory)
    exported["output"] = relative(output)
    relocation = check_export_paths(staged, output)
    final = dict(coverage, coverage_only=False, final_score=True, complete=True, exported=exported,
                 solver_options=solver, full_verification=proofs,
                 published_reference_check=relocation,
                 fixed_frontend_mismatches=len(differences), paired_before_after_rows=len(pairs),
                 preserved_primary_tables=protected,
                 actual_new_source_commits=sorted({manifest["source_code_commit"] for manifest in context["manifests"].values()}),
                 validator_source_compatibility=dict(exact_reviewed_pair=list(VALIDATOR_PAIR), note=VALIDATOR_NOTE),
                 before_after_scope="Observational matched comparison across source revisions, not a controlled single-factor ablation. "
                    "Other source repairs include classification, positive spline duration, disabled-by-default early acceptance, "
                    "read-only model auditing and accounting/resume fixes. Original native-library hashes are unknown where not sealed; "
                    "the new library hash is never imputed to old attempts.",
                 source_differences={split: sorted(name for name in
                     set(context["old_manifests"][split]["source_hashes"]) | set(next(iter(context["manifests"].values()))["source_hashes"])
                     if context["old_manifests"][split]["source_hashes"].get(name) != next(iter(context["manifests"].values()))["source_hashes"].get(name))
                     for split in ("dev", "test")},
                 outputs={name: sha(staged/name) for name in ("selection.json", "records.csv", "summary.csv", "inventory.json",
                    "frontend_differences.json", "paired_before_after.csv", "paired_before_after_summary.csv")})
    final["status_scope"] = "Complete matrix and sealed artifacts verified; common saved-output numerical acceptance, no new collision queries"
    write(staged/"coverage.json", final)
    write(staged/"verification.json", final)
    publish(staged, output)
    return final


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", action="append", help="Repeat for every new run; replaces all three defaults")
    parser.add_argument("--methods", nargs="+", default=["NTO_REFINED"], choices=sorted(NEW_METHODS))
    parser.add_argument("--source-commit", default=SOURCE)
    parser.add_argument("--output", default="results/nto_metis/comparison")
    parser.add_argument("--coverage-only", action="store_true", help="Never produce scores, even when every job is complete")
    args = parser.parse_args(argv)
    paths = [recorded_path(p) for p in (args.run or ["results/nto_metis_"+s for s in SPLIT_SEEDS])]
    try:
        result = run(paths, args.methods, recorded_path(args.output), args.source_commit, args.coverage_only)
    except Exception as exc:
        output = recorded_path(args.output)
        if output.is_relative_to(HERE) and output != HERE:
            write(output/"failed_check.json", dict(final_score=False, error=repr(exc)))
        print(json.dumps(dict(final_score=False,error=repr(exc)), indent=2), file=sys.stderr)
        return 1
    print(json.dumps(dict(output=relative(recorded_path(args.output)), completed=result["completed"],
                         pending=result["pending"], final_score=result["final_score"]), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
