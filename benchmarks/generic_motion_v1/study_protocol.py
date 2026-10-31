"""Fail-closed authorization and frozen inputs for formal study preparation.

Schema v1 (all file/directory paths are absolute and canonical):
  schema_version: 1; state: "approved";
  authorization: {text: <verbatim human authorization>, source: <message reference>};
  manifest: {path, sha256}; input_audit: {path, sha256};
  runtime_source_hashes: {<repository-relative source path>: sha256};
  metrics_contract: {path, sha256};
  model: {robot_sha256, collision_geometry_sha256, files: {path: sha256}};
  data_files: {path: sha256};  # exact manifest + every scene/input-array file
  blocks: {name: {
    selection: "all" | "fixed_subset", problem_count: <integer>,
    problem_ids: [<ordered, unique IDs from the original manifest>],
    roadmap_dirs: [<ordered directories>], roadmap_files: {path: sha256},
    method_pairs: [[method, objective], ...],  # requested Cartesian order, no LGCS-time
    evaluation_config: <fully resolved config>, timeout_seconds: <positive number>
  }}

An input audit must pass, match the manifest SHA/split/count, and contain one
passing classification record per fixed problem, including invalid inputs.
"all" must list every original ID; "fixed_subset" never rewrites a manifest.
Protocol, audit, input, model and map files are read and checked before output
creation, and the exact protocol/audit bytes are copied into each prepared run.
The original protocol and audit must remain available for verify/resume.

This module cannot authenticate a human conversation. An operator must record
real authorization; merely constructing an "approved" JSON is not permission.
No function creates an approved protocol, edits frozen data, or launches a GPU.
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import re

from .common import ROOT, cases, code_inventory, read, sha, recorded_path

MODEL_IDENTITIES = ("robot_sha256", "collision_geometry_sha256")
METHODS = {"EI_ZO", "LGCS", "NTO", "NTO_PAPER", "NTO_REFINED"}
OBJECTIVES = {"minimum_distance", "minimum_time"}


def method_pairs(methods, objectives):
    return [[method, objective] for objective in objectives for method in methods
            if not (method == "LGCS" and objective != "minimum_distance")]


def _path(value, description):
    if not isinstance(value, str) or not value or not Path(value).is_absolute():
        raise ValueError(f"{description} must be an absolute canonical path")
    path = Path(value)
    if str(path.resolve()) != value:
        raise ValueError(f"{description} must be an absolute canonical path")
    return recorded_path(path)


def _digest(value, description):
    if not isinstance(value, str) or re.fullmatch(r"[0-9a-f]{64}", value) is None:
        raise ValueError(f"{description} must be a SHA-256 digest")
    return value


def _file_map(value, description):
    if not isinstance(value, dict) or not value:
        raise ValueError(f"{description} must be a nonempty file/hash mapping")
    result = {}
    for name, expected in value.items():
        path = _path(name, description)
        _digest(expected, description)
        if not path.is_file() or sha(path) != expected:
            raise ValueError(f"{description} hash mismatch: {path}")
        result[str(path)] = expected
    return result


def _file_ref(value, description):
    if not isinstance(value, dict) or not {"path", "sha256"} <= value.keys():
        raise ValueError(f"Missing {description} path/hash")
    path = _path(value["path"], description)
    _file_map({str(path): value["sha256"]}, description)
    return path


def current_model_identity():
    """CPU model construction only; Model.cuda is never called here."""
    from .model import Model
    model = Model()
    return dict(robot_sha256=model.robot_sha256,
                collision_geometry_sha256=model.collision_geometry_sha256,
                files={str((ROOT / name).resolve()): digest for name, digest in model.sources.items()})


def _dataset_files(manifest, data):
    root = manifest.parent.parent
    result = {str(manifest): sha(manifest)}
    for case in data:
        for path_key, hash_key in (("scene_file", "scene_sha256"),
                                   ("input_arrays_file", "input_arrays_sha256")):
            name = case.get(path_key)
            if not isinstance(name, str) or not name:
                raise ValueError(f"Missing frozen data reference {path_key}")
            path = (root / name).resolve()
            if not path.is_relative_to(root):
                raise ValueError("Frozen data references must remain inside the dataset root")
            expected = _digest(case.get(hash_key), hash_key)
            if str(path) in result and result[str(path)] != expected:
                raise ValueError("Conflicting frozen data hashes")
            result[str(path)] = expected
    return _file_map(result, "frozen data")


def _roadmap_files(directories):
    result = {}
    for directory in directories:
        if not directory.is_dir():
            raise ValueError(f"Missing roadmap directory: {directory}")
        for path in sorted(directory.rglob("*")):
            if path.is_file():
                if path.is_symlink() or not path.resolve().is_relative_to(directory):
                    raise ValueError("Roadmap artifacts must be local regular files")
                result[str(path.resolve())] = sha(path)
    return result


def _validate(protocol_path, block_name, manifest, roadmaps, evaluation, timeout,
              pairs, *, selection="all", problem=None, check_current_model=True,
              source_hashes=None):
    if selection != "all" or problem is not None:
        raise ValueError("Study selection must come only from its frozen block; no problem/first-per-family override")
    protocol_path = recorded_path(protocol_path)
    protocol_hash = sha(protocol_path)
    protocol = read(protocol_path)
    if (not isinstance(protocol, dict) or type(protocol.get("schema_version")) is not int
            or protocol["schema_version"] != 1):
        raise ValueError("Unsupported study protocol schema")
    if protocol.get("state") != "approved":
        raise ValueError("Formal study is locked: protocol state is not approved")
    authorization = protocol.get("authorization")
    if not isinstance(authorization, dict) or any(
            not isinstance(authorization.get(key), str) or not authorization[key].strip()
            for key in ("text", "source")):
        raise ValueError("Formal study is locked: nonempty human authorization text and source are required")
    if not isinstance(block_name, str) or not block_name or block_name not in protocol.get("blocks", {}):
        raise ValueError("Unknown frozen study block")
    frozen_sources = protocol.get("runtime_source_hashes")
    if not isinstance(frozen_sources, dict) or not frozen_sources:
        raise ValueError("Study requires nonempty frozen runtime source hashes")
    for name, digest in frozen_sources.items():
        if (not isinstance(name, str) or not name or Path(name).is_absolute()
                or ".." in Path(name).parts or str(Path(name)) != name):
            raise ValueError("Runtime source paths must be canonical repository-relative paths")
        _digest(digest, "runtime source")
    if frozen_sources != (code_inventory() if source_hashes is None else source_hashes):
        raise ValueError("Runtime source snapshot differs from frozen study protocol")
    contract_path = _file_ref(protocol.get("metrics_contract"), "metrics contract")
    block = protocol["blocks"][block_name]
    if not isinstance(block, dict):
        raise ValueError("Malformed study block")
    manifest = recorded_path(manifest)
    if _file_ref(protocol.get("manifest"), "study manifest") != manifest:
        raise ValueError("Study manifest path differs from requested data")
    data = cases(manifest)
    if not data or len({case["split"] for case in data}) != 1:
        raise ValueError("Study manifest must contain one nonempty split")
    if _file_map(protocol.get("data_files"), "study data") != _dataset_files(manifest, data):
        raise ValueError("Study data file inventory differs from the complete frozen dataset")
    audit_path = _file_ref(protocol.get("input_audit"), "input audit")
    audit = read(audit_path)
    valid_count = sum(case.get("validation_status") == "valid_input" for case in data)
    if (audit.get("passed") is not True or audit.get("manifest_sha256") != sha(manifest)
            or audit.get("problem_count") != len(data) or audit.get("split") != data[0]["split"]
            or audit.get("valid_input_count") != valid_count
            or audit.get("invalid_input_count") != len(data) - valid_count):
        raise ValueError("Input audit must pass and cover the same manifest SHA, split and count")
    audit_cases = audit.get("cases")
    if (not isinstance(audit_cases, list) or len(audit_cases) != len(data)
            or {item.get("problem_id") for item in audit_cases} != {case["problem_id"] for case in data}
            or any(item.get("passed") is not True for item in audit_cases)
            or {item["problem_id"]: item.get("validation_status") for item in audit_cases}
               != {case["problem_id"]: case.get("validation_status") for case in data}):
        raise ValueError("Input audit lacks complete passing case classifications")
    model = protocol.get("model")
    if not isinstance(model, dict):
        raise ValueError("Missing canonical model identity")
    for key in MODEL_IDENTITIES:
        _digest(model.get(key), key)
        if any(case.get(key) != model[key] for case in data):
            raise ValueError(f"Frozen manifest canonical model mismatch: {key}")
    _file_map(model.get("files"), "canonical model")
    if check_current_model and model != current_model_identity():
        raise ValueError("Current canonical model differs from the approved study")
    ids = block.get("problem_ids")
    count = block.get("problem_count")
    if (not isinstance(ids, list) or not ids or any(not isinstance(name, str) for name in ids)
            or len(ids) != len(set(ids)) or isinstance(count, bool) or not isinstance(count, int)
            or count != len(ids)):
        raise ValueError("Study block needs an exact nonempty unique problem ID list and count")
    indexed = {case["problem_id"]: case for case in data}
    if set(ids) - indexed.keys():
        raise ValueError("Study block contains unknown problem IDs")
    if block.get("selection") == "all":
        if set(ids) != indexed.keys():
            raise ValueError("All-problem study block omits frozen problem IDs")
    elif block.get("selection") != "fixed_subset":
        raise ValueError("Study block selection must be all or fixed_subset")
    frozen_dirs = block.get("roadmap_dirs")
    if not isinstance(frozen_dirs, list) or not frozen_dirs:
        raise ValueError("Missing frozen roadmap directories")
    directories = [_path(name, "roadmap directory") for name in frozen_dirs]
    if len(set(directories)) != len(directories) or directories != [recorded_path(p) for p in roadmaps]:
        raise ValueError("Requested roadmaps differ from the frozen ordered block")
    map_files = _file_map(block.get("roadmap_files"), "roadmap")
    if map_files != _roadmap_files(directories):
        raise ValueError("Roadmap file inventory differs from the frozen block")
    for directory in directories:
        metadata = read(directory / "metadata.json")
        if metadata.get("status") != "ready" or any(metadata.get(key) != model[key] for key in MODEL_IDENTITIES):
            raise ValueError("Roadmap is not ready or its canonical model differs")
    declared_pairs = block.get("method_pairs")
    if (not isinstance(declared_pairs, list) or not declared_pairs
            or any(not isinstance(pair, list) or len(pair) != 2 or pair[0] not in METHODS
                   or pair[1] not in OBJECTIVES or (pair[0] == "LGCS" and pair[1] != "minimum_distance")
                   for pair in declared_pairs)
            or len({tuple(pair) for pair in declared_pairs}) != len(declared_pairs)
            or declared_pairs != pairs):
        raise ValueError("Requested method/objective pairs differ from frozen study block")
    if not isinstance(block.get("evaluation_config"), dict) or block["evaluation_config"] != evaluation:
        raise ValueError("Resolved evaluation config differs from frozen study block")
    frozen_timeout = block.get("timeout_seconds")
    if (isinstance(frozen_timeout, bool) or not isinstance(frozen_timeout, (float, int))
            or not math.isfinite(frozen_timeout) or frozen_timeout <= 0 or frozen_timeout != timeout):
        raise ValueError("Wall-clock budget differs from frozen study block")
    if sha(protocol_path) != protocol_hash:
        raise ValueError("Study protocol changed during validation")
    binding = dict(path=str(protocol_path), sha256=protocol_hash, block=block_name,
                   snapshot="study_protocol.json", input_audit_path=str(audit_path),
                   input_audit_sha256=sha(audit_path), input_audit_snapshot="study_input_audit.json",
                   metrics_contract_path=str(contract_path), metrics_contract_sha256=sha(contract_path),
                   metrics_contract_snapshot="study_metrics_contract.md")
    return [indexed[name] for name in ids], binding, block


def authorize(*args, **kwargs):
    """Validate a caller-supplied, already approved protocol; create nothing."""
    try:
        return _validate(*args, **kwargs)
    except (OSError, KeyError, TypeError, AttributeError) as exc:
        raise ValueError(f"Malformed or unavailable study evidence: {exc}") from exc


def copy_evidence(run, binding):
    """Preserve exact approved bytes; never synthesize authorization."""
    run = Path(run)
    for path_key, hash_key, destination_key in (
            ("path", "sha256", "snapshot"),
            ("input_audit_path", "input_audit_sha256", "input_audit_snapshot"),
            ("metrics_contract_path", "metrics_contract_sha256", "metrics_contract_snapshot")):
        source = Path(binding[path_key])
        content = source.read_bytes()
        if hashlib.sha256(content).hexdigest() != binding[hash_key]:
            raise ValueError("Study approval evidence changed before copying")
        (run / binding[destination_key]).write_bytes(content)


def verify_binding(run, manifest):
    """Recheck original and copied authorization plus the exact planned jobs."""
    from .common import resolve_case
    case_cache = {}
    run = Path(run)
    binding = manifest.get("study_protocol")
    has_test = manifest.get("split") == "test" or any(
        resolve_case(job, case_cache).get("split") == "test" for job in manifest["jobs"])
    if binding is None:
        if manifest.get('execution_policy') == 'fixed_public_inputs':
            # The current task explicitly authorizes evaluation of the exposed
            # fixed test set. Keep the old approval mechanism only for reading
            # historical studies; a new run needs no copied approval documents.
            if manifest.get('formal_test_run') is not has_test:
                raise ValueError('Fixed-input split and execution metadata differ')
            return
        if has_test or manifest.get("formal_test_run") is True:
            raise ValueError("Formal test execution is locked without bound study approval")
        return
    if not isinstance(binding, dict):
        raise ValueError("Malformed study protocol binding")
    if (binding.get("snapshot") != "study_protocol.json"
            or binding.get("input_audit_snapshot") != "study_input_audit.json"
            or binding.get("metrics_contract_snapshot") != "study_metrics_contract.md"):
        raise ValueError("Unexpected study evidence snapshot path")
    for original, digest, snapshot in (("path", "sha256", "snapshot"),
            ("input_audit_path", "input_audit_sha256", "input_audit_snapshot"),
            ("metrics_contract_path", "metrics_contract_sha256", "metrics_contract_snapshot")):
        _file_map({binding[original]: binding[digest], str((run / binding[snapshot]).resolve()): binding[digest]},
                  "bound study evidence")
    protocol = read(recorded_path(binding["path"]))
    block = protocol["blocks"][binding["block"]]
    data, expected_binding, block = authorize(
        binding["path"], binding["block"], manifest["manifest_path"], block["roadmap_dirs"],
        block["evaluation_config"], manifest["timeout_seconds"], block["method_pairs"],
        check_current_model=False, source_hashes=manifest["source_hashes"])
    resolved_binding = dict(binding)
    for key in ('path', 'input_audit_path', 'metrics_contract_path'):
        resolved_binding[key] = str(recorded_path(binding[key]))
    if resolved_binding != expected_binding or manifest.get("selection") != "study-block":
        raise ValueError("Run study binding differs from frozen authorization")
    if (manifest.get("manifest_sha256") != protocol["manifest"]["sha256"]
            or manifest.get("split") != data[0]["split"]
            or manifest.get("formal_test_run") is not (data[0]["split"] == "test")
            or manifest["roadmap_files"] != block["roadmap_files"]):
        raise ValueError("Run metadata differs from its frozen study block")
    expected = {}
    for directory in block["roadmap_dirs"]:
        metadata = read(recorded_path(directory) / "metadata.json")
        seed = int(metadata.get("seed", metadata.get("roadmap_seed", 0)))
        for case in data:
            pairings = [p for p in case.get("paired_seeds", []) if p["drm_seed"] == seed]
            if len(pairings) != 1:
                raise ValueError("Study requires exactly one pairing for each declared roadmap seed")
            for method, objective in block["method_pairs"]:
                identity = json.dumps([case["problem_id"], objective, method, directory], separators=(",", ":"))
                job_id = f"{case['problem_id']}-{method}-{objective}-{hashlib.sha256(identity.encode()).hexdigest()[:8]}"
                expected[job_id] = dict(case=case, case_id=case["problem_id"],
                    scene_id=case["scene_id"], family=case["family"], method=method, objective=objective,
                    roadmap_dir=directory, roadmap_id=metadata.get("roadmap_id", Path(directory).name),
                    method_seed=int(pairings[0]["method_seed"]), solver_seed=int(pairings[0]["solver_seed"]),
                    evaluation_config=block["evaluation_config"])
    if len(manifest["jobs"]) != len(expected) or {job["job_id"] for job in manifest["jobs"]} != expected.keys():
        raise ValueError("Prepared jobs omit or add tasks outside the exact frozen study block")
    for job in manifest["jobs"]:
        if any((resolve_case(job, case_cache) if key == "case" else job.get(key)) != value
               for key, value in expected[job["job_id"]].items()):
            raise ValueError("Prepared job differs from its frozen study block")
