"""Package v5 source and raw evidence after all computational work has stopped.

The archive keeps repository-relative paths, including untracked v5 scripts.
Native maps and full throughput inputs/batch flags remain local with individual
SHA256 records. Independent samples, mixed verification flags, trajectories,
failed construction attempts and failed run records stay in the archive.

Run directly, outside record_constructed: the packager writes its own execution
status. This avoids archiving a command ledger or log that changes as packaging
finishes. No prior v4 archive, report or result is rewritten.
"""

import argparse
import ast
import datetime
import json
import os
from pathlib import Path
import subprocess
import traceback

from reproduction.common import ROOT, atomic, read
from reproduction.package_completion import describe, encoded, write_verified_archive

REPORTS = ROOT / "reproduction/reports/paper_constructed"
RUNS = ROOT / "reproduction/runs"
ACTIVE = {"running", "preparing", "pending", "in_progress", "queued"}
IGNORED_PARTS = {".git", "__pycache__", ".pytest_cache", ".venv", ".conda"}
RUN_REFERENCE_KEYS = {"run_dir", "base_run", "raw_output_directory", "diagnostic_run"}
COMPUTE_MODULES = {
    "reproduction.run_constructed", "reproduction.paper_constructed.run",
    "reproduction.paper_constructed.throughput", "reproduction.throughput_public_iiwa",
    "reproduction.enumerate_constructed_lgcs", "reproduction.validate_constructed_geometry",
    "reproduction.audit_paper_constructed", "reproduction.paper_constructed.build_assets",
    "reproduction.hardware_constructed.run",
    "reproduction.hardware_constructed.construct_anchors",
    "reproduction.diagnose_constructed_mesh_candidates", "reproduction.replay_constructed_geometry",
}
OLD_REQUIRED_FILES = (
    "reproduction/paper_completion/assets/fr3_33.json",
    "reproduction/paper_completion/assets/fr3_limits_20250411.yaml",
    "reproduction/paper_completion/config.json",
    "reproduction/reports/paper_completion/fr3_geometry_era_comparison.json",
    "reproduction/reports/paper_completion/diagnostics/pipeline/mesh_comparison/results.json",
    "reproduction/reports/paper_completion/optimization_audit.md",
    "reproduction/reports/paper_completion/source_audit.md",
    "reproduction/reports/paper_completion/parameter_trace.csv",
    "reproduction/reports/paper_completion/paperdate_core_result.json",
)
SHARED_MODULES = (
    "__init__.py", "common.py", "forest.py", "optimization.py", "pipeline.py",
    "run.py", "analyze.py", "package_completion.py", "throughput_records.py",
    "throughput_public_iiwa.py", "probe_public_iiwa.py",
)


def repository_path(value):
    path = Path(value)
    path = (path if path.is_absolute() else ROOT / path).resolve()
    if not path.is_relative_to(ROOT):
        raise ValueError("Refusing evidence outside this repository: " + str(path))
    return path


def walk_records(value):
    """Visit nested index/ledger records without interpreting prose as paths."""
    if isinstance(value, dict):
        yield value
        for child in value.values():
            yield from walk_records(child)
    elif isinstance(value, list):
        for child in value:
            yield from walk_records(child)


def index_documents():
    # Top-level indexes, stable supplemental results and command ledgers are
    # small. Deep geometric candidate JSON files do not carry run pointers.
    paths = set(REPORTS.glob("*.json"))
    paths.update((REPORTS / "hardware_simulation").glob("*.json"))
    return {path: read(path) for path in sorted(paths)}


def discover_runs(documents):
    index_path = REPORTS / "run_index.json"
    if index_path not in documents:
        raise FileNotFoundError("Final main-run index is required: " + str(index_path))
    index = documents[index_path]
    runs = {}

    def add(value, scope, source):
        path = repository_path(value)
        if not path.is_dir():
            raise FileNotFoundError(path)
        if not path.is_relative_to(RUNS):
            # Supplemental report-contained artifacts are already included in
            # full. Other repository-contained raw directories are accepted.
            if path == ROOT or path == ROOT / "reproduction":
                raise ValueError("Run reference is too broad: " + str(path))
        record = runs.setdefault(path, {"scopes": [], "discovery": []})
        if scope not in record["scopes"]:
            record["scopes"].append(scope)
        if source not in record["discovery"]:
            record["discovery"].append(source)

    for family in ("forest", "fr3"):
        entry = index.get(family)
        if not isinstance(entry, dict) or not entry.get("run_dir"):
            raise ValueError("run_index requires forest/fr3 run_dir entries")
        add(entry["run_dir"], "main_" + family, str(index_path.relative_to(ROOT)))
    for path, document in documents.items():
        for record in walk_records(document):
            for key in RUN_REFERENCE_KEYS:
                value = record.get(key)
                if isinstance(value, str) and value:
                    add(value, "indexed_or_ledger_raw_artifacts", str(path.relative_to(ROOT)))
    # Capture unsuccessful/unindexed attempts as well as the selected runs.
    # Public iiwa is included only when referenced by the v5 indexes/ledgers;
    # historical v4 throughput folders are deliberately not swept in.
    for pattern in ("*_paper_constructed_*", "*_constructed_LGCS_enumeration", "*_constructed_hardware_simulation"):
        for path in sorted(RUNS.glob(pattern)):
            if path.is_dir():
                add(str(path), "local_v5_attempt", "local_directory_pattern:" + pattern)
    return runs


def assert_stopped(documents, runs):
    """Reject active measurements before reading bulk arrays or hashing maps."""
    for path, document in documents.items():
        if "commands" in path.name:
            for record in walk_records(document):
                if record.get("status") in ACTIVE:
                    raise ValueError("Command ledger still active; run packaging directly after it stops: " + str(path))
    for run in runs:
        manifest_path = run / "manifest.json"
        if not manifest_path.exists():
            # Report-contained supplemental raw directories may use a summary.
            if run.is_relative_to(RUNS):
                raise FileNotFoundError("Raw run manifest is required: " + str(manifest_path))
            continue
        manifest = read(manifest_path)
        if manifest.get("status") in ACTIVE:
            raise ValueError("Stop experiments before packaging: " + str(run))
    for path in REPORTS.rglob("manifest.json"):
        if "source_snapshot" not in path.parts and read(path).get("status") in ACTIVE:
            raise ValueError("Report-contained attempt is still active: " + str(path))
    # Independent of ledger correctness, refuse live computational processes in
    # this repository. No process is killed or modified. Ignore this packager.
    proc = Path("/proc")
    if proc.is_dir():
        for process in proc.iterdir():
            if not process.name.isdigit() or int(process.name) == os.getpid():
                continue
            try:
                if (process / "cwd").resolve() != ROOT:
                    continue
                argv = (process / "cmdline").read_bytes().decode(errors="replace").split("\0")
            except (FileNotFoundError, ProcessLookupError, PermissionError):
                continue
            if any(module in argv for module in COMPUTE_MODULES):
                raise ValueError("Computational process still active (pid " + process.name + ")")


def exclusion_reason(path, throughput_roots):
    if path.suffix.lower() == ".map":
        return "native_serialized_roadmap_retained_locally"
    if any(path.is_relative_to(run) for run in throughput_roots):
        if path.name == "configurations.npy":
            return "full_two_million_configuration_input_retained_locally"
        if path.name == "warmup_flags.npz" or (
            path.name.startswith(("batch_", "repeat_")) and path.name.endswith("_flags.npz")
        ):
            return "full_two_million_batch_flags_retained_locally"
    # flags.npz combines complete CUDA flags with small independent-check
    # indices/flags in v5. Keep it intact rather than lose the verification data.
    return None



def compact_execution_evidence():
    """Keep model/geometry raw evidence in a small verified Git-ready archive."""
    files = {}
    selected = set()
    for directory in sorted(REPORTS.glob("model_construction*")):
        if directory.is_dir():
            selected.update(path for path in directory.rglob("*") if path.is_file())
        elif directory.is_file():
            selected.add(directory)
    geometry = REPORTS / "geometry_audit"
    if geometry.is_dir():
        selected.update(path for path in geometry.rglob("*") if path.is_file())
    selected.update(REPORTS.glob("*.log"))
    selected.update(REPORTS.glob("*commands.json"))
    selected.update((ROOT / "reproduction/logs").glob("*constructed*.log"))
    for document in index_documents().values():
        for record in walk_records(document):
            if isinstance(record.get("log"), str):
                log = repository_path(record["log"])
                if log.is_file() and log.suffix == ".log":
                    selected.add(log)
    selected.update(path for path in (REPORTS / "geometry_index.json", REPORTS / "geometry_audit.md") if path.is_file())
    for path in sorted(selected):
        if any(part in IGNORED_PARTS for part in path.parts):
            continue
        if "packag" in path.name.lower() and path.suffix == ".log":
            continue  # The active packaging process cannot archive its own log.
        source = repository_path(path)
        if path.is_symlink():
            raise ValueError("Unexpected evidence symlink: " + str(path))
        files[str(path.relative_to(ROOT))] = source
    inventory = {name: describe(path) for name, path in sorted(files.items())}
    files["EVIDENCE_FILES.json"] = encoded(inventory)
    expected = {**inventory, "EVIDENCE_FILES.json": describe(files["EVIDENCE_FILES.json"])}
    output = REPORTS / "execution_evidence.tar.gz"
    identity = write_verified_archive(output, files, expected)
    result = dict(path=str(output.relative_to(ROOT)), **identity, members=len(files),
        all_contents_verified=True, inventory_sha256=expected["EVIDENCE_FILES.json"]["sha256"],
        scope="All model-construction attempts including failures, original/subdivided geometry and candidate labels; independent geometry audit samples, pair matrices, metadata, rows and failures; command ledgers and relevant logs. Main-run trajectory raw files are in the larger portable archive.")
    atomic(REPORTS / "execution_evidence_manifest.json", result)
    return result


def collect_files(runs, output):
    files, scopes, exclusions = {}, {}, {}
    throughput_roots = {run for run in runs if "throughput" in run.name}
    skip = {output, *(output.with_suffix(suffix) for suffix in
                     (".manifest.json", ".excluded.json", ".status.json"))}

    def add(path, scope, required=True):
        path = Path(path)
        if not path.exists():
            if required:
                raise FileNotFoundError(path)
            return
        if any(part in IGNORED_PARTS for part in path.relative_to(ROOT).parts):
            return
        if path.is_symlink() and path.resolve() != path:
            raise ValueError("Explicitly resolve source symlinks before packaging: " + str(path))
        resolved = repository_path(path)
        if path.parent == REPORTS and path.name in {
            "portable_bundle_manifest.json", "portable_bundle_exclusions.json",
            "portable_bundle_receipt.json",
        }:
            return  # External receipts cannot embed the archive's own final hash.
        if resolved in skip or not resolved.is_file():
            return
        if path.parent == REPORTS and path.name.startswith(".execution_evidence.tar.gz."):
            return  # Failed archive intermediates remain local, with diagnostics.
        if "packag" in path.name.lower() and path.suffix == ".log":
            return  # Packaging records its own terminal outcome beside output.
        name = str(path.relative_to(ROOT))
        reason = exclusion_reason(resolved, throughput_roots)
        if reason:
            item = exclusions.setdefault(name, dict(local_path=str(resolved),
                repository_relative_path=name, reason=reason, scopes=[]))
            if scope not in item["scopes"]:
                item["scopes"].append(scope)
        else:
            if name in files and files[name] != resolved:
                raise ValueError("Conflicting archive member: " + name)
            files[name] = resolved
            scopes.setdefault(name, set()).add(scope)

    def folder(path, scope, required=True):
        if not path.is_dir():
            if required:
                raise FileNotFoundError(path)
            return
        for item in sorted(path.rglob("*")):
            if item.is_file() or item.is_symlink():
                add(item, scope)

    # These filesystem scans intentionally include untracked/uncommitted work.
    folder(ROOT / "reproduction/paper_constructed", "v5_package")
    folder(ROOT / "reproduction/hardware_constructed", "constructed_hardware_simulation_source", required=False)
    folder(REPORTS, "v5_reports_and_all_model_attempts")
    folder(ROOT / "reproduction/figures/paper_constructed", "v5_figures", required=False)
    folder(ROOT / "reproduction/diagnostics/paper_constructed", "v5_diagnostics", required=False)
    for path in sorted((ROOT / "reproduction").glob("*constructed*.py")):
        add(path, "v5_driver_including_untracked")
    for path in sorted((ROOT / "reproduction/logs").glob("*constructed*.log")):
        add(path, "v5_command_logs")
    # Ledger log filenames can be custom; do not rely solely on the glob.
    for document in index_documents().values():
        for record in walk_records(document):
            if isinstance(record.get("log"), str):
                add(repository_path(record["log"]), "ledger_log")
    for name in SHARED_MODULES:
        add(ROOT / "reproduction" / name, "shared_python_dependency", required=name != "__init__.py")
    folder(ROOT / "reproduction/configs", "shared_configs")
    folder(ROOT / "reproduction/tests", "verification_tests")
    folder(ROOT / "reproduction/paper_protocol", "public_iiwa_python_dependency")
    folder(ROOT / "reproduction/paper_completion", "frozen_source_dependencies_not_old_raw_results")
    for name in OLD_REQUIRED_FILES:
        add(ROOT / name, "explicit_old_source_or_evidence_dependency")
    # Small manuscript source supports equation provenance without copying the
    # historical v4 raw results or its already packaged archive.
    source_text = ROOT / "reproduction/reports/paper_completion/source_evidence/arxiv_text"
    if source_text.is_dir():
        for path in sorted(source_text.rglob("*")):
            if path.is_file() and path.suffix in {".tex", ".bib", ".sty", ".cls"}:
                add(path, "paper_equation_source")
    for name in ("iiwa7_box_collision.urdf", "wsg50_110_welded_fingers_box_collision.urdf"):
        add(ROOT / "reproduction/reports/paper_completion/source_evidence/hardware_assets" / name,
            "public_iiwa_pinned_asset", required=False)
    for name in ("scsplanning", "franka_description"):
        folder(ROOT / "reproduction/vendor" / name, "third_party_source_and_license")
        add(ROOT / "reproduction/vendor" / name / "LICENSE", "required_third_party_license")
    folder(ROOT / "local", "conda_environment_and_launcher")
    for name in ("README.md", "conda-explicit.txt", "requirements.lock.txt", "environment.json"):
        add(ROOT / "reproduction" / name, "reproduction_environment_and_guide")
    add(ROOT / "LICENSE", "repository_license")
    add(ROOT / "README.md", "repository_readme")
    # Native core sources/build rules are included at the current checkout.
    # Large upstream example outputs/data and prior experiment bundles are not.
    tracked = subprocess.check_output(["git", "ls-files", "-z"], cwd=ROOT).decode().split("\0")
    for name in tracked:
        path = ROOT / name
        if name and (name.startswith("csdecomp/src/") or (
            "/" not in name and (path.suffix in {".bzl", ".bazel", ".toml"}
                or name.startswith(("BUILD", "WORKSPACE", "MODULE.bazel", ".bazel", "LICENSE", "NOTICE"))))):
            add(path, "native_core_source_and_build_rules")
    # Resolve local Python imports without importing modules or executing model
    # code. This also catches dependencies of newly written report/test scripts.
    inspected = set()
    while True:
        pending = [path for path in files.values()
                   if path.suffix == ".py" and path.is_relative_to(ROOT / "reproduction")
                   and path not in inspected]
        if not pending:
            break
        for path in pending:
            inspected.add(path)
            package_parts = list(path.relative_to(ROOT).parent.parts)
            for node in ast.walk(ast.parse(path.read_text(), filename=str(path))):
                names = []
                if isinstance(node, ast.Import):
                    names = [alias.name for alias in node.names]
                elif isinstance(node, ast.ImportFrom):
                    prefix = package_parts[:len(package_parts) - node.level + 1] if node.level else []
                    base = ".".join(prefix + ([node.module] if node.module else []))
                    names = [base] + [base + "." + alias.name for alias in node.names]
                for module in names:
                    if not module.startswith("reproduction."):
                        continue
                    target = ROOT.joinpath(*module.split("."))
                    for dependency in (target.with_suffix(".py"), target / "__init__.py"):
                        if dependency.is_file():
                            add(dependency, "static_python_import_dependency")
    for run, record in sorted(runs.items()):
        folder(run, ";".join(record["scopes"]))
    return files, scopes, exclusions


def package(output):
    documents = index_documents()
    runs = discover_runs(documents)
    assert_stopped(documents, runs)
    # First bulk-read phase, after both ledger and process-state checks.
    execution_evidence = compact_execution_evidence()
    files, scopes, exclusions = collect_files(runs, output)
    excluded = [{**record, **describe(Path(record["local_path"]))}
                for _, record in sorted(exclusions.items())]
    excluded_manifest = dict(hash_algorithm="sha256", files=excluded,
        scope="Excluded files remain unchanged at their local paths; each is individually hashed. Small trajectories, validation inputs and mixed verification flags are retained.")
    run_records = []
    for run, discovery in sorted(runs.items()):
        manifest = read(run / "manifest.json") if (run / "manifest.json").exists() else {}
        run_records.append(dict(run_dir=str(run), archive_prefix=str(run.relative_to(ROOT)),
            status=manifest.get("status", "report_contained_supplement"),
            implementation_hash=manifest.get("implementation_hash"), **discovery))
    metadata = dict(protocol="paper_constructed_v5", packaged_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        packaged_at_commit=subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        working_tree_status=subprocess.check_output(["git", "status", "--porcelain"], cwd=ROOT, text=True).splitlines(),
        includes_untracked_v5_source=True, exact_original_paper_reproduction=False,
        execution_evidence=execution_evidence,
        runs=run_records, excluded_files=len(excluded), excluded_bytes=sum(x["bytes"]for x in excluded),
        scope="Repository-relative v5 source, reports, figures, logs, failed model attempts, indexed main runs, pilot and supplemental raw evidence; explicit old source dependencies and licenses. Historical v4 raw bundles are not duplicated.",
        replay_note="Extract over this repository checkout and restore the recorded Conda environment. Native serialized maps can be rebuilt from retained source, seeds, nodes and configuration; their omitted bytes are identified, not claimed embedded.")
    files["BUNDLE_METADATA.json"] = encoded(metadata)
    files["BUNDLE_EXCLUDED_FILES.json"] = encoded(excluded_manifest)
    files["BUNDLE_SOURCE_SCOPES.json"] = encoded({name: sorted(value) for name, value in sorted(scopes.items())})
    inventory = {name: describe(source) for name, source in sorted(files.items())}
    files["BUNDLE_FILES.json"] = encoded(inventory)
    expected = {**inventory, "BUNDLE_FILES.json": describe(files["BUNDLE_FILES.json"])}
    assert_stopped(index_documents(), runs)
    identity = write_verified_archive(output, files, expected)
    result = dict(path=str(output), **identity, files=len(files), all_contents_verified=True,
                  bundle_inventory=expected["BUNDLE_FILES.json"], **metadata)
    atomic(output.with_suffix(".excluded.json"), excluded_manifest)
    atomic(output.with_suffix(".manifest.json"), result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path,
        default=ROOT / "reproduction/artifacts/csdecomp_paper_constructed_20261026.tar.gz")
    args = parser.parse_args()
    output = args.output.resolve()
    # Keep writes in this project's artifact directory and never replace a v4
    # archive even if an incorrect --output is supplied.
    if not output.is_relative_to(ROOT / "reproduction/artifacts"):
        raise ValueError("Place v5 output under reproduction/artifacts")
    if output.exists():
        previous = output.with_suffix(".manifest.json")
        if not previous.exists() or read(previous).get("protocol") != "paper_constructed_v5":
            raise ValueError("Refusing to overwrite an artifact without a v5 manifest")
    status_path = output.with_suffix(".status.json")
    status = dict(status="running", output=str(output),
        command=["python", "-m", "reproduction.package_constructed", "--output", str(output)],
        started_utc=datetime.datetime.now(datetime.timezone.utc).isoformat())
    atomic(status_path, status)
    try:
        result = package(output)
        status.update(status="completed", completed_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                      archive_sha256=result["sha256"], all_contents_verified=True)
        atomic(status_path, status)
        print(json.dumps(result, indent=2), flush=True)
    except BaseException as error:
        status.update(status="failed", exception=type(error).__name__, message=str(error), traceback=traceback.format_exc())
        atomic(status_path, status)
        raise


if __name__ == "__main__":
    main()
