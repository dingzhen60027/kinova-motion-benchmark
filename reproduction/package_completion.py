"""Stream source, evidence and replay inputs into a member-verified archive.

Run only after timed experiments have stopped. Native maps and full throughput
batches stay local; BUNDLE_EXCLUDED_FILES.json records their locations/hashes.
"""

import argparse
import gzip
import hashlib
import io
import json
from pathlib import Path
import subprocess
import tarfile
import tempfile

from reproduction.common import ROOT, atomic, read

CHUNK_BYTES = 1024 * 1024
REPORTS = ROOT / "reproduction/reports/paper_completion"


def stream_digest(stream):
    digest = hashlib.sha256()
    size = 0
    while chunk := stream.read(CHUNK_BYTES):
        digest.update(chunk)
        size += len(chunk)
    return {"bytes": size, "sha256": digest.hexdigest()}


def fingerprint(path):
    info = path.stat()
    return info.st_dev, info.st_ino, info.st_size, info.st_mtime_ns


def describe(source):
    if isinstance(source, bytes):
        return {"bytes": len(source), "sha256": hashlib.sha256(source).hexdigest()}
    before = fingerprint(source)
    with source.open("rb") as stream:
        result = stream_digest(stream)
    if fingerprint(source) != before or result["bytes"] != before[2]:
        raise ValueError("File changed while hashing: " + str(source))
    return result


def encoded(value):
    return (json.dumps(value, indent=2, allow_nan=False) + "\n").encode()


def write_verified_archive(output, files, expected):
    """Write and re-read in bounded chunks; do not extract files to disk.

    Keep the prior output until the new archive passes. A failure leaves its
    partial archive and an explicit diagnostic beside the requested output.
    """
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        dir=output.parent, prefix="." + output.name + ".", suffix=".partial", delete=False
    ) as temporary:
        partial = Path(temporary.name)
    try:
        with partial.open("wb") as raw, gzip.GzipFile(
            filename="", fileobj=raw, mode="wb", mtime=0
        ) as compressed, tarfile.open(fileobj=compressed, mode="w|") as archive:
            for name, source in sorted(files.items()):
                entry = tarfile.TarInfo(name)
                entry.size, entry.mode = expected[name]["bytes"], 0o644
                if isinstance(source, bytes):
                    archive.addfile(entry, io.BytesIO(source))
                else:
                    before = fingerprint(source)
                    if before[2] != entry.size:
                        raise ValueError("File size changed before archiving: " + str(source))
                    with source.open("rb") as stream:
                        archive.addfile(entry, stream)
                    if fingerprint(source) != before:
                        raise ValueError("File changed while archiving: " + str(source))
        seen = set()
        with tarfile.open(partial, mode="r|gz") as archive:
            for member in archive:
                if not member.isfile() or member.name not in expected or member.name in seen:
                    raise ValueError("Unexpected or duplicate archive member: " + member.name)
                with archive.extractfile(member) as stream:
                    actual = stream_digest(stream)
                if actual != expected[member.name]:
                    raise ValueError("Bundle verification failed: " + member.name)
                seen.add(member.name)
        if seen != set(expected):
            raise ValueError("Archive is missing members: " + repr(sorted(set(expected) - seen)))
        identity = describe(partial)
        partial.replace(output)
        return identity
    except BaseException as error:
        atomic(
            partial.with_suffix(".failure.json"),
            dict(output=str(output), partial=str(partial), exception=type(error).__name__, message=str(error)),
        )
        raise


def absolute_run(value):
    path = Path(value)
    return (path if path.is_absolute() else ROOT / path).resolve()


def throughput_runs():
    """Index selected results and retain any local v4 throughput attempts too."""
    found = {}
    index_path = REPORTS / "throughput_index.json"
    if index_path.exists():
        for family, entry in read(index_path).items():
            if not isinstance(entry, dict) or not entry.get("run_dir"):
                raise ValueError("throughput_index entries require run_dir: " + family)
            path = absolute_run(entry["run_dir"])
            if not path.is_dir():
                raise FileNotFoundError(path)
            found[path] = {"family": family, "discovery": "throughput_index.json"}
    for family, pattern in [
        ("forest", "*_paper_completion_forest_throughput"),
        ("fr3", "*_paper_completion_fr3_throughput"),
        ("public_iiwa", "*_public_iiwa_throughput"),
    ]:
        for path in sorted((ROOT / "reproduction/runs").glob(pattern)):
            if path.is_dir():
                found.setdefault(path.resolve(), {"family": family, "discovery": "local_run_directory"})
    return found


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "reproduction/artifacts/csdecomp_paper_completion_20261025.tar.gz",
    )
    args = parser.parse_args()
    output = args.output.resolve()
    index = read(REPORTS / "run_index.json")
    main_runs = {family: absolute_run(index[family]["run_dir"]) for family in ("forest", "fr3")}
    throughputs = throughput_runs()
    run_records = {}
    for family, run in main_runs.items():
        manifest = read(run / "manifest.json")
        if manifest.get("status") in {"running", "preparing", "pending"}:
            raise ValueError("Stop timed experiments before packaging: " + str(run))
        # Forest records its circles in each case input; FR3 has shared scenes.
        required_directories = ["cases", "source_snapshot", "roadmaps"]
        if family == "fr3":
            required_directories.append("scenes")
        for required in required_directories:
            if not (run / required).is_dir():
                raise FileNotFoundError(run / required)
        run_records[family] = dict(run_dir=str(run), status=manifest.get("status"),
                                   implementation_hash=manifest.get("implementation_hash"))
    for run, record in throughputs.items():
        manifest = read(run / "manifest.json") if (run / "manifest.json").exists() else {}
        record["status"] = manifest.get("status", "manifest_unavailable")
        if record["status"] in {"running", "preparing", "pending"}:
            raise ValueError("Stop throughput measurements before packaging: " + str(run))

    files = {}  # Paths or small generated metadata only, never file-content blobs.
    exclusions = {}
    skip = {output, output.with_suffix(".manifest.json"), output.with_suffix(".excluded.json")}

    def add_file(name, path, scope):
        path = path.resolve()
        if path in skip:
            return
        if not path.is_file():
            raise FileNotFoundError(path)
        throughput_root = next((run for run in throughputs if path.is_relative_to(run)), None)
        reason = None
        if path.suffix.lower() == ".map":
            reason = "native_serialized_roadmap_retained_locally"
        elif throughput_root is not None and (
            path.name in {"configurations.npy", "flags.npz"} or path.name.endswith("_flags.npz")
        ):
            reason = "full_two_million_configuration_batch_or_flags_retained_locally"
        if reason:
            record = exclusions.setdefault(str(path), dict(
                local_path=str(path),
                repository_relative_path=str(path.relative_to(ROOT)) if path.is_relative_to(ROOT) else None,
                reason=reason, archive_names=[], scopes=[],
                run_dir=str(throughput_root) if throughput_root else next(
                    (str(run) for run in main_runs.values() if path.is_relative_to(run)), None),
            ))
            if name not in record["archive_names"]:
                record["archive_names"].append(name)
            if scope not in record["scopes"]:
                record["scopes"].append(scope)
            return
        if name in files and files[name] != path:
            raise ValueError("Conflicting archive member: " + name)
        files[name] = path

    compact = REPORTS / "execution_evidence.tar.gz"
    evidence = set((ROOT / "reproduction/logs").glob("*completion*.log"))
    evidence.update(REPORTS.glob("*.log"))
    evidence.update((REPORTS / "source_evidence").rglob("*.log"))
    evidence.update(path for path in (REPORTS / "source_evidence").rglob("*.json")
                    if "connector_responses" not in path.name)
    evidence_files = {str(path.relative_to(ROOT)): path for path in sorted(evidence) if path.is_file()}
    evidence_inventory = {name: describe(path) for name, path in evidence_files.items()}
    evidence_identity = write_verified_archive(compact, evidence_files, evidence_inventory)
    add_file(str(compact.relative_to(ROOT)), compact, "compact_execution_evidence")

    tracked = subprocess.check_output(["git", "ls-files", "-z"], cwd=ROOT).decode().split("\0")
    for name in tracked:
        if name and (name.startswith("reproduction/") or name == "README.md"):
            add_file(name, ROOT / name, "tracked_source_and_history")
    for folder in ("reproduction/vendor/scsplanning", "reproduction/vendor/franka_description", "reproduction/logs"):
        for path in sorted((ROOT / folder).rglob("*")):
            if path.is_file() and not any(part in {".git", "__pycache__", ".venv"} for part in path.parts):
                if (folder.endswith("logs") or path.suffix in {
                    ".py", ".toml", ".txt", ".md", ".yaml", ".xacro", ".stl", ".json"
                } or path.name in {"LICENSE", "NOTICE"}):
                    add_file(str(path.relative_to(ROOT)), path, "third_party_source_or_logs")
    add_file("local/python.sh", ROOT / "local/python.sh", "environment_launcher")
    # Final analysis can be newer than git staging. Keep those reports, figures
    # and diagnostics too. Large raw third-party downloads/connector responses
    # retain the existing policy: tracked files plus compact execution evidence.
    for folder in (REPORTS, ROOT / "reproduction/figures/paper_completion"):
        for path in sorted(folder.rglob("*")):
            if path.is_file() and not path.is_relative_to(REPORTS / "source_evidence"):
                add_file(str(path.relative_to(ROOT)), path, "current_reports_and_figures")
    for family, run in main_runs.items():
        for path in sorted(run.rglob("*")):
            if path.is_file():
                add_file("measured_inputs/" + family + "/" + str(path.relative_to(run)), path, "main_" + family)
    for run, record in sorted(throughputs.items()):
        for path in sorted(run.rglob("*")):
            if path.is_file():
                add_file("throughput_inputs/" + run.name + "/" + str(path.relative_to(run)), path, "throughput_" + record["family"])
    # Select only our measured diagnostics, never the downloaded paper source.
    for label, filename, field in [
        ("paperdate_core", "paperdate_core_result.json", "raw_output_directory"),
        ("lgcs_enumeration", "lgcs_enumeration_results.json", "run_dir"),
    ]:
        result_path = REPORTS / filename
        if result_path.exists():
            value = read(result_path).get(field)
            if value:
                directory = Path(value)
                if not directory.is_absolute():
                    directory = ROOT / directory
                for path in sorted(directory.rglob("*")):
                    if path.is_file():
                        add_file("supplemental_diagnostics/" + label + "/" + str(path.relative_to(directory)),
                                 path, "supplemental_" + label)
    diagnostics = ROOT / "reproduction/diagnostics/paper_completion"
    if diagnostics.is_dir():
        for path in sorted(diagnostics.iterdir()):
            if path.is_file() and path.suffix in {".log", ".json"}:
                add_file("preserved_diagnostics/" + path.name, path, "optional_diagnostics")

    historical = read(REPORTS / "paperdate_source_inventory.json")
    with tempfile.TemporaryDirectory(prefix="csdecomp-paperdate-source-") as temporary:
        for number, entry in enumerate(historical["files"]):
            # Stream immutable Git objects to temporary files. Never substitute a
            # patched paper-date build directory for the inventoried original.
            path = Path(temporary) / str(number)
            with path.open("wb") as stream:
                subprocess.run(["git", "show", historical["revision"] + ":" + entry["path"]],
                               cwd=ROOT, stdout=stream, check=True)
            identity = describe(path)
            if identity["sha256"] != entry["sha256"] or identity["bytes"] != entry["bytes"]:
                raise ValueError("Historical source mismatch: " + entry["path"])
            files["paperdate_source/" + entry["path"]] = path

        excluded_records = []
        for path, record in sorted(exclusions.items()):
            excluded_records.append({**record, **describe(Path(path))})
        excluded_manifest = dict(
            scope="Excluded files stay at the recorded local paths; no file is removed or altered.",
            hash_algorithm="sha256", files=excluded_records,
        )
        metadata = dict(
            packaged_at_commit=subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
            upstream_base="7661a97bb7c4931be73d2bc4f8c45c3094ff476e",
            measured_implementation_hash=run_records["forest"]["implementation_hash"],
            measured_runs=run_records,
            throughput_runs=[dict(run_dir=str(run), **record) for run, record in sorted(throughputs.items())],
            scope=("Apply over upstream checkout. Contains source, locks, reports, figures, logs, third-party "
                   "source/licenses and all files from both indexed main runs except native .map files: frozen "
                   "source snapshots, scenes, roadmap samples, seeds, online IK, candidates, pre-solve regions, "
                   "trajectory coefficients, replay arrays, validation and failures. Throughput records and smaller "
                   "independent verification arrays are included. Native .map files and full two-million-point "
                   "throughput inputs/flags remain local and are individually inventoried in BUNDLE_EXCLUDED_FILES.json. "
                   "Historical reports are retained, not pooled."),
            original_paper_reproduction_complete=False,
            paperdate_source_revision=historical["revision"],
            paperdate_source_files=len(historical["files"]),
            excluded_files=len(excluded_records),
            excluded_bytes=sum(record["bytes"] for record in excluded_records),
            execution_evidence={**evidence_identity, "members": len(evidence_files), "all_contents_verified": True},
        )
        files["BUNDLE_METADATA.json"] = encoded(metadata)
        files["BUNDLE_EXCLUDED_FILES.json"] = encoded(excluded_manifest)
        inventory = {name: describe(source) for name, source in sorted(files.items())}
        files["BUNDLE_FILES.json"] = encoded(inventory)
        # The self-excluding inventory cannot recursively contain its own hash;
        # still verify that member with an independently computed expected hash.
        expected = {**inventory, "BUNDLE_FILES.json": describe(files["BUNDLE_FILES.json"])}
        identity = write_verified_archive(output, files, expected)
        result = dict(path=str(output), **identity, files=len(files), all_contents_verified=True,
                      bundle_inventory=expected["BUNDLE_FILES.json"], **metadata)
        atomic(output.with_suffix(".excluded.json"), excluded_manifest)
        atomic(output.with_suffix(".manifest.json"), result)
        print(json.dumps(result, indent=2), flush=True)


if __name__ == "__main__":
    main()
