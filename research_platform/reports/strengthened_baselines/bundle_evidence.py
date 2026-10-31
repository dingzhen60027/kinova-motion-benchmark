"""Export actual acceptance outputs and verify a portable raw-evidence archive.

This script does not solve, validate trajectories again, or alter sealed runs.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import shutil
import sys
import tarfile


ROOT = Path(__file__).resolve().parents[3]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--label", required=True)
    parser.add_argument("--pytest-artifacts", type=Path, required=True)
    args = parser.parse_args()
    base = Path(__file__).resolve().parent
    destination = base / args.label / "exports"
    destination.mkdir(exist_ok=False)
    rows = []
    checks = []
    sources = []
    for group in ("forest", "fr3", "regression"):
        run = ROOT / "runs" / (args.label + "_" + group)
        sources.append((run, run.relative_to(ROOT)))
        shutil.copytree(run / "reports", destination / group)
        manifest = json.loads((run / "manifest.json").read_text())
        shutil.copy2(run / "environment.json", destination / (group + "_environment.json"))
        summary = json.loads((run / "reports/summary.json").read_text())
        for case in manifest["cases"]:
            jobs = [r for r in summary["jobs"] if r["case_id"] == case["id"]]
            hashes = sorted({r["input_hash"] for r in jobs})
            checks.append(dict(group=group, case=case["id"], methods=len(jobs),
                               shared_input_hashes=hashes, passed=len(jobs) == 8 and len(hashes) == 1))
        mismatches = [name for name, value in manifest["source_hashes"].items()
                      if not (ROOT / name).is_file() or digest(ROOT / name) != value]
        checks.append(dict(group=group, name="frozen_sources_match_workspace",
                           passed=not mismatches, mismatches=mismatches))
        for row in summary["jobs"]:
            rows.append(dict(group=group, **row))
    failures = [r for r in rows if not r["strict_success"]]
    write(destination / "failures.json", failures)
    write(destination / "input_source_checks.json", checks)
    with (destination / "all_jobs.csv").open("w") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(dict(r, history_statuses=json.dumps(r["history_statuses"])) for r in rows)
    counts = []
    for method in sorted({r["method"] for r in rows}):
        dev = [r for r in rows if r["group"] != "regression" and r["method"] == method]
        counts.append(dict(method=method, requested=len(dev),
                           successes=sum(r["strict_success"] for r in dev)))
    write(destination / "development_counts.json", counts)
    # Paired costs have the same environment, units, task and successful inputs.
    paired = []
    for row in rows:
        if row["method"] != "NTO_time_strong" or not row["strict_success"]:
            continue
        for reference in ("SCSTO", "NTO_time"):
            other = next(r for r in rows if (r["group"], r["case_id"], r["method"]) ==
                         (row["group"], row["case_id"], reference))
            if other["strict_success"]:
                paired.append(dict(group=row["group"], case_id=row["case_id"], reference=reference,
                    reference_duration=other["cost"], strong_duration=row["cost"],
                    ratio=row["cost"] / other["cost"]))
    write(destination / "paired_time.json", paired)

    # Copy the real solver tests from the exact pytest invocation, not a rerun.
    selected_tests = ("test_real_richer_time_spline_i0", "test_actual_collision_witness_0",
                      "test_real_exact_forest_segment0", "test_actual_ipopt_rich_time_co0",
                      "test_real_impossible_fixed_two0")
    test_destination = destination / "solver_controls"
    for name in selected_tests:
        shutil.copytree(args.pytest_artifacts / name, test_destination / name)
    write(test_destination / "provenance.json", dict(
        source_directory=str(args.pytest_artifacts.resolve()),
        test_command=["bash", "local/python.sh", "-m", "pytest", "tests/research_platform", "-q"],
        test_log_in_repository=str((base / args.label / "tests.log").relative_to(ROOT)),
        note="Copied existing actual solver outputs; no rerun or mock artifacts."))
    sources.append((test_destination, Path("solver_controls")))
    inventory = {}
    archive = base / args.label / "raw_evidence.tar.gz"
    with tarfile.open(archive, "x:gz") as tar:
        for source, prefix in sources:
            for path in sorted(source.rglob("*")):
                if path.is_file():
                    name = str(prefix / path.relative_to(source))
                    inventory[name] = digest(path)
                    tar.add(path, arcname=name, recursive=False)
    with tarfile.open(archive, "r:gz") as tar:
        actual = {m.name: hashlib.sha256(tar.extractfile(m).read()).hexdigest()
                  for m in tar.getmembers() if m.isfile()}
    assert actual == inventory, "archive member hashes differ"
    write(base / args.label / "archive_inventory.json", inventory)
    write(base / args.label / "archive_receipt.json", dict(
        archive=archive.name, sha256=digest(archive), bytes=archive.stat().st_size,
        members=len(inventory), verified_member_hashes=actual == inventory,
        invocation=[sys.executable, *sys.argv],
        input_source_checks_passed=all(c["passed"] for c in checks),
        method_jobs=len(rows), successes=sum(r["strict_success"] for r in rows), failures=len(failures)))
    print(json.dumps(dict(archive=str(archive), bytes=archive.stat().st_size,
                         files=len(inventory), failures=len(failures), checks=all(c["passed"] for c in checks))))
    return 0 if all(c["passed"] for c in checks) else 1


if __name__ == "__main__":
    raise SystemExit(main())
