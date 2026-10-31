"""Recover exact public paper-date sources and the pinned Bazel executable.

This preparation neither compiles native code nor changes the Conda environment.
Existing source files are verified and never overwritten to hide a mismatch.
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import io
import json
from pathlib import Path
import subprocess
import tarfile
import traceback
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
VENDOR = ROOT / "reproduction/vendor"
SOURCE = VENDOR / "csdecomp_paperdate"
REVISION = "ee94f8145bb32ef6c08b1e575218335b9daf533f"
BAZEL = VENDOR / "paperdate_build_tools/bazel-7.2.1-linux-x86_64"
BAZEL_URL = "https://github.com/bazelbuild/bazel/releases/download/7.2.1/bazel-7.2.1-linux-x86_64"
BAZEL_SHA256 = "80ccd1ecb4b88750fbe5d7622d67072fddcba9da7808f13356555e480bf67875"
REPORT = ROOT / "reproduction/reports/paper_completion/paperdate_preparation.json"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-only", action="store_true", help="Verify prepared files without any network or extraction")
    args = parser.parse_args()
    result = {
        "started_at_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "source_revision": REVISION, "verify_only": args.verify_only,
        "compilation_attempted": False, "conda_mutated": False, "commands": [],
    }

    def command(arguments):
        completed = subprocess.run(arguments, cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        result["commands"].append({"argv": arguments, "returncode": completed.returncode,
                                   "stderr": completed.stderr.decode(errors="replace")})
        return completed

    try:
        exists = command(["git", "cat-file", "-e", REVISION + "^{commit}"])
        if exists.returncode:
            if args.verify_only:
                raise RuntimeError("Pinned commit is not available locally")
            fetched = command(["git", "fetch", "--no-tags", "https://github.com/wernerpe/csdecomp.git", REVISION])
            if fetched.returncode:
                raise RuntimeError("Fetching pinned upstream commit failed; stderr retained")
        listing = command(["git", "ls-tree", "-r", REVISION])
        if listing.returncode:
            raise RuntimeError("Cannot read pinned commit tree")
        if not SOURCE.exists():
            if args.verify_only:
                raise RuntimeError("Pinned source directory does not exist")
            archive = command(["git", "archive", REVISION])
            if archive.returncode:
                raise RuntimeError("Creating exact source archive failed")
            SOURCE.mkdir(parents=True)
            with tarfile.open(fileobj=io.BytesIO(archive.stdout)) as contents:
                for member in contents.getmembers():
                    target = (SOURCE / member.name).resolve()
                    if not target.is_relative_to(SOURCE.resolve()) or not (member.isfile() or member.isdir()):
                        raise RuntimeError("Unexpected path/type in pinned Git archive: " + member.name)
                contents.extractall(SOURCE)
        files = []
        for line in listing.stdout.decode().splitlines():
            metadata, name = line.split("\t", 1)
            expected = metadata.split()[2]
            data = (SOURCE / name).read_bytes()
            actual = hashlib.sha1(b"blob " + str(len(data)).encode() + b"\0" + data).hexdigest()
            files.append({"path": name, "expected_git_blob": expected, "actual_git_blob": actual})
        result["source"] = {"files": len(files), "all_match": all(x["expected_git_blob"] == x["actual_git_blob"] for x in files), "mismatches": [x for x in files if x["expected_git_blob"] != x["actual_git_blob"]]}
        if not result["source"]["all_match"]:
            raise RuntimeError("Source mismatch; original files were not overwritten")
        if not BAZEL.exists():
            if args.verify_only:
                raise RuntimeError("Pinned Bazel executable is absent")
            BAZEL.parent.mkdir(parents=True, exist_ok=True)
            temporary = BAZEL.with_suffix(".download")
            result["download"] = {"url": BAZEL_URL, "temporary_path": str(temporary)}
            with urllib.request.urlopen(BAZEL_URL, timeout=120) as source, temporary.open("wb") as destination:
                while data := source.read(1024 * 1024):
                    destination.write(data)
            actual = hashlib.sha256(temporary.read_bytes()).hexdigest()
            result["download"]["sha256"] = actual
            if actual != BAZEL_SHA256:
                raise RuntimeError("Bazel SHA256 mismatch; download retained for inspection")
            temporary.replace(BAZEL)
            BAZEL.chmod(0o755)
        actual = hashlib.sha256(BAZEL.read_bytes()).hexdigest()
        result["bazel"] = {"path": str(BAZEL), "url": BAZEL_URL, "sha256": actual, "expected_sha256": BAZEL_SHA256, "matches": actual == BAZEL_SHA256}
        if actual != BAZEL_SHA256:
            raise RuntimeError("Existing Bazel SHA256 mismatch; file was not overwritten")
        (VENDOR / "paperdate_distdir").mkdir(parents=True, exist_ok=True)
        result["status"] = "passed"
    except Exception:
        result.update(status="failed", error=traceback.format_exc())
    REPORT.parent.mkdir(parents=True, exist_ok=True)
    if REPORT.exists():
        stamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
        previous = REPORT.parent / "source_evidence" / ("preparation_previous_" + stamp + ".json")
        previous.parent.mkdir(parents=True, exist_ok=True)
        previous.write_bytes(REPORT.read_bytes())
    REPORT.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    return 0 if result["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
