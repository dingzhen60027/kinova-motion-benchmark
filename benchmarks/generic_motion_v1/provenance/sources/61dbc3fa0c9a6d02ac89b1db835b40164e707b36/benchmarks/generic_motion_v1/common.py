"""Small evidence helpers shared by the benchmark command line."""
from pathlib import Path
import hashlib
import json
import os
import re
import subprocess

ROOT = Path(__file__).resolve().parents[2]
BASE = Path(__file__).resolve().parent


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")
    os.replace(temporary, path)


def read(path):
    return json.loads(Path(path).read_text(), parse_constant=_invalid_json_constant)


def _invalid_json_constant(value):
    raise ValueError(f"Nonfinite JSON constant: {value}")


def cases(path):
    records = [json.loads(line, parse_constant=_invalid_json_constant)
               for line in Path(path).read_text().splitlines() if line.strip()]
    ids = [r["problem_id"] for r in records]
    if any(not isinstance(name, str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]*', name) for name in ids):
        raise ValueError("Problem identifiers must be safe nonempty filename components")
    if len(set(ids)) != len(ids):
        raise ValueError("Duplicate problem identifiers")
    return records


def code_inventory():
    files = [BASE / name for name in ("common.py", "batch.py", "model.py", "roadmap.py", "study_protocol.py", "__main__.py", "cli.py", "dataset.py", "health.py")]
    files += list((BASE / "generator").glob("*.py"))
    # Include every runtime adapter, including optional solver and query
    # backends. A run must never omit the implementation it actually executes.
    files += list((BASE / "runners").glob("*.py"))
    files += [p for p in (BASE / "native").rglob("*") if p.is_file() and
              (p.suffix in {'.cpp', '.c', '.h', '.hpp', '.json'} or p.name.startswith(('LICENSE', 'COPYING')))]
    files += list((BASE / "configs").glob("*.json"))
    files += [BASE / 'reports/extended_baselines_v1/trajopt_author_core.tar.gz']
    # LGCS reuses the existing historical transcription without modifying it.
    files += [ROOT / "reproduction/paper_completion/optimization.py",
              ROOT / "reproduction/alignment_diagnosis/geometric_solvers.py"]
    return {str(p.relative_to(ROOT)): sha(p) for p in sorted(files) if p.is_file()}


def head():
    return subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()


def case_reference(case, manifest_path, manifest_sha256):
    """Compact, byte-bound reference to one original frozen manifest record."""
    path = Path(manifest_path).resolve()
    reference = str(path.relative_to(ROOT)) if path.is_relative_to(ROOT) else str(path)
    record_digest = hashlib.sha256(json.dumps(case, sort_keys=True,
        separators=(",", ":"), allow_nan=False).encode()).hexdigest()
    return dict(manifest_path=reference, manifest_sha256=manifest_sha256,
        problem_id=case["problem_id"], problem_sha256=case.get("problem_sha256"),
        record_sha256=record_digest, scene_id=case["scene_id"])


def case_manifest_path(reference):
    path = Path(reference["manifest_path"])
    return (ROOT / path).resolve() if not path.is_absolute() else path.resolve()


def resolve_case(job, cache=None):
    """Read old inline jobs and new compact jobs through one strict accessor.

    A caller may share a cache within one verification/aggregation operation.
    Never retain it across operations: a new call must recheck frozen bytes.
    Returned records are read-only by convention; do not attach them to jobs.
    """
    if "case_ref" not in job:
        case = job.get("case", {})
        if not isinstance(case, dict):
            raise ValueError("Inline job case must be an object")
        return case
    if "case" in job:
        raise ValueError("Job cannot contain both case and case_ref")
    reference = job["case_ref"]
    required = {"manifest_path", "manifest_sha256", "problem_id",
                "problem_sha256", "record_sha256", "scene_id"}
    if not isinstance(reference, dict) or set(reference) != required:
        raise ValueError("Malformed frozen case reference")
    path = case_manifest_path(reference)
    key = (str(path), reference["manifest_sha256"])
    cache = {} if cache is None else cache
    if key not in cache:
        content = path.read_bytes()
        if hashlib.sha256(content).hexdigest() != reference["manifest_sha256"]:
            raise ValueError("Referenced input manifest changed")
        records = [json.loads(line, parse_constant=_invalid_json_constant)
                   for line in content.decode().splitlines() if line.strip()]
        by_id = {row["problem_id"]: row for row in records}
        if len(by_id) != len(records):
            raise ValueError("Referenced input manifest has duplicate problem identifiers")
        cache[key] = by_id
    case = cache[key].get(reference["problem_id"])
    if case is None:
        raise ValueError("Referenced problem is absent from frozen input manifest")
    digest = hashlib.sha256(json.dumps(case, sort_keys=True,
        separators=(",", ":"), allow_nan=False).encode()).hexdigest()
    if (digest != reference["record_sha256"]
            or case.get("problem_sha256") != reference["problem_sha256"]
            or case.get("scene_id") != reference["scene_id"]):
        raise ValueError("Referenced case identity or content differs")
    for job_key, case_key in (("case_id", "problem_id"), ("scene_id", "scene_id"),
                              ("family", "family")):
        if job.get(job_key) != case.get(case_key):
            raise ValueError("Job metadata differs from referenced case: " + job_key)
    return case
