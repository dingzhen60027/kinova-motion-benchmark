"""Immutable inputs and attempts, content receipts, and process-safe run locks."""
from contextlib import contextmanager
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def serial(value):
    if isinstance(value, Path):
        return str(value)
    if type(value).__module__.startswith("numpy") and hasattr(value, "tolist"):
        return value.tolist()
    raise TypeError("Unsupported evidence value: " + type(value).__name__)


def read(path):
    return json.loads(Path(path).read_text())


def atomic(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=".write-", dir=path.parent)
    try:
        with os.fdopen(fd, "w") as stream:
            json.dump(value, stream, indent=2, allow_nan=False, default=serial)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                     allow_nan=False).encode()).hexdigest()


def sha(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as stream:
        while block := stream.read(1024 * 1024):
            h.update(block)
    return h.hexdigest()


def inventory(folder, exclude=()):
    folder = Path(folder)
    result = {}
    for p in sorted(folder.rglob("*")):
        if p.is_symlink():
            raise ValueError("Symlink in experiment evidence: " + str(p))
        relative = str(p.relative_to(folder))
        if p.is_file() and relative not in exclude:
            result[relative] = sha(p)
    return result


def identifier(value):
    if not isinstance(value, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,160}", value):
        raise ValueError("Invalid job identifier")
    return value


@contextmanager
def lock(path):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a+") as stream:
        try:
            fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError("Another process owns this experiment lock: " + str(path)) from exc
        try:
            yield
        finally:
            fcntl.flock(stream, fcntl.LOCK_UN)


def attempts(run, job_id):
    base = Path(run) / "jobs" / identifier(job_id) / "attempts"
    if not base.exists():
        return []
    children = list(base.iterdir())
    if any(p.is_symlink() or not p.is_dir() or not re.fullmatch(r"[0-9]{4,}", p.name) for p in children):
        raise ValueError("Invalid attempt layout: " + str(base))
    return sorted(children, key=lambda p: int(p.name))


def selected_attempt(run, job_id):
    rows = attempts(run, job_id)
    return rows[-1] if rows else None


def new_attempt(run, job_id):
    rows = attempts(run, job_id)
    number = 1 if not rows else int(rows[-1].name) + 1
    target = Path(run) / "jobs" / identifier(job_id) / "attempts" / f"{number:04d}"
    target.mkdir(parents=True, exist_ok=False)
    return target


def verify_attempt(attempt):
    attempt = Path(attempt)
    record = read(attempt / "receipt.json")
    if record.get("receipt_sha256") != digest({k: v for k, v in record.items() if k != "receipt_sha256"}):
        raise ValueError("Attempt receipt changed: " + str(attempt))
    if record["files"] != inventory(attempt, exclude=("receipt.json",)):
        raise ValueError("Attempt evidence changed: " + str(attempt))
    return record


def seal_attempt(attempt, status, **metadata):
    attempt = Path(attempt)
    if (attempt / "receipt.json").exists():
        raise FileExistsError("Attempt already sealed: " + str(attempt))
    record = dict(schema_version=1, status=status, attempt=attempt.name,
                  files=inventory(attempt, exclude=("receipt.json",)), **metadata)
    record["receipt_sha256"] = digest(record)
    atomic(attempt / "receipt.json", record)
    return record


def read_result(run, job_id, _seen=None):
    seen = set() if _seen is None else set(_seen)
    if job_id in seen:
        raise ValueError("Dependency cycle in experiment receipts")
    seen.add(job_id)
    attempt = selected_attempt(run, job_id)
    if attempt is None:
        return None
    if not (attempt / "receipt.json").exists():
        return dict(status="interrupted", solver_success=False, validation_passed=False,
                    attempt=attempt.name, reason="Unsealed attempt; evidence retained")
    receipt = verify_attempt(attempt)
    result = read(attempt / "result.json") if (attempt / "result.json").exists() else {}
    result.update(status=receipt["status"], attempt=attempt.name,
                  process_wall_seconds=receipt.get("wall_seconds"))
    for dependency, expected in receipt.get("dependency_receipts", {}).items():
        current = selected_attempt(run, dependency)
        if current is None or not (current / "receipt.json").exists() or sha(current / "receipt.json") != expected:
            result.update(status="stale_dependency", validation_passed=False,
                          reason="Selected dependency attempt differs from this result")
        else:
            upstream = read_result(run, dependency, seen)
            if upstream["status"] == "stale_dependency" or (
                    receipt["status"] == "success" and upstream["status"] != "success"):
                result.update(status="stale_dependency", validation_passed=False,
                              reason="A transitive dependency is no longer successful")
    if result["status"] != "success":
        # A killed worker may have written success immediately before timeout.
        result["validation_passed"] = False
    return result


def freeze_inputs(run):
    run = Path(run)
    files = {name: sha(run / name) for name in (
        "manifest.json", "resolved.json", "environment.json")}
    files.update({"source_snapshot/" + name: value
                  for name, value in inventory(run / "source_snapshot").items()})
    atomic(run / "identity.json", dict(schema_version=1, files=files))


def verify_inputs(run, current_sources=False):
    run = Path(run)
    frozen = read(run / "identity.json")["files"]
    actual = {name: sha(run / name) for name in (
        "manifest.json", "resolved.json", "environment.json")}
    actual.update({"source_snapshot/" + name: value
                   for name, value in inventory(run / "source_snapshot").items()})
    if actual != frozen:
        raise ValueError("Frozen experiment inputs changed")
    manifest = read(run / "manifest.json")
    if current_sources:
        for name, expected in manifest["source_hashes"].items():
            if not (ROOT / name).is_file() or sha(ROOT / name) != expected:
                raise ValueError("Source changed; create a new run: " + name)
    return manifest


def verify_run(run, require_complete=True):
    run = Path(run)
    manifest = verify_inputs(run)
    expected = {job["id"] for job in manifest["jobs"]}
    present = {p.name for p in (run / "jobs").iterdir()} if (run / "jobs").exists() else set()
    if present - expected:
        raise ValueError("Unexpected jobs outside frozen experiment plan")
    missing, partial, statuses = [], [], {}
    for job_id in sorted(expected):
        rows = attempts(run, job_id)
        if not rows:
            missing.append(job_id)
            continue
        for path in rows:
            if (path / "receipt.json").exists():
                verify_attempt(path)
            else:
                partial.append(str(path.relative_to(run)))
        record = read_result(run, job_id)
        statuses[job_id] = record["status"]
    stale = [job for job, status in statuses.items() if status == "stale_dependency"]
    interrupted = [job for job, status in statuses.items() if status == "interrupted"]
    complete = not missing and not partial and not stale and not interrupted
    method_ids = [j["id"] for j in manifest["jobs"] if j["kind"] == "method"]
    all_valid = complete and bool(method_ids) and all(v == "success" for v in statuses.values()) and all(
        (r := read_result(run, job_id)) is not None and r["status"] == "success"
        and r.get("solver_success") is True and r.get("validation_passed") is True
        for job_id in method_ids)
    return dict(integrity_passed=True, complete=complete, all_valid=all_valid,
                verified=complete if require_complete else True,
                planned_jobs=len(expected), missing_jobs=missing, partial_attempts=partial,
                stale_jobs=stale, interrupted_jobs=interrupted,
                statuses=statuses, scope="Content integrity and completeness; numerical validation is reported separately")
