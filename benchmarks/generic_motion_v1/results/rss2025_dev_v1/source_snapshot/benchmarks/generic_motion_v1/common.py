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
    files = [BASE / name for name in ("common.py", "batch.py", "model.py", "roadmap.py", "__main__.py")]
    files += list((BASE / "generator").glob("*.py"))
    files += [BASE / "runners" / name for name in ("csdecomp_baseline.py", "native_handoff.py", "validate.py", "summarize.py", "replay.py")]
    files += list((BASE / "configs").glob("*.json"))
    return {str(p.relative_to(ROOT)): sha(p) for p in sorted(files) if p.is_file()}


def head():
    return subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
