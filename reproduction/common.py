"""Stable records and source identity for independently rerunnable experiments."""

from __future__ import annotations
import hashlib
import json
import os
from pathlib import Path
import tempfile
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def serial(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    raise TypeError(type(value).__name__)


def digest(value):
    raw = json.dumps(
        value, sort_keys=True, separators=(",", ":"), default=serial, allow_nan=False
    )
    return hashlib.sha256(raw.encode()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def atomic(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, name = tempfile.mkstemp(dir=path.parent, prefix=".", suffix=".tmp")
    try:
        with os.fdopen(fd, "w") as stream:
            stream.write(
                json.dumps(value, indent=2, default=serial, allow_nan=False) + "\n"
            )
        os.replace(name, path)
    finally:
        if os.path.exists(name):
            os.unlink(name)


def sources():
    files = [
        ROOT / "reproduction" / name
        for name in (
            "common.py",
            "forest.py",
            "optimization.py",
            "pipeline.py",
            "run.py",
        )
    ] + [ROOT / "reproduction/configs/forest.json"]
    files += [ROOT / "local/python.sh"]
    return {
        str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
        for p in sorted(files)
    }


def config(path=None):
    return read(path or ROOT / "reproduction/configs/forest.json")
