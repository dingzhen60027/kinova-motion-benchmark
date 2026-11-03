"""Diagnostic persistence and actual deadline enforcement; no GPU in these tests."""

import fcntl
import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import numpy as np
import pytest

from benchmarks.generic_motion_v1 import health


@pytest.mark.parametrize("override", [None, "current", "wrong"])
def test_environment_check_uses_same_existing_conda_override_as_launcher(
    tmp_path, monkeypatch, override
):
    """Relocation must not reject an explicitly selected existing environment."""
    monkeypatch.setattr(health, "ROOT", tmp_path / "relocated-checkout")
    monkeypatch.setitem(sys.modules, "csdecomp", SimpleNamespace(__file__="csdecomp-fixture"))
    monkeypatch.setitem(sys.modules, "pydrake", SimpleNamespace(__file__="drake-fixture"))
    monkeypatch.setattr(health.metadata, "version", lambda name: "fixture")
    monkeypatch.setattr(health.subprocess, "run", lambda argv, **kwargs:
                        subprocess.CompletedProcess(argv, 0, "fixture GPU\n", ""))
    if override is None:
        monkeypatch.delenv("CSDECOMP_CONDA_PREFIX", raising=False)
        expected = health.ROOT / ".conda/envs/csdecomp"
    else:
        expected = Path(sys.prefix) if override == "current" else tmp_path / "wrong-env"
        monkeypatch.setenv("CSDECOMP_CONDA_PREFIX", str(expected))
    result = health._environment()
    assert result["passed"] is (override == "current")
    assert result["expected_conda_prefix"] == str(expected.resolve())


def test_actual_subprocess_deadline_kills_child(tmp_path):
    result = health.timeout_probe(tmp_path / "timeout.log", timeout=0.05)
    assert result["passed"]
    assert result["status"] == "timeout"
    assert result["exit_code"] < 0
    assert result["elapsed_seconds"] < 5
    assert result["diagnostic_only"] is True


@pytest.mark.parametrize("timeout", [0, -1, float("nan"), float("inf")])
def test_bad_deadline_rejected_without_starting_child(tmp_path, timeout):
    path = tmp_path / "timeout.log"
    with pytest.raises(ValueError, match="positive and finite"):
        health.timeout_probe(path, timeout)
    assert not path.exists()


def test_numpy_diagnostics_have_json_serializable_values():
    result = health._jsonable(
        dict(passed=np.bool_(True), data=np.array([1.0, 2.0]), count=np.int64(3))
    )
    assert json.loads(json.dumps(result)) == dict(passed=True, data=[1.0, 2.0], count=3)


def _cpu_diagnostics(monkeypatch, tmp_path):
    monkeypatch.setattr(health, "BASE", tmp_path)
    for name in ("_environment", "_collision_checks", "_solver_checks"):
        monkeypatch.setattr(health, name, lambda: dict(passed=True, fixture=True))
    monkeypatch.setattr(health, "head", lambda: "test-commit")


def test_prior_health_attempt_preserved_and_failure_recorded(tmp_path, monkeypatch):
    _cpu_diagnostics(monkeypatch, tmp_path)
    output = tmp_path / "results/checks.json"
    first = health.check(output)
    assert first["passed"]
    original_attempt = first["attempts"][0]

    def fail():
        raise RuntimeError("diagnostic fixture failure")

    monkeypatch.setattr(health, "_solver_checks", fail)
    second = health.check(output)
    assert not second["passed"]
    assert second["attempts"][0] == original_attempt
    assert second["latest_attempt"] == 2
    failure = second["checks"]["convex_solver"]
    assert failure["status"] == "infrastructure_error"
    assert "diagnostic fixture failure" in failure["error"]
    assert "RuntimeError" in (output.parent / second["attempts"][1]["log"]).read_text()


def test_gpu_owner_prevents_any_diagnostic_execution(tmp_path, monkeypatch):
    _cpu_diagnostics(monkeypatch, tmp_path)
    output = tmp_path / "results/checks.json"
    output.parent.mkdir()
    monkeypatch.setattr(
        health,
        "_collision_checks",
        lambda: pytest.fail("GPU owner must not be disturbed"),
    )
    with (output.parent / ".gpu.lock").open("a+") as owner:
        fcntl.flock(owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
        result = health.check(output)
    assert not result["passed"]
    assert set(result["checks"]) == {"serial_gpu_lock"}
