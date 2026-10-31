"""Dependency checks and a reproducible numerical environment fingerprint."""
import importlib.metadata as metadata
import json
import os
from pathlib import Path
import platform
import subprocess
import sys

from .storage import ROOT, sha

PACKAGES = ["csdecomp", "drake", "numpy", "scipy", "cvxpy", "clarabel", "gurobipy",
            "scsplanning", "pybezier", "networkx", "matplotlib", "pandas", "pytest"]
BINARY_PACKAGES = ["csdecomp", "drake", "numpy", "scipy", "clarabel", "gurobipy"]


def versions():
    result = {}
    for name in PACKAGES:
        try:
            result[name] = metadata.version(name)
        except metadata.PackageNotFoundError:
            result[name] = None
    return result


def gpu_identity():
    try:
        p = subprocess.run(["nvidia-smi", "--query-gpu=name,uuid,driver_version",
                            "--format=csv,noheader"], text=True, capture_output=True, timeout=10)
        return dict(available=p.returncode == 0, devices=p.stdout.strip().splitlines(),
                    error=p.stderr.strip() if p.returncode else None)
    except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
        return dict(available=False, devices=[], error=repr(exc))


def fingerprint():
    binaries = {}
    for name in BINARY_PACKAGES + ["scsplanning", "pybezier"]:
        try:
            dist = metadata.distribution(name)
        except metadata.PackageNotFoundError:
            continue
        for relative in dist.files or []:
            label = str(relative)
            if (".so" in label or (name in ["scsplanning", "pybezier"] and label.endswith(".py"))):
                path = dist.locate_file(relative)
                if path.is_file():
                    binaries[name + "/" + label] = sha(path)
    return dict(python=sys.version, executable=str(Path(sys.executable).resolve()),
                platform=platform.platform(), machine=platform.machine(), packages=versions(),
                threads={k: os.getenv(k) for k in ["OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"]},
                cuda_visible_devices=os.getenv("CUDA_VISIBLE_DEVICES"), gpu=gpu_identity(),
                numerical_file_sha256=binaries)


def dependencies():
    actual = versions()
    expected = {}
    for line in (ROOT / "reproduction/requirements.lock.txt").read_text().splitlines():
        if "==" in line and not line.startswith("#"):
            name, version = line.split("==", 1)
            if name in PACKAGES:
                expected[name] = version
    expected["csdecomp"] = "0.1.2"
    mismatch = {name: dict(expected=value, installed=actual[name]) for name, value in expected.items()
                if actual[name] != value}
    missing = [name for name, version in actual.items() if version is None]
    check = subprocess.run([sys.executable, "-m", "pip", "check"], capture_output=True, text=True, timeout=30)
    return dict(passed=not mismatch and not missing and check.returncode == 0 and sys.version_info[:2] == (3, 10),
                python=sys.version, executable=sys.executable, packages=actual,
                missing=missing, version_mismatches=mismatch,
                pip_check=dict(exit_code=check.returncode, stdout=check.stdout, stderr=check.stderr))


def doctor(probe=False):
    result = dependencies()
    result["gpu"] = gpu_identity()
    if probe and result["passed"] and result["gpu"]["available"]:
        command = [sys.executable, "-m", "research_platform.diagnostics", "--probe"]
        try:
            p = subprocess.run(command, capture_output=True, text=True, timeout=40)
            marker = next((line[7:] for line in reversed(p.stdout.splitlines()) if line.startswith("PROBE: ")), None)
            result["probe"] = json.loads(marker) if marker else dict(passed=False, reason="No probe result")
            result["probe_execution"] = dict(command=command, exit_code=p.returncode, stdout=p.stdout, stderr=p.stderr)
        except subprocess.TimeoutExpired:
            result["probe"] = dict(passed=False, reason="GPU/solver probe timed out")
    result["ready"] = bool(result["passed"] and result["gpu"]["available"] and
                           (not probe or result.get("probe", {}).get("passed", False)))
    return result


def native_probe():
    import numpy as np
    import csdecomp as csd
    import cvxpy as cp
    from pydrake.solvers import MathematicalProgram, SnoptSolver, IpoptSolver, GurobiSolver
    from reproduction.environment_reconstruction.model import FixedForest
    from reproduction.common import read
    cfg = read(ROOT / "reproduction/paper_constructed/config.json")
    scene = read(ROOT / "reproduction/reports/environment_reconstruction/forest_evidence/scene.json")
    env = FixedForest(cfg, scene)
    q = np.vstack([env.start, env.goal, env.base.centers[0]])
    flags = env.cuda(q).tolist()
    solver_checks = {}
    for cls in [SnoptSolver, IpoptSolver, GurobiSolver]:
        solver = cls()
        available = solver.available() and solver.enabled()
        status = dict(available=available)
        if available:
            try:
                prog = MathematicalProgram()
                x = prog.NewContinuousVariables(1)
                prog.AddBoundingBoxConstraint([1.], [2.], x)
                prog.AddQuadraticCost(x[0] * x[0])
                r = solver.Solve(prog)
                status.update(small_problem_solved=bool(r.is_success()),
                              status=str(r.get_solution_result()))
            except Exception as exc:
                status.update(small_problem_solved=False, error=repr(exc))
        solver_checks[cls.__name__] = status
    # LGCS uses gurobipy directly; Drake's Gurobi adapter can be unavailable in
    # a wheel even when the actual backend and a restricted license work.
    try:
        import gurobipy as gp
        with gp.Model() as model:
            model.Params.OutputFlag = 0
            q = model.addVar(lb=1., ub=2.)
            model.setObjective(q*q)
            model.optimize()
            solver_checks["GurobiPy"] = dict(available=True,
                small_problem_solved=model.Status == gp.GRB.OPTIMAL, status=int(model.Status))
    except Exception as exc:
        solver_checks["GurobiPy"] = dict(available=False, error=repr(exc))
    x = cp.Variable()
    problem = cp.Problem(cp.Minimize(cp.square(x)), [x >= 1])
    problem.solve(solver="CLARABEL")
    result = dict(cuda_flags=flags, expected_flags=[True, True, False],
                  clarabel_status=problem.status, solvers=solver_checks,
                  gurobi_note="A small successful solve does not certify an unrestricted license",
                  passed=flags == [True, True, False] and problem.status == "optimal"
                  and solver_checks["SnoptSolver"].get("small_problem_solved", False)
                  and solver_checks["IpoptSolver"].get("small_problem_solved", False))
    print("PROBE: " + json.dumps(result))


if __name__ == "__main__":
    native_probe()
