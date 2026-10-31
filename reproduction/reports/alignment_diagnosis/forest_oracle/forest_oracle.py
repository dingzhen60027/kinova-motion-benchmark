"""Exact segment/circle constraint diagnostic on ten frozen Forest DRM paths.

This is an independent PWL feasibility oracle, not the paper's pointwise NTO.
The number of nodes, path-length objective, endpoints, domain, original warm
start and SNOPT settings are preserved. Only the collision formulation changes
to one exact segment/circle clearance constraint per pair, with a 1 mm margin.
No CUDA, scene generation, route planning or outcome-based selection is used.
"""
from __future__ import annotations

import argparse
from collections import Counter
import datetime
import hashlib
import importlib.metadata
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time
import traceback

import numpy as np
from pydrake.autodiffutils import AutoDiffXd, ExtractGradient, ExtractValue, InitializeAutoDiff
from pydrake.solvers import MathematicalProgram, SnoptSolver


ROOT = Path(__file__).resolve().parents[2]
SOURCE_RUN = ROOT / "reproduction/runs/20261026T025110.685774Z_paper_constructed_forest"


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def clean(value):
    if isinstance(value, np.ndarray):
        return clean(value.tolist())
    if isinstance(value, (np.integer, np.bool_)):
        return value.item()
    if isinstance(value, (float, np.floating)):
        return float(value) if np.isfinite(value) else None
    if isinstance(value, dict):
        return {str(k): clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(v) for v in value]
    if isinstance(value, Path):
        return str(value)
    return value


def write(path, data):
    path.write_text(json.dumps(clean(data), indent=2, allow_nan=False) + "\n")


def read(path):
    return json.loads(path.read_text())


def length_value_jacobian(flat):
    q = np.asarray(flat, float).reshape(-1, 2)
    delta = np.diff(q, axis=0)
    lengths = np.linalg.norm(delta, axis=1)
    direction = np.divide(delta, lengths[:, None], out=np.zeros_like(delta), where=lengths[:, None] > 0)
    gradient = np.zeros_like(q)
    gradient[:-1] -= direction
    gradient[1:] += direction
    return float(lengths.sum()), gradient.ravel()


def distance_value_jacobian(flat, centers):
    """Exact segment-to-circle-center distance and analytic endpoint Jacobian.

    Inside the segment, the derivative of the projection parameter cancels
    because the residual is perpendicular to the segment. At a clipped endpoint
    the nearest endpoint's ordinary norm gradient applies. Zero distance/length
    receives a deterministic zero-norm subgradient; these singular cases are
    recorded by controls and are not used to claim a globally smooth program.
    """
    q = np.asarray(flat, float).reshape(-1, 2)
    distances, rows = [], []
    for segment, (start, end) in enumerate(zip(q[:-1], q[1:])):
        delta = end-start
        length2 = float(delta@delta)
        t = np.clip((centers-start)@delta/max(length2, 1e-30), 0., 1.)
        difference = start+t[:, None]*delta-centers
        norm = np.linalg.norm(difference, axis=1)
        normal = np.divide(difference, norm[:, None], out=np.zeros_like(difference), where=norm[:, None] > 0)
        gradient = np.zeros((len(centers), q.size))
        gradient[:, 2*segment:2*segment+2] = (1-t[:, None])*normal
        gradient[:, 2*segment+2:2*segment+4] = t[:, None]*normal
        distances.extend(norm)
        rows.extend(gradient)
    return np.asarray(distances), np.asarray(rows)


def autodiff_result(x, values, jacobian):
    x = np.asarray(x).ravel()
    values = np.atleast_1d(values)
    if len(x) and isinstance(x[0], AutoDiffXd):
        gradient = np.atleast_2d(jacobian) @ ExtractGradient(x)
        return np.asarray([AutoDiffXd(float(v), row) for v, row in zip(values, gradient)])
    return np.asarray(values, float)


def independent_validation(q, centers, radius, start, goal, lower, upper, margin):
    """Independent 2D projection/cross-product geometry; no solver callbacks."""
    q = np.asarray(q, float)
    if not np.isfinite(q).all():
        return dict(passed=False, margin_passed=False, finite=False)
    minima, collision_pairs = [], []
    for segment, (a, b) in enumerate(zip(q[:-1], q[1:])):
        edge = b-a
        length = np.sqrt(edge[0]**2+edge[1]**2)
        for obstacle, c in enumerate(centers):
            ac = c-a
            if length == 0:
                distance = np.sqrt(ac[0]**2+ac[1]**2)
            else:
                along = (ac[0]*edge[0]+ac[1]*edge[1])/length
                if along <= 0:
                    distance = np.sqrt(ac[0]**2+ac[1]**2)
                elif along >= length:
                    bc = c-b
                    distance = np.sqrt(bc[0]**2+bc[1]**2)
                else:
                    distance = abs(ac[0]*edge[1]-ac[1]*edge[0])/length
            clearance = float(distance-radius)
            minima.append(clearance)
            if clearance < 0:
                collision_pairs.append([segment, obstacle])
    endpoint_error = float(max(np.linalg.norm(q[0]-start), np.linalg.norm(q[-1]-goal)))
    limits_pass = bool(np.all(q >= lower-1e-5) and np.all(q <= upper+1e-5))
    min_clearance = float(min(minima))
    passed = bool(endpoint_error <= 1e-5 and limits_pass and min_clearance >= 0.)
    return dict(passed=passed, margin_passed=bool(min_clearance >= margin-1e-6),
        finite=True, minimum_clearance_m=min_clearance, endpoint_error=endpoint_error,
        limits_pass=limits_pass, pair_clearances_m=minima, collision_pairs=collision_pairs,
        continuous_geometry="exact floating-point line/circle minimum via cross-product and endpoint cases")


def gradient_check(path, centers):
    x = path.ravel().copy()
    values, jac = distance_value_jacobian(x, centers)
    value, cost_grad = length_value_jacobian(x)
    numeric_jac = np.empty_like(jac)
    numeric_cost = np.empty_like(cost_grad)
    h = 1e-6
    for j in range(len(x)):
        direction = np.zeros_like(x)
        direction[j] = h
        numeric_jac[:, j] = (distance_value_jacobian(x+direction, centers)[0]-distance_value_jacobian(x-direction, centers)[0])/(2*h)
        numeric_cost[j] = (length_value_jacobian(x+direction)[0]-length_value_jacobian(x-direction)[0])/(2*h)
    autodiff_x = InitializeAutoDiff(x)
    ad = autodiff_result(autodiff_x, values, jac)
    chain = np.random.default_rng(714).normal(size=(len(x), 3))
    chain_x = np.array([AutoDiffXd(float(v), derivative) for v, derivative in zip(x, chain)])
    chain_result = autodiff_result(chain_x, values, jac)
    record = dict(finite_difference_step=h, constraints=len(values), variables=len(x),
        maximum_distance_jacobian_error=float(np.max(np.abs(jac-numeric_jac))),
        maximum_length_gradient_error=float(np.max(np.abs(cost_grad-numeric_cost))),
        autodiff_identity_error=float(np.max(np.abs(ExtractGradient(ad)-jac))),
        autodiff_chain_error=float(np.max(np.abs(ExtractGradient(chain_result)-jac@chain))),
        length=value)
    record["passed"] = bool(record["maximum_distance_jacobian_error"] < 1e-7
        and record["maximum_length_gradient_error"] < 1e-7
        and record["autodiff_identity_error"] < 1e-12 and record["autodiff_chain_error"] < 1e-12)
    return record


def solve(path, centers, radius, lower, upper, margin, config, directory):
    directory.mkdir(parents=True, exist_ok=True)
    path = np.asarray(path, float)
    prog = MathematicalProgram()
    x = prog.NewContinuousVariables(path.size, "q")
    prog.AddBoundingBoxConstraint(np.tile(lower, len(path)), np.tile(upper, len(path)), x)
    prog.AddBoundingBoxConstraint(path[0], path[0], x[:2])
    prog.AddBoundingBoxConstraint(path[-1], path[-1], x[-2:])

    def objective(z):
        value, jac = length_value_jacobian(ExtractValue(z).ravel())
        return autodiff_result(z, value, jac)[0]

    def constraints(z):
        values, jac = distance_value_jacobian(ExtractValue(z).ravel(), centers)
        return autodiff_result(z, values, jac)

    prog.AddCost(objective, x)
    count = (len(path)-1)*len(centers)
    binding = prog.AddConstraint(constraints, np.full(count, radius+margin), np.full(count, np.inf), x)
    prog.SetInitialGuess(x, path.ravel())
    solver = SnoptSolver()
    for key, value in config.items():
        prog.SetSolverOption(solver.solver_id(), key, value)
    prog.SetSolverOption(solver.solver_id(), "Print file", str((directory/"snopt.log").resolve()))
    before = time.perf_counter()
    result = solver.Solve(prog)
    elapsed = time.perf_counter()-before
    solution = result.GetSolution(x).reshape(path.shape)
    raw_constraints = result.EvalBinding(binding)
    np.savez_compressed(directory/"candidate.npz", original_path=path, solution=solution,
        centers=centers, radius=radius, margin=margin, constraint_distances=raw_constraints,
        constraint_residuals=raw_constraints-radius-margin)
    validation = independent_validation(solution, centers, radius, path[0], path[-1], lower, upper, margin)
    write(directory/"validation.json", validation)
    details = result.get_solver_details()
    detail_record = {key:getattr(details, key) for key in ["info", "xmul", "F", "Fmul", "solve_time"] if hasattr(details, key)}
    write(directory/"solver_details.json", detail_record)
    record = dict(solver="SNOPT", solver_success=bool(result.is_success()),
        solver_status=str(result.get_solution_result()), solver_info=getattr(details,"info",None),
        candidate_cost=length_value_jacobian(solution.ravel())[0],
        solver_objective=result.get_optimal_cost(), original_path_cost=length_value_jacobian(path.ravel())[0],
        original_node_count=len(path), candidate_node_count=len(solution), exact_segment_circle_constraints=count,
        minimum_constraint_residual_m=float(np.min(raw_constraints-radius-margin)),
        validation_passed=validation["passed"], margin_passed=validation["margin_passed"],
        minimum_clearance_m=validation.get("minimum_clearance_m"), solver_seconds=elapsed,
        timing_scope="diagnostic only, no paper speed comparison")
    record["status"] = ("success_numerical" if record["solver_success"] and record["validation_passed"] and record["margin_passed"]
        else "validation_failed" if record["solver_success"] else "optimization_failed")
    write(directory/"result.json", record)
    return record


def controls(output, config, margin):
    centers = np.array([[0., 0.]])
    valid_path = np.array([[-2., 0.], [0., 2.], [2., 0.]])
    impossible_path = np.array([[-2., 0.], [2., 0.]])
    bounds = np.array([-3., -3.]), np.array([3., 3.])
    valid = independent_validation(valid_path, centers, 1., valid_path[0], valid_path[-1], *bounds, margin)
    impossible = independent_validation(impossible_path, centers, 1., impossible_path[0], impossible_path[-1], *bounds, margin)
    # Non-axis-aligned geometry covers interior projection and both endpoint cases.
    gradient_path = np.array([[-2.1, .3], [-.2, 1.8], [1.7, .4], [2.5, -.8]])
    gradient_centers = np.array([[0., 0.], [-4., -.8], [3.9, .7]])
    gradient = gradient_check(gradient_path, gradient_centers)
    assert valid["passed"] and valid["margin_passed"] and not impossible["passed"]
    assert gradient["passed"], gradient
    valid_solve = solve(valid_path, centers, 1., *bounds, margin, config, output/"known_feasible")
    impossible_solve = solve(impossible_path, centers, 1., *bounds, margin, config, output/"known_infeasible")
    assert valid_solve["status"] == "success_numerical", valid_solve
    assert not impossible_solve["solver_success"] and not impossible_solve["validation_passed"], impossible_solve
    record = dict(gradient_check=gradient, known_feasible_input=valid, known_infeasible_input=impossible,
        known_feasible_solve=valid_solve, known_infeasible_solve=impossible_solve,
        passed=True, scope="independent geometry, callback derivatives and solver response controls")
    write(output/"controls.json", record)
    return record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-run", type=Path, default=SOURCE_RUN)
    parser.add_argument("--output", type=Path, default=ROOT/"reproduction/reports/alignment_diagnosis/forest_oracle")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    source_manifest = read(args.source_run/"manifest.json")
    cfg_path = args.source_run/"source_snapshot/reproduction/configs/forest.json"
    cfg = read(cfg_path)
    old = source_manifest["config"]
    selected = [f"e{e:02d}_r00_n00200" for e in range(10)]
    margin = .001
    settings = {"Major iterations limit":old["nto_major_iterations"],
        "Major feasibility tolerance":1e-6, "Major optimality tolerance":1e-6,
        "Time limit":old["optimizer_time_limit_seconds"]}
    paths = [args.source_run/"manifest.json", cfg_path]
    for case in selected:
        directory = args.source_run/"cases"/case
        paths.extend(directory/name for name in ["input.json", "case.json", "seed_path.npz", "NTO_distance/result.json"])
    input_hashes = {str(p.resolve()):sha(p) for p in paths}
    source_hashes = {str(Path(__file__).resolve()):sha(Path(__file__)), str(ROOT/"local/python.sh"):sha(ROOT/"local/python.sh")}
    shutil.copy2(Path(__file__), args.output/"forest_oracle.py")
    manifest = dict(schema_version=1, created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        command="bash local/python.sh -m reproduction.alignment_diagnosis.forest_oracle "+" ".join(sys.argv[1:]),
        source_run=args.source_run.resolve(), cases=selected, selection="all ten scenes, roadmap seed 0, 200 nodes; frozen before oracle outcomes",
        classification="posthoc exact segment/circle oracle; not paper NTO or a replacement benchmark",
        unchanged=["original DRM path warm start", "node count", "PWL length objective", "domain", "endpoints", "SNOPT settings"],
        changed="Nc pointwise collision constraints replaced by every exact segment/circle distance >= obstacle radius + point radius + 1 mm",
        collision_margin_m=margin, obstacle_radius=cfg["obstacle_radius"], point_radius=cfg["point_radius"],
        solver_settings=settings, input_hashes=input_hashes, source_hashes=source_hashes,
        runtime=dict(python=sys.version, numpy=np.__version__, drake=importlib.metadata.version("drake")),
        git_head=subprocess.check_output(["git","rev-parse","HEAD"],cwd=ROOT,text=True).strip())
    write(args.output/"manifest.json", manifest)
    try:
        controls_record = controls(args.output, settings, margin)
    except Exception as error:
        write(args.output/"control_failure.json", dict(error=repr(error), traceback=traceback.format_exc()))
        raise
    radius = cfg["obstacle_radius"]+cfg["point_radius"]
    lower, upper = np.asarray(cfg["domain_lower"]), np.asarray(cfg["domain_upper"])
    records = []
    for case in selected:
        directory = args.output/"cases"/case
        directory.mkdir(parents=True)
        source = args.source_run/"cases"/case
        inp = read(source/"input.json")
        path = np.load(source/"seed_path.npz")["path"]
        centers = np.asarray(inp["centers"])
        gradient = gradient_check(path, centers)
        write(directory/"gradient_check.json", gradient)
        np.savez_compressed(directory/"input.npz", path=path, centers=centers, lower=lower, upper=upper)
        baseline = read(source/"NTO_distance/result.json")
        write(directory/"original_nto_result.json", baseline)
        try:
            assert gradient["passed"], gradient
            record = solve(path, centers, radius, lower, upper, margin, settings, directory)
        except Exception as error:
            record = dict(status="error", error=repr(error), traceback=traceback.format_exc())
            write(directory/"result.json", record)
        records.append(dict(case=case, **record, baseline_status=baseline["status"], baseline_cost=baseline.get("cost")))
        print(json.dumps(clean(records[-1])), flush=True)
    summary = dict(planned_cases=len(selected), recorded_cases=len(records),
        status_counts=dict(Counter(r["status"] for r in records)),
        valid_oracle_cases=sum(r["status"]=="success_numerical" for r in records),
        originally_valid_nto_cases=sum(r["baseline_status"]=="success_numerical" for r in records),
        control_checks_passed=controls_record["passed"],
        all_case_gradient_checks_passed=all(read(args.output/"cases"/case/"gradient_check.json")["passed"] for case in selected),
        input_hashes_unchanged=all(sha(Path(p))==expected for p,expected in input_hashes.items()),
        source_hashes_unchanged=all(sha(Path(p))==expected for p,expected in source_hashes.items()),
        records=records)
    write(args.output/"summary.json", summary)
    lines = ["# Exact Forest segment/circle oracle diagnostic", "",
        "All ten predetermined scenes were evaluated using their original e00…e09_r00_n00200 DRM paths. "
        "The original node count, endpoints, domain, path-length objective, warm start and SNOPT settings were retained. "
        "Pointwise collision constraints were replaced with exact line-segment/circle clearance constraints with a 1 mm margin. "
        "This diagnostic is not the paper's NTO method and does not overwrite earlier results.", "",
        f"Oracle valid cases: **{summary['valid_oracle_cases']}/10**. Original pointwise-NTO valid cases on these same paths: "
        f"**{summary['originally_valid_nto_cases']}/10**. All candidates and solver failures are saved.", "",
        "| Case | Original NTO status | Oracle status | Original nodes | Oracle length (m) | Minimum clearance (mm) |",
        "|---|---|---|---:|---:|---:|"]
    for r in records:
        lines.append(f"| {r['case']} | {r['baseline_status']} | {r['status']} | {r.get('original_node_count','—')} | "
                     f"{r.get('candidate_cost',float('nan')):.6f} | {r.get('minimum_clearance_m',float('nan'))*1000:.6f} |")
    lines += ["", "The analytic Jacobian and path-length gradient were checked with central finite differences for a "
        "separate geometric control and all ten original paths. AutoDiff identity and nontrivial chain gradients were checked. "
        "A known feasible three-node path around one circle was solved and independently validated; a fixed two-node path "
        "through that circle was correctly rejected by both the independent geometry and the solver.", "",
        f"Controls: {summary['control_checks_passed']}; ten input gradient checks: {summary['all_case_gradient_checks_passed']}; "
        f"input hashes unchanged: {summary['input_hashes_unchanged']}; source hashes unchanged: {summary['source_hashes_unchanged']}.", "",
        "The validation computes line-to-center distance independently through cross products and endpoint tests. "
        "This certifies the saved PWL paths against the declared circle model in floating-point arithmetic; it does not "
        "extend to robot swept volumes or nonlinear splines. The 1 mm margin is accepted to the unchanged 1e−6 solver "
        "feasibility scale, while zero-penetration, endpoint and domain checks are also reported separately.", "",
        "A successful oracle would establish that these fixed low-node-count paths can support nontrivial collision-free "
        "optimization under stronger constraints. It would not recover the authors' unpublished NTO choices or reproduce "
        "their performance. A failed local solve would not prove global infeasibility. Timings are recorded for diagnosis "
        "only and must not be compared with the paper. See manifest.json for the frozen configuration, source/input hashes "
        "and exact execution command; cases/* contains raw candidates, constraints, gradients, SNOPT logs and validation."]
    (args.output/"report.md").write_text("\n".join(lines)+"\n")
    print(json.dumps(clean(summary)), flush=True)


if __name__ == "__main__":
    main()
