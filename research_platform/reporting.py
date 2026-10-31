"""Receipt-verified, full-denominator reports and read-only candidate replay.

No planner, CUDA, solver, or current robot model is imported. Reporting preserves
missing/blocked/failed jobs; cost comparisons require common validated inputs.
"""

from __future__ import annotations

from collections import Counter, defaultdict
import csv
import hashlib
import html
import itertools
import json
import math
from pathlib import Path
import tempfile

import numpy as np

METHOD_OBJECTIVES = {
    "DRM": "distance",
    "LSCS": "distance",
    "LGCS": "distance",
    "NTO_distance": "distance",
    "NTO_distance_strong": "distance",
    "SCSTO": "time",
    "NTO_time": "time",
    "NTO_time_strong": "time",
}
ROW_FIELDS = [
    "job_id",
    "case_id",
    "environment",
    "method",
    "environment_seed",
    "roadmap_seed",
    "roadmap_size",
    "status",
    "attempt_number",
    "attempt_count",
    "history_statuses",
    "attempted",
    "solver_success",
    "validation_passed",
    "strict_success",
    "input_hash",
    "cost_type",
    "cost",
    "cost_unit",
    "wall_seconds",
    "worker_wall_seconds",
    "solver_seconds",
    "validation_seconds",
    "axis_semantics",
]


def _read(path):
    return json.loads(Path(path).read_text())


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _write_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile("w", dir=path.parent, delete=False) as stream:
        json.dump(data, stream, indent=2, allow_nan=False)
        stream.write("\n")
        temporary = Path(stream.name)
    temporary.replace(path)


def _write_csv(path, rows, fields=None):
    fields = fields or list(dict.fromkeys(k for row in rows for k in row))
    with Path(path).open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        writer.writerows(
            {
                k: json.dumps(v, sort_keys=True) if isinstance(v, (dict, list)) else v
                for k, v in row.items()
            }
            for row in rows
        )


def _number(value):
    if isinstance(value, bool):
        return None
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def _first_number(mapping, names):
    for key in names:
        value = _number(mapping.get(key))
        if value is not None:
            return value
    return None


def _statistics(values):
    x = [v for v in values if v is not None]
    return dict(
        n=len(x),
        mean=float(np.mean(x)) if x else None,
        median=float(np.median(x)) if x else None,
        sample_sd=float(np.std(x, ddof=1)) if len(x) > 1 else None,
    )


def _attempt_history(run, job_id):
    """Keep retries visible without pooling their timings into method means."""
    from . import storage

    attempts = storage.attempts(run, job_id)
    history = []
    for path in attempts:
        sealed = (path / "receipt.json").is_file()
        receipt = storage.verify_attempt(path) if sealed else {}
        history.append(
            dict(
                attempt_number=int(path.name),
                status=receipt.get("status", "interrupted"),
                sealed=sealed,
            )
        )
    return dict(
        attempt_number=int(attempts[-1].name) if attempts else None,
        attempt_count=len(attempts),
        history_statuses=history,
    )


def _method_jobs(manifest):
    jobs = [j for j in manifest["jobs"] if j["kind"] == "method"]
    # A plan is not implicitly treated as a method: the declared method job
    # grid owns the denominator, including any explicit DRM baseline job.
    seen = set()
    for job in jobs:
        key = (job["case_id"], job["method"])
        if key in seen:
            raise ValueError(f"Duplicate planned case/method: {key}")
        seen.add(key)
    return jobs


def _environment(manifest, job):
    if job.get("environment"):
        return job["environment"]
    for case in manifest.get("cases", []):
        if case.get("id", case.get("case_id")) == job["case_id"] and case.get(
            "environment"
        ):
            return case["environment"]
    env = manifest.get("config", {}).get("environment", "unknown")
    return env if isinstance(env, str) else env.get("name", env.get("kind", "unknown"))


def _row(manifest, job, result):
    result = result or {}
    metrics = result.get("metrics", {})
    timing = result.get("timing", {})
    status = result.get("status", "pending")
    strict = (
        status == "success"
        and result.get("solver_success") is True
        and result.get("validation_passed") is True
    )
    objective = metrics.get("cost_type") or METHOD_OBJECTIVES.get(
        job["method"], "unknown"
    )
    environment = _environment(manifest, job)
    unit = metrics.get(
        "cost_unit",
        (
            "s"
            if objective == "time"
            else (
                "m"
                if environment == "forest"
                else "rad" if environment == "fr3" else None
            )
        ),
    )
    # Worker time is retained even for timeout/error; pending and blocked jobs
    # are not assigned synthetic zero durations or treated as attempts.
    attempted = status not in {
        "pending",
        "blocked",
        "not_run",
        "skipped",
        "not_started",
    }
    return dict(
        job_id=job["id"],
        case_id=job["case_id"],
        environment=environment,
        method=job["method"],
        environment_seed=job.get("environment_seed"),
        roadmap_seed=job.get("roadmap_seed"),
        roadmap_size=job.get("roadmap_size"),
        status=status,
        attempted=attempted,
        solver_success=result.get("solver_success"),
        validation_passed=result.get("validation_passed"),
        strict_success=strict,
        input_hash=result.get("input_hash"),
        cost_type=objective,
        cost=_number(metrics.get("cost")),
        cost_unit=unit,
        wall_seconds=(
            _number(result.get("process_wall_seconds"))
            if _number(result.get("process_wall_seconds")) is not None
            else _first_number(
                timing, ["wall_seconds", "elapsed_seconds", "total_wall_seconds"]
            )
        ),
        worker_wall_seconds=_first_number(
            timing, ["wall_seconds", "elapsed_seconds", "total_wall_seconds"]
        ),
        solver_seconds=_first_number(
            timing, ["solver_seconds", "optimization_seconds"]
        ),
        validation_seconds=_first_number(timing, ["validation_seconds"]),
        axis_semantics=result.get("axis_semantics"),
    )


def paired_rows(rows):
    groups = defaultdict(dict)
    for row in rows:
        groups[(row["environment"], row["cost_type"], row["method"])][
            row["case_id"]
        ] = row
    output = []
    for environment, objective in sorted(
        {(r["environment"], r["cost_type"]) for r in rows}
    ):
        methods = sorted(m for e, o, m in groups if (e, o) == (environment, objective))
        for a, b in itertools.combinations(methods, 2):
            aa, bb = (
                groups[environment, objective, a],
                groups[environment, objective, b],
            )
            for case_id in sorted(set(aa) | set(bb)):
                x, y = aa.get(case_id), bb.get(case_id)
                reason = None
                if x is None or y is None:
                    reason = "method_not_planned_for_case"
                elif not x["strict_success"] or not y["strict_success"]:
                    reason = "not_both_strict_success"
                elif not x["input_hash"] or not y["input_hash"]:
                    reason = "missing_input_hash"
                elif x["input_hash"] != y["input_hash"]:
                    reason = "different_frozen_inputs"
                elif x["cost_unit"] != y["cost_unit"]:
                    reason = "different_cost_units"
                common = reason is None
                cost_common = common and x["cost"] is not None and y["cost"] is not None
                if common and not cost_common:
                    reason = "missing_finite_cost"
                output.append(
                    dict(
                        environment=environment,
                        cost_type=objective,
                        method_a=a,
                        method_b=b,
                        case_id=case_id,
                        both_planned=x is not None and y is not None,
                        status_a=x["status"] if x else "not_planned",
                        status_b=y["status"] if y else "not_planned",
                        common_validated_input=common,
                        common_finite_cost=cost_common,
                        exclusion_reason=reason,
                        input_hash=x["input_hash"] if common else None,
                        cost_a=x["cost"] if cost_common else None,
                        cost_b=y["cost"] if cost_common else None,
                        cost_difference_a_minus_b=(
                            x["cost"] - y["cost"] if cost_common else None
                        ),
                        cost_ratio_a_over_b=(
                            x["cost"] / y["cost"]
                            if cost_common and y["cost"] != 0
                            else None
                        ),
                        wall_ratio_a_over_b=(
                            x["wall_seconds"] / y["wall_seconds"]
                            if common
                            and x["wall_seconds"] is not None
                            and y["wall_seconds"] is not None
                            and y["wall_seconds"] > 0
                            else None
                        ),
                    )
                )
    return output


def summarize_run(run_dir, output_dir=None):
    from . import storage

    run = Path(run_dir).resolve()
    out = Path(output_dir or run / "reports").resolve()
    out.mkdir(parents=True, exist_ok=True)
    manifest = storage.verify_inputs(run)
    jobs = _method_jobs(manifest)
    rows = [
        dict(
            _row(manifest, job, storage.read_result(run, job["id"])),
            **_attempt_history(run, job["id"]),
        )
        for job in jobs
    ]
    summaries = []
    for environment, method in sorted({(r["environment"], r["method"]) for r in rows}):
        part = [
            r for r in rows if r["environment"] == environment and r["method"] == method
        ]
        passed = [r for r in part if r["strict_success"]]
        attempted = [r for r in part if r["attempted"]]
        summaries.append(
            dict(
                environment=environment,
                method=method,
                cost_type=part[0]["cost_type"],
                requested=len(part),
                attempted=len(attempted),
                strict_successes=len(passed),
                strict_success_rate=len(passed) / len(part),
                statuses=dict(sorted(Counter(r["status"] for r in part).items())),
                cost_on_strict_success=_statistics([r["cost"] for r in passed]),
                wall_seconds_latest_attempts=_statistics(
                    [r["wall_seconds"] for r in attempted]
                ),
                solver_seconds_latest_attempts=_statistics(
                    [r["solver_seconds"] for r in attempted]
                ),
                validation_seconds_latest_attempts=_statistics(
                    [r["validation_seconds"] for r in attempted]
                ),
            )
        )
    pairs = paired_rows(rows)
    pair_summaries = []
    for key in sorted(
        {
            (r["environment"], r["cost_type"], r["method_a"], r["method_b"])
            for r in pairs
        }
    ):
        part = [
            r
            for r in pairs
            if (r["environment"], r["cost_type"], r["method_a"], r["method_b"]) == key
        ]
        pair_summaries.append(
            dict(
                environment=key[0],
                cost_type=key[1],
                method_a=key[2],
                method_b=key[3],
                union_cases=len(part),
                both_planned=sum(r["both_planned"] for r in part),
                common_validated_inputs=sum(r["common_validated_input"] for r in part),
                common_finite_costs=sum(r["common_finite_cost"] for r in part),
                exclusions=dict(
                    Counter(
                        r["exclusion_reason"] for r in part if r["exclusion_reason"]
                    )
                ),
                cost_difference_a_minus_b=_statistics(
                    [r["cost_difference_a_minus_b"] for r in part]
                ),
                cost_ratio_a_over_b=_statistics(
                    [r["cost_ratio_a_over_b"] for r in part]
                ),
                common_success_wall_ratio_a_over_b=_statistics(
                    [r["wall_ratio_a_over_b"] for r in part]
                ),
            )
        )
    result = dict(
        schema_version=1,
        name=manifest.get("name", run.name),
        run_dir=str(run),
        manifest_sha256=_sha(run / "manifest.json"),
        reporting_source_sha256=_sha(Path(__file__).resolve()),
        planned_method_jobs=len(jobs),
        status_counts=dict(Counter(r["status"] for r in rows)),
        methods=summaries,
        jobs=rows,
        paired=pair_summaries,
        definitions=dict(
            strict_success="status=success AND solver_success=true AND validation_passed=true",
            success_denominator="Every planned method job, including pending, blocked, timeout and errors",
            attempt_selection="Only the latest attempt of each planned job contributes its status, costs and timings to method statistics; older attempts are never substituted or pooled",
            history_statuses="All attempts in numeric order, with their recorded receipt status and seal state; the selected row status additionally applies current dependency validity",
            paired_cost="Same case, objective, cost units and nonempty frozen input_hash; both strictly validated; finite costs",
            missing="Null means unavailable; missing timings and costs are never replaced by zero",
            ratios="Mean of per-case ratios; zero baseline cost excluded from ratio only",
            timing="wall_seconds uses external attempt process wall time when available; worker wall and solver time retain separate scopes",
            uncertainty="Sample standard deviations only; no iid confidence intervals inferred from dependent roadmap repetitions",
        ),
    )
    _write_csv(out / "summary.csv", rows, ROW_FIELDS)
    _write_csv(
        out / "paired.csv",
        pairs,
        (
            list(pairs[0])
            if pairs
            else [
                "environment",
                "cost_type",
                "method_a",
                "method_b",
                "case_id",
                "common_validated_input",
                "exclusion_reason",
            ]
        ),
    )
    aggregate = []
    for item in summaries:
        flat = {k: v for k, v in item.items() if not isinstance(v, dict)}
        flat["statuses"] = json.dumps(item["statuses"], sort_keys=True)
        for metric in [
            "cost_on_strict_success",
            "wall_seconds_latest_attempts",
            "solver_seconds_latest_attempts",
            "validation_seconds_latest_attempts",
        ]:
            flat.update({metric + "_" + k: v for k, v in item[metric].items()})
        aggregate.append(flat)
    _write_csv(
        out / "method_summary.csv",
        aggregate,
        (
            list(aggregate[0])
            if aggregate
            else ["environment", "method", "requested", "strict_successes"]
        ),
    )
    _write_json(out / "summary.json", result)
    columns = [
        "environment",
        "method",
        "requested",
        "attempted",
        "strict_successes",
        "strict_success_rate",
    ]
    header = "".join(f"<th>{html.escape(k)}</th>" for k in columns)
    cells = "".join(
        "<tr>"
        + "".join(f"<td>{html.escape(str(r[k]))}</td>" for k in columns)
        + "</tr>"
        for r in summaries
    )
    (out / "summary.html").write_text(
        '<!doctype html><meta charset="utf-8"><title>Research run summary</title><style>body{font:15px system-ui;max-width:1100px;margin:40px auto}table{border-collapse:collapse}th,td{padding:10px;border-bottom:1px solid #ccd}th{text-align:left}code{background:#eee}</style>'
        + f'<h1>{html.escape(result["name"])}</h1><p>Full planned denominator: {len(jobs)} method jobs. Pending, blocked, failed and interrupted jobs remain included.</p>'
        + "<table><tr>"
        + header
        + "</tr>"
        + cells
        + "</table><p>Only the latest attempt of each job enters statistics; retries remain visible in summary.csv and summary.json and are never pooled into method averages. Strict success requires successful solver status and independent validation. Paired costs require identical frozen inputs and the same objective; see paired.csv and summary.json.</p>"
        + f'<p>Report implementation SHA-256: <code>{result["reporting_source_sha256"]}</code></p>'
    )
    return result


def _samples(path):
    with np.load(path, allow_pickle=False) as data:
        q = np.asarray(data["q"], float)
        key = next((k for k in ["time", "times", "t"] if k in data), None)
        if key is None:
            raise ValueError(f"Missing saved parameter/time axis: {path}")
        t = np.asarray(data[key], float)
        if q.ndim != 2 or t.ndim != 1 or len(q) != len(t):
            raise ValueError(f"Unaligned trajectory arrays: {path}")
        finite = np.isfinite(t)
        q = q[finite]
        t = t[finite]
        if not len(t) or not np.isfinite(q).all() or np.any(np.diff(t) < 0):
            raise ValueError(f"Invalid saved trajectory axis/configuration: {path}")
        retained = len(t)
        indices = np.unique(np.linspace(0, len(t) - 1, min(1001, len(t))).astype(int))
        return dict(
            q=q[indices].tolist(),
            axis=t[indices].tolist(),
            finite_rows=retained,
            displayed_rows=len(indices),
            excluded_nonfinite_axis_rows=int((~finite).sum()),
            source_rows=np.flatnonzero(finite)[indices].tolist(),
        )


def _fr3_geometry(run, manifest):
    """Read only the frozen model; a working-tree model is never a fallback."""
    source = "reproduction/paper_constructed/assets/fr3_33.json"
    expected = manifest.get("source_hashes", {}).get(source)
    path = run / "source_snapshot" / source
    if not expected or not path.is_file() or _sha(path) != expected:
        raise ValueError(
            "FR3 replay requires the manifest-matching frozen 33-sphere asset"
        )
    asset = _read(path)
    from scipy.spatial.transform import Rotation

    fixed = []
    for i in range(1, 9):
        origin = asset["kinematics"][f"joint{i}"]["kinematic"]
        transform = np.eye(4)
        transform[:3, :3] = Rotation.from_euler(
            "xyz", [origin[k] for k in ["roll", "pitch", "yaw"]]
        ).as_matrix()
        transform[:3, 3] = [origin[k] for k in ["x", "y", "z"]]
        fixed.append(transform.tolist())
    balls = [
        dict(link=name, center=ball["center"], radius=ball["radius"])
        for name, items in asset["spheres"].items()
        for ball in items
    ]
    if len(balls) != 33:
        raise ValueError("Frozen FR3 asset does not contain 33 spheres")
    geometry = dict(
        fixed=fixed,
        balls=balls,
        convention="fixed URDF origins, local z revolute joints, fixed open gripper at 0.04 m",
        source=source,
        sha256=expected,
        fidelity=asset.get("fidelity"),
    )
    return geometry, path


def _fr3_frames(q, geometry):
    """CPU FK, including the fixed open-hand geometry used by the planner."""
    q = np.asarray(q, float)
    if q.shape != (7,) or not np.isfinite(q).all():
        raise ValueError("Invalid saved FR3 configuration")

    def rz(angle):
        c, s = np.cos(angle), np.sin(angle)
        return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

    current = np.eye(4)
    frames = {"fr3_link0": current.copy()}
    for i, origin in enumerate(geometry["fixed"], 1):
        current = current @ np.asarray(origin)
        if i < 8:
            current = current @ rz(q[i - 1])
        frames[f"fr3_link{i}"] = current.copy()
    frames["fr3_hand"] = current @ rz(-np.pi / 4)
    for name, xyz, yaw in [
        ("fr3_leftfinger", [0, 0.04, 0.0584], 0),
        ("fr3_rightfinger", [0, -0.04, 0.0584], np.pi),
        ("fr3_hand_tcp", [0, 0, 0.1034], 0),
    ]:
        transform = rz(yaw)
        transform[:3, 3] = xyz
        frames[name] = frames["fr3_hand"] @ transform
    return frames


def _sphere_centers(q, geometry):
    frames = _fr3_frames(q, geometry)
    return [
        ((frames[ball["link"]] @ np.r_[ball["center"], 1])[:3], ball["radius"])
        for ball in geometry["balls"]
    ]


def replay_case(run_dir, case_id, output_dir=None):
    from . import storage
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle

    run = Path(run_dir).resolve()
    out = Path(output_dir or run / "reports" / "replay" / case_id).resolve()
    out.mkdir(parents=True, exist_ok=True)
    manifest = storage.verify_inputs(run)
    jobs = [j for j in _method_jobs(manifest) if j["case_id"] == case_id]
    if not jobs:
        raise ValueError(f"Case has no planned method jobs: {case_id}")
    environment = _environment(manifest, jobs[0])
    plan_jobs = [
        j
        for j in manifest["jobs"]
        if j["kind"] == "plan" and j.get("case_id") == case_id
    ]
    inputs = {}
    inventory = {}
    traces = []
    missing = []
    plan_hash = None
    plan_result = None
    plan_attempt = None
    for plan in plan_jobs:
        plan_result = storage.read_result(run, plan["id"])
        plan_hash = (plan_result or {}).get("input_hash")
        attempt = storage.selected_attempt(run, plan["id"])
        plan_attempt = attempt
        if attempt and (Path(attempt) / "input.json").exists():
            p = Path(attempt) / "input.json"
            inputs = _read(p)
            inventory[str(p.relative_to(run))] = _sha(p)
            break
    if not inputs:
        # Failed IK may never produce input.json. Preserve the frozen scene
        # context instead of making a blocked case appear to have no obstacles.
        for plan in plan_jobs:
            scene_id = plan.get("dependencies", {}).get("scene")
            if not scene_id:
                continue
            storage.read_result(run, scene_id)
            scene_attempt = storage.selected_attempt(run, scene_id)
            p = Path(scene_attempt) / "scene.json" if scene_attempt else None
            if p and p.exists():
                scene = _read(p)
                inputs = dict(
                    scene=scene,
                    centers=scene.get("centers", scene.get("obstacles", [])),
                )
                inputs.update(
                    {
                        k: scene[k]
                        for k in ["start", "goal", "target_pose"]
                        if scene.get(k) is not None
                    }
                )
                inventory[str(p.relative_to(run))] = _sha(p)
                break
    palette = ["#3264a8", "#269574", "#9b6ab0", "#be7725", "#31554c", "#c54e57"]
    for i, job in enumerate(jobs):
        record = storage.read_result(run, job["id"])
        row = _row(manifest, job, record)
        attempt = storage.selected_attempt(run, job["id"])
        path = Path(attempt) / "trajectory_samples.npz" if attempt else None
        if path is None or not path.exists():
            missing.append(
                dict(
                    method=job["method"],
                    status=row["status"],
                    reason="no_saved_candidate",
                )
            )
            continue
        if plan_hash and row["input_hash"] and plan_hash != row["input_hash"]:
            raise ValueError(f'Candidate uses different frozen inputs: {job["id"]}')
        inventory[str(path.relative_to(run))] = _sha(path)
        try:
            values = _samples(path)
            dimension = len(values["q"][0])
            expected = (
                2
                if environment == "forest"
                else 7 if environment == "fr3" else dimension
            )
            if dimension != expected:
                raise ValueError(f"{environment} candidate has {dimension} coordinates")
        except (ValueError, KeyError) as exc:
            missing.append(
                dict(
                    method=job["method"],
                    status=row["status"],
                    reason="saved_candidate_not_renderable",
                    detail=str(exc),
                )
            )
            continue
        traces.append(
            dict(
                method=job["method"],
                status=row["status"],
                strict_success=row["strict_success"],
                evidence_sealed=(Path(attempt) / "receipt.json").is_file(),
                axis_semantics=row["axis_semantics"] or "unknown_saved_axis",
                color=palette[i % len(palette)],
                **values,
            )
        )
    seed = Path(plan_attempt) / "trajectory_samples.npz" if plan_attempt else None
    if not any(t["method"] == "DRM" for t in traces) and seed and seed.exists():
        # If shared planning failed validation, all methods are blocked. The
        # rejected seed is still inspectable, explicitly as a plan input rather
        # than a successful DRM method run, and never changes the denominator.
        record = plan_result or {}
        traces.append(
            dict(
                method="Shared DRM seed (input)",
                status=record.get("status", "pending"),
                strict_success=record.get("status") == "success"
                and record.get("solver_success") is True
                and record.get("validation_passed") is True,
                evidence_sealed=(Path(plan_attempt) / "receipt.json").is_file(),
                axis_semantics=record.get(
                    "axis_semantics", "normalized_path_parameter"
                ),
                color="#bf3657",
                **_samples(seed),
            )
        )
        inventory[str(seed.relative_to(run))] = _sha(seed)
    payload = dict(
        case_id=case_id,
        environment=environment,
        inputs=inputs,
        traces=traces,
        missing=missing,
        note="Replay shows saved candidates, including failures. Preview progression is normalized independently per method; it is not execution time or wall time.",
    )
    if environment == "fr3":
        geometry, path = _fr3_geometry(run, manifest)
        inventory[str(path.relative_to(run))] = _sha(path)
        payload["robot_geometry"] = geometry
        resolved = _read(run / "resolved.json")
        payload["fr3_scene"] = resolved.get("protocol", {}).get("fr3_scene", {})
        for trace in traces:
            trace["tcp"] = [
                _fr3_frames(q, geometry)["fr3_hand_tcp"][:3, 3].tolist()
                for q in trace["q"]
            ]
    if environment == "forest":
        fig, ax = plt.subplots(figsize=(7, 7))
        centers = inputs.get("centers", inputs.get("obstacle_centers", []))
        radius = inputs.get("scene", inputs).get("obstacle_radius", 0.35)
        for center in centers:
            ax.add_patch(Circle(center, radius, fc="#c9c9c9", ec="#888"))
        for trace in traces:
            q = np.array(trace["q"])
            ax.plot(
                q[:, 0],
                q[:, 1],
                color=trace["color"],
                ls="-" if trace["strict_success"] else "--",
                label=trace["method"] + " / " + trace["status"],
            )
        for k, marker in [("start", "o"), ("goal", "*")]:
            if k in inputs:
                ax.scatter(*inputs[k], c="black", marker=marker, s=65, zorder=5)
        ax.set(
            xlim=(-5, 5),
            ylim=(-5, 5),
            xlabel="x [m]",
            ylabel="y [m]",
            title=case_id + " · saved candidates",
        )
        ax.set_aspect("equal")
        if traces:
            ax.legend(fontsize=8)
    else:
        fig, axes = plt.subplots(4, 2, figsize=(11, 10))
        axes = axes.ravel()
        for trace in traces:
            q = np.array(trace["q"])
            t = np.array(trace["axis"])
            u = (t - t[0]) / (t[-1] - t[0]) if t[-1] > t[0] else np.zeros(len(t))
            for j in range(q.shape[1]):
                axes[j].plot(
                    u,
                    q[:, j],
                    color=trace["color"],
                    ls="-" if trace["strict_success"] else "--",
                    label=trace["method"] + " / " + trace["status"],
                )
        for j, ax in enumerate(axes[:7]):
            ax.set(xlabel="Normalized saved progress", ylabel=f"q{j+1} [rad]")
        axes[-1].axis("off")
        if traces:
            axes[-1].legend(
                *axes[0].get_legend_handles_labels(), loc="center", fontsize=8
            )
        fig.suptitle(case_id + " · joint-space replay of saved candidates")
    fig.tight_layout()
    fig.savefig(out / "replay.png", dpi=170)
    plt.close(fig)
    files = ["replay.png", "replay.html"]
    if environment == "fr3":
        from mpl_toolkits.mplot3d.art3d import Line3DCollection

        fig = plt.figure(figsize=(9, 7))
        ax = fig.add_subplot(111, projection="3d")
        cfg = payload["fr3_scene"]
        center = np.asarray(cfg.get("table_center", [0.4, 0, -0.1]))
        size = np.asarray(cfg.get("table_size", [2.5, 2.5, 0.2]))
        corners = (
            np.array(list(itertools.product([-1, 1], repeat=3))) * size / 2 + center
        )
        edges = [
            [a, b]
            for a, b in itertools.combinations(corners, 2)
            if np.count_nonzero(a != b) == 1
        ]
        ax.add_collection3d(Line3DCollection(edges, colors="#a5a9b3", linewidths=0.8))
        u, v = np.meshgrid(np.linspace(0, 2 * np.pi, 12), np.linspace(0, np.pi, 8))
        unit = np.array([np.cos(u) * np.sin(v), np.sin(u) * np.sin(v), np.cos(v)])

        def spheres(balls, color, alpha):
            for c, r in balls:
                points = unit * r + np.asarray(c)[:, None, None]
                ax.plot_surface(
                    *points, color=color, alpha=alpha, linewidth=0, shade=True
                )

        radius = inputs.get("scene", {}).get(
            "obstacle_radius", cfg.get("obstacle_radius", 0.05)
        )
        spheres([(c, radius) for c in inputs.get("centers", [])], "#333333", 0.9)
        for label, color in [("start", "#cf3c51"), ("goal", "#3264a8")]:
            if inputs.get(label) is not None:
                spheres(_sphere_centers(inputs[label], geometry), color, 0.45)
        for trace in traces:
            ax.plot(
                *np.asarray(trace["tcp"]).T,
                color=trace["color"],
                ls="-" if trace["strict_success"] else "--",
                label=trace["method"] + " / " + trace["status"],
            )
        ax.set(
            xlim=(-0.6, 1.1),
            ylim=(-0.85, 0.85),
            zlim=(-0.25, 1.35),
            xlabel="x [m]",
            ylabel="y [m]",
            zlabel="z [m]",
        )
        ax.view_init(25, -50)
        ax.set_box_aspect([1.7, 1.7, 1.6])
        if traces:
            ax.legend(fontsize=8)
        ax.set_title(
            case_id
            + " · frozen 33-sphere start (red), goal (blue), and saved TCP paths\nWireframe: actual collision box; no visual mesh substituted",
            fontsize=10,
        )
        fig.tight_layout()
        fig.savefig(out / "workspace.png", dpi=170)
        plt.close(fig)
        files.append("workspace.png")
    data = json.dumps(payload, separators=(",", ":"), allow_nan=False).replace(
        "</", "<\\/"
    )
    (out / "replay.html").write_text(REPLAY_HTML.replace("__PAYLOAD__", data))
    result = dict(
        schema_version=1,
        case_id=case_id,
        environment=environment,
        rendered_methods=[t["method"] for t in traces],
        missing=missing,
        manifest_sha256=_sha(run / "manifest.json"),
        reporting_source_sha256=_sha(Path(__file__).resolve()),
        source_artifacts=inventory,
        source_rows={
            t["method"]: dict(
                finite_rows=t["finite_rows"],
                displayed_rows=t["displayed_rows"],
                excluded_nonfinite_axis_rows=t["excluded_nonfinite_axis_rows"],
                axis_semantics=t["axis_semantics"],
            )
            for t in traces
        },
        scope=payload["note"],
        files=files,
    )
    _write_json(out / "replay_manifest.json", result)
    return result


REPLAY_HTML = r"""<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Saved candidate replay</title>
<style>body{font:14px system-ui;margin:24px;color:#203044}h1{font-size:23px}canvas{width:100%;max-width:1100px;height:640px;border:1px solid #ccd;background:#fafbfc;touch-action:none}#legend{line-height:1.8}label{margin-right:16px}input[type=range]{width:45%}.failed{color:#ab2440}button,select{padding:6px 10px}#state{white-space:pre-wrap}</style>
<h1 id="title"></h1><p>Saved candidates only. Dashed lines failed acceptance. Playback progresses independently along each saved axis at a fixed display speed; it is not measured execution time.</p>
<div id="legend"></div><p><button id="play">Play</button> <label>Progress <input type="range" id="progress" min="0" max="1" step="0.001" value="0"></label><span id="position"></span>
<label id="view-label">View <select id="view"><option value="workspace">Workspace / frozen robot</option><option value="joints">Seven joint curves</option></select></label> <button id="reset">Reset view</button></p>
<p id="geometry-note"></p><canvas id="canvas"></canvas><pre id="state"></pre>
<script>
const D=__PAYLOAD__;
const C=document.getElementById('canvas'), X=C.getContext('2d'), R=document.getElementById('progress'), V=document.getElementById('view');
let playing=false, last=null, yaw=-.87, elevation=.44, zoom=1, dragging=null;
const enabled={};
document.getElementById('title').textContent=D.case_id+' · '+D.environment;
document.getElementById('view-label').hidden=D.environment!=='fr3';
document.getElementById('reset').hidden=D.environment!=='fr3';
if(D.environment==='fr3') document.getElementById('geometry-note').textContent='Drag to rotate; wheel to zoom. Actual frozen 33-sphere planner model; red start / blue goal. Gray wireframe is the collision table. TCP curves are projections of saved configurations, not validation of workspace straight lines. '+D.robot_geometry.fidelity;
D.traces.forEach((t,i)=>{
  enabled[i]=true;
  const label=document.createElement('label'), box=document.createElement('input');
  box.type='checkbox'; box.checked=true; box.dataset.method=t.method;
  box.onchange=()=>{enabled[i]=box.checked;draw()};
  label.append(box,document.createTextNode(t.method+' / '+t.status+' ('+t.axis_semantics+')'+(t.evidence_sealed?'':' [unsealed evidence]')));
  label.style.color=t.color; document.getElementById('legend').append(label);
});
D.missing.forEach(t=>{
  const p=document.createElement('div');p.className='failed';
  p.textContent=t.method+' / '+t.status+': '+t.reason+(t.detail?' ('+t.detail+')':'');document.getElementById('legend').append(p);
});
function sampleAt(t,u){
  const target=t.axis[0]+u*(t.axis[t.axis.length-1]-t.axis[0]);let lo=0,hi=t.axis.length-1;
  while(lo<hi){const mid=Math.floor((lo+hi)/2);if(t.axis[mid]<target)lo=mid+1;else hi=mid}
  return lo>0&&target-t.axis[lo-1]<t.axis[lo]-target?lo-1:lo;
}
function line(points,color,dash){
  X.beginPath();points.forEach((p,i)=>i?X.lineTo(p[0],p[1]):X.moveTo(p[0],p[1]));
  X.strokeStyle=color;X.lineWidth=2;X.setLineDash(dash?[6,4]:[]);X.stroke();X.setLineDash([]);
}
function dot(p,color,radius=4,alpha=1){
  X.beginPath();X.arc(p[0],p[1],radius,0,Math.PI*2);X.fillStyle=color;X.globalAlpha=alpha;X.fill();X.globalAlpha=1;
}
const eye=()=>[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]];
function multiply(a,b){return a.map(row=>b[0].map((_,j)=>row.reduce((s,v,k)=>s+v*b[k][j],0)))}
function rz(a){let m=eye();m[0][0]=m[1][1]=Math.cos(a);m[0][1]=-Math.sin(a);m[1][0]=Math.sin(a);return m}
function frames(q){
  let current=eye(), out={fr3_link0:current};
  D.robot_geometry.fixed.forEach((origin,k)=>{current=multiply(current,origin);if(k<7)current=multiply(current,rz(q[k]));out['fr3_link'+(k+1)]=current});
  out.fr3_hand=multiply(current,rz(-Math.PI/4));
  [['fr3_leftfinger',[0,.04,.0584],0],['fr3_rightfinger',[0,-.04,.0584],Math.PI],['fr3_hand_tcp',[0,0,.1034],0]].forEach(([name,p,angle])=>{
    let t=rz(angle);p.forEach((v,k)=>t[k][3]=v);out[name]=multiply(out.fr3_hand,t);
  });return out;
}
function balls(q,color,alpha){
  const f=frames(q);
  return D.robot_geometry.balls.map(b=>({center:f[b.link].slice(0,3).map(row=>row.slice(0,3).reduce((s,v,k)=>s+v*b.center[k],row[3])),radius:b.radius,color,alpha}));
}
function drawWorkspace(w,h,progress,state){
  const scale=Math.min(w,h)*.48*zoom, cy=Math.cos(yaw),sy=Math.sin(yaw),ce=Math.cos(elevation),se=Math.sin(elevation);
  const P=p=>{const dx=p[0]-.3,dy=p[1],dz=p[2]-.45,a=cy*dx+sy*dy,b=-sy*dx+cy*dy;return [w/2+scale*b,h/2-scale*(ce*dz-se*a),ce*a+se*dz]};
  const cfg=D.fr3_scene, c=cfg.table_center||[.4,0,-.1], s=cfg.table_size||[2.5,2.5,.2];
  const corners=[];for(let x of [-1,1])for(let y of [-1,1])for(let z of [-1,1])corners.push([c[0]+x*s[0]/2,c[1]+y*s[1]/2,c[2]+z*s[2]/2]);
  corners.forEach((a,i)=>corners.slice(i+1).forEach(b=>{if(a.filter((v,k)=>v!==b[k]).length===1)line([P(a),P(b)],'#b7bdc7',false)}));
  let primitives=(D.inputs.centers||[]).map(center=>({center,radius:(D.inputs.scene||{}).obstacle_radius||cfg.obstacle_radius||.05,color:'#252a31',alpha:.9}));
  if(D.inputs.start)primitives.push(...balls(D.inputs.start,'#cf3c51',.25));
  if(D.inputs.goal)primitives.push(...balls(D.inputs.goal,'#3264a8',.25));
  D.traces.forEach((t,i)=>{
    if(!enabled[i])return;const j=sampleAt(t,progress);
    line(t.tcp.map(P),t.color,!t.strict_success);primitives.push(...balls(t.q[j],t.color,.7));
    state.push(t.method+': '+t.axis_semantics+' = '+t.axis[j].toFixed(4)+'; saved row '+t.source_rows[j]);
  });
  primitives.map(b=>({...b,p:P(b.center)})).sort((a,b)=>a.p[2]-b.p[2]).forEach(b=>dot(b.p,b.color,Math.max(.5,b.radius*scale),b.alpha));
  [['x',[.2,0,0],'#c33'],['y',[0,.2,0],'#393'],['z',[0,0,.2],'#36b']].forEach(([label,p,color])=>{line([P([0,0,0]),P(p)],color,false);X.fillStyle=color;X.fillText(label,...P(p).slice(0,2))});
}
function draw(){
  const w=C.clientWidth,h=C.clientHeight,ratio=devicePixelRatio||1;C.width=w*ratio;C.height=h*ratio;
  X.setTransform(ratio,0,0,ratio,0,0);X.fillStyle='#fafbfc';X.fillRect(0,0,w,h);
  const progress=+R.value,state=[];document.getElementById('position').textContent=(progress*100).toFixed(1)+'%';
  if(D.environment==='forest'){
    const size=Math.min(w,h)-60,ox=(w-size)/2,oy=30,P=q=>[ox+(q[0]+5)*size/10,oy+(5-q[1])*size/10];
    X.strokeStyle='#8a95a3';X.strokeRect(ox,oy,size,size);
    (D.inputs.centers||D.inputs.obstacle_centers||[]).forEach(c=>dot(P(c),'#ccc',((D.inputs.scene||D.inputs).obstacle_radius||.35)*size/10));
    D.traces.forEach((t,i)=>{if(!enabled[i])return;let j=sampleAt(t,progress);line(t.q.map(P),t.color,!t.strict_success);dot(P(t.q[j]),t.color);state.push(t.method+': '+t.axis_semantics+' = '+t.axis[j].toFixed(4)+'; saved row '+t.source_rows[j])});
    if(D.inputs.start)dot(P(D.inputs.start),'#d8354f',5);if(D.inputs.goal)dot(P(D.inputs.goal),'#245fbb',5);
  }else if(V.value==='workspace'){
    drawWorkspace(w,h,progress,state);
  }else{
    const pad=35,cw=(w-3*pad)/2,ch=(h-5*pad)/4;
    for(let k=0;k<7;k++){
      const ox=pad+(k%2)*(cw+pad),oy=pad+Math.floor(k/2)*(ch+pad);
      X.strokeStyle='#d3dae2';X.strokeRect(ox,oy,cw,ch);X.fillStyle='#334';X.fillText('q'+(k+1)+' [rad]',ox,oy-7);
      const vals=D.traces.filter((_,i)=>enabled[i]).flatMap(t=>t.q.map(q=>q[k]));let lo=vals.length?Math.min(...vals):-1,hi=vals.length?Math.max(...vals):1;
      if(hi-lo<.01){lo-=.1;hi+=.1}
      D.traces.forEach((t,i)=>{if(!enabled[i])return;const a=t.axis[0],span=t.axis[t.axis.length-1]-a,P=(q,j)=>[ox+(span?(t.axis[j]-a)/span:0)*cw,oy+ch-(q[k]-lo)/(hi-lo)*ch];line(t.q.map(P),t.color,!t.strict_success);const j=sampleAt(t,progress);dot(P(t.q[j],j),t.color);if(k===0)state.push(t.method+': '+t.axis_semantics+' = '+t.axis[j].toFixed(4)+'; saved row '+t.source_rows[j])});
    }
  }document.getElementById('state').textContent=state.join('\n');
}
R.oninput=draw;V.onchange=draw;
document.getElementById('reset').onclick=()=>{yaw=-.87;elevation=.44;zoom=1;draw()};
C.onpointerdown=e=>{dragging=[e.clientX,e.clientY];C.setPointerCapture(e.pointerId)};
C.onpointerup=()=>dragging=null;
C.onpointermove=e=>{if(!dragging||D.environment!=='fr3'||V.value!=='workspace')return;yaw+=(e.clientX-dragging[0])*.01;elevation=Math.max(-1.4,Math.min(1.4,elevation+(e.clientY-dragging[1])*.01));dragging=[e.clientX,e.clientY];draw()};
C.addEventListener('wheel',e=>{if(D.environment==='fr3'&&V.value==='workspace'){e.preventDefault();zoom=Math.max(.25,Math.min(4,zoom*Math.exp(-e.deltaY*.001)));draw()}},{passive:false});
document.getElementById('play').onclick=()=>{playing=!playing;last=null;if(playing&&+R.value>=1)R.value=0;document.getElementById('play').textContent=playing?'Pause':'Play';if(playing)requestAnimationFrame(frame)};
function frame(now){if(!playing)return;if(last!==null)R.value=Math.min(1,+R.value+(now-last)/10000);last=now;draw();if(+R.value>=1){playing=false;document.getElementById('play').textContent='Play'}else requestAnimationFrame(frame)}
window.addEventListener('resize',draw);draw();
</script></html>"""
