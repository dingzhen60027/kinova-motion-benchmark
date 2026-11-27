"""Run the fixed-Dev timing-control pilot or its disjoint blocked-chord remainder.

Run with CUDA_VISIBLE_DEVICES="" via the repository's local/python.sh.
"""

import argparse
import sys
import csv
import json
import os
from pathlib import Path
import time

import numpy as np

if __package__:
    from . import paired as p
else:
    import paired as p

ARMS = (*p.ARMS, "minimum_peak")


def cohorts():
    pilot = p.select()
    used = {c["problem_id"] for c in pilot}
    eligible = [json.loads(line) for line in p.MANIFEST.read_text().splitlines()]
    rest = [
        c
        for c in eligible
        if c["split"] == "dev"
        and c["validation_status"] == "valid_input"
        and c["straight_line_collision"] is True
        and c["problem_id"] not in used
    ]
    assert (
        len(pilot) == 12
        and len(rest) == 35
        and not used.intersection(c["problem_id"] for c in rest)
    )
    return {"pilot": pilot, "dev-remainder": rest}


def deterministic_curve(case):
    a, b = map(np.asarray, (case["q_start"], case["q_goal"]))
    d = np.abs(b - a)
    v = np.array(case["velocity_limits_rad_s"])
    base, _ = p.rest_times(b - a, v, np.array(case["acceleration_limits_rad_s2"]))
    T = float(max(base))
    span = np.where(d > 0, T, 0.0)
    ramp = np.where(d > 0, np.minimum(T / 2, T - d / v), 0.0)
    assert np.all(ramp[d > 0] > 0)
    return p.clock_curve(a, b, T, np.zeros(7), span, ramp), T


def validate_dynamics(sample, case, model, cfg, T):
    tol = cfg["feasibility_tolerance"]
    der = sample["derivatives"]
    return bool(
        abs(sample["duration"] - T) <= tol
        and np.max(
            np.linalg.norm(
                sample["q"][[0, -1]] - np.array([case["q_start"], case["q_goal"]]),
                axis=1,
            )
        )
        <= cfg["endpoint_tolerance_rad"]
        and np.all(sample["position_controls"] >= model.lower - tol)
        and np.all(sample["position_controls"] <= model.upper + tol)
        and np.all(
            der["velocity_per_joint"] <= np.array(case["velocity_limits_rad_s"]) + tol
        )
        and np.all(
            der["acceleration_per_joint"]
            <= np.array(case["acceleration_limits_rad_s2"]) + tol
        )
        and der["endpoint_velocity"] <= tol
        and der["continuity_residual"] <= tol
    )


def deterministic(case, scene, cfg):
    begin = time.perf_counter()
    curve, T = deterministic_curve(case)
    sample = p.sample_candidate(curve, cfg)
    clearance = scene.model.clearance(sample["q"])
    motion_valid = validate_dynamics(sample, case, scene.model, cfg, T)
    success = motion_valid and float(clearance.min()) > scene.model.contact_band_m
    return dict(
        arm="minimum_peak",
        case=case["problem_id"],
        scene=case["scene_id"],
        family=case["family"],
        method_seed=case["paired_seeds"][0]["method_seed"],
        duration_lower_bound_s=T,
        cpu_wall_seconds=time.perf_counter() - begin,
        gradient_evaluations=0,
        drake_samples=len(clearance),
        validated_proposals=int(success),
        motion_failures=int(not motion_valid),
        success=bool(success),
        proposals=[
            dict(
                slot=0,
                source="minimum_peak",
                motion_pass=motion_valid,
                dense_count=len(clearance),
                dense_valid=bool(success),
                minimum_clearance_m=float(clearance.min()),
                candidate_sha256=p.digest(
                    curve["controls"].tobytes() + curve["durations"].tobytes()
                ),
            )
        ],
    )


def audit_geometry(rows, case, scene, cfg):
    a, b = np.array(case["q_start"]), np.array(case["q_goal"])
    base, ramp0 = p.rest_times(
        b - a,
        np.array(case["velocity_limits_rad_s"]),
        np.array(case["acceleration_limits_rad_s2"]),
    )
    T = float(max(base))
    checked = {}
    for record in rows:
        for row in record["proposals"]:
            if not row["dense_valid"] or row["candidate_sha256"] in checked:
                continue
            if record["arm"] == "minimum_peak":
                curve, _ = deterministic_curve(case)
            else:
                span = np.where(base > 0, base + np.array(row["width"]) * (T - base), 0)
                ramp = np.divide(ramp0, base, out=np.zeros(7), where=base > 0) * span
                delay = np.array(row["shift"]) * (T - span)
                curve = p.clock_curve(a, b, T, delay, span, ramp)
            assert row["candidate_sha256"] == p.digest(
                curve["controls"].tobytes() + curve["durations"].tobytes()
            )
            sample = p.sample_candidate(curve, cfg)
            clearance = scene.model.clearance(sample["q"])
            ok = bool(
                np.all(clearance > scene.model.contact_band_m)
                and validate_dynamics(sample, case, scene.model, cfg, T)
            )
            checked[row["candidate_sha256"]] = dict(
                verified=ok,
                samples=len(clearance),
                minimum_clearance_m=float(clearance.min()),
            )
            assert (
                ok
            ), "Independent benchmark Drake clearance and explicit-voxel Drake geometries differ on a claimed solution"
    return checked


def verify_existing(out, cases):
    """Read only. Validate every per-query receipt and rederive the published counts."""
    receipt = json.loads((out / "verification.json").read_text())
    assert (
        receipt["verified"]
        and receipt["independently_rechecked_all_successful_schedules"]
    )
    spec = json.loads((out / "study.json").read_text())
    assert spec["cases"] == [c["problem_id"] for c in cases]
    for f, expected in spec["source_hashes"].items():
        assert p.digest((p.ROOT / f).read_bytes()) == expected, f
    for f, expected in receipt["sha256"].items():
        assert p.digest((out / f).read_bytes()) == expected, f
    totals = json.loads((out / "comparison.json").read_text())["arms"]
    checked = {}
    for a in ARMS:
        rows = [
            json.loads((out / "cases" / c["problem_id"] / (a + ".json")).read_text())
            for c in cases
        ]
        for c, r in zip(cases, rows):
            assert r["case"] == c["problem_id"] and r["arm"] == a
            assert len(r["proposals"]) == spec["budget"][a]
            assert r["success"] == any(q["dense_valid"] for q in r["proposals"])
        observed = dict(
            queries=len(rows),
            successful_queries=sum(r["success"] for r in rows),
            successful_proposals=sum(
                q["dense_valid"] for r in rows for q in r["proposals"]
            ),
            total_proposals=sum(len(r["proposals"]) for r in rows),
        )
        assert all(totals[a][k] == v for k, v in observed.items()), a
        checked[a] = observed
    return dict(
        verification="passed", queries=len(cases), cohort=spec["cohort"], arms=checked
    )


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--cohort", choices=("pilot", "dev-remainder"), default="pilot")
    ap.add_argument("--prepare-only", action="store_true")
    ap.add_argument(
        "--output-dir",
        type=Path,
        default=p.OUT,
        help="Root of the mechanism-control outputs",
    )
    ap.add_argument(
        "--verify-existing", action="store_true", help="Verify saved evidence read-only"
    )
    args = ap.parse_args()
    os.environ["CUDA_VISIBLE_DEVICES"] = ""
    p.OUT = args.output_dir.resolve()
    p.Model.cuda = lambda *a, **k: (_ for _ in ()).throw(
        RuntimeError("GPU calls disabled in CPU mechanism control")
    )
    cases = cohorts()[args.cohort]
    out = p.OUT if args.cohort == "pilot" else p.OUT / "dev-remainder"
    if args.verify_existing:
        print(json.dumps(verify_existing(out, cases), indent=2))
        return
    if (out / "verification.json").is_file():
        print(json.dumps(verify_existing(out, cases), indent=2))
        return
    files = [
        Path(__file__),
        Path(p.__file__),
        p.ROOT / "experiments/joint_timing/witness_clocks.py",
        p.ROOT / "configs/evaluation.json",
        p.ROOT / "benchmarks/generic_motion_v1/runners/phase_search.py",
        p.ROOT / "benchmarks/generic_motion_v1/runners/phase_refinement.py",
        p.MANIFEST,
    ]
    protocol = dict(
        cohort=args.cohort,
        cases=[c["problem_id"] for c in cases],
        arms=list(ARMS),
        papers="Fixed-duration independently synchronized joint profiles, as in Hauser and Ng-Thow-Hing (ICRA 2010). minimum_peak is a 1-proposal analytic comparator.",
        source_hashes={
            str(f.resolve().relative_to(p.ROOT)): p.digest(f.read_bytes())
            for f in files
        },
        budget={a: 25 if a != "minimum_peak" else 1 for a in ARMS},
        primary="Any dense CPU-verified candidate at analytic rest-to-rest lower bound per query, matched settings, replica 0.",
        timing="Gradient calls and whole-arm CPU wall time are reported. minimum_peak checks all dense samples with canonical reference distances. Broad-phase CPU screen uses explicit-voxel SceneGraph; every successful candidate is further audited using original canonical Drake clearance. No CUDA or full E2E solver.",
        scope="dev-remainder excludes the pilot queries; a scene can have multiple queries. Dev has prior baseline exposure. No selection by solver success.",
    )
    out.mkdir(parents=True, exist_ok=True)
    spec = out / "study.json"
    if spec.exists():
        assert (
            json.loads(spec.read_text()) == protocol
        ), "cohort contract/source changed"
    else:
        p.save(spec, protocol)
    if args.prepare_only:
        print(
            json.dumps(
                {
                    "cohort": args.cohort,
                    "queries": len(cases),
                    "proposals": len(cases) * 101,
                    "path": str(spec),
                }
            )
        )
        return
    cfg = p.load_config(p.ROOT / "configs/evaluation.json")
    all_records = []
    for case in cases:
        folder = out / "cases" / case["problem_id"]
        folder.mkdir(exist_ok=True, parents=True)
        pending = [a for a in p.ARMS if not (folder / (a + ".json")).exists()]
        if pending:
            for row in p.run_case(case, pending, cfg):
                p.save(folder / (row["arm"] + ".json"), row)
        scene = p.Scene(case)
        if not (folder / "minimum_peak.json").exists():
            p.save(folder / "minimum_peak.json", deterministic(case, scene, cfg))
        rows = [json.loads((folder / (a + ".json")).read_text()) for a in ARMS]
        audits = audit_geometry(rows, case, scene, cfg)
        p.save(folder / "independent_geometry.json", audits)
        all_records.extend(rows)
        print(
            case["problem_id"],
            {r["arm"]: r["validated_proposals"] for r in rows},
            flush=True,
        )
    totals = {}
    successes = {}
    for a in ARMS:
        rows = [r for r in all_records if r["arm"] == a]
        successes[a] = {r["case"] for r in rows if r["success"]}
        totals[a] = dict(
            queries=len(rows),
            scene_layouts=len({r["scene"] for r in rows}),
            successful_queries=len(successes[a]),
            total_proposals=sum(len(r["proposals"]) for r in rows),
            successful_proposals=sum(r["validated_proposals"] for r in rows),
            motion_bound_failures=sum(r["motion_failures"] for r in rows),
            gradient_calls=sum(r["gradient_evaluations"] for r in rows),
            cpu_compute_wall_s=sum(r["cpu_wall_seconds"] for r in rows),
        )
    comparisons = {
        a: {
            b: {"both": len(s & t), "only_first": len(s - t), "only_second": len(t - s)}
            for b, t in successes.items()
        }
        for a, s in successes.items()
    }
    p.save(
        out / "comparison.json",
        dict(
            cohort=args.cohort,
            arms=totals,
            paired=comparisons,
            shared_layouts_with_pilot=len(
                {c["scene_id"] for c in p.select()} & {c["scene_id"] for c in cases}
            ),
        ),
    )
    columns = [
        "arm",
        "case",
        "scene",
        "family",
        "method_seed",
        "duration_lower_bound_s",
        "success",
        "validated_proposals",
        "motion_failures",
        "gradient_evaluations",
        "drake_samples",
        "cpu_wall_seconds",
    ]
    with (out / "case_results.csv").open("w") as f:
        writer = csv.DictWriter(f, fieldnames=columns, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(all_records)
    evidence = sorted((out / "cases").glob("*/*.json"))
    assert len(evidence) == len(cases) * (len(ARMS) + 1)
    p.save(
        out / "verification.json",
        dict(
            verified=True,
            cohort=args.cohort,
            cases=len(cases),
            independently_rechecked_all_successful_schedules=True,
            sha256={
                str(f.relative_to(out)): p.digest(f.read_bytes())
                for f in [
                    *evidence,
                    out / "study.json",
                    out / "comparison.json",
                    out / "case_results.csv",
                ]
            },
        ),
    )
    print(json.dumps(verify_existing(out, cases), indent=2))


if __name__ == "__main__":
    main()
