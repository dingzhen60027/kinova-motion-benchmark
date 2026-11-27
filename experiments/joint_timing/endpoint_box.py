"""Post-hoc CPU counterfactual for the saved successful fixed-duration Dev excursions."""

import argparse
import json
import os
from collections import defaultdict
from dataclasses import replace
from pathlib import Path

import numpy as np

from . import cohort, paired as p
from .excursion_pilot import OUT as INPUT
from .excursions import Motion


def inspect(curve, case, scene, cfg, duration):
    sample = p.sample_candidate(curve, cfg)
    q = sample["q"]
    clearance = scene.model.clearance(q)
    valid_clearance = clearance > scene.model.contact_band_m
    if not np.isfinite(clearance).all() or not np.array_equal(scene.mask(q), valid_clearance):
        raise ValueError("Canonical clearance and explicit sphere geometry disagree")
    step = float(np.max(np.linalg.norm(np.diff(q, axis=0), axis=1)))
    step_ok = step <= cfg["validation_step_rad"] * (1 + 1e-9) + 1e-12
    limits_ok = cohort.validate_dynamics(sample, case, scene.model, cfg, duration)
    start, goal = np.array(case["q_start"]), np.array(case["q_goal"])
    excess = np.maximum.reduce(
        [np.minimum(start, goal) - q.min(0), q.max(0) - np.maximum(start, goal), np.zeros(7)]
    )
    return dict(
        passed=bool(step_ok and limits_ok and np.all(valid_clearance)),
        dynamics_passed=limits_ok,
        samples=len(q),
        sampled_max_step_rad=step,
        min_sampled_clearance_m=float(clearance.min()),
        collision_samples=int(np.sum(clearance <= 0)),
        contact_band_samples=int(np.sum(np.abs(clearance) <= scene.model.contact_band_m)),
        outside_endpoint_box_rad=excess.tolist(),
        derivatives={
            k: v.tolist() if isinstance(v, np.ndarray) else v
            for k, v in sample["derivatives"].items()
        },
        sha256=p.digest(curve["controls"].tobytes() + curve["durations"].tobytes()),
    )


def analyze_case(case, directory, cfg):
    record_file = directory / "excursion.json"
    record = json.loads(record_file.read_text())
    valid = [row for row in record["proposals"] if row["dense_valid"]]
    if len(record["proposals"]) != 25 or record["case"] != case["problem_id"]:
        raise ValueError("Incomplete or mismatched source case")
    result = dict(
        case=record["case"],
        scene=record["scene"],
        family=record["family"],
        record_sha256=p.digest(record_file.read_bytes()),
        trajectories=[],
    )
    if not valid:
        return result
    start, goal = np.array(case["q_start"]), np.array(case["q_goal"])
    scene = p.Scene(case)
    velocity = np.array(case["velocity_limits_rad_s"])
    base, ramp0 = p.rest_times(goal - start, velocity, np.array(case["acceleration_limits_rad_s2"]))
    T, t = record["duration_lower_bound_s"], record["via_time_s"]
    if abs(float(max(base)) - T) > 1e-12:
        raise ValueError("Saved duration disagrees with lower bound")
    result.update(duration_s=T, via_time_s=t)
    with np.load(directory / "trajectories.npz", allow_pickle=False) as blobs:
        for row in valid:
            k = row["slot"]
            original = dict(
                kind="bezier",
                objective="minimum_time",
                controls=blobs[f"slot{k}_controls"],
                durations=blobs[f"slot{k}_durations"],
            )
            before = inspect(original, case, scene, cfg, T)
            if (
                not before["passed"]
                or before["sha256"] != row["candidate_sha256"]
                or before["samples"] != row["reference_count"]
                or abs(before["min_sampled_clearance_m"] - row["min_verified_clearance_m"]) > 1e-12
            ):
                raise ValueError(
                    f"Original evidence does not reproduce for {result['case']} slot {k}"
                )
            active = np.isin(np.arange(7), row["active"])
            via = np.array(row["via_q_rad"])
            clipped = np.where(
                active, np.clip(via, np.minimum(start, goal), np.maximum(start, goal)), via
            )
            span = np.where(base > 0, base + np.array(row["background_width"]) * (T - base), 0.0)
            ramp = np.divide(ramp0, base, out=np.zeros(7), where=base > 0) * span
            delay = np.array(row["background_shift"]) * (T - span)
            motion = Motion(start, goal, T, t, via, active, velocity, delay, span, ramp)
            rebuilt = motion.curve()
            if not all(
                np.array_equal(rebuilt[key], original[key]) for key in ("controls", "durations")
            ):
                raise ValueError(
                    f"Original trajectory parameters do not reconstruct slot {k} exactly"
                )
            after = (
                before
                if np.array_equal(via, clipped)
                else inspect(replace(motion, via=clipped).curve(), case, scene, cfg, T)
            )
            if max(after["outside_endpoint_box_rad"]) > 1e-7:
                raise ValueError("Clipped trajectory leaves the endpoint box")
            result["trajectories"].append(
                dict(
                    slot=k,
                    active=row["active"],
                    via_q_rad=via.tolist(),
                    clipped_via_q_rad=clipped.tolist(),
                    original=before,
                    clipped=after,
                )
            )
    return result


def summary(cases):
    trajectories = [t for c in cases for t in c["trajectories"]]
    return dict(
        queries=len(cases),
        layouts=len({c["scene"] for c in cases}),
        queries_with_original_witness=sum(bool(c["trajectories"]) for c in cases),
        queries_retaining_a_witness=sum(
            any(t["clipped"]["passed"] for t in c["trajectories"]) for c in cases
        ),
        layouts_with_original_witness=len({c["scene"] for c in cases if c["trajectories"]}),
        layouts_retaining_a_witness=len(
            {c["scene"] for c in cases if any(t["clipped"]["passed"] for t in c["trajectories"])}
        ),
        original_witnesses=len(trajectories),
        witnesses_outside_box=sum(
            max(t["original"]["outside_endpoint_box_rad"]) > 1e-7 for t in trajectories
        ),
        clipped_passing=sum(t["clipped"]["passed"] for t in trajectories),
        clipped_with_collision=sum(t["clipped"]["collision_samples"] > 0 for t in trajectories),
        clipped_failing_motion_limits=sum(
            not t["clipped"]["dynamics_passed"] for t in trajectories
        ),
    )


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--cohort", choices=["pilot", "all"], default="pilot")
    ap.add_argument("--input", type=Path, default=INPUT)
    ap.add_argument(
        "--output", type=Path, default=p.ROOT / "runs/phase_search/paper_design/endpoint_box.json"
    )
    args = ap.parse_args()
    if os.environ.get("CUDA_VISIBLE_DEVICES") != "":
        raise ValueError("Set CUDA_VISIBLE_DEVICES to an empty string for this CPU diagnostic")
    if (
        args.output.exists()
        and args.cohort == "pilot"
        and json.loads(args.output.read_text())["cohort"] == "all"
    ):
        raise ValueError("Choose a separate --output for a pilot-only rerun")

    def disabled(*args, **kwargs):
        raise RuntimeError("GPU disabled in this CPU study")

    p.Model.cuda = disabled
    cfg = p.load_config(p.ROOT / "configs/evaluation.json")
    subsets = cohort.cohorts()
    chosen = list(subsets) if args.cohort == "all" else ["pilot"]
    cases, by_cohort, hashes = [], {}, {}
    for name in chosen:
        folder = args.input if name == "pilot" else args.input / name
        spec, comp = folder / "protocol.json", folder / "comparison.json"
        hashes[name] = {path.name: p.digest(path.read_bytes()) for path in (spec, comp)}
        contract = json.loads(spec.read_text())
        for file, checksum in contract["source_sha256"].items():
            if p.digest((p.ROOT / file).read_bytes()) != checksum:
                raise ValueError(f"Source mismatch for {file}")
        if [c["problem_id"] for c in subsets[name]] != contract["cases"]:
            raise ValueError(f"Frozen cases differ from {name} cohort")
        batch = [analyze_case(c, folder / "cases" / c["problem_id"], cfg) for c in subsets[name]]
        by_cohort[name] = summary(batch)
        if (
            by_cohort[name]["original_witnesses"]
            != json.loads(comp.read_text())["arms"]["excursion"]["passing_proposals"]
        ):
            raise ValueError("Inputs disagree with original counts")
        cases.extend(batch)
    groups = defaultdict(list)
    for c in cases:
        groups[c["scene"]].append(c)
    result = dict(
        cohort=args.cohort,
        counterfactual="Coordinatewise clip only the active via joint coordinates to the closed endpoint box; preserve active set, time, endpoints, inactive clocks and rest-to-rest construction.",
        population="Post-hoc analysis of all saved CPU-validated excursion successes among blocked straight-line Dev queries, replicate zero. Selection is conditioned on success. A failed clipped candidate is evidence for that particular pair; alternative trajectories inside the box remain possible.",
        validation="Frozen Bezier sampler, motion bounds, maximum step, canonical signed clearance, explicit voxel spheres; finite sampling on CPU.",
        source_sha256=p.digest(Path(__file__).read_bytes()),
        input_hashes=hashes,
        overall=summary(cases),
        by_cohort=by_cohort,
        by_layout={scene: summary(rows) for scene, rows in sorted(groups.items())},
        cases=cases,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    p.save(args.output, result)
    print(json.dumps(result["overall"], indent=2))


if __name__ == "__main__":
    main()
