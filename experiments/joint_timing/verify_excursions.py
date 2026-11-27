"""Independently reconstruct each Dev excursion and replay its numerical checks."""

import argparse
import json
import os
import subprocess
from pathlib import Path

import numpy as np

from . import cohort
from . import paired as p
from .excursion_pilot import ARMS, OUT, first_hit
from .excursions import Motion


def verify(folder, control, cases):
    contract = json.loads((folder / "protocol.json").read_text())
    summary = json.loads((folder / "comparison.json").read_text())
    assert contract["cases"] == [c["problem_id"] for c in cases]
    head = subprocess.check_output(
        ["git", "-C", str(p.ROOT), "rev-parse", "HEAD"], text=True
    ).strip()
    for path, checksum in contract["source_sha256"].items():
        assert p.digest((p.ROOT / path).read_bytes()) == checksum, path
        if not path.startswith("experiments/"):
            tracked = subprocess.check_output(
                ["git", "-C", str(p.ROOT), "show", f"{head}:{path}"]
            )
            assert p.digest(tracked) == checksum, path
    cfg = p.load_config(p.ROOT / "configs/evaluation.json")
    source_receipt = cohort.verify_existing(control, cases)
    audit = dict(
        cases=len(cases),
        candidates=0,
        canonical_coarse_positions=0,
        canonical_dense_positions=0,
        motion_failures=0,
        verified_candidates=0,
        verified_trajectory_blobs=0,
    )
    sha = {}
    for case, entry in zip(cases, summary["per_case"]):
        pid = case["problem_id"]
        assert entry["case"] == pid
        folder_case = folder / "cases" / pid
        path = folder_case / "excursion.json"
        sha[str(path.relative_to(folder))] = p.digest(path.read_bytes())
        exp = json.loads(path.read_text())
        scene = p.Scene(case)
        assert case["paired_seeds"][0]["method_seed"] == exp["method_seed"]
        start, goal = map(np.asarray, (case["q_start"], case["q_goal"]))
        v, a = map(
            np.asarray,
            (case["velocity_limits_rad_s"], case["acceleration_limits_rad_s2"]),
        )
        base, ramp = p.rest_times(goal - start, v, a)
        T = float(max(base))
        assert exp["duration_lower_bound_s"] == T
        found = []
        saved = folder_case / "trajectories.npz"
        blobs = {}
        if saved.is_file():
            with np.load(saved) as archive:
                blobs = {k: archive[k] for k in archive.files}
            sha[str(saved.relative_to(folder))] = p.digest(saved.read_bytes())
        for slot, row in enumerate(exp["proposals"]):
            assert slot == row["slot"]
            active = np.zeros(7, dtype=bool)
            active[row["active"]] = True
            span = np.where(
                base > 0, base + np.asarray(row["background_width"]) * (T - base), 0.0
            )
            delay = np.asarray(row["background_shift"]) * (T - span)
            m = Motion(
                start,
                goal,
                T,
                exp["via_time_s"],
                np.asarray(row["via_q_rad"]),
                active,
                v,
                delay,
                span,
                np.divide(ramp, base, out=np.zeros(7), where=base > 0) * span,
            )
            curve = m.curve()
            assert (
                p.digest(curve["controls"].tobytes() + curve["durations"].tobytes())
                == row["candidate_sha256"]
            )
            sample = p.sample_candidate(curve, cfg)
            motion = cohort.validate_dynamics(sample, case, scene.model, cfg, T)
            assert motion == row["motion_pass"]
            coarse = (
                scene.model.clearance(m.state(p.U * T)[0]) > scene.model.contact_band_m
            )
            assert bool(coarse.all()) == row["coarse_free"]
            valid = False
            if coarse.all():
                clearances = scene.model.clearance(sample["q"])
                valid = bool(motion and min(clearances) > scene.model.contact_band_m)
                assert row["dense_count"] == len(clearances)
                audit["canonical_dense_positions"] += len(clearances)
            assert valid == row["dense_valid"]
            if valid:
                for field in ("controls", "durations"):
                    assert np.array_equal(
                        curve[field], blobs.pop(f"slot{slot}_{field}")
                    )
                audit["verified_trajectory_blobs"] += 1
                outside = np.maximum.reduce(
                    [
                        np.minimum(start, goal) - sample["q"].min(axis=0),
                        sample["q"].max(axis=0) - np.maximum(start, goal),
                        np.zeros(7),
                    ]
                )
                np.testing.assert_allclose(
                    outside, row["outside_endpoint_range_rad"], atol=1e-12
                )
            found.append(dict(dense_valid=valid))
            audit["canonical_coarse_positions"] += len(coarse)
            audit["motion_failures"] += int(not motion)
            audit["verified_candidates"] += int(valid)
        assert not blobs and len(found) == 25
        audit["candidates"] += len(found)
        arms = {"excursion": found}
        for key in ("guided", "uniform_shift"):
            arms[key] = json.loads(
                (control / "cases" / pid / (key + ".json")).read_text()
            )["proposals"]
        arms["alternating"] = [
            (arms["guided"] if i % 2 == 0 else found)[i // 2] for i in range(25)
        ]
        assert entry["first_hit"] == {key: first_hit(arms[key]) for key in ARMS}
        assert entry["num_passing"] == {
            key: sum(row["dense_valid"] for row in arms[key]) for key in ARMS
        }
    for arm, row in summary["arms"].items():
        assert row["successful_cases"] == sum(
            x["first_hit"][arm] is not None for x in summary["per_case"]
        )
        assert row["passing_proposals"] == sum(
            x["num_passing"][arm] for x in summary["per_case"]
        )
    for path in ["protocol.json", "comparison.json"]:
        sha[path] = p.digest((folder / path).read_bytes())
    report = dict(
        verified=True,
        audit=audit,
        git_head=head,
        source_sha256=contract["source_sha256"],
        artifact_sha256=sha,
        control_receipt=source_receipt,
        verification_source_sha256=p.digest(Path(__file__).read_bytes()),
    )
    p.save(folder / "verification.json", report)
    print(json.dumps(audit))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--cohort", choices=["pilot", "dev-remainder"], default="pilot")
    args = ap.parse_args()
    os.environ["CUDA_VISIBLE_DEVICES"] = ""

    def forbidden(*args, **kwargs):
        raise RuntimeError("GPU disabled in the independent Dev replay")

    p.Model.cuda = forbidden
    name = "" if args.cohort == "pilot" else args.cohort
    verify(OUT / name, p.OUT / name, cohort.cohorts()[args.cohort])


if __name__ == "__main__":
    main()
