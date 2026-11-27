"""Independently validate the saved, CPU-accepted Dev excursion witnesses."""

import argparse
from collections import Counter
import fcntl
import json
from pathlib import Path
import tempfile

import numpy as np

from benchmarks.generic_motion_v1.common import BASE, ROOT, read, sha, write
from benchmarks.generic_motion_v1.model import Model
from benchmarks.generic_motion_v1.roadmap import load_config
from benchmarks.generic_motion_v1.runners.validate import sample_candidate, validate

from . import cohort
from .paired import digest

PILOT = ROOT / "runs/phase_search/excursion_pilot"
OUTPUT = PILOT / "numerical_validation"
FORMAL = ROOT / "runs/phase_search/test/run_manifest.json"
CONFIG = ROOT / "configs/evaluation.json"


def require(condition, message):
    if not condition:
        raise ValueError(message)


def inventory(groups):
    manifest = read(FORMAL)
    cfg = load_config(CONFIG)
    for name in ("model.py", "roadmap.py", "runners/validate.py"):
        path = BASE / name
        require(
            sha(path) == manifest["source_hashes"][str(path.relative_to(ROOT))],
            str(path),
        )
    require(
        all(job["evaluation_config"] == cfg for job in manifest["jobs"]),
        "Formal validation configuration differs",
    )
    cases_by_group = cohort.cohorts()
    candidates = []
    counts = {}
    for name in groups:
        folder = PILOT if name == "pilot" else PILOT / name
        protocol = read(folder / "protocol.json")
        for filename, checksum in protocol["source_sha256"].items():
            require(
                sha(ROOT / filename) == checksum,
                f"Changed pilot source or input: {filename}",
            )
        summary = read(folder / "comparison.json")
        cases = cases_by_group[name]
        require(
            protocol["cases"] == [c["problem_id"] for c in cases],
            "Different frozen cohort",
        )
        observed = 0
        for c in cases:
            pid = c["problem_id"]
            inp = folder / "cases" / pid
            rec = read(inp / "excursion.json")
            require(
                (rec["case"], rec["scene"], rec["arm"], rec["method_seed"])
                == (
                    pid,
                    c["scene_id"],
                    "excursion",
                    c["paired_seeds"][0]["method_seed"],
                ),
                str(inp),
            )
            rows = rec["proposals"]
            require(
                [r["slot"] for r in rows] == list(range(protocol["proposals"])),
                f"Incomplete proposals in {pid}",
            )
            accepted = [r for r in rows if r["dense_valid"]]
            archive = inp / "trajectories.npz"
            if not accepted:
                require(not archive.exists(), f"Unclaimed trajectory archive in {pid}")
                continue
            dq = np.abs(np.subtract(c["q_goal"], c["q_start"]))
            v = np.array(c["velocity_limits_rad_s"])
            a = np.array(c["acceleration_limits_rad_s2"])
            lower = max(np.where(dq <= v * v / a, 2 * np.sqrt(dq / a), dq / v + v / a))
            require(
                abs(lower - rec["duration_lower_bound_s"]) <= 1e-8,
                "Inconsistent physical bound",
            )
            with np.load(archive, allow_pickle=False) as arrays:
                require(
                    set(arrays.files)
                    == {
                        f"slot{r['slot']}_{key}"
                        for r in accepted
                        for key in ("controls", "durations")
                    },
                    f"Archive slots do not match the accepted witnesses in {pid}",
                )
                for r in accepted:
                    slot = r["slot"]
                    curve = dict(
                        kind="bezier",
                        objective="minimum_time",
                        controls=arrays[f"slot{slot}_controls"],
                        durations=arrays[f"slot{slot}_durations"],
                    )
                    actual = digest(
                        curve["controls"].tobytes() + curve["durations"].tobytes()
                    )
                    require(
                        actual == r["candidate_sha256"]
                        and r["motion_pass"]
                        and r["coarse_free"],
                        f"Candidate identity mismatch {pid}/{slot}",
                    )
                    sample = sample_candidate(curve, cfg)
                    require(
                        abs(sample["duration"] - lower) <= 1e-8
                        and len(sample["q"])
                        == r["dense_count"]
                        == r["reference_count"],
                        f"Incorrect frozen sampling or duration {pid}/{slot}",
                    )
                    der = sample["derivatives"]
                    tol = cfg["feasibility_tolerance"]
                    require(
                        np.allclose(
                            sample["q"][[0, -1]],
                            [c["q_start"], c["q_goal"]],
                            atol=cfg["endpoint_tolerance_rad"],
                            rtol=0,
                        )
                        and np.all(der["velocity_per_joint"] <= v + tol)
                        and np.all(der["acceleration_per_joint"] <= a + tol)
                        and der["endpoint_velocity"] <= tol
                        and der["continuity_residual"] <= tol
                        and np.linalg.norm(np.diff(sample["q"], axis=0), axis=1).max()
                        <= cfg["validation_step_rad"] * (1.0 + 1e-9) + 1e-12,
                        f"Offline dynamics/step discrepancy: {pid}/{slot}",
                    )
                    identity = dict(
                        cohort=name,
                        problem_id=pid,
                        case_sha256=c["problem_sha256"],
                        slot=slot,
                        candidate_sha256=actual,
                        physical_lower_bound_s=float(lower),
                        duration_s=sample["duration"],
                        protocol_sha256=sha(folder / "protocol.json"),
                        validator_sha256=sha(BASE / "runners/validate.py"),
                        model_source_sha256=sha(BASE / "model.py"),
                        config_sha256=digest(
                            json.dumps(
                                cfg, sort_keys=True, separators=(",", ":")
                            ).encode()
                        ),
                    )
                    candidates.append((c, curve, identity))
                    observed += 1
        require(
            observed == summary["arms"]["excursion"]["passing_proposals"],
            f"Summary mismatch for {name}",
        )
        counts[name] = dict(cases=len(cases), cpu_accepted_candidates=observed)
    return candidates, cfg, counts


def target(identity):
    return OUTPUT / identity["cohort"] / identity["problem_id"] / str(identity["slot"])


def existing(directory, identity):
    record = read(directory / "receipt.json")
    require(
        record["identity"] == identity,
        f"Stored receipt belongs to another input: {directory}",
    )
    for filename, expected in record["files"].items():
        require(
            sha(directory / filename) == expected,
            f"Stored evidence changed: {directory/filename}",
        )
    return record


def verify_all(groups):
    lock = BASE / "results/.gpu.lock"
    print(f"Waiting for the serial benchmark GPU lock {lock}", flush=True)
    with lock.open("r+") as handle:
        fcntl.flock(handle, fcntl.LOCK_EX)
        candidates, cfg, counts = inventory(groups)
        current_case = None
        outcomes = Counter()
        for c, curve, key in candidates:
            dest = target(key)
            if not dest.exists():
                dest.parent.mkdir(parents=True, exist_ok=True)
                with tempfile.TemporaryDirectory(
                    dir=dest.parent, prefix=".witness-"
                ) as tmp:
                    folder = Path(tmp) / "result"
                    folder.mkdir()
                    try:
                        if current_case != c["problem_id"]:
                            model = Model(c["voxels_xyz"], c["voxel_radius_m"])
                            require(
                                (model.robot_sha256, model.collision_geometry_sha256)
                                == (c["robot_sha256"], c["collision_geometry_sha256"]),
                                "Frozen robot differs",
                            )
                            current_case = c["problem_id"]
                        result = validate(curve, model, c, cfg, out=folder)
                        status = result["status"]
                    except Exception as error:
                        import traceback

                        write(
                            folder / "error.json",
                            dict(
                                exception=repr(error), traceback=traceback.format_exc()
                            ),
                        )
                        status = "infrastructure_error"
                    files = {p.name: sha(p) for p in folder.iterdir() if p.is_file()}
                    write(
                        folder / "receipt.json",
                        dict(identity=key, status=status, files=files),
                    )
                    folder.rename(dest)
            record = existing(dest, key)
            outcomes[record["status"]] += 1
            print(key["problem_id"], key["slot"], record["status"], flush=True)
            if record["status"] == "infrastructure_error":
                raise RuntimeError(f"Infrastructure failure preserved in {dest}")
        print(
            json.dumps(dict(cohorts=counts, formal_validation_counts=outcomes)),
            flush=True,
        )


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("action", choices=["dry-run", "verify", "status"])
    ap.add_argument(
        "--cohort", choices=["pilot", "dev-remainder", "all"], default="all"
    )
    args = ap.parse_args()
    groups = ["pilot", "dev-remainder"] if args.cohort == "all" else [args.cohort]
    candidates, _, counts = inventory(groups)
    print(
        json.dumps(dict(cohorts=counts, hashes_and_kinematics_checked=len(candidates))),
        flush=True,
    )
    if args.action == "status":
        outcomes = Counter()
        for _, _, key in candidates:
            dest = target(key)
            outcomes[existing(dest, key)["status"] if dest.exists() else "pending"] += 1
        print(json.dumps(dict(formal_validation_counts=outcomes)), flush=True)
    if args.action == "verify":
        verify_all(groups)


if __name__ == "__main__":
    main()
