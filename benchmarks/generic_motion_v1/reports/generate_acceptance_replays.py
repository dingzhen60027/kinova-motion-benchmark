"""Render frozen acceptance selections without writing inside sealed attempts."""
import hashlib
import json
from pathlib import Path
import shlex
import subprocess

import numpy as np

from benchmarks.generic_motion_v1.batch import verify

BASE = Path(__file__).resolve().parents[1]
ROOT = BASE.parents[1]


def read(path):
    return json.loads(Path(path).read_text())


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    paired = BASE / "results/dev_paired_v2"
    smoke = BASE / "results/smoke_v3_fixed"
    selected = [dict(row, run="dev_paired_v2")
                for row in read(BASE / "reports/paired_baseline_audit.json")["replay_selection"]]
    assert len(selected) == 10
    seen = set()
    for job in read(smoke / "run_manifest.json")["jobs"]:
        attempt = smoke / "jobs" / job["job_id"] / "attempts/0001"
        result = read(attempt / "result.json")
        if result["status"] == "success_validated_numerical" and job["objective"] not in seen:
            seen.add(job["objective"])
            selected.append(dict(job_id=job["job_id"], status=result["status"],
                attempt=str(attempt.relative_to(BASE)), run="smoke_v3_fixed",
                reason="First sealed success in frozen manifest order for each objective"))
    assert seen == {"minimum_distance", "minimum_time"}
    before = {run.name: verify(run) for run in (paired, smoke)}
    assert all(v["complete"] and v["integrity_passed"] for v in before.values())
    receipts = {str(path.relative_to(BASE)): sha(path)
                for run in (paired, smoke) for path in run.glob("jobs/*/attempts/*/receipt.json")}
    record = dict(selection=selected, runs_before=before, outputs=[],
                  model_use="CPU forward kinematics only; replay does not call CUDA collision or planning",
                  visual_inspection="pending", complete=False)
    acceptance = BASE / "reports/replay_acceptance.json"
    for row in selected:
        attempt = BASE / row["attempt"]
        output = BASE / "results" / row["run"] / "replays" / row["job_id"]
        output.mkdir(parents=True, exist_ok=True)
        argv = ["bash", "local/python.sh", "-m", "benchmarks.generic_motion_v1", "replay",
                "--attempt", str(attempt.relative_to(ROOT)), "--output", str(output.relative_to(ROOT))]
        with (output / "generation.stdout.log").open("w") as out, (output / "generation.stderr.log").open("w") as err:
            process = subprocess.run(argv, cwd=ROOT, stdout=out, stderr=err)
        entry = dict(job_id=row["job_id"], command=shlex.join(argv), return_code=process.returncode,
                     output=str(output.relative_to(BASE)), validation_passed=False)
        record["outputs"].append(entry)
        acceptance.write_text(json.dumps(record, indent=2) + "\n")
        assert process.returncode == 0, entry
        data = read(output / "replay.json")
        assert data["status"] == row["status"]
        assert len(data["fixed_boxes"]) == 5
        assert all(len(frame["moving_boxes"]) == 15 for frame in data["frames"])
        assert data["voxel_radius_m"] == .04
        html = (output / "replay.html").read_text()
        assert "<script src=" not in html and "http://" not in html and "https://" not in html
        for name in ("scene.png", "joints.png"):
            assert (output / name).stat().st_size > 10000
        if row["status"] == "success_validated_numerical":
            source = attempt / data["sample_source"]
            assert data["has_trajectory"] and sha(source) == data["sample_source_sha256"]
            with np.load(source) as samples:
                assert data["original_sample_count"] == len(samples["q"])
                for frame in data["frames"]:
                    i = frame["sample_index"]
                    np.testing.assert_array_equal(frame["q"], samples["q"][i])
                    assert frame["time"] == samples["time"][i]
                    assert frame["clearance_m"] == samples["clearance"][i]
        else:
            assert not data["has_trajectory"] and data["sample_source"] is None
            assert data["original_sample_count"] == data["displayed_frames"] == 2
        entry.update(validation_passed=True, status=data["status"],
            has_trajectory=data["has_trajectory"], original_sample_count=data["original_sample_count"],
            displayed_frames=data["displayed_frames"], time_axis=data["time_axis"],
            files_sha256={name: sha(output / name) for name in ("replay.html", "replay.json", "scene.png", "joints.png")})
        acceptance.write_text(json.dumps(record, indent=2) + "\n")
        print(json.dumps(dict(job_id=row["job_id"], validation_passed=True,
                              status=data["status"], samples=data["original_sample_count"])), flush=True)
    after = {run.name: verify(run) for run in (paired, smoke)}
    assert after == before
    assert receipts == {str(path.relative_to(BASE)): sha(path)
                        for run in (paired, smoke) for path in run.glob("jobs/*/attempts/*/receipt.json")}
    record.update(complete=True, runs_after=after, sealed_receipts_unchanged=True)
    acceptance.write_text(json.dumps(record, indent=2) + "\n")


if __name__ == "__main__":
    main()
