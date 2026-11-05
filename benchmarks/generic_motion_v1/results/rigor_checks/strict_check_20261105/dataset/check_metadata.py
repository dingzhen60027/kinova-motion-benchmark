"""Semantic checks beyond saved hashes: splits, counts, TCP and joint policy."""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path
import subprocess
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[6]
sys.path.insert(0, str(ROOT))
from benchmarks.generic_motion_v1 import dataset
from benchmarks.generic_motion_v1.reports.qualification.model_audit import ManualURDF

BASE = ROOT / "benchmarks/generic_motion_v1"


def check(output):
    output = Path(output)
    if output.exists():
        raise FileExistsError(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    original = [row for row, _, _ in dataset.source_records(BASE)]
    index = dataset.load_index(BASE)
    metadata = dataset.read(BASE / "metadata.json")
    splits = dataset.read(BASE / "splits.json")
    manual = ManualURDF()
    names = [joint["name"] + "_q" for joint in manual.active]
    robot = dataset.read(BASE / "configs/robot.json")
    assert names == robot["joint_names"]
    assert metadata["model"]["periodic_joints"] == [False] * 7
    assert metadata["model"]["joint_types"] == ["revolute"] * 7
    assert np.allclose([joint["upper"] for joint in manual.active], robot["upper"], rtol=0, atol=2e-7)
    assert np.allclose([joint["lower"] for joint in manual.active], robot["lower"], rtol=0, atol=2e-7)
    features = {r["problem_id"]: r for r in index}
    assert len(features) == len(original)
    report = dict(passed=True, git_commit=subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
                  script_sha256=dataset.sha(__file__), counts={}, shared_random_geometry=[],
                  shared_random_geometry_across_splits=[], shared_individual_endpoints=[],
                  maximum_manual_xml_tcp_error_m=0., endpoint_tcp_count=0,
                  all_joints_bounded_revolute=True, cuda_calls=0, optimizer_calls=0)
    geom, endpoints = defaultdict(set), defaultdict(set)
    for row in original:
        item = features[row["problem_id"]]
        assert item["environment_cluster"] == "geometry:" + row["obstacle_geometry_sha256"]
        if row["family"] != "open_control":
            geom[row["obstacle_geometry_sha256"]].add((row["split"], row["scene_id"]))
        for key in ("q_start", "q_goal"):
            if row[key] is not None:
                endpoints[dataset.digest(row[key])].add((row["split"], row["problem_id"], key))
        if row["validation_status"] == "valid_input":
            tcp = np.asarray([manual.fk(row[key])[robot["tcp"]][:3, 3] for key in ("q_start", "q_goal")])
            err = float(np.max(np.abs(tcp - row["endpoint_tcp_world_m"])))
            assert err < 2e-12, row["problem_id"]
            report["maximum_manual_xml_tcp_error_m"] = max(report["maximum_manual_xml_tcp_error_m"], err)
            report["endpoint_tcp_count"] += 2
    report["shared_random_geometry"] = [sorted(value) for value in geom.values() if len(value) > 1]
    report["shared_random_geometry_across_splits"] = [sorted(value) for value in geom.values() if len({x[0] for x in value}) > 1]
    report["shared_individual_endpoints"] = [sorted(value) for value in endpoints.values() if len(value) > 1]
    assert not report["shared_random_geometry"] and not report["shared_individual_endpoints"]
    for split in dataset.SPLITS:
        rows = [r for r in original if r["split"] == split]
        good = [r for r in rows if r["validation_status"] == "valid_input"]
        bad = [r for r in rows if r["validation_status"] != "valid_input"]
        expected = dict(problem_ids=[r["problem_id"] for r in rows],
                        valid_problem_ids=[r["problem_id"] for r in good],
                        generation_failed_problem_ids=[r["problem_id"] for r in bad])
        assert splits[split] == expected, split
        scenes = {r["scene_id"] for r in rows}
        groups = {r["obstacle_geometry_sha256"] for r in good}
        recorded = metadata["counts"][split]
        actual = dict(registered=len(rows), valid=len(good), generation_failed=len(bad), scene_ids=len(scenes),
                      valid_environment_clusters=len(groups),
                      category_valid=dict(Counter(r["family"] for r in good)),
                      distance_bin_valid=dict(Counter(r["endpoint_sampling"]["distance_bin"] for r in good)),
                      straight_line_collision_count=sum(r["straight_line_collision"] is True for r in good))
        assert all(recorded[key] == value for key, value in actual.items()), split
        actual["generation_failure_reasons"] = dict(Counter(
            "no_legal_start" if r["q_start"] is None else "no_legal_goal" if r["q_goal"] is None else r["validation_status"] for r in bad))
        actual["random_valid_layouts"] = len({r["obstacle_geometry_sha256"] for r in good if r["family"] != "open_control"})
        actual["distance_rad_min_median_max"] = np.quantile([r["start_goal_joint_distance"] for r in good], [0, .5, 1]).tolist()
        actual["categories"] = {name: dict(registered=sum(r["family"] == name for r in rows),
                                         valid=sum(r["family"] == name for r in good),
                                         straight_line_colliding=sum(r["family"] == name and r["straight_line_collision"] is True for r in good))
                                for name in sorted({r["family"] for r in rows})}
        report["counts"][split] = actual
    output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(json.dumps(report, indent=2, allow_nan=False))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    check(parser.parse_args().output)
