"""Replay the observed Drake BoxBoxGradient assertion with one CPU pair query.

This file preserves the single-pair operation executed in the audit's inline
Python command. It is a backend diagnostic, not a benchmark solve. With the
audited environment, Drake terminates the process with exit code 1.
"""
from pathlib import Path
import json
import numpy as np

from benchmarks.generic_motion_v1.model import Model


def main():
    base = Path(__file__).resolve().parents[3]
    attempt = base / "results/rss2025_full_dev_v1/jobs/dev-route_choice-002-p00-NTO_PAPER-minimum_distance-48b608ce/attempts/0001"
    case = json.loads((attempt / "input.json").read_text())
    q = np.load(Path(__file__).with_name("last_evaluated_q.npy"))
    model = Model(case["voxels_xyz"], case["voxel_radius_m"])
    native = model.nto_plant()
    native.plant.SetPositions(native.plant_context, q)
    query = native.scene_graph.get_query_output_port().Eval(native.scene_context)
    wanted = {"robotiq::left_inner_finger_collision", "table::Box"}
    pairs = [(a, b) for a, b in native.inspector.GetCollisionCandidates()
             if {native.inspector.GetName(a), native.inspector.GetName(b)} == wanted]
    assert len(pairs) == 1
    print(json.dumps(dict(operation="single CPU double signed-distance query",
                          geometry_pair=sorted(wanted), q=q.tolist())), flush=True)
    distance = query.ComputeSignedDistancePairClosestPoints(*pairs[0])
    print("Query returned instead of reproducing the recorded assertion:", distance.distance)


if __name__ == "__main__":
    main()
