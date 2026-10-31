"""Deterministically fit 33 spheres to the pinned official FR3 collision model.

This is a new approximation, not the unreleased spheres used by the authors.
No experiment outcome is used to select its centers, radii or allocation.
"""

from pathlib import Path
import hashlib
import itertools
import shutil
import xml.etree.ElementTree as ET
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from reproduction.common import ROOT, atomic

SOURCE = ROOT / "reproduction/vendor/franka_description"
ASSETS = Path(__file__).parent / "assets"
REVISION = "7aeeddc449edf8d62b594f9e36a81da53e7796f9"


def stl(path):
    raw = path.read_bytes()
    count = int.from_bytes(raw[80:84], "little")
    if len(raw) != 84 + count * 50:
        raise ValueError("Expected binary STL: " + str(path))
    dtype = np.dtype(
        [("normal", "<f4", 3), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")]
    )
    return np.frombuffer(raw, dtype=dtype, offset=84)["vertices"].astype(float)


def fit(points, count):
    points = np.unique(points, axis=0)
    centers = [points[np.argmax(np.linalg.norm(points - points.mean(0), axis=1))]]
    for _ in range(1, count):
        distances = np.linalg.norm(points[:, None] - np.array(centers)[None], axis=2)
        centers.append(points[np.argmax(distances.min(1))])
    centers = np.array(centers)
    for _ in range(100):
        labels = np.linalg.norm(points[:, None] - centers[None], axis=2).argmin(1)
        updated = np.array([points[labels == i].mean(0) for i in range(count)])
        if np.max(abs(updated - centers)) < 1e-12:
            break
        centers = updated
    radii = (
        np.array(
            [
                np.linalg.norm(points[labels == i] - centers[i], axis=1).max()
                for i in range(count)
            ]
        )
        + 0.0025
    )
    return [dict(center=c, radius=r) for c, r in zip(centers, radii)]


def finger_points(right=False):
    boxes = [
        ([0, 0.0185, 0.011], [0.022, 0.015, 0.020], [0, 0, 0]),
        ([0, 0.0068, 0.0022], [0.022, 0.0088, 0.0038], [0, 0, 0]),
        (
            [0, 0.0159, 0.02835],
            [0.0175, 0.007, 0.0235],
            [-np.pi / 6, 0, np.pi] if right else [np.pi / 6, 0, 0],
        ),
        ([0, 0.00758, 0.04525], [0.0175, 0.0152, 0.0185], [0, 0, 0]),
    ]
    corners = np.array(list(itertools.product([-1.0, 1.0], repeat=3))) / 2
    return np.vstack(
        [
            Rotation.from_euler("xyz", rpy).apply(corners * size) + center
            for center, size, rpy in boxes
        ]
    )


def main():
    ASSETS.mkdir(exist_ok=True)
    counts = [1, 4, 3, 3, 3, 4, 3, 3]
    spheres, coverage, sources = {}, [], []
    for i, count in enumerate(counts):
        path = SOURCE / f"meshes/robots/fr3/collision/link{i}.stl"
        triangles = stl(path)
        spheres[f"fr3_link{i}"] = fit(triangles.reshape(-1, 3), count)
        # Face centroids are an independent probe, not a continuous certificate.
        probes = np.vstack([triangles.reshape(-1, 3), triangles.mean(1)])
        balls = spheres[f"fr3_link{i}"]
        clearance = np.linalg.norm(
            probes[:, None] - np.array([b["center"] for b in balls])[None], axis=2
        ) - np.array([b["radius"] for b in balls])
        coverage.append(
            dict(
                link=f"fr3_link{i}",
                spheres=count,
                mesh_probes=len(probes),
                uncovered_probes=int((clearance.min(1) > 1e-10).sum()),
                maximum_probe_gap=float(clearance.min(1).max()),
            )
        )
        sources.append(
            dict(
                path=str(path.relative_to(SOURCE)),
                sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
            )
        )
    hand = SOURCE / "meshes/robot_ee/franka_hand_white/collision/hand.stl"
    spheres["fr3_hand"] = fit(stl(hand).reshape(-1, 3), 5)
    spheres["fr3_leftfinger"] = fit(finger_points(), 2)
    spheres["fr3_rightfinger"] = fit(finger_points(True), 2)
    for path in [
        hand,
        SOURCE / "robots/fr3/kinematics.yaml",
        SOURCE / "robots/fr3/joint_limits.yaml",
        SOURCE / "end_effectors/common/franka_hand.xacro",
        SOURCE / "robots/common/franka_arm.srdf.xacro",
        SOURCE / "end_effectors/franka_hand/franka_hand.srdf.xacro",
    ]:
        sources.append(
            dict(
                path=str(path.relative_to(SOURCE)),
                sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
            )
        )
    disabled = []
    for filename in [
        "robots/common/franka_arm.srdf.xacro",
        "end_effectors/franka_hand/franka_hand.srdf.xacro",
    ]:
        xml = ET.parse(SOURCE / filename)
        for elem in xml.iter("disable_collisions"):
            names = [elem.attrib[key] for key in ["link1", "link2"]]
            names = [
                name.replace("${arm_prefix}", "")
                .replace("${ee_prefix}", "fr3_")
                .replace("${robot_type}", "fr3")
                .replace("${prefix}", "fr3_")
                for name in names
            ]
            disabled.append(names)
    data = dict(
        source_repository="https://github.com/frankarobotics/franka_description",
        source_revision=REVISION,
        fidelity="new_33_sphere_fit_not_original_paper_model",
        sources=sources,
        fitting="deterministic farthest-point initialization, 100 Lloyd iterations, enclosing cluster radius + 0.0025m",
        continuous_mesh_cover_certified=False,
        spheres=spheres,
        mesh_probe_audit=coverage,
        kinematics=yaml.safe_load((SOURCE / "robots/fr3/kinematics.yaml").read_text()),
        limits=yaml.safe_load((SOURCE / "robots/fr3/joint_limits.yaml").read_text()),
        disabled_collision_links=disabled,
    )
    assert sum(map(len, spheres.values())) == 33
    atomic(ASSETS / "fr3_33.json", data)
    for filename in ["LICENSE", "NOTICE"]:
        shutil.copy2(SOURCE / filename, ASSETS / ("franka_" + filename))
    print("Saved 33-sphere FR3 reconstruction", coverage, "disabled", disabled)


if __name__ == "__main__":
    main()
