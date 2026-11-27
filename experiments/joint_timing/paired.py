"""Equal-proposal, CPU-only, fixed-duration timing ablation for a small Dev cohort."""

import hashlib
import json
import os
from pathlib import Path
import sys
import time
from collections import defaultdict

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "runs/phase_search/mechanism_control"
sys.path.insert(0, str(ROOT))
from benchmarks.generic_motion_v1.model import Model
from benchmarks.generic_motion_v1.roadmap import load_config
from benchmarks.generic_motion_v1.runners.phase_search import (
    PROFILE,
    clock_state,
    clock_curve,
    rest_times,
)
from benchmarks.generic_motion_v1.runners.phase_refinement import clearance_gradient
from benchmarks.generic_motion_v1.runners.validate import sample_candidate

MANIFEST = ROOT / "benchmarks/generic_motion_v1/manifests/dev.jsonl"
ARMS = ["guided", "blind_direction", "uniform_shift", "witness"]
SELECTION_SALT = "kinova-joint-timing-direction-pilot"
N = PROFILE["schedules_per_scale"]
U = np.linspace(0, 1, PROFILE["coarse_stations"])


def digest(s):
    return hashlib.sha256(s).hexdigest()


def save(path, v):
    raw = (
        json.dumps(v, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False)
        + "\n"
    )
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(raw)
    os.replace(tmp, path)


def select():
    groups = defaultdict(lambda: defaultdict(list))
    for line in MANIFEST.read_text().splitlines():
        r = json.loads(line)
        if (
            r["split"] == "dev"
            and r["validation_status"] == "valid_input"
            and r["straight_line_collision"] is True
        ):
            groups[r["family"]][r["scene_id"]].append(r)
    rank = lambda key: digest((SELECTION_SALT + key).encode())
    cases = []
    for family, scenes in sorted(groups.items()):
        for scene in sorted(scenes, key=rank)[:3]:
            cases.append(min(scenes[scene], key=lambda r: rank(r["problem_id"])))
    assert len(cases) == 12 and len({r["scene_id"] for r in cases}) == len(cases)
    return cases


class Scene:
    def __init__(self, case):
        self.model = Model(case["voxels_xyz"], case["voxel_radius_m"])
        self.m = self.model.nto_plant()
        self.port = self.m.scene_graph.get_query_output_port()
        assert self.model.robot_sha256 == case["robot_sha256"]
        assert self.model.collision_geometry_sha256 == case["collision_geometry_sha256"]

    def mask(self, states):
        valid = []
        for q in states:
            self.m.plant.SetPositions(self.m.plant_context, q)
            query = self.port.Eval(self.m.scene_context)
            valid.append(
                not query.ComputeSignedDistancePairwiseClosestPoints(
                    self.model.contact_band_m
                )
            )
        return np.array(valid)


def middle(mask):
    bad = np.flatnonzero(~mask)
    groups = np.split(bad, np.flatnonzero(np.diff(bad) > 1) + 1)
    longest = max(groups, key=len)
    return longest[len(longest) // 2]


def evaluate(scene, case, cfg, start, goal, T, base, ramp0, width, shift):
    span = np.where(base > 0, base + width * (T - base), 0.0)
    ramp = np.divide(ramp0, base, out=np.zeros(7), where=base > 0) * span
    delay = shift * (T - span)
    curve = clock_curve(start, goal, T, delay, span, ramp)
    samples = sample_candidate(curve, cfg)
    ctrl = samples["position_controls"]
    d = samples["derivatives"]
    tol = cfg["feasibility_tolerance"]
    ok = bool(
        np.max(np.linalg.norm(samples["q"][[0, -1]] - [start, goal], axis=1))
        <= cfg["endpoint_tolerance_rad"]
        and np.all(ctrl >= scene.model.lower - tol)
        and np.all(ctrl <= scene.model.upper + tol)
        and np.all(
            d["velocity_per_joint"] <= np.array(case["velocity_limits_rad_s"]) + tol
        )
        and np.all(
            d["acceleration_per_joint"]
            <= np.array(case["acceleration_limits_rad_s2"]) + tol
        )
        and d["endpoint_velocity"] <= tol
        and d["continuity_residual"] <= tol
        and abs(samples["duration"] - T) <= tol
    )
    t = U * T
    q, _ = clock_state(t[:, None], start, goal - start, delay, span, ramp)
    mask = scene.mask(q)
    row = dict(
        motion_pass=ok,
        coarse_free=bool(mask.all()),
        dense_count=0,
        positions_checked=len(q),
        candidate_sha256=digest(
            curve["controls"].tobytes() + curve["durations"].tobytes()
        ),
    )
    if mask.all():
        q, t = samples["q"], samples["time"]
        mask = scene.mask(q)
        row.update(
            dense_count=len(q), positions_checked=row["positions_checked"] + len(q)
        )
    row["dense_valid"] = bool(row["coarse_free"] and ok and mask.all())
    witness = None
    if ok and not mask.all():
        i = middle(mask)
        witness = (float(t[i]), q[i])
    return row, witness


def run_case(case, arms, cfg):
    start, goal = np.array(case["q_start"]), np.array(case["q_goal"])
    base, ramp0 = rest_times(
        goal - start,
        np.array(case["velocity_limits_rad_s"]),
        np.array(case["acceleration_limits_rad_s2"]),
    )
    T = float(max(base))
    seed = case["paired_seeds"][0]["method_seed"]
    assert case["paired_seeds"][0]["replicate"] == 0
    scene = Scene(case)
    line = np.linspace(start, goal, len(U))
    mask = scene.mask(line)
    original = scene.model.clearance(line) > scene.model.contact_band_m
    assert np.array_equal(
        mask, original
    ), "Independent representations disagree on chord"
    if mask.all():
        line = sample_candidate(
            dict(
                kind="polygon",
                objective="minimum_distance",
                points=np.array([start, goal]),
            ),
            cfg,
        )["q"]
        mask = scene.mask(line)
    assert not mask.all(), "Frozen blocked chord has no witness on dense CPU grid"
    anchor = line[middle(mask)]
    if "witness" in arms:
        from experiments.joint_timing.witness_clocks import propose
    results = []
    for arm in arms:
        begun = time.perf_counter()
        width, shift = (
            lambda rng: (rng.uniform(0, 1, (N, 7)), rng.uniform(0, 1, (N, 7)))
        )(np.random.default_rng(np.random.SeedSequence(seed).spawn(3)[0]))
        width[:3] = 0
        gradient_calls = 0
        if arm in ("guided", "witness", "blind_direction"):
            if arm in ("guided", "witness"):
                _, grad = clearance_gradient(scene.model, anchor)
                gradient_calls += 1
                lead = (grad * (goal - start) < 0).astype(float)
            else:
                alt = np.random.default_rng(np.random.SeedSequence([seed, 538751]))
                lead = alt.integers(0, 2, 7).astype(float)
            shift[:3] = [lead, 1.0 - lead, np.full(7, 0.5)]
            shift[3 : N // 2] = 0.7 * lead + 0.3 * shift[3 : N // 2]
        witness = None
        previous = None
        rows = []
        for i in range(N):
            w, s = width[i].copy(), shift[i].copy()
            trial = "prescribed"
            if arm == "witness" and i % 2 == 1 and witness is not None:
                wt, wq = witness
                clearance, grad = clearance_gradient(scene.model, wq)
                gradient_calls += 1
                w, s = propose(
                    start,
                    goal,
                    T,
                    base,
                    ramp0,
                    *previous,
                    wt,
                    grad,
                    clearance_m=float(clearance)
                )
                w, s = np.asarray(w), np.asarray(s)
                assert (
                    w.shape == s.shape == (7,)
                    and np.all((w >= 0) & (w <= 1))
                    and np.all((s >= 0) & (s <= 1))
                )
                trial = "witness_projection"
            row, witness = evaluate(scene, case, cfg, start, goal, T, base, ramp0, w, s)
            previous = (w, s)
            row.update(slot=i, source=trial, width=w.tolist(), shift=s.tolist())
            rows.append(row)
        costs = time.perf_counter() - begun
        results.append(
            dict(
                arm=arm,
                case=case["problem_id"],
                scene=case["scene_id"],
                family=case["family"],
                method_seed=seed,
                duration_lower_bound_s=T,
                cpu_wall_seconds=costs,
                gradient_evaluations=gradient_calls,
                drake_samples=sum(r["positions_checked"] for r in rows),
                validated_proposals=sum(r["dense_valid"] for r in rows),
                motion_failures=sum(not r["motion_pass"] for r in rows),
                success=any(r["dense_valid"] for r in rows),
                proposals=rows,
            )
        )
    return results
