"""CPU Dev ablation at a fixed analytic arrival time; see --help."""

import argparse
import json
import os
import time
from pathlib import Path

import numpy as np
from scipy.optimize import brentq

from . import cohort
from . import paired as p
from .excursions import proposals

OUT = p.ROOT / "runs/phase_search/excursion_pilot"
SALT = 793187
ARMS = ("guided", "uniform_shift", "excursion", "alternating")


def lock_protocol(folder, name, cases):
    files = [
        Path(__file__),
        Path(__file__).with_name("excursions.py"),
        Path(cohort.__file__),
        Path(p.__file__),
        p.MANIFEST,
        p.ROOT / "configs/evaluation.json",
    ]
    cfg_sources = p.ROOT / "benchmarks/generic_motion_v1/runners"
    files.extend(
        cfg_sources / s
        for s in ["phase_search.py", "phase_refinement.py", "validate.py"]
    )
    protocol = dict(
        cohort=name,
        cases=[c["problem_id"] for c in cases],
        replicate=0,
        strata="Eligible preselected Dev queries with a blocked chord. No Test data.",
        deadline="max per-axis rest-to-rest minimum time",
        proposals=25,
        background="same guided draws and clearance gradient as paired.py",
        randomness_salt=SALT,
        via_time="physical time at which the unique critical-joint clock reaches the geometric chord witness",
        excursion="14 single-axis edge probes, followed by 11 random feasible axis subsets and via values",
        alternating="slots 0,2,...,24 = guided slots 0..12; slots 1,3,...,23 = excursion slots 0..11",
        comparisons="Each arm at most 25 slots; first valid one-based index. Report success by case and scene; descriptive Dev evidence.",
        timing="Alternating outcomes replay saved independent proposals; does not measure integrated end-to-end time. Shared cached scene and gradient construction will differ from running two arms.",
        validation="Same frozen 33-point screen, dense adaptive positions, derivative hulls, endpoint test, independent canonical Drake-vs-explicit-sphere check for every dense success. CPU only.",
        source_sha256={
            str(f.relative_to(p.ROOT)): p.digest(f.read_bytes()) for f in files
        },
    )
    folder.mkdir(parents=True, exist_ok=True)
    dest = folder / "protocol.json"
    if dest.exists():
        if json.loads(dest.read_text()) != protocol:
            raise ValueError(
                "Frozen excursion contract does not match these sources or cases"
            )
    else:
        p.save(dest, protocol)


def anchor_time(scene, cfg, start, goal, T, base, ramp):
    line = np.linspace(start, goal, len(p.U))
    mask = scene.mask(line)
    assert np.array_equal(
        mask, scene.model.clearance(line) > scene.model.contact_band_m
    )
    if mask.all():
        line = p.sample_candidate(
            dict(
                kind="polygon",
                objective="minimum_distance",
                points=np.array([start, goal]),
            ),
            cfg,
        )["q"]
        mask = scene.mask(line)
    if mask.all():
        raise RuntimeError("No canonical chord collision witness on the frozen grid")
    i = p.middle(mask)
    witness = line[i]
    j = int(np.argmax(base))

    def progress(t):
        q, _ = p.clock_state(t, start, goal - start, np.zeros(7), base, ramp)
        return (q[j] - witness[j]) / (goal[j] - start[j])

    t = float(brentq(progress, 0.0, T))
    return t, witness, j


def evaluate(case, cfg, directory):
    begin = time.perf_counter()
    start, goal = map(np.asarray, (case["q_start"], case["q_goal"]))
    v, a = map(
        np.asarray, (case["velocity_limits_rad_s"], case["acceleration_limits_rad_s2"])
    )
    base, ramp = p.rest_times(goal - start, v, a)
    T = float(max(base))
    scene = p.Scene(case)
    t, anchor, critical = anchor_time(scene, cfg, start, goal, T, base, ramp)
    clearance, grad = p.clearance_gradient(scene.model, anchor)
    seed = case["paired_seeds"][0]["method_seed"]
    assert case["paired_seeds"][0]["replicate"] == 0
    rng = np.random.default_rng(np.random.SeedSequence(seed).spawn(3)[0])
    width = rng.uniform(0, 1, (p.N, 7))
    shift = rng.uniform(0, 1, (p.N, 7))
    lead = (grad * (goal - start) < 0).astype(float)
    width[:3] = 0
    shift[:3] = [lead, 1 - lead, np.full(7, 0.5)]
    shift[3 : p.N // 2] = 0.7 * lead + 0.3 * shift[3 : p.N // 2]
    gen = proposals(
        start,
        goal,
        T,
        t,
        scene.model.lower,
        scene.model.upper,
        v,
        a,
        grad,
        base,
        ramp,
        width,
        shift,
        np.random.default_rng(np.random.SeedSequence([seed, SALT])),
    )
    setup_wall = time.perf_counter() - begin
    qmin, qmax = np.minimum(start, goal), np.maximum(start, goal)
    rows = []
    blobs = {}
    for slot, m in enumerate(gen):
        tick = time.perf_counter()
        curve = m.curve()
        checksum = p.digest(curve["controls"].tobytes() + curve["durations"].tobytes())
        sample = p.sample_candidate(curve, cfg)
        ok = cohort.validate_dynamics(sample, case, scene.model, cfg, T)
        qs = m.state(T * p.U)[0]
        mask = scene.mask(qs)
        coarse_free = bool(mask.all())
        n_dense = 0
        reference_n = 0
        min_clearance = None
        times = T * p.U
        if coarse_free:
            qs, times = sample["q"], sample["time"]
            mask = scene.mask(qs)
            n_dense = len(qs)
        success = bool(ok and coarse_free and mask.all())
        if success:
            reference = scene.model.clearance(sample["q"])
            min_clearance = float(np.min(reference))
            reference_n = len(reference)
            if min_clearance <= scene.model.contact_band_m:
                raise RuntimeError(
                    "Explicit-voxel Drake and the independent benchmark model disagree"
                )
            blobs[f"slot{slot}_controls"] = curve["controls"]
            blobs[f"slot{slot}_durations"] = curve["durations"]
        collision = None
        if not mask.all():
            k = p.middle(mask)
            collision = dict(time_s=float(times[k]), q_rad=qs[k].tolist())
        outside = np.maximum.reduce(
            [
                qmin - sample["q"].min(axis=0),
                sample["q"].max(axis=0) - qmax,
                np.zeros(7),
            ]
        )
        rows.append(
            dict(
                slot=slot,
                motion_pass=ok,
                coarse_free=coarse_free,
                dense_valid=success,
                positions_checked=len(p.U) + n_dense,
                dense_count=n_dense,
                reference_count=reference_n,
                min_verified_clearance_m=min_clearance,
                collision=collision,
                candidate_sha256=checksum,
                active=np.flatnonzero(m.active).tolist(),
                via_q_rad=m.state([t])[0][0].tolist(),
                outside_endpoint_range_rad=outside.tolist(),
                background_width=width[slot].tolist(),
                background_shift=shift[slot].tolist(),
                wall_s=time.perf_counter() - tick,
            )
        )
    if blobs:
        name = directory / "trajectories.npz"
        tmp = directory / "trajectories.tmp.npz"
        np.savez_compressed(tmp, **blobs)
        os.replace(tmp, name)
    return dict(
        case=case["problem_id"],
        scene=case["scene_id"],
        family=case["family"],
        method_seed=seed,
        duration_lower_bound_s=T,
        critical_joint_zero_index=critical,
        via_time_s=t,
        original_collision_q_rad=anchor.tolist(),
        chord_clearance_m=float(clearance),
        chord_gradient=grad.tolist(),
        setup_wall_s=setup_wall,
        cpu_wall_s=time.perf_counter() - begin,
        arm="excursion",
        proposals=rows,
    )


def first_hit(rows):
    return next((i + 1 for i, row in enumerate(rows) if row["dense_valid"]), None)


def summarize(folder, control, cases):
    per_case = []
    for case in cases:
        pid = case["problem_id"]
        target = folder / "cases" / pid
        refs = {}
        for arm in ("guided", "uniform_shift"):
            data = json.loads((control / "cases" / pid / (arm + ".json")).read_text())
            assert data["method_seed"] == case["paired_seeds"][0]["method_seed"]
            refs[arm] = data["proposals"]
        exp = json.loads((target / "excursion.json").read_text())
        rows = exp["proposals"]
        assert len(rows) == len(refs["guided"]) == len(refs["uniform_shift"]) == 25
        mixed = [(refs["guided"] if i % 2 == 0 else rows)[i // 2] for i in range(25)]
        arms = {**refs, "excursion": rows, "alternating": mixed}
        hits = {key: first_hit(s) for key, s in arms.items()}
        witnesses = [
            dict(
                slot=r["slot"],
                outside_rad=r["outside_endpoint_range_rad"],
                min_verified_clearance_m=r["min_verified_clearance_m"],
                candidate_sha256=r["candidate_sha256"],
            )
            for r in rows
            if r["dense_valid"] and max(r["outside_endpoint_range_rad"]) > 1e-7
        ]
        per_case.append(
            dict(
                case=pid,
                scene=case["scene_id"],
                family=case["family"],
                first_hit=hits,
                num_passing={
                    key: sum(r["dense_valid"] for r in s) for key, s in arms.items()
                },
                witnesses=witnesses,
                excursion_wall_s=exp["cpu_wall_s"],
                motion_failures=sum(not r["motion_pass"] for r in rows),
            )
        )
    successful = {
        arm: {r["case"] for r in per_case if r["first_hit"][arm] is not None}
        for arm in ARMS
    }
    totals = {
        arm: dict(
            successful_cases=len(ids),
            total_cases=len(cases),
            passing_proposals=sum(r["num_passing"][arm] for r in per_case),
        )
        for arm, ids in successful.items()
    }
    pairs = {
        a: {
            b: dict(
                only_first=sorted(x - y), only_second=sorted(y - x), both=len(x & y)
            )
            for b, y in successful.items()
        }
        for a, x in successful.items()
    }
    result = dict(
        arms=totals,
        paired=pairs,
        per_case=per_case,
        total_outside_range_valid_witnesses=sum(len(r["witnesses"]) for r in per_case),
        integrator="combined arm is predeclared offline selection of 25 slots; CPU wall times include independent reference audits and cached controls; no end-to-end speed estimate.",
    )
    p.save(folder / "comparison.json", result)
    print(json.dumps(totals), flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--cohort", choices=["pilot", "dev-remainder"], default="pilot")
    ap.add_argument("--prepare-only", action="store_true")
    args = ap.parse_args()
    if os.environ.get("CUDA_VISIBLE_DEVICES") != "":
        raise RuntimeError(
            'Explicitly set CUDA_VISIBLE_DEVICES="" for the CPU-only study'
        )

    def disabled(*args, **kwargs):
        raise RuntimeError("GPU disabled in this protocol")

    p.Model.cuda = disabled
    subset = cohort.cohorts()[args.cohort]
    folder = OUT if args.cohort == "pilot" else OUT / "dev-remainder"
    control = p.OUT if args.cohort == "pilot" else p.OUT / "dev-remainder"
    lock_protocol(folder, args.cohort, subset)
    if args.prepare_only:
        print(folder / "protocol.json")
        return
    cfg = p.load_config(p.ROOT / "configs/evaluation.json")
    for case in subset:
        dest = folder / "cases" / case["problem_id"]
        dest.mkdir(parents=True, exist_ok=True)
        path = dest / "excursion.json"
        if not path.exists():
            record = evaluate(case, cfg, dest)
            p.save(path, record)
            print(
                case["problem_id"],
                "first passing slot (1-based)",
                first_hit(record["proposals"]),
                flush=True,
            )
    summarize(folder, control, subset)


if __name__ == "__main__":
    main()
