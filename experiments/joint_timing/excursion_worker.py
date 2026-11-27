"""Isolated End-to-End Dev worker for the lower-bound clock-bank comparison."""

import argparse
from contextlib import contextmanager
from pathlib import Path
import time
from types import FunctionType

import numpy as np
from scipy.optimize import brentq

from benchmarks.generic_motion_v1 import batch
from benchmarks.generic_motion_v1.common import ROOT, read, write
from benchmarks.generic_motion_v1.runners import phase_search as phase
from .excursions import proposals
from .excursion_pilot import SALT
from .paired import middle, select

ARMS = ("guided", "excursion")


def admit(job):
    """Only allow exact official Dev jobs in the preselected 12-query matrix."""
    if job.get("objective") != "minimum_time" or job.get("method") != "PHASE_SEARCH":
        raise ValueError("Expected a PHASE_SEARCH minimum_time request")
    if job.get("replica_seed") != 0 or not job.get("case_id", "").startswith("dev-"):
        raise ValueError("Only fixed Dev replicate 0 is supported")
    cohort = {case["problem_id"] for case in select()}
    if job["case_id"] not in cohort:
        raise ValueError("The case is outside the 12-query Dev pilot")
    # Compare the entire official request including resolved configuration and
    # content-bound case_ref before allowing the official worker to dereference it.
    reference = read(ROOT / "runs/phase_search/dev/run_manifest.json")
    expected = next(
        (j for j in reference["jobs"] if j["job_id"] == job.get("job_id")), None
    )
    if job != expected or job["evaluation_config"]["online_budget_seconds"] != 60:
        raise ValueError("Request differs from the original Dev run")
    return job["method_seed"]


@contextmanager
def install(arm, seed, log):
    """Change only the first bank within this fresh worker; restore on exit."""
    if arm not in ARMS:
        raise ValueError("Unknown experimental arm")
    if arm == "guided":
        yield
        return
    original_witness, original_bank = phase.collision_witness, phase.schedules
    scales = phase.PROFILE["duration_scales"]
    if scales != (1.0, 1.25, 1.6, 2.1) or phase.PROFILE["schedules_per_scale"] != 25:
        raise ValueError("The frozen schedule profile changed")
    # Reuse the original function and its RNG for every later scale. A private
    # globals mapping keeps the official PROFILE (also stored in its trace)
    # untouched. The separate via RNG never advances the original stream.
    later_globals = {
        **original_bank.__globals__,
        "PROFILE": {**phase.PROFILE, "duration_scales": scales[1:]},
    }
    later_bank = FunctionType(original_bank.__code__, later_globals)
    anchor = None

    def witness(model, line, mask):
        nonlocal anchor
        answer = original_witness(model, line, mask)
        anchor = line[middle(mask)].copy()
        return answer

    def bank(model, start, goal, velocity, accel, rng, gradient, cfg, deadline):
        if time.perf_counter() >= deadline:
            return
        if anchor is None:
            raise RuntimeError("Missing original worker collision witness")
        base, ramp = phase.rest_times(goal - start, velocity, accel)
        T = float(max(base))
        j = int(np.argmax(base))

        def residual(t):
            q, _ = phase.clock_state(t, start, goal - start, np.zeros(7), base, ramp)
            return (q[j] - anchor[j]) / (goal[j] - start[j])

        t = brentq(residual, 0.0, T)
        n = phase.PROFILE["schedules_per_scale"]
        width = rng.uniform(0, 1, (n, 7))
        shift = rng.uniform(0, 1, (n, 7))
        width[:3] = 0
        lead = (gradient * (goal - start) < 0).astype(float)
        shift[:3] = [lead, 1 - lead, np.full(7, 0.5)]
        shift[3 : n // 2] = 0.7 * lead + 0.3 * shift[3 : n // 2]
        motions = list(
            proposals(
                start,
                goal,
                T,
                t,
                model.lower,
                model.upper,
                velocity,
                accel,
                gradient,
                base,
                ramp,
                width,
                shift,
                np.random.default_rng(np.random.SeedSequence([seed, SALT])),
            )
        )
        stations = T * np.linspace(0, 1, phase.PROFILE["coarse_stations"])
        q = np.stack([m.state(stations)[0] for m in motions])
        # Same CUDA query, order, dense screen and deadline as the original bank.
        free = model.cuda(q.reshape(-1, 7)).reshape(n, -1).all(axis=1)
        lengths = np.linalg.norm(np.diff(q, axis=1), axis=2).sum(axis=1)
        log.update(
            first_layer_count=n,
            first_layer_coarse_free=int(free.sum()),
            via_time_s=t,
            first_layer_dense_checks=0,
            first_layer_returned_slot=None,
        )
        for idx in np.argsort(lengths):
            if not free[idx] or time.perf_counter() >= deadline:
                continue
            curve = motions[idx].curve()
            log["first_layer_dense_checks"] += 1
            if phase.dense_free(model, curve, cfg):
                log["first_layer_returned_slot"] = int(idx)
                yield curve, q[idx], 1.0
                break
        yield from later_bank(
            model, start, goal, velocity, accel, rng, gradient, cfg, deadline
        )

    phase.collision_witness, phase.schedules = witness, bank
    try:
        yield
    finally:
        phase.collision_witness, phase.schedules = original_witness, original_bank


def run(arm, attempt):
    attempt = Path(attempt)
    if (attempt / "result.json").exists() or (
        attempt / "experimental_arm.json"
    ).exists():
        raise FileExistsError("Refusing to overwrite a worker attempt")
    job = read(attempt / "request.json")
    seed = admit(job)
    evidence = dict(
        arm=arm,
        case=job["case_id"],
        method_seed=seed,
        first_layer_count=None,
        method=job["method"],
        cohort="12 blocked-chord Dev queries, replicate 0",
        profile="only the 25 schedules at T=L differ between arms",
    )
    write(attempt / "experimental_arm.json", evidence)
    try:
        with install(arm, seed, evidence):
            batch.worker(attempt)
    finally:
        write(attempt / "experimental_arm.json", evidence)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--arm", choices=ARMS, required=True)
    ap.add_argument("--attempt", type=Path, required=True)
    args = ap.parse_args()
    run(args.arm, args.attempt)


if __name__ == "__main__":
    main()
