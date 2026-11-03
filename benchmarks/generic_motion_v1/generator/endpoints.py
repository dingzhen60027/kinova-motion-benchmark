"""All-joint endpoint sampling using collision predicates, never a planner."""
from __future__ import annotations

import numpy as np

from .scene import rng
from .configuration import validate_endpoint_config


def normalized_distance(start, goal, lower, upper):
    return float(np.sqrt(np.mean(((np.asarray(goal) - start) / (upper - lower)) ** 2)))


def _collision_observations(model, states, name):
    native = np.asarray(model.cuda(states))
    clearance = np.asarray(model.clearance(states))
    if (native.shape != (len(states),) or native.dtype.kind != 'b' or
            clearance.shape != (len(states),) or not np.isfinite(clearance).all()):
        raise ValueError(f'{name} classifiers must return one Boolean and one finite clearance per configuration')
    return native, clearance


def straight_line(model, start, goal, step):
    if not np.isfinite(step) or step <= 0:
        raise ValueError('Straight-line step must be finite and positive')
    count = max(2, int(np.ceil(np.linalg.norm(np.asarray(goal) - start) / step)) + 1)
    if count > 1000000:
        raise ValueError('Straight-line check exceeds one million samples; choose a supported step')
    states = np.linspace(start, goal, count)
    native, clearance = _collision_observations(model, states, 'Straight-line')
    independent = clearance > 0
    mismatch = native != independent
    significant = mismatch & (np.abs(clearance) > model.contact_band_m)
    return {
        "straight_line_collision": bool(not (native.all() and independent.all())),
        "straight_line_samples": count, "straight_line_step_rad": step,
        "straight_line_minimum_clearance_m": float(clearance.min()),
        "straight_line_native_colliding_samples": int((~native).sum()),
        "straight_line_independent_colliding_samples": int((~independent).sum()),
        "straight_line_checker_disagreements": int(mismatch.sum()),
        "straight_line_disagreements_outside_contact_band": int(significant.sum()),
        "straight_line_check_is_numerical": True,
    }, {"straight_line_q": states, "straight_line_cuda_free": native,
        "straight_line_independent_clearance_m": clearance}


def generate_endpoints(model, seed, distance_bin, config):
    validate_endpoint_config(config, model)
    if distance_bin not in config['normalized_distance_bins']:
        raise ValueError('Unknown endpoint distance bin')
    random = rng(seed)
    margin = config["endpoint_joint_margin_rad"]
    minimum, maximum = config["normalized_distance_bins"][distance_bin]
    batch_size = config["endpoint_batch_size"]
    max_attempts = config["endpoint_max_attempts"]
    all_states, all_native, all_clearance = [], [], []
    start = goal = None
    considered = collision_rejected = distance_rejected = 0
    for offset in range(0, max_attempts, batch_size):
        count = min(batch_size, max_attempts - offset)
        states = random.uniform(model.lower + margin, model.upper - margin, (count, 7))
        native, distances = _collision_observations(model, states, 'Endpoint')
        all_states.extend(states)
        all_native.extend(native)
        all_clearance.extend(distances)
        for q, free, clearance in zip(states, native, distances):
            considered += 1
            if not free or clearance <= config["endpoint_clearance_m"]:
                collision_rejected += 1
                continue
            if start is None:
                start = q.copy()
                continue
            normalized = normalized_distance(start, q, model.lower, model.upper)
            if minimum <= normalized < maximum:
                goal = q.copy()
                break
            distance_rejected += 1
        if goal is not None:
            break
    states = np.asarray(all_states).reshape(-1, 7)
    native = np.asarray(all_native, dtype=bool)
    distances = np.asarray(all_clearance)
    significant = (native != (distances > 0)) & (np.abs(distances) > model.contact_band_m)
    record = {
        "q_start": None if start is None else start.tolist(),
        "q_goal": None if goal is None else goal.tolist(),
        "endpoint_sampling": {
            "policy": "uniform all seven joints; accept only limits, native and independent collision checks, clearance and requested normalized distance bin",
            "distance_bin": distance_bin, "distance_bin_bounds": [minimum, maximum],
            "joint_margin_rad": margin, "minimum_clearance_m": config["endpoint_clearance_m"],
            "maximum_attempts": max_attempts, "drawn_configurations": len(states),
            "considered_configurations": considered,
            "unused_batch_tail_configurations": len(states) - considered,
            "collision_or_clearance_rejections": collision_rejected,
            "distance_bin_rejections": distance_rejected,
            "checker_disagreements": int(np.sum(native != (distances > 0))),
            "checker_disagreements_outside_contact_band": int(significant.sum()),
        },
        "validation_status": "valid_input" if goal is not None else "invalid_endpoint",
        "problem_feasibility": "feasibility_unknown",
        "start_goal_joint_distance": None,
        "normalized_start_goal_joint_distance": None,
        "straight_line_collision": None,
    }
    raw = {"endpoint_proposal_q": states, "endpoint_proposal_cuda_free": native,
           "endpoint_proposal_independent_clearance_m": distances}
    if goal is not None:
        record["start_goal_joint_distance"] = float(np.linalg.norm(goal - start))
        record["normalized_start_goal_joint_distance"] = normalized_distance(start, goal, model.lower, model.upper)
        endpoints = np.vstack([start, goal])
        record["endpoint_clearances_m"] = model.clearance(endpoints).tolist()
        record["endpoint_tcp_world_m"] = model.tcp(endpoints).tolist()
        line_report, line_raw = straight_line(model, start, goal, config["straight_line_step_rad"])
        record.update(line_report)
        raw.update(line_raw)
        if line_report["straight_line_disagreements_outside_contact_band"]:
            record["validation_status"] = "model_consistency_failed"
    # A checker disagreement is a model failure even if it prevents the search
    # from finding a pair. Do not report it as ordinary endpoint exhaustion.
    if significant.any():
        record["validation_status"] = "model_consistency_failed"
    return record, raw
