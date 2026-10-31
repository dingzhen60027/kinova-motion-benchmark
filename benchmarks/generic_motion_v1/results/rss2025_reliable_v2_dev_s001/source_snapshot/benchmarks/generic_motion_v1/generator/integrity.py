"""Schema, completeness and deterministic endpoint replay for input audits."""
from __future__ import annotations

from collections import Counter
from pathlib import Path
import numpy as np

from .configuration import validate_config
from .scene import FAMILIES, digest, domain_seed, rng


def safe_path(root, relative):
    if not isinstance(relative, str) or Path(relative).is_absolute():
        raise ValueError('Dataset references must be relative paths')
    path = (root / relative).resolve()
    if not path.is_relative_to(root.resolve()):
        raise ValueError('Dataset reference escapes the dataset root')
    return path


def _expect(condition, message):
    if not condition:
        raise ValueError(message)


def preflight(path, rows, config, receipt, file_sha256):
    """Reject incomplete or internally inconsistent inputs before any GPU call."""
    validate_config(config)
    root, split = path.parent.parent, path.stem
    _expect(split in ('smoke', 'dev', 'test'), 'Unrecognized manifest split')
    _expect(receipt['split'] == split, 'Receipt split does not match manifest filename')
    _expect(type(receipt['seed']) is int and receipt['seed'] >= 0, 'Receipt seed must be a nonnegative integer')
    plan = config['splits'][split]
    expected = {}
    for family in (*FAMILIES, 'open_control'):
        layouts = plan['open_layouts'] if family == 'open_control' else plan['main_layouts_per_family']
        for index in range(layouts):
            sid = f'{split}-{family}-{index:03d}'
            for pair in range(plan['pairs_per_layout']):
                expected[f'{sid}-p{pair:02d}'] = (sid, family, index, pair)
    _expect(len(rows) == len(expected), 'Manifest does not contain the configured number of problems')
    _expect(Counter(row['problem_id'] for row in rows) == Counter(expected.keys()), 'Missing, duplicate or unexpected problem IDs')
    protocol = receipt['protocol']
    _expect(receipt['geometry_protocol_sha256'] == digest(protocol), 'Receipt protocol digest mismatch')
    expected_files = {f'manifests/{split}.jsonl', f'configs/{split}_generator.json'}
    valid_count = 0
    for row in rows:
        identity = row['problem_id']
        sid, family, index, pair = expected[identity]
        _expect(row['split'] == split and row['family'] == family and row['scene_id'] == sid, f'{identity}: ID metadata mismatch')
        _expect(row['benchmark_version'] == config['benchmark_version'] == receipt['benchmark_version'], f'{identity}: benchmark version mismatch')
        _expect(row['world_frame'] == config['world_frame'], f'{identity}: world frame mismatch')
        _expect(row['voxel_radius_m'] == config['voxel_radius_m'], f'{identity}: voxel radius mismatch')
        _expect(row['scene_seed'] == domain_seed(receipt['seed'], 'scene', split, family, index), f'{identity}: scene seed mismatch')
        _expect(row['endpoint_seed'] == domain_seed(receipt['seed'], 'endpoint', split, family, index, pair), f'{identity}: endpoint seed mismatch')
        paired = [dict(replicate=r, drm_seed=r,
            method_seed=domain_seed(receipt['seed'], 'method', split, family, index, pair, r) % 2**32,
            solver_seed=domain_seed(receipt['seed'], 'solver', split, family, index, pair, r) % 2**32) for r in range(3)]
        _expect(row['paired_seeds'] == paired, f'{identity}: paired seeds mismatch')
        for key in ('robot_sha256', 'collision_geometry_sha256'):
            _expect(row[key] == protocol[key], f'{identity}: {key} differs from protocol')
        _expect(row['geometry_protocol_sha256'] == receipt['geometry_protocol_sha256'], f'{identity}: protocol mismatch')
        _expect(row['generator_config_sha256'] == protocol['config_sha256'], f'{identity}: config hash mismatch')
        _expect(row['generator_sources_sha256'] == protocol['generator_sources_sha256'], f'{identity}: source inventory mismatch')
        _expect(row['source_code_commit'] == receipt['source_code_commit'], f'{identity}: source commit mismatch')
        _expect(row['objectives'] == ['minimum_distance', 'minimum_time'], f'{identity}: objectives mismatch')
        _expect(row['velocity_limits_rad_s'] == [1.] * 7 and row['acceleration_limits_rad_s2'] == [1.] * 7 and
                row['endpoint_velocity_rad_s'] == [0.] * 7, f'{identity}: motion constraints differ from v1')
        _expect(row['problem_feasibility'] == 'feasibility_unknown', f'{identity}: unsupported feasibility claim')
        _expect(row['validation_status'] in ('valid_input', 'invalid_endpoint', 'invalid_scene_geometry', 'model_consistency_failed'), f'{identity}: unknown validation status')
        _expect(row['validation_status'] != 'model_consistency_failed', f'{identity}: recorded model consistency failure')
        valid_count += row['validation_status'] == 'valid_input'
        for key in ('q_start', 'q_goal'):
            if row[key] is not None:
                q = np.asarray(row[key], dtype=float)
                _expect(q.shape == (7,) and np.isfinite(q).all(), f'{identity}: {key} must contain seven finite values')
        _expect(row['q_start'] is not None or row['q_goal'] is None, f'{identity}: goal without start')
        _expect(row['scene_file'] == f'scenes/{sid}.json', f'{identity}: noncanonical scene path')
        _expect(row['input_arrays_file'] == f'inputs/{identity}.npz', f'{identity}: noncanonical input path')
        for reference, hash_key in (('scene_file', 'scene_sha256'), ('input_arrays_file', 'input_arrays_sha256')):
            relative = row[reference]
            expected_files.add(relative)
            target = safe_path(root, relative)
            _expect(target.is_file() and file_sha256(target) == row[hash_key], f'{identity}: missing or changed {reference}')
            _expect(receipt['files_sha256'].get(relative) == row[hash_key], f'{identity}: {reference} absent from receipt')
        expected_bin = config['distance_bin_cycle'][pair % len(config['distance_bin_cycle'])]
        sampling = row['endpoint_sampling']
        _expect(sampling['distance_bin'] == expected_bin and sampling['distance_bin_bounds'] == config['normalized_distance_bins'][expected_bin], f'{identity}: distance stratum differs from configuration')
        for output_key, config_key in (('joint_margin_rad', 'endpoint_joint_margin_rad'), ('minimum_clearance_m', 'endpoint_clearance_m'), ('maximum_attempts', 'endpoint_max_attempts')):
            _expect(sampling[output_key] == config[config_key], f'{identity}: endpoint sampling policy mismatch')
    _expect(set(receipt['files_sha256']) == expected_files, 'Receipt file inventory is incomplete or contains unreferenced files')
    _expect(receipt['manifest_path'] == f'manifests/{split}.jsonl', 'Receipt manifest path mismatch')
    _expect(receipt['problem_count'] == len(rows) and receipt['scene_count'] == len({r['scene_id'] for r in rows}), 'Receipt counts mismatch')
    _expect(receipt['valid_input_count'] == valid_count and receipt['invalid_input_count'] == len(rows) - valid_count, 'Receipt validity counts mismatch')


def check_endpoint_arrays(row, root, config, model, *, recheck_collisions=True):
    """Replay PCG64 proposals and first-accepted-pair selection; audit all rejects.

    Fresh classifier calls verify saved observations when requested. This is
    input qualification only, and never consults any planning method.
    """
    identity = row['problem_id']
    with np.load(safe_path(root, row['input_arrays_file']), allow_pickle=False) as archived:
        raw = {key: archived[key] for key in archived.files}
    q = raw['endpoint_proposal_q']
    native, clearance = raw['endpoint_proposal_cuda_free'], raw['endpoint_proposal_independent_clearance_m']
    n = len(q)
    _expect(q.shape == (n, 7) and np.isfinite(q).all() and 0 < n <= config['endpoint_max_attempts'], f'{identity}: invalid proposal array')
    _expect(native.shape == (n,) and native.dtype.kind == 'b' and clearance.shape == (n,) and np.isfinite(clearance).all(), f'{identity}: invalid proposal observations')
    margin = config['endpoint_joint_margin_rad']
    expected = rng(row['endpoint_seed']).uniform(model.lower + margin, model.upper - margin, (n, 7))
    _expect(np.array_equal(q, expected), f'{identity}: proposals do not reproduce the frozen endpoint seed')
    if recheck_collisions:
        _expect(np.array_equal(native, model.cuda(q)), f'{identity}: saved native proposal checks differ from actual checks')
        _expect(np.allclose(clearance, model.clearance(q), rtol=0, atol=1e-10), f'{identity}: saved independent proposal distances differ from actual checks')
    mismatch = native != (clearance > 0)
    significant = mismatch & (np.abs(clearance) > model.contact_band_m)
    _expect(not significant.any(), f'{identity}: proposal checker disagreement outside contact band')
    start = goal = None
    considered = rejected_collision = rejected_distance = 0
    lower_bin, upper_bin = row['endpoint_sampling']['distance_bin_bounds']
    for state, free, distance in zip(q, native, clearance):
        considered += 1
        if not free or distance <= config['endpoint_clearance_m']:
            rejected_collision += 1
            continue
        if start is None:
            start = state
            continue
        normalized = float(np.sqrt(np.mean(((state - start) / (model.upper - model.lower)) ** 2)))
        if lower_bin <= normalized < upper_bin:
            goal = state
            break
        rejected_distance += 1
    for name, expected_q in (('q_start', start), ('q_goal', goal)):
        _expect((expected_q is None and row[name] is None) or
                (expected_q is not None and row[name] is not None and np.array_equal(expected_q, row[name])), f'{identity}: {name} is not the first accepted proposal')
    expected_draws = min(config['endpoint_max_attempts'], ((considered - 1) // config['endpoint_batch_size'] + 1) * config['endpoint_batch_size'])
    _expect(n == expected_draws and (goal is not None or n == config['endpoint_max_attempts']), f'{identity}: truncated proposal search')
    values = dict(drawn_configurations=n, considered_configurations=considered,
        unused_batch_tail_configurations=n-considered, collision_or_clearance_rejections=rejected_collision,
        distance_bin_rejections=rejected_distance, checker_disagreements=int(mismatch.sum()),
        checker_disagreements_outside_contact_band=int(significant.sum()))
    _expect(all(row['endpoint_sampling'][key] == value for key, value in values.items()), f'{identity}: saved endpoint sampling statistics mismatch')
    if row['validation_status'] == 'valid_input':
        _expect(goal is not None, f'{identity}: valid input has no accepted goal')
    if row['validation_status'] == 'invalid_endpoint':
        _expect(goal is None, f'{identity}: valid pair mislabeled invalid_endpoint')
    expected_keys = {'endpoint_proposal_q', 'endpoint_proposal_cuda_free', 'endpoint_proposal_independent_clearance_m'}
    if goal is not None:
        expected_keys |= {'straight_line_q', 'straight_line_cuda_free', 'straight_line_independent_clearance_m'}
        count = max(2, int(np.ceil(np.linalg.norm(goal-start) / config['straight_line_step_rad'])) + 1)
        _expect(count <= 1000000, f'{identity}: straight-line check exceeds one million samples')
        line = raw['straight_line_q']
        _expect(line.shape == (count, 7) and np.array_equal(line, np.linspace(start, goal, count)), f'{identity}: straight-line array mismatch')
        flags, distances = raw['straight_line_cuda_free'], raw['straight_line_independent_clearance_m']
        _expect(flags.shape == (count,) and flags.dtype.kind == 'b' and distances.shape == (count,) and np.isfinite(distances).all(), f'{identity}: malformed straight-line observations')
        _expect(row['straight_line_samples'] == count and row['straight_line_step_rad'] == config['straight_line_step_rad'], f'{identity}: straight-line sampling metadata mismatch')
        _expect(row['straight_line_collision'] is bool(not (flags.all() and (distances > 0).all())), f'{identity}: straight-line observation mismatch')
        if recheck_collisions:
            _expect(np.array_equal(flags, model.cuda(line)), f'{identity}: saved native straight-line checks differ from actual checks')
            _expect(np.allclose(distances, model.clearance(line), rtol=0, atol=1e-10), f'{identity}: saved independent straight-line distances differ from actual checks')
        line_mismatch = flags != (distances > 0)
        line_values = dict(straight_line_native_colliding_samples=int((~flags).sum()),
            straight_line_independent_colliding_samples=int((distances <= 0).sum()),
            straight_line_checker_disagreements=int(line_mismatch.sum()),
            straight_line_disagreements_outside_contact_band=int((line_mismatch & (np.abs(distances) > model.contact_band_m)).sum()))
        _expect(all(row[key] == value for key, value in line_values.items()), f'{identity}: straight-line statistics mismatch')
        _expect(row['straight_line_check_is_numerical'] is True and
                np.isclose(row['straight_line_minimum_clearance_m'], distances.min(), rtol=0, atol=1e-12),
                f'{identity}: straight-line minimum clearance or check semantics mismatch')
        endpoints = np.array([start, goal])
        _expect(np.asarray(row['endpoint_clearances_m']).shape == (2,) and
                np.allclose(row['endpoint_clearances_m'], model.clearance(endpoints), rtol=0, atol=1e-10),
                f'{identity}: endpoint clearance metadata mismatch')
        _expect(np.asarray(row['endpoint_tcp_world_m']).shape == (2, 3) and
                np.allclose(row['endpoint_tcp_world_m'], model.tcp(endpoints), rtol=0, atol=1e-10),
                f'{identity}: endpoint TCP metadata mismatch')
    else:
        _expect(row['straight_line_collision'] is None and row['start_goal_joint_distance'] is None and
                row['normalized_start_goal_joint_distance'] is None, f'{identity}: missing endpoints have invented measurements')
    _expect(set(raw) == expected_keys, f'{identity}: unexpected or missing raw arrays')
    return {'endpoint_proposals_replayed': n, 'proposal_collision_checks_repeated': bool(recheck_collisions)}
