"""Adversarial CPU tests; all generated data use an explicit mock robot."""
from __future__ import annotations

import importlib
import json
from pathlib import Path

import numpy as np
import pytest

from benchmarks.generic_motion_v1.generator.configuration import validate_config
from benchmarks.generic_motion_v1.generator.endpoints import generate_endpoints, straight_line
from benchmarks.generic_motion_v1.generator.manifest import DEFAULT_CONFIG, file_sha256
from benchmarks.generic_motion_v1.generator.scene import digest, generate_scene
from benchmarks.generic_motion_v1.generator.voxelize import voxelize
from benchmarks.generic_motion_v1.tests.test_generator import FakeModel

manifest = importlib.import_module('benchmarks.generic_motion_v1.generator.manifest')
auditor = importlib.import_module('benchmarks.generic_motion_v1.generator.audit')


@pytest.fixture
def dataset(tmp_path, monkeypatch):
    monkeypatch.setattr(manifest, '_require_model_audit', lambda model: {'workspace': {
        'moving_collision_geometry_min': [-2, -2, -2],
        'moving_collision_geometry_max': [2, 2, 2]}})
    monkeypatch.setattr(auditor, '_require_model_audit', lambda model: {})
    result = manifest.generate('smoke', 12, tmp_path, model=FakeModel(), source_code_commit='f' * 40)
    path = Path(result['manifest_path'])
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    receipt_path = tmp_path / 'receipts/smoke_generation.json'
    receipt = json.loads(receipt_path.read_text())
    return tmp_path, path, rows, receipt_path, receipt


def reseal(dataset):
    """Self-consistent hashes are not enough: semantic checks must still fail."""
    root, path, rows, receipt_path, receipt = dataset
    for row in rows:
        value = dict(row)
        value.pop('problem_sha256')
        row['problem_sha256'] = digest(value)
    path.write_text(''.join(json.dumps(row, sort_keys=True, separators=(',', ':')) + '\n' for row in rows))
    receipt['manifest_sha256'] = file_sha256(path)
    receipt['files_sha256']['manifests/smoke.jsonl'] = file_sha256(path)
    receipt_path.write_text(json.dumps(receipt))


def check(dataset):
    return auditor.audit(dataset[1], model=FakeModel())


def test_clean_complete_mock_dataset_passes(dataset):
    result = check(dataset)
    assert result['passed'] is True
    assert result['problem_count'] == 10 and result['endpoint_proposals_replayed'] > 0
    assert result['proposal_collision_checks_repeated'] is True


@pytest.mark.parametrize('mutation,error', [
    ('missing_family', 'configured number'),
    ('duplicate_id', 'duplicate or unexpected'),
    ('changed_endpoint_seed', 'endpoint seed mismatch'),
    ('changed_method_seed', 'paired seeds mismatch'),
    ('unknown_status', 'unknown validation status'),
    ('changed_world', 'world frame mismatch'),
    ('changed_limits', 'motion constraints'),
    ('missing_arrays', 'missing or changed input_arrays_file'),
    ('unreferenced_receipt', 'inventory'),
    ('changed_tcp', 'TCP metadata mismatch'),
])
def test_self_resealed_malformed_dataset_fails_closed(dataset, mutation, error):
    root, _, rows, _, receipt = dataset
    if mutation == 'missing_family':
        rows[:] = [r for r in rows if r['family'] != 'route_choice']
    elif mutation == 'duplicate_id':
        rows[1] = dict(rows[0])
    elif mutation == 'changed_endpoint_seed':
        rows[0]['endpoint_seed'] += 1
    elif mutation == 'changed_method_seed':
        rows[0]['paired_seeds'][0]['method_seed'] += 1
    elif mutation == 'unknown_status':
        rows[0]['validation_status'] = 'not_checked'
    elif mutation == 'changed_world':
        rows[0]['world_frame'] = 'another_frame'
    elif mutation == 'changed_limits':
        rows[0]['velocity_limits_rad_s'][0] = 2.
    elif mutation == 'missing_arrays':
        relative = rows[0]['input_arrays_file']
        (root / relative).unlink()
        del receipt['files_sha256'][relative]
    elif mutation == 'unreferenced_receipt':
        receipt['files_sha256']['inputs/ghost.npz'] = 'f' * 64
    elif mutation == 'changed_tcp':
        rows[0]['endpoint_tcp_world_m'][0][0] += 0.1
    reseal(dataset)
    result = check(dataset)
    assert result['passed'] is False
    assert any(error in item for item in result['errors']), result


@pytest.mark.parametrize('array,error', [
    ('endpoint_proposal_q', 'do not reproduce'),
    ('endpoint_proposal_independent_clearance_m', 'saved independent proposal distances'),
    ('straight_line_independent_clearance_m', 'saved independent straight-line distances'),
])
def test_resealed_raw_observation_changes_are_rejected(dataset, array, error):
    root, _, rows, _, receipt = dataset
    relative = rows[0]['input_arrays_file']
    with np.load(root / relative) as stored:
        arrays = {key: stored[key] for key in stored.files}
    arrays[array].flat[0] += 0.1
    np.savez(root / relative, **arrays)
    rows[0]['input_arrays_sha256'] = file_sha256(root / relative)
    receipt['files_sha256'][relative] = rows[0]['input_arrays_sha256']
    reseal(dataset)
    result = check(dataset)
    assert result['passed'] is False
    assert any(error in item for item in result['errors']), result


def test_same_individual_endpoint_in_other_split_detected(dataset):
    root, _, rows, _, _ = dataset
    other = dict(rows[0])
    other.update(problem_id='other-split', scene_id='other-scene', scene_seed=1,
                 endpoint_seed=2, obstacle_geometry_sha256='different', q_goal=[0.] * 7)
    (root / 'manifests/dev.jsonl').write_text(json.dumps(other) + '\n')
    result = check(dataset)
    assert result['passed'] is False
    assert 'individual_endpoint_shared_across_splits' in result['errors']


@pytest.mark.parametrize('field,value', [
    ('endpoint_joint_margin_rad', -.5), ('endpoint_joint_margin_rad', 4.),
    ('endpoint_clearance_m', -.01), ('endpoint_clearance_m', float('nan')),
    ('straight_line_step_rad', 0), ('endpoint_batch_size', 1.5),
    ('endpoint_max_attempts', 0), ('maximum_scene_proposals', True),
    ('voxel_radius_m', .03), ('roi_max_m', [1, float('inf'), 1]),
])
def test_unsafe_config_rejected(field, value):
    config = json.loads(DEFAULT_CONFIG.read_text())
    config[field] = value
    with pytest.raises(ValueError):
        validate_config(config, FakeModel())


def test_negative_joint_margin_never_produces_out_of_limit_endpoints():
    config = json.loads(DEFAULT_CONFIG.read_text())
    config['endpoint_joint_margin_rad'] = -.5
    with pytest.raises(ValueError, match='endpoint_joint_margin'):
        generate_endpoints(FakeModel(), 0, 'medium', config)


@pytest.mark.parametrize('obstacle', [
    dict(obstacle_id='bad', shape='box', center_m=[0, 0, 0], size_m=[.1] * 3, yaw_rad=float('nan')),
    dict(obstacle_id='bad', shape='sphere', center_m=[1e40, 0, 0], radius_m=.1),
    dict(obstacle_id='bad', shape='sphere', center_m=[float(2**60), 0, 0], radius_m=.1),
    dict(obstacle_id='bad', shape='box', center_m=[0, 0, 0], size_m=[1e9] * 3),
])
def test_unrepresentable_or_unbounded_geometry_rejected(obstacle):
    with pytest.raises(ValueError):
        voxelize([obstacle])


def test_malformed_manifest_is_reported_not_accepted(tmp_path):
    path = tmp_path / 'manifests/smoke.jsonl'
    path.parent.mkdir()
    path.write_text('not json')
    result = auditor.audit(path, model=FakeModel())
    assert result['passed'] is False and result['errors']


def test_audit_cannot_overwrite_input(dataset):
    before = dataset[1].read_bytes()
    with pytest.raises(ValueError, match='frozen dataset'):
        auditor.audit(dataset[1], model=FakeModel(), out=dataset[1])
    assert dataset[1].read_bytes() == before


@pytest.mark.parametrize('seed', [True, -1, 1.5, '12'])
def test_root_seed_must_be_losslessly_recorded(tmp_path, seed):
    with pytest.raises(ValueError, match='seed'):
        manifest.generate('smoke', seed, tmp_path, model=FakeModel())
    assert not (tmp_path / 'manifests').exists()


def test_nonfinite_classifier_observations_cannot_create_endpoints():
    config = json.loads(DEFAULT_CONFIG.read_text())
    model = FakeModel()
    model.clearance = lambda q: np.full(len(q), np.nan)
    with pytest.raises(ValueError, match='finite clearance'):
        generate_endpoints(model, 12, 'short', config)


@pytest.mark.parametrize('value', [np.nan, np.inf, -np.inf])
def test_nonfinite_fixed_checker_cannot_accept_layout(value):
    config = json.loads(DEFAULT_CONFIG.read_text())
    model = FakeModel()
    model.fixed_geometry_sphere_clearance = lambda xyz, radius: np.full(len(xyz), value)
    with pytest.raises(ValueError, match='finite clearance per voxel'):
        generate_scene('sparse_clutter', 12, config, model)


def test_fixed_checker_requires_one_observation_per_voxel():
    config = json.loads(DEFAULT_CONFIG.read_text())
    model = FakeModel()
    model.fixed_geometry_sphere_clearance = lambda xyz, radius: np.ones(1)
    with pytest.raises(ValueError, match='per voxel'):
        generate_scene('sparse_clutter', 12, config, model)


@pytest.mark.parametrize('mutation', ['nan', 'infinite', 'wrong_shape', 'integer_native'])
def test_malformed_straight_line_checker_cannot_create_classification(mutation):
    model = FakeModel()
    if mutation == 'integer_native':
        model.cuda = lambda q: np.ones(len(q), dtype=int)
    else:
        model.clearance = lambda q: (np.ones((len(q), 1)) if mutation == 'wrong_shape'
                                     else np.full(len(q), np.nan if mutation == 'nan' else np.inf))
    with pytest.raises(ValueError, match='Straight-line classifiers'):
        straight_line(model, np.zeros(7), np.full(7, .1), .04)


def test_exhausted_disagreeing_checkers_are_model_failure():
    config = json.loads(DEFAULT_CONFIG.read_text())
    config['endpoint_max_attempts'] = 16
    model = FakeModel()
    model.cuda = lambda q: np.zeros(len(q), dtype=bool)
    row, arrays = generate_endpoints(model, 12, 'short', config)
    assert row['q_start'] is None and row['q_goal'] is None
    assert row['validation_status'] == 'model_consistency_failed'
    assert row['endpoint_sampling']['checker_disagreements_outside_contact_band'] == 16
    assert len(arrays['endpoint_proposal_q']) == 16


@pytest.mark.parametrize('passed,current_source,accepted', [
    (True, True, True), ('false', True, False), (True, False, False), (False, True, False),
])
def test_new_g0_receipt_requires_exact_current_source(tmp_path, monkeypatch, passed, current_source, accepted):
    monkeypatch.setattr(manifest, 'HERE', tmp_path)
    (tmp_path / 'model.py').write_text('current model')
    path = tmp_path / 'reports/qualification/model_audit_current.json'
    path.parent.mkdir(parents=True)
    path.write_text(json.dumps(dict(passed=passed, model=dict(
        robot_sha256=FakeModel.robot_sha256, collision_geometry_sha256=FakeModel.collision_geometry_sha256),
        model_source_sha256=file_sha256(tmp_path / 'model.py') if current_source else 'outdated')))
    if accepted:
        receipt = manifest._require_model_audit(FakeModel())
        assert receipt['_receipt_sha256'] == file_sha256(path)
        assert receipt['_receipt_file'] == 'reports/qualification/model_audit_current.json'
    else:
        with pytest.raises(RuntimeError, match='No passing G0'):
            manifest._require_model_audit(FakeModel())
