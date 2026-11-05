"""Current G0 pointers bind a fresh audit and raw evidence, preserving history."""
import json
from types import SimpleNamespace

import pytest

from benchmarks.generic_motion_v1.generator import manifest


@pytest.fixture
def receipt(tmp_path, monkeypatch):
    monkeypatch.setattr(manifest, 'HERE', tmp_path)
    (tmp_path / 'model.py').write_text('fixture model source\n')
    evidence = tmp_path / 'results/new_check'
    evidence.mkdir(parents=True)
    raw = evidence / 'raw_samples.npz'
    raw.write_bytes(b'raw evidence fixture')
    actual = dict(passed=True, model=dict(robot_sha256='robot', collision_geometry_sha256='geometry'),
                  model_source_sha256=manifest.file_sha256(tmp_path / 'model.py'),
                  workspace={'moving_collision_geometry_min': [-1] * 3, 'moving_collision_geometry_max': [1] * 3},
                  raw_samples_sha256=manifest.file_sha256(raw))
    audit = evidence / 'audit.json'
    audit.write_text(json.dumps(actual))
    value = dict(actual, audit_file=str(audit.relative_to(tmp_path)),
                 audit_sha256=manifest.file_sha256(audit), raw_file=str(raw.relative_to(tmp_path)))
    current = tmp_path / 'reports/model_audit/current.json'
    current.parent.mkdir(parents=True)
    current.write_text(json.dumps(value))
    return current, value, raw, audit


def model():
    return SimpleNamespace(robot_sha256='robot', collision_geometry_sha256='geometry')


def test_current_g0_reference_binds_successful_audit_and_raw_samples(receipt):
    current, value, _, _ = receipt
    result = manifest._require_model_audit(model())
    assert result['_receipt_file'] == 'reports/model_audit/current.json'
    assert result['_receipt_sha256'] == manifest.file_sha256(current)
    assert result['workspace'] == value['workspace']


@pytest.mark.parametrize('field', ['audit', 'raw'])
def test_modified_evidence_cannot_qualify_current_model(receipt, field):
    _, _, raw, audit = receipt
    (audit if field == 'audit' else raw).write_bytes(b'changed evidence')
    with pytest.raises(RuntimeError, match='evidence is missing or changed'):
        manifest._require_model_audit(model())


@pytest.mark.parametrize('mutation', ['workspace', 'failed', 'model', 'source', 'raw_hash'])
def test_self_resealed_pointer_cannot_disagree_with_actual_audit(receipt, mutation):
    current, value, _, _ = receipt
    if mutation == 'workspace':
        value['workspace'] = {'different': True}
    elif mutation == 'failed':
        value['passed'] = False
    elif mutation == 'model':
        value['model'] = dict(robot_sha256='other', collision_geometry_sha256='geometry')
    elif mutation == 'source':
        value['model_source_sha256'] = 'other'
    else:
        value['raw_samples_sha256'] = 'other'
    current.write_text(json.dumps(value))
    with pytest.raises(RuntimeError, match='No passing G0 receipt'):
        manifest._require_model_audit(model())


@pytest.mark.parametrize('field', ['audit_file', 'raw_file'])
@pytest.mark.parametrize('reference', ['../outside', '/tmp/outside'])
def test_evidence_paths_cannot_escape_benchmark_root(receipt, field, reference):
    current, value, _, _ = receipt
    value[field] = reference
    current.write_text(json.dumps(value))
    with pytest.raises(RuntimeError, match='escapes benchmark root'):
        manifest._require_model_audit(model())
