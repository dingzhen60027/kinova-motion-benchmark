"""Audit side effects must not replace the benchmark's canonical model."""
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from benchmarks.generic_motion_v1 import model as module


@pytest.fixture
def inventory(tmp_path, monkeypatch):
    canonical = tmp_path / 'configs/robot.json'
    canonical.parent.mkdir()
    value = {'robot_sha256': 'fixture', 'geometries': [{'size': .1}]}
    canonical.write_text(json.dumps(value))
    monkeypatch.setattr(module, 'HERE', tmp_path)
    monkeypatch.setattr(module, 'Model', lambda: SimpleNamespace(inventory=value))
    return canonical, value


@pytest.mark.parametrize('directory_has_file', [False, True])
def test_existing_audit_directory_refused_before_model_construction(tmp_path, monkeypatch, directory_has_file):
    target = tmp_path / 'existing'
    target.mkdir()
    if directory_has_file:
        (target / 'audit.json').write_bytes(b'unique old evidence')
    before = {p.name: p.read_bytes() for p in target.iterdir()}

    def forbidden():
        raise AssertionError('Existing directory must be rejected before constructing the model')

    monkeypatch.setattr(module, 'Model', forbidden)
    with pytest.raises(FileExistsError):
        module.audit(target)
    assert {p.name: p.read_bytes() for p in target.iterdir()} == before


def test_different_canonical_inventory_fails_without_replacing_it(inventory, tmp_path):
    canonical, observed = inventory
    canonical.write_bytes(b'{"robot_sha256":"other","geometries":[]}\n')
    before = canonical.read_bytes()
    target = tmp_path / 'mismatch_audit'
    report = module.audit(target)
    assert report['passed'] is False
    assert report['errors'] == ['canonical_inventory_mismatch']
    assert canonical.read_bytes() == before
    assert json.loads((target / 'model_inventory.json').read_text()) == observed
    assert json.loads((target / 'audit.json').read_text()) == report
    assert not (target / 'raw_samples.npz').exists()


def test_missing_canonical_inventory_is_explicit_bootstrap_failure(inventory, tmp_path):
    canonical, observed = inventory
    canonical.unlink()
    target = tmp_path / 'missing_audit'
    report = module.audit(target)
    assert report['passed'] is False
    assert report['errors'] == ['canonical_inventory_missing_or_unreadable']
    assert report['inventory_comparison']['canonical_sha256_before'] is None
    assert not canonical.exists()
    assert json.loads((target / 'model_inventory.json').read_text()) == observed
    assert 'explicitly initialize' in report['bootstrap_policy']


def test_malformed_canonical_inventory_is_preserved_and_failure_saved(inventory, tmp_path):
    canonical, _ = inventory
    canonical.write_bytes(b'not json')
    target = tmp_path / 'malformed_audit'
    report = module.audit(target)
    assert report['passed'] is False
    assert report['errors'] == ['canonical_inventory_missing_or_unreadable']
    assert canonical.read_bytes() == b'not json'
    assert (target / 'audit.json').is_file()


def test_default_existing_canonical_report_is_never_overwritten(inventory, tmp_path):
    target = tmp_path / 'reports/model_audit'
    target.mkdir(parents=True)
    evidence = target / 'audit.json'
    evidence.write_bytes(b'existing canonical report')
    with pytest.raises(FileExistsError):
        module.audit()
    assert evidence.read_bytes() == b'existing canonical report'
