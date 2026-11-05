"""Historical read-only evidence remains readable after source/config changes."""
from copy import deepcopy
import importlib.util
from pathlib import Path

import pytest


@pytest.fixture
def control_reader(monkeypatch):
    root=Path(__file__).resolve().parents[1]/'results/rigor_checks/dev_nto_incumbent'
    monkeypatch.syspath_prepend(str(root))
    spec=importlib.util.spec_from_file_location('historical_control_reader',root/'control_summary.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    plan=module.sm.read(module.ctl.PLAN)
    enabled=module.sm.read(module.ctl.ENABLED/'run_manifest.json')
    return module,plan,enabled


def test_historical_control_reader_ignores_current_config_and_source(control_reader,monkeypatch,tmp_path):
    module,plan,enabled=control_reader
    # A future valid configuration can differ; no historical file is touched.
    future=tmp_path/'future_evaluation.json'
    future.write_text('{"online_budget_seconds": 30}')
    monkeypatch.setattr(module.ctl,'CONFIG',future)
    def forbidden(*a,**k):
        raise AssertionError('Read-only history must not require current runtime source')
    monkeypatch.setattr(module.ctl,'assert_sources',forbidden)
    module.verify_saved_plan(plan,enabled)


def test_historical_config_hash_cannot_be_replaced(control_reader):
    module,plan,enabled=control_reader
    bad=deepcopy(plan);bad['control_config_sha256']='0'*64
    with pytest.raises(ValueError,match='configuration hash'):
        module.verify_saved_plan(bad,enabled)


def test_historical_resolved_job_setting_cannot_be_changed(control_reader):
    module,plan,enabled=control_reader
    bad=deepcopy(enabled)
    bad['jobs'][0]['evaluation_config']['drm']['online_edge_step_size']=.02
    with pytest.raises(ValueError,match='original overrides|configuration differs'):
        module.verify_saved_plan(plan,bad)


def test_historical_geometry_selection_still_rejects_changed_question(control_reader):
    module,plan,enabled=control_reader
    bad=deepcopy(plan);bad['problems'][0]['problem_id']='different_question'
    with pytest.raises(ValueError,match='geometry-only'):
        module.verify_saved_plan(bad,enabled)
