"""Real CPU physics acceptance and negative controls; no CUDA or planning calls.

The model was exported/cross-checked by the simulation smoke command.  These
tests reuse it rather than rebuilding a different robot for each controller.
"""
from copy import deepcopy
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from benchmarks.generic_motion_v1.simulation import execution
from benchmarks.generic_motion_v1.simulation.reference import _from_candidate


ROOT = Path(__file__).resolve().parents[3]
MODEL = ROOT / "benchmarks/generic_motion_v1/results/mujoco_smoke/model_sparse/model_metadata.json"
SOURCE = ROOT / "benchmarks/generic_motion_v1/results/baseline_smoke/jobs/smoke-sparse_clutter-000-p00-EI_ZO-minimum_time-3826a1bd/attempts/0001"


@pytest.fixture(scope="module")
def physical_fixture():
    pytest.importorskip("mujoco")
    if not MODEL.is_file():
        pytest.skip("Run the canonical MuJoCo model smoke export before the physical acceptance tests")
    metadata = json.loads(MODEL.read_text())
    metadata["xml_path"] = str(MODEL.parent / "model.xml")
    case = json.loads((SOURCE / "input.json").read_text())
    assert metadata["problem_id"] == case["problem_id"]
    cfg = execution.configuration({"settle_seconds": .1})
    assert metadata["settings"]["timestep_s"] == cfg["timestep_s"]
    return metadata, case, cfg


def hold_reference(q, duration=.2):
    controls = np.broadcast_to(np.asarray(q), (1, 4, 7)).copy()
    return _from_candidate(dict(kind="bezier", objective="minimum_time", controls=controls,
                                durations=np.array([duration])), {})


def hold_case(case):
    result = deepcopy(case)
    result["q_goal"] = list(result["q_start"])
    return result


def test_gravity_compensated_hold_is_real_torque_control(physical_fixture, tmp_path):
    metadata, source, cfg = physical_fixture
    case = hold_case(source)
    result = execution._rollout(metadata, hold_reference(case["q_start"]), case, cfg, tmp_path)
    assert result["status"] == "execution_success_numerical", result
    assert result["maximum_tracking_error_rad"] < 1e-8
    assert result["contact_steps"] == 0
    assert result["saturation_steps"] == 0
    with np.load(tmp_path / "rollout.npz") as data:
        assert len(data["time"]) > 100
        np.testing.assert_allclose(np.diff(data["time"]), .002, atol=1e-15)
        assert np.max(np.abs(data["torque"])) > .1
        np.testing.assert_allclose(data["actual_torque"], data["torque"], atol=1e-12)
        assert np.min(data["independent_clearance_m"]) > 0.
        np.testing.assert_allclose(data["qref"], data["qpos"][:, metadata["qpos_ids"]], atol=1e-8)


def test_disabling_control_really_falls_in_gravity(physical_fixture, tmp_path):
    metadata, source, cfg = physical_fixture
    case = hold_case(source)
    result = execution._rollout(metadata, hold_reference(case["q_start"]), case, cfg, tmp_path, disabled=True)
    assert result["status"] == "execution_failed"
    assert not result["checks"]["tracking"]
    assert result["maximum_tracking_error_rad"] > .05
    with np.load(tmp_path / "rollout.npz") as data:
        np.testing.assert_array_equal(data["torque"], 0.)
        assert np.max(np.abs(data["qpos"][-1] - data["qpos"][0])) > .05
        assert np.max(np.abs(data["qvel"])) > .1


def test_effort_clipping_does_not_teleport_back_to_reference(physical_fixture, tmp_path):
    metadata, source, cfg = physical_fixture
    metadata = deepcopy(metadata)
    metadata["effort_limits_nm"] = [.0001] * 7
    case = hold_case(source)
    result = execution._rollout(metadata, hold_reference(case["q_start"]), case, cfg, tmp_path)
    assert result["status"] == "execution_failed"
    assert result["saturation_steps"] > 0
    assert result["maximum_tracking_error_rad"] > .05
    with np.load(tmp_path / "rollout.npz") as data:
        assert np.max(np.abs(data["torque"])) <= .0001 + 1e-15
        assert np.max(np.abs(data["torque_requested"])) > .01
        assert np.max(np.abs(data["actual_torque"])) <= .0001 + 1e-15


@pytest.mark.parametrize("field", ["q_start", "q_goal"])
def test_reference_cannot_change_frozen_endpoints(physical_fixture, tmp_path, field):
    metadata, source, cfg = physical_fixture
    case = hold_case(source)
    ref = hold_reference(case["q_start"])
    case[field][0] += .1
    with pytest.raises(ValueError, match="frozen"):
        execution._rollout(metadata, ref, case, cfg, tmp_path)


def test_step_cap_prevents_silent_partial_execution(physical_fixture, tmp_path):
    metadata, source, cfg = physical_fixture
    case = hold_case(source)
    cfg = dict(cfg, maximum_steps=2)
    with pytest.raises(ValueError, match="step cap"):
        execution._rollout(metadata, hold_reference(case["q_start"]), case, cfg, tmp_path)


def test_actual_contact_is_recorded_as_execution_failure(physical_fixture, tmp_path):
    import mujoco
    metadata, source, cfg = physical_fixture
    case = hold_case(source)
    model = mujoco.MjModel.from_xml_path(metadata["xml_path"])
    data = mujoco.MjData(model)
    data.qpos[metadata["qpos_ids"]] = case["q_start"]
    mujoco.mj_forward(model, data)
    target_id = metadata["canonical_geom_ids"][5]
    center = data.geom_xpos[target_id]
    # An explicitly injected negative fixture; no benchmark source is changed.
    original_xml = Path(metadata["xml_path"])
    tree = ET.parse(original_xml)
    for mesh in tree.findall("./asset/mesh"):
        mesh.set("file", str((original_xml.parent / mesh.attrib["file"]).resolve()))
    ET.SubElement(tree.find("worldbody"), "geom", name="negative_fixture_obstacle", type="sphere",
                  size="0.05", pos=" ".join(map(str, center)), contype="0", conaffinity="0")
    ET.SubElement(tree.find("contact"), "pair", geom1=metadata["canonical_geom_names"][5],
                  geom2="negative_fixture_obstacle", condim="1", margin="0")
    altered = tmp_path / "negative_fixture.xml"
    tree.write(altered)
    metadata = dict(metadata, xml_path=str(altered))
    result = execution._rollout(metadata, hold_reference(case["q_start"], .01), case,
                                dict(cfg, settle_seconds=0.), tmp_path)
    assert result["status"] == "execution_failed"
    assert not result["checks"]["contact_free"]
    assert result["contact_steps"] > 0
    assert result["first_penetrations"][0]["time"] == 0.
    assert result["first_penetrations"][0]["distance_m"] < -.01
    # The extra negative-fixture obstacle is absent from the independent frozen
    # scene, so disagreement must be surfaced rather than voted away.
    assert not result["checks"]["collision_checker_agreement"]


def test_untimed_path_rejected_before_model_or_integration(monkeypatch, tmp_path):
    case = dict(problem_id="test", scene_id="scene", family="open_control", validation_status="valid_input")
    result = dict(problem_id="test", method="RRTCONNECT", status="success_validated_numerical", objective="minimum_distance")
    monkeypatch.setattr(execution, "checked_attempt", lambda _: (case, result, {}))
    monkeypatch.setattr(execution, "_rollout", lambda *args, **kwargs: pytest.fail("An untimed path reached physics integration"))
    source = tmp_path / "source"
    source.mkdir()
    (source / "receipt.json").write_text("{}")
    (source / "input.json").write_text("{}")
    result = execution.run(source, tmp_path / "out")
    assert result["status"] == "unsupported_untimed_path"
    assert (tmp_path / "out/result.json").is_file()


@pytest.mark.parametrize("fault", ["warning", "clock_reset"])
def test_integration_warnings_and_clock_resets_cannot_pass(physical_fixture, tmp_path, monkeypatch, fault):
    import mujoco
    metadata, source, cfg = physical_fixture
    case = hold_case(source)
    original = mujoco.mj_step2

    def injected(model, data):
        original(model, data)
        if fault == "warning":
            data.warning[int(mujoco.mjtWarning.mjWARN_BADQACC)].number += 1
        else:
            data.time = 0.

    monkeypatch.setattr(mujoco, "mj_step2", injected)
    result = execution._rollout(metadata, hold_reference(case["q_start"]), case, cfg, tmp_path)
    assert result["status"] == "execution_failed"
    assert not result["checks"]["integration_complete"]
    assert (tmp_path / "integration_failure.log").is_file()
    expected = "warnings" if fault == "warning" else "clock reset"
    assert expected in (tmp_path / "integration_failure.log").read_text()


def test_missing_source_receipt_is_preserved_as_failure(tmp_path):
    source = tmp_path / "source"
    source.mkdir()
    result = execution.run(source, tmp_path / "out")
    assert result["status"] == "infrastructure_error"
    assert (tmp_path / "out/failure.log").is_file()
    assert (tmp_path / "out/result.json").is_file()
