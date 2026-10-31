"""CPU-only temporary fixtures for immutable development-study identities."""
from copy import deepcopy
import json

import pytest

from benchmarks.generic_motion_v1.reports.paper_comparators_v1 import run_comparison_study as driver


@pytest.fixture
def prepared(tmp_path):
    road = tmp_path / "roadmap"
    road.mkdir()
    metadata = dict(seed=0, roadmap_id="fixture-roadmap-s0")
    (road / "metadata.json").write_text(json.dumps(metadata))
    data = [dict(problem_id=f"dev-fixture-{i}", scene_id=f"dev-fixture-scene-{i}",
                 family="fixture", split="dev", q_start=[float(i)]*7,
                 q_goal=[float(i)+.1]*7,
                 paired_seeds=[dict(drm_seed=0, method_seed=100+i, solver_seed=200+i)])
            for i in range(2)]
    path = tmp_path / "dev.jsonl"
    path.write_text("\n".join(json.dumps(c) for c in data)+"\n")
    methods = [("NTO_PAPER", "minimum_distance"), ("NTO_PAPER", "minimum_time")]
    frozen = dict(runtime_source_hashes={"fixture.py": "fixture-hash"},
                  evaluation_config={"fixture_parameter": 1}, timeout_seconds=60.,
                  declaration=dict(methods=methods))
    block = dict(planned_jobs=4, split="dev", manifest=str(path), roadmap=str(road))
    jobs=[]
    for case in data:
        for method, objective in methods:
            jobs.append(dict(case_id=case["problem_id"], scene_id=case["scene_id"],
                             family=case["family"], method=method, objective=objective,
                             roadmap_dir=str(road), roadmap_id=metadata["roadmap_id"],
                             method_seed=case["paired_seeds"][0]["method_seed"],
                             solver_seed=case["paired_seeds"][0]["solver_seed"],
                             case=deepcopy(case), evaluation_config=frozen["evaluation_config"]))
    manifest=dict(jobs=jobs, source_hashes=frozen["runtime_source_hashes"], split="dev",
                  timeout_seconds=60., manifest_path=str(path), manifest_sha256=driver.sha(path))
    return manifest, block, frozen


def test_exact_complete_fixture_matrix_is_accepted(prepared):
    driver.check_existing_run(*prepared)


@pytest.mark.parametrize("mutation", [
    lambda x: x["jobs"][0].update(case_id="dev-wrong-id"),
    lambda x: x["jobs"][0].update(method_seed=999),
    lambda x: x["jobs"][0].update(solver_seed=999),
    lambda x: x.update(timeout_seconds=61.),
    lambda x: x["jobs"][0]["case"]["q_start"].__setitem__(0, 2.),
    lambda x: x["jobs"][0].update(roadmap_id="different-roadmap"),
    lambda x: x["jobs"][0].update(method="NTO_REFINED"),
    lambda x: x["jobs"].__setitem__(0, deepcopy(x["jobs"][1])),
], ids=["case_id", "method_seed", "solver_seed", "timeout", "changed_endpoint",
        "roadmap_id", "method_replacement", "duplicate_same_count"])
def test_equal_job_count_does_not_hide_changed_identity(prepared, mutation):
    manifest, block, frozen = deepcopy(prepared)
    mutation(manifest)
    assert len(manifest["jobs"]) == block["planned_jobs"]
    with pytest.raises(ValueError):
        driver.check_existing_run(manifest, block, frozen)


@pytest.mark.parametrize("changed", ["config", "roadmap"])
def test_config_and_roadmap_bytes_checked_against_freeze(tmp_path, monkeypatch, changed):
    config = tmp_path / "config.json"
    config.write_text('{"fixture":1}')
    declaration = tmp_path / "development_protocol.json"
    declaration.write_text('{"fixture":"protocol"}')
    roadmap = tmp_path / "roadmap.bin"
    roadmap.write_bytes(b"fixed roadmap fixture")
    monkeypatch.setattr(driver, "OUT", tmp_path)
    monkeypatch.setattr(driver, "CONFIG", config)
    monkeypatch.setattr(driver, "code_inventory", lambda: {})
    frozen = dict(runtime_source_hashes={}, execution_script_sha256=driver.sha(driver.__file__),
                  declaration_sha256=driver.sha(declaration), config_file_sha256=driver.sha(config),
                  data_files={}, roadmap_files={str(roadmap):driver.sha(roadmap)},
                  external_dependency_files={})
    driver.check_frozen(frozen)
    if changed == "config":
        config.write_text('{"fixture":2}')
    else:
        roadmap.write_bytes(b"different roadmap of the same length")
    with pytest.raises(ValueError, match="configuration changed|roadmap_files changed"):
        driver.check_frozen(frozen)
