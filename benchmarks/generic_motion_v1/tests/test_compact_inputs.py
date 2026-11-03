"""Compact input references preserve exact tasks and old sealed-run readers."""
import copy
import json
from pathlib import Path

import pytest

from benchmarks.generic_motion_v1 import batch, common
from benchmarks.generic_motion_v1.runners import replay, results, summarize


@pytest.fixture
def prepared(tmp_path, monkeypatch):
    monkeypatch.setattr(batch, "code_inventory", lambda: {})
    data = []
    for i in range(2):
        data.append(dict(problem_id=f"p{i}", scene_id=f"s{i}", family="sparse_clutter", split="dev",
            validation_status="valid_input", q_start=[0.] * 7, q_goal=[1.] * 7,
            robot_sha256="robot", collision_geometry_sha256="geometry", problem_sha256=f"input{i}",
            voxels_xyz=[[float(n), float(n), float(n)] for n in range(1000)],
            paired_seeds=[dict(drm_seed=s, method_seed=100+s, solver_seed=200+s) for s in range(3)]))
    source = tmp_path / "inputs.jsonl"
    source.write_text("".join(json.dumps(case)+"\n" for case in data))
    run = batch.prepare(source, [], tmp_path / "run", objectives=["minimum_distance"],
                        methods=["RRTCONNECT", "RRTSTAR"], seed_indices=[0, 1, 2])
    manifest = common.read(run / "run_manifest.json")
    return run, source, data, manifest


def test_new_plan_has_reference_only_and_same_cases(prepared):
    run, _, data, manifest = prepared
    assert manifest["version"] == 2
    assert len(manifest["jobs"]) == 12
    assert all("case" not in j and "case_ref" in j for j in manifest["jobs"])
    assert all("voxels_xyz" not in json.dumps(j) for j in manifest["jobs"])
    cache = {}
    assert {common.resolve_case(j, cache)["problem_id"] for j in manifest["jobs"]} == {r["problem_id"] for r in data}
    assert len(cache) == 1
    assert batch.verify(run)["integrity_passed"]


def test_inline_old_job_is_unchanged():
    case = dict(problem_id="old", q_start=[0.] * 7)
    assert common.resolve_case(dict(case=case)) is case


def test_no_ambiguous_double_input_format(prepared):
    job = dict(prepared[3]["jobs"][0], case={})
    with pytest.raises(ValueError, match="both case and case_ref"):
        common.resolve_case(job)


@pytest.mark.parametrize("field,value", [("record_sha256", "altered"), ("problem_sha256", "different"), ("scene_id", "other"), ("problem_id", "missing")])
def test_input_reference_changes_are_rejected(prepared, field, value):
    job = copy.deepcopy(prepared[3]["jobs"][0])
    job["case_ref"][field] = value
    with pytest.raises(ValueError, match="Referenced"):
        common.resolve_case(job)


def test_changed_source_bytes_rejected_on_new_operation(prepared):
    _, source, _, manifest = prepared
    job = manifest["jobs"][0]
    assert common.resolve_case(job)
    source.write_text(source.read_text()+"\n")
    with pytest.raises(ValueError, match="manifest changed"):
        common.resolve_case(job)


def test_job_header_cannot_change_scene(prepared):
    job = dict(prepared[3]["jobs"][0], scene_id="another")
    with pytest.raises(ValueError, match="Job metadata differs"):
        common.resolve_case(job)


def test_verify_refuses_reference_to_another_manifest_even_identical_bytes(prepared, tmp_path):
    run, source, _, manifest = prepared
    copied = tmp_path / "other.jsonl"
    copied.write_bytes(source.read_bytes())
    manifest["jobs"][0]["case_ref"]["manifest_path"] = str(copied)
    common.write(run / "run_manifest.json", manifest)
    common.write(run / "identity.json", dict(run_manifest_sha256=common.sha(run / "run_manifest.json")))
    result = batch.verify(run)
    assert not result["integrity_passed"]
    assert any("not bound to the run input manifest" in e for e in result["errors"])


def test_shared_cache_loads_manifest_once(prepared, monkeypatch):
    _, source, _, manifest = prepared
    original = Path.read_bytes
    count = [0]
    def tracked(path):
        if path == source:
            count[0] += 1
        return original(path)
    monkeypatch.setattr(Path, "read_bytes", tracked)
    cache = {}
    for job in manifest["jobs"]:
        common.resolve_case(job, cache)
    assert count == [1]


def test_legacy_plan_still_verifies(prepared):
    run, _, _, manifest = prepared
    for job in manifest["jobs"]:
        case = common.resolve_case(job)
        del job["case_ref"]
        job["case"] = case
    manifest["version"] = 1
    manifest.pop("input_storage")
    common.write(run / "run_manifest.json", manifest)
    common.write(run / "identity.json", dict(run_manifest_sha256=common.sha(run / "run_manifest.json")))
    assert batch.verify(run)["integrity_passed"]


def test_worker_receives_identical_full_case_without_expanding_request(prepared, monkeypatch):
    from benchmarks.generic_motion_v1.runners import csdecomp_baseline
    run, _, data, manifest = prepared
    job = manifest["jobs"][0]
    attempt = run / "jobs" / job["job_id"] / "attempts/0001"
    common.write(attempt / "request.json", job)
    seen = []
    def fake_run(case, roadmap, out, **kwargs):
        seen.append(case)
        return dict(status="solver_failed", success=False)
    monkeypatch.setattr(csdecomp_baseline, "run_case", fake_run)
    monkeypatch.setattr(batch.sys, "platform", "cpu-test-fixture")
    batch.worker(attempt)
    assert seen == [data[0]]
    assert "case" not in common.read(attempt / "request.json")
    assert common.read(attempt / "result.json")["status"] == "solver_failed"


def test_failure_replay_resolves_reference_without_saved_input(prepared):
    run, _, data, manifest = prepared
    job = manifest["jobs"][0]
    attempt = run / "jobs" / job["job_id"] / "attempts/0001"
    common.write(attempt / "result.json", dict(status="timeout"))
    case, result, _ = replay._inputs(attempt)
    assert case == data[0]
    assert result["status"] == "timeout"


def test_new_pending_denominators_and_legacy_summary(prepared, monkeypatch):
    run, _, _, manifest = prepared
    monkeypatch.setattr(batch, "verify", lambda *a, **k: dict(integrity_passed=True))
    selected, planned = results._selected_rows([], dict(id="new", runs=[str(run)]))
    assert not selected and len(planned) == 12
    assert all(p["case"]["validation_status"] == "valid_input" for p in planned)
    table = results.summarize([], dict(comparisons=[dict(id="new", runs=[str(run)])]))
    status = [row for row in table if row.get("row_type") == "status"]
    by_key = {(row["method"], row["category"]): row for row in status}
    assert len(by_key) == len(status) == 4
    assert set(by_key) == {(method, category) for method in ("RRTCONNECT", "RRTSTAR")
                           for category in ("all", "sparse_clutter")}
    for row in by_key.values():
        assert row["planned_jobs"] == row["legal_jobs"] == row["pending_jobs"] == 6
        assert row["actual_attempts"] == row["finished_legal_jobs"] == 0
        assert row["invalid_input_jobs"] == row["frontend_excluded_jobs"] == 0
        assert row["success_numerical"] == row["scene_clusters"] == 0
        assert row["success_rate"] is row["success_ci_low"] is row["success_ci_high"] is None
    monkeypatch.setattr(summarize, "_verify_run", lambda *a, **k: dict(integrity_passed=True))
    legacy = summarize.summarize(run)
    assert legacy["planned_jobs"] == 12
    assert legacy["finished_jobs"] == 0


@pytest.mark.parametrize("inline", [False, True])
def test_reference_manifest_rejects_missing_reference_or_inline_fallback(prepared, inline):
    run, _, _, manifest = prepared
    job = manifest["jobs"][0]
    case = common.resolve_case(job)
    del job["case_ref"]
    if inline:
        job["case"] = case
    common.write(run / "run_manifest.json", manifest)
    common.write(run / "identity.json", dict(run_manifest_sha256=common.sha(run / "run_manifest.json")))
    check = batch.verify(run)
    assert not check["integrity_passed"]
    assert "requires only case_ref" in check["errors"][0]
