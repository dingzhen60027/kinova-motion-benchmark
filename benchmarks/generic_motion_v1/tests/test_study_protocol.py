"""Formal-study gate tests use only temporary CPU fixtures, never real approval."""
import copy
import json
from types import SimpleNamespace

import pytest

from benchmarks.generic_motion_v1 import batch, common, study_protocol as study
from benchmarks.generic_motion_v1.roadmap import load_config


def write(path, value):
    common.write(path, value)


@pytest.fixture
def bundle(tmp_path, monkeypatch):
    runtime_sources = {"benchmarks/generic_motion_v1/study_protocol.py": common.sha(study.__file__)}
    monkeypatch.setattr(batch, "code_inventory", lambda: dict(runtime_sources))
    monkeypatch.setattr(study, "code_inventory", lambda: dict(runtime_sources))
    monkeypatch.setattr(batch, "head", lambda: "unit-fixture-not-an-experiment")
    monkeypatch.setattr(batch, "execute", lambda *a, **k: pytest.fail("Do not execute any planner"))
    asset = tmp_path / "robot.asset"
    asset.write_text("CPU unit fixture, not a robot model")
    model = dict(robot_sha256="a" * 64, collision_geometry_sha256="b" * 64,
                 files={str(asset): common.sha(asset)})
    monkeypatch.setattr(study, "current_model_identity", lambda: copy.deepcopy(model))
    data = []
    data_files = {}
    for i in range(3):
        scene = tmp_path / "scenes" / f"s{i}.json"
        arrays = tmp_path / "inputs" / f"p{i}.npz"
        write(scene, {"fixture_scene": i})
        arrays.parent.mkdir(exist_ok=True)
        arrays.write_bytes(f"hash fixture {i}; not numerical collision evidence".encode())
        case = dict(problem_id=f"test-p{i}", split="test", scene_id=f"s{i}",
                    family="sparse_clutter", validation_status="invalid_endpoint" if i == 2 else "valid_input",
                    robot_sha256=model["robot_sha256"], collision_geometry_sha256=model["collision_geometry_sha256"],
                    scene_file=str(scene.relative_to(tmp_path)), scene_sha256=common.sha(scene),
                    input_arrays_file=str(arrays.relative_to(tmp_path)), input_arrays_sha256=common.sha(arrays),
                    paired_seeds=[dict(drm_seed=seed, method_seed=10 + seed, solver_seed=20 + seed)
                                  for seed in (0, 1)])
        data.append(case)
        data_files.update({str(scene): common.sha(scene), str(arrays): common.sha(arrays)})
    manifest = tmp_path / "manifests" / "test.jsonl"
    manifest.parent.mkdir()
    manifest.write_text("".join(json.dumps(case) + "\n" for case in data))
    data_files[str(manifest)] = common.sha(manifest)
    audit_path = tmp_path / "input_audit.json"
    audit = dict(passed=True, manifest_sha256=common.sha(manifest), problem_count=3,
                 valid_input_count=2, invalid_input_count=1, split="test",
                 cases=[dict(problem_id=case["problem_id"], passed=True,
                             validation_status=case["validation_status"]) for case in data])
    write(audit_path, audit)
    maps = [tmp_path / f"roadmap-{i}" for i in (0, 1)]
    for seed, directory in enumerate(maps):
        write(directory / "metadata.json", dict(status="ready", seed=seed, roadmap_id=f"fixture-map-{seed}",
            robot_sha256=model["robot_sha256"], collision_geometry_sha256=model["collision_geometry_sha256"]))
        (directory / "native.map").write_bytes(b"temporary fixture; not a usable native map")
        write(directory / "nested" / "provenance.json", {"fixture": True})
    methods = ["EI_ZO", "NTO_PAPER", "LGCS"]
    objectives = ["minimum_distance", "minimum_time"]
    config = load_config()
    block = dict(selection="all", problem_count=3, problem_ids=[case["problem_id"] for case in data],
                 roadmap_dirs=[str(maps[0])], roadmap_files=study._roadmap_files(maps[:1]),
                 method_pairs=study.method_pairs(methods, objectives), evaluation_config=config,
                 timeout_seconds=60.)
    subset = copy.deepcopy(block)
    subset.update(selection="fixed_subset", problem_count=1, problem_ids=["test-p1"],
                  roadmap_dirs=[str(p) for p in maps], roadmap_files=study._roadmap_files(maps))
    contract = tmp_path / "metrics_contract.md"
    contract.write_text("Unit fixture: invalid inputs never enter legal-task denominators.\n")
    protocol = dict(schema_version=1, state="approved",
        authorization={"text": "UNIT TEST ONLY: synthetic approval fixture, no actual authorization",
                       "source": "pytest tmp_path; no user message"},
        manifest={"path": str(manifest), "sha256": common.sha(manifest)},
        input_audit={"path": str(audit_path), "sha256": common.sha(audit_path)},
        runtime_source_hashes=dict(runtime_sources),
        metrics_contract={"path": str(contract), "sha256": common.sha(contract)},
        model=copy.deepcopy(model), data_files=data_files, blocks={"main": block, "subset": subset})
    protocol_path = tmp_path / "unit_fixture_protocol.json"
    write(protocol_path, protocol)
    return SimpleNamespace(root=tmp_path, manifest=manifest, data=data, maps=maps,
        model=model, asset=asset, audit=audit, audit_path=audit_path,
        contract=contract,
        protocol=protocol, protocol_path=protocol_path, config=config,
        methods=methods, objectives=objectives)


def prepare(b, block="main", **changes):
    args = dict(objectives=b.objectives, methods=b.methods, config=b.config,
                study_protocol=b.protocol_path, study_block=block, timeout=60.)
    args.update(changes)
    roadmaps = changes.pop("roadmaps", b.maps if block == "subset" else b.maps[:1])
    args.pop("roadmaps", None)
    return batch.prepare(b.manifest, roadmaps, b.root / "run", **args)


def save_protocol(b):
    write(b.protocol_path, b.protocol)


def save_audit(b):
    write(b.audit_path, b.audit)
    b.protocol["input_audit"]["sha256"] = common.sha(b.audit_path)
    save_protocol(b)


def rewrite_run_identity(run, manifest):
    write(run / "run_manifest.json", manifest)
    write(run / "identity.json", {"run_manifest_sha256": common.sha(run / "run_manifest.json")})


def test_sealed_legacy_absolute_protocol_survives_clone_without_rewriting(bundle, monkeypatch):
    import shutil
    run = prepare(bundle)
    before = {str(p.relative_to(bundle.root)): common.sha(p)
              for p in bundle.root.rglob('*') if p.is_file()}
    clone = bundle.root.parent / (bundle.root.name + '-relocated')
    shutil.copytree(bundle.root, clone)
    base = clone / 'benchmarks/generic_motion_v1'
    common.write(base / 'provenance/index.json', {
        'recorded_repository_roots': [str(bundle.root)], 'commits': {}})
    monkeypatch.setattr(common, 'ROOT', clone)
    monkeypatch.setattr(common, 'BASE', base)
    monkeypatch.setattr(batch, 'ROOT', clone)
    monkeypatch.setattr(batch, 'BASE', base)
    monkeypatch.setattr(study, 'ROOT', clone)
    checked = batch.verify(clone / run.relative_to(bundle.root))
    assert checked['integrity_passed'], checked
    # Original bindings, authorization and hashes stay byte-for-byte intact.
    assert all(common.sha(clone / name) == digest for name, digest in before.items())
    (clone / bundle.asset.relative_to(bundle.root)).write_text('wrong cloned model')
    assert not batch.verify(clone / run.relative_to(bundle.root))['integrity_passed']
    assert all(common.sha(bundle.root / name) == digest for name, digest in before.items())


def test_test_stays_locked_without_protocol(bundle):
    with pytest.raises(ValueError, match="locked"):
        prepare(bundle, study_protocol=None, study_block=None)
    assert not (bundle.root / "run").exists()


@pytest.mark.parametrize("missing", ["study_protocol", "study_block"])
def test_protocol_and_block_must_be_supplied_together(bundle, missing):
    with pytest.raises(ValueError, match="supplied together"):
        prepare(bundle, **{missing: None})


@pytest.mark.parametrize("mutation", ["draft", "missing-text", "blank-text", "missing-source", "schema"])
def test_missing_real_authorization_fields_remain_locked(bundle, mutation):
    if mutation == "draft":
        bundle.protocol["state"] = "draft"
    elif mutation == "schema":
        bundle.protocol["schema_version"] = True
    elif mutation == "blank-text":
        bundle.protocol["authorization"]["text"] = "  \n"
    else:
        bundle.protocol["authorization"].pop("text" if mutation == "missing-text" else "source")
    save_protocol(bundle)
    with pytest.raises(ValueError):
        prepare(bundle)
    assert not (bundle.root / "run").exists()


def test_approved_fixture_prepares_all_ids_including_invalid_without_execution(bundle):
    run = prepare(bundle)
    manifest = common.read(run / "run_manifest.json")
    assert manifest["formal_test_run"] is True and manifest["selection"] == "study-block"
    assert len(manifest["jobs"]) == 15
    assert sum(common.resolve_case(job)["validation_status"] == "invalid_endpoint" for job in manifest["jobs"]) == 5
    assert (run / "study_protocol.json").read_bytes() == bundle.protocol_path.read_bytes()
    assert (run / "study_input_audit.json").read_bytes() == bundle.audit_path.read_bytes()
    assert manifest["study_protocol"]["sha256"] == common.sha(bundle.protocol_path)
    assert manifest["study_protocol"]["block"] == "main"
    assert not (run / "jobs").exists()
    check = batch.verify(run)
    assert check["integrity_passed"] and not check["complete"]


def test_fixed_subset_uses_original_manifest_and_exact_block_ids(bundle):
    before = bundle.manifest.read_bytes()
    run = prepare(bundle, block="subset")
    manifest = common.read(run / "run_manifest.json")
    assert len(manifest["jobs"]) == 10
    assert {job["case_id"] for job in manifest["jobs"]} == {"test-p1"}
    assert manifest["manifest_path"] == str(bundle.manifest)
    assert bundle.manifest.read_bytes() == before
    assert batch.verify(run)["integrity_passed"]


@pytest.mark.parametrize("override", [{"problem": "test-p0"}, {"selection": "first-per-family"}])
def test_protocol_rejects_runtime_selection_overrides(bundle, override):
    with pytest.raises(ValueError, match="selection must come only"):
        prepare(bundle, **override)


@pytest.mark.parametrize("mutation", ["manifest-hash", "data-hash", "missing-data", "model-id", "model-file", "map-hash", "missing-map"])
def test_wrong_model_data_and_roadmap_hashes_are_rejected(bundle, mutation):
    p = bundle.protocol
    if mutation == "manifest-hash":
        p["manifest"]["sha256"] = "0" * 64
    elif mutation == "data-hash":
        p["data_files"][str(bundle.manifest)] = "0" * 64
    elif mutation == "missing-data":
        p["data_files"].pop(str(bundle.manifest))
    elif mutation == "model-id":
        p["model"]["robot_sha256"] = "0" * 64
    elif mutation == "model-file":
        bundle.asset.write_text("changed canonical asset")
    elif mutation == "map-hash":
        p["blocks"]["main"]["roadmap_files"][str(bundle.maps[0] / "native.map")] = "0" * 64
    else:
        p["blocks"]["main"]["roadmap_files"].pop(str(bundle.maps[0] / "native.map"))
    save_protocol(bundle)
    with pytest.raises(ValueError):
        prepare(bundle)
    assert not (bundle.root / "run").exists()


def test_current_cpu_model_identity_must_match_protocol(bundle, monkeypatch):
    changed = copy.deepcopy(bundle.model)
    changed["collision_geometry_sha256"] = "0" * 64
    monkeypatch.setattr(study, "current_model_identity", lambda: changed)
    with pytest.raises(ValueError, match="Current canonical model"):
        prepare(bundle)


@pytest.mark.parametrize("mutation", ["failed", "manifest", "count", "missing-case", "wrong-status", "invalid-count"])
def test_input_audit_must_pass_for_every_original_id_and_classification(bundle, mutation):
    if mutation == "failed":
        bundle.audit["passed"] = False
    elif mutation == "manifest":
        bundle.audit["manifest_sha256"] = "0" * 64
    elif mutation == "count":
        bundle.audit["problem_count"] = 2
    elif mutation == "missing-case":
        bundle.audit["cases"].pop()
    elif mutation == "wrong-status":
        bundle.audit["cases"][-1]["validation_status"] = "valid_input"
    else:
        bundle.audit["invalid_input_count"] = 0
    save_audit(bundle)
    with pytest.raises(ValueError, match="Input audit"):
        prepare(bundle)


@pytest.mark.parametrize("mutation", ["count", "missing-all", "duplicate", "unknown", "empty"])
def test_frozen_block_ids_cannot_be_missing_duplicated_or_unknown(bundle, mutation):
    block = bundle.protocol["blocks"]["main"]
    if mutation == "count":
        block["problem_ids"].pop()
    elif mutation == "missing-all":
        block["problem_ids"].pop()
        block["problem_count"] = 2
    elif mutation == "duplicate":
        block["problem_ids"][-1] = block["problem_ids"][0]
    elif mutation == "unknown":
        block["problem_ids"][-1] = "test-unknown"
    else:
        block["problem_ids"] = []
        block["problem_count"] = 0
    save_protocol(bundle)
    with pytest.raises(ValueError, match="ID|problem"):
        prepare(bundle)


@pytest.mark.parametrize("override", ["config", "budget", "method", "roadmap"])
def test_requested_protocol_must_match_frozen_block(bundle, override):
    changes = {}
    if override == "config":
        config = copy.deepcopy(bundle.config)
        config["eizo"]["delta"] = .123
        changes["config"] = config
    elif override == "budget":
        changes["timeout"] = 30.
    elif override == "method":
        changes["methods"] = ["EI_ZO"]
    else:
        changes["roadmaps"] = bundle.maps[1:]
    with pytest.raises(ValueError):
        prepare(bundle, **changes)


@pytest.mark.parametrize("artifact", ["original-protocol", "copied-protocol", "original-audit", "copied-audit", "original-contract", "copied-contract", "data", "model", "roadmap"])
def test_verify_rejects_original_or_copied_evidence_mutation(bundle, artifact):
    run = prepare(bundle)
    target = {"original-protocol": bundle.protocol_path, "copied-protocol": run / "study_protocol.json",
              "original-audit": bundle.audit_path, "copied-audit": run / "study_input_audit.json",
              "original-contract": bundle.contract, "copied-contract": run / "study_metrics_contract.md",
              "data": bundle.root / "inputs" / "p0.npz", "model": bundle.asset,
              "roadmap": bundle.maps[0] / "native.map"}[artifact]
    target.write_bytes(target.read_bytes() + b"\n")
    check = batch.verify(run)
    assert not check["integrity_passed"]
    assert any("study protocol" in error for error in check["errors"])
    with pytest.raises(ValueError):
        batch.resume(run)


@pytest.mark.parametrize("mutation", ["drop-job", "case", "config", "seed", "flag", "block"])
def test_rehashed_run_manifest_still_cannot_change_frozen_study(bundle, mutation):
    run = prepare(bundle)
    manifest = common.read(run / "run_manifest.json")
    if mutation == "drop-job":
        manifest["jobs"].pop()
    elif mutation == "case":
        manifest["jobs"][0]["case_ref"]["record_sha256"] = "tampered"
    elif mutation == "config":
        manifest["jobs"][0]["evaluation_config"]["eizo"]["delta"] = .123
    elif mutation == "seed":
        manifest["jobs"][0]["method_seed"] += 1
    elif mutation == "flag":
        manifest["formal_test_run"] = False
    else:
        manifest["study_protocol"]["block"] = "subset"
    rewrite_run_identity(run, manifest)
    check = batch.verify(run)
    assert not check["integrity_passed"]
    assert any("study protocol" in error for error in check["errors"])


@pytest.mark.parametrize("split", ["smoke", "dev"])
def test_legacy_non_test_runs_without_protocol_remain_read_verifiable(bundle, split):
    data = copy.deepcopy(bundle.data)
    for case in data:
        case["split"] = split
    bundle.manifest.write_text("".join(json.dumps(case) + "\n" for case in data))
    run = prepare(bundle, study_protocol=None, study_block=None)
    manifest = common.read(run / "run_manifest.json")
    manifest.pop("study_protocol")  # Actual old receipts predate the new field.
    rewrite_run_identity(run, manifest)
    assert manifest["formal_test_run"] is False
    assert batch.verify(run)["integrity_passed"]


def test_cli_exposes_protocol_for_prepare_only_run(bundle, monkeypatch):
    from benchmarks.generic_motion_v1 import __main__ as cli
    monkeypatch.setattr("sys.argv", ["benchmark", "run", "--manifest", str(bundle.manifest),
        "--roadmaps", str(bundle.maps[0]), "--output", str(bundle.root / "run"),
        "--methods", *bundle.methods, "--study-protocol", str(bundle.protocol_path),
        "--study-block", "main", "--prepare-only"])
    monkeypatch.setattr(batch, "resume", lambda *a, **k: pytest.fail("prepare-only cannot execute"))
    cli.main()
    assert common.read(bundle.root / "run" / "run_manifest.json")["formal_test_run"] is True


def test_inventory_records_gate_and_statistical_module():
    inventory = common.code_inventory()
    assert "benchmarks/generic_motion_v1/study_protocol.py" in inventory
    assert "benchmarks/generic_motion_v1/runners/study_metrics.py" in inventory


@pytest.mark.parametrize("mutation", ["missing", "empty", "wrong-hash", "malformed-hash"])
def test_runtime_sources_are_required_and_cannot_change_after_preregistration(bundle, mutation):
    if mutation == "missing":
        bundle.protocol.pop("runtime_source_hashes")
    elif mutation == "empty":
        bundle.protocol["runtime_source_hashes"] = {}
    else:
        name = next(iter(bundle.protocol["runtime_source_hashes"]))
        bundle.protocol["runtime_source_hashes"][name] = "0" * 64 if mutation == "wrong-hash" else "invalid"
    save_protocol(bundle)
    with pytest.raises(ValueError, match="[Rr]untime"):
        prepare(bundle)


def test_read_verification_uses_run_snapshot_not_current_runtime_inventory(bundle, monkeypatch):
    run = prepare(bundle)
    monkeypatch.setattr(study, "code_inventory", lambda: pytest.fail("Historical read verification cannot depend on current runtime"))
    assert batch.verify(run, current_sources=False)["integrity_passed"]


@pytest.mark.parametrize("mutation", ["missing", "wrong-hash"])
def test_metrics_contract_must_be_bound_before_preparation(bundle, mutation):
    if mutation == "missing":
        bundle.protocol.pop("metrics_contract")
    else:
        bundle.protocol["metrics_contract"]["sha256"] = "0" * 64
    save_protocol(bundle)
    with pytest.raises(ValueError, match="metrics contract"):
        prepare(bundle)


def test_change_during_copy_cannot_return_an_approved_plan(bundle, monkeypatch):
    original = study.copy_evidence
    def copy_then_change_contract(run, binding):
        original(run, binding)
        bundle.contract.write_text("contract changed after approval was copied")
    monkeypatch.setattr(study, "copy_evidence", copy_then_change_contract)
    with pytest.raises(ValueError, match="changed while preparing"):
        prepare(bundle)
    check = common.read(bundle.root / "run" / "preparation_verification.json")
    assert check["integrity_passed"] is False
    assert not (bundle.root / "run" / "jobs").exists()
