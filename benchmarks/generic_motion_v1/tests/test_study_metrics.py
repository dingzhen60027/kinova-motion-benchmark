"""CPU-only accounting fixtures, never experimental or collision evidence.

The real batch integrity verifier is tested elsewhere. These small temporary
receipt fixtures isolate statistical denominators and protocol-pooling rules.
"""
import copy
import json
import math

import numpy as np
import pytest

from benchmarks.generic_motion_v1.runners import study_metrics as study


METHODS = tuple(pair for pair, label in study.LABELS.items() if label in study.PAPER_LABELS)
DISTANCE = (("EI_ZO", "minimum_distance"), ("LGCS", "minimum_distance"))
SUCCESS = "success_validated_numerical"


def _write(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data))


def _read(path):
    return json.loads(path.read_text())


def _case(name, family="sparse_clutter", valid="valid_input"):
    return dict(
        problem_id=name, scene_id="scene-" + name, family=family, split="dev",
        validation_status=valid, q_start=[0.] * 7, q_goal=[.25] + [0.] * 6,
        velocity_limits_rad_s=[1.] * 7, acceleration_limits_rad_s2=[1.] * 7,
        endpoint_velocity_rad_s=[0.] * 7, straight_line_collision=True,
        endpoint_sampling={"distance_bin": "short"},
        start_goal_joint_distance=.25, normalized_start_goal_joint_distance=.1,
        robot_sha256="fixture-robot", collision_geometry_sha256="fixture-geometry",
        problem_sha256="fixture-input-" + name,
    )


@pytest.fixture
def make_run(tmp_path, monkeypatch):
    # Never bypass validation of real evidence: all outputs here are tmp_path
    # fixtures and the returned marker explicitly identifies the substitution.
    monkeypatch.setattr(study, "verify", lambda *a, **k: dict(
        integrity_passed=True, complete=True, fixture_only=True))
    def accounting_acceptance(run,manifest,job,attempt,cache):
        # This fixture has no trajectory coefficients, collision arrays or
        # sealed process evidence. Mock acceptance only for these synthetic
        # statistical inputs; production never exempts fixture markers. Real
        # acceptance integration is tested in test_run_summary_acceptance.
        from benchmarks.generic_motion_v1.runners.results import STATUS
        result=_read(attempt/'result.json');validation_path=attempt/'validation.json'
        validation=_read(validation_path) if validation_path.exists() else {}
        return dict(status=STATUS[result['status']],
                    objective_value=validation.get('cost'),objective_unit=validation.get('cost_unit'))
    monkeypatch.setattr('benchmarks.generic_motion_v1.runners.summarize._accepted_attempt',
                        accounting_acceptance)

    def make(name, cases=None, methods=METHODS, outcomes=None, config=None, sources=None):
        root = tmp_path / name
        cases = cases if cases is not None else [_case("p0")]
        outcomes = outcomes or {}
        roadmap = tmp_path / "fixture-map"
        _write(roadmap / "metadata.json", dict(
            roadmap_id="fixture-map", requested_nodes=6000, actual_nodes=6000, seed=0))
        jobs = []
        for case in cases:
            for method, objective in methods:
                label = study.LABELS[(method, objective)]
                values = outcomes.get((case["problem_id"], label), {})
                legal = case["validation_status"] == "valid_input"
                status = values.get("status", SUCCESS if legal else case["validation_status"])
                cost = values.get("cost", 3.)
                job_id = f"{case['problem_id']}-{label}"
                job = dict(
                    job_id=job_id, case_id=case["problem_id"], case=case,
                    scene_id=case["scene_id"], family=case["family"],
                    method=method, objective=objective, method_seed=12,
                    roadmap_id="fixture-map", roadmap_dir=str(roadmap),
                    evaluation_config=copy.deepcopy(config or {"eizo": {"delta": .005}}),
                )
                jobs.append(job)
                attempt = root / "jobs" / job_id / "attempts" / "0001"
                _write(attempt / "result.json", dict(
                    status=status, frontend_native_success=legal, drm_success=legal,
                    timing=values.get("timing", {"warm_query_seconds": 2.}),
                ))
                _write(attempt / "process.json", dict(
                    cold_e2e_seconds=values.get("cold", 10.), sampled_gpu_peak_mib=None))
                _write(attempt / "receipt.json", dict(status=status, files={}, fixture_only=True))
                _write(attempt / "candidate.json", dict(solver_success=status == SUCCESS))
                _write(attempt / "validation.json", dict(
                    passed=values.get("validation_passed", status == SUCCESS), cost=cost,
                    cost_unit="rad" if objective == "minimum_distance" else "s"))
                if legal:
                    path = np.array([case["q_start"], case["q_goal"]])
                    np.save(attempt / "path.npy", path, allow_pickle=False)
                    _write(attempt / "frontend_validation" / "validation.json", dict(
                        passed=True, cost=float(np.linalg.norm(path[1] - path[0])),
                        minimum_sampled_clearance_m=.01))
        _write(root / "run_manifest.json", dict(
            jobs=jobs, timeout_seconds=60., source_code_commit="fixture-source",
            source_hashes=sources or {
                "model.py": "model-v1", "runners/paper_graph.py": "graph-v1",
                "runners/validate.py": "validator-v1", "runners/summarize.py": "report-v1",
            }))
        return root

    return make


def test_size_groups_use_requested_budget_and_keep_actual_nodes(make_run, tmp_path):
    run = make_run("isolated-node")
    metadata_path = tmp_path / "fixture-map" / "metadata.json"
    metadata = _read(metadata_path)
    metadata["actual_nodes"] = 5999
    _write(metadata_path, metadata)
    report = study.analyze([run], tmp_path / "size-report")
    assert {row["roadmap_nodes"] for row in report["rows"]} == {6000}
    assert {row["actual_roadmap_nodes"] for row in report["rows"]} == {5999}
    assert {group["roadmap_nodes"] for group in report["groups"]} == {6000}
    assert report["drm_references"][0]["actual_roadmap_nodes"] == 5999
    assert report["complete"] and report["comparable"]


@pytest.mark.parametrize("status,failed_stage,exception,candidate,validation", [
    ("unsupported", "trajectory_optimization", "license rejected", False, False),
    ("timeout", None, None, True, True),
    ("infrastructure_error", "collision_recovery", "CUDA error", True, True),
    ("solver_failed", None, None, False, True),
    ("solver_failed", None, None, True, False),
])
def test_incomplete_backend_counters_are_not_completed_stage_measurements(
        status, failed_stage, exception, candidate, validation):
    raw = dict(model_setup_seconds=.1, drm_path_seconds=.2, eizo_inflation_seconds=.3,
        trajectory_optimization_seconds=0., validation_seconds=0., collision_recovery_seconds=0.)
    original = copy.deepcopy(raw)
    selected, complete, excluded, reasons = study.completed_stage_times(raw,
        dict(status=status, failed_stage=failed_stage, exception=exception), candidate, validation)
    assert raw == original
    assert selected == dict(model_setup_seconds=.1, drm_path_seconds=.2)
    assert not complete and reasons
    assert set(excluded) == study.BACKEND_STAGE_KEYS.intersection(raw)


@pytest.mark.parametrize("status", [SUCCESS, "solver_failed", "validation_failed"])
def test_normally_returned_backend_keeps_real_zero_recovery_even_when_failed(status):
    raw = dict(eizo_inflation_seconds=.2, trajectory_optimization_seconds=.5,
               validation_seconds=.1, collision_recovery_seconds=0.)
    selected, complete, excluded, reasons = study.completed_stage_times(
        raw, dict(status=status), True, True)
    assert selected == raw and complete
    assert excluded == reasons == []


def test_stage_aggregation_excludes_initialized_zeros_and_retains_raw_evidence(make_run):
    measured = dict(model_setup_seconds=.1, eizo_inflation_seconds=.2,
                    trajectory_optimization_seconds=2., validation_seconds=1.,
                    collision_recovery_seconds=0.)
    initialized = dict(measured, trajectory_optimization_seconds=0., validation_seconds=0.)
    run = make_run("stage-accounting", [_case("ok"), _case("unsupported"), _case("failed")],
        methods=(("LGCS", "minimum_distance"),), outcomes={
            ("ok", "LGCS"): dict(timing=measured),
            ("unsupported", "LGCS"): dict(status="unsupported", timing=initialized),
            ("failed", "LGCS"): dict(status="solver_failed", timing=measured),
        })
    attempt = run / "jobs/unsupported-LGCS/attempts/0001"
    result = _read(attempt / "result.json")
    result.update(failed_stage="trajectory_optimization", exception="license rejected")
    _write(attempt / "result.json", result)
    (attempt / "candidate.json").unlink()
    (attempt / "validation.json").unlink()
    rows, _ = study.load_rows([run])
    unsupported = next(row for row in rows if row["case_id"] == "unsupported")
    assert unsupported["timing"] == initialized
    assert unsupported["completed_stage_seconds"] == {"model_setup_seconds": .1}
    grouped = study.group_metrics(rows)
    assert grouped["legal_jobs"] == 3
    assert grouped["statistics"]["cold_e2e_seconds"]["all_legal"]["count"] == 3
    assert grouped["complete_backend_timing_jobs"] == 2
    assert grouped["incomplete_backend_timing_jobs"] == 1
    for name in ("eizo_inflation_seconds", "trajectory_optimization_seconds",
                 "validation_seconds", "collision_recovery_seconds"):
        assert grouped["stage_seconds"][name]["all_legal"]["count"] == 2
        assert grouped["stage_seconds"][name]["failed_legal"]["count"] == 1
    assert grouped["stage_seconds"]["trajectory_optimization_seconds"]["all_legal"]["mean"] == 2.
    assert grouped["stage_seconds"]["collision_recovery_seconds"]["all_legal"]["mean"] == 0.
    assert grouped["stage_seconds"]["model_setup_seconds"]["all_legal"]["count"] == 3


@pytest.mark.parametrize("mismatch", ["path", "outcome"])
def test_frontend_mismatch_saved_diagnostics_are_explicitly_not_comparable(
        make_run, tmp_path, monkeypatch, mismatch):
    run = make_run("different-frontend")
    rows, manifests = study.load_rows([run])
    if mismatch == "path":
        rows[0]["path_sha256"] = "different-path"
    else:
        rows[0]["native_drm_success"] = False
    # This only substitutes in-memory statistical fixtures, not real receipts.
    monkeypatch.setattr(study, "load_rows", lambda _: (rows, manifests))
    output = tmp_path / "diagnostic"
    with pytest.raises(ValueError, match="Shared frontend mismatch"):
        study.analyze([run], output)
    diagnostic = _read(output / "metrics.json")
    assert diagnostic["complete"] is False and diagnostic["comparable"] is False
    assert diagnostic["pairing_differences"]
    assert "diagnostic" in diagnostic["incomparability_reason"]


def test_reporting_subset_reuses_only_requested_sealed_observations(make_run, tmp_path):
    run = make_run("report-subset", [_case("p0"), _case("p1")])
    report = study.analyze([run], tmp_path / "subset-report", problem_ids=["p1"])
    assert report["requested_jobs"] == 5
    assert report["reporting_problem_ids"] == ["p1"]
    assert {row["case_id"] for row in report["rows"]} == {"p1"}
    for invalid in ([], ["missing"], ["p1", "p1"], [12]):
        with pytest.raises(ValueError):
            study.analyze([run], tmp_path / "invalid-subset", problem_ids=invalid)


@pytest.mark.parametrize("invalid_status", [
    "invalid_endpoint", "invalid_scene_geometry", "model_consistency_failed"])
def test_invalid_inputs_never_dilute_legal_latency(make_run, invalid_status):
    run = make_run("latency", [_case("ok"), _case("timeout"), _case("bad", valid=invalid_status)],
        methods=DISTANCE[:1], outcomes={
            ("ok", "LSCS"): {"cold": 5., "timing": {"optimization_seconds": 2.}},
            ("timeout", "LSCS"): {"status": "timeout", "cold": 60.1, "timing": {}},
            ("bad", "LSCS"): {"cold": .001, "timing": {"optimization_seconds": .001}},
        })
    rows, _ = study.load_rows([run])
    result = study.group_metrics(rows)
    assert (result["requested_jobs"], result["legal_jobs"], result["invalid_jobs"]) == (3, 2, 1)
    assert result["valid_success"]["estimate"] == .5
    assert result["valid_success"]["denominator"] == 2
    assert result["statistics"]["cold_e2e_seconds"]["all_legal"]["mean"] == pytest.approx(32.55)
    assert result["statistics"]["cold_e2e_seconds"]["failed_legal"]["count"] == 1
    assert result["stage_seconds"]["optimization_seconds"]["all_legal"]["count"] == 1
    cutoff = {v["threshold_seconds"]: v for v in result["observed_output_attainment"]}
    assert cutoff[2.]["estimate"] == 0
    assert cutoff[5.]["estimate"] == cutoff[60.]["estimate"] == .5


def test_open_control_is_separate_and_uses_one_physical_cluster(make_run, tmp_path):
    cases = [_case("obstacle"), _case("open-a", "open_control"), _case("open-b", "open_control")]
    outcomes = {(case["problem_id"], label): {"cold": 1. if case["family"] == "open_control" else 10.}
                for case in cases for label in study.PAPER_LABELS}
    report = study.analyze([make_run("controls", cases, outcomes=outcomes)], tmp_path / "out")
    groups = {g["scope"]: g for g in report["groups"] if g["label"] == "LSCS"}
    assert groups["obstacle_main"]["legal_jobs"] == 1
    assert groups["obstacle_main"]["statistics"]["cold_e2e_seconds"]["all_legal"]["mean"] == 10.
    control = groups["open_control"]
    assert control["legal_jobs"] == 2 and control["legal_scene_clusters"] == 1
    assert control["valid_success"]["ci95"] is None
    assert control["statistics"]["cold_e2e_seconds"]["all_legal"]["mean"] == 1.
    assert len(report["drm_references"]) == 3  # Not five copies of each shared seed.
    assert all(r["seed_cost_rad"] == .25 for r in report["drm_references"])


def test_resolved_pair_subset_is_symmetric_without_changing_primary(make_run):
    cases = [_case(f"p{i}") for i in range(4)]
    outcomes = {
        ("p0", "LGCS"): {"status": "unsupported"},
        ("p1", "LSCS"): {"status": "infrastructure_error"},
        ("p3", "LGCS"): {"status": "solver_failed"},
    }
    rows, _ = study.load_rows([make_run("resolved", cases, DISTANCE, outcomes)])
    pair = study.pair_metrics(rows)[0]  # Alphabetical order: LGCS - LSCS.
    assert (pair["method_a"], pair["method_b"]) == ("LGCS", "LSCS")
    assert pair["paired_legal_jobs"] == pair["valid_success_difference"]["denominator"] == 4
    assert pair["valid_success_difference"]["estimate"] == -.25
    assert pair["resolved_pair_success_difference"]["denominator"] == 2
    assert pair["resolved_pair_success_difference"]["estimate"] == -.5
    assert "selection-biased" in pair["selection_note"]
    lgcs = study.group_metrics([r for r in rows if r["label"] == "LGCS"])
    assert lgcs["valid_success"]["denominator"] == 4
    assert lgcs["conditional_resolved_success"]["denominator"] == 3


def test_cost_and_latency_use_the_same_common_success_cases(make_run):
    cases = [_case(f"p{i}") for i in range(3)]
    outcomes = {
        ("p0", "LSCS"): {"cost": 4., "cold": 10.},
        ("p0", "LGCS"): {"cost": 3., "cold": 14.},
        ("p1", "LSCS"): {"cost": 100., "cold": 1.},
        ("p1", "LGCS"): {"status": "solver_failed", "cost": .01, "cold": 50., "validation_passed": True},
        ("p2", "LSCS"): {"status": "solver_failed", "cost": .01, "cold": 40.},
        ("p2", "LGCS"): {"cost": 50., "cold": 2.},
    }
    rows, _ = study.load_rows([make_run("paired", cases, DISTANCE, outcomes)])
    pair = study.pair_metrics(rows)[0]
    assert pair["common_success_cost_difference"]["estimate"] == -1.
    assert pair["common_success_cold_latency_difference"]["estimate"] == 4.
    assert pair["common_success_cost_difference"]["denominator"] == 1
    assert pair["common_success_cold_latency_difference"]["denominator"] == 1
    assert pair["observed_cold_latency_difference"]["estimate"] == 5.
    assert pair["observed_cold_latency_difference"]["denominator"] == 3
    failed = next(r for r in rows if r["case_id"] == "p1" and r["label"] == "LGCS")
    assert failed["final_validation_passed"] and not failed["success"]
    assert failed["cost"] is None  # A feasible failed-solver candidate cannot enter costs.


def test_pair_metrics_rejects_missing_fixed_task(make_run):
    rows, _ = study.load_rows([make_run("missing-pair", [_case("p0"), _case("p1")], DISTANCE)])
    rows = [r for r in rows if not (r["case_id"] == "p1" and r["label"] == "LGCS")]
    with pytest.raises(ValueError, match="pairing keys"):
        study.pair_metrics(rows)


@pytest.mark.parametrize("field,value", [
    ("input_legal", False), ("input_sha256", "other-input"),
    ("robot_sha256", "other-robot"), ("geometry_sha256", "other-geometry"),
    ("cluster_id", "other-world"), ("budget_seconds", 10.),
    ("family", "open_control"), ("distance_bin", "long"),
    ("straight_line_collision", False),
])
def test_fixed_input_identity_checked_before_legality_and_stratum_filters(
        make_run, tmp_path, monkeypatch, field, value):
    run = make_run("mixed-inputs")
    rows, manifests = study.load_rows([run])
    lgcs = next(row for row in rows if row["label"] == "LGCS")
    lgcs[field] = value
    with pytest.raises(ValueError, match="Fixed-task input mismatch"):
        study.pair_metrics(rows)
    monkeypatch.setattr(study, "load_rows", lambda _: (rows, manifests))
    output = tmp_path / "mixed-input-output"
    with pytest.raises(ValueError, match="Fixed-task input mismatch"):
        study.analyze([run], output)
    assert not (output / "metrics.json").exists()


def test_different_invalid_versions_cannot_disappear_from_pairing_checks(make_run):
    run = make_run("all-invalid", [_case("p0", valid="invalid_endpoint")])
    rows, _ = study.load_rows([run])
    assert study.pair_metrics(rows) == []  # Identical invalid slots are accounting only.
    rows[0]["input_sha256"] = "different-invalid-version"
    with pytest.raises(ValueError, match="input_sha256"):
        study.pair_metrics(rows)


@pytest.mark.parametrize("remove_all", [False, True])
def test_analyze_requires_all_five_methods_for_each_fixed_task(make_run, tmp_path, remove_all):
    run = make_run("five-methods", [_case("p0"), _case("p1")])
    manifest = _read(run / "run_manifest.json")
    manifest["jobs"] = [job for job in manifest["jobs"] if not (
        job["method"] == "LGCS" and (remove_all or job["case_id"] == "p1"))]
    _write(run / "run_manifest.json", manifest)
    match = "Missing or unexpected methods" if remove_all else "Incomplete method pairing"
    with pytest.raises(ValueError, match=match):
        study.analyze([run], tmp_path / "out")
    assert not (tmp_path / "out" / "metrics.json").exists()


@pytest.mark.parametrize("same_manifest", [False, True])
def test_duplicate_observations_cannot_be_counted_twice(make_run, same_manifest):
    run = make_run("duplicate", methods=DISTANCE[:1])
    runs = [run, run]
    if same_manifest:
        manifest = _read(run / "run_manifest.json")
        manifest["jobs"].append(copy.deepcopy(manifest["jobs"][0]))
        _write(run / "run_manifest.json", manifest)
        runs = [run]
    with pytest.raises(ValueError, match="Duplicate study observation"):
        study.load_rows(runs)


@pytest.mark.parametrize("within_run", [False, True])
def test_changed_evaluation_config_cannot_be_pooled(make_run, within_run):
    left = make_run("left-config", [_case("p0")], methods=DISTANCE)
    if within_run:
        manifest = _read(left / "run_manifest.json")
        manifest["jobs"][1]["evaluation_config"]["eizo"]["delta"] = .01
        _write(left / "run_manifest.json", manifest)
        runs = [left]
    else:
        right = make_run("right-config", [_case("p1")], methods=DISTANCE,
                         config={"eizo": {"delta": .01}})
        runs = [left, right]
    with pytest.raises(ValueError, match="Different evaluation configurations"):
        study.load_rows(runs)


@pytest.mark.parametrize("source", ["model.py", "roadmap.py", "batch.py", "runners/csdecomp_baseline.py",
    "runners/native_handoff.py", "runners/paper_nto.py", "runners/paper_lgcs.py",
    "runners/paper_graph.py", "runners/validate.py", "runners/optimization.py"])
def test_changed_algorithm_model_or_validator_source_is_rejected(make_run, source):
    left = make_run("left-source", [_case("p0")], methods=DISTANCE[:1], sources={source: "v1"})
    right = make_run("right-source", [_case("p1")], methods=DISTANCE[:1], sources={source: "v2"})
    with pytest.raises(ValueError, match="Different algorithm/model/validator source protocols"):
        study.load_rows([left, right])


def test_any_source_snapshot_change_is_rejected_under_frozen_pooling_policy(make_run):
    left = make_run("left-report", [_case("p0")], methods=DISTANCE[:1],
                    sources={"model.py": "same", "runners/summarize.py": "v1"})
    right = make_run("right-report", [_case("p1")], methods=DISTANCE[:1],
                     sources={"model.py": "same", "runners/summarize.py": "v2"})
    with pytest.raises(ValueError, match="Different algorithm/model/validator source protocols"):
        study.load_rows([left, right])


def test_different_commits_with_identical_source_snapshots_can_be_pooled(make_run):
    left = make_run("left-commit", [_case("p0")], methods=DISTANCE[:1])
    right = make_run("right-commit", [_case("p1")], methods=DISTANCE[:1])
    manifest = _read(right / "run_manifest.json")
    manifest["source_code_commit"] = "different-commit-same-recorded-sources"
    _write(right / "run_manifest.json", manifest)
    rows, manifests = study.load_rows([left, right])
    assert len(rows) == len(manifests) == 2


def test_disjoint_cases_with_different_wall_budgets_cannot_be_pooled(make_run, tmp_path):
    left = make_run("left-budget", [_case("p0")])
    right = make_run("right-budget", [_case("p1")])
    manifest = _read(right / "run_manifest.json")
    manifest["timeout_seconds"] = 10.
    _write(right / "run_manifest.json", manifest)
    with pytest.raises(ValueError, match="[Bb]udget"):
        study.analyze([left, right], tmp_path / "mixed-budget-output")


@pytest.mark.parametrize("identity", ["robot_sha256", "collision_geometry_sha256"])
def test_disjoint_cases_with_different_canonical_models_cannot_be_pooled(make_run, identity):
    left = make_run("left-model", [_case("p0")], methods=DISTANCE[:1])
    case = _case("p1")
    case[identity] = "different-canonical-model-or-collision-filters"
    right = make_run("right-model", [case], methods=DISTANCE[:1])
    with pytest.raises(ValueError, match="Different canonical robot/geometry identities"):
        study.load_rows([left, right])


@pytest.mark.parametrize("integrity,complete", [(False, True), (True, False)])
def test_unverified_or_unfinished_run_is_rejected(make_run, monkeypatch, integrity, complete):
    run = make_run("unverified", methods=DISTANCE[:1])
    monkeypatch.setattr(study, "verify", lambda *a, **k: dict(
        integrity_passed=integrity, complete=complete, fixture_only=True))
    with pytest.raises(ValueError, match="incomplete/unverified"):
        study.load_rows([run])


def test_endpoint_distance_reference_uses_all_seven_joint_coordinates():
    case = _case("distance")
    case["q_start"] = [1.] * 7
    case["q_goal"] = [2., 3., 4., 5., 6., 7., 8.]
    assert study.objective_reference(case, "minimum_distance") == pytest.approx(math.sqrt(140))
    case["q_start"], case["q_goal"] = case["q_goal"], case["q_start"]
    assert study.objective_reference(case, "minimum_distance") == pytest.approx(math.sqrt(140))


@pytest.mark.parametrize("distance,expected", [(0., 0.), (.25, 1.), (1., 2.), (4., 5.)])
def test_rest_to_rest_time_reference_triangular_and_trapezoidal(distance, expected):
    case = _case("time")
    case["q_goal"] = [-distance] + [0.] * 6
    assert study.objective_reference(case, "minimum_time") == pytest.approx(expected)


def test_time_reference_uses_slowest_axis_and_its_own_limits():
    case = _case("asymmetric-time")
    case["q_goal"] = [.25, 4., 2., 0., 0., 0., 0.]
    case["velocity_limits_rad_s"] = [1., 2., .5, 1., 1., 1., 1.]
    case["acceleration_limits_rad_s2"] = [1., 1., .25, 1., 1., 1., 1.]
    # Axes require 1, 4, and 6 seconds respectively; simultaneous motion uses max.
    assert study.objective_reference(case, "minimum_time") == pytest.approx(6.)


@pytest.mark.parametrize("field,value", [
    ("velocity_limits_rad_s", [0.] * 7),
    ("acceleration_limits_rad_s2", [-1.] * 7),
    ("velocity_limits_rad_s", [float("inf")] * 7),
    ("acceleration_limits_rad_s2", [1.] * 6),
])
def test_invalid_motion_limits_cannot_produce_time_reference(field, value):
    case = _case("bad-limits")
    case[field] = value
    with pytest.raises(ValueError, match="Invalid fixed motion-limit data"):
        study.objective_reference(case, "minimum_time")


@pytest.mark.parametrize("objective", ["minimum_distance", "minimum_time"])
def test_invalid_endpoint_has_no_fabricated_reference(objective):
    case = _case("invalid", valid="invalid_endpoint")
    case["q_start"] = case["q_goal"] = None
    assert study.objective_reference(case, objective) is None
