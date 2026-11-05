"""Small synthetic accounting fixtures, never reported as planning results."""
import copy
import importlib.util
from pathlib import Path

import pytest

from benchmarks.generic_motion_v1.common import BASE, read, sha, write
from benchmarks.generic_motion_v1.runners.methods import METHODS, objectives

_spec = importlib.util.spec_from_file_location("nto_metis_completion", Path(__file__).with_name("complete.py"))
gate = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(gate)


@pytest.fixture
def cohort(tmp_path, monkeypatch):
    root = tmp_path
    base = root/"benchmarks/generic_motion_v1"
    robot = read(BASE/"configs/robot.json")
    write(base/"configs/robot.json", robot)
    monkeypatch.setattr(gate, "ROOT", root)
    monkeypatch.setattr(gate, "HERE", base/"results/nto_metis")
    monkeypatch.setattr(gate, "BASE", base)
    monkeypatch.setattr(gate,"recorded_path",lambda value: Path(value) if Path(value).is_absolute() else root/value)
    monkeypatch.setattr(gate.subprocess, "check_output", lambda *a, **kw: "commit\n")
    paths = []
    for split, seeds in gate.SPLIT_SEEDS.items():
        case = dict(problem_id=split+"-open_control-000-p00", scene_id=split+"-open_control-000", split=split,
            family="open_control", validation_status="valid_input", problem_sha256="input-"+split,
            robot_sha256=robot["robot_sha256"], collision_geometry_sha256=robot["collision_geometry_sha256"],
            q_start=[0.]*7, q_goal=[0.]*7,
            paired_seeds=[dict(drm_seed=seed,method_seed=41+seed,solver_seed=83+seed) for seed in seeds])
        manifest_path = base/"manifests"/(split+".jsonl")
        manifest_path.parent.mkdir(parents=True, exist_ok=True)
        import json
        manifest_path.write_text(json.dumps(case)+"\n")
        jobs = []
        for method in METHODS:
            for seed in seeds:
                for objective in objectives(method):
                    jobs.append(dict(case_id=case["problem_id"],case=case,method=method,objective=objective,
                        job_id=f"{method}-{objective}-{seed}",replica_seed=seed,method_seed=41+seed,solver_seed=83+seed,
                        roadmap_id="r"+str(seed),roadmap_dir="roadmap"+str(seed),roadmap_used=method not in gate.INDEPENDENT,
                        evaluation_config=dict(online_budget_seconds=60.,nto_refined=dict(minimum_time_control_points=16))))
        old = dict(split=split, jobs=jobs, timeout_seconds=60.,source_code_commit="old",source_hashes={
            "benchmarks/generic_motion_v1/runners/validate.py":"validator"})
        write(base/"results"/("baseline_"+split)/"run_manifest.json",old)
        selected = copy.deepcopy([job for job in jobs if job["method"] == "NTO_REFINED"])
        for job in selected:
            job["evaluation_config"]["ipopt"] = dict(gate.IPOPT)
        new = dict(old,source_code_commit="commit",jobs=selected,manifest_sha256=sha(manifest_path))
        path=base/"results"/("nto_metis_"+split)
        write(path/"run_manifest.json",new)
        write(path/"identity.json",dict(run_manifest_sha256=sha(path/"run_manifest.json")))
        paths.append(path)
    return paths


def rewrite(path, callback):
    file=path/"run_manifest.json"
    document=read(file);callback(document);write(file,document)
    write(path/"identity.json",dict(run_manifest_sha256=sha(file)))


def test_pending_never_exports_scores(cohort, monkeypatch):
    def forbidden(*a,**kw):
        raise AssertionError("Export/solver verification must not run for pending cohorts")
    monkeypatch.setattr(gate.results,"export",forbidden)
    monkeypatch.setattr(gate.batch,"verify",forbidden)
    output=gate.HERE/"monitor"
    result=gate.run(cohort,["NTO_REFINED"],output)
    assert result["pending"] == 10
    assert result["final_score"] is False and result["coverage_only"] is True
    assert list(p.name for p in output.iterdir()) == ["coverage.json"]


def test_coverage_only_never_exports_after_every_job_is_complete(cohort,monkeypatch):
    context=gate.inspect_runs(cohort,["NTO_REFINED"],"commit")
    for row in context["progress"]:
        row.update(pending=0,completed=row["planned_jobs"])
    monkeypatch.setattr(gate,"inspect_runs",lambda *a,**kw:context)
    def forbidden(*a,**kw):
        raise AssertionError("Monitoring must not invoke export or solver evidence checks")
    monkeypatch.setattr(gate.results,"export",forbidden)
    monkeypatch.setattr(gate,"check_solver_options",forbidden)
    output=gate.HERE/"monitor"
    result=gate.run(cohort,["NTO_REFINED"],output,coverage_only=True)
    assert result["pending"] == 0 and result["final_score"] is False
    assert not (output/"summary.csv").exists()


@pytest.mark.parametrize("mutation,match",[
    (lambda d:d["jobs"].pop(),"complete fixed input"),
    (lambda d:d["jobs"].append(copy.deepcopy(d["jobs"][0])),"complete fixed input"),
    (lambda d:d["jobs"][0]["evaluation_config"].update(nto_refined={"early_accept_validated":True}),"Early validated"),
    (lambda d:d["jobs"][0].update(method_seed=999),"frozen planner/solver"),
    (lambda d:d["jobs"][0].update(roadmap_id="different"),"roadmap_id"),
    (lambda d:d["jobs"][0]["evaluation_config"].update(unrelated_tuning=True),"beyond explicit"),
    (lambda d:d["jobs"][0]["evaluation_config"]["ipopt"].update(spral_scaling="matching"),"beyond explicit"),
    (lambda d:d["source_hashes"].update({"benchmarks/generic_motion_v1/runners/validate.py":"other"}),"validator source"),
])
def test_matrix_and_protocol_mutations_rejected(cohort,mutation,match):
    rewrite(cohort[0],mutation)
    with pytest.raises(ValueError,match=match):
        gate.inspect_runs(cohort,["NTO_REFINED"],"commit")


def test_missing_split_and_repeated_run_rejected(cohort):
    with pytest.raises(ValueError,match="All three complete splits"):
        gate.inspect_runs(cohort[:-1],["NTO_REFINED"],"commit")
    with pytest.raises(ValueError,match="Duplicate run path"):
        gate.inspect_runs(cohort+[cohort[0]],["NTO_REFINED"],"commit")


def test_helper_only_commit_can_reuse_identical_bound_runtime(cohort):
    rewrite(cohort[-1],lambda d:d.update(source_code_commit="helper-only-commit"))
    assert gate.inspect_runs(cohort,["NTO_REFINED"],"commit")["progress"][-1]["split"] == "test"
    rewrite(cohort[-1],lambda d:d["source_hashes"].update({"different_algorithm.py":"new"}))
    with pytest.raises(ValueError,match="frozen runtime source"):
        gate.inspect_runs(cohort,["NTO_REFINED"],"commit")


def test_first_terminal_not_later_success_and_unsealed_is_visible(cohort):
    job=read(cohort[0]/"run_manifest.json")["jobs"][0]
    parent=cohort[0]/"jobs"/job["job_id"]/"attempts"
    write(parent/"0001/receipt.json",dict(status="interrupted",retry_pending=True))
    write(parent/"0002/receipt.json",dict(status="timeout"))
    write(parent/"0003/receipt.json",dict(status="success_validated_numerical"))
    (parent/"0004").mkdir()
    selected,unsealed,count=gate.first_terminal(cohort[0],job)
    assert selected.name == "0002" and count == 4 and unsealed[0].endswith("0004")


def test_complete_verification_cannot_accept_partial_or_tampered(monkeypatch):
    for proof in (dict(integrity_passed=True,complete=False),dict(integrity_passed=False,complete=True)):
        monkeypatch.setattr(gate.batch,"verify",lambda *a,**kw:proof)
        with gate.require_complete_verification({}):
            with pytest.raises(ValueError,match="incomplete/unverified"):
                gate.batch.verify(gate.ROOT)


def test_fixed_frontend_mismatch_keeps_end_to_end_and_removes_entire_conditional_split(cohort):
    context=gate.inspect_runs(cohort,["NTO_REFINED"],"commit")
    selection=gate.make_selection(cohort,context,["NTO_REFINED"],[dict(split="test")])
    by_id={s["id"]:s for s in selection["comparisons"]}
    assert "current_test_nto_metis" in by_id
    assert "current_test_nto_metis_fixed_frontend" not in by_id
    assert "current_dev_nto_metis_fixed_frontend" in by_id
    methods=by_id["current_test_nto_metis"]["runs"][0]["methods"]
    assert "NTO_REFINED" not in methods and {"NTO","NTO_PAPER"} <= set(methods)


def test_actual_ipopt_options_and_missing_candidate_evidence(cohort):
    attempt=cohort[0]/"jobs/job/attempts/0001";attempt.mkdir(parents=True)
    entry=dict(attempt=attempt,job=dict(method="NTO_REFINED",objective="minimum_time"))
    (attempt/"candidate.npz").write_bytes(b"synthetic candidate fixture")
    with pytest.raises(ValueError,match="lacks actual option"):
        gate.check_solver_options([entry])
    params=dict(solver="IPOPT",parameters=dict(gate.IPOPT),linear_system=dict(options=dict(gate.IPOPT),
        avoids_matching_order=True,constraints_changed=False,native_library_patched=False,
        native_binary=dict(loaded_matches_package=True,sha256=gate.NATIVE_SHA,drake_version="1.51.1")))
    write(attempt/"nto_parameters.json",params)
    assert gate.check_solver_options([entry])["checked_parameter_files"] == 1
    params["parameters"]["spral_scaling"]="matching"
    write(attempt/"nto_parameters.json",params)
    with pytest.raises(ValueError,match="Actual IPOPT options"):
        gate.check_solver_options([entry])


def test_failed_export_never_publishes_summary(cohort,monkeypatch):
    context=gate.inspect_runs(cohort,["NTO_REFINED"],"commit")
    for row in context["progress"]:
        row.update(pending=0,completed=row["planned_jobs"])
    context["entries"]=[]
    monkeypatch.setattr(gate,"inspect_runs",lambda *a,**kw:context)
    monkeypatch.setattr(gate,"frontend_differences",lambda *a,**kw:[])
    def failed(output,**kw):
        (output/"summary.csv").write_text("Unpublished deliberately failing fixture\n")
        raise ValueError("fixture export failure")
    monkeypatch.setattr(gate.results,"export",failed)
    output=gate.HERE/"comparison"
    with pytest.raises(ValueError,match="fixture export failure"):
        gate.run(cohort,["NTO_REFINED"],output)
    assert not (output/"summary.csv").exists()
    assert len(list(output.parent.glob(".comparison.checking-*/summary.csv"))) == 1


def test_before_after_pairs_first_attempt_and_no_best_of(cohort):
    run=cohort[0]
    original=read(run/"run_manifest.json")
    original["jobs"]=original["jobs"][:1]
    write(run/"run_manifest.json",original)
    case=original["jobs"][0]["case"]
    old=run.parent/"baseline_smoke"
    base=dict(lifecycle="terminal",problem_id=case["problem_id"],scene_id=case["scene_id"],cluster_id="shared",
        split="smoke",category="open_control",method="NTO_REFINED",objective="minimum_distance",replica_seed="0",
        attempt_number="1",input_sha256="input",robot_sha256="robot",geometry_sha256="geometry",validation_protocol_sha256="protocol",
        budget_seconds="60",planner_seed="41",roadmap_id="r0",input_valid="True",status=gate.results.SUCCESS,
        objective_value="5",cold_e2e_seconds="3",source_commit="old",record_id="old",attempt_path="old/0001")
    before=dict(base,run=gate.relative(old))
    after=dict(base,run=gate.relative(run),objective_value="4",cold_e2e_seconds="2",source_commit="new",record_id="new",attempt_path="new/0001")
    later=dict(after,attempt_number="2",objective_value="1",record_id="retry")
    pairs,summary=gate.paired_before_after([before,later,after],[run],{"smoke":old},["NTO_REFINED"])
    assert len(pairs) == 1 and pairs[0]["new_record_id"] == "new"
    assert pairs[0]["common_success_cost_new_minus_old"] == -1
    assert pairs[0]["cold_seconds_new_minus_old"] == -1
    assert pairs[0]["objective_unit"] == "rad"
    assert next(r for r in summary if r["category"] == "all" and r["metric"] == "success_difference")["mean"] == 0


def test_atomic_publication_keeps_selection_inventory_and_relative_links(cohort):
    output=gate.HERE/"comparison"
    staged=output.with_name(".comparison.checking-fixture")
    selection=dict(comparisons=[dict(id="fixture",mode="end_to_end",runs=[str(cohort[0])])])
    write(staged/"selection.json",selection)
    write(staged/"inventory.json",dict(summary_selection=gate.relative(output/"selection.json"),
        summary_selection_sha256=sha(staged/"selection.json"),runs=[dict(run=gate.relative(cohort[0]),
        manifest_sha256=sha(cohort[0]/"run_manifest.json"))]))
    source=cohort[0]/"jobs";source.mkdir(exist_ok=True)
    import os
    for folder in ("logs","trajectories"):
        directory=staged/folder;directory.mkdir()
        (directory/"fixture_run").symlink_to(os.path.relpath(source,directory),target_is_directory=True)
    (staged/"summary.csv").write_text("verified fixture summary\n")
    write(staged/"coverage.json",dict(final_score=True))
    write(output/"coverage.json",dict(pending=1,final_score=False))
    write(output/"failed_check.json",dict(error="retained prior failure fixture"))
    failure_sha=sha(output/"failed_check.json")
    checked=gate.check_export_paths(staged,output)
    assert checked["relocated_directory_links"] == 2
    gate.publish(staged,output)
    assert not staged.exists() and (output/"summary.csv").is_file()
    assert (output/"logs/fixture_run").resolve() == source
    assert (output/"trajectories/fixture_run").resolve() == source
    inventory=read(output/"inventory.json")
    assert gate.recorded_path(inventory["summary_selection"]) == output/"selection.json"
    assert sha(output/"selection.json") == inventory["summary_selection_sha256"]
    assert sha(output/"failed_check.json") == failure_sha
    assert not list(output.parent.glob(".comparison.previous-*"))


def test_publication_failure_restores_pending_view(cohort,monkeypatch):
    output=gate.HERE/"comparison"
    staged=output.with_name(".comparison.checking-fixture")
    write(staged/"verification.json",dict(final_score=True))
    (staged/"summary.csv").write_text("unpublished fixture\n")
    write(output/"coverage.json",dict(pending=1))
    original=Path.rename
    def fail_stage(self,target):
        if self == staged:
            raise OSError("injected rename failure")
        return original(self,target)
    monkeypatch.setattr(Path,"rename",fail_stage)
    with pytest.raises(OSError,match="injected rename failure"):
        gate.publish(staged,output)
    assert read(output/"coverage.json") == dict(pending=1)
    assert not (output/"summary.csv").exists() and staged.exists()


def test_existing_completed_export_is_never_overwritten(cohort):
    output=gate.HERE/"comparison"
    staged=output.with_name(".comparison.checking-fixture")
    write(staged/"verification.json",dict(final_score=True))
    output.mkdir()
    (output/"summary.csv").write_text("original fixture export\n")
    before=sha(output/"summary.csv")
    with pytest.raises(ValueError,match="already contains artifacts"):
        gate.publish(staged,output)
    assert sha(output/"summary.csv") == before and staged.exists()


def test_moved_export_rejects_broken_inventory_selection_hash(cohort):
    output=gate.HERE/"comparison"
    staged=output.with_name(".comparison.checking-fixture")
    write(staged/"selection.json",dict(comparisons=[]))
    write(staged/"inventory.json",dict(summary_selection=gate.relative(output/"selection.json"),summary_selection_sha256="wrong"))
    with pytest.raises(ValueError,match="selection reference/hash"):
        gate.check_export_paths(staged,output)


def test_selection_retains_every_existing_extension_baseline(cohort):
    context=gate.inspect_runs(cohort,["NTO_REFINED"],"commit")
    selection=gate.make_selection(cohort,context,["NTO_REFINED"],[])
    for split in ("dev","test"):
        spec=next(s for s in selection["comparisons"] if s["id"] == "current_"+split+"_nto_metis")
        declared=[]
        for item in spec["runs"]:
            manifest=read(gate.ROOT/item["path"]/"run_manifest.json")
            declared.extend(gate.job_key(j) for j in manifest["jobs"] if j["method"] in item["methods"])
        expected=gate.expected_keys(context["datasets"][split],METHODS,gate.SPLIT_SEEDS[split])
        assert len(declared) == len(set(declared)) and set(declared) == expected
        assert {"CHOMP","STOMP","TRAJOPT","RRTCONNECT","RRTSTAR"} <= {k[2] for k in declared}
        conditional=next(s for s in selection["comparisons"] if s["id"] == "current_"+split+"_nto_metis_fixed_frontend")
        conditional_methods=set().union(*(set(item["methods"]) for item in conditional["runs"]))
        assert conditional_methods == set(METHODS)-gate.INDEPENDENT


def test_export_discovers_selected_external_run_outside_supplied_roots(cohort,monkeypatch):
    # Exercise the real export/discovery orchestration. Only reading attempts
    # and statistics are test doubles; no solver/data-quality claim is made.
    external=cohort[0].parent/"external_extension_profile"
    write(external/"run_manifest.json",dict(jobs=[],split="test"))
    selection=gate.HERE/"external_selection_fixture.json"
    write(selection,dict(comparisons=[dict(id="fixture_external",mode="end_to_end",
        runs=[dict(path=str(external),methods=["RRTCONNECT","CHOMP","STOMP","TRAJOPT"])])]))
    discovered=[]
    def collect(runs,verify_files=True):
        assert verify_files is True
        discovered.extend(runs)
        return [],[]
    monkeypatch.setattr(gate.results,"collect",collect)
    monkeypatch.setattr(gate.results,"summarize",lambda *a,**kw:[])
    gate.results.export(gate.HERE/"external_export_fixture",roots=cohort,selection=selection)
    assert set(discovered) == set(cohort+[external])
