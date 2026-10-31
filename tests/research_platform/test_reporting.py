"""Reporting invariants exercised through real sealed temporary attempts (CPU only)."""

import json
from pathlib import Path
import shutil

import numpy as np
import pytest

from research_platform import reporting, storage

ROOT = Path(__file__).resolve().parents[2]


def make_run(tmp_path, methods=("DRM", "LSCS"), count=1, environment="forest"):
    run = tmp_path / "run"
    run.mkdir()
    cases = [
        dict(id=f"c{i}", environment_seed=i, roadmap_seed=0, roadmap_size=200)
        for i in range(count)
    ]
    jobs = [dict(id=f"p{i}", kind="plan", case_id=f"c{i}") for i in range(count)]
    jobs += [
        dict(
            id=f"c{i}_{method}",
            kind="method",
            case_id=f"c{i}",
            method=method,
            environment_seed=i,
            roadmap_seed=0,
            roadmap_size=200,
            dependencies={"plan": f"p{i}"},
        )
        for i in range(count)
        for method in methods
    ]
    manifest = dict(
        schema_version=1,
        name="report test",
        config={"environment": environment},
        cases=cases,
        jobs=jobs,
        source_hashes={},
    )
    (run / "source_snapshot").mkdir()
    if environment == "fr3":
        relative = "reproduction/paper_constructed/assets/fr3_33.json"
        target = run / "source_snapshot" / relative
        target.parent.mkdir(parents=True)
        shutil.copyfile(ROOT / relative, target)
        manifest["source_hashes"][relative] = storage.sha(target)
    storage.atomic(run / "manifest.json", manifest)
    storage.atomic(
        run / "resolved.json",
        {
            "protocol": {
                "fr3_scene": {
                    "table_center": [0.4, 0, -0.1],
                    "table_size": [2.5, 2.5, 0.2],
                }
            }
        },
    )
    storage.atomic(run / "environment.json", {})
    storage.freeze_inputs(run)
    return run


def attempt(
    run,
    job_id,
    status="success",
    *,
    cost=2.0,
    input_hash="shared",
    solver=True,
    valid=True,
    wall=3.0,
    sealed=True,
    q=None,
    axis=None,
    inputs=None,
):
    out = storage.new_attempt(run, job_id)
    result = dict(
        status=status,
        solver_success=solver,
        validation_passed=valid,
        metrics=dict(cost=cost, cost_type="distance"),
        input_hash=input_hash,
        timing={"wall_seconds": 1.0, "optimization_seconds": 0.2},
        axis_semantics="normalized_path_parameter",
    )
    storage.atomic(out / "result.json", result)
    if inputs is not None:
        storage.atomic(out / "input.json", inputs)
    if q is not None:
        np.savez_compressed(
            out / "trajectory_samples.npz",
            q=q,
            time=axis,
            native=np.ones(len(q)),
            independent=np.ones(len(q)),
        )
    if sealed:
        storage.seal_attempt(out, status, wall_seconds=wall)
    return out


def test_full_denominator_and_external_timeout_time(tmp_path):
    run = make_run(tmp_path, count=6)
    for method in ["DRM", "LSCS"]:
        attempt(run, f"c0_{method}")
        attempt(run, f"c1_{method}", "validation_failed", valid=False)
        attempt(run, f"c2_{method}", "blocked", valid=False, solver=False)
        attempt(run, f"c3_{method}", "timeout", wall=120.0)
        # c4 is never started; c5 retains an interrupted partial result.
        attempt(run, f"c5_{method}", sealed=False)
    summary = reporting.summarize_run(run)
    assert summary["planned_method_jobs"] == 12
    for method in summary["methods"]:
        assert method["requested"] == 6
        assert method["strict_successes"] == 1
        assert method["strict_success_rate"] == 1 / 6
        assert method["attempted"] == 4
        assert method["statuses"]["pending"] == 1
        assert method["statuses"]["interrupted"] == 1
        # Timeout killed after worker wrote success: receipt overrides it.
        assert method["wall_seconds_latest_attempts"]["mean"] == 42.0
    assert summary["paired"][0]["union_cases"] == 6
    assert summary["paired"][0]["common_validated_inputs"] == 1
    assert (run / "reports" / "method_summary.csv").is_file()


def test_paired_requires_same_inputs_and_valid_objective(tmp_path):
    run = make_run(tmp_path, methods=("DRM", "LSCS", "SCSTO"), count=3)
    for index in range(3):
        attempt(run, f"c{index}_DRM", cost=0.0 if index == 0 else 2.0)
        attempt(
            run,
            f"c{index}_LSCS",
            cost=1.0,
            input_hash="changed" if index == 1 else "shared",
            valid=index != 2,
        )
        out = attempt(run, f"c{index}_SCSTO", sealed=False)
        result = storage.read(out / "result.json")
        result["metrics"]["cost_type"] = "time"
        storage.atomic(out / "result.json", result)
        storage.seal_attempt(out, "success")
    summary = reporting.summarize_run(run)
    assert len(summary["paired"]) == 1  # time and distance never paired
    paired = summary["paired"][0]
    assert paired["common_validated_inputs"] == 1
    assert paired["exclusions"] == {
        "different_frozen_inputs": 1,
        "not_both_strict_success": 1,
    }
    assert paired["cost_ratio_a_over_b"]["mean"] == 0.0


def test_latest_partial_attempt_does_not_reuse_old_success(tmp_path):
    run = make_run(tmp_path, methods=("DRM",))
    attempt(run, "c0_DRM")
    attempt(run, "c0_DRM", sealed=False)
    summary = reporting.summarize_run(run)
    assert summary["methods"][0]["strict_successes"] == 0
    assert summary["methods"][0]["statuses"] == {"interrupted": 1}


def test_tampered_result_and_manifest_rejected(tmp_path):
    run = make_run(tmp_path)
    out = attempt(run, "c0_DRM")
    (out / "result.json").write_text("{}")
    with pytest.raises(ValueError, match="Attempt evidence changed"):
        reporting.summarize_run(run)
    (run / "manifest.json").write_text("{}")
    with pytest.raises(ValueError, match="Frozen experiment inputs changed"):
        reporting.summarize_run(run)


def test_replay_failure_axis_and_snapshot_artifacts(tmp_path):
    run = make_run(tmp_path)
    attempt(
        run,
        "p0",
        inputs=dict(
            start=[-4.0, -4.0],
            goal=[4.0, 4.0],
            centers=[[0.0, 0.0]],
            scene={"obstacle_radius": 0.35},
        ),
    )
    q = [[-4.0, -4.0], [-2.0, 1.0], [4.0, 4.0], [0.0, 0.0]]
    attempt(
        run,
        "c0_DRM",
        "validation_failed",
        valid=False,
        q=q,
        axis=[0.0, 0.2, 1.0, np.nan],
    )
    result = reporting.replay_case(run, "c0")
    assert result["rendered_methods"] == ["DRM"]
    assert result["reporting_source_sha256"] == storage.sha(reporting.__file__)
    assert result["missing"] == [
        {"method": "LSCS", "status": "pending", "reason": "no_saved_candidate"}
    ]
    assert result["source_rows"]["DRM"]["excluded_nonfinite_axis_rows"] == 1
    report = (run / "reports" / "replay" / "c0" / "replay.html").read_text()
    assert '"strict_success":false' in report
    assert '"axis_semantics":"normalized_path_parameter"' in report
    assert (run / "reports" / "replay" / "c0" / "replay.png").stat().st_size > 10000
    assert storage.verify_inputs(run)  # reports do not alter frozen experiment inputs


def test_replay_refuses_incompatible_overlay(tmp_path):
    run = make_run(tmp_path, methods=("DRM",))
    attempt(run, "p0", inputs={"start": [0.0, 0.0], "goal": [1.0, 1.0]})
    attempt(
        run,
        "c0_DRM",
        input_hash="different",
        q=[[0.0, 0.0], [1.0, 1.0]],
        axis=[0.0, 1.0],
    )
    with pytest.raises(ValueError, match="different frozen inputs"):
        reporting.replay_case(run, "c0")


def test_fr3_replay_uses_frozen_geometry(tmp_path):
    run = make_run(tmp_path, methods=("DRM",), environment="fr3")
    q0 = [0.0, -0.5, 0.0, -2.0, 0.0, 1.5, 0.7]
    q1 = [0.2, -0.5, 0.0, -2.0, 0.0, 1.5, 0.7]
    attempt(
        run,
        "p0",
        inputs=dict(
            start=q0,
            goal=q1,
            centers=[[0.7, 0.2, 0.4]],
            scene={"obstacle_radius": 0.05},
        ),
    )
    attempt(run, "c0_DRM", q=[q0, q1], axis=[0.0, 1.0])
    result = reporting.replay_case(run, "c0")
    out = run / "reports" / "replay" / "c0"
    assert "workspace.png" in result["files"]
    assert (out / "workspace.png").stat().st_size > 10000
    frozen = (
        run / "source_snapshot" / "reproduction/paper_constructed/assets/fr3_33.json"
    )
    frozen.write_text("{}")
    with pytest.raises(ValueError, match="Frozen experiment inputs changed"):
        reporting.replay_case(run, "c0")


def test_replay_rejects_invalid_axis(tmp_path):
    path = tmp_path / "samples.npz"
    np.savez_compressed(path, q=[[0.0, 0.0], [1.0, 1.0]], time=[1.0, 0.0])
    with pytest.raises(ValueError, match="Invalid saved trajectory"):
        reporting._samples(path)


def test_replay_javascript_controls_and_fk_without_browser(tmp_path):
    """Execute the actual emitted JS; this is not a real browser visual test."""
    import subprocess

    node = shutil.which("node")
    if not node:
        pytest.skip("Node is unavailable; browser-independent JS check not run")
    run = make_run(tmp_path, methods=("DRM",), environment="fr3")
    q0 = [0.0, -0.5, 0.0, -2.0, 0.0, 1.5, 0.7]
    q1 = [0.2, -0.5, 0.0, -2.0, 0.0, 1.5, 0.7]
    attempt(
        run,
        "p0",
        inputs=dict(start=q0, goal=q1, centers=[], scene={"obstacle_radius": 0.05}),
    )
    attempt(run, "c0_DRM", q=[q0, q1], axis=[0.0, 1.0])
    reporting.replay_case(run, "c0")
    page = run / "reports" / "replay" / "c0" / "replay.html"
    script = tmp_path / "check.js"
    script.write_text(r"""
const fs=require('node:fs'),vm=require('node:vm'),assert=require('node:assert/strict');
const source=fs.readFileSync(process.argv[2],'utf8').split('<script>')[1].split('</script>')[0];
const items={},callbacks=[],contexts=[];
const context=new Proxy({}, {get:(target,key)=>target[key]||(()=>{}), set:(target,key,value)=>(target[key]=value,true)});
function element(tag){return {tag,children:[],dataset:{},style:{},value:tag==='progress'?'0':'workspace',clientWidth:1000,clientHeight:640,
 append(...a){this.children.push(...a)},getContext(){return context},setPointerCapture(){},addEventListener(name,fn){this['on'+name]=fn}}}
const sandbox={document:{getElementById(id){return items[id]||(items[id]=element(id))},createElement:element,createTextNode:text=>({text})},
 window:{addEventListener(){}},devicePixelRatio:1,requestAnimationFrame:fn=>callbacks.push(fn),console};
vm.createContext(sandbox);vm.runInContext(source,sandbox);
assert.match(items.state.textContent,/normalized_path_parameter = 0.0000/);
const box=items.legend.children[0].children[0];box.checked=false;box.onchange();assert.equal(items.state.textContent,'');
box.checked=true;box.onchange();items.progress.value='1';items.progress.oninput();assert.match(items.state.textContent,/= 1.0000/);
items.view.value='joints';items.view.onchange();assert.match(items.state.textContent,/= 1.0000/);
items.view.value='workspace';items.view.onchange();
items.canvas.onpointerdown({clientX:0,clientY:0,pointerId:1});items.canvas.onpointermove({clientX:25,clientY:20});
assert.notEqual(vm.runInContext('yaw',sandbox),-.87);items.canvas.onpointerup();items.reset.onclick();assert.equal(vm.runInContext('yaw',sandbox),-.87);
let prevented=false;items.canvas.onwheel({deltaY:-100,preventDefault(){prevented=true}});assert.ok(prevented);assert.ok(vm.runInContext('zoom',sandbox)>1);
items.play.onclick();assert.equal(items.play.textContent,'Pause');assert.equal(items.progress.value,0);
callbacks.shift()(0);callbacks.shift()(5000);assert.equal(+items.progress.value,.5);items.play.onclick();assert.equal(items.play.textContent,'Play');
assert.equal(vm.runInContext('sampleAt({axis:[0,.1,1]},.12)',sandbox),1);
const actual=vm.runInContext('frames(D.traces[0].q[1]).fr3_hand_tcp.map(row=>row[3]).slice(0,3)',sandbox);
const expected=vm.runInContext('D.traces[0].tcp[1]',sandbox);
actual.forEach((v,i)=>assert.ok(Math.abs(v-expected[i])<1e-12));
console.log(JSON.stringify({javascript_controls:'passed',cpu_and_js_fk_max_tolerance:1e-12,browser_rendering:'not_tested'}));
""")
    result = subprocess.run(
        [node, str(script), str(page)], text=True, capture_output=True, check=True
    )
    assert json.loads(result.stdout)["javascript_controls"] == "passed"


def test_failed_shared_seed_is_visible_without_counting_as_method_success(tmp_path):
    run = make_run(tmp_path, methods=("DRM",))
    attempt(
        run,
        "p0",
        "validation_failed",
        valid=False,
        inputs={"start": [0.0, 0.0], "goal": [1.0, 1.0], "centers": []},
        q=[[0.0, 0.0], [1.0, 1.0]],
        axis=[0.0, 1.0],
    )
    attempt(run, "c0_DRM", "blocked", valid=False, solver=False)
    replay = reporting.replay_case(run, "c0")
    assert replay["rendered_methods"] == ["Shared DRM seed (input)"]
    assert replay["missing"][0]["status"] == "blocked"
    summary = reporting.summarize_run(run)
    assert summary["planned_method_jobs"] == 1
    assert summary["methods"][0]["strict_successes"] == 0
    page = (run / "reports" / "replay" / "c0" / "replay.html").read_text()
    assert (
        '"method":"Shared DRM seed (input)","status":"validation_failed","strict_success":false'
        in page
    )


def test_malformed_candidate_does_not_hide_other_saved_methods(tmp_path):
    run = make_run(tmp_path)
    attempt(run, "p0", inputs={"start": [0.0, 0.0], "goal": [1.0, 1.0], "centers": []})
    attempt(run, "c0_DRM", q=[[0.0, 0.0], [1.0, 1.0]], axis=[0.0, 1.0])
    attempt(
        run,
        "c0_LSCS",
        "validation_failed",
        valid=False,
        q=[[0.0, 0.0], [1.0, 1.0]],
        axis=[1.0, 0.0],
    )
    replay = reporting.replay_case(run, "c0")
    assert replay["rendered_methods"] == ["DRM"]
    assert replay["missing"][0]["reason"] == "saved_candidate_not_renderable"
    assert any("c0_LSCS" in name for name in replay["source_artifacts"])


def test_attempt_history_visible_but_only_latest_used_in_statistics(tmp_path):
    import csv

    run = make_run(tmp_path, methods=("DRM",), count=2)
    first = attempt(run, "c0_DRM", "timeout", valid=False, cost=99.0, wall=120.0)
    attempt(run, "c0_DRM", cost=2.0, wall=3.0)
    summary = reporting.summarize_run(run)
    method = summary["methods"][0]
    assert method["requested"] == 2
    assert method["cost_on_strict_success"]["mean"] == 2.0
    assert method["wall_seconds_latest_attempts"]["mean"] == 3.0
    assert "wall_seconds_all_attempts" not in method
    row, pending = summary["jobs"]
    assert row["attempt_number"] == 2
    assert row["attempt_count"] == 2
    assert row["history_statuses"] == [
        {"attempt_number": 1, "status": "timeout", "sealed": True},
        {"attempt_number": 2, "status": "success", "sealed": True},
    ]
    assert pending["attempt_number"] is None
    assert pending["attempt_count"] == 0
    assert pending["history_statuses"] == []
    assert summary["reporting_source_sha256"] == storage.sha(reporting.__file__)
    with (run / "reports" / "summary.csv").open() as stream:
        csv_rows = list(csv.DictReader(stream))
    assert json.loads(csv_rows[0]["history_statuses"]) == row["history_statuses"]
    # A good latest result must not conceal tampered historical evidence.
    (first / "result.json").write_text("{}")
    with pytest.raises(ValueError, match="Attempt evidence changed"):
        reporting.summarize_run(run)
