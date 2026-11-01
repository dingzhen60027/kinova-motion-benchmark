import csv
import json

import numpy as np
import pytest

from benchmarks.generic_motion_v1.runners import trend_audit as audit


def test_unchanged_straight_initialization_is_not_reported_as_improvement():
    path = np.zeros((3, 7)); path[:, 0] = [0, .5, 1]
    result = audit.path_comparison(path, path.copy())
    assert result["relation"] == "unchanged"
    assert result["initial_at_endpoint_lower_bound"]
    assert result["length_relative_delta"] == 0


def test_length_increase_is_preserved_despite_smoothness_solver_success():
    initial = np.zeros((3, 7)); initial[:, 0] = [0, .5, 1]
    final = initial.copy(); final[1, 1] = .1
    result = audit.path_comparison(initial, final)
    assert result["relation"] == "longer"
    assert result["length_delta_rad"] > 0
    assert result["endpoint_change_rad"] == 0


def test_vertex_reduction_can_compare_different_knot_counts():
    initial = np.zeros((3, 7)); initial[:, 0] = [0, .5, 1]; initial[1, 1] = .5
    result = audit.path_comparison(initial, initial[[0, -1]])
    assert result["relation"] == "shorter"
    assert result["maximum_corresponding_knot_change_rad"] is None


def _record(directory, record_id="first", comparison="current_test"):
    directory.mkdir()
    path = np.zeros((3, 7)); path[:, 0] = [0, .5, 1]
    np.savez(directory/"optimizer_initial.npz", initial=path)
    np.savez(directory/"optimizer_solution.npz", points=path)
    (directory/"optimizer_result.json").write_text(json.dumps({"iterations": 1, "stop_reason": "stationary"}))
    return dict(record_id=record_id, method="CHOMP", problem_id="p1", scene_id="s1",
        cluster_id="s1", input_valid="True", status="success_numerical",
        path_length_rad="1", result_path=str(directory/"result.json"),
        selected_comparisons=json.dumps([comparison]))


def test_audit_respects_exact_selection_and_retains_original_status(tmp_path):
    selected = _record(tmp_path/"selected")
    historical = _record(tmp_path/"old", "old", "historical_current_test")
    records = tmp_path/"records.csv"
    with records.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(selected))
        writer.writeheader(); writer.writerows([selected, historical])
    result = audit.audit(records, ["current_test"], tmp_path/"diagnostics")
    assert result["rows"] == 1 and result["diagnostic_errors"] == 0
    assert result["methods"][0]["statuses"] == {"success_numerical": 1}
    assert result["methods"][0]["unchanged_already_straight"] == 1
    with pytest.raises(FileExistsError):
        audit.audit(records, ["current_test"], tmp_path/"diagnostics")


def test_array_summary_disagreement_is_evidence_error_not_hidden(tmp_path):
    record = _record(tmp_path/"run")
    record["path_length_rad"] = ".5"
    result = audit.diagnose(record, "current_test")
    assert result["status"] == "success_numerical"
    assert "disagrees" in result["diagnostic_error"]
    assert result["final_length_rad"] == 1
    assert json.loads(result["artifact_sha256"])


def test_missing_initializer_never_masquerades_as_no_change(tmp_path):
    record = _record(tmp_path/"run")
    (tmp_path/"run/optimizer_initial.npz").unlink()
    result = audit.diagnose(record, "current_test")
    assert result["diagnostic_error"]
    assert "relation" not in result


def test_duplicate_record_identity_refused_before_output(tmp_path):
    record = _record(tmp_path/"run")
    records = tmp_path/"records.csv"
    with records.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(record))
        writer.writeheader(); writer.writerows([record, record])
    with pytest.raises(ValueError, match="Duplicate"):
        audit.audit(records, ["current_test"], tmp_path/"diagnostics")
    assert not (tmp_path/"diagnostics").exists()
