"""CPU synthetic provenance probes, explicitly not planning evidence."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import tempfile

import numpy as np
import pytest

from benchmarks.generic_motion_v1 import batch
from benchmarks.generic_motion_v1.common import read, write, sha
from benchmarks.generic_motion_v1.runners.summarize import _row
from benchmarks.generic_motion_v1.tests.test_batch import prepared


def main(output):
    report = {"scope": "Synthetic CPU accounting fixtures; no robot or GPU calls, not planning results.",
              "source_hashes": {str(p): sha(p) for p in [Path(batch.__file__),
                  Path(__file__).resolve().parents[2] / "runners/summarize.py"]}}
    with tempfile.TemporaryDirectory(prefix="model-accounting-probes-") as directory, pytest.MonkeyPatch.context() as patch:
        root = Path(directory)
        run, _, _ = prepared(root, patch)
        job = read(run / "run_manifest.json")["jobs"][0]
        attempt = run / "jobs" / job["job_id"] / "attempts/0001"
        write(attempt / "request.json", job)
        write(attempt / "process.json", {"exit_code": 0, "timed_out": False})
        write(attempt / "validation.json", {"passed": True, "status": "success_validated_numerical", "cost": 2., "cost_unit": "rad"})
        np.savez(attempt / "trajectory_samples.npz", q=np.zeros((2, 7)))
        np.savez(attempt / "candidate.npz", points=np.zeros((2, 7)))
        write(attempt / "result.json", {"status": "success_validated_numerical", "success": True,
              "method": "WRONG_METHOD", "problem_id": "WRONG_CASE",
              "validation": {"cost": 999., "cost_unit": "rad"}})
        write(attempt / "receipt.json", {"status": "success_validated_numerical", "files":
            {p.name: sha(p) for p in attempt.iterdir() if p.is_file()}})
        report["sealed_conflicting_provenance"] = batch.verify(run)
        report["summary_cost_source"] = {"independent_validation_cost": 2.,
             "conflicting_result_validation_cost": 999., "observed_row_cost": _row(run, job)["cost"]}
        original = subprocess.Popen
        script = ("import json,pathlib,sys; out=pathlib.Path(sys.argv[1]);"
                  "(out/'result.json').write_text(json.dumps(dict(status='frontend_failed',success=False,"
                  "problem_id='WRONG_CASE',method='WRONG_METHOD')))")
        patch.setattr(batch.subprocess, "Popen", lambda argv, **kw: original(
            [sys.executable, "-c", script, argv[-1]], **kw))
        patch.setattr(batch, "_gpu_memory", lambda pid: None)
        result = batch.execute(job, run, 10.)
        preservation_paths = [attempt.parent / "0002" / name for name in
            ("result_before_provenance_rejection.json", "result_before_identity_rejection.json")]
        report["worker_identity_conflict"] = {"returned_status": result["status"],
            "returned_method": result.get("method"), "returned_problem_id": result.get("problem_id"),
            "original_conflict_preserved": any(p.exists() for p in preservation_paths),
            "preserved_originals": {p.name: read(p) for p in preservation_paths if p.exists()}}
    write(output, report)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    main(parser.parse_args().output)
