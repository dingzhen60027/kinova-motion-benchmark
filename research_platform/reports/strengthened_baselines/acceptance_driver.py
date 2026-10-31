"""Run a fixed, small baseline acceptance grid and retain command evidence.

Run from the repository using its existing Conda launcher. This is development
acceptance, not a benchmark ranking. Numerical failures remain in the grid.
"""
import argparse
import datetime
import hashlib
import json
from pathlib import Path
import re
import subprocess
import time


ROOT = Path(__file__).resolve().parents[3]


def write(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def inventory(path):
    return {str(p.relative_to(path)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted((path / "jobs").rglob("*")) if p.is_file()}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--label", required=True)
    args = parser.parse_args()
    if not re.fullmatch(r"[a-z0-9_]+", args.label):
        raise ValueError("label must contain lowercase letters, digits and underscores")
    evidence = Path(__file__).parent / args.label
    evidence.mkdir(exist_ok=False)
    commands = []
    checks = []
    launcher = ["bash", "local/python.sh"]

    def command(name, arguments):
        record = dict(name=name, argv=arguments, cwd=str(ROOT),
                      started_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(), status="running")
        commands.append(record)
        write(evidence / "commands.json", commands)
        print("START " + name, flush=True)
        started = time.perf_counter()
        with (evidence / (name + ".log")).open("w") as stream:
            try:
                result = subprocess.run(arguments, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT, timeout=1800)
                record.update(exit_code=result.returncode, status="finished")
            except subprocess.TimeoutExpired:
                record.update(exit_code=None, status="command_timeout")
        record["wall_seconds"] = time.perf_counter() - started
        write(evidence / "commands.json", commands)
        checks.append(dict(name=name, passed=record["exit_code"] == 0))
        print("END " + name + " " + str(record["exit_code"]), flush=True)
        return record["exit_code"] == 0

    cli = launcher + ["-m", "research_platform"]
    if not command("tests", launcher + ["-m", "pytest", "tests/research_platform", "-q"]):
        write(evidence / "checks.json", checks)
        return 1
    command("doctor", cli + ["doctor", "--probe", "--output", str(evidence / "doctor.json")])
    specifications = [
        ("regression", "forest_baseline_regression", "e000_r000_n00200", 1),
        ("forest", "forest_baselines", "e010_r000_n00200", 4),
        ("fr3", "fr3_baselines", "e010_r000_n03000", 2),
    ]
    write(evidence / "protocol.json", dict(
        purpose="fixed development acceptance; not formal performance evaluation",
        development_cases=6, known_regression_cases=1, methods_per_case=8,
        selection="Forest consecutive seeds10-13, FR3 consecutive seeds10-11; Figure6 separate known regression",
        replacement_policy="never replace blocked, failed or timed-out cases",
        external_job_budget_seconds=120, strong_nto_cumulative_budget_seconds=30,
        shared_input="same model, scene, endpoints, originating DRM seed and final validator within each case",
        source_driver_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()))
    for name, config, replay, count in specifications:
        run = ROOT / "runs" / (args.label + "_" + name)
        command(name + "_config", cli + ["config", "configs/research/" + config + ".json",
            "--output", str(evidence / (name + "_resolved.json"))])
        command(name + "_run", cli + ["run", "--config", "configs/research/" + config + ".json", "--output", str(run)])
        command(name + "_verify", cli + ["verify", "--run", str(run)])
        command(name + "_replay", cli + ["replay", "--run", str(run), "--case", replay])
        before = inventory(run)
        command(name + "_resume", cli + ["resume", "--run", str(run)])
        after = inventory(run)
        unchanged = before == after
        checks.append(dict(name=name + "_resume_preserves_all_job_files", passed=unchanged, files=len(before)))
        write(evidence / (name + "_resume.json"), dict(unchanged=unchanged, before=before, after=after))
        summary = json.loads((run / "reports" / "summary.json").read_text())
        manifest = json.loads((run / "manifest.json").read_text())
        checks.append(dict(name=name + "_complete_denominator", passed=sum(j["kind"] == "method" for j in manifest["jobs"]) == count*8))
        for source in ["research_platform/strong_nto.py", "research_platform/continuous_checks.py"]:
            checks.append(dict(name=name + "_frozen_" + Path(source).stem, passed=source in manifest["source_hashes"]))
        write(evidence / (name + "_summary.json"), summary)
    for old in ["forest", "fr3", "batch"]:
        run = ROOT / "runs" / ("platform_final_v4_" + old)
        if run.is_dir():
            command("historical_" + old + "_verify", cli + ["verify", "--run", str(run)])
    write(evidence / "checks.json", dict(passed=all(c["passed"] for c in checks), checks=checks))
    return 0 if all(c["passed"] for c in checks) else 1


if __name__ == "__main__":
    raise SystemExit(main())
