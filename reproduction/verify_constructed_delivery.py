"""Check saved delivery evidence without rerunning a planner or changing results."""

import ast
import hashlib
import json
from pathlib import Path
import re
import subprocess

from reproduction.common import ROOT, atomic, read
from reproduction.paper_constructed.run import require_identity
from reproduction.hardware_constructed.run import check_identity

OUT = ROOT / "reproduction/reports/paper_constructed"
BASE_HEAD = "da370fa93d682db1ced2e5817b41ff90f38a1440"


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def command(argv):
    result = subprocess.run(argv, cwd=ROOT, text=True, capture_output=True)
    return dict(command=argv, exit_code=result.returncode,
                stdout=result.stdout.strip(), stderr=result.stderr.strip())


def main():
    checks = {}
    index = read(OUT / "run_index.json")
    for family, entry in index.items():
        run = Path(entry["run_dir"])
        require_identity(read(run / "manifest.json"))
        summary = read(OUT / family / "summary.json")
        audit = read(OUT / "audit" / family / "audit.json")
        resume = read(OUT / "audit" / family / "resume_check.json")
        assert summary["requested"] == audit["cases"] == 400
        assert not audit["issues"] and not summary["artifact_issues"]
        assert not summary["source_integrity_issues"]
        assert resume["exit_code"] == 0 and not resume["changed_case_files"]
        checks[family] = dict(requested=400, case_files=audit["case_files"],
                             resume_unchanged=True, source_runtime_identity=True)
    hardware = read(OUT / "hardware_simulation/report_audit.json")
    check_identity(read(Path(hardware["run_dir"]) / "manifest.json"))
    assert hardware["status"] == "passed"
    assert hardware["report_sha256"] == sha(OUT / "hardware_simulation/report.md")
    assert hardware["formal"]["full_declared_condition"]
    checks["hardware"] = dict(requested=hardware["formal"]["requested"],
        strict_virtual_successes=hardware["formal"]["strict_virtual_successes"],
        accounting_audit=hardware["status"], physical_experiment=False)
    replay = read(OUT / "official_replay/summary.json")
    assert replay["status"] == "completed" and replay["errors"] == 0
    assert not replay["incomplete_scope"]
    assert read(OUT / "official_replay/input_reverification.json")["passed"]
    checks["official_replay"] = dict(checked=replay["checked_trajectories"],
                                     errors=replay["errors"], input_identity=True)
    ledger = read(OUT / "verification_commands.json")
    test = next(row for row in ledger if row["label"] == "frozen_candidate_tests")
    assert test["exit_code"] == 0 and "32 passed" in (ROOT / test["log"]).read_text()
    checks["numerical_tests"] = dict(passed=32, log=test["log"], sha256=sha(ROOT / test["log"]))
    qa = read(OUT / "figure_qa.json")
    assert qa["source_sha256"] == sha(ROOT / "reproduction/paper_constructed/plot.py")
    for item in qa["figures"]:
        assert item["pdf_sha256"] == sha(ROOT / "reproduction/figures/paper_constructed" / (item["name"] + ".pdf"))
        assert item["pdf_text_audit"]["below_minimum_count"] == 0
    checks["figures"] = dict(pdf_figures=len(qa["figures"]), source_checks=qa["source_checks"]["summary"])
    sources = set((ROOT / "reproduction").glob("*constructed*.py"))
    for package in ("paper_constructed", "hardware_constructed"):
        sources.update((ROOT / "reproduction" / package).glob("*.py"))
    for path in sources:
        ast.parse(path.read_text(), filename=str(path))
    checks["python_syntax"] = dict(files=len(sources), passed=True)
    documents = [ROOT / "README.md", ROOT / "reproduction/README.md",
                 OUT / "reproduction_report.md", OUT / "hardware_simulation/report.md"]
    missing = []
    for path in documents:
        for target in re.findall(r"\]\(([^)]+)\)", path.read_text()):
            target = target.strip("<>").split("#", 1)[0]
            if not target or ":" in target:
                continue
            if not (path.parent / target).exists():
                missing.append(dict(document=str(path), target=target))
    assert not missing, missing
    checks["local_document_links"] = dict(documents=len(documents), missing=missing)
    branch = command(["git", "branch", "--show-current"])
    assert branch["stdout"] == "main" and branch["exit_code"] == 0
    old = command(["git", "diff", "--name-only", BASE_HEAD, "--",
                   "reproduction/paper_completion", "reproduction/reports/paper_completion"])
    assert old["exit_code"] == 0 and not old["stdout"]
    old_bundle = ROOT / "reproduction/artifacts/csdecomp_paper_completion_20261025.tar.gz"
    old_hash = sha(old_bundle)
    assert old_hash == "0c5c8a3d5861e04810cc1cfc58e9d49d5cd8359972d2a22dd2a11f2ff9c8d939"
    checks["preservation"] = dict(base_head=BASE_HEAD, branch=branch,
        historical_tracked_source_report_diff=old, historical_bundle_sha256=old_hash)
    permission = command(["gh", "repo", "view", "wernerpe/csdecomp", "--json", "nameWithOwner,viewerPermission,url"])
    checks["upstream_permission"] = permission
    atomic(OUT / "delivery_verification.json", dict(status="passed", checks=checks,
        scope="Saved evidence, identities, accounting and delivery links; does not rerun numerical experiments or establish original-paper equivalence."))
    print(json.dumps(checks, indent=2))


if __name__ == "__main__":
    main()
