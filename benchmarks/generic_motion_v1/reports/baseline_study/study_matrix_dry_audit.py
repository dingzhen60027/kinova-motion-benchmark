"""CPU-only matrix accounting; never imports/calls authorize, prepare or a planner."""
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re

from ...common import BASE, ROOT, cases, code_inventory, read, sha, write
from ...roadmap import load_config


def main():
    out = Path(__file__).resolve().parent
    manifest = BASE / "manifests/test.jsonl"
    data = cases(manifest)
    indexed = {case["problem_id"]: case for case in data}
    selected = [case["problem_id"] for case in data if case["family"] != "open_control"
        and int(case["problem_id"].rsplit("-p", 1)[1]) == int(case["scene_id"].rsplit("-", 1)[1]) % 5]
    selected_cases = [indexed[name] for name in selected]
    pairs = [(method, objective) for objective in ("minimum_distance", "minimum_time")
             for method in ("EI_ZO", "LGCS", "NTO_PAPER")
             if not (method == "LGCS" and objective != "minimum_distance")]
    families = ("sparse_clutter", "dense_clutter", "bottleneck", "route_choice", "open_control")
    requests = [(f"main_s{seed:03d}_{family}", "main", 6000, seed,
                 [case["problem_id"] for case in data if case["family"] == family])
                for seed in range(3) for family in families]
    requests += [(f"size_n{nodes}_s{seed:03d}", "size_effect", nodes, seed, selected)
                 for nodes in (3000, 9000, 12000) for seed in range(3)]
    source_hashes = code_inventory()
    config = load_config(BASE / "configs/rss2025_eizo.json")
    all_keys = []
    phase_keys = {"main": set(), "size_effect": set()}
    blocks = []
    for name, phase, nodes, seed, ids in requests:
        roadmap = BASE / f"roadmaps/rss2025_n{nodes}_s{seed:03d}"
        metadata = read(roadmap / "metadata.json")
        assert metadata["status"] == "ready" and metadata["seed"] == seed
        assert metadata["requested_nodes"] == nodes
        files = {str(path.resolve()): sha(path) for path in sorted(roadmap.rglob("*")) if path.is_file()}
        jobs = []
        for case_id in ids:
            case = indexed[case_id]
            pairing = next(pair for pair in case["paired_seeds"] if pair["drm_seed"] == seed)
            for method, objective in pairs:
                identity = json.dumps([case_id, objective, method, str(roadmap)], separators=(",", ":"))
                job_id = f"{case_id}-{method}-{objective}-{hashlib.sha256(identity.encode()).hexdigest()[:8]}"
                jobs.append(dict(job_id=job_id, case_id=case_id, scene_id=case["scene_id"],
                    family=case["family"], method=method, objective=objective,
                    roadmap_id=metadata["roadmap_id"], evaluation_config=config,
                    roadmap_dir=str(roadmap), method_seed=int(pairing["method_seed"]),
                    solver_seed=int(pairing["solver_seed"]), solver_seed_used=False,
                    solver_seed_note="The current solvers start deterministically from the fixed DRM path.", case=case))
                key = (case_id, nodes, seed, method, objective)
                all_keys.append(key)
                phase_keys[phase].add(key)
        # This is a byte-size model only. It is never written as a run manifest,
        # never receives approval, and cannot be passed to any execution API.
        estimate = dict(version=1, source_code_commit="0" * 40,
            created_utc="2026-10-28T00:00:00.000000+00:00", manifest_path=str(manifest),
            manifest_sha256=sha(manifest), source_hashes=source_hashes, roadmap_files=files,
            selection="study-block", timeout_seconds=60., split="test", formal_test_run=True,
            study_protocol=dict(path=str(out / "formal_protocol_v1.json"), sha256="0" * 64,
                block=name, snapshot="study_protocol.json", input_audit_path=str(out / "test_full_reaudit.json"),
                input_audit_sha256="0" * 64, input_audit_snapshot="study_input_audit.json",
                metrics_contract_path=str(BASE / "BASELINE_PROTOCOL.md"), metrics_contract_sha256="0" * 64,
                metrics_contract_snapshot="study_metrics_contract.md"),
            jobs=jobs,
            denominator="All declared legal tasks, including solver/front-end/timeout failures. No replacement.",
            seed_policy="Roadmap PCG64 seed paired with the exposed EI-ZO seed; deterministic solver initialization.")
        size = len((json.dumps(estimate, indent=2, allow_nan=False) + "\n").encode())
        blocks.append(dict(name=name, phase=phase, requested_nodes=nodes, actual_nodes=metadata["actual_nodes"],
            roadmap_seed=seed, case_count=len(ids), registered_jobs=len(jobs),
            legal_jobs=sum(job["case"]["validation_status"] == "valid_input" for job in jobs),
            invalid_jobs=sum(job["case"]["validation_status"] != "valid_input" for job in jobs),
            modeled_run_manifest_bytes=size, conservative_manifest_bytes=size + 16384,
            problem_ids_sha256=hashlib.sha256(json.dumps(ids, separators=(",", ":")).encode()).hexdigest()))
    expected_main = {(case["problem_id"], 6000, seed, method, objective)
                     for case in data for seed in range(3) for method, objective in pairs}
    expected_size = {(case_id, nodes, seed, method, objective) for case_id in selected
                     for nodes in (3000, 9000, 12000) for seed in range(3) for method, objective in pairs}
    reused = {(case_id, 6000, seed, method, objective) for case_id in selected
              for seed in range(3) for method, objective in pairs}
    duplicate_count = len(all_keys) - len(set(all_keys))
    assert len(data) == 420 and sum(case["validation_status"] == "valid_input" for case in data) == 395
    assert len(selected) == len(set(selected)) == len({case["scene_id"] for case in selected_cases}) == 80
    assert Counter(case["family"] for case in selected_cases) == {family: 20 for family in families[:-1]}
    assert sum(case["validation_status"] == "valid_input" for case in selected_cases) == 75
    assert phase_keys["main"] == expected_main and phase_keys["size_effect"] == expected_size
    assert not phase_keys["main"] & phase_keys["size_effect"] and reused <= expected_main
    assert len(blocks) == 24 and len(all_keys) == 9900 and duplicate_count == 0
    assert sum(block["legal_jobs"] for block in blocks) == 9300
    assert sum(block["invalid_jobs"] for block in blocks) == 600
    assert max(block["conservative_manifest_bytes"] for block in blocks) < 100_000_000
    coordinator = out / "run_formal_study.py"
    block_identity_check = bool(re.search(r"manifest\['study_protocol'\]\['block'\]\s*!=\s*name", coordinator.read_text()))
    assert block_identity_check, "Coordinator must validate the specific block, not just protocol SHA/job count"
    result = dict(passed=True, audit_kind="CPU-only input/metadata set and serialization accounting",
        generated_utc=datetime.now(timezone.utc).isoformat(), manifest_sha256=sha(manifest),
        freezer_sha256=sha(out / "freeze_formal_protocol.py"), coordinator_sha256=sha(coordinator),
        script_sha256=sha(__file__), runtime_source_hashes=source_hashes,
        block_count=24, main_block_count=15, scaling_block_count=9,
        registered_jobs=9900, legal_planner_jobs=9300, invalid_input_slots=600,
        main_jobs=6300, scaling_additional_jobs=3600, reused_main_6000_jobs=len(reused),
        duplicate_job_keys=duplicate_count, missing_main_keys=0, missing_scaling_keys=0,
        subset_cases=80, subset_legal_cases=75, subset_invalid_cases=5,
        subset_family_counts=dict(Counter(case["family"] for case in selected_cases)),
        subset_distance_bins=dict(Counter(case["endpoint_sampling"]["distance_bin"] for case in selected_cases)),
        subset_selection="endpoint index = scene index modulo 5; fixed inputs only, no solver outcomes read",
        maximum_manifest_bytes=max(block["modeled_run_manifest_bytes"] for block in blocks),
        maximum_manifest_bound_bytes=max(block["conservative_manifest_bytes"] for block in blocks),
        manifest_size_scope="Current batch.prepare JSON shape; 16 KiB extra allowance per file. No actual run manifest was written.",
        coordinator_block_identity_check_present=block_identity_check,
        authorize_calls=0, prepare_calls=0, freezer_calls=0, planner_calls=0, gpu_calls=0,
        blocks=blocks)
    write(out / "study_matrix_dry_audit.json", result)
    print(json.dumps({key: result[key] for key in ("passed", "registered_jobs", "legal_planner_jobs",
        "invalid_input_slots", "duplicate_job_keys", "maximum_manifest_bytes", "subset_distance_bins")}, indent=2))


if __name__ == "__main__":
    main()
