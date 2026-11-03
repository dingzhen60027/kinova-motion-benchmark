"""Serial GPU jobs with an external deadline and immutable attempt receipts."""
from pathlib import Path
import datetime
import fcntl
import hashlib
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time

from .common import (BASE, ROOT, cases, code_inventory, head, read, sha, write,
                     case_reference, case_manifest_path, resolve_case, recorded_path)
from .runners.methods import METHODS, INDEPENDENT, objectives as supported_objectives
from .runners.failure_classification import (INFRASTRUCTURE_STATUSES, exception_needs_attention,
    handoff_failure_evidence, unexpected_process_exit)

TERMINAL_STATUSES = {"success_validated_numerical", "timeout", "invalid_endpoint",
    "invalid_scene_geometry", "model_consistency_failed", "collision_failed", "limit_failed",
    "solver_failed", "frontend_failed", "validation_undetermined", "infrastructure_error", "unsupported",
    "planner_timeout", "planner_failed"}


def _positive_timeout(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
        raise ValueError("timeout must be a positive finite number")


def _unique(values, name, allowed=None):
    if not values or len(set(values)) != len(values):
        raise ValueError(f"{name} must be nonempty and contain no duplicates")
    if allowed is not None and not set(values) <= allowed:
        raise ValueError(f"Unsupported {name}: {set(values) - allowed}")


def _result_identity(job):
    identity=dict(problem_id=job['case_id'], scene_id=job['scene_id'], family=job['family'],
                method=job['method'], objective=job['objective'], roadmap_id=job['roadmap_id'],
                method_seed=job['method_seed'])
    if job.get('execution_mode')=='fixed_initial_regions':
        identity.update(execution_mode=job['execution_mode'],initialization_digest=job['initialization_digest'])
    return identity


def prepare(manifest, roadmaps, output, *, objectives, methods, selection="all", problem=None,
            timeout=60., config=None, study_protocol=None, study_block=None, seed_indices=None,
            allow_test=False, snapshot_sources=True, mode="end_to_end", initial_source_run=None):
    _positive_timeout(timeout)
    if mode not in ('end_to_end','fixed_initial_regions'):
        raise ValueError('Unsupported execution mode')
    if (mode=='fixed_initial_regions') != (initial_source_run is not None):
        raise ValueError('Fixed initial regions requires an explicit source run')
    if mode=='fixed_initial_regions' and (set(methods)!={'EI_ZO','LGCS'} or objectives!=['minimum_distance'] or problem or selection!='all'):
        raise ValueError('Fixed initial regions requires both EI_ZO/LGCS path backends for the complete eligible cohort')
    _unique(objectives, "objectives", {"minimum_distance", "minimum_time"})
    _unique(methods, "methods", set(METHODS))
    roadmaps = [Path(p).resolve() for p in (roadmaps or [])]
    if roadmaps:
        _unique(roadmaps, "roadmaps")
        if seed_indices is not None:
            raise ValueError('Use roadmap seed pairing or --seed-indices, not both')
    else:
        if not set(methods) <= INDEPENDENT:
            raise ValueError('DRM-initialized methods require --roadmaps')
        seed_indices = [0] if seed_indices is None else seed_indices
        _unique(seed_indices, 'seed indices')
        if any(isinstance(s, bool) or not isinstance(s, int) or s < 0 for s in seed_indices):
            raise ValueError('Seed indices must be nonnegative integers declared by the dataset')
    if selection not in {"all", "first-per-family"}:
        raise ValueError("Unsupported selection")
    from .roadmap import load_config
    evaluation = load_config(config)
    output = Path(output).resolve()
    if output.exists():
        raise FileExistsError("Output already exists; use resume or a new run directory")
    manifest = Path(manifest).resolve()
    data = cases(manifest)
    input_manifest_sha256 = sha(manifest)
    if not data or len({c["split"] for c in data}) != 1:
        raise ValueError("Manifest must be nonempty and contain one split")
    if (study_protocol is None) != (study_block is None):
        raise ValueError("--study-protocol and --study-block must be supplied together")
    approval = None
    block = None
    if study_protocol is not None:
        from .study_protocol import authorize, method_pairs
        data, approval, block = authorize(study_protocol, study_block, manifest, roadmaps,
            evaluation, timeout, method_pairs(methods, objectives), selection=selection, problem=problem)
    elif any(c["split"] == "test" for c in data) and not allow_test:
        raise ValueError("Formal test execution is locked in v1 acceptance. Test generation only.")
    if problem:
        data = [c for c in data if c["problem_id"] == problem]
        if not data:
            raise ValueError("Unknown problem identifier")
    if selection == "first-per-family":
        selected = {}
        for case in data:
            selected.setdefault(case["family"], case)
        data = list(selected.values())
    initialization_bindings={};initialization_coverage=None
    if mode=='fixed_initial_regions':
        from .runners.shared_initialization import prepare_index
        seed_roadmaps=[(read(r/'metadata.json')['seed'],read(r/'metadata.json')['roadmap_id']) for r in roadmaps]
        initialization_bindings,initialization_coverage=prepare_index(initial_source_run,data,seed_roadmaps,evaluation)
        if not initialization_bindings:
            raise ValueError('No complete source initializations; no backend cohort can be prepared')
    jobs = []
    roadmap_files = dict(block["roadmap_files"]) if block is not None else {}
    roadmap_ids = set()
    for entry in (roadmaps or seed_indices):
        roadmap = Path(entry).resolve() if roadmaps else None
        metadata_path = roadmap / "metadata.json" if roadmap is not None else None
        if metadata_path is not None and not metadata_path.is_file():
            raise FileNotFoundError(metadata_path)
        metadata = read(metadata_path) if metadata_path is not None else dict(
            status='ready', roadmap_id=f'independent_s{entry:03d}', seed=entry)
        if metadata.get("status") != "ready":
            raise ValueError(f"Roadmap is not ready: {roadmap}")
        roadmap_identity = metadata.get("roadmap_id", roadmap.name if roadmap else None)
        if roadmap_identity in roadmap_ids:
            raise ValueError(f"Duplicate roadmap identity: {roadmap_identity}")
        roadmap_ids.add(roadmap_identity)
        for file in roadmap.iterdir() if roadmap is not None else []:
            if file.is_file():
                roadmap_files[str(file)] = sha(file)
        seed = int(metadata.get("seed", metadata.get("roadmap_seed", 0)))
        for case in data:
            shared=initialization_bindings.get((case['problem_id'],seed))
            if mode=='fixed_initial_regions' and shared is None:
                continue
            pairings = [p for p in case.get("paired_seeds", []) if p["drm_seed"] == seed]
            if len(pairings) != 1:
                raise ValueError(f"Roadmap seed {seed} is not declared in {case['problem_id']}")
            pairing = pairings[0]
            for objective in objectives:
                for method in methods:
                    if objective not in supported_objectives(method):
                        continue
                    identity = json.dumps([case["problem_id"], objective, method, str(roadmap) if roadmap else roadmap_identity], separators=(",", ":"))
                    if shared is not None:
                        identity=json.dumps([identity,mode,shared['initialization_digest']],separators=(',',':'))
                    job_id = f"{case['problem_id']}-{method}-{objective}-{hashlib.sha256(identity.encode()).hexdigest()[:8]}"
                    jobs.append(dict(job_id=job_id, case_id=case["problem_id"], scene_id=case["scene_id"],
                        family=case["family"], method=method, objective=objective,
                        roadmap_id=roadmap_identity, evaluation_config=evaluation,
                        roadmap_dir=str(roadmap) if roadmap else None,
                        roadmap_used=method not in INDEPENDENT, replica_seed=seed,
                        method_seed=int(pairing["method_seed"]),
                        solver_seed=int(pairing["solver_seed"]), solver_seed_used=False,
                        solver_seed_note="Reserved solver_seed is unused. Native OMPL/STOMP RNG uses method_seed; local deterministic optimizers use the fixed DRM seed path.",
                        case_ref=case_reference(case, manifest, input_manifest_sha256)))
                    if shared is not None:
                        jobs[-1].update(execution_mode=mode,shared_initialization=shared,
                            initialization_digest=shared['initialization_digest'])
    _unique([j["job_id"] for j in jobs], "job IDs")
    sources = code_inventory()
    commit = head()
    if not snapshot_sources:
        # Refuse before creating a directory that cannot subsequently resume.
        mismatches = _git_source_errors(commit, sources)
        if mismatches:
            raise ValueError('Commit runtime changes before preparing: ' + ', '.join(mismatches))
    output.mkdir(parents=True)
    if approval is not None:
        from .study_protocol import copy_evidence
        copy_evidence(output, approval)
    if snapshot_sources:
        for name in sources:
            dest = output / "source_snapshot" / name
            dest.parent.mkdir(parents=True, exist_ok=True)
            dest.write_bytes((ROOT / name).read_bytes())
    write(output / "run_manifest.json", dict(version=2, input_storage="manifest_reference", source_code_commit=head(),
        created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(), manifest_path=str(manifest),
        manifest_sha256=input_manifest_sha256, source_hashes=sources, roadmap_files=roadmap_files,
        selection="study-block" if approval is not None else selection,
        timeout_seconds=float(timeout), split=data[0]["split"] if data else None,
        formal_test_run=(approval is not None or allow_test) and data[0]["split"] == "test",
        execution_policy="fixed_public_inputs" if allow_test else None,
        source_recording="snapshot" if snapshot_sources else "git",
        study_protocol=approval, jobs=jobs,
        denominator="All declared legal tasks, including solver/front-end/timeout failures. No replacement.",
        seed_policy="Frozen paired method_seed controls EI-ZO, OMPL and STOMP RNG; independent OMPL uses roadmap identity only as a repetition pairing key, never loads its map.",
        omitted_unsupported_combinations=[dict(method=m, objective=o) for m in methods for o in objectives if o not in supported_objectives(m)]))
    if initialization_coverage is not None:
        write(output/'initialization_coverage.json',initialization_coverage)
        manifest_record=read(output/'run_manifest.json')
        manifest_record.update(execution_mode=mode,initialization_coverage_sha256=sha(output/'initialization_coverage.json'),
            denominator='Every paired job declared eligible before execution. Source loading/revalidation failures remain in this conditional denominator.',
            shared_initialization_source=initialization_coverage['source_run'])
        write(output/'run_manifest.json',manifest_record)
    write(output / "identity.json", dict(run_manifest_sha256=sha(output / "run_manifest.json")))
    if approval is not None:
        # Recheck after snapshot/copy creation so a concurrent source, model,
        # protocol or asset edit cannot produce an apparently approved plan.
        check = verify(output, current_sources=True)
        if not check["integrity_passed"]:
            write(output / "preparation_verification.json", check)
            raise ValueError("Study evidence changed while preparing: " + json.dumps(check))
    return output


def _git_source_errors(commit, sources):
    # Fresh public history deliberately does not contain the old run commits.
    # Their exact source bytes live once in a portable, hash-bound capsule.
    # Never use the capsule to mask a mismatch in an available Git commit.
    import re
    if not isinstance(commit, str) or re.fullmatch(r'[0-9a-f]{40}', commit) is None:
        return list(sources)
    available = subprocess.run(['git', 'cat-file', '-e', f'{commit}^{{commit}}'], cwd=ROOT,
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0
    if not available:
        try:
            index = read(BASE / 'provenance/index.json')
            recorded = index['commits'][commit]
            sources_root = BASE / 'provenance/sources'
            capsule = sources_root / commit
            root = capsule.resolve()
            if (sources_root.is_symlink() or capsule.is_symlink()
                    or not root.is_relative_to(BASE.resolve())):
                return list(sources)
            errors = []
            for name, expected in sources.items():
                relative = Path(name)
                path = (root / relative).resolve()
                if (relative.is_absolute() or '..' in relative.parts or str(relative) != name
                        or not path.is_relative_to(root) or (root / relative).is_symlink()
                        or recorded.get(name) != expected
                        or not path.is_file() or sha(path) != expected):
                    errors.append(name)
            return errors
        except (OSError, ValueError, KeyError, TypeError):
            return list(sources)
    errors = []
    for name, expected in sources.items():
        found = subprocess.run(['git', 'show', f'{commit}:{name}'], cwd=ROOT,
                               stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        if found.returncode or hashlib.sha256(found.stdout).hexdigest() != expected:
            errors.append(name)
    return errors


def verify(run, current_sources=False, allow_unsealed=False):
    run = Path(run)
    errors = []
    def fail(message):
        return dict(integrity_passed=False, complete=False, completed_jobs=0,
                    planned_jobs=0, errors=[message], unsealed_attempts=[])
    try:
        manifest = read(run / "run_manifest.json")
        required = {"jobs", "manifest_path", "manifest_sha256", "source_hashes", "roadmap_files", "timeout_seconds"}
        if not isinstance(manifest, dict) or not required <= manifest.keys():
            return fail("Incomplete run manifest")
        _positive_timeout(manifest["timeout_seconds"])
        reference_format = manifest.get("version") == 2 or manifest.get("input_storage") == "manifest_reference"
        if reference_format and (manifest.get("version") != 2 or manifest.get("input_storage") != "manifest_reference"):
            return fail("Inconsistent reference manifest format")
        _unique([job["job_id"] for job in manifest["jobs"]], "job IDs")
        for job in manifest["jobs"]:
            if reference_format and ("case_ref" not in job or "case" in job):
                return fail("Reference manifest requires only case_ref for every job")
            if not isinstance(job["job_id"], str) or Path(job["job_id"]).name != job["job_id"] or job["job_id"] in {".", ".."}:
                return fail("Invalid job path")
    except (OSError, ValueError, KeyError, TypeError) as exc:
        return fail(f"Unreadable or malformed run manifest: {exc}")
    try:
        if read(run / "identity.json")["run_manifest_sha256"] != sha(run / "run_manifest.json"):
            errors.append("run manifest changed")
    except (OSError, ValueError, KeyError, TypeError):
        errors.append("missing or malformed run identity")
    manifest_path = recorded_path(manifest["manifest_path"])
    if not manifest_path.is_file() or sha(manifest_path) != manifest["manifest_sha256"]:
        errors.append("input manifest changed")
    case_cache = {}
    for job in manifest["jobs"]:
        if "case_ref" not in job:
            continue
        try:
            reference = job["case_ref"]
            if (case_manifest_path(reference) != manifest_path
                    or reference["manifest_sha256"] != manifest["manifest_sha256"]):
                raise ValueError("Job reference is not bound to the run input manifest")
            resolve_case(job, case_cache)
        except (OSError, ValueError, TypeError, KeyError) as exc:
            errors.append("invalid case reference: " + str(exc))
    try:
        from .study_protocol import verify_binding
        verify_binding(run, manifest)
    except (OSError, ValueError, KeyError, TypeError, AttributeError) as exc:
        errors.append(f"study protocol verification failed: {exc}")
    if manifest.get('source_recording') == 'git':
        errors.extend('Git source mismatch: ' + name for name in
                      _git_source_errors(manifest['source_code_commit'], manifest['source_hashes']))
    for name, expected in manifest["source_hashes"].items():
        if manifest.get('source_recording') != 'git':
            snapshot = run / "source_snapshot" / name
            if not snapshot.is_file() or sha(snapshot) != expected:
                errors.append("source snapshot changed: " + name)
        if current_sources and (not (ROOT / name).is_file() or sha(ROOT / name) != expected):
            errors.append("current source changed; use a new run: " + name)
    for name, expected in manifest["roadmap_files"].items():
        relocated = recorded_path(name)
        if not relocated.is_file() or sha(relocated) != expected:
            errors.append("roadmap changed: " + name)
    if any(j.get('execution_mode')=='fixed_initial_regions' for j in manifest['jobs']) and manifest.get('execution_mode')!='fixed_initial_regions':
        errors.append('Conditioned jobs require a fixed_initial_regions run declaration')
    if manifest.get('execution_mode')=='fixed_initial_regions':
        try:
            from .runners.shared_initialization import check_bindings
            coverage=read(run/'initialization_coverage.json')
            if sha(run/'initialization_coverage.json')!=manifest.get('initialization_coverage_sha256'):
                raise ValueError('Initialization eligibility coverage changed')
            declarations=coverage['coverage']
            if (coverage.get('mode')!='fixed_initial_regions'
                    or any(type(r.get('eligible')) is not bool for r in declarations)):
                raise ValueError('Malformed initialization eligibility declarations')
            declared={(r['problem_id'],r['replica_seed']):r for r in declarations}
            expected={k for k,r in declared.items() if r['eligible']}
            if (len(declared)!=len(declarations)
                    or coverage.get('registered_task_replicas')!=len(declarations)
                    or coverage.get('eligible_task_replicas')!=len(expected)
                    or coverage.get('planned_paired_jobs')!=2*len(expected)):
                raise ValueError('Initialization coverage counts contradict predeclared tasks')
            paired={};binding_checks=[]
            for job in manifest['jobs']:
                key=(job['case_id'],job['replica_seed'])
                binding=job['shared_initialization'];declaration=declared.get(key,{})
                if (job.get('execution_mode')!='fixed_initial_regions' or job.get('objective')!='minimum_distance'
                        or job.get('initialization_digest')!=binding.get('initialization_digest')
                        or job.get('initialization_digest')!=declaration.get('initialization_digest')
                        or job.get('replica_seed')!=binding.get('replica_seed')
                        or job.get('roadmap_id')!=declaration.get('roadmap_id')
                        or binding.get('source_run')!=coverage.get('source_run')
                        or binding.get('source_run_manifest_sha256')!=coverage.get('source_run_manifest_sha256')):
                    raise ValueError('Conditioned job identity contradicts its source/eligibility declaration')
                paired.setdefault(key,[]).append(job)
                binding_checks.append((binding,resolve_case(job,case_cache),job['evaluation_config'],job['method_seed'],job['roadmap_id']))
            if set(paired)!=expected or any(len(j)!=2 or {v['method'] for v in j}!={'EI_ZO','LGCS'} or len({v['initialization_digest'] for v in j})!=1 for j in paired.values()):
                raise ValueError('Paired jobs differ from predeclared initialization eligibility')
            check_bindings(binding_checks)
        except (OSError,ValueError,KeyError,TypeError,AttributeError) as exc:
            errors.append('Shared initialization binding failed: '+str(exc))
    complete = 0
    unsealed = []
    for job in manifest["jobs"]:
        path = run / "jobs" / job["job_id"] / "attempts"
        for attempt in sorted(path.glob("[0-9][0-9][0-9][0-9]")):
            if not (attempt / "receipt.json").exists():
                unsealed.append(str(attempt))
                if not allow_unsealed:
                    errors.append("unsealed attempt: " + str(attempt))
                continue
            try:
                receipt = read(attempt / "receipt.json")
                files = receipt["files"]
                if not isinstance(files, dict) or "result.json" not in files:
                    raise ValueError("Receipt must include result.json")
                if not receipt.get("retry_pending") and not {"request.json", "process.json"} <= files.keys():
                    raise ValueError("Completed receipt must include request and process evidence")
                for name, expected in files.items():
                    candidate = (attempt / name).resolve()
                    if not candidate.is_relative_to(attempt.resolve()) or name == "receipt.json":
                        raise ValueError("Invalid receipt path")
                    if not candidate.is_file() or sha(candidate) != expected:
                        errors.append("artifact changed: " + str(candidate))
                # Historical replay/ files were generated after sealing; they
                # are explicitly not experimental evidence in these receipts.
                actual = {str(p.relative_to(attempt)) for p in attempt.rglob("*")
                          if p.is_file() and p.name != "receipt.json" and "replay" not in p.relative_to(attempt).parts}
                if actual - files.keys():
                    errors.append("unsealed artifacts: " + str(attempt))
                result = read(attempt / "result.json")
                if "runtime_sources.json" in files:
                    runtime = read(attempt / "runtime_sources.json")
                    unchanged = runtime["before"] == runtime["after"]
                    if runtime["before"] != manifest["source_hashes"] or runtime.get("unchanged") is not unchanged:
                        errors.append("runtime source evidence conflicts with frozen run: " + str(attempt))
                    if not unchanged and result.get("status") != "infrastructure_error":
                        errors.append("source change was not classified as infrastructure error: " + str(attempt))
                if not receipt.get("retry_pending") and any(result.get(key) != value for key, value in _result_identity(job).items()):
                    errors.append("result identity differs from frozen job: " + str(attempt))
                if result.get("status") not in TERMINAL_STATUSES or receipt.get("status") != result.get("status"):
                    errors.append("receipt/result status mismatch: " + str(attempt))
                if not receipt.get("retry_pending"):
                    if read(attempt / "request.json") != job:
                        errors.append("request differs from frozen job: " + str(attempt))
                    process = read(attempt / "process.json")
                    if result.get("status") == "success_validated_numerical":
                        if result.get("success") is False or process.get("exit_code") != 0 or process.get("timed_out") is not False:
                            errors.append("success contradicts process evidence: " + str(attempt))
                        if not {"validation.json", "trajectory_samples.npz", "candidate.npz"} <= files.keys():
                            errors.append("success lacks sealed candidate/validation evidence: " + str(attempt))
                        else:
                            validation = read(attempt / "validation.json")
                            cost = validation.get("cost")
                            expected_unit = {"minimum_distance": "rad", "minimum_time": "s"}.get(job.get("objective"))
                            if (validation.get("passed") is not True or validation.get("status") != result["status"]
                                    or isinstance(cost, bool) or not isinstance(cost, (int, float))
                                    or not math.isfinite(cost) or cost < 0 or validation.get("cost_unit") != expected_unit):
                                errors.append("success contradicts numerical validation: " + str(attempt))
            except (OSError, ValueError, TypeError, KeyError) as exc:
                errors.append(f"invalid attempt receipt {attempt}: {exc}")
        attempts = sorted(path.glob("[0-9][0-9][0-9][0-9]"))
        if attempts and (attempts[-1] / "receipt.json").exists():
            try:
                if not read(attempts[-1] / "receipt.json").get("retry_pending"):
                    complete += 1
            except (OSError, ValueError, TypeError):
                pass
    return dict(integrity_passed=not errors, complete=not errors and complete == len(manifest["jobs"]),
                completed_jobs=complete, planned_jobs=len(manifest["jobs"]), errors=errors, unsealed_attempts=unsealed)


def _gpu_memory(pid):
    try:
        r = subprocess.run(["nvidia-smi", "--query-compute-apps=pid,used_memory", "--format=csv,noheader,nounits"],
                           capture_output=True, text=True, timeout=2.)
        values = [float(line.split(",")[1]) for line in r.stdout.splitlines()
                  if line.split(",")[0].strip() == str(pid)]
        return max(values) if values else None
    except (OSError, ValueError, subprocess.TimeoutExpired):
        return None



def _external_stop_outcome(result, *, timed_out, returncode, attempt=None, config=None, source_hashes=None):
    """Preserve reported infrastructure errors if the external deadline follows."""
    handoff = (handoff_failure_evidence(attempt, result,
        expected_tolerance=(config or {}).get('region_coverage_tolerance_rad'),
        source_hashes=source_hashes or {}) if attempt is not None else None)
    abnormal_exit = unexpected_process_exit(dict(exit_code=returncode, timed_out=timed_out))
    attention = exception_needs_attention(result, handoff_evidence=handoff) or abnormal_exit
    if result.get('status') in INFRASTRUCTURE_STATUSES:
        status = result['status']
    else:
        status = 'infrastructure_error' if attention or not timed_out else 'timeout'
    result.update(status=status, success=False,
                  error='External job deadline' if timed_out else f'Worker exit {returncode}')
    if timed_out:
        result['deadline_exceeded'] = True
    if abnormal_exit and returncode is not None and returncode < 0:
        result['error_category'] = 'native_process_signal'
        result['signal_number'] = -returncode


def execute(job, run, timeout):
    _positive_timeout(timeout)
    runtime_sources_before = code_inventory()
    if runtime_sources_before != read(Path(run) / "run_manifest.json")["source_hashes"]:
        raise ValueError("Current source differs from frozen run before execution")
    parent = Path(run) / "jobs" / job["job_id"] / "attempts"
    previous = [int(p.name) for p in parent.glob("[0-9][0-9][0-9][0-9]")]
    attempt = parent / f"{max(previous, default=0) + 1:04d}"
    attempt.mkdir(parents=True, exist_ok=False)
    write(attempt / "request.json", job)
    argv = [sys.executable, "-m", "benchmarks.generic_motion_v1", "_worker", "--attempt", str(attempt)]
    start = time.perf_counter()
    peak = None
    timed_out = False
    stop_monitor = threading.Event()
    with (attempt / "stdout.log").open("w") as stdout, (attempt / "stderr.log").open("w") as stderr:
        process = subprocess.Popen(argv, cwd=ROOT, stdout=stdout, stderr=stderr, start_new_session=True)
        def sample_memory():
            nonlocal peak
            while not stop_monitor.is_set() and process.poll() is None:
                memory = _gpu_memory(process.pid)
                if memory is not None:
                    peak = memory if peak is None else max(peak, memory)
                stop_monitor.wait(.2)
        monitor = threading.Thread(target=sample_memory, daemon=True)
        monitor.start()
        try:
            # Deadline enforcement must never wait behind nvidia-smi polling.
            process.wait(timeout=max(0., timeout - (time.perf_counter() - start)))
        except subprocess.TimeoutExpired:
            timed_out = True
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=2.)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        finally:
            stop_monitor.set()
    wall = time.perf_counter() - start
    runtime_sources_after = code_inventory()
    write(attempt / "runtime_sources.json", dict(before=runtime_sources_before,
        after=runtime_sources_after, unchanged=runtime_sources_before == runtime_sources_after))
    write(attempt / "process.json", dict(argv=argv, exit_code=process.returncode, timeout_seconds=timeout,
        timed_out=timed_out, cold_e2e_seconds=wall, sampled_gpu_peak_mib=peak,
        gpu_peak_scope="Per-process nvidia-smi samples at approximately 0.2 s; not an exact allocator high-water mark.",
        includes="fresh process, imports, CUDA context, model/map load, warm-up, online planning, validation and output"))
    result_path = attempt / "result.json"
    try:
        result = read(result_path) if result_path.exists() else {}
        if not isinstance(result, dict):
            raise ValueError("Worker result must be an object")
    except (ValueError, TypeError):
        (attempt / "malformed_worker_result.txt").write_bytes(result_path.read_bytes())
        result = {}
    if timed_out or process.returncode != 0 or not result:
        if result:
            write(attempt / "result_before_external_stop.json", result)
        _external_stop_outcome(result,timed_out=timed_out,returncode=process.returncode,
            attempt=attempt,config=job.get('evaluation_config'),source_hashes=runtime_sources_before)
    if result.get("status") not in TERMINAL_STATUSES or runtime_sources_before != runtime_sources_after:
        write(attempt / "result_before_provenance_rejection.json", result)
        result.update(status="infrastructure_error", success=False,
                      error="Invalid worker status or source changed during execution")
    identity = _result_identity(job)
    mismatches = {key: dict(expected=value, actual=result[key]) for key, value in identity.items()
                  if result.get(key) is not None and result[key] != value}
    if mismatches:
        write(attempt / "result_before_identity_rejection.json", result)
        result.update(status="infrastructure_error", success=False,
                      error="Worker identity differs from frozen request", identity_conflicts=mismatches)
    result.update(identity, cold_e2e_seconds=wall, sampled_gpu_peak_mib=peak)
    write(result_path, result)
    files = {str(p.relative_to(attempt)): sha(p) for p in sorted(attempt.rglob("*")) if p.is_file()}
    write(attempt / "receipt.json", dict(status=result["status"], files=files))
    print(json.dumps(dict(job_id=job["job_id"], status=result["status"], seconds=round(wall, 3))), flush=True)
    return result


def resume(run, max_jobs=None):
    if max_jobs is not None and (isinstance(max_jobs, bool) or not isinstance(max_jobs, int) or max_jobs < 0):
        raise ValueError("max_jobs must be a nonnegative integer")
    run = Path(run).resolve()
    # Keep this inode for the lifetime of the run: unlinking a locked file
    # would let another coordinator lock a different inode at the same path.
    # This coordinator-level file is deliberately outside attempt receipts.
    with (run / ".resume.lock").open("a+") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError(f"Another coordinator is already resuming this run: {run}") from exc
        try:
            # A per-run lock alone permits two separate runs to share the GPU.
            # Main evaluation timing uses one worker across the whole project.
            shared = BASE / 'results' / '.gpu.lock'
            shared.parent.mkdir(parents=True, exist_ok=True)
            with shared.open('a+') as gpu_lock:
                try:
                    fcntl.flock(gpu_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                except BlockingIOError as exc:
                    raise RuntimeError('Another benchmark run currently owns the GPU') from exc
                try:
                    return _resume_locked(run, max_jobs)
                finally:
                    fcntl.flock(gpu_lock, fcntl.LOCK_UN)
        finally:
            fcntl.flock(lock, fcntl.LOCK_UN)


def _resume_locked(run, max_jobs=None):
    check = verify(run, allow_unsealed=True)
    if not check["integrity_passed"]:
        raise ValueError(json.dumps(check))
    if check["complete"]:
        return check
    compatible = verify(run, current_sources=True, allow_unsealed=True)
    if not compatible["integrity_passed"]:
        raise ValueError(json.dumps(compatible))
    # Preserve a coordinator/host interruption as a sealed failed attempt,
    # then retry in a new numbered directory with the same frozen inputs.
    for name in check["unsealed_attempts"]:
        interrupted = Path(name)
        prior = read(interrupted / "result.json") if (interrupted / "result.json").exists() else None
        if prior is not None:
            write(interrupted / "result_before_interruption.json", prior)
        write(interrupted / "result.json", dict(status="infrastructure_error", success=False,
            error_category="coordinator_interrupted", error="Unsealed attempt retained before resume",
            previous_partial_status=prior.get("status") if prior else None))
        files = {str(p.relative_to(interrupted)): sha(p) for p in sorted(interrupted.rglob("*")) if p.is_file()}
        write(interrupted / "receipt.json", dict(status="infrastructure_error", retry_pending=True, files=files))
    manifest = read(run / "run_manifest.json")
    completed = 0
    for job in manifest["jobs"]:
        attempts = sorted((run / "jobs" / job["job_id"] / "attempts").glob("[0-9][0-9][0-9][0-9]"))
        if attempts and (attempts[-1] / "receipt.json").exists():
            if not read(attempts[-1] / "receipt.json").get("retry_pending"):
                continue
        if max_jobs is not None and completed >= max_jobs:
            break
        if code_inventory() != manifest["source_hashes"]:
            raise ValueError("Current source changed while resuming; use a new run")
        execute(job, run, manifest["timeout_seconds"])
        completed += 1
    result = verify(run)
    write(run / "verification.json", result)
    return result


def worker(attempt):
    if sys.platform.startswith("linux"):
        import ctypes
        parent = os.getppid()
        if ctypes.CDLL(None, use_errno=True).prctl(1, signal.SIGTERM, 0, 0, 0) != 0:
            raise OSError(ctypes.get_errno(), "Unable to set parent-death signal")
        if os.getppid() != parent:
            os.kill(os.getpid(), signal.SIGTERM)
    from .runners.csdecomp_baseline import run_case
    attempt = Path(attempt)
    job = read(attempt / "request.json")
    try:
        roadmap_dir = recorded_path(job['roadmap_dir']) if job.get('roadmap_dir') else None
        result = run_case(resolve_case(job), roadmap_dir, attempt, objective=job["objective"],
                          seed=job["method_seed"], method=job["method"], config=job.get("evaluation_config"),
                          shared_initialization=job.get('shared_initialization'))
        write(attempt / "result.json", result)
    except Exception as exc:
        import traceback
        traceback.print_exc()
        write(attempt / "result.json", dict(status="infrastructure_error", success=False,
                                            error=repr(exc), traceback=traceback.format_exc()))
        raise
