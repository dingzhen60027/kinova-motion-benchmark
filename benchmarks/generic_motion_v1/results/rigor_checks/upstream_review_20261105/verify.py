"""Recheck the upstream-backed workaround without rerunning any planner."""
import argparse
from collections import Counter
import hashlib
from importlib.metadata import distribution
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[5]
BASE = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
import numpy as np
from benchmarks.generic_motion_v1.batch import verify
from benchmarks.generic_motion_v1.common import code_inventory, read
from benchmarks.generic_motion_v1.runners.ipopt_options import DEFAULTS, evidence
from benchmarks.generic_motion_v1.runners.results import collect


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def check(output):
    if output.exists():
        raise FileExistsError(output)
    checks, runs = {}, []
    csd = read(BASE/'csdecomp_sources.json')
    checks['official_csdecomp_sources_match'] = csd['comparison_summary']['official_tree_bytes_all_identical']
    checks['official_csdecomp_wheel_matches'] = csd['comparison_summary']['installed_package_and_bundled_libraries_identical_to_hash_verified_official_wheel']
    package = distribution('csdecomp')
    checks['installed_csdecomp_unchanged'] = all(sha(package.locate_file(item['path'])) == item['sha256']
                                               for item in csd['installed_csdecomp']['files'])
    reg = read(BASE/'regression_final.json')
    checks['full_regression'] = (reg['returncode'] == 0 and sha(BASE/'regression_final.log') == reg['log_sha256']
                                and reg['runtime_sources_before'] == reg['runtime_sources_after'] == code_inventory())
    route = read(BASE/'native_routes_final/summary.json')
    checks['identical_mathematical_transcription'] = route['same_transcription']
    modes = {mode['mode']: mode for mode in route['modes']}
    checks['faulting_function_route_avoided'] = (route['expected_route_verified']
        and modes['default_matching']['route']['entry_count'] > 0
        and modes['metis_mc64']['route']['entry_count'] == 0
        and all(mode['exit_code'] == 0 and not mode['native_sigsegv_observed'] for mode in modes.values()))
    native = read(BASE/'native_localization.json')
    checks['native_route_artifacts_unchanged'] = all(sha(ROOT/name) == expected
                                                   for name, expected in native['artifact_sha256'].items())
    binary = evidence()['native_binary']
    checks['loaded_native_library_unchanged'] = binary['sha256'] == native['current_library_sha256']
    frozen = read(BASE.parent/'strict_check_20261105/dataset/run/summary.json')['protected_files_before']
    checks['frozen_input_files_unchanged'] = all(sha(ROOT/name) == expected for name, expected in frozen.items())
    checks['historical_records_unchanged'] = sha(ROOT/'results/records.csv') == '2cb418255861d5daa850b2756425b08f5beaf76aa9fe99d21686c2ebbf31662c'
    checks['historical_summary_unchanged'] = sha(ROOT/'results/summary.csv') == '20d2b4cb081b5fbf22020f2e574c3c1eb8d15455726fb1f3d47c47a81efb60b1'
    policies = []
    for name, count in (('online_smoke', 8), ('original_case', 1)):
        run = BASE/name
        integrity = verify(run, current_sources=True)
        rows, inventory = collect([run])
        checks[name+'_complete'] = integrity['integrity_passed'] and integrity['complete'] and integrity['planned_jobs'] == len(rows) == count
        checks[name+'_no_native_crash'] = all(row['status'] != 'infrastructure_error' for row in rows)
        for path in (run/'jobs').rglob('nto_parameters.json'):
            record = read(path)
            options = record['parameters']
            if 'max_cpu_time' not in options:
                continue  # SNOPT uses a different wall-time option.
            policy = record.get('linear_system', {})
            ok = (all(options.get(key) == value for key, value in DEFAULTS.items())
                  and policy.get('options') == DEFAULTS
                  and policy.get('native_binary', {}).get('sha256') == binary['sha256'])
            policies.append(dict(path=str(path.relative_to(ROOT)), passed=ok))
        runs.append(dict(run=name, integrity=integrity, inventory=inventory,
            statuses=dict(Counter(row['status'] for row in rows)),
            outcomes=[{key: row.get(key) for key in ('method', 'objective', 'problem_id', 'status',
                'objective_value', 'objective_unit', 'cold_e2e_seconds', 'attempt_path')} for row in rows]))
    checks['real_ipopt_calls_use_explicit_policy'] = len(policies) >= 5 and all(item['passed'] for item in policies)
    old = ROOT/'results/rigor_checks/dev_nto_incumbent/dev_seed0/jobs/dev-dense_clutter-001-p01-NTO_REFINED-minimum_time-4e4af77e/attempts/0001'
    new = next((BASE/'original_case/jobs').glob('*/attempts/0001'))
    checks['original_crash_record_preserved'] = sha(old/'receipt.json') == 'b262dc28131d7cb7f68c859a250613daa991df00b1edce2fe420872c22f91e14'
    checks['original_case_same_input_and_frontend'] = (sha(old/'input.json') == sha(new/'input.json')
        and np.array_equal(np.load(old/'path.npy'), np.load(new/'path.npy')))
    with np.load(old/'refinement/round_00/nto_initial.npz') as a, np.load(new/'refinement/round_00/nto_initial.npz') as b:
        checks['original_case_same_initial_spline'] = a.files == b.files and all(np.array_equal(a[key], b[key]) for key in a.files)
    old_cfg, new_cfg = read(old/'evaluation.json'), read(new/'evaluation.json')
    checks['only_linear_system_policy_changes_original_config'] = (
        {key: value for key, value in new_cfg.items() if key != 'ipopt'} == old_cfg
        and new_cfg['ipopt'] == DEFAULTS)
    artifact_hashes = {str(path.relative_to(BASE)): sha(path) for path in sorted(BASE.rglob('*'))
        if path.is_file() and path != output and path.name not in ('.resume.lock', 'verification.json')
        and '__pycache__' not in path.parts and path.suffix != '.tmp'}
    result = dict(checked_scopes_passed=all(checks.values()), checks=checks,
        source_commit=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        scope='Upstream source audit and native backend stability regression; no formal-test reruns or score replacement',
        native_binary=binary, policy=DEFAULTS, policy_records=policies, runs=runs,
        protected_input_files=len(frozen), artifact_sha256=artifact_hashes,
        limitations=['This avoids the implicated matching-order path; it does not install an IPOPT/SPRAL native patch.',
                     'The old crash lacks a sealed original library hash, matrix and full stack; #848 is not an established cause.',
                     'Numerical collision validation is not a mathematical continuous certificate.'])
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps(dict(checked_scopes_passed=result['checked_scopes_passed'], checks=checks,
                         statuses={run['run']: run['statuses'] for run in runs})))
    return 0 if result['checked_scopes_passed'] else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    raise SystemExit(check(parser.parse_args().output.resolve()))
