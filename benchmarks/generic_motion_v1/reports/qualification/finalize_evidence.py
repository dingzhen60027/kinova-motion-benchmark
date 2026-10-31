"""Compile final read-only checks; never alter prior experiments or input assets."""
import json
import re
from pathlib import Path
from benchmarks.generic_motion_v1.common import BASE, head, code_inventory, read, write, sha
from benchmarks.generic_motion_v1.batch import verify
from .freeze_integrity import check


def main():
    out=BASE/'reports/qualification'
    historical={p.parent.name:verify(p.parent) for p in sorted((BASE/'results').glob('*/run_manifest.json'))}
    write(out/'historical_integrity_final.json',historical)
    freeze=check();write(out/'freeze_final.json',freeze)
    input_audit=read(out/'generator_reaudit_receipt.json')
    candidate=read(out/'baseline_saved_candidate_reaudit.json')
    test_log=(out/'final_pytest.log').read_text()
    test_match=re.search(r'(\d+) passed[^\n]*in [0-9.]+s',test_log)
    tests_passed=int(test_match.group(1)) if test_match else 0
    checks=dict(input_audit=input_audit['passed'],freeze=freeze['passed'],
        model=read(out/'model_audit_current.json')['passed'],cross_split=read(out/'cross_split_audit.json')['passed'],
        candidate_contracts=candidate['statuses']=={'passed_contract_reaudit':candidate['candidates']},
        unit_tests=tests_passed>0 and not re.search(r'\d+ failed',test_log),
        current_integration=historical['qualification_integration_v2']['integrity_passed'] and historical['qualification_integration_v2']['complete'])
    runtime=[]
    run=BASE/'results/qualification_integration_v2'
    for p in sorted(run.glob('jobs/*/attempts/0001/result.json')):
        result=read(p);process=read(p.parent/'process.json')
        runtime.append(dict(attempt=str(p.parent.relative_to(BASE)),method=result['method'],objective=result['objective'],
            status=result['status'],cold_e2e_seconds=process['cold_e2e_seconds'],metrics=result['metrics'],
            result_sha256=sha(p),process_sha256=sha(p.parent/'process.json')))
    status=dict(verification_source_commit=head(),source_hashes=code_inventory(),
        engineering_checks_passed=all(checks.values()),checks=checks,
        qualification_scope='Canonical Kinova static geometric planning inputs and bounded numerical checks; controlled development use only.',
        formal_test_execution_enabled=False,publication_comparison_ready=False,
        reason='Formal test execution was not authorized. Public benchmark external validity, native assertion root causes and equal collision-query budgets remain unresolved; family labels are workspace geometry only.',
        runtime_integration_source_commit=read(run/'run_manifest.json')['source_code_commit'],
        runtime_integration=runtime,tests={'passed':tests_passed,'log':'reports/qualification/final_pytest.log','log_sha256':sha(out/'final_pytest.log')},
        input_only_reaudit=input_audit,freeze_integrity_passed=freeze['passed'],
        historical_integrity={k:{x:v[x] for x in ['integrity_passed','complete','completed_jobs','planned_jobs','errors']} for k,v in historical.items()},
        immutable_evidence_sha256={str(p.relative_to(BASE)):sha(p) for p in [
            out/'model_audit_current.json',out/'model_audit_current_raw.npz',out/'baseline_saved_candidate_reaudit.json',
            out/'generator_reaudit_receipt.json',out/'cross_split_audit.json',out/'historical_deadlines.json']})
    write(out/'qualification_status.json',status)
    print(json.dumps({k:status[k] for k in ['verification_source_commit','engineering_checks_passed','formal_test_execution_enabled','publication_comparison_ready']},indent=2))
    return 0 if status['engineering_checks_passed'] else 1

if __name__=='__main__':raise SystemExit(main())
