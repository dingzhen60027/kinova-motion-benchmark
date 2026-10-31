"""Derive test-generation gate from sealed actual evidence, without planning."""
import json
from pathlib import Path
from benchmarks.generic_motion_v1.common import BASE,read,write,sha,head
from benchmarks.generic_motion_v1.batch import verify

def main():
    smoke=BASE/'results/smoke_v3_fixed'
    dev=BASE/'results/dev_eizo_v2'
    paired=BASE/'results/dev_paired_v2'
    checks={name:verify(path) for name,path in [('smoke',smoke),('dev',dev),('paired',paired)]}
    outcomes=[read(p) for p in smoke.glob('jobs/*/attempts/*/result.json')]
    objectives={r['objective'] for r in outcomes if r['status']=='success_validated_numerical'}
    replays=list((BASE/'results/smoke_preflight_v1').rglob('replay.json'))
    paths=['reports/model_audit/audit.json','reports/smoke_input_audit.json',
           'reports/dev_input_audit.json','reports/dev_actual_determinism.json',
           'reports/resume_and_pairing_acceptance.json','manifests/smoke.jsonl','manifests/dev.jsonl']
    record=dict(G0=read(BASE/paths[0])['passed'] is True,
        G1=read(BASE/paths[1])['passed'] is True and read(BASE/paths[1])['problem_count']==10
           and checks['smoke']['complete'] and checks['smoke']['integrity_passed']
           and objectives=={'minimum_distance','minimum_time'} and len(replays)>=2,
        G2=read(BASE/paths[2])['passed'] is True and read(BASE/paths[2])['problem_count']==60
           and read(BASE/paths[3])['passed'] is True and checks['dev']['complete'] and checks['dev']['integrity_passed'],
        G4_development_execution=checks['paired']['complete'] and checks['paired']['integrity_passed'],
        source_commit=head(), checks=checks, evidence_sha256={p:sha(BASE/p) for p in paths},
        replay_evidence_sha256={str(p.relative_to(BASE)):sha(p) for p in replays},
        authorization='Only generate and audit 420 test inputs; no test planner execution.',
        limitations=['NTO paired development subset has four short problems; no overall ranking.',
                     'Native collision-query counters and internal pre-shortcut path are unavailable.',
                     'Equal added-collision-query budget comparison remains unsupported.'])
    assert all(record[k] for k in ['G0','G1','G2']),record
    write(BASE/'reports/acceptance.json',record)
    print(json.dumps({k:record[k] for k in ['G0','G1','G2','G4_development_execution']}))

if __name__=='__main__':main()
