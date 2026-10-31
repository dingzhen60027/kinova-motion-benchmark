# Formal study: completion-only postprocessing

Static integration review, 2026-10-28. This note does **not** report completed
formal experiments or audit results. The review read source and interfaces only;
no partial performance, receipt scan, numerical auditor, solver, GPU work or
formal metrics export was invoked. Runtime HEAD was still
`c8adbd8cef44db25b41559e8fbf2b38e120b624a`.

## Execution order and bindings

1. Wait for the formal coordinator to finish all 24 blocks, with
   `formal_completed_blocks.json` declaring `complete: true` and 9900 slots.
   This is necessary but not sufficient: the audit driver independently checks
   every sealed receipt after the completion barrier.
2. `audit_formal_study audit`: verify the frozen protocol/source/input/matrix and
   all 24 runs, then invoke the SCS, NTO and LGCS independent auditors per block
   (72 auditor invocations). Any failure retains evidence and prevents a passing
   combined audit. The SCS copy requires the explicit full expected source commit.
3. Export main metrics from exactly the 15 `phase == main` index entries: 6300
   registered rows, retaining invalid-input accounting.
4. Export scaling metrics from **all 24** index entries, using
   `formal_size_effect_ids.json` as `--problem-ids-json`: 4800 rows. Filtering is
   performed after all supplied runs are verified. The 1200 observations at 6000
   nodes are reused from main, never planned or counted again.
5. `seal-bundle`: bind both metrics files, the passing independent audits,
   protocol SHA, every run-manifest SHA, and each reported job's identity,
   legality and terminal status. Accounting is 9900 distinct slots, 9300 legal
   planning calls and 600 invalid-input slots.
6. `render_formal_report`: check the bundle, exact matrices, method pairings,
   denominators, scalar statistics and 6000-node row equality, then write the
   report and static figures.

The reviewed interfaces match: SCS/NTO expose `frozen_planning_commit`, LGCS
exposes `source_commit`; the driver selects the appropriate field. All expose
`run_manifest_sha256` and job records. The metrics exporter supplies the
`job_id`, case/method/seed/input-legality fields required by the sealer. Bundle
fields match the renderer. Paths are absolute in the frozen index/protocol.
Requested roadmap size is the grouping variable; retained nodes are separately
reported, including the 2999-node realization of a 3000-node request.

The exact command entry points below reuse the existing Conda interpreter via
`local/python.sh`. Run the block **only after all timed formal planning has
ended**. It reads the frozen index for run selection rather than using filename
globs. It stops at the first failed command, saves actual argv/stdout/stderr,
and refuses to replace existing postprocessing outputs.

```bash
cd /home/eric/wjg/wd/csdecomp
bash local/python.sh - <<'PY'
from datetime import datetime, timezone
from pathlib import Path
import hashlib
import json
import subprocess
import sys
from benchmarks.generic_motion_v1.common import code_inventory

root = Path.cwd()
report_dir = root / 'benchmarks/generic_motion_v1/reports/baseline_study'
protocol_path = report_dir / 'formal_protocol_v1.json'
index_path = report_dir / 'formal_run_index.json'
completed_path = report_dir / 'formal_completed_blocks.json'
source = 'c8adbd8cef44db25b41559e8fbf2b38e120b624a'
protocol_sha = '8d152ac237cf39fe9602bcf54d47fbe8e30471dd5e3096ff05375ff4b3fcd7d7'
protocol = json.loads(protocol_path.read_text())
index = json.loads(index_path.read_text())
completed = json.loads(completed_path.read_text())
if not (completed.get('complete') is True and
        completed.get('completed_jobs') == completed.get('total_jobs') == 9900):
    raise RuntimeError('All timed formal planning must finish before postprocessing')
if hashlib.sha256(protocol_path.read_bytes()).hexdigest() != protocol_sha:
    raise RuntimeError('Frozen protocol hash changed')
if protocol['source_code_commit'] != source or index['protocol_sha256'] != protocol_sha:
    raise RuntimeError('Unexpected source or protocol binding')
if code_inventory() != protocol['runtime_source_hashes']:
    raise RuntimeError('Current runtime/metrics source differs from frozen study')
main_runs = [item['run'] for item in index['blocks'] if item['phase'] == 'main']
all_runs = [item['run'] for item in index['blocks']]
if len(main_runs) != 15 or len(all_runs) != len(set(all_runs)) or len(all_runs) != 24:
    raise RuntimeError('Expected exactly 15 main and 24 total runs')

audit_dir = report_dir / 'formal_audit'
main_dir = report_dir / 'formal_main_metrics'
size_dir = report_dir / 'formal_size_metrics'
bundle = audit_dir / 'audit_bundle.json'
command_log = report_dir / 'formal_postprocess_commands.jsonl'
final_report = report_dir.parent / 'benchmark_v1_formal_baselines.md'
for path in (audit_dir, main_dir, size_dir, command_log, final_report):
    if path.exists():
        raise FileExistsError(f'Preserve existing evidence; choose fresh output paths: {path}')

python = sys.executable
module = 'benchmarks.generic_motion_v1.reports.baseline_study.audit_formal_study'
metrics_module = 'benchmarks.generic_motion_v1.runners.study_metrics'
renderer_module = 'benchmarks.generic_motion_v1.reports.baseline_study.render_formal_report'
commands = [
    ('audit', [python, '-m', module, 'audit',
        '--index', str(index_path), '--completed', str(completed_path),
        '--protocol', str(protocol_path), '--output', str(audit_dir),
        '--expected-source', source, '--expected-protocol-sha', protocol_sha,
        '--planning-finished']),
    ('main_metrics', [python, '-m', metrics_module,
        '--runs', *main_runs, '--output', str(main_dir)]),
    ('size_metrics', [python, '-m', metrics_module,
        '--runs', *all_runs, '--output', str(size_dir),
        '--problem-ids-json', str(report_dir / 'formal_size_effect_ids.json')]),
    ('seal_bundle', [python, '-m', module, 'seal-bundle',
        '--integrity', str(audit_dir / 'integrity_audit.json'),
        '--main-metrics', str(main_dir / 'metrics.json'),
        '--scaling-metrics', str(size_dir / 'metrics.json'), '--output', str(bundle)]),
    ('render', [python, '-m', renderer_module,
        '--main-metrics', str(main_dir / 'metrics.json'),
        '--scaling-metrics', str(size_dir / 'metrics.json'),
        '--audit-bundle', str(bundle), '--protocol', str(protocol_path),
        '--run-index', str(index_path), '--output', str(final_report)]),
]
for stage, argv in commands:
    if code_inventory() != protocol['runtime_source_hashes']:
        raise RuntimeError('Frozen runtime/metrics source changed during postprocessing')
    entry = dict(stage=stage, argv=argv, cwd=str(root),
                 started_utc=datetime.now(timezone.utc).isoformat())
    with command_log.open('a') as stream:
        stream.write(json.dumps(dict(entry, event='start')) + '\n')
    with (report_dir / f'formal_postprocess_{stage}.log').open('x') as log:
        result = subprocess.run(argv, cwd=root, stdout=log,
                                stderr=subprocess.STDOUT, check=False)
    with command_log.open('a') as stream:
        stream.write(json.dumps(dict(entry, event='finish', returncode=result.returncode,
            finished_utc=datetime.now(timezone.utc).isoformat())) + '\n')
    if result.returncode:
        raise RuntimeError(f'{stage} failed; preserve logs and incomplete audit evidence')
print(final_report)
PY
```

This block is documentation, not a command that has already been run. Existing
failed attempts or audits must not be deleted to satisfy the fresh-path guard.
Use a distinct output name for a deliberate retry and retain the failed record.

## Validation boundaries

- Input legality comes from the frozen full input re-audit, with all 25 invalid
  registrations retained. A legal input is not proof of global feasibility.
- Independent auditors verify sealed provenance, saved trajectory algebra,
  numerical limit/membership checks and success/failure accounting. Saved
  collision evidence is not a new geometry query or a continuous collision
  certificate.
- Confidence intervals come from the frozen `study_metrics`/scene-cluster
  bootstrap implementation and its existing statistical regression tests.
  **The three independent trajectory auditors do not independently recompute
  bootstrap intervals.** The renderer checks denominators, observation counts,
  recorded scalar summaries and matrix identity; it does not rerun bootstrap.
- A failed evidence gate must be investigated as saved evidence. It must not be
  bypassed by editing the frozen input, dropping failed planner calls, changing
  matrix membership or making a metrics `complete` flag true by hand.
- Shared-frontend mismatch or missing cold latency is a legitimate publication
  gate. Whether either occurs can be established only after completion; this
  static review deliberately inspected no partial outcomes.
