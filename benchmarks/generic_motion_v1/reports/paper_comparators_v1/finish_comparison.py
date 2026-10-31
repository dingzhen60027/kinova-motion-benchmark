"""Aggregate, audit and replay complete measurements; never execute a planner."""
import sys
from pathlib import Path
from .run_comparison_study import OUT, command, read, check_frozen


def main():
    done = read(OUT/'completed_blocks.json')
    if not done['complete'] or done['completed_jobs'] != 380:
        raise ValueError('Finish all timed planning before analysis and rendering')
    frozen = read(OUT/'execution_freeze.json')
    check_frozen(frozen)
    output = OUT/'comparison_metrics'
    if output.exists():
        raise FileExistsError('Preserve previous analysis; use its explicit audit/replay commands')
    runs = list(frozen['reused_evidence']) + [b['run'] for b in frozen['blocks']]
    labels = ['LSCS','LGCS','NTO-distance','SCSTO','NTO-time',
              'NTO-refined-distance','NTO-refined-time']
    command([sys.executable,'-m','benchmarks.generic_motion_v1.runners.study_metrics',
             '--runs',*runs,'--output',str(output),'--expected-labels',*labels],
            'comparison','metrics')
    command([sys.executable,'-m','benchmarks.generic_motion_v1.reports.paper_comparators_v1.audit_comparison',
             '--metrics',str(output/'metrics.json'),'--output',str(OUT/'comparison_audit.json')],
            'comparison','audit')
    command([sys.executable,'-m','pytest','-q',str(OUT/'test_comparison_study.py'),
             str(OUT/'test_audit_comparison.py'),str(OUT/'test_report.py'),str(OUT/'test_make_replays.py')],
            'comparison','tests')
    command([sys.executable,'-m','benchmarks.generic_motion_v1.reports.paper_comparators_v1.make_replays',
             '--planning-finished'], 'comparison','replays')
    command([sys.executable,'-m','benchmarks.generic_motion_v1.reports.paper_comparators_v1.build_report',
             '--metrics',str(output/'metrics.json'),'--audit',str(OUT/'comparison_audit.json'),
             '--source-audit',str(OUT/'paper_coverage_audit.md'),
             '--replay-index',str(OUT/'replays/index.html'),
             '--output',str(OUT.parent/'benchmark_v1_paper_comparators.md')],
            'comparison','report')


if __name__ == '__main__':
    main()
