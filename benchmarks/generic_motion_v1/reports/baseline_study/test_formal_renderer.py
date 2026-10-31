"""Tiny synthetic interface tests, never scientific results or planner inputs.

No formal result directory, runtime checker, GPU, solver or bootstrap is read or
called. These tests exercise refusal behavior using five in-memory fake rows.
"""
import copy
import json
from pathlib import Path
import tempfile
import unittest

from .render_formal_report import METHODS, load_evidence, protocol_rows, require_matrix, row_key, sha


def fixture():
    rows = []
    for label in METHODS:
        rows.append(dict(case_id='fixture-only', roadmap_nodes=6000, roadmap_seed=0,
            label=label, run='/fixture/run', split='test', budget_seconds=60,
            attempt_sealed=True, retry_pending=False, completed_stage_seconds={},
            backend_timing_complete=False, success=False, status='frontend_failed',
            input_legal=True, cold_e2e_seconds=1., input_sha256='fixture-input',
            robot_sha256='fixture-robot', geometry_sha256='fixture-geometry',
            q_start=[0.]*7, q_goal=[1.]*7, family='fixture-family', distance_bin='short',
            straight_line_collision=True, cluster_id='fixture-world', method_seed=0))
    data = dict(complete=True, comparable=True, schema_version=1, pairing_differences=[],
        expected_method_labels=list(METHODS), requested_jobs=5, rows=rows,
        runs=[dict(run='/fixture/run', source_commit='fixture-commit', manifest_sha256='fixture-manifest',
                   integrity=dict(complete=True, integrity_passed=True))])
    expected = {row_key(row): '/fixture/run' for row in rows}
    return data, expected, {'/fixture/run': 'fixture-manifest'}


class FormalRendererRefusalTests(unittest.TestCase):
    def test_small_complete_matrix(self):
        data, expected, runs = fixture()
        self.assertEqual(len(require_matrix(data, expected, runs, 'fixture-commit')), 5)

    def reject(self, mutation, message):
        data, expected, runs = fixture()
        mutation(data)
        with self.assertRaisesRegex(ValueError, message):
            require_matrix(data, expected, runs, 'fixture-commit')

    def test_incomplete(self):
        self.reject(lambda d: d.update(complete=False), 'Incomplete')

    def test_frontend_incomparable(self):
        self.reject(lambda d: d.update(comparable=False), 'incomparable')

    def test_missing_method(self):
        self.reject(lambda d: d['rows'].pop(), 'matrix count')

    def test_duplicate_reused_6000(self):
        self.reject(lambda d: d['rows'].__setitem__(4, copy.deepcopy(d['rows'][0])), 'Duplicate observation')

    def test_unsealed(self):
        self.reject(lambda d: d['rows'][0].update(attempt_sealed=False), 'Unsealed')

    def test_missing_failure_latency(self):
        self.reject(lambda d: d['rows'][0].update(cold_e2e_seconds=None), 'missing output latency')

    def test_success_without_solver(self):
        self.reject(lambda d: d['rows'][0].update(success=True, status='success_validated_numerical',
            native_drm_success=True, drm_success=True, solver_reported_success=False, final_validation_passed=True),
            'lacks input, frontend, solver')

    def test_case_mutated_for_method(self):
        self.reject(lambda d: d['rows'][1].update(q_goal=[2.]*7), 'Fixed input differs')

    def test_old_stage_schema(self):
        self.reject(lambda d: d['rows'][0].pop('completed_stage_seconds'), 'timing eligibility')

    def test_wrong_run(self):
        self.reject(lambda d: d['rows'][0].update(run='/fixture/replacement'), 'wrong registered block')

    def test_no_integrity(self):
        self.reject(lambda d: d['runs'][0]['integrity'].update(integrity_passed=False), 'unverified or incomplete')

    def test_changed_runtime(self):
        self.reject(lambda d: d['runs'][0].update(source_commit='different'), 'commit changed')

    def test_protocol_not_success_selected(self):
        block = dict(phase='main', requested_node_budgets=[6000], roadmap_seeds=[0],
            problem_count=2, problem_ids=['fixture-valid', 'fixture-invalid'],
            method_pairs=[['EI_ZO', 'minimum_distance'], ['LGCS', 'minimum_distance'],
                          ['NTO_PAPER', 'minimum_distance'], ['EI_ZO', 'minimum_time'],
                          ['NTO_PAPER', 'minimum_time']])
        protocol = {'blocks': {'fixture-block': block}}
        index = {'blocks': [{'block': 'fixture-block', 'run': '/fixture/run'}]}
        expected = protocol_rows(protocol, index, {'main'})
        self.assertEqual(len(expected), 10)
        self.assertEqual(len(protocol_rows(protocol, index, {'main'}, {'fixture-invalid'})), 5)

    def audit_fixture_rejected(self, mutation, message):
        """All files are tiny synthetic temporary audit-interface fixtures."""
        with tempfile.TemporaryDirectory(prefix='formal-renderer-fixture-') as tmp:
            root = Path(tmp)
            def put(name, content):
                path = root/name; path.write_text(json.dumps(content)); return path
            input_data = dict(passed=True, errors=[], manifest_sha256='fixture-manifest',
                problem_count=1, valid_input_count=1, invalid_input_count=0)
            audit_data = dict(complete=True, passed=True, errors=[], integrity_passed=True,
                independent_passed=True, all_outcomes_reviewed=True, covered_job_slots=5,
                reviewed_methods=list(METHODS), run_manifest_sha256={'/fixture/run': 'fixture-run-sha'})
            bundle_data = dict(schema_version=1, kind='formal_baseline_audit_bundle', complete=True,
                passed=True, errors=[], source_code_commit='fixture-commit',
                run_manifest_sha256={'/fixture/run': 'fixture-run-sha'})
            mutation(input_data, audit_data, bundle_data)
            input_path = put('input.json', input_data)
            protocol = dict(state='approved', source_code_commit='fixture-commit',
                blocks={'fixture-block': {}}, input_audit={'sha256': sha(input_path)},
                manifest={'sha256': 'fixture-manifest'},
                dataset_accounting=dict(registered=1, legal=1, input_generation_failures=0),
                execution=dict(total_distinct_job_slots=5))
            protocol_path = put('protocol.json', protocol)
            index_path = put('index.json', dict(protocol_sha256=sha(protocol_path),
                blocks=[dict(block='fixture-block', run='/fixture/run', run_manifest_sha256='fixture-run-sha')]))
            audit_data['protocol_sha256'] = sha(protocol_path)
            audit_path = put('audit.json', audit_data)
            main_path, scaling_path = put('main.json', {}), put('scaling.json', {})
            bundle_data.update(protocol_sha256=sha(protocol_path), run_index_sha256=sha(index_path),
                metrics_sha256={'main': 'intentionally-not-the-fixture-hash', 'scaling': sha(scaling_path)},
                evidence={'input': {'path': str(input_path), 'sha256': sha(input_path)},
                          'integrity': {'path': str(audit_path), 'sha256': sha(audit_path)},
                          'independent': {'path': str(audit_path), 'sha256': sha(audit_path)}})
            bundle_path = put('bundle.json', bundle_data)
            with self.assertRaisesRegex(ValueError, message):
                load_evidence(main_path, scaling_path, bundle_path, protocol_path, index_path)

    def test_bundle_not_complete(self):
        self.audit_fixture_rejected(lambda i,a,b: b.update(complete=False), 'bundle is incomplete')

    def test_input_audit_failure(self):
        self.audit_fixture_rejected(lambda i,a,b: i.update(passed=False), 'Input audit failed')

    def test_independent_audit_required(self):
        self.audit_fixture_rejected(lambda i,a,b: a.update(independent_passed=False), 'independent audit did not complete')

    def test_failures_must_be_independently_reviewed(self):
        self.audit_fixture_rejected(lambda i,a,b: a.update(all_outcomes_reviewed=False), 'all methods and outcome classes')

    def test_metrics_must_be_sealed_by_hash(self):
        self.audit_fixture_rejected(lambda i,a,b: None, 'main metrics are not bound')


if __name__ == '__main__':
    unittest.main()
