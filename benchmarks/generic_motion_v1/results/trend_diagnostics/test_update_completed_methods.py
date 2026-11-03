"""Small CPU fixtures for catalog refresh; these are not planning observations."""
import copy
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import update_completed_methods as refresh


def fixture():
    methods = sorted({method for method, _ in refresh.audit.PAIRS})
    original = dict(methods=[dict(method=method, display_name=method+' retained identity',
        objectives=[obj for name, obj in refresh.audit.PAIRS if name == method],
        roadmap_required=method not in ('RRTCONNECT', 'RRTSTAR'), stopping='original backend stop',
        implementation='original reviewed implementation', baseline_eligible=True,
        source='reviewed source for '+method, source_files=['reviewed/'+method+'.py'],
        constraints='original canonical constraints', native_objective='original optimization objective',
        distinction='historical backend still unresolved', correctness='historical correctness description',
        verification='historical verification description', initialization='historical initialization description',
        actual_coverage={'end_to_end': {'old': 'historical score'}}, repaired_dev_seed0={'old': 1})
        for method in methods], source_sha256={'runners/methods.py': 'old'},
        known_backend_issue={'evidence': 'historical_backend.json', 'status': 'Unfixed'})
    cfg = dict(drm={'max_number_planning_attempts': 1000, 'max_iterations_steering_to_node': 1000,
                    'max_nodes_to_expand': 100000, 'online_edge_step_size': .01},
               ompl={'planning_seconds': 60., 'validation_reserve_seconds': 5., 'simplification_seconds': .5},
               nto={'minimum_control_points': 16}, nto_refined={'minimum_time_control_points': 16},
               drm_query={'backend': 'safe_native', 'build_dir': 'native/build/drm_consistent_query_portable'})
    sources = {'benchmarks/generic_motion_v1/'+key: 'frozen-'+key for key in (
        'runners/methods.py', 'runners/native_drm_corrections.py', 'runners/nto_collision_constraint.py')}
    manifests = [dict(split=split, timeout_seconds=60, source_hashes=sources,
                      source_code_commit='frozen-commit', jobs=[{'evaluation_config': cfg}])
                 for split in refresh.audit.SEEDS]
    specs = [dict(id='current_'+split+'_repaired', mode='end_to_end', runs=['run_'+split])
             for split in refresh.audit.SEEDS]
    records, summary = [], []
    for spec, split in zip(specs, refresh.audit.SEEDS):
        for method, objective in refresh.audit.PAIRS:
            records.append(dict(method=method, objective=objective, split=split, run='run_'+split,
                replica_seed='0', status='success_numerical', input_valid='True',
                selected_comparisons=json.dumps([spec['id']])))
            summary.append(dict(comparison=spec['id'], mode='end_to_end', method=method,
                objective=objective, split=split, category='all', row_type='status',
                planned_jobs='1', actual_attempts='1', legal_jobs='1', finished_legal_jobs='1',
                invalid_input_jobs='0', pending_jobs='0', success_numerical='1',
                **{state: '0' for state in refresh.FAILURES}))
    return original, records, summary, specs, manifests


def build(data, acceptance=None):
    return refresh.build_catalog(*data, acceptance or {'passed': True, 'complete_primary_suite': True},
                                 {'updated_at_utc': 'synthetic-test-time'})


def test_refresh_preserves_identity_replaces_only_current_coverage_and_describes_protocol():
    data = fixture()
    before = copy.deepcopy(data)
    result = build(data)
    assert data == before  # catalog and source records are never modified in place
    for old, method in zip(data[0]['methods'], result['methods']):
        for key in ('method', 'display_name', 'implementation', 'baseline_eligible', 'objectives',
                    'source', 'source_files', 'constraints', 'native_objective'):
            assert method[key] == old[key]
        if method['method'] not in ('NTO', 'NTO_PAPER', 'NTO_REFINED'):
            for key in ('distinction', 'correctness', 'verification', 'initialization'):
                if method['method'] == 'RRTCONNECT' and key == 'distinction':
                    continue  # Current budget must replace the historical 10-second description.
                assert method[key] == old[key]
        assert 'repaired_dev_seed0' not in method
        assert set(method['actual_coverage']) == {'end_to_end'}
        for objective in method['objectives']:
            assert set(method['actual_coverage']['end_to_end'][objective]) == {'smoke', 'dev', 'test'}
    by_name = {m['method']: m for m in result['methods']}
    assert 'first exact solution' in by_name['RRTCONNECT']['stopping']
    assert 'current native search ceiling is 60 s' in by_name['RRTCONNECT']['distinction']
    assert 'Historical runs used a 10 s ceiling' in by_name['RRTCONNECT']['distinction']
    assert '5 s final-validation' in by_name['RRTSTAR']['stopping']
    assert '1000 endpoint-connection trials per endpoint per attempt' in by_name['EI_ZO']['stopping']
    assert 'no collision constraint is dropped' in by_name['NTO']['collision_backend_repair']
    assert 'Native-first solve' in by_name['NTO']['collision_backend_repair']
    assert 'at most one guarded recovery solve' in by_name['NTO']['collision_backend_repair']
    assert 'same original query deadline' in by_name['NTO']['collision_backend_repair']
    for name in ('NTO', 'NTO_PAPER', 'NTO_REFINED'):
        assert 'Direct native solver callback evaluations are not instrumented' in by_name[name]['collision_evaluation_accounting']
        assert 'not zero collision work' in by_name[name]['collision_evaluation_accounting']
        assert by_name[name]['historical_backend_evidence'] == 'historical_backend.json'
        assert 'complete-cohort' in by_name[name]['verification']
        assert 'no continuous certificate' in by_name[name]['verification']
        assert 'unresolved' not in by_name[name]['distinction']
        assert by_name[name]['correctness'] != 'historical correctness description'
    assert '149 interior k/150' in by_name['NTO_PAPER']['distinction']
    assert 'Real SNOPT path and IPOPT time' in by_name['NTO_PAPER']['correctness']
    assert 'inserts no additional knots or controls' in by_name['NTO_PAPER']['initialization']
    assert 'IPOPT for both tracks' in by_name['NTO']['distinction']
    assert 'at least 16 controls' in by_name['NTO']['initialization']
    assert 'at least 16 controls' in by_name['NTO_REFINED']['initialization']
    assert 'separate native-first collision backend' in by_name['NTO_REFINED']['distinction']
    assert 'Current outcomes are reported only in actual_coverage' in by_name['NTO_REFINED']['current_time_parameterization']
    refined = by_name['NTO_REFINED']['stopping']
    assert 'At most 4 refinement rounds' in refined
    assert 'at most 1 guarded recovery Solve' in refined
    assert 'at most 5 actual solver invocations' in refined
    assert 'share the original 60 s deadline' in refined
    assert '4 native solves' not in refined
    assert 'strictly repaired' in result['known_backend_issue']['status']
    assert result['source_sha256']['runners/methods.py'] == 'frozen-runners/methods.py'
    # A repeat refresh must not stack DRM stopping descriptions.
    repeated = build((result, *data[1:]))
    assert [m['stopping'] for m in repeated['methods']] == [m['stopping'] for m in result['methods']]
    for method in repeated['methods']:
        if method['method'] in ('NTO', 'NTO_PAPER', 'NTO_REFINED'):
            assert method['historical_backend_evidence'] == 'historical_backend.json'


def test_time_control_descriptions_come_from_each_frozen_profile_not_old_partial_scores():
    data = fixture()
    cfg = data[4][0]['jobs'][0]['evaluation_config']
    cfg['nto']['minimum_control_points'] = 20
    cfg['nto_refined']['minimum_time_control_points'] = 24
    refined = next(m for m in data[0]['methods'] if m['method'] == 'NTO_REFINED')
    refined['current_time_parameterization'] = 'old dev success36/60->34/60 under60s'
    methods = {m['method']: m for m in build(data)['methods']}
    assert 'at least 20 controls' in methods['NTO']['initialization']
    assert 'at least 24 controls' in methods['NTO_REFINED']['initialization']
    assert 'At least 24 time-spline controls' in methods['NTO_REFINED']['current_time_parameterization']
    assert '36/60' not in methods['NTO_REFINED']['current_time_parameterization']
    assert 'inserts no additional knots or controls' in methods['NTO_PAPER']['initialization']


@pytest.mark.parametrize('defect', ['missing_summary', 'duplicate_summary', 'wrong_count', 'old_ompl_cap', 'old_drm', 'missing_source'])
def test_refresh_rejects_incomplete_or_stale_evidence(defect):
    data = fixture()
    if defect == 'missing_summary':
        data[2].pop()
    elif defect == 'duplicate_summary':
        data[2].append(data[2][0])
    elif defect == 'wrong_count':
        data[2][0]['success_numerical'] = '0'
    elif defect == 'old_ompl_cap':
        data[4][0]['jobs'][0]['evaluation_config']['ompl']['planning_seconds'] = 10
    elif defect == 'old_drm':
        data[4][0]['jobs'][0]['evaluation_config']['drm']['max_number_planning_attempts'] = 10
    elif defect == 'missing_source':
        data[4][0]['source_hashes'].pop('benchmarks/generic_motion_v1/runners/methods.py')
    with pytest.raises(ValueError):
        build(data)


def test_incomplete_primary_acceptance_cannot_write_catalog():
    with pytest.raises(ValueError, match='three repaired primary'):
        build(fixture(), {'passed': True, 'complete_primary_suite': False})


def test_pending_cli_leaves_catalog_bytes_unchanged_and_skips_expensive_verification(tmp_path, monkeypatch):
    selection, catalog, report = (tmp_path/name for name in ('selection.json', 'methods.json', 'update.json'))
    selection.write_text(json.dumps({'comparisons': [dict(id='test', runs=[str(tmp_path/'not_prepared')])]}))
    catalog.write_bytes(b'{"historical_catalog":true}\n')
    before = catalog.read_bytes()
    def forbidden(*args, **kwargs):
        pytest.fail('Incomplete runs must not start expensive acceptance or catalog derivation')
    monkeypatch.setattr(refresh.audit, 'review', forbidden)
    assert refresh.main(['--comparisons', 'test', '--selection', str(selection), '--output', str(catalog),
                         '--report', str(report)]) == 2
    assert catalog.read_bytes() == before
    assert not report.exists()
    pending = json.loads(report.with_name('update.pending.json').read_text())
    assert pending['status'] == 'pending' and pending['catalog_written'] is False


def test_failed_acceptance_does_not_write_catalog(tmp_path, monkeypatch):
    run = tmp_path/'run'
    run.mkdir()
    selection, catalog = tmp_path/'selection.json', tmp_path/'methods.json'
    selection.write_text(json.dumps({'comparisons': [dict(id='all', runs=[str(run)])]}))
    catalog.write_bytes(b'{"historical_catalog":true}\n')
    monkeypatch.setattr(refresh, 'preflight', lambda runs: [{'state': 'ready_for_verification'}])
    monkeypatch.setattr(refresh.audit, 'review', lambda *a, **kw:
                        {'passed': False, 'status': 'failed', 'errors': ['receipt mismatch']})
    args = SimpleNamespace(selection=selection, output=catalog, comparisons=['all'],
                           records=tmp_path/'records.csv', summary=tmp_path/'summary.csv', reference_run=None)
    before = catalog.read_bytes()
    result = refresh.update(args)
    assert result['status'] == 'failed' and result['catalog_written'] is False
    assert catalog.read_bytes() == before


def test_preflight_does_not_treat_interrupted_receipt_as_terminal(tmp_path):
    run = tmp_path/'run'
    receipt = run/'jobs'/'j'/'attempts'/'0001'/'receipt.json'
    receipt.parent.mkdir(parents=True)
    receipt.write_text(json.dumps({'retry_pending': True}))
    (run/'run_manifest.json').write_text(json.dumps({'split': 'smoke', 'jobs': [{'job_id': 'j'}]}))
    assert refresh.preflight([run])[0]['state'] == 'pending'
    receipt.write_text(json.dumps({'retry_pending': False}))
    assert refresh.preflight([run])[0]['state'] == 'ready_for_verification'


@pytest.mark.parametrize('later_retry_pending', [None, True])
def test_preflight_later_incomplete_retry_cannot_hide_terminal_attempt(tmp_path, later_retry_pending):
    run = tmp_path/'run'
    first = run/'jobs'/'j'/'attempts'/'0001'
    later = first.with_name('0002')
    first.mkdir(parents=True)
    later.mkdir()
    (first/'receipt.json').write_text(json.dumps({'retry_pending': False, 'status': 'timeout'}))
    if later_retry_pending is not None:
        (later/'receipt.json').write_text(json.dumps({'retry_pending': later_retry_pending}))
    (run/'run_manifest.json').write_text(json.dumps({'split': 'smoke', 'jobs': [{'job_id': 'j'}]}))
    result = refresh.preflight([run])[0]
    assert result['state'] == 'ready_for_verification' and result['terminal'] == 1


def test_comparison_ids_follow_selection_without_binding_old_smoke_directory(tmp_path):
    names = ('repaired_native_smoke', 'repaired_dev_all_seeds', 'repaired_test')
    selection = {'comparisons': [dict(id=identifier, mode='end_to_end', runs=[str(tmp_path/name)])
                  for identifier, name in zip(refresh.DEFAULT_COMPARISONS, names)]}
    runs, specs = refresh.audit.resolve_scope(selection, comparison_args=refresh.DEFAULT_COMPARISONS)
    assert set(runs) == {tmp_path/name for name in names}
    assert {spec['id'] for spec in specs} == set(refresh.DEFAULT_COMPARISONS)
