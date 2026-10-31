"""Reporting safeguards for all-legal denominators and conditional costs."""
from copy import deepcopy
import numpy as np
import pytest

from benchmarks.generic_motion_v1.reports.paper_comparators_v1.build_report import (
    check_statistics, check_ratio, scope_rows, ratio, paired, validate_metrics, straight_line_rows)
from benchmarks.generic_motion_v1.reports.paper_comparators_v1.audit_comparison import LABELS, SUCCESS


def ratio_record(values, clusters):
    return dict(numerator=sum(values), denominator=len(values),
                estimate=float(np.mean(values)) if values else None,
                independent_scene_clusters=clusters, ci95=[0., 1.] if clusters >= 2 else None,
                bootstrap_seed=45827, bootstrap_replicates=2000)


def test_unavailable_times_are_not_zero_filled():
    check_statistics(dict(count=2, mean=3., p50=3., p90=3.8, p95=3.9), [2., None, 4.])
    with pytest.raises(ValueError):
        check_statistics(dict(count=3, mean=2., p50=2., p90=3.6, p95=3.8), [2., None, 4.])


def test_empty_conditioned_cost_is_not_zero_cost():
    check_statistics(dict(count=0, mean=None, p50=None, p90=None, p95=None), [])
    with pytest.raises(ValueError):
        check_statistics(dict(count=0, mean=0., p50=0., p90=0., p95=0.), [])


def test_timeout_is_in_all_legal_latency():
    check_statistics(dict(count=2, mean=30.5, p50=30.5, p90=54.1, p95=57.05), [1., 60.])
    with pytest.raises(ValueError):
        check_statistics(dict(count=1, mean=1., p50=1., p90=1., p95=1.), [1., 60.])


@pytest.mark.parametrize('field,value', [('denominator', 3), ('numerator', 0), ('estimate', .9),
    ('independent_scene_clusters', 3), ('bootstrap_seed', 0), ('bootstrap_replicates', 100), ('ci95', None)])
def test_cluster_ratio_mutations_are_rejected(field, value):
    saved = ratio_record([1, 0], 2); saved[field] = value
    with pytest.raises(ValueError): check_ratio(saved, [{'cluster_id':'a'}, {'cluster_id':'b'}], [1, 0])


def test_open_control_single_cluster_has_no_ci():
    saved = ratio_record([1, 1], 1)
    check_ratio(saved, [{'cluster_id':'shared'}]*2, [1, 1])
    assert '单一场景' in ratio(saved)
    saved['ci95'] = [1., 1.]
    with pytest.raises(ValueError): check_ratio(saved, [{'cluster_id':'shared'}]*2, [1, 1])


def test_control_never_enters_obstacle_stratum():
    rows = [dict(split='smoke', roadmap_nodes=6000, label='LSCS', family=family)
            for family in ('sparse_clutter', 'open_control')]
    base = dict(split='smoke', roadmap_nodes=6000, label='LSCS')
    assert [r['family'] for r in scope_rows(rows, dict(base, scope='obstacle_main'))] == ['sparse_clutter']
    assert [r['family'] for r in scope_rows(rows, dict(base, scope='open_control'))] == ['open_control']
    with pytest.raises(ValueError): scope_rows(rows, dict(base, scope='unknown'))


def paired_fixture():
    rows = []
    for i in range(180):
        for label, success, cost in [('LGCS', i<100, 1.), ('LSCS', i<110, 2.)]:
            rows.append(dict(case_id=str(i), roadmap_id='r', method_seed=0,
                split='dev', family='clutter', label=label, input_legal=True,
                cluster_id=str(i//9), success=success, cost=cost if success else None, cold_e2e_seconds=3.))
    p = dict(split='dev', roadmap_nodes=6000, scope='obstacle_main', objective='minimum_distance',
             method_a='LGCS', method_b='LSCS', paired_legal_jobs=180,
             valid_success_difference=ratio_record([0]*100+[-1]*10+[0]*70, 20),
             common_success_cost_difference=ratio_record([-1.]*100, 12),
             observed_cold_latency_difference=ratio_record([0.]*180, 20))
    p['valid_success_difference']['ci95']=[-.2,0.]
    p['common_success_cost_difference']['ci95']=[-1.,-1.]
    p['observed_cold_latency_difference']['ci95']=[0.,0.]
    return dict(rows=rows, paired_comparisons=[p])


def test_pair_orientation_and_joint_success_denominator():
    output = paired(paired_fixture(), ('minimum_distance','LSCS','LGCS'))
    assert output['common_success_jobs'] == 100
    assert output['common_success_cost_difference']['estimate'] == 1.
    assert output['common_success_cost_difference']['ci95'] == [1.,1.]
    assert output['valid_success_difference']['estimate'] == pytest.approx(10/180)


def test_pair_cost_cannot_use_single_method_success_count():
    data = paired_fixture()
    data['paired_comparisons'][0]['common_success_cost_difference']['denominator'] = 110
    with pytest.raises(ValueError): paired(data, ('minimum_distance','LSCS','LGCS'))


def test_paired_comparison_cannot_be_missing():
    data = paired_fixture(); data['paired_comparisons'] = []
    with pytest.raises(ValueError): paired(data, ('minimum_distance','LSCS','LGCS'))


def test_report_rejects_failed_or_stale_audit_before_rendering():
    with pytest.raises(ValueError, match='passing'):
        validate_metrics({}, dict(passed=False, errors=['incomplete']), 'sha')
    with pytest.raises(ValueError, match='different metrics'):
        validate_metrics({}, dict(passed=True, errors=[], metrics_sha256='old'), 'new')


def straight_line_fixture():
    groups = {}
    for label in LABELS:
        groups['dev', 'obstacle_main', label] = dict(legal_jobs=180, legal_status_counts={SUCCESS:35})
        groups['dev', 'straight_line:True', label] = dict(legal_jobs=141, legal_status_counts={})
        groups['dev', 'straight_line:False', label] = dict(legal_jobs=39, legal_status_counts={SUCCESS:35})
    return groups


def test_blocked_zero_success_is_displayed_without_dropping_denominator():
    rows = straight_line_rows(straight_line_fixture())
    assert len(rows) == 7
    assert all(row[1:] == ['0/141', '35/39'] for row in rows)
    assert sum('refined' in row[0] for row in rows) == 2


@pytest.mark.parametrize('field,value', [('legal_jobs', 140), ('legal_status_counts', {SUCCESS:1})])
def test_straight_line_strata_must_partition_main(field, value):
    groups = straight_line_fixture()
    groups['dev', 'straight_line:True', 'LSCS'][field] = value
    with pytest.raises(ValueError, match='partition'):
        straight_line_rows(groups)


def test_unknown_straight_line_group_cannot_be_silently_omitted():
    groups = straight_line_fixture()
    groups.pop(('dev', 'straight_line:False', 'NTO-distance'))
    with pytest.raises(KeyError): straight_line_rows(groups)
