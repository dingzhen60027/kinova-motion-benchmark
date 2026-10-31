"""Backend opt-ins must not rewrite the historical evaluation contract."""
import pytest

from benchmarks.generic_motion_v1.roadmap import HERE, load_config


def test_original_profile_keeps_implicit_gurobi_contract():
    original = load_config(HERE / 'configs/rss2025_eizo.json')
    assert 'lgcs' not in original
    explicit = load_config({'lgcs': {'solver': 'gurobi'}})
    assert explicit.pop('lgcs') == {'solver': 'gurobi'}
    assert explicit == load_config()


def test_open_solver_profile_keeps_identical_model_and_eizo_settings():
    original = load_config(HERE / 'configs/rss2025_eizo.json')
    revised = load_config(HERE / 'configs/rss2025_reliable.json')
    assert revised.pop('lgcs') == {'solver': 'scip'}
    assert revised.pop('drm_query')['backend'] == 'safe_native'
    assert revised == original


@pytest.mark.parametrize('value', [None, 'scip', {}, {'solver': 'auto'},
                                    {'solver': 'scip', 'fallback': True}])
def test_unsupported_or_implicit_fallback_solver_is_rejected(value):
    with pytest.raises(ValueError, match='lgcs'):
        load_config({'lgcs': value})


@pytest.mark.parametrize('value', [None, {}, {'backend': 'python'},
    {'backend': 'safe_native', 'build_dir': ''},
    {'backend': 'safe_native', 'build_dir': '/tmp/build', 'budget_multiplier': 2}])
def test_undeclared_native_query_adapters_are_rejected(value):
    with pytest.raises(ValueError, match='drm_query'):
        load_config({'drm_query': value})
