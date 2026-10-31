"""Configuration boundaries must fail before expensive or irreversible work."""
from copy import deepcopy
import json
from pathlib import Path
import shutil

import pytest

from research_platform.config import (
    BASE_PROTOCOL, FOREST_PROTOCOL, SCENE_PROFILE, FOREST_REFERENCE, FR3_ASSET,
    ROOT, ConfigError, case_grid, config_hash, load_config, resolve_config, validate_config,
)


@pytest.fixture
def config():
    return dict(version=1, name='unit_research', environment='forest',
                environment_seeds=[0], roadmap_seeds=[0], roadmap_sizes=[200],
                methods=['DRM', 'LSCS'], scene_profile='figure6')


@pytest.fixture
def source_repo(tmp_path):
    for name in [BASE_PROTOCOL, FOREST_PROTOCOL, SCENE_PROFILE, FOREST_REFERENCE, FR3_ASSET]:
        target=tmp_path/name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT/name, target)
    return tmp_path


def test_default_parameters_are_visible_and_input_is_not_mutated(config):
    before=deepcopy(config)
    canonical=validate_config(config)
    assert canonical['planning']=={'lazy_edge_step': .01, 'validation_step': .002}
    assert canonical['limits']=={'job_timeout_seconds': 120., 'solver_seconds': 30.}
    canonical['methods'].append('SCSTO')
    assert config==before


@pytest.mark.parametrize('section,key', [(None,'methdos'), ('limits','timeuot'),
                                         ('planning','validation'), ('method_options','nto_margin')])
def test_misspelled_keys_do_not_silently_change_experiment(config, section, key):
    if section is None: config[key]=1
    else: config[section]={key:1}
    with pytest.raises(ConfigError, match='unknown keys'):
        validate_config(config)


@pytest.mark.parametrize('bad_name', ['../other', '/absolute', 'a/b', r'a\b', '.', '', 'UPPER', 'with space'])
def test_run_names_cannot_traverse_or_ambiguously_address_paths(config, bad_name):
    config['name']=bad_name
    with pytest.raises(ConfigError, match='name'):
        validate_config(config)


@pytest.mark.parametrize('field,value', [
    ('version', True), ('version', 2), ('environment_seeds', [True]),
    ('environment_seeds', [-1]), ('environment_seeds', [0.0]),
    ('roadmap_seeds', [1,1]), ('roadmap_seeds', []),
    ('roadmap_sizes', [200,200]), ('roadmap_sizes', [0]),
    ('methods', ['LSCS','LSCS']), ('methods', ['fictional_solver']),
])
def test_invalid_ids_and_duplicate_experiments_are_rejected(config, field, value):
    config[field]=value
    with pytest.raises(ConfigError):
        validate_config(config)


@pytest.mark.parametrize('limits', [
    {'job_timeout_seconds': 0}, {'solver_seconds': -1},
    {'job_timeout_seconds': True}, {'job_timeout_seconds': 10**400},
    {'job_timeout_seconds': 30, 'solver_seconds': 30},
    {'job_timeout_seconds': 20, 'solver_seconds': 30},
])
def test_solver_budget_cannot_escape_outer_watchdog(config, limits):
    config['limits']=limits
    with pytest.raises(ConfigError):
        validate_config(config)


@pytest.mark.parametrize('raw', [
    '{"version":1,"version":2}',
    '{"limits":{"solver_seconds":10,"solver_seconds":20}}',
    '{"version":NaN}', '{"version":Infinity}', '{"version":-Infinity}',
])
def test_invalid_json_is_rejected_before_schema_resolution(tmp_path, raw):
    path=tmp_path/'invalid.json';path.write_text(raw)
    with pytest.raises(ConfigError): load_config(path)


@pytest.mark.parametrize('nonfinite', [float('nan'),float('inf'),float('-inf')])
def test_nonfinite_values_cannot_enter_hash_or_config(config, nonfinite):
    config['planning']={'lazy_edge_step': nonfinite}
    with pytest.raises(ConfigError): validate_config(config)
    with pytest.raises(ConfigError): config_hash(config)


def test_overflowing_json_numeric_value_is_rejected(config, tmp_path):
    raw=json.dumps(config)[:-1]+',"limits":{"job_timeout_seconds":1e999}}'
    path=tmp_path/'overflow.json';path.write_text(raw)
    with pytest.raises(ConfigError):load_config(path)


def test_validation_cannot_be_coarser_than_planning(config):
    config['planning']={'lazy_edge_step': .01, 'validation_step': .1}
    with pytest.raises(ConfigError,match='validation_step'):validate_config(config)


def test_fixed_illustration_cannot_be_counted_as_multiple_random_environments(config):
    config['environment_seeds']=[0,1]
    with pytest.raises(ConfigError,match='one fixed illustration'):validate_config(config)
    config['scene_profile']='random'
    assert len(case_grid(config))==2


def test_cross_environment_scene_profiles_are_rejected(config):
    config['scene_profile']='public_table'
    with pytest.raises(ConfigError):validate_config(config)
    config['environment']='fr3'
    assert validate_config(config)['scene_profile']=='public_table'
    config['scene_profile']='random'
    with pytest.raises(ConfigError):validate_config(config)


def test_case_grid_is_complete_and_deterministic(config):
    config.update(scene_profile='random',environment_seeds=[1,9],roadmap_seeds=[2,3],roadmap_sizes=[200,800])
    grid=case_grid(config)
    assert len(grid)==8 and len({c['id'] for c in grid})==8
    assert grid[0]==dict(id='e001_r002_n00200',environment_seed=1,roadmap_seed=2,roadmap_size=200)
    assert grid[-1]==dict(id='e009_r003_n00800',environment_seed=9,roadmap_seed=3,roadmap_size=800)
    assert grid==case_grid(config)


def test_equivalent_defaults_and_json_key_order_have_one_identity(config,tmp_path):
    canonical=validate_config(config)
    path=tmp_path/'config.json'
    path.write_text(json.dumps(dict(reversed(list(canonical.items()))),indent=4))
    assert config_hash(load_config(path))==config_hash(canonical)
    assert config_hash(validate_config(config))==config_hash(canonical)


def test_resolved_protocol_applies_actual_settings_and_freezes_figure(config,source_repo):
    config.update(methods=['DRM','NTO_distance','SCSTO'],
        limits={'job_timeout_seconds': 200,'solver_seconds':45},
        planning={'lazy_edge_step':.02,'validation_step':.001},
        method_options={'nto_collision_bound':.003,'nto_major_iterations':500,'bezier_degree':5})
    resolved=resolve_config(config,source_repo)
    protocol=resolved['protocol'];scene=resolved['scene_definition']
    assert protocol['optimizer_time_limit_seconds']==45
    assert protocol['case_timeout_seconds']==200
    assert protocol['lazy_edge_step']==.02 and protocol['validation_step']==.001
    assert protocol['nto_collision_bound']==.003 and protocol['nto_major_iterations']==500
    assert protocol['bezier_degree']==5 and protocol['method_order']==['NTO_distance','SCSTO']
    assert scene['profile']=='figure6' and len(scene['centers'])==15
    assert protocol['start']==scene['start'] and protocol['goal']==scene['goal']
    identity=resolved.pop('resolved_sha256')
    assert identity==config_hash(resolved)


def test_random_forest_does_not_reuse_the_illustration_obstacles(config,source_repo):
    config.update(scene_profile='random',environment_seeds=[0,1,2])
    scene=resolve_config(config,source_repo)['scene_definition']
    assert 'centers' not in scene and 'obstacle_centers' not in scene
    assert scene['sampling']['algorithm']=='numpy.default_rng(environment_seed).uniform'
    assert scene['sampling']['count']==15
    assert scene['sampling']['center_lower']==[-3.5,-3.5]


def test_fr3_resolves_declared_public_table_and_retains_unknowns(config,source_repo):
    config.update(environment='fr3',scene_profile='public_table',roadmap_sizes=[3000])
    result=resolve_config(config,source_repo)
    assert result['protocol']['fr3_scene']['table_size']==[2.5,2.5,.2]
    assert result['protocol']['fr3_scene']['right_ee_upper'][1]<0
    assert result['scene_definition']['evidence']['unknowns']
    assert FR3_ASSET in result['source_hashes']
    assert result['protocol']['assumptions']['random_inputs']


def test_changed_source_and_research_option_change_resolved_identity(config,source_repo):
    first=resolve_config(config,source_repo)
    path=source_repo/FOREST_REFERENCE
    reference=json.loads(path.read_text());reference['start'][0]+=.01
    path.write_text(json.dumps(reference))
    second=resolve_config(config,source_repo)
    assert first['config_sha256']==second['config_sha256']
    assert first['resolved_sha256']!=second['resolved_sha256']
    assert first['source_hashes'][FOREST_REFERENCE]!=second['source_hashes'][FOREST_REFERENCE]
    config['method_options']={'nto_collision_bound':.002}
    third=resolve_config(config,source_repo)
    assert second['config_sha256']!=third['config_sha256']
    assert second['resolved_sha256']!=third['resolved_sha256']


def test_source_symlink_cannot_escape_declared_repository(config,source_repo,tmp_path_factory):
    external=tmp_path_factory.mktemp('external')/'reference.json'
    path=source_repo/FOREST_REFERENCE
    shutil.copyfile(path,external);path.unlink();path.symlink_to(external)
    with pytest.raises(ConfigError,match='escapes repository'):resolve_config(config,source_repo)


@pytest.mark.parametrize('name,expected_cases', [('forest_smoke',1),('fr3_smoke',1),('forest_batch',3)])
def test_shipped_configs_resolve_without_gpu(name,expected_cases):
    config=load_config(ROOT/'configs/research'/f'{name}.json')
    result=resolve_config(config)
    assert len(case_grid(config))==expected_cases
    assert len(config['methods'])==6
    assert result['resolved_sha256']


def test_empty_scene_options_preserve_existing_config_and_resolved_identity(config,source_repo):
    before=resolve_config(config,source_repo)
    config['scene_options']={}
    after=resolve_config(config,source_repo)
    assert before==after
    assert 'scene_options' not in after['config']


def test_forest_geometry_options_reach_scene_and_native_protocol(config,source_repo):
    config['scene_profile']='random'
    config['scene_options']=dict(obstacles=9,obstacle_radius=.2,domain_lower=[-6,-4],domain_upper=[6,4],
        obstacle_center_lower=[-2,-1],obstacle_center_upper=[2,1],start=[-5,-3],goal=[5,3])
    resolved=resolve_config(config,source_repo)
    scene=resolved['scene_definition'];protocol=resolved['protocol']
    for key,value in config['scene_options'].items():
        assert scene[key]==value and protocol[key]==value
    assert scene['sampling']==dict(algorithm='numpy.default_rng(environment_seed).uniform',
        center_lower=[-2.,-1.],center_upper=[2.,1.],count=9)
    assert scene['explicit_scene_overrides']['values']['obstacles']==9
    assert protocol['research_platform']['explicit_scene_overrides']==scene['explicit_scene_overrides']


def test_fr3_geometry_and_endpoint_budget_options_reach_backend(config,source_repo):
    config.update(environment='fr3',scene_profile='public_table',roadmap_sizes=[3000],
        scene_options=dict(obstacle_count=12,obstacle_radius=.07,obstacle_lower=[.3,-.3,.1],
            obstacle_upper=[.7,.3,.7],right_ee_lower=[.25,-.6,.25],right_ee_upper=[.75,-.2,.9],
            left_ee_lower=[.25,.2,.25],left_ee_upper=[.75,.6,.9],table_size=[2,2,.3],
            table_center=[.4,0,-.15],offline_map_size=[2,2,2],offline_map_center=[.4,0,.7],
            endpoint_sampling_limit=5000,ik_attempts=4))
    resolved=resolve_config(config,source_repo)
    effective=resolved['protocol']['fr3_scene']
    for key,value in config['scene_options'].items():assert effective[key]==value
    assert resolved['scene_definition']['fr3_scene']==effective
    assert resolved['scene_definition']['explicit_scene_overrides']


@pytest.mark.parametrize('options', [
    {'obstacles':0},{'obstacles':True},{'obstacle_radius':0},
    {'obstacle_center_lower':[-1]}, {'domain_upper':[5,True]},
    {'domain_lower':[4,4],'domain_upper':[-4,-4]},
    {'obstacle_center_lower':[1,1],'obstacle_center_upper':[0,0]},
    {'start':[float('nan'),0]}, {'table_size':[1,1,1]}, {'typo':1},
])
def test_invalid_forest_geometry_is_rejected_before_generation(config,options):
    config.update(scene_profile='random',scene_options=options)
    with pytest.raises(ConfigError):validate_config(config)


def test_figure_artwork_cannot_be_silently_modified(config):
    config['scene_options']={'obstacle_radius':.4}
    with pytest.raises(ConfigError,match='geometry is fixed'):validate_config(config)


@pytest.mark.parametrize('options', [
    {'obstacle_count':0},{'table_size':[1,0,1]},{'offline_map_size':[1,-1,1]},
    {'obstacle_lower':[1,1]},{'left_ee_lower':[0,.5,0],'left_ee_upper':[1,.1,1]},
    {'endpoint_sampling_limit':0},{'ik_attempts':False},{'domain_upper':[1,1]},
])
def test_invalid_fr3_geometry_is_rejected_before_generation(config,options):
    config.update(environment='fr3',scene_profile='public_table',scene_options=options)
    with pytest.raises(ConfigError):validate_config(config)


@pytest.mark.parametrize('options', [
    {'start':[-100,0]}, {'goal':[0,100]}, {'domain_upper':[-6,5]},
    {'obstacle_center_upper':[100,100]},
])
def test_partial_forest_override_checked_against_resolved_defaults(config,source_repo,options):
    config.update(scene_profile='random',scene_options=options)
    with pytest.raises(ConfigError):resolve_config(config,source_repo)


def test_partial_fr3_override_checked_against_resolved_map_and_box(config,source_repo):
    config.update(environment='fr3',scene_profile='public_table',scene_options={'obstacle_upper':[9,.4,.8]})
    with pytest.raises(ConfigError,match='offline collision-map'):resolve_config(config,source_repo)
    config['scene_options']={'right_ee_lower':[.2,.5,.2]}
    with pytest.raises(ConfigError,match='right_ee_lower'):resolve_config(config,source_repo)


def test_explicit_geometry_change_has_distinct_research_identity(config,source_repo):
    config['scene_profile']='random'
    original=resolve_config(config,source_repo)
    config['scene_options']={'obstacle_radius':.4}
    changed=resolve_config(config,source_repo)
    assert original['config_sha256']!=changed['config_sha256']
    assert original['resolved_sha256']!=changed['resolved_sha256']
    assert original['source_hashes']==changed['source_hashes']
