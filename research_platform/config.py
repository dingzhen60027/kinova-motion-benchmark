"""Strict, CPU-only configuration and source-resolved experiment identity.

All defaults are explicit in the canonical user configuration. Resolved protocol
identity additionally includes the exact legacy configuration and scene assets.
No planner, model, CUDA library, or numerical runtime is imported here.
"""
from __future__ import annotations

from copy import deepcopy
import hashlib
import itertools
import json
import math
from pathlib import Path
import re
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
SCHEMA_VERSION = 1
METHODS = ('DRM', 'LSCS', 'LGCS', 'SCSTO', 'NTO_distance', 'NTO_time',
           'NTO_distance_strong', 'NTO_time_strong')
ROOT_KEYS = {'version', 'name', 'environment', 'environment_seeds', 'roadmap_seeds',
             'roadmap_sizes', 'methods', 'scene_profile', 'limits', 'planning', 'method_options', 'scene_options', 'strong_nto'}
REQUIRED_KEYS = ROOT_KEYS - {'limits', 'planning', 'method_options', 'scene_options', 'strong_nto'}
LIMIT_DEFAULTS = {'job_timeout_seconds': 120.0, 'solver_seconds': 30.0}
PLANNING_DEFAULTS = {'lazy_edge_step': 0.01, 'validation_step': 0.002}
OPTION_KEYS = {'nto_collision_bound', 'nto_influence_distance', 'nto_major_iterations',
               'bezier_degree', 'scsto_relative_tolerance', 'maximum_recovery_iterations'}
STRONG_NTO_DEFAULTS = {
    'minimum_control_points': 16, 'maximum_control_points': 64,
    'initial_collocation_points': 150, 'maximum_collocation_points': 1000,
    'maximum_refinements': 5, 'maximum_new_points': 64, 'clearance': 0.001,
}
BASE_PROTOCOL = 'reproduction/paper_constructed/config.json'
FOREST_PROTOCOL = 'reproduction/configs/forest.json'
SCENE_PROFILE = 'reproduction/environment_reconstruction/profile.json'
FOREST_REFERENCE = 'reproduction/reports/environment_reconstruction/forest_evidence/scene.json'
FR3_ASSET = 'reproduction/paper_constructed/assets/fr3_33.json'
FOREST_SCENE_KEYS = {'obstacles', 'obstacle_radius', 'obstacle_center_lower', 'obstacle_center_upper',
                     'domain_lower', 'domain_upper', 'start', 'goal'}
FR3_SCENE_KEYS = {'obstacle_count', 'obstacle_radius', 'obstacle_lower', 'obstacle_upper',
                  'right_ee_lower', 'right_ee_upper', 'left_ee_lower', 'left_ee_upper',
                  'table_size', 'table_center', 'offline_map_size', 'offline_map_center',
                  'endpoint_sampling_limit', 'ik_attempts'}


class ConfigError(ValueError):
    """A configuration cannot identify a valid, reproducible experiment."""


def _object(value: Any, label: str, allowed: set[str], required: set[str] = frozenset()):
    if type(value) is not dict:
        raise ConfigError(f'{label} must be a JSON object')
    unknown = set(value) - allowed
    if unknown:
        raise ConfigError(f'{label} has unknown keys: {sorted(unknown)}')
    missing = required - set(value)
    if missing:
        raise ConfigError(f'{label} is missing required keys: {sorted(missing)}')
    return value


def _finite_tree(value: Any, label: str = 'config'):
    if type(value) is float and not math.isfinite(value):
        raise ConfigError(f'{label} must not contain NaN or infinity')
    if isinstance(value, dict):
        if any(type(k) is not str for k in value):
            raise ConfigError(f'{label} object keys must be strings')
        for key, child in value.items():
            _finite_tree(child, f'{label}.{key}')
    elif isinstance(value, list):
        for index, child in enumerate(value):
            _finite_tree(child, f'{label}[{index}]')


def _int(value, label, minimum, maximum):
    if type(value) is not int or not minimum <= value <= maximum:
        raise ConfigError(f'{label} must be an integer in [{minimum}, {maximum}]')
    return value


def _number(value, label, *, minimum=0.0, inclusive=False, maximum=None):
    try:
        finite = type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        finite = False
    if not finite:
        raise ConfigError(f'{label} must be a finite number, not a boolean')
    if (value < minimum if inclusive else value <= minimum):
        relation = '>=' if inclusive else '>'
        raise ConfigError(f'{label} must be {relation} {minimum}')
    if maximum is not None and value > maximum:
        raise ConfigError(f'{label} must be <= {maximum}')
    return float(value)


def _integer_list(value, label, minimum, maximum):
    if type(value) is not list or not value:
        raise ConfigError(f'{label} must be a nonempty array')
    result = [_int(item, f'{label}[{i}]', minimum, maximum) for i, item in enumerate(value)]
    if len(set(result)) != len(result):
        raise ConfigError(f'{label} must not contain duplicates')
    return result


def _vector(value, label, dimension, positive=False):
    if type(value) is not list or len(value) != dimension:
        raise ConfigError(f'{label} must contain exactly {dimension} finite coordinates')
    result = []
    for index, item in enumerate(value):
        if positive:
            result.append(_number(item, f'{label}[{index}]'))
        else:
            try:
                finite = type(item) in (int, float) and math.isfinite(item)
            except OverflowError:
                finite = False
            if not finite:
                raise ConfigError(f'{label}[{index}] must be finite and not a boolean')
            result.append(float(item))
    return result


def _ordered_bounds(values, pairs):
    for lower, upper in pairs:
        if lower in values and upper in values and not all(a < b for a, b in zip(values[lower], values[upper])):
            raise ConfigError(f'scene_options.{lower} must be strictly below {upper} in every coordinate')


def _scene_options(raw, environment, profile):
    allowed = FOREST_SCENE_KEYS if environment == 'forest' else FR3_SCENE_KEYS
    raw = _object(raw, 'scene_options', allowed)
    if raw and profile == 'figure6':
        raise ConfigError('figure6 scene geometry is fixed; use random for explicit scene_options')
    result = {}
    for key, value in raw.items():
        if key in ('obstacles', 'obstacle_count'):
            result[key] = _int(value, f'scene_options.{key}', 1, 10000)
        elif key == 'endpoint_sampling_limit':
            result[key] = _int(value, f'scene_options.{key}', 1, 10000000)
        elif key == 'ik_attempts':
            result[key] = _int(value, f'scene_options.{key}', 1, 10000)
        elif key == 'obstacle_radius':
            result[key] = _number(value, f'scene_options.{key}')
        else:
            result[key] = _vector(value, f'scene_options.{key}', 2 if environment == 'forest' else 3,
                                  positive=key in ('table_size', 'offline_map_size'))
    pairs = [('domain_lower','domain_upper'),('obstacle_center_lower','obstacle_center_upper')] if environment == 'forest' else [
        ('obstacle_lower','obstacle_upper'),('right_ee_lower','right_ee_upper'),('left_ee_lower','left_ee_upper')]
    _ordered_bounds(result, pairs)
    return result


def _effective_scene_bounds(scene, environment):
    pairs = [('domain_lower','domain_upper'),('obstacle_center_lower','obstacle_center_upper')] if environment == 'forest' else [
        ('obstacle_lower','obstacle_upper'),('right_ee_lower','right_ee_upper'),('left_ee_lower','left_ee_upper')]
    _ordered_bounds(scene, pairs)
    if environment == 'forest':
        for key in ('start','goal','obstacle_center_lower','obstacle_center_upper'):
            if not all(a <= x <= b for a,x,b in zip(scene['domain_lower'],scene[key],scene['domain_upper'])):
                raise ConfigError(f'Effective {key} must lie inside the Forest domain')
    else:
        lower=[c-size/2 for c,size in zip(scene['offline_map_center'],scene['offline_map_size'])]
        upper=[c+size/2 for c,size in zip(scene['offline_map_center'],scene['offline_map_size'])]
        for key in ('obstacle_lower','obstacle_upper'):
            if not all(a <= x <= b for a,x,b in zip(lower,scene[key],upper)):
                raise ConfigError(f'Effective {key} must lie inside the offline collision-map workspace')


def _json_pairs(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ConfigError(f'Duplicate JSON key: {key}')
        result[key] = value
    return result


def _invalid_constant(token):
    raise ConfigError(f'Nonstandard JSON numeric constant: {token}')


def _read_json(path):
    try:
        return json.loads(Path(path).read_text(encoding='utf-8'),
                          object_pairs_hook=_json_pairs, parse_constant=_invalid_constant)
    except (json.JSONDecodeError, UnicodeDecodeError) as exc:
        raise ConfigError(f'Invalid JSON in {path}: {exc}') from exc


def validate_config(config: dict) -> dict:
    """Return a fresh canonical config, rejecting unknown or unsafe fields."""
    _finite_tree(config)
    raw = _object(config, 'config', ROOT_KEYS, REQUIRED_KEYS)
    version = _int(raw['version'], 'version', SCHEMA_VERSION, SCHEMA_VERSION)
    name = raw['name']
    if type(name) is not str or re.fullmatch(r'[a-z][a-z0-9_-]{0,63}', name) is None:
        raise ConfigError('name must be a lowercase slug of 1-64 characters; paths are not allowed')
    environment = raw['environment']
    if type(environment) is not str or environment not in ('forest', 'fr3'):
        raise ConfigError('environment must be forest or fr3')
    profile = raw['scene_profile']
    choices = ('figure6', 'random') if environment == 'forest' else ('public_table',)
    if type(profile) is not str or profile not in choices:
        raise ConfigError(f'scene_profile for {environment} must be one of {choices}')
    environments = _integer_list(raw['environment_seeds'], 'environment_seeds', 0, 2**32-1)
    roadmaps = _integer_list(raw['roadmap_seeds'], 'roadmap_seeds', 0, 2**32-1)
    sizes = _integer_list(raw['roadmap_sizes'], 'roadmap_sizes', 2, 1_000_000)
    if profile == 'figure6' and environments != [0]:
        raise ConfigError('figure6 is one fixed illustration; environment_seeds must be exactly [0]')
    methods = raw['methods']
    if type(methods) is not list or not methods or any(type(m) is not str or m not in METHODS for m in methods):
        raise ConfigError(f'methods must be a nonempty array selected from {METHODS}')
    if len(set(methods)) != len(methods):
        raise ConfigError('methods must not contain duplicates')
    if len(environments)*len(roadmaps)*len(sizes) > 1_000_000:
        raise ConfigError('Configuration exceeds the 1000000-case materialized-grid limit; split into explicit runs')
    limits_raw = _object(raw.get('limits', {}), 'limits', set(LIMIT_DEFAULTS))
    limits = {k: _number(limits_raw.get(k, default), f'limits.{k}') for k, default in LIMIT_DEFAULTS.items()}
    if limits['solver_seconds'] >= limits['job_timeout_seconds']:
        raise ConfigError('limits.solver_seconds must be strictly smaller than job_timeout_seconds')
    planning_raw = _object(raw.get('planning', {}), 'planning', set(PLANNING_DEFAULTS))
    planning = {k: _number(planning_raw.get(k, default), f'planning.{k}') for k, default in PLANNING_DEFAULTS.items()}
    if planning['validation_step'] > planning['lazy_edge_step']:
        raise ConfigError('planning.validation_step must not exceed lazy_edge_step')
    options_raw = _object(raw.get('method_options', {}), 'method_options', OPTION_KEYS)
    options = {}
    for key, value in options_raw.items():
        if key == 'nto_collision_bound':
            options[key] = _number(value, key, inclusive=True)
        elif key == 'nto_influence_distance':
            options[key] = _number(value, key)
        elif key == 'nto_major_iterations':
            options[key] = _int(value, key, 1, 1_000_000)
        elif key == 'bezier_degree':
            options[key] = _int(value, key, 3, 12)
        elif key == 'scsto_relative_tolerance':
            options[key] = _number(value, key)
            if options[key] >= 1:
                raise ConfigError('scsto_relative_tolerance must be smaller than 1')
        elif key == 'maximum_recovery_iterations':
            options[key] = _int(value, key, 0, 1000)
    scene_options = _scene_options(raw.get('scene_options', {}), environment, profile)
    canonical = dict(version=version, name=name, environment=environment,
                environment_seeds=environments, roadmap_seeds=roadmaps, roadmap_sizes=sizes,
                methods=list(methods), scene_profile=profile, limits=limits,
                planning=planning, method_options=options)
    # Preserve previous identities when the optional extension is absent/empty.
    if scene_options:
        canonical['scene_options'] = scene_options
    use_strong = any(m.endswith('_strong') for m in methods)
    if 'strong_nto' in raw and not use_strong:
        raise ConfigError('strong_nto options require a strengthened NTO method')
    if use_strong:
        strong_raw = _object(raw.get('strong_nto', {}), 'strong_nto', set(STRONG_NTO_DEFAULTS))
        strong = {}
        for key, default in STRONG_NTO_DEFAULTS.items():
            value = strong_raw.get(key, default)
            label = f'strong_nto.{key}'
            if key == 'clearance':
                strong[key] = _number(value, label, inclusive=True)
            elif key in ('minimum_control_points', 'maximum_control_points'):
                strong[key] = _int(value, label, 4, 512)
            elif key == 'maximum_refinements':
                strong[key] = _int(value, label, 0, 100)
            else:
                strong[key] = _int(value, label, 1, 10000)
        for lower, upper in [('minimum_control_points', 'maximum_control_points'),
                             ('initial_collocation_points', 'maximum_collocation_points')]:
            if strong[lower] > strong[upper]:
                raise ConfigError(f'strong_nto.{lower} must not exceed {upper}')
        canonical['strong_nto'] = strong
    return canonical


def load_config(path: str | Path) -> dict:
    """Read strict JSON, including detection of duplicate keys at every depth."""
    return validate_config(_read_json(path))


def config_hash(value: Any) -> str:
    """Deterministic SHA-256; callers use canonical configs/resolved documents."""
    _finite_tree(value)
    raw = json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)
    return hashlib.sha256(raw.encode('utf-8')).hexdigest()


def case_grid(config: dict) -> list[dict]:
    cfg = validate_config(config)
    return [dict(id=f'e{e:03d}_r{r:03d}_n{n:05d}', environment_seed=e,
                 roadmap_seed=r, roadmap_size=n)
            for e, r, n in itertools.product(cfg['environment_seeds'], cfg['roadmap_seeds'], cfg['roadmap_sizes'])]


def resolve_config(config: dict, repo_root: str | Path = ROOT) -> dict:
    """Resolve the declared public profile without modifying any source assets.

    The resolved hash covers the config, effective protocol, scene definition,
    and every source file read below. It is not the paper's original asset hash.
    """
    cfg = validate_config(config)
    root = Path(repo_root).resolve()
    source_hashes = {}
    def source(name):
        path = (root / name).resolve()
        if not path.is_relative_to(root):
            raise ConfigError(f'Resolved source escapes repository: {name}')
        data = path.read_bytes()
        source_hashes[name] = hashlib.sha256(data).hexdigest()
        try:
            return json.loads(data.decode('utf-8'), object_pairs_hook=_json_pairs,
                              parse_constant=_invalid_constant)
        except (json.JSONDecodeError, UnicodeDecodeError) as exc:
            raise ConfigError(f'Invalid source JSON in {path}: {exc}') from exc
    protocol = deepcopy(source(BASE_PROTOCOL))
    scene_profile = source(SCENE_PROFILE)
    protocol['protocol'] = f'research_platform_v{SCHEMA_VERSION}:{cfg["name"]}'
    protocol['fidelity'] = 'constructed_public_paper_conditions_not_original_author_benchmark'
    protocol['environment_seeds'] = cfg['environment_seeds'].copy()
    protocol['roadmap_seeds'] = cfg['roadmap_seeds'].copy()
    protocol['roadmap_sizes'][cfg['environment']] = cfg['roadmap_sizes'].copy()
    protocol['method_order'] = [m for m in cfg['methods'] if m != 'DRM']
    protocol['optimizer_time_limit_seconds'] = cfg['limits']['solver_seconds']
    protocol['case_timeout_seconds'] = cfg['limits']['job_timeout_seconds']
    protocol.update(cfg['planning'])
    protocol.update(cfg['method_options'])
    if 'strong_nto' in cfg:
        protocol['strong_nto'] = deepcopy(cfg['strong_nto'])
    # Keep historical assumptions for unchanged parameters and add reconstruction
    # evidence separately; overwriting the whole assumptions object loses provenance.
    protocol['research_platform'] = dict(config_version=SCHEMA_VERSION,
        environment=cfg['environment'], scene_profile=cfg['scene_profile'],
        configured_methods=cfg['methods'].copy(),
        method_options=deepcopy(cfg['method_options']),
        timing_scope='per isolated job; solver cap strictly below external wall watchdog',
        scene_evidence=deepcopy(scene_profile['evidence']))
    if cfg['environment'] == 'forest':
        defaults = source(FOREST_PROTOCOL)
        reference = source(FOREST_REFERENCE)
        keys = ['domain_lower', 'domain_upper', 'obstacles', 'obstacle_radius',
                'obstacle_center_lower', 'obstacle_center_upper', 'point_radius']
        geometry = {k: deepcopy(reference.get(k, defaults[k])) for k in keys}
        geometry.update(start=deepcopy(reference['start']), goal=deepcopy(reference['goal']))
        geometry.update(deepcopy(cfg.get('scene_options', {})))
        _effective_scene_bounds(geometry, 'forest')
        protocol.update(deepcopy(geometry))
        scene = dict(kind='forest', profile=cfg['scene_profile'], **geometry)
        if cfg['scene_profile'] == 'figure6':
            scene.update(scene_id=reference['scene_id'], centers=deepcopy(reference['centers']),
                         obstacle_centers=deepcopy(reference['centers']),
                         evidence_class=reference['evidence_class'],
                         source_member=reference['source_member'], source_url=reference['source_url'],
                         endpoint_semantics=reference['endpoint_semantics'])
        else:
            scene.update(evidence_class='constructed_uniform_forest_with_figure6_endpoints',
                endpoint_semantics='Reuse displayed Figure 6 endpoints; not original all-case endpoints',
                sampling=dict(algorithm='numpy.default_rng(environment_seed).uniform',
                    center_lower=deepcopy(geometry['obstacle_center_lower']),
                    center_upper=deepcopy(geometry['obstacle_center_upper']), count=geometry['obstacles']))
    else:
        source(FR3_ASSET)
        protocol['fr3_scene'].update(deepcopy(scene_profile['fr3_scene_overrides']))
        protocol['fr3_scene'].update(deepcopy(cfg.get('scene_options', {})))
        _effective_scene_bounds(protocol['fr3_scene'], 'fr3')
        scene = dict(kind='fr3', profile='public_table',
            fr3_scene=deepcopy(protocol['fr3_scene']),
            coordinate_convention=scene_profile['coordinate_convention'],
            evidence=deepcopy(scene_profile['evidence']),
            table_visual=deepcopy(scene_profile['table_visual']))
    if cfg.get('scene_options'):
        explicit = dict(values=deepcopy(cfg['scene_options']),
                        source='explicit user research configuration; not recovered paper parameters')
        scene['explicit_scene_overrides'] = deepcopy(explicit)
        protocol['research_platform']['explicit_scene_overrides'] = explicit
    resolved = dict(schema_version=SCHEMA_VERSION, config=cfg, protocol=protocol,
                    scene_definition=scene, source_hashes=source_hashes,
                    config_sha256=config_hash(cfg))
    resolved['resolved_sha256'] = config_hash(resolved)
    return resolved
