"""Validate generator contracts before model calls or partial output writes.

The frozen v1 defaults are unchanged. Invalid inputs that formerly reached
NumPy or silently weakened endpoint checks now raise a descriptive ValueError.
"""
from __future__ import annotations

import numbers
import numpy as np


def _keys(value, expected, name):
    if not isinstance(value, dict) or set(value) != set(expected):
        raise ValueError(f"{name} must have exactly these fields: {sorted(expected)}")


def _number(value, name, minimum=0, *, strict=False):
    if isinstance(value, bool) or not isinstance(value, numbers.Real) or not np.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    if value < minimum or (strict and value == minimum):
        raise ValueError(f"{name} must be {'greater than' if strict else 'at least'} {minimum}")


def _integer(value, name, minimum=1):
    if isinstance(value, bool) or not isinstance(value, numbers.Integral) or value < minimum:
        raise ValueError(f"{name} must be an integer at least {minimum}")


def _range(value, name, *, positive=True):
    if not isinstance(value, list) or len(value) != 2:
        raise ValueError(f"{name} must be a two-value interval")
    for x in value:
        _number(x, name, strict=positive)
    if value[0] > value[1]:
        raise ValueError(f"{name} interval must be ordered")


def validate_endpoint_config(config, model=None):
    _number(config['endpoint_joint_margin_rad'], 'endpoint_joint_margin_rad')
    _number(config['endpoint_clearance_m'], 'endpoint_clearance_m')
    _number(config['straight_line_step_rad'], 'straight_line_step_rad', strict=True)
    for field in ('endpoint_batch_size', 'endpoint_max_attempts'):
        _integer(config[field], field)
    _keys(config['normalized_distance_bins'], ('short', 'medium', 'long'), 'normalized_distance_bins')
    prior_high = -1
    for key in ('short', 'medium', 'long'):
        interval = config['normalized_distance_bins'][key]
        _range(interval, f'normalized_distance_bins.{key}', positive=False)
        if interval[0] >= interval[1] or interval[0] < prior_high or interval[1] > 1:
            raise ValueError('Normalized distance bins must be nonempty, ordered, disjoint and within [0, 1]')
        prior_high = interval[1]
    cycle = config['distance_bin_cycle']
    if not isinstance(cycle, list) or not cycle or any(k not in config['normalized_distance_bins'] for k in cycle):
        raise ValueError('distance_bin_cycle must contain recognized distance bin names')
    if model is not None:
        lower, upper = np.asarray(model.lower), np.asarray(model.upper)
        if (lower.shape != (7,) or upper.shape != (7,) or not np.isfinite(lower).all() or
                not np.isfinite(upper).all() or np.any(upper - lower <= 2 * config['endpoint_joint_margin_rad'])):
            raise ValueError('Seven finite joint intervals must remain nonempty after applying the joint margin')


def validate_config(config, model=None):
    expected = {'benchmark_version', 'rng', 'world_frame', 'voxel_radius_m', 'maximum_voxel_cell_side_m',
        'roi_min_m', 'roi_max_m', 'endpoint_joint_margin_rad', 'endpoint_clearance_m', 'endpoint_max_attempts',
        'endpoint_batch_size', 'straight_line_step_rad', 'normalized_distance_definition', 'normalized_distance_bins',
        'distance_bin_cycle', 'maximum_scene_proposals', 'sparse_clutter', 'dense_clutter', 'bottleneck',
        'route_choice', 'splits'}
    _keys(config, expected, 'generator configuration')
    literals = {'benchmark_version': 'kinova7_generic_v1',
        'rng': 'numpy.random.PCG64; SHA256 domain-separated uint64 seeds',
        'world_frame': 'world; right-handed; metres; base_link identity',
        'normalized_distance_definition': 'sqrt(mean(((q_goal-q_start)/(upper-lower))**2))'}
    for key, expected in literals.items():
        if config[key] != expected:
            raise ValueError(f'{key} does not describe the supported v1 implementation')
    if config['voxel_radius_m'] != .04:
        raise ValueError('v1 voxel_radius_m is fixed at 0.04; another radius requires a new protocol')
    _number(config['maximum_voxel_cell_side_m'], 'maximum_voxel_cell_side_m', strict=True)
    if config['maximum_voxel_cell_side_m'] > .04:
        raise ValueError('maximum_voxel_cell_side_m cannot exceed 0.04')
    low, high = np.asarray(config['roi_min_m']), np.asarray(config['roi_max_m'])
    if low.shape != (3,) or high.shape != (3,) or not np.isfinite(low).all() or not np.isfinite(high).all() or np.any(low >= high):
        raise ValueError('ROI must contain three finite, nonempty coordinate intervals')
    _integer(config['maximum_scene_proposals'], 'maximum_scene_proposals')
    validate_endpoint_config(config, model)
    for family in ('sparse_clutter', 'dense_clutter'):
        fields = {'obstacles_min', 'obstacles_max', 'sphere_radius_range_m'}
        if family == 'sparse_clutter':
            fields.add('box_size_range_m')
        item = config[family]
        _keys(item, fields, family)
        for key in ('obstacles_min', 'obstacles_max'):
            _integer(item[key], f'{family}.{key}')
        if item['obstacles_min'] > item['obstacles_max']:
            raise ValueError(f'{family} obstacle count interval is reversed')
        _range(item['sphere_radius_range_m'], f'{family}.sphere_radius_range_m')
        if family == 'sparse_clutter':
            _range(item['box_size_range_m'], f'{family}.box_size_range_m')
    item = config['bottleneck']
    _keys(item, ('gap_range_m', 'wall_length_m', 'wall_thickness_m', 'wall_height_m'), 'bottleneck')
    _range(item['gap_range_m'], 'bottleneck.gap_range_m')
    for key in ('wall_length_m', 'wall_thickness_m', 'wall_height_m'):
        _number(item[key], f'bottleneck.{key}', strict=True)
    item = config['route_choice']
    _keys(item, ('cluster_separation_range_m', 'spheres_per_cluster', 'cluster_spread_m', 'sphere_radius_range_m'), 'route_choice')
    _range(item['cluster_separation_range_m'], 'route_choice.cluster_separation_range_m')
    _range(item['sphere_radius_range_m'], 'route_choice.sphere_radius_range_m')
    _integer(item['spheres_per_cluster'], 'route_choice.spheres_per_cluster')
    _number(item['cluster_spread_m'], 'route_choice.cluster_spread_m')
    _keys(config['splits'], ('smoke', 'dev', 'test'), 'splits')
    for split, item in config['splits'].items():
        _keys(item, ('main_layouts_per_family', 'pairs_per_layout', 'open_layouts'), f'splits.{split}')
        for key in ('main_layouts_per_family', 'pairs_per_layout', 'open_layouts'):
            _integer(item[key], f'splits.{split}.{key}', minimum=0 if key == 'open_layouts' else 1)
    return config
