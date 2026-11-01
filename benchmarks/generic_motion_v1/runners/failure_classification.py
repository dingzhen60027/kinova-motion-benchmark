"""Closed, metadata-only exception rules shared by execution and reporting.

An optimizer's unsuccessful return is an ordinary outcome. An unrecognized
exception is infrastructure failure, including when a deadline follows it.
This module does not import a planner, collision library or solver.
"""
import ast
import hashlib
import json
import math
from pathlib import Path
import re


KNOWN_DRM_NO_ROUTE = '[DRMPlanner] No route found through tested roadmap connections.'
HANDOFF_SOURCE = 'benchmarks/generic_motion_v1/runners/native_handoff.py'
HANDOFF_KIND = 'native_handoff_numerical_rejection'
HANDOFF_BOUNDED_MESSAGE = 'Native anchor containment cannot satisfy the fixed normalized correction bound'
KNOWN_TIMEOUT_MESSAGES = frozenset({
    'NTO construction exhausted online budget', 'Paper NTO construction exhausted online budget',
    'Native collision solve exhausted its shared deadline', 'Native assertion recovery exhausted its shared deadline',
    'No online time remains after reserving common validation',
    'NTO refinement geometry check exhausted shared online budget',
    'NTO refinement exhausted shared online budget',
    'NTO refinement exhausted shared online budget after geometry check',
    'TrajOpt derivative evaluation exhausted shared budget', 'TrajOpt shared budget exhausted before solve',
    'TrajOpt exhausted shared online deadline', 'BPMPD transport deadline exceeded',
    'LGCS construction exhausted the remaining solver budget', 'LGCS construction exhausted online budget',
    'Online budget exhausted before alternate DRM search', 'Online budget exhausted before alternate native Plan',
    'CHOMP/STOMP exhausted shared planning budget', 'Canonical geometry derivative exhausted shared deadline',
})
KNOWN_STAGES = frozenset({'input_contract', 'model_setup', 'dependency_probe', 'gpu_initialization_warmup',
    'sampling_planning', 'validation', 'roadmap_contract', 'shared_initialization_load', 'roadmap_load',
    'online_collision_map', 'drm_path', 'drm_path_validation', 'trajectory_optimization', 'eizo_inflation',
    'collision_recovery'})
# Exact existing validator guards for numerically unusable solver candidates.
# Configuration/model mismatch and collision-checker API faults are not here.
KNOWN_VALIDATION_REJECTIONS = frozenset({
    'Numerical sampling cap exceeded or nonfinite derivative bound',
    'Numerical sampling cap exceeded',
    'A PWL path must have at least two finite 7D points',
    'Malformed Bezier controls or durations', 'Malformed B-spline',
    'Nonpositive B-spline duration',
    'Candidate produced nonfinite or malformed numerical samples',
    'Time objective requires a finite positive physical duration',
    'Nonfinite derivative bounds cannot establish feasibility',
    'Nonfinite region residual cannot establish feasibility',
})
INFRASTRUCTURE_STATUSES = frozenset({'infrastructure_error', 'unsupported', 'model_consistency_failed'})


def exception_parts(value):
    """Parse a one-message exception repr without executing it."""
    try:
        node = ast.parse(value, mode='eval').body
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
                and len(node.args) == 1 and not node.keywords):
            message = ast.literal_eval(node.args[0])
            if isinstance(message, str):
                return node.func.id, message
    except (ValueError, SyntaxError, TypeError):
        pass
    return None, None


def known_exception_category(result, *, handoff_evidence=None, legacy_drm_evidence=None):
    """Recognize only explicitly supported exception outcomes from small JSON."""
    kind, message = exception_parts(result.get('exception'))
    phase, raw = result.get('failed_stage'), result.get('status')
    if kind == 'RuntimeError' and message == KNOWN_DRM_NO_ROUTE:
        if phase == 'drm_path' and raw in ('frontend_failed', 'timeout'):
            return 'native_no_route'
    elif kind == 'RuntimeError' and message == LEGACY_DRM_NO_ROUTE and legacy_drm_evidence is not None:
        if phase == 'drm_path' and raw == 'frontend_failed':
            return 'reviewed_legacy_native_no_route'
    elif kind == 'TimeoutError':
        if raw == 'timeout' and (message in KNOWN_TIMEOUT_MESSAGES or
                any(message == 'Online deadline before '+stage for stage in KNOWN_STAGES)):
            return 'query_deadline'
    elif kind == 'OmplNoExactSolution':
        status = result.get('native_planner_status')
        if (phase == 'sampling_planning' and status in ('Approximate solution', 'Timeout')
                and message == 'OMPL did not return an exact solution: '+status
                and raw in ('planner_timeout', 'planner_failed', 'timeout')):
            return 'ompl_no_exact_solution'
    elif kind == 'ValueError' and message in KNOWN_VALIDATION_REJECTIONS:
        if phase in ('validation', 'drm_path_validation') and raw in ('validation_undetermined', 'timeout'):
            return 'numerical_candidate_rejected'
    elif kind == 'NativeHandoffError' and handoff_evidence is not None:
        if phase == 'collision_recovery' and raw in ('solver_failed', 'timeout'):
            return HANDOFF_KIND
    return None


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def handoff_failure_evidence(attempt, result, *, expected_tolerance, source_hashes, receipt=None):
    """Bind a typed handoff rejection to its unchanged bound and saved evidence.

    A sealed reader supplies ``receipt``. Execution before sealing uses the same
    field/content checks without claiming receipt verification. Old ValueError
    records receive no exemption here and are never rewritten.
    """
    marker = result.get('failure_classification')
    kind, message = exception_parts(result.get('exception'))
    if (kind != 'NativeHandoffError' or not isinstance(marker, dict)
            or marker.get('kind') != HANDOFF_KIND or marker.get('source') != HANDOFF_SOURCE
            or not isinstance(source_hashes, dict)
            or marker.get('source_sha256') != source_hashes.get(HANDOFF_SOURCE)
            or not isinstance(marker.get('source_sha256'), str)
            or not re.fullmatch('[0-9a-f]{64}', marker['source_sha256'])
            or result.get('failed_stage') != 'collision_recovery'
            or result.get('status') not in ('solver_failed', 'timeout')
            or not _finite(expected_tolerance) or expected_tolerance < 0):
        return None
    name = marker.get('evidence')
    if not isinstance(name, str) or not re.fullmatch(
            r'(?:coverage/recovery_[0-9]{3}|recovery/[0-9]{3})/native_handoff/[0-9]{4}/handoff.json', name):
        return None
    attempt = Path(attempt).resolve()
    path = (attempt/name).resolve()
    if not path.is_relative_to(attempt):
        return None
    try:
        content = path.read_bytes()
        if receipt is not None and receipt.get('files', {}).get(name) != hashlib.sha256(content).hexdigest():
            return None
        evidence = json.loads(content)
    except (OSError, ValueError, TypeError):
        return None
    if not isinstance(evidence, dict):
        return None
    code = marker.get('code')
    allowed = evidence.get('allowed_normalized_correction')
    corrections = [evidence.get(key) for key in ('maximum_normalized_correction',
                   'maximum_total_normalized_offset_change_from_input')]
    if (evidence.get('status') != 'rejected' or evidence.get('error') != result['exception']
            or evidence.get('error_code') != code or evidence.get('fixed_anchors_changed') is not False
            or evidence.get('native_predicate_tolerance') != 0.
            or not _finite(allowed) or allowed != expected_tolerance
            or not all(_finite(value) and value >= 0 for value in corrections)):
        return None
    if code == 'correction_exceeds_tolerance':
        maximum = max(corrections)
        if maximum <= allowed or message != f'Native handoff correction {maximum:.9g} exceeds fixed tolerance {allowed:.9g}':
            return None
    elif code == 'bounded_predicate_rejected':
        if message != HANDOFF_BOUNDED_MESSAGE or max(corrections) > allowed:
            return None
        searches = evidence.get('bounded_native_offset_search')
        if not isinstance(searches, list) or not any(
                isinstance(item, dict) and item.get('status') == 'no_passing_offset_within_bound'
                and isinstance(item.get('row'), int) and not isinstance(item['row'], bool) and item['row'] >= 0
                and _finite(item.get('initial_offset')) and _finite(item.get('maximum_permitted_offset'))
                and isinstance(item.get('native_predicate_calls'), int)
                and not isinstance(item['native_predicate_calls'], bool) and item['native_predicate_calls'] >= 1
                for item in searches):
            return None
    else:
        return None
    return dict(kind=HANDOFF_KIND, code=code, evidence=name, receipt_verified=receipt is not None)


def exception_needs_attention(result, *, handoff_evidence=None, legacy_drm_evidence=None):
    """A deadline never makes an unknown/CUDA/backend failure ordinary."""
    if result.get('status') in INFRASTRUCTURE_STATUSES:
        return True
    category = result.get('error_category')
    if category is not None and not (category == HANDOFF_KIND and handoff_evidence is not None
                         and known_exception_category(result, handoff_evidence=handoff_evidence) == HANDOFF_KIND):
        return True
    if 'cuda' in str(result.get('exception', '')).lower():
        return True
    return bool(result.get('exception')) and known_exception_category(
        result, handoff_evidence=handoff_evidence, legacy_drm_evidence=legacy_drm_evidence) is None


def unexpected_process_exit(process):
    code = process.get('exit_code')
    return code not in (None, 0) and not (process.get('timed_out') is True and code in (-15, -9))


# Reader-only compatibility for the reviewed historical, non-aborting native
# DRM build. New runtime must not accept this message without these artifacts.
LEGACY_DRM_NO_ROUTE = '[DRMPlanner] No collision-free path exists in roadmap.'
LEGACY_DRM_SOURCE = 'benchmarks/generic_motion_v1/runners/native_drm_query.py'
LEGACY_DRM_SOURCE_SHA256 = '0778b54c108793ae15fe3e54ef5bb8489ccc7d1c31bd95bbe237eac5d0334549'
LEGACY_DRM_BUILD = {
    'patched_source_sha256':'12ef03c722a7c7fbe67623bc2eff6b177b13c87c85deb1420e623ebdfb1a78c1',
    'binary_sha256':'a3ca2a23e7ca84bfc90542a7e761b3f3f10b33c92889ff1ef85a477c902f8c83',
    'native_library_sha256':'0a048625a9d745ac409cff55791d360fe9e91ad1b8d7a3b82245a1a024c57738',
    'upstream_source_sha256':'159300e72e24c2ed398c2c1453a94c10f1f34024f20b014c06e89825380658d6',
}


def legacy_drm_failure_evidence(attempt, result, *, source_hashes, receipt):
    """Recognize one exact old search outcome using sealed source/trace/build."""
    if (exception_parts(result.get('exception')) != ('RuntimeError', LEGACY_DRM_NO_ROUTE)
            or result.get('status') != 'frontend_failed' or result.get('failed_stage') != 'drm_path'
            or not isinstance(source_hashes, dict)
            or source_hashes.get(LEGACY_DRM_SOURCE) != LEGACY_DRM_SOURCE_SHA256
            or not isinstance(receipt, dict) or not isinstance(receipt.get('files'), dict)):
        return None
    attempt = Path(attempt).resolve()
    documents = {}
    try:
        for name in ('exception.txt', 'query.json', 'build_receipt.json'):
            path = (attempt/name).resolve()
            if not path.is_relative_to(attempt):
                return None
            content = path.read_bytes()
            if receipt['files'].get(name) != hashlib.sha256(content).hexdigest():
                return None
            documents[name] = content.decode() if name.endswith('.txt') else json.loads(content)
    except (OSError, ValueError, TypeError):
        return None
    trace = documents['exception.txt'].splitlines()
    query, build = documents['query.json'], documents['build_receipt.json']
    if (len(trace) < 3 or not re.fullmatch(r'  File "[^"\n]*/'+re.escape(LEGACY_DRM_SOURCE)+r'", line 194, in query', trace[-3])
            or trace[-2] != '    success,path=module.plan(planner,start,goal,np.asfortranarray(voxels.T),float(radius))'
            or trace[-1] != 'RuntimeError: '+LEGACY_DRM_NO_ROUTE
            or not isinstance(query, dict) or not isinstance(build, dict)
            or query.get('exception') != result['exception'] or query.get('exception_type') != 'RuntimeError'
            or query.get('status') != 'native_exception' or query.get('query_success') is not False
            or query.get('cuda_changed') is not False or query.get('endpoint_change') is not False
            or query.get('search_budget_changed') is not False or build.get('status') != 'ready'
            or any(query.get(key) != value or build.get(key) != value for key,value in LEGACY_DRM_BUILD.items())):
        return None
    return dict(kind='reviewed_legacy_native_no_route', source=LEGACY_DRM_SOURCE,
                source_sha256=LEGACY_DRM_SOURCE_SHA256, source_line=194, receipt_verified=True)
