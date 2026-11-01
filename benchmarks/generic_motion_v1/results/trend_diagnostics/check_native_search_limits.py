"""Controlled whole-dev DRM-only check of upstream native search limits.

Run serially after the main GPU batch. This is diagnostic, not a new main
baseline score or a change to the frozen evaluation configuration.
"""
from pathlib import Path
import fcntl
import json
import sys

ROOT = next(p for p in Path(__file__).resolve().parents if (p/'scripts/evaluate.py').is_file())
sys.path.insert(0, str(ROOT))
from benchmarks.generic_motion_v1.runners import drm_diagnostics

cfg = drm_diagnostics.load_config(ROOT/'configs/evaluation.json')
baseline_options = dict(cfg['drm'])
# Restore only the three count limits to upstream drm_planner.h defaults.
# Geometry, online sampling, padding, roadmap and numerical validator stay fixed.
cfg['drm'].update(max_number_planning_attempts=100,
                  max_iterations_steering_to_node=50, max_nodes_to_expand=100000)
# Use the same project-wide GPU lock as the batch coordinator.
gpu_lock = (ROOT/'results/.gpu.lock').open('a+')
fcntl.flock(gpu_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
out = Path(__file__).with_name('frontend_native_limits')
out.mkdir(exist_ok=False)
(out/'comparison_scope.json').write_text(json.dumps(dict(
    baseline_count_limits=baseline_options, diagnostic_options=cfg['drm'],
    scope='DRM-only development sensitivity; no main config edit; no test queries',
    inherited_frontend_driver='benchmarks.generic_motion_v1.runners.drm_diagnostics.frontend',
    timing='Includes query and validation, excludes model construction; not formal end-to-end scores'), indent=2)+'\n')
# Explicit driver dependency injection preserves the same compiled implementation
# and audit code without editing runtime source while its main batch is frozen.
drm_diagnostics.load_config = lambda _: cfg
result = drm_diagnostics.frontend(ROOT/cfg['drm_query']['build_dir'], out, 0)
print(json.dumps(result, indent=2))
