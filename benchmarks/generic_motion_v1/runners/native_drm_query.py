"""Isolated CPU repair of native DRM exhausted-search error handling.

The build copies the upstream translation unit and applies two exact changes:
empty nearest-node search returns -1, and an exhausted A* frontier returns false.
All other C++ host code and all installed CUDA code are retained. No Python graph
search or endpoint/model/iteration-budget substitution is used. Build explicitly;
loading never compiles or modifies the existing environment.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import sysconfig
import time
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
BASE = ROOT / 'benchmarks/generic_motion_v1'
SOURCE = ROOT / 'csdecomp/src/planning/drm_planner.cpp'
BINDING = BASE / 'native/drm_safe_query.cpp'
PATCHES = (
    ('  assert(closest_node >= 0);\n',
     '  if (closest_node < 0) return -1;  // Preserve exhausted-search failure.\n'),
    ('  std::abort();  // Raise SIGABRT\n',
     '  return false;  // An exhausted A* frontier is a search failure.\n'),
)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write(path, value):
    Path(path).write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False)+'\n')


def patched_source(text):
    for old, new in PATCHES:
        if text.count(old) != 1:
            raise ValueError('Upstream DRM no longer matches the exact reviewed error-handling patch')
        text = text.replace(old, new, 1)
    return text


def native_library():
    import csdecomp.pycsdecomp_bindings as native
    return Path(native.__file__).resolve()


def build(out, profile='failure_exits'):
    """Compile only CPU source; existing CUDA functions are dynamic dependencies."""
    if profile not in ('failure_exits', 'endpoint_edges'):
        raise ValueError('Unknown native host repair profile')
    out = Path(out).resolve()
    if out.exists():
        raise FileExistsError(f'Preserve existing sidecar build: {out}')
    out.mkdir(parents=True)
    native = native_library()
    before = sha(native)
    external = ROOT / 'reproduction/vendor/paperdate_bazel_output/external'
    pybind = external / 'pybind11_bazel~~internal_configure_extension~pybind11/include'
    includes = [Path(sysconfig.get_path('include')), pybind, external/'eigen',
                external/'cereal/include', external/'fmt/include', external/'yaml-cpp/include',
                ROOT/'csdecomp/src', Path('/usr/local/cuda-12.8/include')]
    if any(not path.is_dir() for path in includes):
        raise FileNotFoundError('Existing local C++ headers are unavailable; no package install attempted')
    # Use the exact binding ABI key exported by the installed wheel. libstdc++
    # retains this C++ ABI; explicit runtime layout checks guard every query.
    strings = subprocess.check_output(['strings', str(native)], text=True)
    internals = re.findall(r'^__pybind11_internals_v4_gcc_libstdcpp(_cxxabi[0-9]+)__$', strings, re.M)
    if len(set(internals)) != 1:
        raise ValueError('Installed pybind11 ABI is not the reviewed v4/gcc/libstdc++ ABI')
    patched = out/'drm_planner.cpp'
    source_text = patched_source(SOURCE.read_text())
    if profile == 'endpoint_edges':
        from .native_drm_corrections import correct_source
        source_text = correct_source(source_text)
    patched.write_text(source_text)
    binding = BINDING
    if profile == 'endpoint_edges':
        # Read-only native diagnostic entry points exercise the exact compiled
        # implementation, including zero-length steering and isolated vertices.
        binding = out/'drm_safe_query.cpp'
        prefix, suffix = BINDING.read_text().rsplit('\n}', 1)
        binding.write_text(prefix + '''
  m.def("path_exists", [](DrmPlanner& p, int start, int goal) {
    return p.CheckPathExists(start, goal, {}, nullptr);
  });
  m.def("steer", [](DrmPlanner& p, const Eigen::VectorXf& start,
                    const Eigen::VectorXf& goal, const csdecomp::Voxels& voxels,
                    float radius) {
    return p.SteerFunctionIsSafe(start, goal, voxels, radius);
  });
}''' + suffix)
    binary = out / ('_drm_safe_query' + sysconfig.get_config_var('EXT_SUFFIX'))
    argv = ['/usr/bin/g++', '-std=c++17', '-O2', '-shared', '-fPIC', '-fvisibility=hidden',
            '-Wl,-Bsymbolic', '-D_GLIBCXX_USE_CXX11_ABI=1',
            '-fabi-version='+internals[0].removeprefix('_cxxabi10'),
            '-DPYBIND11_BUILD_ABI="'+internals[0]+'"', '-DFMT_HEADER_ONLY',
            *['-I'+str(path) for path in includes], str(binding), str(patched), str(native),
            '-o', str(binary)]
    abi_macros = subprocess.check_output(['/usr/bin/g++', argv[argv.index('-fabi-version='+internals[0].removeprefix('_cxxabi10'))],
        '-dM', '-E', '-x', 'c++', '-'], input='', text=True)
    if '#define __GXX_ABI_VERSION '+internals[0].removeprefix('_cxxabi') not in abi_macros:
        raise RuntimeError('Compiler ABI does not match the native binding ABI')
    (out/'compiler_macros.txt').write_text(abi_macros)
    source_files = {str(path.relative_to(ROOT)): sha(path) for path in sorted((ROOT/'csdecomp/src').rglob('*.h'))}
    source_files.update({str(SOURCE.relative_to(ROOT)): sha(SOURCE), str(BINDING.relative_to(ROOT)): sha(BINDING)})
    receipt = dict(status='building', profile=profile, source_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        scope=('Native CPU DRM failure exits, endpoint-local rejected edges, zero-edge reachability and endpoint-inclusive steering; unchanged CUDA'
               if profile == 'endpoint_edges' else 'Native CPU DRM error handling only; existing wheel CUDA functions are linked unchanged'),
        native_library=str(native), native_library_sha256=before,
        upstream_source_sha256=sha(SOURCE), patched_source_sha256=sha(patched), binding_sha256=sha(BINDING),
        source_files=source_files, python=sys.version, pybind_abi=internals[0], compiler_argv=argv,
        include_paths=list(map(str, includes)), compiler_abi_macro=internals[0].removeprefix('_cxxabi'),
        compiler_macros_sha256=sha(out/'compiler_macros.txt'), compiler=subprocess.check_output(['/usr/bin/g++','--version'],text=True),
        patches=[dict(before=old, after=new) for old,new in PATCHES])
    receipt['compiled_binding'] = str(binding)
    receipt['compiled_binding_sha256'] = sha(binding)
    if profile == 'endpoint_edges':
        from .native_drm_corrections import CORRECTIONS
        receipt['patches'].extend(dict(before=old,after=new) for old,new in CORRECTIONS)
    write(out/'build.json', receipt)
    started = time.perf_counter()
    with (out/'build.stdout.log').open('w') as stdout, (out/'build.stderr.log').open('w') as stderr:
        result = subprocess.run(argv, cwd=ROOT, stdout=stdout, stderr=stderr)
    receipt.update(returncode=result.returncode, build_seconds=time.perf_counter()-started,
                   original_installation_unchanged=sha(native)==before,
                   status='ready' if result.returncode==0 and sha(native)==before else 'failed')
    if binary.is_file():
        receipt.update(binary=str(binary), binary_sha256=sha(binary))
    write(out/'build.json', receipt)
    if receipt['status'] != 'ready':
        raise RuntimeError(f'Isolated CPU build failed; retained logs in {out}')
    return receipt


def load(build_dir):
    build_dir = Path(build_dir).resolve()
    receipt = json.loads((build_dir/'build.json').read_text())
    if receipt.get('status') != 'ready':
        raise ValueError('Safe DRM sidecar build is not ready')
    for path, digest in [(native_library(), receipt['native_library_sha256']),
                         (Path(receipt['binary']), receipt['binary_sha256']),
                         (SOURCE, receipt['upstream_source_sha256']), (BINDING, receipt['binding_sha256'])]:
        if sha(path) != digest:
            raise ValueError(f'Safe DRM source or dependency changed: {path}')
    for relative, expected in receipt['source_files'].items():
        if sha(ROOT/relative) != expected:
            raise ValueError(f'Safe DRM native header/source changed: {relative}')
    if receipt.get('compiled_binding') and sha(receipt['compiled_binding']) != receipt['compiled_binding_sha256']:
        raise ValueError('Safe DRM compiled diagnostic binding changed')
    key = '_drm_safe_query'
    if key in sys.modules:
        module = sys.modules[key]
        if Path(module.__file__).resolve() != Path(receipt['binary']).resolve():
            raise ValueError('A different safe DRM sidecar is already loaded')
    else:
        spec = importlib.util.spec_from_file_location(key, receipt['binary'])
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        sys.modules[key] = module
    return module, receipt


def layout_guard(module, planner):
    actual = module.layout(planner)
    expected = dict(num_joints=planner.num_joints, nodes=len(planner.drm.id_to_node_map),
                    collision_nodes=len(planner.collision_set), loaded=planner.drm_loaded,
                    initialized=planner.is_initialized)
    for key in ('max_number_planning_attempts','max_iterations_steering_to_node',
                'max_nodes_to_expand','online_edge_step_size'):
        expected[key] = getattr(planner.options,key)
    if actual != expected:
        raise RuntimeError('Native sidecar ABI/layout disagrees with installed planner; refusing query')
    return actual


def query(planner, q_start, q_goal, voxels_xyz, radius, *, build_dir, out=None):
    """Return (success, native_path, evidence), preserving native no-path errors.

    Existing callers should classify a raised native RuntimeError as a frontend
    failure, just as before. The unchanged strict seed validator must still run
    after a successful query; this does not fix native finite-sampling gaps.
    """
    module, receipt = load(build_dir)
    before = layout_guard(module,planner)
    start, goal = np.asarray(q_start), np.asarray(q_goal)
    voxels = np.asarray(voxels_xyz,dtype=np.float32)
    if voxels.size == 0: voxels = voxels.reshape(0,3)
    if (start.shape != (planner.num_joints,) or goal.shape != start.shape or
            not np.isfinite(np.r_[start,goal]).all() or voxels.ndim != 2 or voxels.shape[1] != 3 or
            not np.isfinite(voxels).all() or not np.isfinite(radius) or radius < 0):
        raise ValueError('Malformed fixed DRM query input')
    if not before['loaded'] or not before['initialized']:
        raise ValueError('The native planner must be initialized and loaded before querying')
    options=planner.options
    if (options.max_number_planning_attempts <= 1 or options.max_num_blocked_edges_before_discard_node <= 1
            or options.max_iterations_steering_to_node < 1 or options.max_nodes_to_expand < 1
            or not np.isfinite(options.online_edge_step_size) or options.online_edge_step_size <= 0
            or not np.isfinite(options.voxel_padding) or options.voxel_padding < 0):
        raise ValueError('Native DRM options would violate its preconditions')
    evidence = dict(implementation=receipt.get('scope','Isolated original C++ DRM with two non-aborting failure returns'),
        profile=receipt.get('profile','failure_exits'),
        binary_sha256=receipt['binary_sha256'], native_library_sha256=receipt['native_library_sha256'],
        patched_source_sha256=receipt['patched_source_sha256'], upstream_source_sha256=receipt['upstream_source_sha256'],
        build_directory=str(Path(build_dir).resolve()), layout_before=before, endpoint_change=False,
        q_start=start.tolist(), q_goal=goal.tolist(), online_voxel_radius_m=float(radius),
        cuda_changed=False, search_budget_changed=False, status='running')
    output=None if out is None else Path(out)
    if output is not None:
        output.mkdir(parents=True,exist_ok=True)
        if (output/'query.json').exists():raise FileExistsError('Preserve previous safe DRM query evidence')
        write(output/'build_receipt.json',receipt)
        write(output/'query.json',evidence)
    began=time.perf_counter()
    try:
        success,path=module.plan(planner,start,goal,np.asfortranarray(voxels.T),float(radius))
    except Exception as error:
        evidence.update(status='native_exception',query_success=False,exception_type=type(error).__name__,
            exception=repr(error),elapsed_seconds=time.perf_counter()-began,
            layout_after=layout_guard(module,planner))
        if output is not None:write(output/'query.json',evidence)
        raise
    evidence.update(status='returned',query_success=bool(success),elapsed_seconds=time.perf_counter()-began,
                    layout_after=layout_guard(module,planner))
    if output is not None:write(output/'query.json',evidence)
    return bool(success),np.asarray(path,float),evidence


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command',choices=['build'])
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--profile',choices=['failure_exits','endpoint_edges'],default='failure_exits')
    args=parser.parse_args()
    print(json.dumps(build(args.output,profile=args.profile),indent=2))


if __name__=='__main__':
    main()
