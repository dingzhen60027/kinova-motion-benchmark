"""Explicit SPRAL policy for the pinned Drake IPOPT backend.

Use separate METIS ordering and MC64 scaling to avoid SPRAL's matching-order
path. IPOPT 3.14.19 otherwise overwrites spral_order according to its scaling
choice. This is a solver configuration workaround, not a native-library patch.
"""
from functools import lru_cache
import hashlib
from importlib.metadata import version
from pathlib import Path

DEFAULTS = dict(linear_solver='spral', spral_order='metis', spral_scaling='mc64')
SOURCES = dict(
    ipopt_interface='https://github.com/coin-or/Ipopt/blob/releases/3.14.19/src/Algorithm/LinearSolvers/IpSpralSolverInterface.cpp',
    singular_factorization_fix='https://github.com/coin-or/Ipopt/pull/848',
    drake_pin='https://github.com/RobotLocomotion/drake/blob/v1.51.1/tools/workspace/ipopt_internal/repository.bzl')


def settings(config=None):
    supplied = (config or {}).get('ipopt', {})
    if not isinstance(supplied, dict) or set(supplied) - DEFAULTS.keys():
        raise ValueError('ipopt accepts only linear_solver, spral_order and spral_scaling')
    value = dict(DEFAULTS, **supplied)
    if (value['linear_solver'] != 'spral'
            or value['spral_order'] not in ('metis', 'matching')
            or value['spral_scaling'] not in ('none', 'mc64', 'matching')):
        raise ValueError('Unsupported IPOPT/SPRAL linear-system configuration')
    if ((value['spral_order'] == 'matching') != (value['spral_scaling'] == 'matching')):
        raise ValueError('SPRAL matching ordering and scaling must be selected together; scaling can override ordering')
    return value


def evidence(config=None):
    options = settings(config)
    import pydrake
    library = Path(pydrake.__file__).resolve().parent / 'lib/libdrake.so'
    stat = library.stat()
    mapped = _loaded_library_paths()
    if mapped != [str(library.resolve())]:
        raise RuntimeError('Loaded libdrake differs from the selected package: '+repr(mapped))
    return dict(options=options, avoids_matching_order=options['spral_order'] == 'metis',
                policy='explicit_SPRAL_order_and_scaling', native_library_patched=False,
                constraints_changed=False, sources=SOURCES,
                native_binary=dict(drake_version=version('drake'), library=str(library),
                                   loaded_library_paths=mapped, loaded_matches_package=True,
                                   bytes=stat.st_size,
                                   sha256=_binary_sha(str(library), stat.st_dev, stat.st_ino,
                                                      stat.st_size, stat.st_mtime_ns, stat.st_ctime_ns)))


def _loaded_library_paths():
    # Import the native bindings before inspecting this Ubuntu process.
    import pydrake.solvers
    paths = set()
    for line in Path('/proc/self/maps').read_text().splitlines():
        fields = line.split(None, 5)
        if len(fields) != 6:
            continue
        path = fields[5]
        if Path(path.removesuffix(' (deleted)')).name == 'libdrake.so':
            paths.add(str(Path(path).resolve()))
    return sorted(paths)


@lru_cache(maxsize=4)
def _binary_sha(path, device, inode, size, mtime_ns, ctime_ns):
    # Fingerprint once per actual file identity within this fresh worker.
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()
