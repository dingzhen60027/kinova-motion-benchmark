# Conda environment

The [project README](../README.md) is the entry point for the current Kinova benchmark. This directory contains the existing environment launcher and package records. Reuse the configured environment without reinstalling or upgrading it.

## Use the existing environment

From the repository root:

```bash
bash local/python.sh -c 'import sys, csdecomp; print(sys.executable); print(csdecomp.__file__)'
bash local/python.sh scripts/evaluate.py check
```

The default prefix is `.conda/envs/csdecomp`. To reuse an environment outside this checkout, set `CSDECOMP_CONDA_PREFIX` to its absolute path. From another working directory, invoke `local/python.sh` by its absolute path; it changes to the repository root internally.

The launcher removes inherited `PYTHONHOME`, `PYTHONPATH` and `LD_LIBRARY_PATH` in its child process and disables user site-packages. OpenBLAS and MKL use one thread; OMP defaults to four and honors an explicit override. It does not modify the parent shell or Conda base.

## Verified setup

| Component | Recorded version |
|---|---|
| Operating system | Ubuntu 22.04.5 LTS |
| GPU | NVIDIA Quadro RTX 8000, 48 GiB |
| NVIDIA driver / system CUDA toolkit | 570.211.01 / 12.8 |
| Python / CSDecomp | 3.10.22 / 0.1.2 |
| Drake / NumPy / SciPy | 1.51.1 / 2.2.6 / 1.15.3 |
| CVXPY / Clarabel | 1.7.5 / 0.11.1 |
| SCSPlanning / pybezier | 0.1.0 / 0.1.4 |

[Saved health checks](../benchmarks/generic_motion_v1/results/rigor_checks/health.json) record import locations, GPU queries, solver checks and negative fixtures. Installed CSDecomp is a compiled wheel: editing C++/CUDA source does not change the loaded binary without rebuilding it.

## Package records and native dependencies

| File | Scope |
|---|---|
| [environment.yml](environment.yml) | Conda packages and current Python dependency versions; no machine-specific prefix |
| [conda-explicit.txt](conda-explicit.txt) | Exact Linux Conda package builds and sources |
| [requirements.lock.txt](requirements.lock.txt) | Current Python dependency versions, excluding Conda-owned packaging tools; pinned Git sources |

These files describe the main Conda environment. The verified baseline stack additionally uses **OMPL 2.0.1**, **PySCIPOpt 6.3.0**, isolated **pybind11 2.13.6**, and compiled DRM, STOMP and TrajOpt sidecars. Their sources, build records and hashes are retained under [native/](../benchmarks/generic_motion_v1/native/), the adapters and per-run manifests. Isolated packages currently load from `.local_setup/solver_probes/`; native rebuilds also need the recorded compiler, Eigen/CUDA headers and upstream build dependencies. Those local installations are not supplied by cloning this repository.

`setup.sh` restores the main Python environment and runs upstream example checks; `setup_research.sh` is the retained Forest/FR3 helper. Neither is a verified one-command installation of the complete current Kinova baseline stack on a clean machine. Reusing the configured host is the tested path; this checkout needs no dependency installation.

Earlier setup logs remain in the local, Git-ignored `.local_setup/` directory. Published evidence is linked above, so public documentation does not depend on those local-only reports. Upstream example instructions that use uv do not change this project's Conda workflow.
