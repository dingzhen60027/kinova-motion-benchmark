# Environment setup

[Project overview](../README.md) · [Benchmark commands](../benchmarks/generic_motion_v1/README.md#commands)

Choose the matching workflow:

- **This repository is already configured:** follow steps 1–5 below. No installation or upgrade is needed.
- **A fresh clone or machine:** follow [Python dependency restoration](#new-machine-python-dependencies), then resolve the listed native dependencies before running the full suite. Complete clean-machine installation has not yet been validated.

## Existing configured checkout

### 1. Open the repository

Open a terminal in the directory containing `README.md`, `scripts/` and `local/`. All subsequent commands run there.

```bash
pwd
test -f local/python.sh && test -f scripts/evaluate.py
```

The `test` command should exit successfully without output. From another working directory, invoke `local/python.sh` by its absolute path; the launcher changes to the repository root internally.

### 2. Check the NVIDIA driver

```bash
nvidia-smi
```

Expect your GPU and driver to appear. The recorded host is Ubuntu 22.04.5 x86-64 with a Quadro RTX 8000 (48 GiB), driver 570.211.01 and CUDA toolkit 12.8. A working driver is required for GPU collision checking; source builds additionally need CUDA headers.

### 3. Select the project Python

For the existing environment inside this checkout:

```bash
export CSDECOMP_CONDA_PREFIX="$PWD/.conda/envs/csdecomp"
test -x "$CSDECOMP_CONDA_PREFIX/bin/python"
```

If the environment is elsewhere, use `conda env list` to find its absolute prefix and substitute that path in the `export` command. Do not create or reinstall an environment that already works.

```bash
bash local/python.sh -c 'import sys, csdecomp, pydrake; print(sys.executable); print(csdecomp.__file__); print(pydrake.__file__)'
```

All three paths should be inside the chosen prefix. `conda activate` is unnecessary: the launcher calls that Python directly. The prefix selects Python only; a new clone still needs its own repository-local native dependencies described below.

### 4. Verify packages, data and core runtime

```bash
bash local/python.sh -m pip check
bash local/python.sh scripts/benchmark.py check
bash local/python.sh scripts/evaluate.py check
```

Read the results in order:

| Check | Expected result | What it verifies |
|---|---|---|
| `pip check` | `No broken requirements found` | Installed Python package dependency consistency |
| `benchmark.py check` | Passed integrity checks; 490 registered / 463 valid inputs | Frozen input hashes and references; no new collision audit |
| `evaluate.py check` | Top-level `passed: true` | Selected Python environment, GPU collision positive/negative fixtures, Clarabel solves and timeout handling |

The runtime check writes its diagnostic record to `results/checks.json`. It checks the core runtime, not every optional planner backend. Existing published checks are available in [health.json](../benchmarks/generic_motion_v1/results/rigor_checks/health.json).

### 5. Exercise the planning pipeline

```bash
bash local/python.sh scripts/evaluate.py run \
  --split smoke --problem smoke-bottleneck-000-p00 \
  --methods EI_ZO --track both --seeds 0 --output runs/environment-smoke
bash local/python.sh scripts/evaluate.py verify --run runs/environment-smoke
```

Choose a new output path. `verify` should return `integrity_passed: true` and `complete: true`; these mean intact, completed records, not that every planner succeeded. Inspect the recorded outcomes with:

```bash
bash local/python.sh scripts/evaluate.py status --run runs/environment-smoke
```

This step exercises DRM → EI-ZO → LSCS/SCSTO. Continue interrupted work with `resume --run runs/environment-smoke`; completed failures remain recorded.

## New machine: Python dependencies

These steps restore the **main Python environment** on Linux x86-64. Prerequisites are Git, an installed Conda command and a working NVIDIA driver. They do not install the repository-local native planner components in the next section.

**1. Obtain the repository.** Skip cloning if it already exists; enter that checkout instead.

```bash
git clone https://github.com/dingzhen60027/kinova-motion-benchmark.git
cd kinova-motion-benchmark
conda --version
nvidia-smi
```

**2. Create a new project-local environment from the Conda lock.** Run this only when the target directory does not exist.

```bash
export CSDECOMP_CONDA_PREFIX="$PWD/.conda/envs/csdecomp"
if [ -e "$CSDECOMP_CONDA_PREFIX" ]; then
  echo "Environment already exists; reuse it instead of recreating it."
else
  conda create --yes --prefix "$CSDECOMP_CONDA_PREFIX" --file local/conda-explicit.txt
fi
```

**3. Install the pinned Python dependencies into the newly created environment.** Skip this installation for an already configured checkout.

```bash
bash local/python.sh -m pip install -r local/requirements.lock.txt
bash local/python.sh -m pip check
```

The [Conda lock](conda-explicit.txt) fixes Linux package builds; the [pip lock](requirements.lock.txt) records the Python packages, pinned Git sources and CSDecomp wheel. [environment.yml](environment.yml) is the readable specification. The installed versions match the verified host, but these restoration steps have not been validated end to end on a clean host.

**4. Resolve the native dependencies below, then return to steps 3–5 of the existing-checkout workflow.** Setting a Conda prefix alone does not supply these files.

## Native planner dependencies

| Component | Recorded dependency / location | Implementation and build entry |
|---|---|---|
| OMPL planners | OMPL 2.0.1 in `.local_setup/solver_probes/ompl` | [OMPL adapter](../benchmarks/generic_motion_v1/runners/ompl_baselines.py) |
| LGCS / SCIP | PySCIPOpt 6.3.0 in `.local_setup/solver_probes/pyscipopt` | [SCIP adapter](../benchmarks/generic_motion_v1/runners/paper_lgcs_scip.py) |
| STOMP | pybind11 2.13.6 headers and compiled native bridge | [STOMP build and adapter](../benchmarks/generic_motion_v1/runners/chomp_stomp.py) |
| TrajOpt | Author SQP/BPMPD core and compiled bridge | [TrajOpt build and adapter](../benchmarks/generic_motion_v1/runners/trajopt_baseline.py) |
| DRM corrections | Compiled sidecar, matching CSDecomp ABI, CUDA/Eigen/upstream headers | [DRM build and adapter](../benchmarks/generic_motion_v1/runners/native_drm_query.py) |

Source/build evidence is retained under [native/](../benchmarks/generic_motion_v1/native/) and the run manifests. Some required installations and upstream build headers are local-only; cloning does not recreate them. Full native-stack installation on a clean machine remains an explicit setup gap.

`local/setup.sh` reinstalls Python dependencies and runs upstream examples; `local/setup_research.sh` is the earlier Forest/FR3 helper. Neither is required when reusing this checkout, and neither installs the complete current native stack.

## Optional MuJoCo simulation

Reuse the project Conda environment above. This adds physical execution checks to saved trajectories; it does not replace the planning benchmark.

**1. Install the optional pinned packages.** Skip installation if this checkout already has them.

```bash
bash local/python.sh -m pip install -r local/requirements-mujoco.txt
bash local/python.sh -m pip check
bash local/python.sh -c 'import sys, mujoco; print(sys.executable); print(mujoco.__version__)'
```

Expect `No broken requirements found`, the selected Conda Python and MuJoCo `3.3.7`. The [optional lock](requirements-mujoco.txt) records the eight packages installed for this layer; the existing project environment and planner dependencies remain in use.

**2. Execute one saved time trajectory.** Use a new output directory.

```bash
bash local/python.sh scripts/simulate.py run \
  --attempt results/nto_metis_smoke/jobs/smoke-bottleneck-000-p00-NTO_REFINED-minimum_time-633cc65e/attempts/0001 \
  --output runs/mujoco-one
bash local/python.sh scripts/simulate.py verify --run runs/mujoco-one
```

`result.json` contains execution checks; `rollout.npz` saves actual states, reference states, torque and contact evidence. `execution_success_numerical` means the declared numerical execution checks passed. A completed execution can also fail these checks; it is retained with its evidence.

**3. Execute a completed planning cohort, then resume or summarize it.**

```bash
bash local/python.sh scripts/simulate.py batch \
  --source-run results/baseline_smoke --methods EI_ZO \
  --output runs/mujoco-smoke
bash local/python.sh scripts/simulate.py resume --run runs/mujoco-smoke
bash local/python.sh scripts/simulate.py verify --run runs/mujoco-smoke
bash local/python.sh scripts/simulate.py summarize --run runs/mujoco-smoke
```

Only time-track jobs are selected. The first terminal planning attempt is preserved, including planning failures; completed simulation jobs are verified and never rerun. An unfinished cohort requires unchanged simulator source. Results are written to that cohort's `records.csv` and `summary.json`.

**4. Render or view the simulated motion.** Rendering requires an available `ffmpeg` executable and working OpenGL/EGL; headless rendering has been tested on the recorded NVIDIA host.

```bash
ffmpeg -version
bash local/python.sh scripts/simulate.py render \
  --run runs/mujoco-one --output runs/mujoco-one/render
```

Open the generated `overview.png` or `rollout.mp4`. For a local desktop window:

```bash
MUJOCO_GL=glfw bash local/python.sh scripts/simulate.py view --run runs/mujoco-one
```

The desktop viewer command has not been exercised on this host. It replays the saved simulated states; the physical execution happens in `run` or `batch`. Add `--collision-debug` to render/view to inspect collision geometry. [Model, controller and validation details](../benchmarks/generic_motion_v1/README.md#mujoco-execution).

## Troubleshooting

| Symptom | Check |
|---|---|
| `nvidia-smi` fails | Fix driver/GPU visibility before running GPU checks |
| `bin/python: No such file` | Confirm the absolute `CSDECOMP_CONDA_PREFIX` and that the environment exists |
| Imports come from another environment | Run through `local/python.sh` and inspect all three printed paths |
| Missing OMPL, SCIP or native bridge after cloning | Check the repository-local dependencies above; changing Conda activation will not create them |
| Run output already exists | Use `resume` for that run or select a new output directory |

The launcher isolates inherited Python/library paths and user site-packages, uses one OpenBLAS/MKL thread and defaults OMP to four. It leaves the parent shell and Conda base unchanged. Editing C++/CUDA source requires rebuilding the loaded binary.
