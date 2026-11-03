# Kinova Motion Planning Benchmark

A benchmark for **7-DoF robot motion planning** with fixed start/goal configurations and static obstacles, built on [CSDecomp](https://github.com/wernerpe/csdecomp). Compare **path length** and **trajectory duration** using shared inputs, validation and evaluation scripts.

[Environment](local/README.md) · [Benchmark guide](benchmarks/generic_motion_v1/README.md) · [Configuration](configs/evaluation.json) · [Results](benchmarks/generic_motion_v1/results/summary.csv)

## Setup and quick start

The tested setup is **Ubuntu 22.04, an NVIDIA GPU, and Conda Python 3.10**. Follow these steps in the existing configured repository. For a fresh machine, start with the [installation guide](local/README.md#new-machine-python-dependencies).

### 1. Check the GPU

From the repository root:

```bash
nvidia-smi
```

The command should list your NVIDIA GPU and driver without an error.

### 2. Select the Conda environment

```bash
export CSDECOMP_CONDA_PREFIX="$PWD/.conda/envs/csdecomp"
bash local/python.sh -c 'import sys, csdecomp, pydrake; print(sys.executable); print(csdecomp.__file__); print(pydrake.__file__)'
```

If your existing environment is elsewhere, replace the prefix with its absolute path. All three printed paths should be inside that environment. The launcher selects Python directly; `conda activate` is unnecessary.

### 3. Check dependencies and inputs

```bash
bash local/python.sh -m pip check
bash local/python.sh scripts/benchmark.py check
bash local/python.sh scripts/evaluate.py check
```

Expect `No broken requirements found` and passing dataset/core runtime checks. The runtime check executes GPU collision queries and Clarabel solves. Additional planner dependencies are listed in the [environment guide](local/README.md#native-planner-dependencies).

### 4. Run one problem

```bash
bash local/python.sh scripts/evaluate.py run \
  --split smoke --problem smoke-bottleneck-000-p00 \
  --methods EI_ZO --track both --seeds 0 \
  --output runs/example
bash local/python.sh scripts/evaluate.py verify --run runs/example
```

Use a new output directory. Verification should report `integrity_passed: true` and `complete: true`; each planner outcome is recorded separately. To continue an interrupted run, use `resume --run runs/example`. [Batch evaluation, summaries and replay →](benchmarks/generic_motion_v1/README.md#commands)

## Benchmark

Kinova Gen3 with a fixed gripper, table/equipment and five obstacle categories: **sparse clutter, dense clutter, bottleneck, route choice, open control**.

| Split | Use | Valid / registered problems |
|---|---|---:|
| `smoke` | Quick functional checks | 8 / 10 |
| `dev` | Development and configuration | 60 / 60 |
| `test` | Baseline evaluation | 395 / 420 |

Generation failures remain recorded. The valid test set covers 75 random layouts plus a shared base environment; it has prior evaluation exposure. This is a Kinova benchmark, distinct from the original paper's Franka experiments. [Data and protocol details →](benchmarks/generic_motion_v1/README.md)

## Baselines and results

Validated successes on **395 test problems, seed 0, 60 seconds/query**. Dev was evaluated with three seeds; test seeds 1 and 2 remain unrun. Validation is numerical, not a continuous collision certificate.

| Method | Path | Time |
|---|---:|---:|
| DRM + EI-ZO + LSCS / SCSTO | 338 | 338 |
| LGCS reconstruction | 327 | — |
| NTO published-profile reconstruction | 113 | 269 |
| NTO refined adaptation | 276 | 297 |
| NTO legacy adaptation (historical profile) | 123 | 265 |
| OMPL RRTConnect | 362 | — |
| OMPL RRT* | 316 | — |
| CHOMP adaptation | 336 | — |
| STOMP official core | 336 | — |
| TrajOpt adaptation | 313 | — |

All methods share the final validator. Path length and execution duration are evaluated separately from computation time; cost comparisons use paired common-success problems. [Method definitions](benchmarks/generic_motion_v1/README.md#methods) · [Raw records](benchmarks/generic_motion_v1/results/records.csv) · [Selected comparisons](benchmarks/generic_motion_v1/results/selection.json)

The three NTO rows are configurations of one method family. `NTO_REFINED` is the strengthened comparison profile; the other two preserve the published-parameter reconstruction and historical configuration. [Configuration differences and verified failure causes →](benchmarks/generic_motion_v1/README.md#nto-profiles-and-failure-analysis)

## Project structure

```text
scripts/      Dataset and evaluation CLI
configs/      Experiment configuration
benchmark/    Fixed problems, scenes, planner adapters and validator
results/      Recorded runs, summaries, trajectories and logs
local/        Conda launcher and dependency records
csdecomp/     Upstream C++/CUDA library
```

`benchmark/` and `results/` are aliases to the existing benchmark package and its outputs. [Full reference and extension points →](benchmarks/generic_motion_v1/README.md)

## Acknowledgments

Based on Werner et al., [*Superfast Configuration-Space Convex Set Computation on GPUs for Online Motion Planning*, RSS 2025](https://www.roboticsproceedings.org/rss21/p045.html). Method sources and implementation differences are documented in the [benchmark guide](benchmarks/generic_motion_v1/README.md#methods).

[MIT license](LICENSE). Upstream code, robot assets and third-party dependencies retain their respective licenses and attribution.
