# Reproducing CSDecomp experiments

> Historical Forest/FR3 reconstruction and reproduction evidence. References to
> “current” or “latest” below refer to that study. For the current Kinova
> benchmark, protocol and commands, use the [project README](../README.md).

A runnable constructed benchmark for the simulation experiments in
[Superfast Configuration-Space Convex Set Computation on GPUs for Online Motion Planning](https://arxiv.org/abs/2504.10783v1).
The work uses the existing Ubuntu 22.04 Conda environment and stays on `main`.
Upstream C++/CUDA sources and historical experiment results are preserved.

The latest [environment reconstruction](environment_reconstruction/README.md)
adds a vector-derived Forest scene, a public-asset FR3 tabletop profile and
interactive previews. Its [24-case acceptance report](reports/environment_reconstruction/report.md)
keeps two FR3 failures and distinguishes recovered parameters from assumptions.
It is an environment check, separate from the full v5 grid below.

The latest full comparison protocol is **`paper_constructed_v5`**. It constructs the missing
33-sphere FR3 asset from pinned official geometry, includes all 33 spheres in
dynamic obstacle checking, and fixes independent start/target sampling streams.
The geometry is selected using geometric criteria before planning outcomes.
Published settings and constructed choices are separated in the experimental
contract. Validation and method failures are retained alongside successes.

The [v4 report](reports/paper_completion/reproduction_report.md) preserves the
earlier 800-case experiment and its demonstrated FR3 sphere-fit defect. Original
author assets, exact random tasks and physical KUKA records remain unavailable.
The current constructed model has its own independent audit; neither generation
is presented as a recovery of the private original experiment.

## Measured coverage

Strictly validated candidates on the fixed v5 grids are shown below. The checks
use the declared collision model, endpoint and applicable derivative tolerances;
they are sampled checks, not continuous geometric certificates.

| Method | Forest (400 requested) | FR3 (400 requested) |
|---|---:|---:|
| LSCS | 388 | 393 |
| LGCS | 385 | 338 |
| SCSTO | 387 | 393 |
| NTO distance | 120 | 188 |
| NTO time | 243 | 344 |

FR3 includes seven DRM failures and 54 Gurobi license rejections. Those outcomes
remain in the denominator. The [result review](reports/paper_constructed/posthoc_result_review.md)
documents the remaining differences from the paper; completing a grid does not
establish numerical agreement with the published results.

The subsequent [alignment diagnosis](reports/alignment_diagnosis/report.md)
locates NTO collisions between constraint points, the time-spline velocity-bound
conservatism, and two geometric-solver failure mechanisms. It includes actual
same-input sensitivity experiments and a separate continuous Forest oracle.
These diagnostics preserve the v5 table above; stronger collision constraints
and unpublished spline settings are not relabeled as the authors' baseline.

The separate [iiwa/WSG simulation](reports/paper_constructed/hardware_simulation/report.md)
completed 15 of 15 fixed scene changes with validated virtual state transitions.
It supplied 35,000 nodes and retained 34,975 after graph construction. Its
synthetic clouds and kinematic state updates are not camera or physical-robot
measurements. The earlier 800-node smoke run failed both planning queries and is
retained alongside the successful full-size run.

## Start here

| Resource | Purpose |
|---|---|
| [Protocol and commands](paper_constructed/README.md) | One case, full grids, resume, validation, summaries and replay |
| [Diagnosis commands](alignment_diagnosis/README.md) | Frozen-input audits, controlled NTO variants and independent geometric checks |
| [Alignment diagnosis](reports/alignment_diagnosis/report.md) | Confirmed mechanisms, tested corrections and remaining paper differences |
| [Experimental contract](reports/paper_constructed/experimental_contract.md) | Published requirements, declared distributions, pair counts and selection rules |
| [Measured results and limitations](reports/paper_constructed/reproduction_report.md) | Actual v5 cases, failures, denominators and remaining differences |
| [Independent geometry audit](reports/paper_constructed/geometry_audit.md) | Fixed official reference, geometry samples and collision disagreement scope |
| [Model construction record](reports/paper_constructed/model_construction.md) | All attempts, final `model_construction_04` selection, sphere allocation and asset hashes |
| [Source evidence](reports/paper_constructed/source_evidence.json) | Primary URLs, paper locations, source hashes and task-distribution audit |
| [Historical parameter trace](reports/paper_completion/parameter_trace.csv) | Published values and v4 implementation evidence; v5 changes are in its contract |
| [Historical source audit](reports/paper_completion/source_audit.md) | Paper source, GitHub history, original-core build and hardware evidence |

## Environment

From the repository root:

```bash
bash reproduction/setup.sh
bash local/python.sh -m pytest -q reproduction/tests
```

The setup script verifies `.conda/envs/csdecomp`, or restores the locked
packages when absent. Reusing the current environment has been tested; a second
clean installation has not. The launcher sets BLAS threads to one and OpenMP
threads to four.

The measured environment uses Python 3.10.22, csdecomp 0.1.2, Drake 1.51.1,
NumPy 2.2.6, Clarabel 0.11.1 and gurobipy 13.0.3 on a Quadro RTX 8000.
[Current environment inventory](reports/paper_constructed/environment.json), [Conda lock](conda-explicit.txt) and
[Python lock](requirements.lock.txt) record the exact dependencies. The native
wheel postdates the paper. The isolated paper-date core was built successfully
and passed collision, DRM and EI-ZO checks on a constructed scene. Its
[execution record](reports/paper_completion/paperdate_core_result.json) is separate
from the historical 800 main trials. Both the v4 and constructed v5 runners use
0.1.2. The historical core has no GPU recovery interface. Neither the environment
nor the experiment binary was replaced; these probes do not establish complete
reproduction of the paper.

SNOPT and IPOPT are exercised in actual solves. Clarabel substitutes for
unlicensed MOSEK. Gurobi's restricted quadratic-model license can reject larger
LGCS problems; those attempts remain failures in the results.

## Run

```bash
# Full pipeline on one predeclared FR3 case.
bash local/python.sh -m reproduction.paper_constructed.run run --environment fr3 --environments 0 --roadmap-seeds 0 --sizes 3000

# Forest and FR3: 400 requested cases each, all five methods, run serially.
bash local/python.sh -m reproduction.run_constructed

# Resume the exact directory printed as RUN_DIR; terminal failures are preserved.
bash local/python.sh -m reproduction.paper_constructed.run resume --run-dir <RUN_DIR>
```

The full grids use ten environment seeds, ten roadmap seeds and four published
roadmap sizes. A completed case grid includes failed plans and optimizations.
It is not a claim that all requested trajectories succeeded.
Seeds 0–9 define a fixed public constructed task list; they are not claimed as
the authors' seeds or held-out data. Each FR3 scene retains the first qualifying
start and target from separate deterministic streams, with a strict 200000-draw
limit per side. No target is regenerated after a planner or solver fails.

## Validate and inspect

```bash
bash local/python.sh -m reproduction.paper_constructed.analyze --run-dir <FOREST_RUN> <FR3_RUN> --output reproduction/reports/paper_constructed --audit --collision-depths
bash local/python.sh -m reproduction.audit_paper_constructed --run-dir <FOREST_RUN> <FR3_RUN> --output reproduction/reports/paper_constructed/audit --resume
bash local/python.sh -m reproduction.report_constructed
bash local/python.sh -m reproduction.paper_constructed.plot --summary-dir reproduction/reports/paper_constructed --output reproduction/figures/paper_constructed
```

Figures read saved trajectories; planning is not rerun. Analyses separate solver
success, native CUDA collision checks and strict independent validation. CUDA
CFR is independent of solver success; their conjunction is reported separately.
Every requested case contributes to the main denominator. Paired costs use common
validated cases; paired timings use common recorded timing pairs. Dense trajectory
checks do not establish continuous collision freedom.

The 240-second watchdog covers an entire case, including all five methods.
LGCS and NTO configure 60-second solver limits; LSCS and SCSTO have no separate
60-second limit and rely on the case watchdog. Recovery can invoke further
solves. Fixed method order can therefore censor later methods. Summaries preserve interrupted attempts and methods not
reached by the timeout separately from solver failures.

## Repository layout

| Path | Contents |
|---|---|
| `paper_constructed/config.json` | Current frozen parameters and assumptions |
| `paper_constructed/assets/` | Constructed FR3 spheres, 2025 joint limits and provenance |
| `paper_constructed/model.py` | Real Drake/native plants and collision-constrained pose IK |
| `paper_constructed/forest.py`, `data.py` | Native offline maps and fixed scene generation |
| `paper_constructed/optimization.py` | LSCS, LGCS, author SCSTO and Drake NTO |
| `paper_constructed/pipeline.py` | CUDA EI-ZO, second routes, region edits and recovery |
| `paper_constructed/validation.py` | Geometry, endpoint, derivative and trajectory checks |
| `paper_constructed/run.py` | Isolated workers, budgets, checkpoints and source identity |
| `tests/` | Numerical and native model/solver regression tests |
| `reports/paper_constructed/` | Current results, failed attempts, audits and command ledgers |
| `paper_completion/`, `reports/paper_completion/` | Frozen v4 implementation and historical evidence |
| `runs/` | Actual inputs, maps, coefficients, replay samples and exceptions |
| `figures/paper_constructed/` | Source-linked PDF, SVG, PNG and TIFF figures |
| `vendor/` | Pinned third-party source and isolated historical build materials |

Raw runs and downloaded sources stay on disk outside ordinary Git commits.
Checksum inventories and compact evidence are committed. Portable archives in
`artifacts/` preserve reproducible sources and reports; their manifests state
which large raw files are excluded.

The [v4 report](reports/paper_completion/reproduction_report.md),
[v3 protocol and report](reports/paper_protocol/reproduction_report.md),
[v1 Forest report](reports/reproduction_report.md), and v2 diagnostics remain
unchanged. Their measurements are never pooled with the current protocol.
