# Paper-grounded reconstruction protocol

`paper_constructed_v5` implements the simulation workflow of
[arXiv:2504.10783v1](https://arxiv.org/abs/2504.10783v1), using the published PDF,
original arXiv TeX, released code, and pinned official robot geometry.
[The report](../reports/paper_constructed/reproduction_report.md) records measured
outcomes and unresolved differences. Historical v1–v4 implementations and runs
are retained unchanged.

This version constructs the unreleased FR3 sphere model and random tasks from
public sources under an explicit experimental contract. Sphere allocation is
chosen from geometric coverage and volume, before any planning outcomes.
The official geometry is audited independently, with both excess volume and
potential undercoverage reported. These are constructed benchmark results;
the original private inputs and physical KUKA experiment are not claimed.

[Experimental contract](../reports/paper_constructed/experimental_contract.md)
separates published settings, implementation choices and remaining differences.
The [v4 implementation and report](../reports/paper_completion/reproduction_report.md)
are preserved unchanged, including their model defect and unsuccessful attempts.

## Run and inspect

Use the existing Conda prefix `.conda/envs/csdecomp` from the repository root:

```bash
# One case, including map construction, online IK, all methods and validation.
bash local/python.sh -m reproduction.paper_constructed.run run --environment fr3 --environments 0 --roadmap-seeds 0 --sizes 3000

# Both complete 400-case grids, serially, with command/exit-code ledgers.
bash local/python.sh -m reproduction.run_constructed

# Keep the exact RUN_DIR printed by the runner.
bash local/python.sh -m reproduction.paper_constructed.run resume --run-dir <RUN_DIR>

# Run these outside timed experiments.
bash local/python.sh -m pytest -q reproduction/tests
bash local/python.sh -m reproduction.paper_constructed.analyze --run-dir <FOREST_RUN> <FR3_RUN> --output reproduction/reports/paper_constructed --audit --collision-depths
bash local/python.sh -m reproduction.audit_paper_constructed --run-dir <FOREST_RUN> <FR3_RUN> --output reproduction/reports/paper_constructed/audit --resume
bash local/python.sh -m reproduction.report_constructed
bash local/python.sh -m reproduction.paper_constructed.plot --summary-dir reproduction/reports/paper_constructed --output reproduction/figures/paper_constructed
```

The collision-throughput driver accepts one completed environment-family run.
For each of its ten environment seeds, it executes five timed batches of two
million configurations and a separate warmup: 50 timed batches per invocation.

```bash
bash local/python.sh -m reproduction.paper_constructed.throughput --run-dir <RUN_DIR>
```

Resume checks numerical dependency versions, source identities and frozen
inputs. Both terminal failures and successes are preserved. A source change
requires a new run. Analysis and plotting do not change computational identity.

## Parameter provenance

| Setting | Forest | FR3 | Source |
|---|---|---|---|
| Environment seeds × roadmap seeds × sizes | 10 × 10 × 4 | 10 × 10 × 4 | PDF p. 9, VII-A5 |
| Roadmap sizes | 200, 400, 800, 1600 | 3000, 6000, 9000, 12000 | PDF p. 9, VII-A5 |
| Neighbors; task/configuration distance caps | 10; 10 / 10 | 10; 10 / 10 | PDF p. 14, Table V |
| Offline voxel side; lazy edge step | 0.06 m; 0.1 | 0.06 m; 0.1 | PDF p. 14, Appendix D/Table V |
| EI-ZO δ / ε | 0.05 / 0.01 | 0.005 / 0.005 | PDF p. 14, Tables III/IV |
| EI-ZO particles / mixing steps / faces | 1000 / 30 / 10 | 10000 / 60 / 10 | PDF p. 14, Tables III/IV |
| EI-ZO margin / τ / iteration cap | 0.01 / 0.5 / 30 | 0.01 / 0.5 / 30 | PDF and TeX comment |
| Distance NTO solver; Nc | SNOPT; 20 | SNOPT; 150 | Paper appendix |
| Time NTO solver; Nc | SNOPT; 100 | IPOPT; 150 | Paper appendix |

Formal published values take precedence over conflicting draft comments.
[config.json](config.json) freezes every implementation choice.
The 30-iteration cap can stop EI-ZO before its statistical test accepts; as
Section IV-C explains, this capped profile does not certify the δ/ε guarantee
for every returned region. Saved trajectories are validated separately.

The historical v4 [parameter trace](../reports/paper_completion/parameter_trace.csv)
separates published values, draft evidence and the earlier implementation.
The v5 contract records its changed model, task streams and NTO grid; the v4
trace is not a claim that those changes were used in earlier measurements.

NTO retains the actual DRM nodes. Distance uses order-two B-splines; time uses
order four with duplicated endpoint controls. The formal PDF specifies `Nc` collision-check points. This version uses exactly
`Nc` uniformly spaced interior locations `k/(Nc+1)`, for `k=1,...,Nc`. The
point count follows the PDF; the exact locations are a declared construction.
The draft TeX uses `Nc-1` internal points, retained only in historical v4. Endpoint duplication, influence
distance, solver options and budget limits are explicit reconstruction choices.

LSCS uses Clarabel in place of unlicensed MOSEK. LGCS uses the cited perspective
flow formulation, projected algebraically onto a split graph, with direct Gurobi
MISOCP optimization. `NonConvex=0` remains enforced. Restricted-license failures
are recorded. SCSTO calls the pinned author biconvex implementation. See the
[optimization audit](../reports/paper_completion/optimization_audit.md), including
actual numerical checks and unsuccessful formulation attempts.

PDF Eq. (19d) equates adjacent path nodes, not velocities; taken literally with
distinct start and goal nodes, it is infeasible. The implementation follows the
text's continuous piecewise-linear SOCP interpretation, with this apparent
typesetting ambiguity retained for author confirmation.

## Model and online pipeline

Both environments build native node, adjacency, pose and voxel collision maps
offline. Obstacles enter online. The API receives half-side `0.03` for the
published `0.06 m` voxel side. LGCS also uses native DRM for its second route.
If that route is unavailable, the first route is retained.

FR3 start configurations and target poses are fixed per environment. After
forming the online collision set, each roadmap's pose map supplies collision-free
IK initializations. Actual IK, every candidate, canonical float32 endpoints and
pose residuals are saved. Targets are never regenerated in response to a failed
planner or solver.
Obstacle, start and target streams are fixed independently. Targets are the FK
pushforward of free joint-uniform samples in the left TCP box, not Cartesian-
or orientation-uniform samples. Seeds 0–9 are a fixed constructed list already
used in prior diagnostics; they are not held-out or original author seeds.
v5 changes geometry, fixed-base obstacle checks, the NTO grid and endpoint
streams together. Shared seed labels across v4/v5 do not imply identical
endpoints or accepted roadmap nodes. Cross-version results are not a controlled
model ablation; paired method comparisons use common cases within v5.

The included FR3 asset uses official geometry with verified byte equivalence
to prepublication meshes. Joint limits are pinned to official revision
`39f8c0bcea9bc48ae5334edcb2aa2a504f2d2c85` (2025-04-11), before six ranges were
widened in 2026. Sphere centers/radii and filters remain a reconstruction.
All 33 robot spheres, including the anchored base, participate in online
obstacle checks: 33 × 20 = 660 dynamic pairs. The kinematic model still has
seven actuated joints. Static collision pairs follow the official SRDF and
Drake weld rules; measured counts are reported without adding artificial pairs
to match the paper's unexplained 527 static/self pairs. See
[model provenance](assets/fr3_33.json) and the
[independent official-geometry audit](../reports/paper_constructed/geometry_audit.md).

Region recovery preserves the original path, applies outward float32 rounding
with a numerical error bound, and checks native containment before proceeding.
It does not relax collision thresholds. A native 0.1-step DRM path that fails
the stricter independent seed check is saved as `invalid_seed_path` for LSCS,
LGCS and SCSTO; those three methods skip inflation and optimization, while NTO
still receives the path. This is a local input-validation gate, not a recovered
author acceptance rule. These cases remain in every native-DRM-success
denominator, so the conditional rates are not exact reproductions of the
paper's evaluation procedure.

## Evidence and interpretation

Each run saves a manifest, source snapshot, exact inputs, offline maps, online
IK attempts, path seeds, all pre-solve regions, solver metadata, trajectory
coefficients, replay samples and exceptions. Analysis reports separately:

- Native DRM success and independently valid seed paths.
- Solver success, native CUDA collision checks and strict independent validation.
- All requested cases, actual method attempts and DRM-conditioned rates.
- Costs on common validated cases and times on common recorded timing pairs.

CUDA CFR counts saved CUDA-free trajectories given native DRM success,
independently of solver status. A failed nonlinear solve can still end on a
CUDA-free trajectory. The solver-success conjunction and strict independently
validated success are separate metrics; missing trajectories are not counted
free. Interrupted method attempts without completed timings remain attempts,
but do not enter timing means.

Five methods run in fixed order under a shared 240-second case watchdog.
LGCS and NTO configure 60-second solver limits; LSCS and SCSTO have no separate
60-second limit and rely on the case watchdog. Recovery can invoke further
solves. Later methods can therefore remain unstarted when
earlier work exhausts the case budget. Reports and case-grid figures distinguish
these censored methods from solver failures, and timing ECDFs include completed
timing records only. The scene replay selects the first frozen case by ID;
the FR3 illustration shows LSCS, SCSTO and NTO-time TCP curves when saved.

Independent geometry uses the same declared model in Drake; Forest also has
analytic line-circle checks. Dense curves have bounded sampling steps and
independent derivative/control-point checks. These are numerical checks, not
continuous collision certificates or evidence that a fitted model is original.

TO+CS timing excludes offline preparation, IK, warmup, final-candidate validation
and the surrounding pre-solve/replay writes. Construction timing does include
alternate-roadmap serialization/loading, alternate-path validation and recovery
coverage-record writes. Exception wall-time fallbacks are flagged separately;
there is no claim that all file IO was subtracted. CPU/GPU, CUDA, solvers and the installed native
wheel differ from the paper. The isolated paper-date core was built successfully
and passed collision, DRM and EI-ZO checks on a constructed scene; see its
[execution record](../reports/paper_completion/paperdate_core_result.json).
It has no GPU recovery interface and has not replaced the 0.1.2 binary used for
the measured simulation grids. These checks do not establish complete reproduction of the
paper, and the published 17.1× acceleration has not been reproduced.
