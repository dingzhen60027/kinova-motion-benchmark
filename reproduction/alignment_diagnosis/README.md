# Diagnosing differences from the paper

These tools audit frozen `paper_constructed_v5` runs and execute controlled
replays. They do not change the original 800-case results or identify any new
parameter choice as the authors' undisclosed configuration.

Read the [diagnosis report](../reports/alignment_diagnosis/report.md) for measured
findings, the complete sensitivity matrix, and the remaining limitations.
Raw arrays, logs, failed attempts and source snapshots are retained in
`../reports/alignment_diagnosis/evidence.tar.gz`, with per-file hashes and a
read-back verification receipt. Extract into a scratch directory to inspect the
bundle without overwriting the current checkout. Original v5 run inputs are in
the prior constructed-experiment archive and are not duplicated in this bundle.

## Inputs and environment

Use the existing `.conda/envs/csdecomp` through `bash local/python.sh`. Restore the
v5 raw evidence before running these tools. The collocation and sensitivity
tools resolve paths from `reproduction/reports/paper_constructed/run_index.json`;
the Forest oracle accepts an explicit `--source-run` instead. The current recorded
paths refer to this checkout. In another checkout, update that index to the
restored run directories before starting a new diagnosis. Never edit the old
run manifests to make a resume succeed.

## Saved-trajectory audits

```bash
# Analytic Forest geometry versus the actual Drake collocation constraints.
bash local/python.sh -m reproduction.alignment_diagnosis.collocation \
  --output reproduction/diagnostics/forest_collocation_new

# See the dedicated CLI for the full FR3 saved-candidate audit.
bash local/python.sh -m reproduction.alignment_diagnosis.fr3_collocation --help
```

The Forest audit covers all 800 saved NTO candidates. Its raw categories based
on the dimensionless smooth residual are diagnostic thresholds, not claims
about solver correctness or physical penetration depth. The report combines
them with the independent continuous line–circle analysis. The FR3 audit uses
all 786 saved candidates and preserves the 14 combinations missing because DRM
failed. A nonnegative query with a zero search threshold supplies a sign only;
it does not measure a positive clearance.

## Same-input sensitivity matrix

```bash
bash local/python.sh -m reproduction.alignment_diagnosis.ablation \
  --environment forest --output reproduction/diagnostics/forest_sensitivity_new
bash local/python.sh -m reproduction.alignment_diagnosis.ablation \
  --environment fr3 --output reproduction/diagnostics/fr3_sensitivity_new

# Resume uses the exact saved matrix and skips completed attempts, including failures.
bash local/python.sh -m reproduction.alignment_diagnosis.ablation \
  --output reproduction/diagnostics/fr3_sensitivity_new --resume

bash local/python.sh -m reproduction.alignment_diagnosis.summarize \
  --run reproduction/diagnostics/forest_sensitivity_new \
  --run reproduction/diagnostics/fr3_sensitivity_new \
  --output reproduction/diagnostics/sensitivity_comparison_new
```

The matrix selects environment seeds 0–9, roadmap seed 0, and the smallest
published roadmap size, by input ID. Each environment has 110 attempts: six
distance variants and five time variants on ten identical frozen inputs.
Every attempt runs in its own process with a 100-second wall limit; the original
solver limit is 60 seconds. All failures, candidates, dense validation samples,
source snapshots, and input identities are saved. These are post-hoc diagnostics,
not a held-out benchmark or a runtime comparison. Summary generation checks
inputs after execution; before resuming, restore the same original input files.

Distance variants isolate clearance (0, 1 mm, 10 mm) and Nc (published count or
4×). Time variants isolate clearance and shape-preserving knot insertion to 16
or 32 controls. Knot insertion preserves the initial curve and its derivatives,
but changes the finite-dimensional trajectory family and the conservatism of
velocity bounds. Final feasibility tolerances are unchanged.

## Continuous Forest diagnostic

`forest_oracle.py` solves the same PWL length problem with the same original
number of nodes, but replaces sampled collisions with analytic segment–circle
constraints. It requires 1 mm clearance, with a declared 1 µm numerical margin
tolerance. It is an independent mechanism check, **not the paper's point-sampled
NTO baseline**. The 400-case result and its one margin failure are kept separate
from the original 120/400 result.

```bash
bash local/python.sh -m reproduction.alignment_diagnosis.forest_oracle \
  --all-cases --source-run <RESTORED_FOREST_RUN> \
  --output reproduction/diagnostics/forest_oracle_new
```

The source, task-distribution, derivative, geometric-replay and solver-control
evidence is linked from the diagnosis report. Published condition differences,
unavailable licenses and unavailable physical hardware remain explicit.

## Opt-in geometric corrections

`geometric_solvers.lgcs_strict` retains the original GCS model and sets explicit
integer and continuous feasibility tolerances. `scsto_preserve_knots` retains
every initialization knot, checks initial region containment, and reuses the
author's alternating solver. Both return candidates compatible with the v5
validator; neither mutates the installed modules or overrides old results.
They were verified on all five saved geometric failures plus four normal
controls. These checks do not solve Gurobi's license-size limit.

```bash
bash local/python.sh -m reproduction.alignment_diagnosis.geometric_replay --help
```
