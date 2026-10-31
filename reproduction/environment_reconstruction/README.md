# Paper environment reconstruction

Runnable Forest and FR3 environments derived from the paper text, Figure 6 and
the authors' public assets. This package checks environment construction and
task execution; it does not replace the historical v5 benchmark or report a new
full-method comparison.

| Environment | Reconstruction | Remaining uncertainty |
|---|---|---|
| Forest Figure 6 | 15 circle centers and displayed start/goal recovered from vector artwork; published 10 m domain and 0.35 m radius | One illustrative scene, not the original ten random environments |
| Forest random scenes | Same ten v5 obstacle realizations, now with Figure 6 endpoints | Endpoints shared across scenes are a declared choice |
| FR3 | Seven joints, existing audited 33-sphere model, public table mesh/box, 20 spheres, right-configuration to left-pose tasks | Author sphere fit/filter, exact obstacle law, table transform and task list unavailable |

All assumptions are explicit in [profile.json](profile.json). The public table
has a **1.5 m visual mesh and 2.5 m collision box**. Both are shown in the
preview. Its use for FR3 is a documented hypothesis; its presence in an author
repository does not establish that it is the original benchmark directive.

## Run

From the repository root, using the existing Conda environment:

```bash
# Recover the illustrative Forest geometry from the cached author source.
bash local/python.sh -m reproduction.environment_reconstruction.forest_reference

# Use a new output path. Preparation freezes all inputs before planning.
bash local/python.sh -m reproduction.environment_reconstruction.run prepare \
  --run reproduction/runs/environment_reconstruction_demo

# Five native maps, four illustrative Forest sizes, ten Forest and ten FR3 tasks.
bash local/python.sh -m reproduction.environment_reconstruction.run smoke \
  --run reproduction/runs/environment_reconstruction_demo
bash local/python.sh -m reproduction.environment_reconstruction.run verify \
  --run reproduction/runs/environment_reconstruction_demo

# PNG and standalone HTML, rendered from saved configurations and paths.
bash local/python.sh -m reproduction.environment_reconstruction.render \
  reproduction/runs/environment_reconstruction_demo \
  reproduction/runs/environment_reconstruction_demo_preview
```

Repeating `smoke` resumes the same run. It verifies input, dependency, source and
completed-output hashes; completed failures stay failures. Interrupted partial
jobs are preserved and counted, never overwritten. Use a separate output path
for a new attempt. A direct `case --run PATH --case fr3_e00` command is available
for an unexecuted case after its map has a completion receipt; existing results
are protected.

Tests exercise actual CUDA/Drake/analytic geometry and evidence preservation:

```bash
bash local/python.sh -m pytest -q \
  reproduction/environment_reconstruction/test_fixed_forest.py \
  reproduction/environment_reconstruction/test_run_integrity.py
```

## Saved acceptance run

The [report](../reports/environment_reconstruction/report.md) records 21 prepared
scenes, 21,504 collision classifications, 24 native planning cases, 19 tests and
the two retained FR3 failures. The [interactive preview](../reports/environment_reconstruction/preview/fr3_interactive.html)
includes every FR3 scene and distinguishes collision geometry from display-only
official meshes. Open the HTML locally; it has no CDN dependency.

Raw maps, input samples, trajectories, logs and source snapshots for both the
initial run and the corrected evidence-preservation runner are archived in
`../reports/environment_reconstruction/evidence.tar.gz`. Restore at that report
directory with `tar -xzf evidence.tar.gz`. SHA-256 manifests accompany the
archive. Historical results and all 24 input tasks remain unchanged.
