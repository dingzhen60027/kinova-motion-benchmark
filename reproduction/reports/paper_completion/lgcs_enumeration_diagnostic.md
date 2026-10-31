# License-independent saved-input diagnostic

All 32 license-blocked inputs from the completed primary FR3 run were actually
processed. Every worker exited with code 0; no case timed out or reached the
10,000-path cap. Across 2,898 corridor SOCPs, 31 cases established the numerical
minimum over all simple paths. Case `e04_r07_n12000` remains unresolved because
corridors 48 and 144 returned `kSolverSpecificError`; its saved best candidate is
not described as globally optimal.

Independent Drake validation checked 37,800 points: 24 best candidates passed and
8 had collisions, totaling 425 colliding samples. Of the 24 passing candidates,
23 also have an established numerical saved-region optimum. The remaining passing
candidate belongs to the unresolved case. No new region inflation or recovery was
performed after these saved inputs, and no CUDA checker was called by this
diagnostic. Passing means the declared dense Drake, endpoint, bound and region
checks passed; it is not a continuous collision certificate.

The sum of per-case wall times was 41.662 seconds; graph enumeration and corridor
SOCP time totaled 23.775 seconds. Neither is Gurobi timing. All original input and
primary result hashes were rechecked unchanged, so the main benchmark still
contains its original 32 `blocked_license` results.

The stable entry is `lgcs_enumeration_results.json`, with detailed case outcomes
in `lgcs_enumeration_cases.csv` and `lgcs_enumeration_summary.json`. Raw data are in
`reproduction/runs/20261025T100744.754623Z_LGCS_enumeration_diagnostic`; the complete
file hash list is `lgcs_enumeration_raw_inventory.json`. Earlier pure Python graph
checks remain in `enumeration_driver_static_checks.json`.

The diagnostic is mathematically appropriate for the saved distance objective.
Every source-to-target simple path through the region-intersection graph defines
a convex corridor problem solved by the existing `lscs` SOCP. Conversely, a path
that revisits a convex region can replace the subpath from its first entry to its
last exit by the straight segment inside that region. The triangle inequality
means this replacement cannot increase Euclidean length. Nonnegative, disjoint
flow cycles likewise cannot improve the source-to-target objective. Thus an
optimal route can be selected from the finite simple paths; taking the minimum
of their solved SOCP costs recovers the saved-region optimum, subject to numerical
solver statuses and tolerances. This does not recover the original author model
or certify collision-freeness of imperfect inflated regions.

The actual blocked graphs had 5–9 regions and 14–316 complete simple paths per
case. This does not establish general scalability: dense graphs can have
factorially many simple paths. The driver first
inventories only actual `blocked_license` results and their saved pre-solve inputs.
Inventory mode reads JSON/NPZ only and does not recompute graph intersections.
Execute mode is permitted only for a completed primary run, reconstructs exactly
the frozen LGCS adjacency tests, and imposes a 10,000-corridor cap and a 120-second
whole-case subprocess deadline. Either limit leaves an explicitly unresolved
diagnostic; no global-optimum claim is made for an incomplete enumeration or an
unresolved corridor solve.

```sh
# Inventory only; no solver or GPU calls.
bash local/python.sh -m reproduction.enumerate_blocked_lgcs --run-dir RUN_DIRECTORY

# Actual executed diagnostic (a rerun creates a separate output directory):
bash local/python.sh -m reproduction.enumerate_blocked_lgcs \
  --run-dir reproduction/runs/20261025T092858.440656Z_paper_completion_fr3 \
  --execute --case-timeout 120 --path-limit 10000
```

Every attempt receives a new output directory. It records original input hashes,
the region graph, every enumerated sequence and solver status, candidate arrays,
the best candidate, process logs, deadlines and exceptions. The best candidate is
checked against the independent Drake scene (FR3), or exact segment–circle
geometry (Forest), with dense inputs retained. This driver makes no CUDA calls
and performs no extra inflation or recovery. An optimal saved-region solution
that fails independent collision validation remains a failed geometric result.

The output is a separate Clarabel enumeration diagnostic. Its runtime is not
Gurobi runtime, its successes do not replace `blocked_license` in Table I, and a
completed case is only a numerical optimum over the saved input regions. The
original licensed-solver failure and all source snapshots remain intact.
